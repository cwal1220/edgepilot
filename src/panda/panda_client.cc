#include "panda/panda_client.h"

#include "panda/panda_can_codec.h"
#include "panda/panda_protocol.h"

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <unistd.h>

namespace {

constexpr uint8_t kUsbRequestOut = LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE;
constexpr uint8_t kUsbRequestIn = LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE;
constexpr uint8_t kCanRxEndpoint = 0x81;
constexpr uint8_t kCanTxEndpoint = 3;
constexpr int kRecvSize = 0x4000;
constexpr int kUsbTxSoftLimit = 0x100;
constexpr int kUsbBulkWriteRetries = 3;
constexpr uint32_t kMaxConsecutiveMalformedRx = 3;

} // namespace

PandaClient::~PandaClient()
{
    close();
}

bool PandaClient::connect(const std::string &serial)
{
    close();
    int err = libusb_init(&ctx_);
    if (err != 0) {
        std::fprintf(stderr, "panda: libusb_init failed err=%d\n", err);
        return false;
    }

    libusb_device **dev_list = nullptr;
    const ssize_t count = libusb_get_device_list(ctx_, &dev_list);
    if (count < 0) {
        close();
        return false;
    }

    for (ssize_t i = 0; i < count; ++i) {
        libusb_device_descriptor desc {};
        if (libusb_get_device_descriptor(dev_list[i], &desc) != 0) continue;
        if (desc.idVendor != kPandaVendorId || desc.idProduct != kPandaAppProductId) continue;

        libusb_device_handle *handle = nullptr;
        if (libusb_open(dev_list[i], &handle) != 0 || !handle) continue;

        unsigned char desc_serial[64] = {};
        const int serial_len = libusb_get_string_descriptor_ascii(
            handle, desc.iSerialNumber, desc_serial, sizeof(desc_serial));
        const std::string found_serial = serial_len > 0
            ? std::string(reinterpret_cast<char *>(desc_serial), static_cast<size_t>(serial_len))
            : std::string();
        if (!serial.empty() && serial != found_serial) {
            libusb_close(handle);
            continue;
        }

        dev_handle_ = handle;
        usb_serial_ = found_serial;
        break;
    }
    libusb_free_device_list(dev_list, 1);

    if (!dev_handle_) {
        close();
        return false;
    }

    if (libusb_kernel_driver_active(dev_handle_, 0) == 1)
        libusb_detach_kernel_driver(dev_handle_, 0);

    err = libusb_set_configuration(dev_handle_, 1);
    if (err != 0 && err != LIBUSB_ERROR_BUSY) {
        mark_usb_error(err, "libusb_set_configuration");
        close();
        return false;
    }
    err = libusb_claim_interface(dev_handle_, 0);
    if (err != 0) {
        mark_usb_error(err, "libusb_claim_interface");
        close();
        return false;
    }

    uint8_t packet_versions[2] = {};
    if (!control_read(kPandaRequestPacketVersions, 0, 0, packet_versions, sizeof(packet_versions)) ||
        packet_versions[0] != kPandaHealthPacketVersion ||
        packet_versions[1] != kPandaCanPacketVersion) {
        std::fprintf(stderr,
                     "panda: unsupported packet versions health=%u can=%u (expected %u/%u)\n",
                     packet_versions[0], packet_versions[1],
                     kPandaHealthPacketVersion, kPandaCanPacketVersion);
        close();
        return false;
    }
    health_packet_version_ = packet_versions[0];
    can_packet_version_ = packet_versions[1];

    uint8_t hw_query = 0;
    if (control_read(kPandaRequestHwType, 0, 0, &hw_query, sizeof(hw_query)))
        hw_type_ = hw_query;
    // 버전 문자열은 길이가 정해져 있지 않아(널 없이 올 수도 있다) 받은 만큼만 쓴다
    char version[kPandaVersionMaxBytes] = {};
    const int version_len = libusb_control_transfer(
        dev_handle_, kUsbRequestIn, kPandaRequestVersion, 0, 0,
        reinterpret_cast<unsigned char *>(version), sizeof(version), 1000);
    if (version_len > 0)
        firmware_version_.assign(version, strnlen(version, static_cast<size_t>(version_len)));
    comms_healthy_ = true;
    consecutive_malformed_rx_ = 0;
    return true;
}

void PandaClient::close()
{
    if (dev_handle_) {
        libusb_release_interface(dev_handle_, 0);
        libusb_close(dev_handle_);
        dev_handle_ = nullptr;
    }
    if (ctx_) {
        libusb_exit(ctx_);
        ctx_ = nullptr;
    }
    usb_serial_.clear();
    firmware_version_.clear();
    hw_type_ = 0;
    health_packet_version_ = 0;
    can_packet_version_ = 0;
    comms_healthy_ = true;
}

bool PandaClient::control_write(uint8_t request, uint16_t value, uint16_t index, unsigned timeout_ms)
{
    if (!dev_handle_) return false;
    const int err = libusb_control_transfer(dev_handle_, kUsbRequestOut, request, value, index,
                                            nullptr, 0, timeout_ms);
    if (err < 0) {
        mark_usb_error(err, "control_write");
        return false;
    }
    return true;
}

bool PandaClient::control_read(uint8_t request, uint16_t value, uint16_t index,
                               void *data, uint16_t length, unsigned timeout_ms)
{
    if (!dev_handle_) return false;
    const int err = libusb_control_transfer(dev_handle_, kUsbRequestIn, request, value, index,
                                            static_cast<unsigned char *>(data), length, timeout_ms);
    if (err < 0) {
        mark_usb_error(err, "control_read");
        return false;
    }
    return err == length;
}

int PandaClient::bulk_read(uint8_t endpoint, uint8_t *data, int length, unsigned timeout_ms)
{
    if (!dev_handle_) return 0;
    int transferred = 0;
    const int err = libusb_bulk_transfer(dev_handle_, endpoint, data, length, &transferred, timeout_ms);
    if (err == LIBUSB_ERROR_TIMEOUT) {
        // libusb may return a timeout after transferring part of the requested
        // buffer. Those bytes are a valid prefix of the Panda CAN stream.
        return transferred;
    }
    if (err == LIBUSB_ERROR_OVERFLOW) {
        comms_healthy_ = false;
        std::fprintf(stderr, "panda: CAN receive overflow transferred=%d\n", transferred);
        return transferred;
    }
    if (err != 0) {
        mark_usb_error(err, "bulk_read");
        return 0;
    }
    return transferred;
}

int PandaClient::bulk_write(uint8_t endpoint, const uint8_t *data, int length, unsigned timeout_ms)
{
    if (!dev_handle_) return 0;
    int total_transferred = 0;
    for (int attempt = 0;
         attempt <= kUsbBulkWriteRetries && total_transferred < length;
         ++attempt) {
        int transferred = 0;
        const int err = libusb_bulk_transfer(
            dev_handle_, endpoint,
            const_cast<uint8_t *>(data + total_transferred),
            length - total_transferred, &transferred, timeout_ms);
        total_transferred += transferred;
        if (err == 0) {
            if (transferred == 0 && total_transferred < length) {
                std::fprintf(stderr, "panda: zero-length successful bulk write\n");
                comms_healthy_ = false;
                return -1;
            }
            continue;
        }
        if (err == LIBUSB_ERROR_TIMEOUT) {
            ++usb_tx_timeouts_;
            return total_transferred;
        }
        const bool retryable =
            err == LIBUSB_ERROR_IO || err == LIBUSB_ERROR_OVERFLOW ||
            err == LIBUSB_ERROR_INTERRUPTED || err == LIBUSB_ERROR_BUSY;
        if (retryable && attempt < kUsbBulkWriteRetries &&
            total_transferred < length) {
            ++usb_tx_retries_;
            usleep(1000);
            continue;
        }
        mark_usb_error(err, "bulk_write");
        return -1;
    }
    return total_transferred;
}

void PandaClient::mark_usb_error(int err, const char *where)
{
    std::fprintf(stderr, "panda: usb error %d (%s) in %s\n",
                 err, libusb_strerror(static_cast<libusb_error>(err)), where);
    if (err == LIBUSB_ERROR_NO_DEVICE) close();
    comms_healthy_ = false;
}

bool PandaClient::set_safety_model(uint16_t safety_model, uint16_t safety_param)
{
    return control_write(kPandaRequestSafetyModel, safety_model, safety_param);
}

bool PandaClient::send_heartbeat(bool engaged)
{
    return control_write(kPandaRequestHeartbeat, engaged ? 1 : 0, 0, 100);
}

bool PandaClient::get_health(PandaHealth *health)
{
    if (!health) return false;
    PandaHealthPacket packet {};
    if (!control_read(kPandaRequestHealth, 0, 0, &packet, sizeof(packet), 100)) return false;
    health->uptime = packet.uptime_pkt;
    health->voltage = packet.voltage_pkt;
    health->current = packet.current_pkt;
    health->can_rx_errs = packet.can_rx_errs_pkt;
    health->can_send_errs = packet.can_send_errs_pkt;
    health->can_fwd_errs = packet.can_fwd_errs_pkt;
    health->gmlan_send_errs = packet.gmlan_send_errs_pkt;
    health->faults = packet.faults_pkt;
    health->ignition_line = packet.ignition_line_pkt;
    health->ignition_can = packet.ignition_can_pkt;
    health->controls_allowed = packet.controls_allowed_pkt;
    health->gas_interceptor_detected = packet.gas_interceptor_detected_pkt;
    health->car_harness_status = packet.car_harness_status_pkt;
    health->usb_power_mode = packet.usb_power_mode_pkt;
    health->safety_mode = packet.safety_mode_pkt;
    health->safety_param = packet.safety_param_pkt;
    health->fault_status = packet.fault_status_pkt;
    health->power_save_enabled = packet.power_save_enabled_pkt;
    health->heartbeat_lost = packet.heartbeat_lost_pkt;
    health->alternative_experience = packet.alternative_experience_pkt;
    health->blocked_msg_cnt = packet.blocked_msg_cnt_pkt;
    health->interrupt_load = packet.interrupt_load;
    return true;
}

bool PandaClient::receive(std::vector<PandaCanFrame> *frames, int timeout_ms)
{
    if (!frames) return false;
    frames->clear();
    uint8_t data[kRecvSize] = {};
    const int recv = bulk_read(kCanRxEndpoint, data, sizeof(data), static_cast<unsigned>(timeout_ms));
    if (recv <= 0) return comms_healthy_;
    if (recv == kRecvSize) std::fprintf(stderr, "panda: receive buffer full\n");
    return unpack_can_buffer(data, recv, frames);
}

bool PandaClient::send(const std::vector<PandaCanFrame> &frames)
{
    if (frames.empty()) return true;
    std::vector<uint8_t> packed;
    if (!pack_can_buffer(frames, &packed)) return false;
    for (size_t pos = 0; pos < packed.size();) {
        const size_t chunk = std::min<size_t>(kUsbTxSoftLimit, packed.size() - pos);
        const int sent = bulk_write(kCanTxEndpoint, packed.data() + pos, static_cast<int>(chunk), 5);
        if (sent <= 0) return false;
        pos += static_cast<size_t>(sent);
    }
    return true;
}

bool PandaClient::unpack_can_buffer(const uint8_t *data, int size, std::vector<PandaCanFrame> *frames)
{
    std::string error;
    if (!panda_can_unpack_buffer(data, size, &recv_buf_, frames, &error)) {
        ++malformed_rx_batches_;
        ++consecutive_malformed_rx_;
        std::fprintf(stderr, "panda: dropping malformed CAN USB batch: %s\n",
                     error.c_str());
        recv_buf_.clear();
        frames->clear();
        if (consecutive_malformed_rx_ >= kMaxConsecutiveMalformedRx) {
            comms_healthy_ = false;
            return false;
        }
        return true;
    }
    consecutive_malformed_rx_ = 0;
    return true;
}

bool PandaClient::pack_can_buffer(const std::vector<PandaCanFrame> &frames, std::vector<uint8_t> *out)
{
    std::string error;
    if (!panda_can_pack_buffer(frames, out, &error)) {
        std::fprintf(stderr, "panda: %s\n", error.c_str());
        return false;
    }
    return true;
}
