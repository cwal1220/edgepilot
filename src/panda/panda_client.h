#ifndef PANDA_CLIENT_H
#define PANDA_CLIENT_H

#include "panda/panda_can_codec.h"
#include "panda/panda_protocol.h"

#include <libusb.h>

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

struct PandaHealth {
    uint32_t uptime = 0;
    uint32_t voltage = 0;
    uint32_t current = 0;
    uint32_t can_rx_errs = 0;
    uint32_t can_send_errs = 0;
    uint32_t can_fwd_errs = 0;
    uint32_t gmlan_send_errs = 0;
    uint32_t faults = 0;
    uint8_t ignition_line = 0;
    uint8_t ignition_can = 0;
    uint8_t controls_allowed = 0;
    uint8_t gas_interceptor_detected = 0;
    uint8_t car_harness_status = 0;
    uint8_t usb_power_mode = 0;
    uint8_t safety_mode = 0;
    uint16_t safety_param = 0;
    uint8_t fault_status = 0;
    uint8_t power_save_enabled = 0;
    uint8_t heartbeat_lost = 0;
    uint16_t alternative_experience = 0;
    uint32_t blocked_msg_cnt = 0;
    float interrupt_load = 0.0f;
};

class PandaClient {
public:
    PandaClient() = default;
    ~PandaClient();

    bool connect(const std::string &serial = "");
    void close();
    bool connected() const { return dev_handle_ != nullptr; }
    bool comms_healthy() const { return comms_healthy_; }
    const std::string &usb_serial() const { return usb_serial_; }
    // 접속할 때 읽은 펌웨어 버전 문자열(0xd6). 못 읽었으면 빈 문자열이다.
    const std::string &firmware_version() const { return firmware_version_; }
    uint8_t hw_type() const { return hw_type_; }
    uint8_t health_packet_version() const { return health_packet_version_; }
    uint8_t can_packet_version() const { return can_packet_version_; }
    uint32_t usb_tx_timeouts() const { return usb_tx_timeouts_; }
    uint32_t usb_tx_retries() const { return usb_tx_retries_; }
    uint32_t malformed_rx_batches() const { return malformed_rx_batches_; }

    bool set_safety_model(uint16_t safety_model, uint16_t safety_param);
    bool send_heartbeat(bool engaged);
    bool get_health(PandaHealth *health);
    bool receive(std::vector<PandaCanFrame> *frames, int timeout_ms = 10);
    bool send(const std::vector<PandaCanFrame> &frames);

private:
    bool control_write(uint8_t request, uint16_t value, uint16_t index, unsigned timeout_ms = 1000);
    bool control_read(uint8_t request, uint16_t value, uint16_t index,
                      void *data, uint16_t length, unsigned timeout_ms = 1000);
    int bulk_read(uint8_t endpoint, uint8_t *data, int length, unsigned timeout_ms);
    int bulk_write(uint8_t endpoint, const uint8_t *data, int length, unsigned timeout_ms);
    void mark_usb_error(int err, const char *where);
    bool unpack_can_buffer(const uint8_t *data, int size, std::vector<PandaCanFrame> *frames);
    bool pack_can_buffer(const std::vector<PandaCanFrame> &frames, std::vector<uint8_t> *out);

    libusb_context *ctx_ = nullptr;
    libusb_device_handle *dev_handle_ = nullptr;
    std::string usb_serial_;
    std::string firmware_version_;
    uint8_t hw_type_ = 0;
    uint8_t health_packet_version_ = 0;
    uint8_t can_packet_version_ = 0;
    bool comms_healthy_ = true;
    uint32_t usb_tx_timeouts_ = 0;
    uint32_t usb_tx_retries_ = 0;
    uint32_t malformed_rx_batches_ = 0;
    uint32_t consecutive_malformed_rx_ = 0;
    std::vector<uint8_t> recv_buf_;
};

#endif
