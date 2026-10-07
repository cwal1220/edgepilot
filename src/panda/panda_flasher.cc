#include "panda/panda_flasher.h"

#include "panda/panda_protocol.h"

#include <libusb.h>
#include <unistd.h>

#include <cstdio>
#include <cstring>

namespace {

constexpr uint8_t kRequestOut = LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE;
constexpr uint8_t kRequestIn = LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE;
constexpr int kReenumerateTimeoutMs = 15000;  // 재부팅 뒤 다시 열거될 때까지(파이썬 panda도 15초)
constexpr int kPollMs = 200;
constexpr unsigned kControlTimeoutMs = 1000;
constexpr unsigned kEraseTimeoutMs = 5000;    // bootstub은 섹터를 다 지운 뒤에 응답한다
constexpr unsigned kWriteTimeoutMs = 2000;
constexpr size_t kWriteChunk = 16;            // 파이썬 panda의 flash_static과 같은 단위

/* libusb 컨텍스트 하나와 열린 판다 하나. 열면 인터페이스 0을 쥔다. */
class PandaUsb {
public:
    PandaUsb() { ok_ = libusb_init(&ctx_) == 0; }
    ~PandaUsb()
    {
        close();
        if (ok_) libusb_exit(ctx_);
    }
    bool ok() const { return ok_; }

    // 붙어 있는 판다의 product id(없으면 0)
    uint16_t present() const
    {
        libusb_device **list = nullptr;
        const ssize_t count = libusb_get_device_list(ctx_, &list);
        uint16_t found = 0;
        for (ssize_t i = 0; i < count && !found; ++i) {
            libusb_device_descriptor desc {};
            if (libusb_get_device_descriptor(list[i], &desc) != 0 || desc.idVendor != kPandaVendorId) continue;
            if (desc.idProduct == kPandaAppProductId || desc.idProduct == kPandaBootstubProductId)
                found = desc.idProduct;
        }
        if (count >= 0) libusb_free_device_list(list, 1);
        return found;
    }

    bool wait_for(uint16_t product_id) const
    {
        for (int waited = 0; waited < kReenumerateTimeoutMs; waited += kPollMs) {
            if (present() == product_id) return true;
            usleep(kPollMs * 1000);
        }
        return false;
    }

    // 0이면 열려 있다. 아니면 libusb 오류 코드(LIBUSB_ERROR_BUSY: 다른 프로세스가 쥐고 있다)
    int open(uint16_t product_id)
    {
        close();
        handle_ = libusb_open_device_with_vid_pid(ctx_, kPandaVendorId, product_id);
        if (!handle_) return LIBUSB_ERROR_NO_DEVICE;
        if (libusb_kernel_driver_active(handle_, 0) == 1) libusb_detach_kernel_driver(handle_, 0);
        int err = libusb_set_configuration(handle_, 1);
        if (err == 0 || err == LIBUSB_ERROR_BUSY) err = libusb_claim_interface(handle_, 0);
        if (err != 0) {
            libusb_close(handle_);
            handle_ = nullptr;
        }
        return err;
    }

    void close()
    {
        if (!handle_) return;
        libusb_release_interface(handle_, 0);
        libusb_close(handle_);
        handle_ = nullptr;
    }

    int write(uint8_t request, uint16_t value, unsigned timeout_ms = kControlTimeoutMs)
    {
        return libusb_control_transfer(handle_, kRequestOut, request, value, 0, nullptr, 0, timeout_ms);
    }

    int read(uint8_t request, uint8_t *data, uint16_t length)
    {
        return libusb_control_transfer(handle_, kRequestIn, request, 0, 0, data, length, kControlTimeoutMs);
    }

    int bulk_write(const uint8_t *data, int length)
    {
        int transferred = 0;
        const int err = libusb_bulk_transfer(handle_, kPandaFlashEndpoint, const_cast<uint8_t *>(data), length,
                                             &transferred, kWriteTimeoutMs);
        return err == 0 ? transferred : err;
    }

private:
    bool ok_ = false;
    libusb_context *ctx_ = nullptr;
    libusb_device_handle *handle_ = nullptr;
};

std::string usb_error(const char *what, int err)
{
    return std::string(what) + ": " + libusb_error_name(err);
}

/* bootstub의 flasher 응답(12바이트): [4:8] 매직, [8:12] 다음에 쓸 주소. 아니면 false. */
bool read_flasher_echo(PandaUsb &usb, uint32_t *prog_ptr)
{
    uint8_t echo[12] = {};
    if (usb.read(kPandaRequestFlasherEcho, echo, sizeof(echo)) != static_cast<int>(sizeof(echo))) return false;
    if (std::memcmp(echo + 4, kPandaFlasherMagic, sizeof(kPandaFlasherMagic)) != 0) return false;
    if (prog_ptr) std::memcpy(prog_ptr, echo + 8, sizeof(*prog_ptr));
    return true;
}

std::string read_version(PandaUsb &usb)
{
    char version[kPandaVersionMaxBytes] = {};
    const int got = usb.read(kPandaRequestVersion, reinterpret_cast<uint8_t *>(version), sizeof(version));
    return got > 0 ? std::string(version, strnlen(version, static_cast<size_t>(got))) : std::string();
}

} // namespace

PandaUsbMode panda_usb_mode()
{
    PandaUsb usb;
    if (!usb.ok()) return PandaUsbMode::None;
    switch (usb.present()) {
    case kPandaAppProductId: return PandaUsbMode::App;
    case kPandaBootstubProductId: return PandaUsbMode::Bootstub;
    default: return PandaUsbMode::None;
    }
}

PandaFlashResult flash_panda(const PandaImage &image, const PandaFlashProgress &progress)
{
    PandaFlashResult result;
    auto fail = [&](const char *error, const std::string &detail) {
        result.error = error;
        result.detail = detail;
        return result;
    };
    auto report = [&](const char *step, int percent) {
        if (progress) progress(step, percent);
    };
    PandaUsb usb;
    if (!usb.ok()) return fail("usb_error", "libusb_init failed");
    const uint16_t found = usb.present();
    if (!found) return fail("no_panda", "no panda on USB");

    report("bootstub", 0);
    if (found == kPandaAppProductId) {
        // 앱에서는 하드웨어를 확인하고 bootstub으로 보낸다. 판다가 곧바로 재부팅해서 응답은 오류다.
        const int err = usb.open(kPandaAppProductId);
        if (err == LIBUSB_ERROR_BUSY) return fail("busy", "another process holds the panda (pandad running?)");
        if (err != 0) return fail("usb_error", usb_error("open application", err));
        uint8_t hw_type = 0;
        if (usb.read(kPandaRequestHwType, &hw_type, 1) != 1) return fail("usb_error", "hardware type read failed");
        if (!panda_hw_type_flashable(hw_type))
            return fail("hw_unsupported", std::string(panda_hw_type_name(hw_type)) + " (type " +
                                              std::to_string(hw_type) + ") is not an STM32F413 board");
        usb.write(kPandaRequestEnterBootloader, 1);
        usb.close();
        if (!usb.wait_for(kPandaBootstubProductId)) return fail("bootstub_timeout", "panda did not come back as bootstub");
    }

    const int err = usb.open(kPandaBootstubProductId);
    if (err == LIBUSB_ERROR_BUSY) return fail("busy", "another process holds the panda (pandad running?)");
    if (err != 0) return fail("usb_error", usb_error("open bootstub", err));
    if (!read_flasher_echo(usb, nullptr)) return fail("flasher_missing", "bootstub flasher did not answer");

    report("erase", 0);
    if (usb.write(kPandaRequestFlashUnlock, 0) < 0) return fail("erase_failed", "flash unlock failed");
    for (uint16_t sector = kPandaAppFirstSector; sector <= kPandaAppLastSector; ++sector) {
        const int erased = usb.write(kPandaRequestFlashErase, sector, kEraseTimeoutMs);
        if (erased < 0) return fail("erase_failed", usb_error(("erase sector " + std::to_string(sector)).c_str(), erased));
        report("erase", 100 * (sector - kPandaAppFirstSector + 1) / (kPandaAppLastSector - kPandaAppFirstSector + 1));
    }

    const size_t size = image.bytes.size();
    int last_percent = -1;
    for (size_t offset = 0; offset < size; offset += kWriteChunk) {
        const int length = static_cast<int>(std::min(kWriteChunk, size - offset));
        const int written = usb.bulk_write(image.bytes.data() + offset, length);
        if (written != length)
            return fail("write_failed", written < 0 ? usb_error("write", written)
                                                    : "short write at offset " + std::to_string(offset));
        const int percent = static_cast<int>(100 * (offset + static_cast<size_t>(length)) / size);
        if (percent / 5 != last_percent / 5) {
            report("write", percent);
            last_percent = percent;
        }
    }

    // bootstub이 앱 시작부터 이미지 끝까지 받았는지: 다음에 쓸 주소가 정확히 끝이어야 한다
    report("verify", 100);
    uint32_t prog_ptr = 0;
    if (!read_flasher_echo(usb, &prog_ptr)) return fail("verify_failed", "flasher stopped answering after the write");
    const uint32_t expected = kPandaAppStart + static_cast<uint32_t>(size);
    if (prog_ptr != expected) {
        char detail[96];
        std::snprintf(detail, sizeof(detail), "bootstub wrote up to 0x%08x, expected 0x%08x", prog_ptr, expected);
        return fail("verify_failed", detail);
    }

    report("reboot", 100);
    usb.write(kPandaRequestReset, 0);
    usb.close();
    // 서명이 맞지 않으면 bootstub은 앱으로 넘어가지 않고 bootstub으로 다시 열거된다
    if (!usb.wait_for(kPandaAppProductId))
        return fail("app_timeout", "panda did not start the new firmware (still in bootstub? bad signature?)");

    report("check", 100);
    for (int attempt = 0; attempt < 5 && result.version.empty(); ++attempt) {
        if (usb.open(kPandaAppProductId) == 0) result.version = read_version(usb);
        usb.close();
        if (result.version.empty()) usleep(300 * 1000);
    }
    if (result.version != image.version)
        return fail("version_mismatch", "panda reports '" + result.version + "', image is '" + image.version + "'");
    result.ok = true;
    return result;
}
