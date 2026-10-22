#include "maixcam2/maix_display.h"

#include "ax_middleware.hpp"
#include "hud/hud_tiles.h"
#include "maix_app.hpp"
#include "maixcam2/maix_frames.h"

#include <fcntl.h>
#include <linux/fb.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

using namespace maix::middleware::maixcam2;

namespace {

// MaixCDK DisplayAx::__config_vo_param(rotate=1)과 같다. 패널은 480x640 DSI.
void config_vo(ax_vo_param_t *param, int width, int height) {
    std::memset(param, 0, sizeof(*param));
    AX_VO_SYNC_INFO_T sync = {};
    sync.u16Vact = 640;
    sync.u16Vbb = 30;
    sync.u16Vfb = 30;
    sync.u16Hact = 480;
    sync.u16Hbb = 30;
    sync.u16Hfb = 30;
    sync.u16Hpw = 40;
    sync.u16Vpw = 11;
    sync.u32Pclk = 24750;
    sync.bIdv = AX_TRUE;
    sync.bIhs = AX_FALSE;
    sync.bIvs = AX_TRUE;
    SAMPLE_VO_CONFIG_S cfg = {};
    cfg.u32VDevNr = 1;
    cfg.stVoDev[0].u32VoDev = 0;
    cfg.stVoDev[0].enMode = AX_VO_MODE_OFFLINE;
    cfg.stVoDev[0].enVoIntfType = AX_VO_INTF_DSI;
    cfg.stVoDev[0].enIntfSync = AX_VO_OUTPUT_USER;
    cfg.stVoDev[0].enVoOutfmt = AX_VO_OUT_FMT_UNUSED;
    cfg.stVoDev[0].u32SyncIndex = 2;
    for (auto &layer : cfg.stVoLayer) {
        layer.bindVoDev[0] = layer.bindVoDev[1] = SAMPLE_VO_DEV_MAX;
        layer.enChnFrmFmt = AX_FORMAT_YUV420_SEMIPLANAR;
    }
    cfg.stGraphicLayer[0].u32FbNum = 1;
    cfg.stGraphicLayer[0].stFbConf[0].u32Index = 0;
    cfg.stGraphicLayer[0].stFbConf[0].u32Fmt = AX_FORMAT_ARGB8888;
    cfg.stGraphicLayer[0].stFbConf[0].u32ResoW = height;
    cfg.stGraphicLayer[0].stFbConf[0].u32ResoH = width;
    cfg.stVoLayer[0].stVoLayerAttr.stImageSize.u32Width = height;
    cfg.stVoLayer[0].stVoLayerAttr.stImageSize.u32Height = width;
    cfg.stVoLayer[0].stVoLayerAttr.enPixFmt = AX_FORMAT_YUV420_SEMIPLANAR;
    cfg.stVoLayer[0].stVoLayerAttr.u32DispatchMode = 1;
    cfg.stVoLayer[0].stVoLayerAttr.f32FrmRate = 60.0f;
    cfg.stVoLayer[0].u32ChnNr = 1;
    std::memcpy(&param->vo_cfg, &cfg, sizeof(cfg));
    std::memcpy(&param->sync_info, &sync, sizeof(sync));
}

// 16:9 소스의 가운데를 화면 비율(kWidth:kHeight = 4:3)로 자른다. IVPS 정렬(폭 16,
// 위치 2)을 맞춘다. app_config.h kPreviewAspect와 같은 영역이다.
void center_crop(AX_VIDEO_FRAME_T *frame, int width, int height) {
    int crop_w = height * MaixDisplay::kWidth / MaixDisplay::kHeight;
    if (crop_w > width) crop_w = width;
    crop_w &= ~15;
    frame->s16CropX = static_cast<AX_S16>(((width - crop_w) / 2) & ~1);
    frame->s16CropY = 0;
    frame->s16CropWidth = static_cast<AX_S16>(crop_w);
    frame->s16CropHeight = static_cast<AX_S16>(height);
}

// /boot/board의 key=value (disp_flip 등). 없으면 기본값.
int board_config(const char *key, int fallback) {
    std::ifstream file("/boot/board");
    std::string line;
    const std::string want = std::string(key) + "=";
    while (std::getline(file, line))
        if (line.compare(0, want.size(), want) == 0) return std::atoi(line.c_str() + want.size());
    return fallback;
}

// 백라이트는 PWM3(pwmchip0/pwm3, 주기 100 µs). duty = 주기 x 밝기(%) x disp_max_backlight(%).
bool write_backlight(float percent) {
    const std::string pwm = "/sys/class/pwm/pwmchip0/pwm3";
    if (!std::ifstream(pwm + "/period")) std::ofstream("/sys/class/pwm/pwmchip0/export") << 3;
    const long period = 100000;
    const int max_percent = board_config("disp_max_backlight", 95);
    const long duty = static_cast<long>(period * std::clamp(percent, 0.0f, 100.0f) / 100.0f * max_percent / 100.0f);
    std::ofstream period_file(pwm + "/period");
    period_file << period;
    std::ofstream duty_file(pwm + "/duty_cycle");
    duty_file << duty;
    std::ofstream enable_file(pwm + "/enable");
    enable_file << 1;
    return period_file.good() && duty_file.good() && enable_file.good();
}

// MaixCDK 화면이 닫히며 백라이트를 0으로 두므로 다시 켠다. 밝기는 /boot/configs maix_backlight_value(%)다.
// overlayd는 곧바로 웹 기기 설정(display.json)의 밝기로 바꾼다.
void enable_backlight() {
    write_backlight(static_cast<float>(std::atoi(maix::app::get_sys_config_kv("backlight", "value", "50").c_str())));
}

}  // namespace

// HUD를 쓰는 그래픽 레이어 fb0. MaixCDK VO의 레이어 1 채널은 fb0를 화면에 묶어 두려고 그대로
// 만들되 프레임은 보내지 않는다. 그림판은 캐시 가능한 보통 메모리이고 fb0(쓰기 결합, 순차 쓰기
// 약 1 GB/s)에는 바뀐 칸만 옮긴다.
struct Framebuffer {
    int fd = -1;
    uint8_t *map = nullptr;
    size_t length = 0;
    uint8_t *page = nullptr;  // 지금 화면에 나오는 쪽
    int line = 0;             // fb0 행 바이트
};

// VO 생성자도 AX SYS 초기화 뒤에 만든다.
struct MaixDisplay::Impl {
    SYS sys{false};
    std::unique_ptr<VO> video_ptr;
    std::unique_ptr<VO> osd_ptr;
    int video_ch = -1;
    int osd_ch = -1;
    AX_POOL dst_pool = AX_INVALID_POOLID;
    Framebuffer fb;
    std::vector<uint32_t> canvas;      // 세로 HUD 그림판(kHeight x kWidth)
    std::vector<uint32_t> last_dirty;  // 지난번 옮긴 칸(행마다 비트)
    bool copy_all = true;              // 다음엔 전부 옮긴다(처음, 또는 지난번에 칸 정보 없이 옮겼다)
};

namespace {

bool open_framebuffer(Framebuffer *fb, int width, int height) {
    fb->fd = open("/dev/fb0", O_RDWR | O_CLOEXEC);
    if (fb->fd < 0) return false;
    fb_fix_screeninfo fix{};
    fb_var_screeninfo var{};
    if (ioctl(fb->fd, FBIOGET_FSCREENINFO, &fix) != 0 || ioctl(fb->fd, FBIOGET_VSCREENINFO, &var) != 0 ||
        var.bits_per_pixel != 32 || static_cast<int>(var.xres) != width || static_cast<int>(var.yres) != height) {
        std::fprintf(stderr, "display: fb0 is not %dx%d 32 bpp\n", width, height);
        return false;
    }
    void *map = mmap(nullptr, fix.smem_len, PROT_READ | PROT_WRITE, MAP_SHARED, fb->fd, 0);
    if (map == MAP_FAILED) return false;
    fb->map = static_cast<uint8_t *>(map);
    fb->length = fix.smem_len;
    fb->line = static_cast<int>(fix.line_length);
    fb->page = fb->map + static_cast<size_t>(var.yoffset) * fb->line;
    return true;
}

}  // namespace

MaixDisplay::MaixDisplay() : impl_(new Impl) {
    if (impl_->sys.init() != maix::err::ERR_NONE) throw std::runtime_error("AX SYS init failed");
    impl_->video_ptr.reset(new VO());
    impl_->osd_ptr.reset(new VO());
    VO &video_vo = *impl_->video_ptr;
    VO &osd_vo = *impl_->osd_ptr;
    const int flip = board_config("disp_flip", 1);
    const int mirror = board_config("disp_mirror", 0);
    ax_vo_param_t param;
    config_vo(&param, kWidth, kHeight);
    if (video_vo.init(&param) != maix::err::ERR_NONE) throw std::runtime_error("VO init failed");
    ax_vo_channel_param_t video = {
        kWidth, kHeight, AX_FORMAT_YUV420_SEMIPLANAR, AX_FORMAT_YUV420_SEMIPLANAR_VU, 60, 0, mirror, flip, 0, 90,
        -1,     -1};
    impl_->video_ch = video_vo.get_unused_channel(0);
    if (impl_->video_ch < 0 || video_vo.add_channel(0, impl_->video_ch, &video) != maix::err::ERR_NONE)
        throw std::runtime_error("VO video channel failed");

    config_vo(&param, kWidth, kHeight);
    if (osd_vo.init(&param) != maix::err::ERR_NONE) throw std::runtime_error("VO OSD init failed");
    ax_vo_channel_param_t osd = {
        kWidth, kHeight, AX_FORMAT_BGRA8888, AX_FORMAT_YUV420_SEMIPLANAR_VU, 60, 0, mirror, flip, 0, 90, -1, -1};
    impl_->osd_ch = osd_vo.get_unused_channel(1);
    if (impl_->osd_ch < 0 || osd_vo.add_channel(1, impl_->osd_ch, &osd) != maix::err::ERR_NONE)
        throw std::runtime_error("VO OSD channel failed");
    // 그림판은 MaixCDK가 레이어 1을 TDP(90°, disp_flip이면 FLIP)로 fb0에 넣던 방향과 같다: disp_flip=1,
    // disp_mirror=0인 이 보드는 그대로 transpose다(fb0 덤프로 확인).
    if (!open_framebuffer(&impl_->fb, kHeight, kWidth)) throw std::runtime_error("fb0 open failed");
    impl_->canvas.assign(static_cast<size_t>(kWidth) * kHeight, 0);
    impl_->last_dirty.assign(kWidth, 0);

    enable_backlight();
}

bool MaixDisplay::set_backlight(bool on, float percent) { return write_backlight(on ? percent : 0.0f); }

// 생성자를 끝낸 뒤에만 불린다(던지면 프로세스가 끝난다). 풀만 첫 프레임 때 만든다.
MaixDisplay::~MaixDisplay() {
    Framebuffer &fb = impl_->fb;
    // 멈춘 HUD가 화면에 남지 않게 비운다
    for (int y = 0; y < kWidth; ++y) std::memset(fb.page + static_cast<size_t>(y) * fb.line, 0, kHeight * 4);
    munmap(fb.map, fb.length);
    close(fb.fd);
    // VO가 마지막으로 받은 블록을 채널을 닫을 때 돌려주므로 풀은 그 뒤에 없앤다.
    impl_->osd_ptr->del_channel(1, impl_->osd_ch);
    impl_->osd_ptr->deinit();
    impl_->video_ptr->del_channel(0, impl_->video_ch);
    impl_->video_ptr->deinit();
    impl_->osd_ptr.reset();
    impl_->video_ptr.reset();
    if (impl_->dst_pool != AX_INVALID_POOLID) AX_POOL_DestroyPool(impl_->dst_pool);
    impl_->sys.deinit();
}

bool MaixDisplay::show_video_phys(unsigned long long phys, int width, int height,
                                  const std::function<bool()> &source_still_valid) {
    // 풀은 첫 프레임 때 만든다. camerad가 VI를 열며 AX 공용 풀을 다시 설정하므로
    // 그 전에 만든 풀은 무효가 될 수 있다.
    if (impl_->dst_pool == AX_INVALID_POOLID) {
        impl_->dst_pool = make_noncache_pool(kWidth * kHeight * 3 / 2, 3);
        if (impl_->dst_pool == AX_INVALID_POOLID) return false;
    }
    Frame out(impl_->dst_pool, kWidth, kHeight, nullptr, 0, AX_FORMAT_YUV420_SEMIPLANAR);
    AX_VIDEO_FRAME_T src = nv12_frame(phys, width, height), dst = {};
    center_crop(&src, width, height);
    out.get_video_frame(&dst);
    AX_IVPS_CROP_RESIZE_ATTR_T attr = stretch_resize();
    attr.tAspectRatio.eAligns[0] = AX_IVPS_ASPECT_RATIO_HORIZONTAL_CENTER;
    attr.tAspectRatio.eAligns[1] = AX_IVPS_ASPECT_RATIO_VERTICAL_CENTER;
    attr.tAspectRatio.nBgColor = 0x000000;
    return AX_IVPS_CropResizeTdp(&src, &dst, &attr) == 0 && source_still_valid() &&
           impl_->video_ptr->push(0, impl_->video_ch, &out) == maix::err::ERR_NONE;
}

MaixDisplay::OverlayBuffer MaixDisplay::begin_overlay() {
    return {reinterpret_cast<uint8_t *>(impl_->canvas.data()), kHeight * 4};
}

// 그림판 행마다 이번(dirty)과 지난번에 그린 칸을 합쳐, 이어진 칸끼리 한 번에 fb0로 옮긴다. 지난번
// 칸은 이번에 지워져 0이 된 자리라 함께 옮겨야 한다.
void MaixDisplay::end_overlay(const uint32_t *dirty, int tile_shift) {
    Framebuffer &fb = impl_->fb;
    constexpr int kRowBytes = kHeight * 4;
    for (int y = 0; y < kWidth; ++y) {
        const uint8_t *src =
            reinterpret_cast<const uint8_t *>(impl_->canvas.data()) + static_cast<size_t>(y) * kRowBytes;
        uint8_t *dst = fb.page + static_cast<size_t>(y) * fb.line;
        uint32_t &last = impl_->last_dirty[y];
        if (!dirty || impl_->copy_all) {
            std::memcpy(dst, src, kRowBytes);
            last = dirty ? dirty[y] : 0;
            continue;
        }
        for_each_tile_run(dirty[y] | last, [&](int first, int end) {
            const int x0 = (first << tile_shift) * 4, x1 = std::min(kRowBytes, (end << tile_shift) * 4);
            std::memcpy(dst + x0, src + x0, static_cast<size_t>(x1 - x0));
        });
        last = dirty[y];
    }
    impl_->copy_all = !dirty;
}
