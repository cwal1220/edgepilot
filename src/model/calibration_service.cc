#include "model/calibration_service.h"

#include "common/utils_file.h"
#include "common/utils_json.h"
#include "common/utils_process.h"
#include "common/utils_math.h"

#include <algorithm>
#include <array>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <iomanip>
#include <sstream>
#include <sys/stat.h>
#include <unistd.h>

namespace {

constexpr int kPersistVersion = 1;
constexpr auto kPersistInterval = std::chrono::seconds(60);
constexpr float kPersistDeltaRad = 0.01f * 3.14159265358979323846f / 180.0f;
constexpr auto kResetPollInterval = std::chrono::seconds(1);

struct StoredCalibration {
    float rpy[3] = {};
    float spread[3] = {};
    int valid_blocks = 0;
    float height_m = OnlineCalibrator::kHeightInit;  // 없으면(예전 파일) HEIGHT_INIT
};

bool load_stored_calibration(const std::string &path, StoredCalibration *stored,
                             std::string *error)
{
    if (!stored) return false;
    const std::string text = read_text_file(path);
    if (text.empty()) return false;
    try {
        float version = 0.0f;
        float valid_blocks = 0.0f;
        std::array<float, 3> rpy{};
        std::array<float, 3> spread{};
        if (!parse_json_float_value(text, "version", &version) ||
            static_cast<int>(std::lround(version)) != kPersistVersion ||
            !parse_json_float_array(text, "rpy_rad", &rpy) ||
            !parse_json_float_value(text, "valid_blocks", &valid_blocks)) {
            if (error) *error = "missing or unsupported fields";
            return false;
        }
        parse_json_float_array(text, "spread_rad", &spread);
        float height = OnlineCalibrator::kHeightInit;
        if (parse_json_float_value(text, "height_m", &height) && std::isfinite(height))
            stored->height_m = height;
        for (int i = 0; i < 3; ++i) {
            stored->rpy[i] = rpy[static_cast<size_t>(i)];
            stored->spread[i] = spread[static_cast<size_t>(i)];
        }
        stored->valid_blocks = std::max(0, static_cast<int>(std::lround(valid_blocks)));
        return true;
    } catch (const std::exception &exc) {
        if (error) *error = exc.what();
        return false;
    }
}

bool ensure_params_dir(const std::string &path)
{
    if (mkdir(path.c_str(), 0755) == 0 || errno == EEXIST) return true;
    std::fprintf(stderr, "calibration: mkdir %s failed: %s\n",
                 path.c_str(), std::strerror(errno));
    return false;
}

bool save_stored_calibration(const std::string &params_dir, const std::string &path,
                             const float rpy[3], const OnlineCalibrator::Snapshot &snapshot)
{
    if (!ensure_params_dir(params_dir)) return false;
    std::ostringstream json;
    json << std::setprecision(9)
         << "{\n"
         << "  \"version\": " << kPersistVersion << ",\n"
         << "  \"rpy_rad\": [" << rpy[0] << ", " << rpy[1] << ", " << rpy[2] << "],\n"
         << "  \"spread_rad\": [" << snapshot.spread[0] << ", "
         << snapshot.spread[1] << ", " << snapshot.spread[2] << "],\n"
         << "  \"valid_blocks\": " << snapshot.valid_blocks << ",\n"
         << "  \"height_m\": " << snapshot.height_m << "\n"
         << "}\n";
    if (!write_file_atomic(path, json.str())) {
        std::fprintf(stderr, "calibration: write %s failed: %s\n",
                     path.c_str(), std::strerror(errno));
        return false;
    }
    return true;
}

float max_rpy_delta(const float a[3], const float b[3])
{
    return std::max(std::fabs(a[0] - b[0]),
                    std::max(std::fabs(a[1] - b[1]), std::fabs(a[2] - b[2])));
}

void copy_rpy(float dst[3], const float src[3])
{
    for (int i = 0; i < 3; ++i) dst[i] = src[i];
}

} // namespace

CalibrationService::CalibrationService(const AppConfig &config)
    : auto_enabled_(config.calibration_auto),
      manual_override_(config.manual_calibration),
      log_enabled_(config.log_calibration),
      params_dir_(params_dir()),
      calibration_path_(param_path("calibration.json")),
      reset_request_path_(env_string("EDGEPILOT_CALIBRATION_RESET_PATH",
                                     "/dev/shm/edgepilot_calibration_reset"))
{
    fixed_rpy_[0] = config.manual_roll;
    fixed_rpy_[1] = config.manual_pitch;
    fixed_rpy_[2] = config.manual_yaw;

    StoredCalibration stored;
    std::string load_error;
    const bool loaded = load_stored_calibration(calibration_path_, &stored, &load_error);
    if (!loaded && !load_error.empty())
        std::fprintf(stderr, "calibration: ignoring %s: %s\n",
                     calibration_path_.c_str(), load_error.c_str());

    if (manual_override_) {
        restored_ = calibrator_.restore(fixed_rpy_, 5);
        last_snapshot_ = calibrator_.snapshot();
        set_fixed_projection();
        if (restored_ && (!loaded || max_rpy_delta(stored.rpy, fixed_rpy_) > 1e-7f) &&
            save_stored_calibration(params_dir_, calibration_path_, fixed_rpy_, last_snapshot_)) {
            copy_rpy(persisted_rpy_, fixed_rpy_);
            has_persisted_ = true;
            last_persist_ = std::chrono::steady_clock::now();
        } else if (loaded) {
            copy_rpy(persisted_rpy_, stored.rpy);
            has_persisted_ = true;
        }
    } else if (loaded && calibrator_.restore(stored.rpy, stored.valid_blocks, stored.spread,
                                             stored.height_m)) {
        restored_ = true;
        copy_rpy(persisted_rpy_, stored.rpy);
        has_persisted_ = true;
        last_persist_ = std::chrono::steady_clock::now();
        if (auto_enabled_) {
            calibrator_.output_rpy(online_rpy_);
            last_snapshot_ = calibrator_.snapshot();
            projection_ = make_projection_state(online_rpy_[0], online_rpy_[1], online_rpy_[2]);
        } else {
            copy_rpy(fixed_rpy_, stored.rpy);
            last_snapshot_ = calibrator_.snapshot();
            set_fixed_projection();
        }
    } else {
        set_fixed_projection();
        last_snapshot_ = calibrator_.snapshot();
    }

    persist_thread_ = std::thread(&CalibrationService::persist_loop, this);

    const char *mode = manual_override_ ? "manual" :
        (can_apply_online() ? (restored_ ? "auto-pose-restored" : "auto-pose-live") :
         (restored_ ? "stored" : "fixed"));
    std::fprintf(stderr, "calibration mode=%s overlay_auto=%d manual_override=%d\n",
                 mode, can_apply_online() ? 1 : 0, manual_override_ ? 1 : 0);
    const ProjectionState current = projection_;
    std::fprintf(stderr,
                 "projection calibration deg roll=%.3f pitch=%.3f yaw=%.3f params=%s\n",
                 rad_to_deg(current.roll), rad_to_deg(current.pitch), rad_to_deg(current.yaw),
                 calibration_path_.c_str());
}

CalibrationService::~CalibrationService()
{
    {
        std::lock_guard<std::mutex> lock(persist_mutex_);
        persist_stop_ = true;
    }
    persist_cv_.notify_one();
    if (persist_thread_.joinable()) persist_thread_.join();
}

void CalibrationService::persist_loop()
{
    std::unique_lock<std::mutex> lock(persist_mutex_);
    while (true) {
        persist_cv_.wait(lock, [this] { return persist_pending_ || persist_stop_; });
        if (!persist_pending_) return;  // 멈출 때 남은 저장은 마저 한다
        const PersistJob job = persist_job_;
        persist_pending_ = false;
        lock.unlock();
        if (job.remove) {
            if (std::remove(calibration_path_.c_str()) == 0 || errno == ENOENT)
                std::fprintf(stderr, "calibration: removed %s\n", calibration_path_.c_str());
            else
                std::fprintf(stderr, "calibration: remove %s failed: %s\n",
                             calibration_path_.c_str(), std::strerror(errno));
        } else if (save_stored_calibration(params_dir_, calibration_path_, job.rpy, job.snapshot))
            std::fprintf(stderr,
                         "calibration: saved %s rpy_deg=(%.3f %.3f %.3f) height=%.3f m validBlocks=%d\n",
                         calibration_path_.c_str(), rad_to_deg(job.rpy[0]),
                         rad_to_deg(job.rpy[1]), rad_to_deg(job.rpy[2]),
                         job.snapshot.height_m, job.snapshot.valid_blocks);
        lock.lock();
    }
}

void CalibrationService::set_fixed_projection()
{
    projection_ = make_projection_state(fixed_rpy_[0], fixed_rpy_[1], fixed_rpy_[2]);
}

void CalibrationService::input_rpy(float rpy[3]) const
{
    const float *src = can_apply_online() ? online_rpy_ : fixed_rpy_;
    rpy[0] = src[0];
    rpy[1] = src[1];
    rpy[2] = src[2];
}

const char *CalibrationService::mode_name(const OnlineCalibrator::Snapshot &snapshot) const
{
    if (!can_apply_online()) return "manual";
    return snapshot.status == CalibrationStatus::Calibrated
        ? "auto-pose-valid"
        : "auto-pose-unvalid";
}

/* 웹 콘솔의 "캘리브레이션 초기화"는 요청 파일을 만들기만 한다. 마운트를
 * 바꾼 뒤 SSH 없이 처음부터 다시 수렴시키려는 것이다. 모델 루프에서 1초에 한 번 tmpfs의
 * 파일 하나를 확인할 뿐이라 비용은 없다. 주행 중 초기화는 웹 콘솔이 해제 상태에서만
 * 받는다(openpilot의 Reset Calibration처럼). */
void CalibrationService::poll_reset_request()
{
    const auto now = std::chrono::steady_clock::now();
    if (now - last_reset_poll_ < kResetPollInterval) return;
    last_reset_poll_ = now;
    if (access(reset_request_path_.c_str(), F_OK) != 0) return;
    if (unlink(reset_request_path_.c_str()) != 0 && errno != ENOENT) {
        std::fprintf(stderr, "calibration: unlink %s failed: %s\n",
                     reset_request_path_.c_str(), std::strerror(errno));
        return;
    }
    if (!can_apply_online()) {
        std::fprintf(stderr, "calibration: reset request ignored (manual calibration)\n");
        return;
    }
    reset_online();
}

void CalibrationService::reset_online()
{
    calibrator_ = OnlineCalibrator();
    calibrator_.output_rpy(online_rpy_);
    last_snapshot_ = calibrator_.snapshot();
    projection_ = make_projection_state(online_rpy_[0], online_rpy_[1], online_rpy_[2]);
    restored_ = false;
    has_persisted_ = false;
    copy_rpy(persisted_rpy_, online_rpy_);
    last_valid_blocks_ = -1;
    {
        // 아직 쓰지 않은 저장이 있으면 버리고, 파일을 지운다. 새 값은 다시 수렴하면 저장된다.
        std::lock_guard<std::mutex> lock(persist_mutex_);
        persist_job_ = PersistJob{};
        persist_job_.remove = true;
        persist_pending_ = true;
    }
    persist_cv_.notify_one();
    std::fprintf(stderr, "calibration: reset requested, recalibrating from scratch\n");
}

void CalibrationService::update(const ParsedModelOutput &output, float v_ego)
{
    poll_reset_request();
    if (!can_apply_online()) return;
    if (!output.has_pose) return;

    const OnlineCalibrator::UpdateResult result = calibrator_.update(output.pose, v_ego);
    last_snapshot_ = result.snapshot;
    calibrator_.output_rpy(online_rpy_);
    projection_ = make_projection_state(online_rpy_[0],
                                        online_rpy_[1],
                                        online_rpy_[2]);

    maybe_persist(result);
    maybe_log(result);
}

void CalibrationService::maybe_persist(const OnlineCalibrator::UpdateResult &result)
{
    if (!result.block_completed || result.snapshot.status != CalibrationStatus::Calibrated)
        return;

    const auto now = std::chrono::steady_clock::now();
    if (has_persisted_) {
        if (now - last_persist_ < kPersistInterval) return;
        if (max_rpy_delta(persisted_rpy_, online_rpy_) < kPersistDeltaRad) return;
    }
    {
        std::lock_guard<std::mutex> lock(persist_mutex_);
        copy_rpy(persist_job_.rpy, online_rpy_);
        persist_job_.snapshot = result.snapshot;
        // 초기화가 남긴 삭제 표시를 지운다. 두면 초기화 뒤 저장이 매번 파일 삭제가 되어,
        // 재시작하면 저장소 기본값(upload_to_board.sh가 빈 자리에 채움)으로 돌아간다.
        persist_job_.remove = false;
        persist_pending_ = true;
    }
    persist_cv_.notify_one();
    copy_rpy(persisted_rpy_, online_rpy_);
    has_persisted_ = true;
    last_persist_ = now;
}

void CalibrationService::maybe_log(const OnlineCalibrator::UpdateResult &result)
{
    if (!log_enabled_) return;

    const auto now = std::chrono::steady_clock::now();
    const bool first = last_log_.time_since_epoch().count() == 0;
    const bool elapsed = first || now - last_log_ >= std::chrono::seconds(1);
    const bool status_changed = result.snapshot.status != last_status_ ||
        result.snapshot.valid_blocks != last_valid_blocks_;
    if (!elapsed && !result.block_completed && !status_changed) return;

    last_log_ = now;
    last_status_ = result.snapshot.status;
    last_valid_blocks_ = result.snapshot.valid_blocks;

    std::fprintf(stderr,
                 "\ncalib mode=%s status=%s accepted=%llu rejected=%llu "
                 "validBlocks=%d blockSamples=%d last=%s rpy_deg=(%.3f %.3f %.3f) "
                 "spread_deg=(%.3f %.3f %.3f) height=%.3f m\n",
                 mode_name(result.snapshot),
                 calibration_status_name(result.snapshot.status),
                 static_cast<unsigned long long>(result.snapshot.accepted_samples),
                 static_cast<unsigned long long>(result.snapshot.rejected_samples),
                 result.snapshot.valid_blocks,
                 result.snapshot.block_sample_count,
                 result.accepted ? "accepted" : "rejected",
                 rad_to_deg(result.snapshot.rpy[0]),
                 rad_to_deg(result.snapshot.rpy[1]),
                 rad_to_deg(result.snapshot.rpy[2]),
                 rad_to_deg(result.snapshot.spread[0]),
                 rad_to_deg(result.snapshot.spread[1]),
                 rad_to_deg(result.snapshot.spread[2]),
                 result.snapshot.height_m);
}
