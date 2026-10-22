#!/usr/bin/env python3
"""tools/model/recording_reader.py의 바이너리 배치가 C++ 헤더의 고정값과 같은지 본다:
ControlState·LearnerState·ModelState의 offsetof 고정값과 크기, LocalizationState·ImuSample·
CalibrationState 크기, 이벤트 레코드 머리·프레임 인덱스 레코드 크기, 기록 타입 번호, 읽는 녹화 버전 범위. 이벤트
파일을 고르고 끊긴 꼬리에서 끝내는 규칙이 C++ 리더와 같은지, iter_states가 상태와 CAN·IMU 묶음을 읽는지 본다.
도구가 쓰는 MaixCAM2 카메라 내부 파라미터(tools/model/model_warp.py)가 src/common/app_config.h와 같은지, 그 워프 행렬이
런타임(gtest_model_input_transform의 기준값)과 같은지도 본다.
ctest가 저장소 루트에서 돌린다. 두 도구가 numpy를 쓰므로 numpy가 없으면 77로 끝나고 ctest는 건너뛴 것으로 센다."""

import re
import struct
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools" / "model"))

try:
    import numpy as np
except ModuleNotFoundError:
    print("skipped: numpy is not installed")
    sys.exit(77)

import model_warp  # noqa: E402
from cpp_headers import IPC, cpp_flags, cpp_offsets, cpp_size, pinned  # noqa: E402
import recording_reader as rr  # noqa: E402

FORMAT = (ROOT / "src" / "recording" / "recording_format.h").read_text(encoding="utf-8")
EVENT_LOG_READER = (ROOT / "src" / "recording" / "event_log_reader.h").read_text(encoding="utf-8")
APP_CONFIG = (ROOT / "src" / "common" / "app_config.h").read_text(encoding="utf-8")
MODEL_OUTPUT = (ROOT / "src" / "common" / "model_output.h").read_text(encoding="utf-8")


def struct_fields(name: str, text: str) -> list[tuple[str, str, int]]:
    """C++ 구조체의 (형식, 이름, 배열 길이) 목록. uint64_t·uint32_t·int32_t·float·uint8_t 필드만 있는 구조체용."""
    body = re.search(r"struct " + name + r" \{(.*?)\n\};", text, re.S).group(1)
    return [(kind, field, int(count or 1)) for kind, field, count in
            re.findall(r"(uint64_t|uint32_t|int32_t|float|uint8_t) (\w+)(?:\[(\d+)\])?", body)]


def dtype_fields(dtype) -> list[tuple[str, str, int]]:
    kinds = {"<u8": "uint64_t", "<u4": "uint32_t", "<i4": "int32_t", "<f4": "float", "|u1": "uint8_t"}
    out = []
    for name in dtype.names:
        field = dtype.fields[name][0]
        base = field.base if field.shape else field
        out.append((kinds[base.str], name, int(np.prod(field.shape)) if field.shape else 1))
    return out


def offset_in(dtype, name: str) -> int:
    return dtype.fields[name][1]


class RecordingReaderLayout(unittest.TestCase):
    def test_control_state(self):
        offsets = cpp_offsets("CONTROL_STATE")
        self.assertGreater(len(offsets), 40)
        for name, offset in offsets.items():
            self.assertIn(name, rr.CONTROL_STATE.names, "the C++ field names")
            self.assertEqual(offset_in(rr.CONTROL_STATE, name), offset, name)
        # C++ 구조체는 uint64_t로 시작해 8바이트 경계까지 끝을 덧댄다. dtype에는 그 덧댐이 없다.
        self.assertEqual(-(-rr.CONTROL_STATE.itemsize // 8) * 8, cpp_size("ControlState"))

    def test_learner_state(self):
        offsets = cpp_offsets("LEARNER_STATE")
        self.assertGreater(len(offsets), 5)
        for name, offset in offsets.items():
            self.assertEqual(offset_in(rr.LEARNER_STATE, name), offset, name)
        self.assertEqual(rr.LEARNER_STATE.itemsize, cpp_size("LearnerState"))

    def test_localization_and_imu(self):
        """크기만 대조하면 필드 순서가 바뀌어도 모른다: 형식·이름·배열 길이를 차례대로 대조한다."""
        self.assertEqual(rr.LOCALIZATION_STATE.itemsize, cpp_size("LocalizationState"))
        self.assertEqual(dtype_fields(rr.LOCALIZATION_STATE), struct_fields("LocalizationState", IPC))
        self.assertEqual(rr.IMU_SAMPLE.itemsize, cpp_size("ImuSample"))
        self.assertEqual(dtype_fields(rr.IMU_SAMPLE), struct_fields("ImuSample", IPC))
        self.assertEqual(rr.IMU_BATCH_HEAD.size, pinned(r"sizeof\(ImuBatch\) == (\d+) \+"), "ImuBatch head")

    def test_model_state(self):
        self.assertEqual(rr.RECORDING_VERSION, pinned(r"kRecordingVersion = (\d+)", FORMAT))
        layout = {name: rr.MODEL_STATE.fields[name][1] for name in rr.MODEL_STATE.names}
        self.assertEqual(rr.MODEL_STATE.itemsize, cpp_size("ModelState"))
        offsets = cpp_offsets("MODEL_STATE")
        self.assertGreater(len(offsets), 5)
        for name, offset in offsets.items():
            self.assertEqual(layout[name], offset, name)
        for name in ("calibration", "plan_yaw", "camera_offset_m", "gas_press_probs", "flow_calibration",
                     "calibration_source", "lead_flow", "lead_track"):
            self.assertEqual(layout[name], pinned(r"offsetof\(ModelState, " + name + r"\) == (\d+)"), name)
        self.assertEqual(rr.CALIBRATION_STATE.itemsize, cpp_size("CalibrationState"))
        self.assertEqual(dtype_fields(rr.POSE_STATE), struct_fields("PoseState", IPC))
        self.assertEqual(dtype_fields(rr.LEAD_STATE), struct_fields("LeadState", IPC))
        self.assertEqual(dtype_fields(rr.FLOW_CALIBRATION_STATE), struct_fields("FlowCalibrationState", IPC))
        self.assertEqual(dtype_fields(rr.CALIBRATION_SOURCE_STATE), struct_fields("CalibrationSourceState", IPC))
        self.assertEqual(dtype_fields(rr.LEAD_FLOW_STATE), struct_fields("LeadFlowState", IPC))
        self.assertEqual(dtype_fields(rr.LEAD_TRACK_STATE), struct_fields("LeadTrackState", IPC))

    def test_magics(self):
        def magic(name: str) -> bytes:
            chars = re.search(name + r"\[8\] = \{([^}]*)\}", FORMAT).group(1)
            return "".join(re.findall(r"'(.)'", chars)).encode()
        self.assertEqual(rr.EVENT_LOG_MAGIC, magic("kEventLogMagic"))
        self.assertEqual(rr.FRAME_INDEX_MAGIC, magic("kFrameIndexMagic"))

    def test_recording_format(self):
        self.assertEqual(rr.MAX_RECORD_PAYLOAD, 1 << pinned(r"kMaxPayloadBytes = 1U << (\d+);", EVENT_LOG_READER))
        self.assertEqual(rr.EVENT_RECORD_HEADER.size, cpp_size("EventRecordHeader", FORMAT))
        self.assertEqual(rr.RECORDED_CAN_FRAME.itemsize, cpp_size("RecordedCanFrame", FORMAT))
        self.assertEqual(dtype_fields(rr.RECORDED_CAN_FRAME), struct_fields("RecordedCanFrame", FORMAT))
        self.assertEqual(len(struct_fields("RecordedCanBatchHeader", FORMAT)) * 4, rr.RECORDED_CAN_BATCH_HEAD.size)
        self.assertEqual(rr.FRAME_INDEX_RECORD.itemsize, cpp_size("FrameIndexRecord", FORMAT))
        body = re.search(r"enum class RecordType : uint16_t \{(.*?)\};", FORMAT, re.S).group(1)
        types = {name: int(value) for name, value in re.findall(r"(\w+) = (\d+),", body)}
        self.assertEqual(len(types), 8)
        for name, value in types.items():
            constant = "RECORD_" + re.sub(r"(?<!^)(?=[A-Z])", "_", name).upper()
            self.assertEqual(getattr(rr, constant), value, constant)
        self.assertEqual(rr.LAST_RECORD_TYPE, max(types.values()))


class EventFiles(unittest.TestCase):
    """끊긴 꼬리에서 끝내는 규칙이 C++ 리더(EventLogReader)와 같다."""

    @staticmethod
    def event_file(path: Path, tail: bytes = b"") -> Path:
        header = struct.pack("<8sIIQQ", b"EDGELOG1", rr.RECORDING_VERSION, struct.calcsize("<8sIIQQ"), 0, 0)
        record = rr.EVENT_RECORD_HEADER.pack(123, rr.RECORD_CONTROL_STATE, 0, 4) + b"\x11" * 4
        path.write_bytes(header + record + tail)
        return path

    def test_stops_where_the_cpp_reader_stops(self):
        with tempfile.TemporaryDirectory() as tmp:
            for name, tail in (("zero", b"\0" * 32),  # tmpfs가 차서 0으로 채워진 꼬리
                               ("type", rr.EVENT_RECORD_HEADER.pack(1, rr.LAST_RECORD_TYPE + 1, 0, 0)),
                               ("long", rr.EVENT_RECORD_HEADER.pack(1, rr.RECORD_CAN_RX, 0, rr.MAX_RECORD_PAYLOAD + 1)
                                + b"\0" * (rr.MAX_RECORD_PAYLOAD + 1))):
                path = self.event_file(Path(tmp) / f"{name}.bin", tail=tail)
                index = rr.record_index(path, path.read_bytes())
                self.assertEqual(index[["timestamp_ns", "type"]].tolist(), [(123, rr.RECORD_CONTROL_STATE)], name)

    def test_route_without_segments(self):
        """segments/가 없는 route는 영상 세그먼트가 없을 뿐이다(일괄 실행이 멈추지 않는다)."""
        with tempfile.TemporaryDirectory() as tmp:
            self.assertEqual(rr.route_segments(Path(tmp)), [])


class States(unittest.TestCase):
    """iter_states: 상태 레코드는 레코드마다, CAN·IMU 묶음은 프레임·샘플마다 한 줄이고 그 레코드의 머리 시각을 받는다.
    줄은 청크 버퍼를 붙잡지 않는 사본이다. 크기가 다른 상태 페이로드(배치가 바뀌었다)는 받지 않는다."""

    @staticmethod
    def write_route(root: Path, records: list[tuple[int, int, bytes]]) -> Path:
        header = struct.pack("<8sIIQQ", b"EDGELOG1", rr.RECORDING_VERSION, struct.calcsize("<8sIIQQ"), 0, 0)
        (root / "events").mkdir()
        (root / "events" / "000.bin").write_bytes(header + b"".join(
            rr.EVENT_RECORD_HEADER.pack(ts, rtype, 0, len(payload)) + payload for ts, rtype, payload in records))
        (root / "events" / "001.bin").write_bytes(header)  # 머리만 있는 청크
        return root

    def test_reads_states_and_batches(self):
        model = np.zeros(2, rr.MODEL_STATE)
        model["frame_id"] = [42, 43]
        model["lanes"][0, 1, :, 0] = -1.75
        model["lead_track"]["relative_speed_mps"][1] = -2.25
        control = np.zeros(1, rr.CONTROL_STATE)
        control["ego_speed_kph"] = 61.5
        frames = np.zeros(2, rr.RECORDED_CAN_FRAME)
        frames["address"] = [0x251, 0x484]
        frames["data"][1, :8] = range(1, 9)
        sample = np.zeros(1, rr.IMU_SAMPLE)
        sample["timestamp_ns"] = 7
        records = [(10, rr.RECORD_MODEL_STATE, model[0].tobytes()),
                   (11, rr.RECORD_CONTROL_STATE, control.tobytes() + bytes(rr.CONTROL_STATE_SIZE - control.nbytes)),
                   (12, rr.RECORD_CAN_RX, rr.RECORDED_CAN_BATCH_HEAD.pack(2, 0) + frames.tobytes()),
                   (13, rr.RECORD_IMU, rr.IMU_BATCH_HEAD.pack(0, 1, 0) + sample.tobytes()),
                   (14, rr.RECORD_MODEL_STATE, model[1].tobytes())]
        types = (rr.RECORD_MODEL_STATE, rr.RECORD_CONTROL_STATE, rr.RECORD_CAN_RX, rr.RECORD_IMU)
        with tempfile.TemporaryDirectory() as tmp:
            chunks = list(rr.iter_states(self.write_route(Path(tmp), records), *types))
        self.assertEqual(len(chunks), 2)
        ts, models = chunks[0][rr.RECORD_MODEL_STATE]
        self.assertEqual((ts.tolist(), models["frame_id"].tolist()), ([10, 14], [42, 43]))
        self.assertAlmostEqual(float(models["lanes"][0, 1, 5, 0]), -1.75)
        self.assertAlmostEqual(float(models["lead_track"]["relative_speed_mps"][1]), -2.25)
        self.assertEqual(models.base.nbytes, models.nbytes, "줄은 청크 버퍼가 아닌 자기 사본을 가리킨다")
        ts, controls = chunks[0][rr.RECORD_CONTROL_STATE]
        self.assertEqual((ts.tolist(), controls["ego_speed_kph"].tolist()), ([11], [61.5]))
        ts, can = chunks[0][rr.RECORD_CAN_RX]
        self.assertEqual((ts.tolist(), can["address"].tolist()), ([12, 12], [0x251, 0x484]))
        self.assertEqual(can["data"][1, :8].tolist(), list(range(1, 9)))
        ts, imu = chunks[0][rr.RECORD_IMU]
        self.assertEqual((ts.tolist(), imu["timestamp_ns"].tolist()), ([13], [7]))
        for rtype in types:
            self.assertEqual(len(chunks[1][rtype][1]), 0, rtype)

    def test_rejects_a_changed_state_layout(self):
        with tempfile.TemporaryDirectory() as tmp:
            route = self.write_route(Path(tmp), [(1, rr.RECORD_MODEL_STATE, bytes(3736))])
            with self.assertRaises(ValueError):
                list(rr.iter_states(route, rr.RECORD_MODEL_STATE))


class LaneBenchmark(unittest.TestCase):
    """tools/model/lane_benchmark.py의 순정 차선 신호 해석과 모델 차선의 3차 맞춤."""

    def test_decodes_lka14(self):
        import lane_benchmark as lb
        # 왼쪽(비트 0~31): 품질 3, 위치 -1.25 m, 방향각 0.01 rad.
        # 오른쪽(비트 32~): 품질 1, 위치 +1.5 m, 방향각 -0.02 rad.
        def side(quality, position_m, heading_rad):
            position = round(position_m * 256) & 0x3FF
            heading = round((heading_rad + 0.0625) / 1.52588e-5)
            return quality | position << 8 | heading << 18
        raw = np.array([side(3, -1.25, 0.01) | side(1, 1.5, -0.02) << 32], dtype=np.uint64)
        lanes = lb.decode_lka14(raw)
        self.assertEqual((int(lanes["left_quality"][0]), int(lanes["right_quality"][0])), (3, 1))
        self.assertAlmostEqual(float(lanes["left_position"][0]), -1.25)
        self.assertAlmostEqual(float(lanes["right_position"][0]), 1.5)
        self.assertAlmostEqual(float(lanes["left_heading"][0]), 0.01, places=4)
        self.assertAlmostEqual(float(lanes["right_heading"][0]), -0.02, places=4)

    def test_cubic_fit_reads_position_and_heading_at_x0(self):
        import lane_benchmark as lb
        a, b, c, d = 1.7, 0.02, -3e-4, 2e-6  # 3차식 차선이면 맞춤이 그대로 나온다
        x = lb.X_IDXS
        y = a + b * x + c * x ** 2 + d * x ** 3
        position, heading = lb.lane_position_heading(y[None, :])
        x0 = lb.X0_M
        self.assertAlmostEqual(float(position[0]), a + b * x0 + c * x0 ** 2 + d * x0 ** 3, places=6)
        self.assertAlmostEqual(float(heading[0]), float(np.arctan(b + 2 * c * x0 + 3 * d * x0 ** 2)), places=6)

    def test_lane_change_flag_is_the_runtime_bit(self):
        import lane_benchmark as lb
        self.assertEqual(lb.HUD_LANE_CHANGING, 1 << cpp_flags("kHudFlag")["LaneChanging"])

class CameraIntrinsics(unittest.TestCase):
    def test_maixcam2_matches_app_config(self):
        for axis, value in (("Fx", model_warp.MAIXCAM2_FX_1080), ("Fy", model_warp.MAIXCAM2_FY_1080),
                            ("Cx", model_warp.MAIXCAM2_CX_1080), ("Cy", model_warp.MAIXCAM2_CY_1080)):
            match = re.search(r"constexpr float kCamera" + axis + r" = ([\d.]+)f;", APP_CONFIG)
            self.assertIsNotNone(match, axis)
            self.assertEqual(float(match.group(1)), value, axis)

    def test_default_camera_height_matches_the_runtime(self):
        """장착을 모를 때의 높이(model_warp.CAMERA_HEIGHT_M)가 kModelHeight(기기 설정 기본값)와 같다."""
        match = re.search(r"constexpr float kModelHeight = ([\d.]+)f;", MODEL_OUTPUT)
        self.assertIsNotNone(match)
        self.assertEqual(float(match.group(1)), model_warp.CAMERA_HEIGHT_M)


class ModelWarp(unittest.TestCase):
    def test_projection_matches_the_cpp_golden(self):
        """model_warp.projection_matrix가 런타임(ModelInputTransform)과 같은 행렬을 낸다. 기준값은
        gtest_model_input_transform ProjectionGoldenSharedWithModelWarpPy가 C++ 코드로 확인하는 것이다."""
        source = (ROOT / "tests" / "gtest_model_input_transform.cc").read_text(encoding="utf-8")
        block = source[source.index("// GOLDEN BEGIN: roll"):source.index("// GOLDEN END")]
        number = r"(-?[\d.]+(?:e-?\d+)?)"
        cases = re.findall(r"\{" + r"f?,\s*".join([number] * 3) + r"f,\s*(true|false),\s*" + number + r"f,\s*"
                           + number + r"f,\s*\{([^}]*)\}\}", block)
        self.assertEqual(len(cases), 5)
        intrinsics = model_warp.maixcam2_intrinsics(1280, 720)
        for roll, pitch, yaw, sbig, offset, height, values in cases:
            expected = np.array([float(v) for v in values.split(",")])
            actual = model_warp.projection_matrix(*intrinsics, np.array([float(roll), float(pitch), float(yaw)]),
                                                  sbig == "true", float(offset), float(height)).ravel()
            tolerance = np.maximum(1.0, np.abs(expected)) * 1e-5
            tolerance[6:8] = 1e-9
            np.testing.assert_array_less(np.abs(actual - expected), tolerance, f"{roll} {pitch} {yaw} {sbig}")


class ModelTools(unittest.TestCase):
    def test_desire_queue_matches_the_runtime(self):
        """make_m2_data.py의 desire 큐가 런타임(model_temporal.h set_desire)과 같은 펄스를 남긴다. 기준값은
        gtest_model_output(DesireSequenceMatchesTheToolQueue)과 같은 수열의 것이다. 갈리면 eval 기준과 양자화 표본이
        보드 입력과 달라진다(회전 펄스 지우기가 빠졌던 적이 있다)."""
        sys.path.insert(0, str(ROOT / "tools" / "model" / "axmodel"))
        import make_m2_data
        history, prev = np.zeros((100, 8), np.float32), np.zeros(8, np.float32)
        for desire in (0, 3, 3, 0, 1, 1, 1, 0, 0, 2, 2, 3, 0, 1, 1, 4, 4, 0, 1, 1, 2):
            history, prev = make_m2_data.desire_step(history, prev, desire)
        pulses = sorted((99 - int(t), int(c)) for t, c in zip(*np.nonzero(history)))
        self.assertEqual(pulses, [(0, 2), (2, 1), (5, 4), (9, 3), (19, 3)])


if __name__ == "__main__":
    unittest.main()
