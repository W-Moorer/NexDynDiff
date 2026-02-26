# Double Pendulum Acceleration Alignment Fix Summary

Date: 2026-02-26

## 1. Problem
- `RunDoublePendulumScene.cpp` exported acceleration by reading `RigidBodyHandler::get_acceleration()`.
- In current engine semantics, that field corresponds to prescribed/external acceleration terms and is not the solved kinematic acceleration time history.
- Result: exported `Link*_Acc_NexDynDiff.csv` contained near-zero values at many samples, causing inflated acceleration comparison errors.

## 2. Fix Implemented
- Added reusable utility:
  - `nexdyndiff/src/utils/TimeSeriesUtils.h`
  - `nexdyndiff::DifferentiateTimeSeries(time, values, min_dt)`
- Integrated utility export:
  - `nexdyndiff/src/utils/include.h`
  - `nexdyndiff/CMakeLists.txt` (`UTILS_FILES`)
- Updated runner:
  - `tools/RunDoublePendulumScene.cpp`
  - Acceleration is now computed from velocity time series via `DifferentiateTimeSeries(...)` and written back to CSV outputs.

## 3. Tests Added
- New test file:
  - `tests/TimeSeriesUtils.cpp`
- Test coverage:
  - Size mismatch returns empty derivative.
  - Scalar linear signal derivative correctness.
  - `Eigen::Vector3d` linear signal derivative correctness.
  - Repeated timestamp guard (`min_dt`) yields finite outputs.
- Build integration:
  - `tests/CMakeLists.txt` includes `TimeSeriesUtils.cpp`.

## 4. Verification Results
- Build:
  - `cmake --build build --config Release --target nexdyndiff_tests DoublePendulumRunner`
  - Success.
- Unit tests:
  - `build/tests/Release/nexdyndiff_tests.exe`
  - Passed: **23 test cases, 93 assertions**.
- Double pendulum rerun:
  - `build/tools/Release/DoublePendulumRunner.exe models/double_pendulum_scene.json output/double_pendulum`
  - Generated 3001 samples.
- Comparison rerun:
  - `python tools/CompareDoublePendulum.py --reference-dir assets/models/double_pendulum --simulation-dir output/double_pendulum --figure-dir output/figures/double_pendulum`
  - Summary (`output/double_pendulum/DoublePendulumComparisonSummary.txt`):
    - Pos average_rmse: `4.59567043e-03`
    - Vel average_rmse: `2.05272296e-02`
    - Acc average_rmse: `1.28005449e-01`

## 5. Output Artifacts
- Scene JSON: `models/double_pendulum_scene.json`
- Simulation CSVs: `output/double_pendulum/*.csv`
- Comparison metrics/summary:
  - `output/double_pendulum/DoublePendulumComparisonMetrics.csv`
  - `output/double_pendulum/DoublePendulumComparisonSummary.txt`
- Figures (Times New Roman):
  - `output/figures/double_pendulum/Pos_Comparison.png`
  - `output/figures/double_pendulum/Vel_Comparison.png`
  - `output/figures/double_pendulum/Acc_Comparison.png`

