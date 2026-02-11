# Monocular + IMU (Mono-Inertial) Setup Guide

This repository is now configured to **build in mono-inertial-only mode by default** (`BUILD_MONO_INERTIAL_ONLY=ON`).
That mode compiles only:

- `mono_inertial_euroc`
- `mono_inertial_tum_vi`

If you want all camera modes back (stereo, RGB-D, etc.), configure CMake with:

```bash
cmake -S . -B build -DBUILD_MONO_INERTIAL_ONLY=OFF
```

---

## 1) Ubuntu 18.04 build notes

This project accepts **OpenCV >= 3.0** (Ubuntu 18.04 commonly has OpenCV 3.2), plus Eigen3 and Pangolin.

Typical build flow:

```bash
chmod +x build.sh
./build.sh
```

Or with explicit CMake:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DBUILD_MONO_INERTIAL_ONLY=ON
cmake --build build -j$(nproc)
```

---

## 2) Test first on EuRoC (recommended)

1. Download a EuRoC sequence (ASL format), for example `MH_01_easy`.
2. Assume dataset path: `/data/EuRoC/MH_01_easy`.
3. Run:

```bash
./Examples/Monocular-Inertial/mono_inertial_euroc \
  Vocabulary/ORBvoc.txt \
  ./Examples/Monocular-Inertial/EuRoC.yaml \
  /data/EuRoC/MH_01_easy \
  ./Examples/Monocular-Inertial/EuRoC_TimeStamps/MH01.txt
```

If tracking is unstable, first verify:
- timestamps are strictly increasing,
- IMU and camera are time-synchronized,
- EuRoC settings file path is correct.

---

## 3) Move to your own monocular camera + IMU

For your sensor, prepare one YAML like `Examples/Monocular-Inertial/my_mono_imu.yaml`.

Critical parameters to set correctly:

1. **Camera intrinsics + distortion**
   - `Camera.fx`, `Camera.fy`, `Camera.cx`, `Camera.cy`
   - distortion coefficients
   - image width/height

2. **Frame rate + IMU rate**
   - `Camera.fps`
   - `IMU.Frequency`

3. **Camera-IMU extrinsics**
   - `Tbc` (transform body/IMU to camera, as required by the file format)

4. **IMU noise parameters**
   - `IMU.NoiseGyro`, `IMU.NoiseAcc`
   - `IMU.GyroWalk`, `IMU.AccWalk`

Use `Calibration_Tutorial.pdf` for calibration workflow.

---

## 4) Practical bring-up checklist for your own sensor

1. Start with static motion (sensor standing still) for a few seconds.
2. Then do smooth handheld motion with:
   - some translation,
   - some rotation,
   - enough texture in scene.
3. Avoid fast shaking and severe motion blur during initial tests.
4. Check that IMU units match expected units:
   - gyro in **rad/s**,
   - accelerometer in **m/s²**.
5. Verify camera and IMU timestamps share the same clock domain.

---

## 5) ROS usage for mono-inertial

If you use ROS (Melodic on Ubuntu 18.04):

```bash
rosrun ORB_SLAM3 Mono_Inertial Vocabulary/ORBvoc.txt PATH_TO_YOUR_YAML
```

Expected topics:
- image: `/camera/image_raw`
- IMU: `/imu`

If your topics differ, remap them in launch/CLI.
