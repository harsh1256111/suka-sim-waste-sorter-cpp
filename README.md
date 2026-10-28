# SUKA-Sim: AI-Vision Dry Waste Sorting Controller & Qt HMI (`C++17` / `Qt` / `Linux` / `GitLab CI`)

An industrial C++17 and Qt Framework simulation of an optical-pneumatic dry waste sorting line inspired by modern Material Recovery Facilities (MRFs) and AI air-sorting machines.

## Key Engineering Highlights (Aligned with Industrial Robotics & Waste Sorting)

1. **Deterministic Time-of-Flight (TOF) Valve Timing (`C++17`)**:
   - Computes real-time actuation delays (`ms`) from the optical camera scan-line to a 32-nozzle pneumatic air bar using live rotary encoder belt velocity (`m/s`).
   - Adapts air-jet pulse duration (`10 ms – 45 ms`) dynamically based on estimated object mass (`g`) and bounding-box surface area (`mm²`).
2. **Multi-Polymer Optical Classification Routing**:
   - Segregates `PET_CLEAR`, `PET_COLORED`, `HDPE`, `PP`, `MLP` (Multi-Layered Plastic), and `BLACK_PLASTIC` across 4 dedicated high-speed pneumatic valve banks while rejecting low-confidence inferences (`< 0.78`).
3. **Hardware Safety Interlocks & Thread-Safe Telemetry**:
   - Protects hardware using `std::mutex`-guarded state checks for pneumatic manifold pressure drops (`< 4.5 bar`), belt stalls (`<= 0.1 m/s`), and operator Emergency Stop (`E-STOP`) triggers.
4. **Qt Widgets Operator HMI (`src/qt_hmi/`)**:
   - Provides a real-time operator dashboard using Qt Signals & Slots (`QMainWindow`, `QTimer`, `QTableWidget`, `QDoubleSpinBox`) to adjust conveyor belt speed, monitor yield/mass recovery, and trigger emergency stops.
5. **Automated Unit Tests & GitLab/GitHub CI/CD Pipelines**:
   - Includes a 6-test assertion suite (`--test`), `Makefile`, `CMakeLists.txt`, `.gitlab-ci.yml`, and `.github/workflows/ci.yml` for automated Linux build verification.

## Project Structure

```text
ishitva-waste-sorter-hmi/
├── include/
│   └── sorting_engine.hpp          # Thread-safe C++17 optical-pneumatic sorting controller
├── src/
│   ├── ishitva_suka_waste_sorter.cpp # CLI simulator + built-in unit test runner
│   └── qt_hmi/
│       ├── hmi_mainwindow.hpp      # Qt Widgets HMI MainWindow header (Signals/Slots)
│       └── hmi_mainwindow.cpp      # Qt Widgets HMI implementation
├── CMakeLists.txt                  # CMake build configuration (CLI + optional Qt5/Qt6 GUI)
├── Makefile                        # Fast Linux g++ Makefile (all, test, simulate)
├── .gitlab-ci.yml                  # GitLab CI/CD automated build & test pipeline
└── .github/workflows/ci.yml        # GitHub Actions CI workflow
```

## Quick Start (Linux Terminal)

```bash
# 1. Compile with g++ (C++17, zero warnings)
make all

# 2. Run the automated unit test suite
make test

# 3. Run the live conveyor sorting simulation
make simulate
```

## How to Push This Project to Your GitHub / GitLab

```bash
unzip ishitva-waste-sorter-hmi.zip -d ishitva-waste-sorter-hmi
cd ishitva-waste-sorter-hmi
git init
git add .
git commit -m "feat: add C++17 optical-pneumatic dry waste sorting controller, Qt HMI, and CI pipeline"
git branch -M main
git remote add origin https://github.com/<your-username>/ishitva-waste-sorter-hmi.git
git push -u origin main
```

## Resume Bullet Points You Can Copy-Paste

* **Optical-Pneumatic Dry Waste Sorting Controller & Qt HMI (`C++17`, `Qt`, `Linux`, `GitLab CI`)**: Engineered a thread-safe C++17 control engine simulating high-speed segregation of `PET`, `HDPE`, `PP`, `MLP`, and black plastics across a 32-nozzle pneumatic valve manifold.
* **Real-Time Sensor-to-Actuator Timing & Safety Interlocks**: Implemented deterministic time-of-flight nozzle delay calculations (`ms`) based on conveyor encoder velocity (`2.5 m/s`) and mass-adaptive air-pulse scaling, guarded by pneumatic manifold pressure (`< 4.5 bar`) and `E-STOP` interlocks.
* **Automated Testing & CI/CD Pipeline**: Built a Qt Widgets operator HMI utilizing Signals & Slots and automated Linux builds and unit tests via `CMake`, `Makefile`, and `.gitlab-ci.yml`.
