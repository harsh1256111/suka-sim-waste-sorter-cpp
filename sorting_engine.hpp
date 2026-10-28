#ifndef ISHITVA_SORTING_ENGINE_HPP
#define ISHITVA_SORTING_ENGINE_HPP

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <map>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace waste_sorter {

/**
 * @brief Dry waste polymer categories inspired by NIR/RGB optical sorting systems.
 */
enum class PolymerType {
    PET_CLEAR,      // Polyethylene Terephthalate (Clear bottles)
    PET_COLORED,    // Colored PET
    HDPE,           // High-Density Polyethylene (Milk jugs, shampoo bottles)
    PP,             // Polypropylene (Caps, food containers)
    MLP,            // Multi-Layered Plastic (Snack wrappers)
    BLACK_PLASTIC,  // Carbon-black plastic (Difficult NIR, detected via AI vision)
    CONTAMINANT     // Non-recyclable / residue / metal / stone
};

/**
 * @brief Pneumatic air-jet nozzle bank assignments on the sorting chute.
 */
enum class NozzleBank : int {
    PASS_THROUGH = 0, // Drops into residue/contaminant bin without air pulse
    BANK_A_PET   = 1, // High-speed air valve bank 1 (PET stream)
    BANK_B_HDPE  = 2, // High-speed air valve bank 2 (HDPE stream)
    BANK_C_PP    = 3, // High-speed air valve bank 3 (PP & Black Plastic stream)
    BANK_D_MLP   = 4  // High-speed air valve bank 4 (MLP recovery stream)
};

/**
 * @brief Bounding box and optical sensor telemetry emitted by the AI Vision node.
 */
struct VisionDetection {
    uint64_t item_id;
    PolymerType polymer;
    double confidence;          // [0.0, 1.0] model inference confidence
    double bbox_center_x_mm;    // Lateral position across conveyor width (0 - 1200 mm)
    double bbox_area_mm2;       // Estimated surface area in mm^2
    double estimated_mass_g;    // Estimated object weight in grams
    uint64_t capture_time_ms;   // Monotonic timestamp when item crossed camera line
};

/**
 * @brief Hardware actuator command sent to the PLC / Pneumatic Valve Controller.
 */
struct ValveActuationCommand {
    uint64_t item_id;
    NozzleBank bank;
    int nozzle_index;           // Specific valve index [0..31] across the 1200mm bar
    uint64_t trigger_delay_ms;  // Delay from camera line to nozzle line based on belt speed
    uint32_t pulse_duration_ms; // Valve open duration scaled by item mass/area
    bool valid;
};

/**
 * @brief Real-time operational metrics for Qt HMI display and SQLite/CSV persistence.
 */
struct TelemetrySnapshot {
    uint64_t total_inspected = 0;
    uint64_t total_sorted = 0;
    uint64_t rejected_low_confidence = 0;
    uint64_t safety_interlock_aborts = 0;
    double total_recovered_mass_g = 0.0;
    double current_belt_speed_mps = 0.0;
    double air_manifold_pressure_bar = 0.0;
    std::map<PolymerType, uint64_t> counts_by_polymer;
};

/**
 * @brief Convert PolymerType enum to human-readable string.
 */
inline std::string polymerToString(PolymerType type) {
    switch (type) {
        case PolymerType::PET_CLEAR:     return "PET_CLEAR";
        case PolymerType::PET_COLORED:   return "PET_COLORED";
        case PolymerType::HDPE:          return "HDPE";
        case PolymerType::PP:            return "PP";
        case PolymerType::MLP:           return "MLP";
        case PolymerType::BLACK_PLASTIC: return "BLACK_PLASTIC";
        case PolymerType::CONTAMINANT:   return "CONTAMINANT";
    }
    return "UNKNOWN";
}

/**
 * @brief Convert NozzleBank enum to human-readable string.
 */
inline std::string nozzleBankToString(NozzleBank bank) {
    switch (bank) {
        case NozzleBank::PASS_THROUGH: return "PASS_THROUGH";
        case NozzleBank::BANK_A_PET:   return "BANK_A_PET";
        case NozzleBank::BANK_B_HDPE:  return "BANK_B_HDPE";
        case NozzleBank::BANK_C_PP:    return "BANK_C_PP";
        case NozzleBank::BANK_D_MLP:   return "BANK_D_MLP";
    }
    return "UNKNOWN_BANK";
}

/**
 * @brief Deterministic real-time sorting controller for optical-pneumatic dry waste segregation.
 *
 * Models the core pipeline used in automated Material Recovery Facilities (MRFs):
 *  1. Ingests bounding-box detections from an AI computer vision camera node.
 *  2. Filters detections against a configurable confidence threshold.
 *  3. Calculates exact time-of-flight delay (ms) from the camera scan line to the
 *     pneumatic nozzle bar using rotary encoder belt speed (m/s).
 *  4. Computes the lateral nozzle index (0..31) and air-pulse duration (ms) from object mass.
 *  5. Enforces hardware safety interlocks (minimum pneumatic manifold pressure & belt stall protection).
 */
class SortingController {
public:
    SortingController(double camera_to_nozzle_distance_mm = 850.0,
                      double conveyor_width_mm = 1200.0,
                      int num_nozzles = 32,
                      double min_confidence = 0.78,
                      double min_pressure_bar = 4.5)
        : camera_to_nozzle_mm_(camera_to_nozzle_distance_mm),
          conveyor_width_mm_(conveyor_width_mm),
          num_nozzles_(num_nozzles),
          min_confidence_(min_confidence),
          min_pressure_bar_(min_pressure_bar),
          belt_speed_mps_(2.5),
          manifold_pressure_bar_(6.2),
          emergency_stop_(false) {
        if (camera_to_nozzle_mm_ <= 0.0 || conveyor_width_mm_ <= 0.0 || num_nozzles_ <= 0) {
            throw std::invalid_argument("Invalid physical geometry parameters for SortingController.");
        }
    }

    void setBeltSpeedMps(double speed_mps) {
        std::lock_guard<std::mutex> lock(mutex_);
        belt_speed_mps_ = speed_mps;
    }

    void setManifoldPressureBar(double pressure_bar) {
        std::lock_guard<std::mutex> lock(mutex_);
        manifold_pressure_bar_ = pressure_bar;
    }

    void setEmergencyStop(bool estop) {
        std::lock_guard<std::mutex> lock(mutex_);
        emergency_stop_ = estop;
    }

    /**
     * @brief Map polymer type to target pneumatic valve bank.
     */
    static NozzleBank mapPolymerToBank(PolymerType polymer) {
        switch (polymer) {
            case PolymerType::PET_CLEAR:
            case PolymerType::PET_COLORED:
                return NozzleBank::BANK_A_PET;
            case PolymerType::HDPE:
                return NozzleBank::BANK_B_HDPE;
            case PolymerType::PP:
            case PolymerType::BLACK_PLASTIC:
                return NozzleBank::BANK_C_PP;
            case PolymerType::MLP:
                return NozzleBank::BANK_D_MLP;
            case PolymerType::CONTAMINANT:
            default:
                return NozzleBank::PASS_THROUGH;
        }
    }

    /**
     * @brief Process a single AI vision detection and compute the pneumatic valve command.
     */
    ValveActuationCommand processDetection(const VisionDetection& det) {
        std::lock_guard<std::mutex> lock(mutex_);
        telemetry_.total_inspected++;
        telemetry_.current_belt_speed_mps = belt_speed_mps_;
        telemetry_.air_manifold_pressure_bar = manifold_pressure_bar_;

        // 1. Hardware & Safety Interlock validation
        if (emergency_stop_ || belt_speed_mps_ <= 0.1 || manifold_pressure_bar_ < min_pressure_bar_) {
            telemetry_.safety_interlock_aborts++;
            return {det.item_id, NozzleBank::PASS_THROUGH, -1, 0, 0, false};
        }

        // 2. Confidence threshold gate
        if (det.confidence < min_confidence_) {
            telemetry_.rejected_low_confidence++;
            return {det.item_id, NozzleBank::PASS_THROUGH, -1, 0, 0, false};
        }

        // 3. Determine target pneumatic bank
        NozzleBank bank = mapPolymerToBank(det.polymer);
        if (bank == NozzleBank::PASS_THROUGH) {
            telemetry_.counts_by_polymer[det.polymer]++;
            return {det.item_id, NozzleBank::PASS_THROUGH, -1, 0, 0, false};
        }

        // 4. Compute lateral nozzle index [0 .. num_nozzles_ - 1]
        double clamped_x = std::clamp(det.bbox_center_x_mm, 0.0, conveyor_width_mm_ - 0.001);
        int nozzle_idx = static_cast<int>((clamped_x / conveyor_width_mm_) * num_nozzles_);
        nozzle_idx = std::clamp(nozzle_idx, 0, num_nozzles_ - 1);

        // 5. Compute time-of-flight delay (ms): distance (mm) / speed (mm/ms == m/s)
        // Because 1 m/s == 1 mm/ms, delay_ms = distance_mm / belt_speed_mps
        uint64_t delay_ms = static_cast<uint64_t>(std::llround(camera_to_nozzle_mm_ / belt_speed_mps_));

        // 6. Compute adaptive air-pulse duration (12 ms base + mass compensation, clamped 10..45 ms)
        double raw_pulse = 12.0 + 0.35 * std::max(0.0, det.estimated_mass_g);
        uint32_t pulse_ms = static_cast<uint32_t>(std::clamp(std::llround(raw_pulse), 10LL, 45LL));

        // 7. Update telemetry counters
        telemetry_.total_sorted++;
        telemetry_.total_recovered_mass_g += std::max(0.0, det.estimated_mass_g);
        telemetry_.counts_by_polymer[det.polymer]++;

        return {det.item_id, bank, nozzle_idx, delay_ms, pulse_ms, true};
    }

    TelemetrySnapshot getTelemetrySnapshot() const {
        std::lock_guard<std::mutex> lock(mutex_);
        TelemetrySnapshot snap = telemetry_;
        snap.current_belt_speed_mps = belt_speed_mps_;
        snap.air_manifold_pressure_bar = manifold_pressure_bar_;
        return snap;
    }

    void resetTelemetry() {
        std::lock_guard<std::mutex> lock(mutex_);
        telemetry_ = TelemetrySnapshot{};
    }

    /**
     * @brief Export telemetry snapshot as a single CSV line for logging/analytics.
     */
    std::string exportTelemetryCsvRow() const {
        TelemetrySnapshot snap = getTelemetrySnapshot();
        std::ostringstream oss;
        double purity_pct = (snap.total_inspected > 0)
            ? (100.0 * static_cast<double>(snap.total_sorted) / static_cast<double>(snap.total_inspected))
            : 0.0;
        oss << snap.total_inspected << ","
            << snap.total_sorted << ","
            << snap.rejected_low_confidence << ","
            << snap.safety_interlock_aborts << ","
            << std::fixed << std::setprecision(2)
            << snap.total_recovered_mass_g << ","
            << snap.current_belt_speed_mps << ","
            << snap.air_manifold_pressure_bar << ","
            << purity_pct;
        return oss.str();
    }

private:
    double camera_to_nozzle_mm_;
    double conveyor_width_mm_;
    int num_nozzles_;
    double min_confidence_;
    double min_pressure_bar_;

    double belt_speed_mps_;
    double manifold_pressure_bar_;
    bool emergency_stop_;

    TelemetrySnapshot telemetry_;
    mutable std::mutex mutex_;
};

} // namespace waste_sorter

#endif // ISHITVA_SORTING_ENGINE_HPP
