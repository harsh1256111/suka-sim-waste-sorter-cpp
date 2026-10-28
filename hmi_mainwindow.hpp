#ifndef ISHITVA_HMI_MAINWINDOW_HPP
#define ISHITVA_HMI_MAINWINDOW_HPP

#include <QMainWindow>
#include <QLabel>
#include <QPushButton>
#include <QDoubleSpinBox>
#include <QTableWidget>
#include <QTimer>
#include "sorting_engine.hpp"

/**
 * @brief Qt Widgets Industrial HMI Dashboard for real-time conveyor monitoring.
 *
 * Connects the C++17 SortingController backend to a live operator control panel
 * using Qt Signals & Slots, a 100ms QTimer telemetry loop, and emergency-stop controls.
 */
class HmiMainWindow : public QMainWindow {
    Q_OBJECT

public:
    explicit HmiMainWindow(QWidget* parent = nullptr);
    ~HmiMainWindow() override = default;

signals:
    void valveActuatedSignal(quint64 itemId, const QString& polymer, int nozzleIdx, quint64 delayMs);
    void safetyInterlockTriggered(const QString& reason);

private slots:
    void onSimulationTick();
    void onBeltSpeedChanged(double speedMps);
    void onEmergencyStopToggled();

private:
    waste_sorter::SortingController controller_;
    QTimer* sim_timer_;
    uint64_t next_item_id_ = 2000;
    bool estop_active_ = false;

    QLabel* lbl_total_inspected_;
    QLabel* lbl_total_sorted_;
    QLabel* lbl_recovered_mass_;
    QLabel* lbl_interlock_status_;
    QDoubleSpinBox* spin_belt_speed_;
    QPushButton* btn_estop_;
    QTableWidget* tbl_event_log_;
};

#endif // ISHITVA_HMI_MAINWINDOW_HPP
