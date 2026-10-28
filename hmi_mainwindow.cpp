#include "hmi_mainwindow.hpp"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QGroupBox>
#include <QDateTime>

HmiMainWindow::HmiMainWindow(QWidget* parent)
    : QMainWindow(parent),
      controller_(850.0, 1200.0, 32, 0.78, 4.5) {
    setWindowTitle("SUKA-Sim Industrial Waste Sorting HMI (C++17 / Qt)");
    resize(920, 600);

    auto* central = new QWidget(this);
    auto* main_layout = new QVBoxLayout(central);

    // Top KPI Telemetry Bar
    auto* kpi_group = new QGroupBox("Real-Time Conveyor & Pneumatic Telemetry", central);
    auto* kpi_layout = new QHBoxLayout(kpi_group);
    lbl_total_inspected_ = new QLabel("Inspected: 0", kpi_group);
    lbl_total_sorted_ = new QLabel("Sorted: 0", kpi_group);
    lbl_recovered_mass_ = new QLabel("Recovered: 0.0 g", kpi_group);
    lbl_interlock_status_ = new QLabel("Status: RUNNING (6.2 bar)", kpi_group);

    kpi_layout->addWidget(lbl_total_inspected_);
    kpi_layout->addWidget(lbl_total_sorted_);
    kpi_layout->addWidget(lbl_recovered_mass_);
    kpi_layout->addWidget(lbl_interlock_status_);
    main_layout->addWidget(kpi_group);

    // Operator Hardware Controls
    auto* ctrl_group = new QGroupBox("PLC & Conveyor Controls", central);
    auto* ctrl_layout = new QHBoxLayout(ctrl_group);
    spin_belt_speed_ = new QDoubleSpinBox(ctrl_group);
    spin_belt_speed_->setRange(0.5, 4.5);
    spin_belt_speed_->setSingleStep(0.1);
    spin_belt_speed_->setValue(2.5);
    spin_belt_speed_->setSuffix(" m/s");

    btn_estop_ = new QPushButton("EMERGENCY STOP", ctrl_group);
    btn_estop_->setStyleSheet("background-color: #b91c1c; color: white; font-weight: bold; padding: 6px;");

    ctrl_layout->addWidget(new QLabel("Conveyor Speed:", ctrl_group));
    ctrl_layout->addWidget(spin_belt_speed_);
    ctrl_layout->addStretch();
    ctrl_layout->addWidget(btn_estop_);
    main_layout->addWidget(ctrl_group);

    // Event Log Table
    tbl_event_log_ = new QTableWidget(0, 5, central);
    tbl_event_log_->setHorizontalHeaderLabels({"Item ID", "Polymer", "Valve Bank", "Nozzle #", "TOF Delay (ms)"});
    tbl_event_log_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    main_layout->addWidget(tbl_event_log_);

    setCentralWidget(central);

    // Connect Signals & Slots
    sim_timer_ = new QTimer(this);
    connect(sim_timer_, &QTimer::timeout, this, &HmiMainWindow::onSimulationTick);
    connect(spin_belt_speed_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &HmiMainWindow::onBeltSpeedChanged);
    connect(btn_estop_, &QPushButton::clicked, this, &HmiMainWindow::onEmergencyStopToggled);

    sim_timer_->start(250);
}

void HmiMainWindow::onBeltSpeedChanged(double speedMps) {
    controller_.setBeltSpeedMps(speedMps);
}

void HmiMainWindow::onEmergencyStopToggled() {
    estop_active_ = !estop_active_;
    controller_.setEmergencyStop(estop_active_);
    lbl_interlock_status_->setText(estop_active_ ? "Status: E-STOP ENGAGED" : "Status: RUNNING (6.2 bar)");
}

void HmiMainWindow::onSimulationTick() {
    using namespace waste_sorter;
    static const PolymerType cycle[] = {
        PolymerType::PET_CLEAR, PolymerType::HDPE, PolymerType::BLACK_PLASTIC,
        PolymerType::PP, PolymerType::MLP
    };
    PolymerType poly = cycle[next_item_id_ % 5];
    double pos_mm = static_cast<double>((next_item_id_ * 137) % 1180);
    VisionDetection det{next_item_id_++, poly, 0.93, pos_mm, 4200.0, 26.5,
                        static_cast<uint64_t>(QDateTime::currentMSecsSinceEpoch())};

    auto cmd = controller_.processDetection(det);
    auto snap = controller_.getTelemetrySnapshot();

    lbl_total_inspected_->setText(QString("Inspected: %1").arg(snap.total_inspected));
    lbl_total_sorted_->setText(QString("Sorted: %1").arg(snap.total_sorted));
    lbl_recovered_mass_->setText(QString("Recovered: %1 g").arg(snap.total_recovered_mass_g, 0, 'f', 1));

    if (cmd.valid) {
        int row = tbl_event_log_->rowCount();
        tbl_event_log_->insertRow(row);
        tbl_event_log_->setItem(row, 0, new QTableWidgetItem(QString::number(cmd.item_id)));
        tbl_event_log_->setItem(row, 1, new QTableWidgetItem(QString::fromStdString(polymerToString(poly))));
        tbl_event_log_->setItem(row, 2, new QTableWidgetItem(QString::fromStdString(nozzleBankToString(cmd.bank))));
        tbl_event_log_->setItem(row, 3, new QTableWidgetItem(QString::number(cmd.nozzle_index)));
        tbl_event_log_->setItem(row, 4, new QTableWidgetItem(QString::number(cmd.trigger_delay_ms)));
        tbl_event_log_->scrollToBottom();
    }
}
