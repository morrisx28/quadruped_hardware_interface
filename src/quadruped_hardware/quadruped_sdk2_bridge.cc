#include "quadruped_sdk2_bridge.h"

QuadrupedSdk2Bridge::QuadrupedSdk2Bridge(const MotorConfig &config)
{
    can_id_list = config.can_id_list;
    mst_id_list = config.mst_id_list;
    motor_type = config.motor_type;
    motor_offset = config.motor_offset;
    direction = config.direction;
    pos_limit_ = config.pos_limit;
    num_motor_ = can_id_list.size();

    for (int i=0; i < num_motor_; i++)
    {
        dm_data_list.push_back(damiao::DmActData{.motorType = static_cast<damiao::DM_Motor_Type>(motor_type[i]),
        .mode = damiao::MIT_MODE,
        .can_id=static_cast<uint16_t>(can_id_list[i]),
        .mst_id=static_cast<uint16_t>(mst_id_list[i])});
    }
    motor_control = std::make_shared<damiao::Motor_Control>(nom_baud,dat_baud,
      config.dev_sn,&dm_data_list);
    xsens_imu_data = std::make_shared<ImuSharedData>();

    if (config.set_zero)
    {
        SetMotorToZero();
    }

    last_fault_recovery_time_.resize(num_motor_, std::chrono::steady_clock::now());
    last_disconnect_log_time_.resize(num_motor_, std::chrono::steady_clock::now());

    low_cmd_go_suber_.reset(new ChannelSubscriber<unitree_go::msg::dds_::LowCmd_>(TOPIC_LOWCMD));
    low_cmd_go_suber_->InitChannel(bind(&QuadrupedSdk2Bridge::LowCmdGoHandler, this, placeholders::_1), 1);

    low_state_go_puber_.reset(new ChannelPublisher<unitree_go::msg::dds_::LowState_>(TOPIC_LOWSTATE));
    low_state_go_puber_->InitChannel();
    
    if (have_imu_) {
        InitXsensIMU();
        xsens_imu_thread = std::thread(&QuadrupedSdk2Bridge::ProcessXsensData, this);
    }

    init_time_ = std::chrono::steady_clock::now();
    last_poll_time_ = init_time_;
    lowStatePuberThreadPtr = CreateRecurrentThreadEx("lowstate", UT_CPU_ID_NONE, 2000, &QuadrupedSdk2Bridge::PublishLowStateGo, this);

}

QuadrupedSdk2Bridge::~QuadrupedSdk2Bridge()
{
    is_running = false;

    if (low_cmd_go_suber_) {
        low_cmd_go_suber_->CloseChannel();
    }

    if (lowStatePuberThreadPtr) {
        std::static_pointer_cast<RecurrentThread>(lowStatePuberThreadPtr)->Wait(1000000);
        lowStatePuberThreadPtr.reset();
    }

    if (have_imu_) {
        xsens_imu_thread.join();
        CloseXsensIMU();
    }

}

void QuadrupedSdk2Bridge::SetMotorToZero()
{
    for (int i = 0; i < num_motor_; i++) 
    {
        motor_control->set_zero_position(*motor_control->getMotor(can_id_list[i]));
    }
}

void QuadrupedSdk2Bridge::LowCmdGoHandler(const void *msg)
{
    if (!is_running) return;
    const unitree_go::msg::dds_::LowCmd_ *cmd = (const unitree_go::msg::dds_::LowCmd_ *)msg;
    for (int i = 0; i < num_motor_; i++)
    {
        auto motor = motor_control->getMotor(can_id_list[i]);
        uint8_t err = motor->Get_ERR();
        if (err > 1) continue;

        double raw_cmd_q = cmd->motor_cmd()[i].q();
        double cur_pos = (motor->Get_Position() - motor_offset[i]) * direction[i];

        // Only snap the command to the limit once both the current joint pos and
        // the incoming cmd have reached the same side of the constraint, so cmds
        // that move away from the limit are still passed through unclamped.
        double joint_cmd_q = raw_cmd_q;
        if (cur_pos >= pos_limit_[i].upper && raw_cmd_q >= pos_limit_[i].upper)
        {
            joint_cmd_q = pos_limit_[i].upper;
        }
        else if (cur_pos <= pos_limit_[i].lower && raw_cmd_q <= pos_limit_[i].lower)
        {
            joint_cmd_q = pos_limit_[i].lower;
        }

        double cmd_q = joint_cmd_q * direction[i] + motor_offset[i];
        double cmd_dq = cmd->motor_cmd()[i].dq() * direction[i];
        double cmd_tau = cmd->motor_cmd()[i].tau() * direction[i];

        motor_control->control_mit(*motor, cmd->motor_cmd()[i].kp(), cmd->motor_cmd()[i].kd(),
                                                            cmd_q, cmd_dq, cmd_tau);
    }
}

void QuadrupedSdk2Bridge::PublishLowStateGo()
{
    if (!is_running) return;

    auto poll_now = std::chrono::steady_clock::now();
    if (std::chrono::duration<double>(poll_now - last_poll_time_).count() >= poll_interval_sec_)
    {
        last_poll_time_ = poll_now;
        for (int i = 0; i < num_motor_; i++)
        {
            auto motor = motor_control->getMotor(can_id_list[i]);
            if (motor->GetTimeSinceLastFeedback() >= poll_interval_sec_)
            {
                motor_control->refresh_motor_status(*motor);
            }
        }
    }

    for (uint16_t i = 0;i < num_motor_; i++)
    {
        auto motor = motor_control->getMotor(can_id_list[i]);
        double pos = motor->Get_Position();
        double vel = motor->Get_Velocity();
        double tau = motor->Get_tau();
        uint8_t err = motor->Get_ERR();

        low_state_go_.motor_state()[i].q() = (pos - motor_offset[i]) * direction[i];
        low_state_go_.motor_state()[i].dq() = vel * direction[i];
        low_state_go_.motor_state()[i].tau_est() = tau * direction[i];
        low_state_go_.motor_state()[i].mode() = err;
        low_state_go_.motor_state()[i].temperature() = static_cast<uint8_t>(motor->Get_T_MOS());

        if (!IsMotorConnected(i)) {
            auto now = std::chrono::steady_clock::now();
            double since_init = std::chrono::duration<double>(now - init_time_).count();
            double log_elapsed = std::chrono::duration<double>(now - last_disconnect_log_time_[i]).count();
            // Stay quiet during the startup grace window so motors have time to
            // report in before we declare them disconnected.
            if (since_init >= init_grace_sec_ && log_elapsed >= 2.0) {
                last_disconnect_log_time_[i] = now;
                std::cerr << "[Motor " << i << " CAN_ID=0x" << std::hex << can_id_list[i]
                          << std::dec << "] not connected (no feedback received)" << std::endl;
            }
        } else if (err > 1) {
            HandleMotorFault(i);
        }
    }
    
    if (have_imu_)
    {
        low_state_go_.imu_state().quaternion()[0] = xsens_imu_data->quaternion[0];
        low_state_go_.imu_state().quaternion()[1] = xsens_imu_data->quaternion[1];
        low_state_go_.imu_state().quaternion()[2] = xsens_imu_data->quaternion[2];
        low_state_go_.imu_state().quaternion()[3] = xsens_imu_data->quaternion[3];

        low_state_go_.imu_state().gyroscope()[0] = xsens_imu_data->gyro[0];
        low_state_go_.imu_state().gyroscope()[1] = xsens_imu_data->gyro[1];
        low_state_go_.imu_state().gyroscope()[2] = xsens_imu_data->gyro[2];

        low_state_go_.imu_state().accelerometer()[0] = xsens_imu_data->accel[0];
        low_state_go_.imu_state().accelerometer()[1] = xsens_imu_data->accel[1];
        low_state_go_.imu_state().accelerometer()[2] = xsens_imu_data->accel[2];
    }
    low_state_go_puber_->Write(low_state_go_);

}

void QuadrupedSdk2Bridge::InitXsensIMU()
{
    xsens_control = XsControl::construct();
    XsPortInfoArray xsens_portInfoArray = XsScanner::scanPorts();
    for (auto const &portInfo : xsens_portInfoArray)
    {
        if (portInfo.deviceId().isMti() || portInfo.deviceId().isMtig())
        {
            xsens_mtPort = portInfo;
            break;
        }
    }
    if (xsens_mtPort.empty()) {
        std::cerr << "No MTi device found. IMU thread exiting." << std::endl;
        return;
    }
    if (!xsens_control->openPort(xsens_mtPort.portName().toStdString(), xsens_mtPort.baudrate())) {
        std::cerr << "Could not open IMU port. IMU thread exiting." << std::endl;
        return;
    }
    // Get the device object
    cout << "Found a device with ID: " << xsens_mtPort.deviceId().toString().toStdString() << " @ port: " << xsens_mtPort.portName().toStdString() << ", baudrate: " << xsens_mtPort.baudrate() << endl;
	xsens_device = xsens_control->device(xsens_mtPort.deviceId());
	assert(xsens_device != 0);

	cout << "Device: " << xsens_device->productCode().toStdString() << ", with ID: " << xsens_device->deviceId().toString() << " opened." << endl;

    xsens_device->addCallbackHandler(&xsens_callback);
    if (!xsens_device->gotoConfig()) return;
    xsens_device->readEmtsAndDeviceConfiguration();
    XsOutputConfigurationArray configArray;
    configArray.push_back(XsOutputConfiguration(XDI_PacketCounter, 0));
	configArray.push_back(XsOutputConfiguration(XDI_SampleTimeFine, 0));
    if (xsens_device->deviceId().isVru() || xsens_device->deviceId().isAhrs())
	{
		configArray.push_back(XsOutputConfiguration(XDI_Quaternion, 100));
        configArray.push_back(XsOutputConfiguration(XDI_RateOfTurnHR, 1000));
        configArray.push_back(XsOutputConfiguration(XDI_AccelerationHR, 1000));
	}
    if (!xsens_device->setOutputConfiguration(configArray)) return;
    if (!xsens_device->gotoMeasurement()) return;
}

void QuadrupedSdk2Bridge::ProcessXsensData()
{
    
    while (is_running) {
        if (xsens_callback.packetAvailable()) {         
            XsDataPacket packet = xsens_callback.getNextPacket();

            if (packet.containsOrientation()) {
                XsQuaternion q = packet.orientationQuaternion();
                xsens_imu_data->quaternion[0] = q.w();
                xsens_imu_data->quaternion[1] = q.x();
                xsens_imu_data->quaternion[2] = q.y();
                xsens_imu_data->quaternion[3] = q.z();
                XsEuler euler = packet.orientationEuler();
                xsens_imu_data->rpy[0] = euler.roll();
                xsens_imu_data->rpy[1] = euler.pitch();
                xsens_imu_data->rpy[2] = euler.yaw();
            }
            if (packet.containsRateOfTurnHR()) {
                XsVector gyr_hr = packet.rateOfTurnHR();
                for (int i = 0; i < 3; ++i) {
                    xsens_imu_data->gyro[i] = gyr_hr[i];
                }
            }
            if (packet.containsAccelerationHR()) {
                XsVector acc_hr = packet.accelerationHR();
                for (int i = 0; i < 3; ++i) {
                    xsens_imu_data->accel[i] = acc_hr[i];
                }
            }
        }
        XsTime::msleep(1);
    }
}

bool QuadrupedSdk2Bridge::IsMotorConnected(int motor_idx) const
{
    auto motor = motor_control->getMotor(can_id_list[motor_idx]);
    return motor->GetTimeSinceLastFeedback() < feedback_timeout_sec_;
}

void QuadrupedSdk2Bridge::HandleMotorFault(int i)
{
    auto now = std::chrono::steady_clock::now();
    double elapsed = std::chrono::duration<double>(now - last_fault_recovery_time_[i]).count();
    if (elapsed < 2.0) {
        return;
    }
    last_fault_recovery_time_[i] = now;

    uint8_t err = motor_control->getMotor(can_id_list[i])->Get_ERR();
    std::cerr << "[Motor " << i << " CAN_ID=0x" << std::hex << can_id_list[i]
              << "] fault ERR=0x" << static_cast<int>(err) << std::dec
              << " - sending clear+enable" << std::endl;

    motor_control->control_cmd(can_id_list[i], 0xFB);
    usleep(5000);
    motor_control->control_cmd(can_id_list[i], 0xFC);
}

void QuadrupedSdk2Bridge::CloseXsensIMU()
{
    if (xsens_control == nullptr) return;
    if (!xsens_mtPort.empty()) {
        xsens_control->closePort(xsens_mtPort.portName().toStdString());
    }
    xsens_control->destruct();
    xsens_control = nullptr;
}

