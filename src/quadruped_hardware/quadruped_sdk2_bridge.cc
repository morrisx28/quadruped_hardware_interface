#include "quadruped_sdk2_bridge.h"

QuadrupedSdk2Bridge::QuadrupedSdk2Bridge(const char *dev_sn)
{
    for (int i=0; i < num_motor_; i++)
    {
        dm_data_list.push_back(damiao::DmActData{.motorType = static_cast<damiao::DM_Motor_Type>(motor_type[i]),
        .mode = damiao::MIT_MODE,
        .can_id=static_cast<uint16_t>(can_id_list[i]),
        .mst_id=static_cast<uint16_t>(mst_id_list[i])});
    }
    motor_control = std::make_shared<damiao::Motor_Control>(nom_baud,dat_baud,
      dev_sn,&dm_data_list);
    xsens_imu_data = std::make_shared<ImuSharedData>();

    if (set_zero_)
    {
        SetMotorToZero();
    }

    low_cmd_go_suber_.reset(new ChannelSubscriber<unitree_go::msg::dds_::LowCmd_>(TOPIC_LOWCMD));
    low_cmd_go_suber_->InitChannel(bind(&QuadrupedSdk2Bridge::LowCmdGoHandler, this, placeholders::_1), 1);

    low_state_go_puber_.reset(new ChannelPublisher<unitree_go::msg::dds_::LowState_>(TOPIC_LOWSTATE));
    low_state_go_puber_->InitChannel();
    
    if (have_imu_) {
        InitXsensIMU();
        xsens_imu_thread = std::thread(&QuadrupedSdk2Bridge::ProcessXsensData, this);
    }

    lowStatePuberThreadPtr = CreateRecurrentThreadEx("lowstate", UT_CPU_ID_NONE, 2000, &QuadrupedSdk2Bridge::PublishLowStateGo, this);

}

QuadrupedSdk2Bridge::~QuadrupedSdk2Bridge()
{
    is_running = false;
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
    const unitree_go::msg::dds_::LowCmd_ *cmd = (const unitree_go::msg::dds_::LowCmd_ *)msg;
    for (int i = 0; i < num_motor_; i++)
    {
        double cmd_q = cmd->motor_cmd()[i].q() * direction[i] + motor_offset[i];
        double cmd_dq = cmd->motor_cmd()[i].dq() * direction[i];
        double cmd_tau = cmd->motor_cmd()[i].tau() * direction[i];

        motor_control->control_mit(*motor_control->getMotor(can_id_list[i]), cmd->motor_cmd()[i].kp(), cmd->motor_cmd()[i].kd(), 
                                                            cmd_q, cmd_dq, cmd_tau);
    }
}

void QuadrupedSdk2Bridge::PublishLowStateGo()
{

    for (uint16_t i = 0;i < num_motor_; i++)
    {
        double pos = motor_control->getMotor(can_id_list[i])->Get_Position();
        double vel = motor_control->getMotor(can_id_list[i])->Get_Velocity();
        double tau = motor_control->getMotor(can_id_list[i])->Get_tau();
        
        low_state_go_.motor_state()[i].q() = (pos - motor_offset[i]) * direction[i];
        low_state_go_.motor_state()[i].dq() = vel * direction[i];
        low_state_go_.motor_state()[i].tau_est() = tau * direction[i];
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

void QuadrupedSdk2Bridge::CloseXsensIMU()
{
    xsens_control->closePort(xsens_mtPort.portName().toStdString());
    xsens_control->destruct();
}

