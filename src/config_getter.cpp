// File header
#include <config_getter.h>

using sbg::ConfigGetter;

/*!
 * Class to apply configuration to a device.
 */
//---------------------------------------------------------------------//
//- Constructor                                                       -//
//---------------------------------------------------------------------//

ConfigGetter::ConfigGetter(SbgEComHandle &ref_sbg_com_handle, SbgDeviceConfiguration &ref_device_configuration):
m_ref_sbg_com_handle(ref_sbg_com_handle),
m_ref_device_configuration(ref_device_configuration)
{

}

//---------------------------------------------------------------------//
//- Private  methods                                                  -//
//---------------------------------------------------------------------//

void ConfigGetter::checkConfigurationGet(const SbgErrorCode& ref_sbg_error_code, const std::string& ref_conf_title) const
{
    if (ref_sbg_error_code == SBG_INVALID_PARAMETER)
    {
        RCLCPP_WARN(rclcpp::get_logger("Config"), "SBG_DRIVER - [Config] Configuration %s is not available for the connected device.", ref_conf_title.c_str());
    }
    else if (ref_sbg_error_code != SBG_NO_ERROR)
    {
        std::string error_message("[Config] Unable to get the ");
        error_message.append(ref_conf_title);
        error_message.append(" configuration : ");
        error_message.append(sbgErrorCodeToString(ref_sbg_error_code));

        rclcpp::exceptions::throw_from_rcl_error(RMW_RET_ERROR, error_message);
    }
}

void ConfigGetter::getInitCondition(SbgEComInitConditionConf& ref_init_condition)
{
  //
  // Get the initial condition of the device, compare with the loaded parameters.
  // If the conditions are different, update the device configuration with the loaded parameters.
  //
  SbgErrorCode              error_code;

  error_code = sbgEComCmdSensorGetInitCondition(&m_ref_sbg_com_handle, &ref_init_condition);

  checkConfigurationGet(error_code, std::string("Init conditions"));
}

void ConfigGetter::getMotionProfile(SbgEComModelInfo& ref_motion_profile)
{
  //
  // Get the motion profile ID.
  //
  SbgErrorCode      error_code;

  error_code = sbgEComCmdSensorGetMotionProfileInfo(&m_ref_sbg_com_handle, &ref_motion_profile);

  checkConfigurationGet(error_code, std::string("Motion profile"));

}

void ConfigGetter::getImuAlignement( SbgEComSensorAlignmentInfo& ref_sensor_align, sbg::SbgVector3<float>& ref_level_arms)
{
  //
  // Get the IMU alignement and level arms, and compare with the parameters.
  // If the alignement are differents, update the device with the loaded parameters.
  //
  SbgErrorCode                error_code;
  float                       level_arms_device[3];

  error_code = sbgEComCmdSensorGetAlignmentAndLeverArm(&m_ref_sbg_com_handle, &ref_sensor_align, level_arms_device);
  ref_level_arms = sbg::SbgVector3<float>(level_arms_device[0], level_arms_device[1], level_arms_device[2]);

  checkConfigurationGet(error_code, std::string("IMU alignement"));

}

void ConfigGetter::getAidingAssignement( SbgEComAidingAssignConf& ref_aiding_assign)
{
  //
  // Get the aiding assignement, and compare with the loaded parameters.
  // If the assignement are differents, udpdate the device with the loaded parameters.
  //
  SbgErrorCode            error_code;

  error_code = sbgEComCmdSensorGetAidingAssignment(&m_ref_sbg_com_handle, &ref_aiding_assign);

  checkConfigurationGet(error_code, std::string("Aiding assignement"));
}

void ConfigGetter::getMagModel( SbgEComModelInfo& ref_mag_model)
{
  //
  // Get the magnetometer model, and compare with the loaded parameter.
  // If the model are different, update the device with the loaded parameter.
  //
  SbgErrorCode      error_code;

  error_code = sbgEComCmdMagGetModelInfo(&m_ref_sbg_com_handle, &ref_mag_model);

  checkConfigurationGet(error_code, std::string("Magnetometer model"));
}

void ConfigGetter::getMagRejection( SbgEComMagRejectionConf& ref_mag_rejection)
{
  //
  // Get the magnetometer rejection model, and compare with the loaded parameter.
  // If the model are different, update the device with the loaded parameter.
  //
  SbgErrorCode            error_code;

  error_code = sbgEComCmdMagGetRejection(&m_ref_sbg_com_handle, &ref_mag_rejection);

  checkConfigurationGet(error_code, std::string("Magnetometer rejection"));

}

void ConfigGetter::getGnssModel( SbgEComModelInfo& ref_gnss_model)
{
  //
  // Get the Gnss model, and compare with the loaded model.
  // If the models are different, update the device with the loaded model.
  //
  SbgErrorCode      error_code;

  error_code = sbgEComCmdGnss1GetModelInfo(&m_ref_sbg_com_handle, &ref_gnss_model);

  checkConfigurationGet(error_code, std::string("Gnss model"));

}

void ConfigGetter::getGnssInstallation( SbgEComGnssInstallation& ref_gnss_installation)
{
  //
  // Get the Gnss level arm, and compare with the loaded parameters.
  // If the level arms are different, update the device with the loaded parameters.
  //
  SbgErrorCode            error_code;

  error_code = sbgEComCmdGnss1InstallationGet(&m_ref_sbg_com_handle, &ref_gnss_installation);

  checkConfigurationGet(error_code, std::string("Gnss level arms"));
}

void ConfigGetter::getGnssRejection( SbgEComGnssRejectionConf& ref_gnss_rejection)
{
  //
  // Get the Gnss rejection, and compare with the loaded parameters.
  // If the rejection are different, update the device with the loaded parameters.
  //
  SbgErrorCode              error_code;

  error_code = sbgEComCmdGnss1GetRejection(&m_ref_sbg_com_handle, &ref_gnss_rejection);

  checkConfigurationGet(error_code, std::string("Gnss rejection"));
}

void ConfigGetter::getOdometer( SbgEComOdoConf& ref_odometer)
{
  //
  // Get the odometer configuration, and compare with the loaded parameters.
  // If the conf are different, update the device with the loaded parameters.
  //
  SbgErrorCode    error_code;

  error_code = sbgEComCmdOdoGetConf(&m_ref_sbg_com_handle, &ref_odometer);

  checkConfigurationGet(error_code, std::string("Odometer"));
}

void ConfigGetter::getOdometerLevelArm( SbgVector3<float>& odometer_level_arms)
{
  //
  // Get the odometer level arm, and compare with the loaded parameters.
  // If the level arms are different, update the device with the loaded parameters.
  //
  float         lever_arm[3];
  SbgErrorCode  error_code;

  error_code = sbgEComCmdOdoGetLeverArm(&m_ref_sbg_com_handle, lever_arm);

  checkConfigurationGet(error_code, std::string("Odometer level arms"));

  odometer_level_arms = SbgVector3<float>(lever_arm, 3);

}

void ConfigGetter::getOdometerRejection( SbgEComOdoRejectionConf& ref_odometer_rejection)
{
  //
  // Get the odometer rejection mode, and compare with the loaded parameter.
  // If the mode are different, update the device with the loaded parameter.
  //
  SbgErrorCode            error_code;

  error_code = sbgEComCmdOdoGetRejection(&m_ref_sbg_com_handle, &ref_odometer_rejection);

  checkConfigurationGet(error_code, std::string("Odometer rejection"));
}

void ConfigGetter::getOutput(SbgEComOutputPort output_port, SbgEComOutputMode& ref_output_mode, ConfigStore::SbgLogOutput &ref_log_output)
{
  SbgErrorCode      error_code;

  //
  // Get the current output mode for the device and the selected log ID.
  // If output modes are different, udpate the device mode with the one loaded from the parameters.
  //
  error_code = sbgEComCmdOutputGetConf(&m_ref_sbg_com_handle, output_port, ref_log_output.message_class, ref_log_output.message_id, &ref_output_mode);

  if (error_code == SBG_INVALID_PARAMETER)
  {
    RCLCPP_WARN(rclcpp::get_logger("Config"), "SBG_DRIVER - [Config] Output is not available for this device : Class [%d] - Id [%d]", ref_log_output.message_class, ref_log_output.message_id);
  }
  else if (error_code != SBG_NO_ERROR)
  {
    std::string error_message("[Config] Unable to get output for the device : Class [");
    error_message.append(std::to_string(ref_log_output.message_class));
    error_message.append("] - Id [");
    error_message.append(std::to_string(ref_log_output.message_id));
    error_message.append("] : ");
    error_message.append(sbgErrorCodeToString(error_code));

    rclcpp::exceptions::throw_from_rcl_error(RMW_RET_ERROR, error_message);
  }
}

//---------------------------------------------------------------------//
//- Parameters                                                        -//
//---------------------------------------------------------------------//

//---------------------------------------------------------------------//
//- Operations                                                        -//
//---------------------------------------------------------------------//

void ConfigGetter::getConfiguration()
{

  //
  // Get the connected device's configuration.
  //
  getInitCondition(m_ref_device_configuration.initConditionConf);
  getMotionProfile(m_ref_device_configuration.motionProfile);
  getImuAlignement(m_ref_device_configuration.sensorAlignmentInfo, m_ref_device_configuration.imu_level_arms);
  getAidingAssignement(m_ref_device_configuration.aidingAssignConf);
  getMagModel(m_ref_device_configuration.magModelInfo);
  getMagRejection(m_ref_device_configuration.magRejectionMode);
  getGnssModel(m_ref_device_configuration.gnssModelInfo);
  getGnssInstallation(m_ref_device_configuration.gnssInstallation);
  getGnssRejection(m_ref_device_configuration.gnssRejectionMode);
  getOdometer(m_ref_device_configuration.odometerConf);
  getOdometerLevelArm(m_ref_device_configuration.odometerLevelArms);
  getOdometerRejection(m_ref_device_configuration.odometerRejectionMode);

  // TODO Get output
  //
  // Get the output, with all output defined in the store.
  //
//  std::vector<ConfigStore::SbgLogOutput>& ref_output_modes = m_ref_device_configuration.outputMode;
//
//  for (const ConfigStore::SbgLogOutput& ref_output : m_ref_device_configuration.outputMode)
//  {
//    getOutput(m_ref_device_configuration.getOutputPort(), ref_output);
//  }
}

void ConfigGetter::saveConfiguration(void)
{
  getConfiguration();

}
