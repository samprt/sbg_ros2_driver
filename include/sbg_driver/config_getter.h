/*!
*	\file         config_getter.h
*	\author       Samuel Prouten
*	\date         25/01/2023
*
*	\brief        Get configuration from the device.
*
*   Class that gets the configuration from the device and displays or saves it to a file.
*
*	\section CodeCopyright Copyright Notice
*	MIT License
*
*	Copyright (c) 2020 SBG Systems
*
*	Permission is hereby granted, free of charge, to any person obtaining a copy
*	of this software and associated documentation files (the "Software"), to deal
*	in the Software without restriction, including without limitation the rights
*	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
*	copies of the Software, and to permit persons to whom the Software is
*	furnished to do so, subject to the following conditions:
*
*	The above copyright notice and this permission notice shall be included in all
*	copies or substantial portions of the Software.
*
*	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
*	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
*	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
*	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
*	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
*	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
*	SOFTWARE.
*/


#ifndef CONFIG_GETTER_H
#define CONFIG_GETTER_H

// Standard headers
#include <limits>
#include <string>

// Project headers
#include <config_store.h>

namespace sbg
{
    struct SbgDeviceConfiguration{
        SbgEComInitConditionConf initConditionConf;
        SbgEComModelInfo motionProfile;
        SbgEComSensorAlignmentInfo sensorAlignmentInfo;
        SbgVector3<float> imu_level_arms;
        SbgEComAidingAssignConf aidingAssignConf;
        SbgEComModelInfo magModelInfo;
        SbgEComMagRejectionConf magRejectionMode;
        SbgEComModelInfo gnssModelInfo;
        SbgEComGnssInstallation gnssInstallation;
        SbgEComGnssRejectionConf gnssRejectionMode;
        SbgEComOdoConf odometerConf;
        SbgVector3<float> odometerLevelArms;
        SbgEComOdoRejectionConf odometerRejectionMode;
        SbgEComOutputMode outputMode;
    };
/*!
 * Class to apply configuration to a device.
 */
class ConfigGetter
{
private:

  SbgEComHandle&         m_ref_sbg_com_handle;
  SbgDeviceConfiguration m_ref_device_configuration;

  //---------------------------------------------------------------------//
  //- Private  methods                                                  -//
  //---------------------------------------------------------------------//

  /*!
   * Check if the configuration getter worked properly.
   * This function will log a warning information if the parameter is not available for this device.
   *
   * \param[in] ref_sbg_error_code          Error code from the configuration getter.
   * \param[in] ref_conf_title              String to identify the configuration.
   * \throw                                 Unable to get the configuration.
   */
  void checkConfigurationGet(const SbgErrorCode& ref_sbg_error_code, const std::string& ref_conf_title) const;

  /*!
   * Get the initial condition parameters.
   *
   * \param[in] ref_init_condition          Initial condition conf to apply.
   */
  void getInitCondition(SbgEComInitConditionConf& ref_init_condition);

  /*!
   * Get the motion profile.
   *
   * \param[in] ref_motion_profile          Motion profile configuration to apply.
   */
  void getMotionProfile( SbgEComModelInfo& ref_motion_profile);

  /*!
   * Get the IMU alignement.
   *
   * \param[in] ref_sensor_align            Sensor IMU alignement configuration to apply.
   * \param[in] ref_level_arms              X, Y, Z level arms to apply.
   */
  void getImuAlignement( SbgEComSensorAlignmentInfo& ref_sensor_align,  SbgVector3<float>& ref_level_arms);

  /*!
   * Get the aiding assignement.
   *
   * \param[in] ref_aiding_assign           Aiding assignement configuration to apply.
   */
  void getAidingAssignement( SbgEComAidingAssignConf& ref_aiding_assign);

  /*!
   * Get the magnetometers model.
   *
   * \param[in] ref_mag_model               Magnetometers model configuration to apply.
   */
  void getMagModel( SbgEComModelInfo& ref_mag_model);

  /*!
   * Get the magnetometers rejection.
   *
   * \param[in] ref_mag_rejection           Magnetometers rejection configuration to apply.
   */
  void getMagRejection( SbgEComMagRejectionConf& ref_mag_rejection);

  /*!
   * Get the Gnss model.
   *
   * \param[in] ref_gnss_model              Gnss model configuration to apply.
   */
  void getGnssModel( SbgEComModelInfo& ref_gnss_model);

  /*!
   * Get the Gnss installation.
   *
   * \param[in] ref_gnss_installation       Gnss installation configuration to apply.
   */
  void getGnssInstallation( SbgEComGnssInstallation& ref_gnss_installation);

  /*!
   * Get the Gnss rejection.
   *
   * \param[in] ref_gnss_rejection          Gnss rejection configuration to apply.
   */
  void getGnssRejection( SbgEComGnssRejectionConf& ref_gnss_rejection);

  /*!
   * Get the odometer.
   *
   * \param[in] ref_odometer                Odometer configuration to apply.
   */
  void getOdometer( SbgEComOdoConf& ref_odometer);

  /*!
   * Get the odometer level arm.
   *
   * \param[in] odometer_level_arms         X,Y,Z odometer level arms to apply.
   */
  void getOdometerLevelArm( SbgVector3<float>& odometer_level_arms);

  /*!
   * Get the odometer rejection.
   *
   * \param[in] ref_odometer_rejection      Odometer rejection configuration to apply.
   */
  void getOdometerRejection( SbgEComOdoRejectionConf& ref_odometer_rejection);

  /*!
   * Get the output for the SBG log.
   * If a Log is not available for the connected device, a warning will be logged.
   * It will be user responsability to check.
   *
   * \param[in] output_port       Output communication port.
   * \param[in] ref_log_output    Log output to get.
   * \throw                       Unable to get the output.
   */
  void getOutput(SbgEComOutputPort output_port,  SbgEComOutputMode& ref_output_mode, ConfigStore::SbgLogOutput &ref_log_output);

public:

  //---------------------------------------------------------------------//
  //- Constructor                                                       -//
  //---------------------------------------------------------------------//

  /*!
   * Default constructor.
   *
   * \param[in] ref_com_handle                  SBG communication handle.
   * \param[in] ref_sbg_device_configuration    SBG device configuration.
   */
  ConfigGetter(SbgEComHandle &ref_sbg_com_handle, SbgDeviceConfiguration& ref_sbg_device_configuration);

  //---------------------------------------------------------------------//
  //- Parameters                                                        -//
  //---------------------------------------------------------------------//

  //---------------------------------------------------------------------//
  //- Operations                                                        -//
  //---------------------------------------------------------------------//

  /*!
   * Apply a configuration to the SBG device.
   */
  void getConfiguration();

  /*!
   * Save the device configuration to a file.
   */
  void saveConfiguration(void);
};
}

#endif // CONFIG_GETTER_H
