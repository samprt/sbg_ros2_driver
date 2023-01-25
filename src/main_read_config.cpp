#include <sbg_device.h>

using sbg::SbgDevice;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::Node node_handle("sbg_device");

  try
  {
    RCLCPP_INFO(node_handle.get_logger(), "SBG DRIVER - Init node, load params and connect to the device.");
    SbgDevice sbg_device(node_handle);

    RCLCPP_INFO(node_handle.get_logger(), "SBG DRIVER - Initialize device for receiving data");
    sbg_device.initDeviceForReadingConfig();

    return 0;
  }
  catch (std::exception const& refE)
  {
    RCLCPP_ERROR(node_handle.get_logger(), "SBG_DRIVER - %s", refE.what());
  }

  return 0;
}
