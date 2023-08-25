#include <ros/ros.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/LaserScan.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <junbot_msgs/SensorState.h>
#include <junbot_msgs/VersionInfo.h>
#include <string>
#include <librealsense2/rs.hpp>

ros::Publisher jun_diagnostics_pub;

diagnostic_msgs::DiagnosticStatus camera_state;
diagnostic_msgs::DiagnosticStatus teensy_state;
diagnostic_msgs::DiagnosticStatus lidar_state;

rs2::context ctx;

bool isTeensyConnected = false;
bool isLidarConnected = false;

std::string t265_serial_number = "908412110411";
std::string d455_1_serial_number = "117122250794";
std::string d455_2_serial_number = "117122250006";


void setDiagnosisMsg(diagnostic_msgs::DiagnosticStatus *diag, uint8_t level, std::string name, std::string message, std::string hardware_id)
{
  diag->level = level;
  diag->name  = name;
  diag->message = message;
  diag->hardware_id = hardware_id;
}

void setLidarDiagnosis(uint8_t level, std::string message)
{
  setDiagnosisMsg(&lidar_state, level, "Lidar Sensor", message, "Sick TiM 781");
}

void LidarMsgCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
  setLidarDiagnosis(diagnostic_msgs::DiagnosticStatus::OK, "Good Condition");
}

void setTeensyDiagnosis(uint8_t level, std::string message)
{
  setDiagnosisMsg(&teensy_state, level, "Teensy", message, "Teensy");
}

void setCameraDiagnosis(uint8_t level, std::string message)
{
  setDiagnosisMsg(&camera_state, level, "Camera", message, "Intel Realsense");
}

void sensorStateMsgCallback(const std_msgs::Float32::ConstPtr &msg)
{  
  if(msg->data != 0.0){
    setTeensyDiagnosis(diagnostic_msgs::DiagnosticStatus::OK, "Good Condition");
  }
  else if(msg->data == 0.0)
  {
    setTeensyDiagnosis(diagnostic_msgs::DiagnosticStatus::ERROR, "Bad Condition");
  }
}

void checkCameraConnection()
{
  rs2::device_list devices = ctx.query_devices();

  bool isD455_1_Connected = false;
  bool isD455_2_Connected = false;
  bool isT265Connected = false;

  for (rs2::device device : devices)
  {
    std::string serial_number = device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
    
    if (serial_number == t265_serial_number)
    {
      isT265Connected = true;
    }
    else if (serial_number == d455_1_serial_number)
    {
      isD455_1_Connected = true;
    }
    else if (serial_number == d455_2_serial_number)
    {
      isD455_2_Connected = true;
    }
  }

  bool isCameraConnected = isT265Connected && isD455_1_Connected && isD455_2_Connected;

  if(isCameraConnected)
  {
    setCameraDiagnosis(diagnostic_msgs::DiagnosticStatus::OK, "Good Condition");
  }
  else if(!isCameraConnected)
  {
    setCameraDiagnosis(diagnostic_msgs::DiagnosticStatus::ERROR, "Bad Condition");
  }
}
  
void msgPub()
{
  diagnostic_msgs::DiagnosticArray j_diagnostics;

  j_diagnostics.header.stamp = ros::Time::now();

  j_diagnostics.status.clear();
  j_diagnostics.status.push_back(camera_state);
  j_diagnostics.status.push_back(teensy_state);
  j_diagnostics.status.push_back(lidar_state);

  jun_diagnostics_pub.publish(j_diagnostics);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "junbot_diagnostic");
  ros::NodeHandle nh;

  jun_diagnostics_pub  = nh.advertise<diagnostic_msgs::DiagnosticArray>("/junbot_diagnostics", 10);
  jun_version_info_pub = nh.advertise<junbot_msgs::VersionInfo>("version_info", 10);

  ros::Subscriber lidar         = nh.subscribe("scan", 10, LidarMsgCallback);
  ros::Subscriber teensy  = nh.subscribe("/cmd_vol_fb", 10, sensorStateMsgCallback);

  ros::Rate loop_rate(1);

  while (ros::ok())
  {
    checkCameraConnection();

    msgPub();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
