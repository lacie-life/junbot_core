#include <ros/ros.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/LaserScan.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <junbot_msgs/SensorState.h>
#include <junbot_msgs/VersionInfo.h>
#include <std_msgs/Float32.h>
#include <string>
#include <nlohmann/json.hpp>
#include <std_msgs/String.h>


ros::Publisher jun_diagnostics_pub;

diagnostic_msgs::DiagnosticStatus camera_state;
diagnostic_msgs::DiagnosticStatus teensy_state;
diagnostic_msgs::DiagnosticStatus lidar_state;

bool isTeensyConnected = false;
bool isLidarConnected = false;
bool isCamreraConnected = false;
bool isT265Connected = false;
bool isD455Connected = false;

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

void T265StateCallback(const std_msgs::String::ConstPtr &msg)
{
    using json = nlohmann::json;
    std::string tmp = msg->data.c_str();
    
    json parsedJson = json::parse(tmp);
    
    if(parsedJson["t265_state"] == "1")
    {
        isT265Connected = true;
    }
    else {
        isT265Connected = false;
    }
}

void D455StateCallback(const std_msgs::String::ConstPtr &msg)
{

    using json = nlohmann::json;
    std::string tmp = msg->data.c_str();
    
    json parsedJson = json::parse(tmp);
    
    if(parsedJson["d455_state"] != 0)
    {
        isD455Connected = true;
    }
    else {
        isD455Connected = false;
    }
}

void checkCameraConnection()
{

  isCamreraConnected = isT265Connected && isD455Connected;

  if(isCamreraConnected)
  {
    setCameraDiagnosis(diagnostic_msgs::DiagnosticStatus::OK, "Good Condition");
  }
  else if(!isCamreraConnected)
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

  ros::Subscriber lidar         = nh.subscribe("scan", 10, LidarMsgCallback);
  ros::Subscriber teensy  = nh.subscribe("/cmd_vol_fb", 10, sensorStateMsgCallback);
  ros::Subscriber t265_camera  = nh.subscribe("/t265_state", 10, T265StateCallback);
  ros::Subscriber d455_camera  = nh.subscribe("/d455_state", 10, D455StateCallback);

  ros::Rate loop_rate(1);

  while (ros::ok())
  {
    if(!isCamreraConnected)
    {
        checkCameraConnection();
    }
    msgPub();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}