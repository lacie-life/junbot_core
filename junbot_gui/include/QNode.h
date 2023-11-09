#ifndef QNODE_H
#define QNODE_H

#ifndef Q_MOC_RUN

#include <ros/ros.h>

#endif

#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <image_transport/image_transport.h>
// #include <move_base_msgs/MoveBaseAction.h>
// #include <move_base_msgs/MoveBaseActionFeedback.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <tf/transform_listener.h>
#include <diagnostic_msgs/DiagnosticArray.h>

#include <QDebug>
#include <QImage>
#include <QLabel>
#include <QSettings>
#include <QStringListModel>
#include <QThread>
#include <QtConcurrent/QtConcurrent>
#include <map>
#include <string>

#include "QRobotItem.h"
#include "QRobotUltis.h"
#include "AppConstants.h"

// typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


class QNode : public QThread {
Q_OBJECT

public:
    QNode(int argc, char **argv);

    virtual ~QNode();

    bool init();

    bool init(const std::string &master_url, const std::string &host_url);

    void move_base(char k, float speed_linear, float speed_trun);

    void set_goal(QString frame, double x, double y, double z, double w);

    bool set_goal_once(QString frame, QRobotPose goal, int idx, int targetId);

    bool set_multi_goal(QString frame, std::vector<QRobotPose> goals, std::vector<int> targetId);

    void cancel_goal();

    QMap<QString, QString> get_topic_list();

    void run();

    QStringListModel *loggingModel();

    void log(const AppEnums::QLogLevel &level, const std::string &msg);

public slots:

    void sendNextTarget();

    void publishRobotStatus(QString state);

signals:

    void loggingUpdated();

    void rosShutdown();

    void updateRobotSpeed(QRobotSpeed speed);

    void Master_shutdown();

    void updateRoboPose(QRobotPose pos);

    void updateRobotStatus(AppEnums::QRobotStatus status);

    void updateBatteryVoltage(float voltage);

    void updateBatteryPercentage(float percentage);

    void updateGoalReached(int i);

    void updateAllGoalDone();

    void obstacleUpdate(QString id);

    void updateSensorStatus(int i);

private:
    int init_argc;
    char **init_argv;

    ros::Subscriber cmdVel_sub;
    ros::Subscriber chatter_subscriber;
    ros::Subscriber pos_sub;
    ros::Subscriber m_laserSub;
    ros::Subscriber battery_sub;
    ros::Subscriber m_batteryVoltageSub;
    ros::Subscriber m_batteryPercentageSub;
    ros::Subscriber m_plannerPathSub;
    ros::Subscriber m_compressedImgSub0;
    ros::Subscriber m_compressedImgSub1;
    ros::Subscriber m_robotDiagnosticsSub;
    ros::Subscriber m_obstaclesSub;
    ros::Subscriber m_targetReachedSub;

    ros::Publisher goal_pub;
    ros::Publisher cmd_pub;
    ros::Publisher m_initialposePub;
    ros::Publisher m_robotStatePub;
    ros::Publisher m_robotTargetIdPub;
    ros::Publisher m_MissionStart;
    image_transport::Publisher m_imageMapPub;

    // MoveBaseClient *movebase_client;
    QStringListModel logging_model;
    QString show_mode = "control";

    image_transport::Subscriber image_sub0;
    ros::Subscriber map_sub;
    QString video0_format;

    QString odom_topic;
    QString batteryState_topic;
    QString batteryVoltage_topic;
    QString batteryPercentage_topic;
    QString pose_topic;
    QString laser_topic;
    QString map_topic;
    QString initPose_topic;
    QString naviGoal_topic;
    QString robotDiagnostics_topic;
    QString robotState_topic;
    QString targetId_topic;
    QString obstacles_topic;
    QString mission_topic;
    QString targetReach_topic;

    std::string path_topic;
    QPolygon mapPonits;
    QPolygonF plannerPoints;
    QPolygonF laserPoints;

    int m_threadNum = 4;
    int m_frameRate = 40;

    // Map coordinate setting
    float m_mapOriginX;
    float m_mapOriginY;
    QPointF m_wordOrigin;
    float m_mapResolution;
    bool m_bMapIsInit = false;
    // tf::TransformListener m_tfListener(ros::Duration(10));
    // ros::Timer m_rosTimer;

    tf::TransformListener *m_robotPoselistener;
    tf::TransformListener *m_Laserlistener;
    std::string base_frame, laser_frame, map_frame;

    std::vector<QRobotPose> m_goals;
    std::vector<int> m_targetIds;
    int m_current_goals_id;
    QString m_goal_frame;

private:
    void speedCallback(const nav_msgs::Odometry::ConstPtr &msg);

    void SubAndPubTopic();

    void batteryVoltageCallback(const std_msgs::Float32 &message);

    void batteryPercentageCallback(const std_msgs::Float32 &message);

    void robotDiagnosticsCallback(const diagnostic_msgs::DiagnosticArray &message);

    void obstacleCallback(const std_msgs::String &message);

    void targetArrivedCallback(const std_msgs::String &message);
};

#endif // QNODE_H
