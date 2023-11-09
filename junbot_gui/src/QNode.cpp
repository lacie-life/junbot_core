#include "QNode.h"
#include <QDebug>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <QJsonDocument>
#include <QJsonObject>

QNode::QNode(int argc, char **argv)
        : init_argc(argc), init_argv(argv) {
    // Setting topic for ROS connection
    QSettings topic_setting("junbot_gui", "settings");
    odom_topic = topic_setting.value("topic/topic_odom", "odom").toString();
    batteryState_topic =
            topic_setting.value("topic/topic_power", "battery_state").toString();

    // Robot state
    batteryVoltage_topic = "cmd_vol_fb";
    batteryPercentage_topic = "cmd_bat_fb";
    robotDiagnostics_topic = "junbot_diagnostics";
    robotState_topic = "robot_status";
    targetId_topic = "robot_target_id";
    obstacles_topic = "object_detected";
    mission_topic = "mission_started";
    targetReach_topic = "goal_arrived";
    
    // initPose_topic =
    //         topic_setting.value("topic/topic_init_pose", "move_base_simple/goal")
    //                 .toString();
    naviGoal_topic =
            topic_setting.value("topic/topic_goal", "initialpose").toString();
    pose_topic = topic_setting.value("topic/topic_amcl", "amcl_pose").toString();
    m_frameRate = topic_setting.value("main/framerate", 40).toInt();
    m_threadNum = topic_setting.value("main/thread_num", 6).toInt();


    // UI display
    QSettings settings("junbot_gui", "Displays");
    map_topic = settings.value("Map/topic", QString("/map")).toString();
    laser_topic = settings.value("Laser/topic", QString("/scan")).toString();
    laser_frame =
            settings.value("frame/laserFrame", "/base_scan").toString().toStdString();
    map_frame = settings.value("frame/mapFrame", "/map").toString().toStdString();
    base_frame =
            settings.value("frame/baseFrame", "/base_link").toString().toStdString();
    path_topic =
            settings.value("GlobalPlan/topic", "/movebase").toString().toStdString();

    // Register MetaType for Qt signal and slot
    qRegisterMetaType<QRobotPose>("QRobotPose");
    qRegisterMetaType<AppEnums::QRobotStatus>("AppEnums::QRobotStatus");
    qRegisterMetaType<QVector<int>>("QVector<int>");
}

QNode::~QNode() {
    if (ros::isStarted()) {
        ros::shutdown();  // explicitly needed since we use ros::start();
        ros::waitForShutdown();
    }
    wait();
}

bool QNode::init() {
    ros::init(init_argc, init_argv, "junbot_gui",
              ros::init_options::AnonymousName);
    if (!ros::master::check()) {
        return false;
    }
    ros::start();  // explicitly needed since our nodehandle is going out of
    // scope.
    SubAndPubTopic();
    start();
    return true;
}

bool QNode::init(const std::string &master_url, const std::string &host_url) {
    std::map<std::string, std::string> remappings;

    CONSOLE << "ROS Node Init";

    remappings["__master"] = master_url;
    remappings["__hostname"] = host_url;
    ros::init(remappings, "junbot_gui", ros::init_options::AnonymousName);
    if (!ros::master::check()) {
        return false;
    }
    ros::start();  // explicitly needed since our nodehandle is going out of
    // scope.
    SubAndPubTopic();
    start();
    CONSOLE << "QNode Init done";
    return true;
}

void QNode::SubAndPubTopic() {
    ros::NodeHandle n;
    // Pub and Sub nesessary topic
    // Add your ros communications here.
    cmdVel_sub = n.subscribe<nav_msgs::Odometry>(odom_topic.toStdString(), 200,
                                                 &QNode::speedCallback, this);

    m_batteryVoltageSub = n.subscribe(batteryVoltage_topic.toStdString(), 1000,
                                      &QNode::batteryVoltageCallback, this);

    m_batteryPercentageSub = n.subscribe(batteryPercentage_topic.toStdString(), 1000,
                                         &QNode::batteryPercentageCallback, this);

    m_robotDiagnosticsSub = n.subscribe(robotDiagnostics_topic.toStdString(), 1000, 
                                         &QNode::robotDiagnosticsCallback, this);

    m_obstaclesSub = n.subscribe(obstacles_topic.toStdString(), 1000,
                                         &QNode::obstacleCallback, this);

    m_targetReachedSub = n.subscribe(targetReach_topic.toStdString(), 1000,
                                         &QNode::targetArrivedCallback, this);                                      

    goal_pub = n.advertise<move_base_msgs::MoveBaseActionGoal>(
            "/move_base/goal", 1000);

    m_MissionStart = n.advertise<std_msgs::String>(mission_topic.toStdString(), 10);

    cmd_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

    // m_initialposePub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>(
    //         initPose_topic.toStdString(), 10);

    image_transport::ImageTransport it(n);

    // m_imageMapPub = it.advertise("image/map", 10);

    m_robotStatePub = n.advertise<std_msgs::String>(robotState_topic.toStdString(), 10);

    m_robotTargetIdPub = n.advertise<std_msgs::String>(targetId_topic.toStdString(), 10);

    m_robotPoselistener = new tf::TransformListener;
    m_Laserlistener = new tf::TransformListener;
    try {
        m_robotPoselistener->waitForTransform(map_frame, base_frame, ros::Time(0),
                                              ros::Duration(0.4));
        m_Laserlistener->waitForTransform(map_frame, laser_frame, ros::Time(0),
                                          ros::Duration(0.4));
    } catch (tf::TransformException &ex) {
        log(AppEnums::QLogLevel::Err, ("laser and robot pose tf listener: " + QString(ex.what()))
                .toStdString());
    }
    // movebase_client = new MoveBaseClient("move_base", true);
    // movebase_client->waitForServer(ros::Duration(1.0));
}

QMap<QString, QString> QNode::get_topic_list() {
    ros::master::V_TopicInfo topic_list;
    ros::master::getTopics(topic_list);
    QMap<QString, QString> res;
    for (auto topic: topic_list) {
        res.insert(QString::fromStdString(topic.name),
                   QString::fromStdString(topic.datatype));
    }
    return res;
}


void QNode::publishRobotStatus(QString state)
{
    std_msgs::String msg;
    msg.data = state.toStdString();
    m_robotStatePub.publish(msg);
}

void QNode::batteryVoltageCallback(const std_msgs::Float32 &message) {
    float tmp = (float)message.data;
    // CONSOLE << tmp;
    emit updateBatteryVoltage(tmp);
}

void QNode::batteryPercentageCallback(const std_msgs::Float32 &message) {
    float tmp = (float)message.data;
    // CONSOLE << tmp;
    emit updateBatteryPercentage(tmp);
}

void QNode::robotDiagnosticsCallback(const diagnostic_msgs::DiagnosticArray &message_holder) {

    // CONSOLE << "robot state callback";
    
    // for (int i = 0; i < message_holder.status.size(); i++) {
    //     if (message_holder.status[i].name == "Teensy") {
    //         CONSOLE << "Teesy : " << message_holder.status[i].message.c_str();
    //     }
    //     if(message_holder.status[i].name == "Lidar") {
    //         CONSOLE << "Lidar : " << message_holder.status[i].message.c_str();
    //     }
    //     if(message_holder.status[i].name == "Camera") {
    //         CONSOLE << "Camera : " << message_holder.status[i].message.c_str();
    //     }
    // }
    // TODO: Update status to UI
    emit updateSensorStatus(1);
}

void QNode::obstacleCallback(const std_msgs::String &message_holder) {
    CONSOLE << "obstacles callback";
    CONSOLE << message_holder.data.c_str();

    // TODO: Add emit signal to GUI
    QJsonDocument tmp;
    tmp = QJsonDocument::fromJson(message_holder.data.c_str());

    QJsonObject jobj = tmp.object();

    QString id = jobj["id"].toString();

    emit obstacleUpdate(id);
}

void QNode::set_goal(QString frame, double x, double y, double z, double w) {
    geometry_msgs::PoseStamped goal;

    goal.header.frame_id = "map";

    goal.header.stamp = ros::Time::now();
    goal.pose.position.x = x;
    goal.pose.position.y = y;
    goal.pose.position.z = 0;
    goal.pose.orientation.z = z;
    goal.pose.orientation.w = 1.0;

    move_base_msgs::MoveBaseActionGoal tempGoal;
    tempGoal.goal.target_pose = goal;

    goal_pub.publish(tempGoal);
    ros::spinOnce();
}

// TODO: Update ArUcO pose and Id
bool QNode::set_goal_once(QString frame, QRobotPose goal, int idx, int target_id) {
    
    // while (!movebase_client->waitForServer(ros::Duration(5.0))) 
    // {
    //     ROS_INFO("Waiting for the move_base action server to come up");
    // }

    CONSOLE << "Index: " << idx;

    geometry_msgs::PoseStamped _goal;

    _goal.header.frame_id = "map";

    _goal.header.stamp = ros::Time::now();
    _goal.pose.position.x = goal.x;
    _goal.pose.position.y = goal.y;
    // CONSOLE << "Here ? ";
    _goal.pose.position.z = 0;
    _goal.pose.orientation.z = goal.theta;
    _goal.pose.orientation.w = 1.0;

    // CONSOLE << "Here ? ";

    // move_base_msgs::MoveBaseGoal tempGoal;
    // tempGoal.target_pose = _goal;

    // TODO: Publish goal marker information
    // std_msgs::Int32 tmp;
    // tmp.data = target_id;
    // m_robotTargetIdPub.publish(tmp);

    // { id: ,
    //   target_x: ,
    //   target_y: ,
    //   target_w: ,
    //   ref_x: ,
    //   ref_y: ,
    //   ref_w: ,
    //}

    QJsonObject jobj;
    
    jobj["id"] = target_id;
    jobj["target_x"] = goal.x;
    jobj["target_y"] = goal.y;
    jobj["target_w"] = goal.theta;
    jobj["ref_x"] = goal.x-0.5;
    jobj["ref_y"] = goal.y;
    jobj["ref_w"] = goal.theta;

    QString jString = QJsonDocument(jobj).toJson(QJsonDocument::Compact);

    std_msgs::String tmp_msg;
    tmp_msg.data = jString.toStdString();
    m_robotTargetIdPub.publish(tmp_msg);

    // movebase_client->cancelAllGoals();
    // movebase_client->sendGoal(tempGoal);

    // TODO: Crash here ...
    // movebase_client->waitForResult();

    move_base_msgs::MoveBaseActionGoal tempGoal;
    tempGoal.goal.target_pose = _goal;

    goal_pub.publish(tempGoal);

    CONSOLE << "Sent Goal " << target_id;
    ros::spinOnce();
    
    // if(movebase_client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    // {
    //     ROS_INFO("Goal %d reached", i);
    //     emit updateGoalReached(i);
    //     return true;
    // }
    // else
    // {
    //     return false;
    // }

    return true;
}

bool QNode::set_multi_goal(QString frame, std::vector<QRobotPose> goals, std::vector<int> target_id)
{
    // TODO: Update find optimal path ????
    m_goals = goals;
    m_targetIds = target_id;
    m_goal_frame = frame;
    m_current_goals_id = 0;

    CONSOLE << target_id.size();
    CONSOLE << goals.size();

    bool check = set_goal_once(m_goal_frame, 
                                m_goals[m_current_goals_id], 
                                m_current_goals_id, m_targetIds[m_current_goals_id]);

    // for (auto goal : goals)
    // {
    //   if(!set_goal_once(frame, goal, i))
    //   {
    //     return false;
    //   }
    //   else{
    //     ROS_INFO("Goal %d reached", i);
    //     emit updateGoalReached(i);
    //     i++;
    //   }
    // }
    return true;
}

void QNode::sendNextTarget()
{
    m_current_goals_id++;

    if(m_current_goals_id >= m_goals.size())
    {
        emit updateAllGoalDone();
        return;
    }
    else{
        bool check = set_goal_once(m_goal_frame, m_goals[m_current_goals_id], m_current_goals_id,  m_targetIds[m_current_goals_id]);
    }
}

void QNode::targetArrivedCallback(const std_msgs::String &message)
{
    CONSOLE << "target arrived callback";
    CONSOLE << message.data.c_str();

    QString tmp = QString::fromStdString(message.data.c_str());

    QJsonDocument json_tmp = QJsonDocument::fromJson(tmp.toUtf8());

    QString tmp_state = json_tmp["status"].toString();
    QString tmp_id = json_tmp["id"].toString();

    if(tmp_state == "done")
    {
        emit updateGoalReached(m_current_goals_id);
    }
    else
    {
        CONSOLE << "Target " << tmp_id << "failed"; 
    }  
}

void QNode::cancel_goal() {
    // movebase_client->cancelAllGoals();
}

void QNode::speedCallback(const nav_msgs::Odometry::ConstPtr &msg) {
    QRobotSpeed speed;
    speed.vx = msg->twist.twist.linear.x;
    speed.vy = msg->twist.twist.angular.y;

    emit updateRobotSpeed(speed);
}

void QNode::run() {
    ros::Rate loop_rate(m_frameRate);
    ros::AsyncSpinner spinner(m_threadNum);
    spinner.start();

    while (ros::ok()) {
        // updateRobotPose();
        emit updateRobotStatus(AppEnums::QRobotStatus::Ready);
        loop_rate.sleep();
    }

    Q_EMIT
    rosShutdown();  // used to signal the gui for a shutdown (useful to roslaunch)
}

void QNode::move_base(char k, float speed_linear, float speed_turn) {

    std::map<char, std::vector<float>> moveBindings{
            {'i', {1,  0,  0,  0}}, // forward
            {'o', {1,  0,  0,  -1}},
            {'j', {0,  0,  0,  1}}, // rotate left
            {'l', {0,  0,  0,  -1}}, // rotate right
            {'u', {1,  0,  0,  1}},
            {',', {-1, 0,  0,  0}}, // backward
            {'.', {-1, 0,  0,  1}},
            {'m', {-1, 0,  0,  -1}},
            {'O', {1,  -1, 0,  0}},
            {'I', {1,  0,  0,  0}},
            {'J', {0,  1,  0,  0}},
            {'L', {0,  -1, 0,  0}},
            {'U', {1,  1,  0,  0}},
            {'<', {-1, 0,  0,  0}},
            {'>', {-1, -1, 0,  0}},
            {'M', {-1, 1,  0,  0}},
            {'t', {0,  0,  1,  0}},
            {'b', {0,  0,  -1, 0}},
            {'k', {0,  0,  0,  0}}, // stop
            {'K', {0,  0,  0,  0}}};

    char key = k;

    if (k == 'k') {
        // movebase_client->cancelAllGoals();
    }

    float x = moveBindings[key][0];
    float y = moveBindings[key][1];
    float z = moveBindings[key][2];
    float th = moveBindings[key][3];

    float speed = speed_linear;
    float turn = speed_turn;
    // Update the Twist message
    geometry_msgs::Twist twist;
    twist.linear.x = x * speed;
    twist.linear.y = y * speed;
    twist.linear.z = z * speed;

    twist.angular.x = 0;
    twist.angular.y = 0;
    twist.angular.z = th * turn;

    // Publish it and resolve any remaining callbacks
    cmd_pub.publish(twist);
    ros::spinOnce();
}

QStringListModel *QNode::loggingModel() {
    return &logging_model;
}

void QNode::log(const AppEnums::QLogLevel &level, const std::string &msg) {
    logging_model.insertRows(logging_model.rowCount(), 1);
    std::stringstream logging_model_msg;
    switch (level) {
        case (AppEnums::QLogLevel::Debug): {
            logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
            break;
        }
        case (AppEnums::QLogLevel::Info): {
            logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
            break;
        }
        case (AppEnums::QLogLevel::Warn): {
            emit updateRobotStatus(AppEnums::QRobotStatus::NotReady);
            logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
            break;
        }
        case (AppEnums::QLogLevel::Err): {
            emit updateRobotStatus(AppEnums::QRobotStatus::NotReady);
            logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
            break;
        }
        case (AppEnums::QLogLevel::Fatal): {
            logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
            break;
        }
    }
    QVariant new_row(QString(logging_model_msg.str().c_str()));
    logging_model.setData(logging_model.index(logging_model.rowCount() - 1),
                          new_row);
    emit loggingUpdated();  // used to readjust the scrollbar
}
