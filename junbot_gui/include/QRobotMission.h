//
// Created by lacie on 26/05/2023.
//

#ifndef JUNBOT_GUI_QROBOTMISSION_H
#define JUNBOT_GUI_QROBOTMISSION_H

#include <iostream>
#include <QObject>
#include <QPoint>
#include "AppConstants.h"

class QRobotMission {
public:
    explicit QRobotMission();

    ~QRobotMission();

    void setMissionId(int id) { mission_id = id; }
    void setMissionType(int type) { mission_type = type; }
    void setMissionStatus(AppEnums::QMissionStatus status) { mission_status = status; }
    void setStartPoint(QPoint point) { start_point = point; }
    void setCurrentPoint(QPoint point) { current_point = point; }
    void setGoalPoint(QPoint point) { goal_point = point; }

    int getMissionId() { return mission_id; }
    int getMissionType() { return mission_type; }
    AppEnums::QMissionStatus getMissionStatus() { return mission_status; }
    QPoint getStartPoint() { return start_point; }
    QPoint getCurrentPoint() { return current_point; }
    QPoint getGoalPoint() { return goal_point; }

private:
    int mission_id;
    int mission_type;

    QPoint start_point;
    QPoint current_point;
    QPoint goal_point;

    AppEnums::QMissionStatus mission_status;
};


#endif //JUNBOT_GUI_QROBOTMISSION_H
