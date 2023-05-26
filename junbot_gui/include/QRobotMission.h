//
// Created by lacie on 26/05/2023.
//

#ifndef JUNBOT_GUI_QROBOTMISSION_H
#define JUNBOT_GUI_QROBOTMISSION_H

#include <iostream>
#include <QObject>
#include "AppConstants.h"

class QRobotMission {
public:
    explicit QRobotMission();

    ~QRobotMission();

private:
    int mission_id;
    int mission_type;

    AppEnums::QMissionStatus mission_status;
};


#endif //JUNBOT_GUI_QROBOTMISSION_H
