//
// Created by lacie on 26/05/2023.
//

#include "QRobotMission.h"

QRobotMission::QRobotMission() {
    mission_id = 0;
    mission_type = 0;
    mission_status = AppEnums::QMissionStatus::Idle;
}

QRobotMission::~QRobotMission() {

}

