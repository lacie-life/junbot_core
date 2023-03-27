//
// Created by lacie on 27/03/2023.
//

#include "ObjectDatabase.h"

ObjectDatabase::ObjectDatabase()
{
    DataBaseSize = 0;
    mObjects.clear();
    mObjects.clear();

    mvInterestNames = {-1, 0, 62, 58, 41, 77,
                       66, 75, 64, 45, 56, 60};

    // Different colors correspond to different objects
    for (int i = 0; i < mvInterestNames.size(); i++) // background with
    {
        mvColors.push_back(cv::Scalar( i*10 + 40, i*10 + 40, i*10 + 40));
    }
    mvColors[1] = cv::Scalar(255,0,255);

    // object size
    for (int i = 0; i < mvInterestNames.size(); i++)
    {
        // voc dataset 20 types of objects
        mvSizes.push_back(0.6);
    }
    mvSizes[5] = 0.06;   // Bottles within 0.06 meters are considered to be the same object
    mvSizes[10] = 0.5;    // Chair
    mvSizes[2] = 0.25;  // monitor
}

ObjectDatabase::~ObjectDatabase()
{
}