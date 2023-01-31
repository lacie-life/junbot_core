//
// Created by lacie on 28/01/2023.
//

#include "ObjectDatabase.h"

bool Cluster::operator ==(const std::string &x){
    return(this->object_name == x);
}

ObjectDatabase::ObjectDatabase()
{
    DataBaseSize = 0;
    mClusters.clear();

    mvInterestNames = {"background", "person", "tvmonitor", "pottedplant", "cup", "teddy bear",
                       "keyboard", "vase", "mouse", "bowl", "chair", "diningtable"};

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

cv::Scalar  ObjectDatabase::getObjectColor(int class_id)
{
    return mvColors[class_id];
}

cv::Scalar  ObjectDatabase::getObjectColor(string class_name)
{
    for (int i = 0; i < mvInterestNames.size(); i++)
    {
        if(class_name== mvInterestNames[i])
        {
            return mvColors[i];
        }
    }
    return mvColors[0];
}

float ObjectDatabase::getObjectSize(int class_id)
{
    return mvSizes[class_id];
}

std::vector<Cluster> ObjectDatabase::getObjectByName(std::string objectName)
{

    std::vector<Cluster>::iterator iter   = mClusters.begin()-1;
    std::vector<Cluster>::iterator it_end = mClusters.end();
    std::vector<Cluster> sameName;
    while(true)
    {
        iter = find(++iter, it_end, objectName);
        if (iter != it_end )
            sameName.push_back(*iter);
        else
            break;
    }
    return sameName;
}

void ObjectDatabase::addObject(Cluster& cluster)
{
    // 1. Check the total quantity, if the database is empty, join directly
    if(!mClusters.size())
    {
        DataBaseSize++;
        cluster.object_id = DataBaseSize;
        mClusters.push_back(cluster);
        return;
    }
    else
    {
        // 2. The object already exists in the database, find out whether the new object already exists in the database
        std::vector<Cluster>::iterator iter   = mClusters.begin()-1;
        std::vector<Cluster>::iterator it_end = mClusters.end();
        std::vector<std::vector<Cluster>::iterator> likely_obj;// iterator over objects with the same name
        while(true)
        {
            iter = find(++iter, it_end, cluster.object_name);// search by name
            if (iter != it_end )// find one, store it
                likely_obj.push_back(iter);
            else // can't find it
                break;
        }

        // 3. If not found, add it directly to the database
        std::vector<Cluster>::iterator best_close;// most recent index
        float center_distance=100;// Corresponding distance
        if(!likely_obj.size())
        {
            DataBaseSize++;
            cluster.object_id = DataBaseSize;
            mClusters.push_back(cluster);
            return;
        }
        else // Find multiple objects with the same name in the database
        {
            // 4. Go through each object with the same name and find the one closest to the center point
            for(unsigned int j = 0; j < likely_obj.size(); j++)
            {
                std::vector<Cluster>::iterator& temp_iter = likely_obj[j];
                Cluster& temp_cluster = *temp_iter;
                Eigen::Vector3f dis_vec = cluster.centroid - temp_cluster.centroid;// center point connection vector
                float dist = dis_vec.norm();
                if(dist < center_distance)
                {
                    center_distance = dist; // shortest distance
                    best_close      = temp_iter;// corresponding index
                }
            }
            // 5. If the distance is smaller than the object size, it is considered to be the same object in the same space,
            // and the information of the object in the database is updated
            for (int i = 0; i < mvInterestNames.size(); i++)
            {
                if(cluster.object_name == mvInterestNames[i])
                {
                    if(center_distance < mvSizes[i])
                    {
                        best_close->prob    = (best_close->prob + cluster.prob)/2.0; // Comprehensive confidence
                        best_close->centroid = (best_close->centroid + cluster.centroid)/2.0; // center mean
                        best_close->size     = (best_close->size + cluster.size)/2.0; // center size
                    }
                    else
                    {
                        // 6. If the distance exceeds the size of the object, it is considered to be the same object in a different position,
                        // and it is directly put into the database
                        DataBaseSize++;
                        cluster.object_id = DataBaseSize;
                        mClusters.push_back(cluster);
                    }
                }
            }

        }
    }

    // Database size limit,
    // if it exceeds a certain size,
    // delete the target with low confidence
    return;
}


