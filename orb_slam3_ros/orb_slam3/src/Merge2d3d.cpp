//
// Created by lacie on 28/01/2023.
//

#include "Merge2d3d.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

Merge2d3d::Merge2d3d()
{
    mStat.setMeanK (30);
    mStat.setStddevMulThresh (1.0);

    mVoxel.setLeafSize( 0.01, 0.01, 0.01);
    mpOD = new(ObjectDatabase);
}

Merge2d3d::~Merge2d3d()
{
    delete mpOD;
}

void Merge2d3d::merge(std::vector<Object>& objects, cv::Mat depth, PointCloud::Ptr pclMap)
{
    if(!objects.empty())
    {
        for(unsigned int i=0; i<objects.size(); i++)
        {
            //Cluster* pcluster = new(Cluster);
            Cluster cluster;
            bool ok = mergeOne(objects[i], cluster, depth, pclMap);
            if(ok)
                mpOD->addObject(cluster);
        }
    }
    // std::cout<< "OD size: " << mpOD->mClusters.size() << std::endl;
}

bool Merge2d3d::mergeOne(Object& object, Cluster& cluster, cv::Mat depth_img, PointCloud::Ptr pclMap)
{
    if(object.prob >0.54)
    {
        float* depth = (float*)depth_img.data;

        cv::Rect_<float> rect = object.rect;
        int beg = (int)rect.x + ((int)rect.y-1)*depth_img.cols - 1;

        int count = 0;
        float sum = 0;
        int row_beg = (int)rect.height*0.3;
        int row_end = (int)rect.height*0.7;
        int col_beg = (int)rect.width*0.3;
        int col_end = (int)rect.width*0.7;
        for(int k=row_beg; k<row_end; k++)
        {
            int start = beg + k*depth_img.cols + col_beg;
            int end   = beg + k*depth_img.cols + col_end ;
            for(int j = start; j<end; j++)
            {
                float d = depth[j];
                if (d < 0.5 || d > 4.0)
                    continue;
                sum += d;
                count++;
            }
        }
        float depth_threshold = 0.0;
        if(count>0) depth_threshold = sum / count;
        else
            return false;

        pcl::PointIndices indices;
        row_beg = (int)rect.height*0.2;
        row_end = (int)rect.height*0.8;
        col_beg = (int)rect.width*0.2;
        col_end = (int)rect.width*0.8;
        for(int k=row_beg; k<row_end; k++)
        {
            int start = beg + k*depth_img.cols + col_beg;
            int end   = beg + k*depth_img.cols + col_end;
            for(int j = start; j<end; j++)
            {
                if( abs(depth[j] - depth_threshold) < 0.2 )
                    indices.indices.push_back (j);
            }
        }

        if(indices.indices.size() < 50)
            return false;

        mExtractInd.setInputCloud(pclMap);

        mExtractInd.setIndices (boost::make_shared<const pcl::PointIndices> (indices));
        PointCloud::Ptr before (new PointCloud);
        mExtractInd.filter (*before);

        mVoxel.setInputCloud( before );
        mVoxel.filter( *before );

        PointCloud::Ptr after_stat (new PointCloud);
        mStat.setInputCloud (before);
        mStat.filter (*after_stat);
        if(after_stat->width * after_stat->height< 30)
            return false;

        Eigen::Vector4f cent;
        pcl::compute3DCentroid(*after_stat, cent);

        Eigen::Vector4f minPt, maxPt;
        pcl::getMinMax3D (*after_stat, minPt, maxPt);

        cluster.object_name = object.object_name;
        cluster.class_id    = object.class_id;
        cluster.prob        = object.prob;
        //cluster->minPt       = Eigen::Vector3f(minPt[0], minPt[1], minPt[2]);
        //cluster->maxPt       = Eigen::Vector3f(maxPt[0], maxPt[1], maxPt[2]);
        cluster.size        = Eigen::Vector3f(maxPt[0]-minPt[0], maxPt[1]-minPt[1], maxPt[2]-minPt[2]);
        cluster.centroid    = Eigen::Vector3f(cent[0],  cent[1],  cent[2]);

        return true;
    }
    return false;
}
