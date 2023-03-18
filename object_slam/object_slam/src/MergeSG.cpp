//
// Created by lacie on 28/01/2023.
//

#include "MergeSG.h"

MergeSG::MergeSG():

        plane_comparator_(new pcl::PlaneCoefficientComparator<PointT, pcl::Normal>),
        euclidean_comparator_(new pcl::EuclideanPlaneCoefficientComparator<PointT, pcl::Normal>),
        rgb_comparator_(new pcl::RGBPlaneCoefficientComparator<PointT, pcl::Normal>),
        edge_aware_comparator_(new pcl::EdgeAwarePlaneComparator<PointT, pcl::Normal>),
        euclidean_cluster_comparator_(new pcl::EuclideanClusterComparator<PointT, pcl::Normal, pcl::Label>)
{

    mPlanComparator = kPlaneCoefficientComparator;
    mPlanNormal_angle_threshold = 3.0;//   2.0 0.01 45.0
    normal_distance_threshold = 0.02;  // 0.02  0.001  0.1
    mMin_plane_inliers = 10000;// 1000 - 10000

    mPlane_minimum_points=10000;// 1000 2000 3000 - 20000

    mNormal_max_depth_change = 0.02;      // 0.02  0.001  0.1
    mNormal_smooth_size = 20.0f;          // 30.0  1.0  100.0

    mEuclidean_distance_threshold = 0.01f;// 0.01 0.001 0.1

    mObject_minimum_points = 1000;// 50 100 200 - 5000

    applyConfig();

    mpOD = new(ObjectDatabase);
}

MergeSG::~MergeSG()
{
    //  delete plane_comparator_;
    //  delete euclidean_comparator_;
    //  delete rgb_comparator_;
    //  delete edge_aware_comparator_;
    //  delete euclidean_cluster_comparator_;

    delete mpOD;
}

void MergeSG::merge(std::vector<Object>& objects, cv::Mat depth, PointCloudT::Ptr pclMap)
{
    std::cout << "Object number: " << objects.size() << "\n";

    std::vector<Cluster> clusters;
    extract(objects, pclMap, clusters);

    for(std::vector<Cluster>::iterator it = clusters.begin(); it != clusters.end(); ++it)
    {
        mpOD->addObject(*it);
    }

    std::cout << "ObjectDatabase: " << mpOD->mClusters.size() << "\n";
}

void MergeSG::extract(std::vector<Object>& objects,
                      PointCloudT::Ptr point_ptr,
                      std::vector<Cluster>& clusters)
{
    clusters.clear();

    std::vector<pcl::PointIndices> cluster_indices;
    PointCloudT::Ptr cloud_segment(new PointCloudT);

    segment(point_ptr, cloud_segment, cluster_indices);

    //std::cout << "cluster  size  " << cluster_indices.size() << std::endl;

    std::vector<Object3d> object3ds;

    for ( std::vector<pcl::PointIndices>::iterator indices_it = cluster_indices.begin();
          indices_it != cluster_indices.end(); ++indices_it )
    {
        try
        {
            pcl::PointCloud<PointXYZPixel>::Ptr seg(new pcl::PointCloud<PointXYZPixel>);
            copyPointCloud(cloud_segment, indices_it->indices, seg);

            cv::Rect_<float> obj_roi;
            bool ok = getProjectedROI(seg, obj_roi);
            if(!ok) continue;

            //std::cout << "3dobject_roi: " << obj_roi.x     << " "
            //                            << obj_roi.y     << " "
            //                            << obj_roi.width << " "
            //                            << obj_roi.height << std::endl;


            auto cmp_x = [](PointXYZPixel const& l, PointXYZPixel const& r) { return l.x < r.x; };
            auto minmax_x = std::minmax_element(seg->begin(), seg->end(), cmp_x);

            auto cmp_y = [](PointXYZPixel const& l, PointXYZPixel const& r) { return l.y < r.y; };
            auto minmax_y = std::minmax_element(seg->begin(), seg->end(), cmp_y);

            auto cmp_z = [](PointXYZPixel const& l, PointXYZPixel const& r) { return l.z < r.z; };
            auto minmax_z = std::minmax_element(seg->begin(), seg->end(), cmp_z);


            auto sum_x = [](double sum_x, PointXYZPixel const& l){return sum_x + l.x;};
            auto sumx = std::accumulate(seg->begin(), seg->end(), 0.0, sum_x);
            double mean_x =  sumx / seg->size();

            auto sum_y = [](double sum_y, PointXYZPixel const& l){return sum_y + l.y;};
            auto sumy = std::accumulate(seg->begin(), seg->end(), 0.0, sum_y);
            double mean_y =  sumy / seg->size();

            auto sum_z = [](double sum_z, PointXYZPixel const& l){return sum_z + l.z;};
            auto sumz = std::accumulate(seg->begin(), seg->end(), 0.0, sum_z);
            double mean_z =  sumz / seg->size();

            Object3d object3d;
            object3d.rect     = obj_roi;
            object3d.minPt    = Eigen::Vector3f(minmax_x.first->x, minmax_y.first->y, minmax_z.first->z);
            object3d.maxPt    = Eigen::Vector3f(minmax_x.second->x,minmax_y.second->y,minmax_z.second->z);
            object3d.centroid = Eigen::Vector3f(mean_x, mean_y, mean_z);

            object3d.sizePt   = Eigen::Vector3f(object3d.maxPt[0]-object3d.minPt[0],
                                                object3d.maxPt[1]-object3d.minPt[1],
                                                object3d.maxPt[2]-object3d.minPt[2]);

            object3d.boxCenter= Eigen::Vector3f(object3d.minPt[0]+object3d.sizePt[0]/2.0,
                                                object3d.minPt[1]+object3d.sizePt[1]/2.0,
                                                object3d.minPt[2]+object3d.sizePt[2]/2.0);

            object3ds.push_back(object3d);
        }
        catch (std::exception& e)
        {
            std::cout << e.what() << std::endl;
        }
    }

    std::cout << "Object3ds size  " << object3ds.size() << std::endl;

    findMaxIntersectionRelationships(objects, object3ds, clusters);
}

void MergeSG::findMaxIntersectionRelationships(std::vector<Object>& objects,
                                               std::vector<Object3d>& object3d,
                                               std::vector<Cluster>& clusters)
{
    for (std::vector<Object>::iterator obj2d_it = objects.begin();
         obj2d_it != objects.end(); ++obj2d_it)
    {
        std::vector<Object3d>::iterator max_it = object3d.begin();
        double max = 0;
        cv::Rect_<float>  rect2d = obj2d_it->rect;

//        std::cout << "2dobject_roi: " << rect2d.x     << " "
//                                      << rect2d.y     << " "
//                                      << rect2d.width << " "
//                                      << rect2d.height << std::endl;

        for (std::vector<Object3d>::iterator it = max_it;
             it != object3d.end(); ++it)
        {
            cv::Rect_<float> rect3d = it->rect;
            double area = getMatch(rect2d, rect3d);

            // std::cout << "match: " << area << std::endl;

            if (area < max)
            {
                continue;
            }

            max = area;
            max_it = it;
        }

        if (max <= 0)
        {
            std::cout << "Cannot find correlated 3D object " << std::endl;
            continue;
        }

        Cluster cluster;
        /*
        cluster.object.rect = obj2d_it->rect;
        cluster.object.prob = obj2d_it->prob;
        cluster.object.object_name = obj2d_it->object_name;

        cluster.centroid  = max_it->centroid;
        cluster.minPt     = max_it->minPt;
        cluster.maxPt     = max_it->maxPt;
        cluster.sizePt    = max_it->sizePt;
        cluster.boxCenter = max_it->boxCenter;
        */

        cluster.object_name = obj2d_it->object_name;
        cluster.class_id    = obj2d_it->class_id;
        cluster.prob        = obj2d_it->prob;

        cluster.size        = max_it->sizePt;
        cluster.centroid    = max_it->centroid;

        clusters.push_back(cluster);

        object3d.erase(max_it);
    }
}

bool MergeSG::getProjectedROI(const pcl::PointCloud<PointXYZPixel>::ConstPtr& point_cloud,
                              cv::Rect_<float> & roi)
{
    auto cmp_x = [](PointXYZPixel const& l, PointXYZPixel const& r) { return l.pixel_x < r.pixel_x; };

    auto minmax_x = std::minmax_element(point_cloud->begin(), point_cloud->end(), cmp_x);

    roi.x = minmax_x.first->pixel_x;
    auto max_x = minmax_x.second->pixel_x;
    if(roi.x >= 0 && max_x >= roi.x)
    {
        roi.width = max_x - roi.x;

        auto cmp_y = [](PointXYZPixel const& l, PointXYZPixel const& r) { return l.pixel_y < r.pixel_y; };
        auto minmax_y = std::minmax_element(point_cloud->begin(), point_cloud->end(), cmp_y);
        roi.y = minmax_y.first->pixel_y;
        auto max_y = minmax_y.second->pixel_y;
        if(roi.y >= 0 && max_y >= roi.y)
        {
            roi.height = max_y - roi.y;
            return true;
        }
        return false;
    }
    return false;
}

double MergeSG::getMatch(const cv::Rect_<float> & r1, const cv::Rect_<float> & r2)
{
    cv::Rect2i ir1(r1), ir2(r2);
    /* calculate center of rectangle #1  */
    cv::Point2i c1(ir1.x + (ir1.width >> 1), ir1.y + (ir1.height >> 1));
    /* calculate center of rectangle #2  */
    cv::Point2i c2(ir2.x + (ir2.width >> 1), ir2.y + (ir2.height >> 1));

    double a1 = ir1.area(), a2 = ir2.area(), a0 = (ir1 & ir2).area();
    /* calculate the overlap rate*/
    double overlap = a0 / (a1 + a2 - a0);
    /* calculate the deviation between centers #1 and #2*/
    double deviate = sqrt(powf((c1.x - c2.x), 2) + powf((c1.y - c2.y), 2));
    /* calculate the length of diagonal for the rectangle in average size*/
    double len_diag = sqrt(powf(((ir1.width + ir2.width) >> 1), 2) + powf(((ir1.height + ir2.height) >> 1), 2));

    /* calculate the match rate. The more overlap, the more matching. Contrary, the more deviation, the less matching*/

    return overlap * len_diag / deviate;
}


void MergeSG::segment(const PointCloudT::ConstPtr& cloud,
                      PointCloudT::Ptr& cloud_segment,
                      std::vector<pcl::PointIndices>& cluster_indices)
{

    std::cout << "Total original point size = " << cloud->size() << std::endl;

    pcl::copyPointCloud(*cloud, *cloud_segment);

    applyConfig();

    pcl::PointCloud<pcl::Normal>::Ptr normal_cloud(new pcl::PointCloud<pcl::Normal>);
    estimateNormal(cloud, normal_cloud);

    std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > > regions;
    pcl::PointCloud<pcl::Label>::Ptr labels(new pcl::PointCloud<pcl::Label>);
    std::vector<pcl::PointIndices> label_indices;

    segmentPlanes(cloud, normal_cloud, regions, labels, label_indices);

    std::cout << "find plane : " << label_indices.size() << std::endl;

    segmentObjects(cloud, regions, labels, label_indices, cluster_indices);

}


void MergeSG::estimateNormal(const PointCloudT::ConstPtr& cloud,
                             pcl::PointCloud<pcl::Normal>::Ptr& normal_cloud)
{
    normal_estimation_.setInputCloud(cloud);
    normal_estimation_.compute(*normal_cloud);

    float* distance_map = normal_estimation_.getDistanceMap();
    boost::shared_ptr<pcl::EdgeAwarePlaneComparator<PointT, pcl::Normal> > eapc =
            boost::dynamic_pointer_cast<pcl::EdgeAwarePlaneComparator<PointT, pcl::Normal> >(edge_aware_comparator_);
    eapc->setDistanceMap(distance_map);
    eapc->setDistanceThreshold(0.01f, false);

}

void MergeSG::segmentPlanes(
        const PointCloudT::ConstPtr& cloud,
        const pcl::PointCloud<pcl::Normal>::Ptr& normal_cloud,
        std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > >& regions,
        pcl::PointCloud<pcl::Label>::Ptr labels,
        std::vector<pcl::PointIndices>& label_indices)
{

    double mps_start = pcl::getTime ();
    std::vector<pcl::ModelCoefficients> model_coefficients;
    std::vector<pcl::PointIndices> inlier_indices;
    std::vector<pcl::PointIndices> boundary_indices;

    plane_segmentation_.setInputNormals(normal_cloud);
    plane_segmentation_.setInputCloud(cloud);

    plane_segmentation_.segmentAndRefine(regions,
                                         model_coefficients,
                                         inlier_indices,
                                         labels,
                                         label_indices,
                                         boundary_indices);
    // mps.segment (regions); // not refinement ====
    double mps_end = pcl::getTime ();
    std::cout << "MPS+Refine took: " << double(mps_end - mps_start) << std::endl;
}



void MergeSG::segmentObjects(
        const PointCloudT::ConstPtr& cloud,
        std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > >& regions,
        pcl::PointCloud<pcl::Label>::Ptr labels,
        std::vector<pcl::PointIndices>& label_indices,
        std::vector<pcl::PointIndices>& cluster_indices)
{


    std::vector<bool> plane_labels;
    plane_labels.resize(label_indices.size(), false);
    for (size_t i = 0; i < label_indices.size(); i++)
    {

        if (label_indices[i].indices.size() > mPlane_minimum_points)
        {
            plane_labels[i] = true;
        }
    }

    euclidean_cluster_comparator_->setInputCloud(cloud);
    euclidean_cluster_comparator_->setLabels(labels);
    euclidean_cluster_comparator_->setExcludeLabels(plane_labels);

    pcl::PointCloud<pcl::Label> euclidean_labels;


    pcl::OrganizedConnectedComponentSegmentation<PointT, pcl::Label>
            euclidean_segmentation(euclidean_cluster_comparator_);

    euclidean_segmentation.setInputCloud(cloud);
    euclidean_segmentation.segment(euclidean_labels, cluster_indices);


    auto func = [this](pcl::PointIndices indices) { return indices.indices.size() < this->mObject_minimum_points; };

    cluster_indices.erase(std::remove_if(cluster_indices.begin(), cluster_indices.end(), func), cluster_indices.end());

    PCL_INFO ("Got %d euclidean clusters!\n", cluster_indices.size ());

}

void MergeSG::applyConfig()
{
    //normal_estimation_.setNormalEstimationMethod(normal_estimation_.SIMPLE_3D_GRADIENT);
    normal_estimation_.setNormalEstimationMethod(normal_estimation_.COVARIANCE_MATRIX);
    normal_estimation_.setMaxDepthChangeFactor(mNormal_max_depth_change);
    normal_estimation_.setNormalSmoothingSize(mNormal_smooth_size);

    euclidean_cluster_comparator_->setDistanceThreshold(mEuclidean_distance_threshold, false);

    plane_segmentation_.setMinInliers(mMin_plane_inliers);
    plane_segmentation_.setAngularThreshold(pcl::deg2rad(mPlanNormal_angle_threshold));
    plane_segmentation_.setDistanceThreshold(normal_distance_threshold);

    if (mPlanComparator == kPlaneCoefficientComparator)
    {
        plane_segmentation_.setComparator(plane_comparator_);
    }
    else if (mPlanComparator == kEuclideanPlaneCoefficientComparator)
    {
        plane_segmentation_.setComparator(euclidean_comparator_);
    }
    else if (mPlanComparator == kRGBPlaneCoefficientComparator)
    {
        plane_segmentation_.setComparator(rgb_comparator_);
    }
    else if (mPlanComparator == kEdgeAwarePlaneComaprator)
    {
        plane_segmentation_.setComparator(edge_aware_comparator_);
    }
}

void MergeSG::copyPointCloud(const PointCloudT::ConstPtr& original,
                             const std::vector<int>& indices,
                             pcl::PointCloud<PointXYZPixel>::Ptr& dest)
{
    pcl::copyPointCloud(*original, indices, *dest);
    uint32_t width = original->width;
    for (uint32_t i = 0; i < indices.size(); i++)
    {
        dest->points[i].pixel_x = indices[i] % width;
        dest->points[i].pixel_y = indices[i] / width;
    }
}
