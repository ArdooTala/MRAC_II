#include <ros/ros.h>
#include <ros/console.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Transform.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

//#include <iostream>
#include <vector>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <sstream>

#include <iostream>
#include <pcl/io/pcd_io.h>

#include <boost/thread/thread.hpp>

#include <pcl/filters/extract_indices.h>
#include <pcl/common/transforms.h>

#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/radius_outlier_removal.h>

//#include <tf/transform_listener.h>
//#include <tf/transform_datatypes.h>
//#include <tf_conversions/tf_eigen.h>
//#include <eigen_conversions/eigen_msg.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "my_pcl_tutorial/lookup_transform.h"



ros::Publisher pub;


void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
//    tf::TransformListener listener;

    // Container for original & filtered data
    pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
    pcl::PCLPointCloud2 cloud_filtered;

    // Convert to PCL data type
    pcl_conversions::toPCL(*cloud_msg, *cloud);

    // Perform the DOWNSAMPLING
    pcl::VoxelGrid<pcl::PCLPointCloud2> stefano;
    stefano.setInputCloud (cloudPtr);
    stefano.setLeafSize (0.01, 0.01, 0.01);
    stefano.filter (cloud_filtered);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered2 (new pcl::PointCloud <pcl::PointXYZRGB>);
    pcl::fromPCLPointCloud2(cloud_filtered, *cloud_filtered2);



//    ros::ServiceClient client = nh.serviceClient<my_pcl_tutorial::lookup_transform>("robot2kinect_transform");
    my_pcl_tutorial::lookup_transform srv;
//    srv.request;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
    if (ros::service::call("robot2kinect_transform", srv))
    {
//        geometry_msgs::Transform kos = srv.response;

        Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();

        transform_2.translation() << srv.response.Transformation.translation.x,
            srv.response.Transformation.translation.y, srv.response.Transformation.translation.z;

        transform_2.rotate (Eigen::Quaternionf (srv.response.Transformation.rotation.w,
                                                srv.response.Transformation.rotation.x,
                                                srv.response.Transformation.rotation.y,
                                                srv.response.Transformation.rotation.z));

//        ROS_INFO("Transform Recieved . . .");
        pcl::transformPointCloud (*cloud_filtered2, *transformed_cloud, transform_2);

    }
    else
    {
        ROS_ERROR("Failed to call service add_two_ints");
        transformed_cloud = cloud_filtered2;
    }

    size_t num_kirs = transformed_cloud->size();
//    ROS_WARN("KIIIIIIIIIIIR %d", (int)num_kirs);

//    tf::StampedTransform _transform;
//    tf::TransformListener listener;
//    listener.waitForTransform("/camera_rgb_optical_frame", "/ABB", ros::Time::now(), ros::Duration(2) );
//
//    try{
//        listener.lookupTransform("/camera_rgb_optical_frame", "/ABB", ros::Time::now(), _transform);
//    }
//    catch (tf::TransformException &ex) {
//        ROS_ERROR("%s",ex.what());
//    }
//
//    Eigen::Affine3d kir;
//    tf::transformTFToEigen(_transform, kir);
//



    // build the condition
    pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr range_cond (new pcl::ConditionAnd<pcl::PointXYZRGB> ());

    range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZRGB> ("x", pcl::ComparisonOps::GT, 0)));
    range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZRGB> ("x", pcl::ComparisonOps::LT, 850)));

    range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZRGB> ("y", pcl::ComparisonOps::GT, -0.160)));
    range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZRGB> ("y", pcl::ComparisonOps::LT,  0.160)));

    range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZRGB> ("z", pcl::ComparisonOps::GT, 0.0)));
    range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZRGB> ("z", pcl::ComparisonOps::LT, 0.100)));

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1 (new pcl::PointCloud <pcl::PointXYZRGB>);

    // build the filter
    pcl::ConditionalRemoval<pcl::PointXYZRGB> condrem;
    condrem.setCondition (range_cond);
//    condrem.setInputCloud (cloud_filtered2);
    condrem.setInputCloud (transformed_cloud);
    condrem.setKeepOrganized(false);
    // apply filter
    condrem.filter (*cloud1);

    size_t num_points = cloud1->size();
    ROS_WARN("KIIIIIIIIIIIR %d", (int)num_points);


 //   pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1 (new pcl::PointCloud <pcl::PointXYZRGB>);
 //   pcl::fromPCLPointCloud2(cloud_filtered3, *cloud1);

/*

    pcl::search::Search<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);

    pcl::IndicesPtr indices (new std::vector <int>);

    pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;
    reg.setInputCloud (cloud1);
    // reg.setIndices (indices);
    reg.setSearchMethod (tree);
    reg.setDistanceThreshold (10);
    reg.setPointColorThreshold (15);
    reg.setRegionColorThreshold (8);
    reg.setMinClusterSize (150);

    std::vector<pcl::PointIndices> clusters;
    reg.extract (clusters);

//    std::cout << clusters.size() << std::endl;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    colored_cloud = reg.getColoredCloud ();
//    pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud2 (new pcl::PointCloud <pcl::PointXYZRGB>);


   // Create the filtering object
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud (cloud1);
    extract.setNegative (false); //we want this paper

    int intended_r = 225;
    int intended_g = 0;
    int intended_b = 0;
    int min_error = 800;

    for (size_t i = 0; i < clusters.size(); ++i) {
        pcl::PointIndices paper = clusters[i];
        extract.setIndices (boost::make_shared<const pcl::PointIndices> (paper));
        extract.filter (*color_cloud);
        size_t num_points = color_cloud->size();

        int r = 0;
        int g = 0;
        int b = 0;

        for (size_t j = 0; j < num_points; ++j) {
            pcl::PointXYZRGB pt = color_cloud->points[j];
            r += pt.r;
            g += pt.g;
            b += pt.b;
        }

        r /= num_points;
        g /= num_points;
        b /= num_points;

        int error = abs(intended_r - r) + abs(intended_g - g) + abs(intended_b - b);
        std::cout << error << ", ";
        if (error < min_error) {
            min_error = error;
            *colored_cloud = *color_cloud;
        }
    }

    std::cout << std::endl;
*/

    pcl::PCLPointCloud2 cleancloud;
    pcl::toPCLPointCloud2(*cloud1, cleancloud);
//    pcl::toPCLPointCloud2(*cloud_filtered2, cleancloud);

    // Convert to ROS data type
    sensor_msgs::PointCloud2 output;
    pcl_conversions::fromPCL(cleancloud, output);

    std::stringstream ss;
    ss << "/ABB";
    output.header.frame_id = ss.str();

    // Publish the data
    pub.publish (output);
}

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "my_pcl");
    ros::NodeHandle nh;

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb); //, listener, _transform

    // Create a ROS publisher for the output point cloud
    pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);

    // Spin
    ros::spin ();
}