#include <sstream>
#include <iostream>
#include <vector>

#include <Eigen/Core>
#include <boost/thread/thread.hpp>

#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/time.h>
#include <pcl/console/print.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/filters/filter.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/features/normal_3d.h>



ros::Publisher pub;

void
cloud_cb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg) {
    const float leaf = 0.01f;

    // Container for original & filtered data
    pcl::PCLPointCloud2 *cloud = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
    pcl::PCLPointCloud2 cloud_filtered;

    // Convert to PCL data type
    pcl_conversions::toPCL(*cloud_msg, *cloud);

    // Perform the actual filtering
    pcl::VoxelGrid <pcl::PCLPointCloud2> sor;
    sor.setInputCloud(cloudPtr);
    sor.setLeafSize(leaf, leaf, leaf);
    sor.filter(cloud_filtered);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1(new pcl::PointCloud <pcl::PointXYZRGB>);
    pcl::fromPCLPointCloud2(cloud_filtered, *cloud1);

    pcl::search::Search<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree <pcl::PointXYZRGB>);
//    pcl::IndicesPtr indices (new std::vector <int>);

    pcl::RegionGrowingRGB <pcl::PointXYZRGB> reg;
    reg.setInputCloud(cloud1);
    // reg.setIndices (indices);
    reg.setSearchMethod(tree);
    reg.setDistanceThreshold(10);
    reg.setPointColorThreshold(5);
    reg.setRegionColorThreshold(4);
    reg.setMinClusterSize(200);

    std::vector <pcl::PointIndices> clusters;
    reg.extract(clusters);

    std::cout << clusters.size() << std::endl;

//    // Create the filtering object
//    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
//    pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud (new pcl::PointCloud <pcl::PointXYZRGB>);
//
//
//    pcl::PointIndices inds = clusters[0];
//
//    // Extract the inliers
//    extract.setInputCloud (cloud1);
//    extract.setIndices (boost::make_shared<const pcl::PointIndices> (inds));
//    extract.setNegative (false);
//    extract.filter (*colored_cloud);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud();

    pcl::PCLPointCloud2 clean_cloud;
    pcl::toPCLPointCloud2(*colored_cloud, clean_cloud);


/*
//pose estimation of rigid objects

    // Create the normal estimation class
    pcl::NormalEstimation <pcl::PointXYZ, pcl::PointNormal> ne;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr ntree (new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setSearchMethod(ntree);
    ne.setRadiusSearch(0.03);

    // SCENE NORMALS
    pcl::PointCloud<pcl::PointXYZ>::Ptr scene(new pcl::PointCloud <pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(clean_cloud, *scene);
    ne.setInputCloud(scene);
    pcl::PointCloud<pcl::PointNormal>::Ptr scene_normals (new pcl::PointCloud<pcl::PointNormal>);
    ne.compute(*scene_normals);

    // OBJECT NORMALS
    pcl::PointCloud<pcl::PointXYZ>::Ptr object (new pcl::PointCloud <pcl::PointXYZ>);
    pcl::io::loadPCDFile<pcl::PointXYZ>("/home/ardoo/catkin_ws/src/my_pcl_tutorial/Ball_InScale.pcd", *object);
    pcl::VoxelGrid<pcl::PointXYZ> grid;
    grid.setLeafSize (leaf, leaf, leaf);
    grid.setInputCloud (object);
    grid.filter (*object);
    ne.setInputCloud(object);
    pcl::PointCloud<pcl::PointNormal>::Ptr object_normals (new pcl::PointCloud<pcl::PointNormal>);
    ne.compute(*object_normals);

    // FEATURE CLOUDS
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr object_features (new pcl::PointCloud <pcl::FPFHSignature33>);
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr scene_features (new pcl::PointCloud <pcl::FPFHSignature33>);

    // ALIGNED OBJECT
    pcl::PointCloud<pcl::PointNormal>::Ptr object_aligned (new pcl::PointCloud <pcl::PointNormal>);

    // Estimate features
    pcl::FPFHEstimationOMP <pcl::PointXYZ, pcl::PointNormal, pcl::FPFHSignature33> fest;
    fest.setRadiusSearch(0.06);

    fest.setInputCloud(object);
    fest.setInputNormals(object_normals);
    fest.compute(*object_features);

    fest.setInputCloud(scene);
    fest.setInputNormals(scene_normals);
    fest.compute(*scene_features);

    // Perform alignment
    pcl::SampleConsensusPrerejective <pcl::PointNormal, pcl::PointNormal, pcl::FPFHSignature33> align;
    align.setMaximumIterations(50000); // Number of RANSAC iterations
    align.setNumberOfSamples(3); // Number of points to sample for generating/prerejecting a pose
    align.setCorrespondenceRandomness(15); // Number of nearest features to use
    align.setSimilarityThreshold(0.9f); // Polygonal edge length similarity threshold *****************
    align.setMaxCorrespondenceDistance(2.5f * leaf); // Inlier threshold
    align.setInlierFraction(0.05f); // Required inlier fraction for accepting a pose hypothesis

    align.setInputSource(object_normals);
    align.setSourceFeatures(object_features);
    align.setInputTarget(scene_normals);
    align.setTargetFeatures(scene_features);

    align.align(*object_aligned);

    pcl::PointCloud<pcl::PointXYZ>::Ptr aligned (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud (*scene, *aligned, align.getFinalTransformation());

    pcl::PCLPointCloud2 final_cloud;
    pcl::toPCLPointCloud2(*aligned, final_cloud);
*/
//====PUBLISH=================================================================================
    sensor_msgs::PointCloud2 output;
    pcl_conversions::fromPCL(clean_cloud, output);

    std::stringstream ss;
    ss << "camera_rgb_optical_frame";
    output.header.frame_id = ss.str();
    pub.publish(output);
//============================================================================================
}

int
main(int argc, char **argv) {
    ros::init(argc, argv, "my_pcl");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("input", 1, cloud_cb);
    pub = nh.advertise<sensor_msgs::PointCloud2>("output", 1);

    ros::spin();
}
