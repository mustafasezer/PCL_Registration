#ifndef PCL_TOOLS_H
#define PCL_TOOLS_H


#include <ios>
#include <iostream>
#include <fstream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <string>
#include <pcl/visualization/pcl_visualizer.h>
#include <Eigen/Geometry>


class pcl_tools
{
    struct transformation_relation{
        //int id;
        int parent_id;
        Eigen::Matrix4f T;
        bool completed;
        //struct transformation_relation* parent;
    };
public:
    pcl_tools();
    Eigen::Matrix4f apply_icp(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out, bool viewResult=true);
    Eigen::Matrix4f apply_ndt(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud, Eigen::Matrix4f init_guess);
    int apply_icp(std::string path_in, std::string path_out);
    pcl::PointCloud<pcl::PointXYZ>::Ptr loadPCD(std::string path);
    pcl::PointCloud<pcl::PointXYZ>::Ptr transform_pcd(pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud, Eigen::Matrix4f transform_matrix);
    int downsample_pcd();
    void viewPCD(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string name="point_cloud", int r=255, int g=255, int b=255);
    void viewICPResult(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_targ, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_aligned);
    pcl::PointCloud<pcl::PointXYZ>::Ptr getSlice(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float z1, float z2, std::string field_name = "z");
    void savePCD(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int scan_no);
    void savePCD(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string path);
    Eigen::Matrix3f quaternion_to_rotation(float x, float y, float z, float w);
    Eigen::Matrix4f createTransformationMatrix(Eigen::Matrix3f rotation, Eigen::Vector3f translation);
    int scale_pcd(float scaling_factor);
    void getPCDStatistics(std::string dir, int low_ind, int up_ind);
    pcl::PointCloud<pcl::PointXYZ>::Ptr projectPCD(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float a, float b, float c, float d);
    Eigen::Matrix4f getInitialGuess(int input, int target);
    Eigen::Matrix4f getTransformation(int input, int target);
    int getInitialGuesses();

    //pcl::visualization::PCLVisualizer *viewer;

    /*struct timespec t1, t2;
    double elapsed_time;
    volatile long long i;*/

    //std::vector<struct transformation_relation> transformations;

    struct transformation_relation transformations[83];

};

#endif // PCL_TOOLS_H
