#include <ios>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <string>
#include <sstream>
#include <vector>




//Transform
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "pcl_tools.h"

#include <pcl/filters/crop_box.h>
#include <pcl/common/common.h>

using namespace std;
struct timespec t1, t2;
double elapsed_time;
volatile long long i;


int file_exists (const char *filename)
{
    struct stat   buffer;
    return (stat (filename, &buffer) == 0);
}

void start_timer_(){
    clock_gettime(CLOCK_MONOTONIC,  &t1);
}

void end_timer_(string message = "Elapsed time:"){
    clock_gettime(CLOCK_MONOTONIC,  &t2);
    elapsed_time = (t2.tv_sec - t1.tv_sec) + (double) (t2.tv_nsec - t1.tv_nsec) * 1e-9;
    std::cout << message << " " << elapsed_time << " seconds" << std::endl;
}


void print_pcd_xyz(pcl::PointCloud<PointT>::Ptr cloud){
    for (size_t i = 0; i < cloud->points.size (); ++i)
        std::cout << "    " << cloud->points[i].x
                  << " "    << cloud->points[i].y
                  << " "    << cloud->points[i].z << std::endl;
}


// This function displays the help
void
showHelp(char * program_name)
{
    std::cout << std::endl;
    std::cout << "Usage: " << program_name << " cloud_filename.[pcd|ply]" << std::endl;
    std::cout << "-h:  Show this help." << std::endl;
}

void create_launch_file(int min_index, int max_index){
    //Create launch file for icp_result
    for(int i=min_index; i<=max_index; i++){
        std::cout << "<node pkg=\"pcl_ros\" type=\"pcd_to_pointcloud\" name=\"cloud_" << i << "\" args=\"/home/mustafasezer/dataset/Downsampled_Projected_3.5_4.5/scan_" << i << ".pcd 0.1\" output=\"screen\">\n\t<param name=\"frame_id\" value=\"iser_transform_1\" />\n\t<remap from=\"cloud_pcd\" to=\"cloud_" << i << "\" />\n</node>\n\n";
    }
}


int main()
{
    pcl_tools pcl_tool;

    pcl_tool.getInitialGuesses("/home/mustafasezer/Initial Guesses.txt");

    /*pcl_tool.getTransformations("/home/mustafasezer/icp_results.txt", pcl_tool.transformations_icp);
    pcl_tool.getTransformations("/home/mustafasezer/ndt_results.txt", pcl_tool.transformations_ndt);

    pcl_tool.computeGlobalTransformations();*/

    fstream icp_result, ndt_result;
    icp_result.open("/home/mustafasezer/icp_results.txt", ios::out);
    ndt_result.open("/home/mustafasezer/ndt_results.txt", ios::out);

    fstream overall_icp, overall_ndt;
    overall_icp.open("/home/mustafasezer/overall_icp.txt", ios::out);
    overall_ndt.open("/home/mustafasezer/overall_ndt.txt", ios::out);


    for(int i=1; i<=pcl_tool.MAX_NUM_SCANS; i++){
        if(pcl_tool.transformations[i-1].is_parent){
            std::cout << "scan_" << i <<  " is a parent transformation" << std::endl;
            overall_icp << "scan_" << i <<  " to scan_" << i << " \n1 0 0 0\n0 1 0 0\n0 0 1 0\n0 0 0 1\n\n";
            overall_ndt << "scan_" << i <<  " to scan_" << i << " \n1 0 0 0\n0 1 0 0\n0 0 1 0\n0 0 0 1\n\n";
            icp_result << "scan_" << i << " to scan_" << i << std::endl;
            icp_result << pcl_tool.transformations[i-1].init_guess << std::endl << std::endl;
            ndt_result << "scan_" << i << " to scan_" << i << std::endl;
            ndt_result << pcl_tool.transformations[i-1].init_guess << std::endl << std::endl;
            continue;
        }
        if(pcl_tool.transformations[i-1].ok == false){
            std::cout << "Transformation problematic, scan_" << i << " is being skipped" << std::endl;
            continue;
        }
        if(pcl_tool.transformations[i-1].parent_id>83 || pcl_tool.transformations[i-1].parent_id<1){
            std::cout << "Parent id problematic, scan_" << i << " is being skipped" << std::endl;
            continue;
        }
        stringstream target_filename, input_filename;
        target_filename.clear();
        input_filename.clear();
        input_filename << "/home/mustafasezer/Desktop/downsampled_scaled_rgbclouds/cloud_" << i << "_0.1.pcd";
        target_filename << "/home/mustafasezer/Desktop/downsampled_scaled_rgbclouds/cloud_" << pcl_tool.transformations[i-1].parent_id << "_0.1.pcd";
        string strtarget_filename = target_filename.str();
        string strinput_filename = input_filename.str();
        if (!file_exists (strtarget_filename.c_str()))
        {
            std::cerr << target_filename.str() << " does not exist" << std::endl;
            continue;
        }
        else if (!file_exists (strinput_filename.c_str()))
        {
            std::cerr << input_filename.str() << " does not exist" << std::endl;
            continue;
        }

        //Read input cloud
        std::cout << "Reading input cloud " << input_filename.str() << std::endl;
        start_timer_();
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
        if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (input_filename.str(), *cloud_in_rgb) == -1) //* load the file
        {
            string temp(input_filename.str());
            PCL_ERROR ("Couldn't read %s\n", temp.c_str());
            continue;
        }
        pcl::PointCloud<PointT>::Ptr cloud_in(new pcl::PointCloud<PointT>);
        pcl::copyPointCloud(*cloud_in_rgb, *cloud_in);
        //pcl::PointCloud<PointT>::Ptr cloud_in = pcl_tool.loadPCD(input_filename.str());
        end_timer_("Input cloud loaded in:");
        if(cloud_in->points.size() == 0){
            std::cout << input_filename.str() << " empty" << std::endl;
            continue;
        }

        //Read target cloud
        std::cout << "Reading target cloud " << target_filename.str() << std::endl;
        start_timer_();
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_targ_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
        if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (target_filename.str(), *cloud_targ_rgb) == -1) //* load the file
        {
            string temp(target_filename.str());
            PCL_ERROR ("Couldn't read %s\n", temp.c_str());
            continue;
        }
        pcl::PointCloud<PointT>::Ptr cloud_targ(new pcl::PointCloud<PointT>);
        pcl::copyPointCloud(*cloud_targ_rgb, *cloud_targ);
        //pcl::PointCloud<PointT>::Ptr cloud_targ = pcl_tool.loadPCD(target_filename.str());
        end_timer_("Target cloud loaded in:");

        if(cloud_targ->points.size() == 0){
            std::cout << target_filename.str() << " empty" << std::endl;
            cloud_in.reset();
            cloud_targ.reset();
            continue;
        }

        bool filter_xy_range =false;
        if(filter_xy_range){
            cloud_in = pcl_tool.getSlice(cloud_in, -50, 50, "x");
            cloud_in = pcl_tool.getSlice(cloud_in, -50, 50, "y");
            cloud_targ = pcl_tool.getSlice(cloud_targ, -50, 50, "x");
            cloud_targ = pcl_tool.getSlice(cloud_targ, -50, 50, "y");
        }

        pcl::PointCloud<PointT>::Ptr cloud_in_transformed = pcl_tool.transform_pcd(cloud_in, pcl_tool.transformations[i-1].init_guess);

        Eigen::Matrix4f init_guess;
        init_guess.setIdentity();

        //Whole data
        //Eigen::Matrix4f transform_matrix_ndt = pcl_tool.apply_ndt(cloud_in_transformed, cloud_targ, init_guess);
        //Eigen::Matrix4f transform_matrix_icp = pcl_tool.apply_icp(cloud_in_transformed, cloud_targ, true);

        //Only specific z range
        pcl::PointCloud<PointT>::Ptr cloud_in_transformed_filtered = pcl_tool.getSlice(cloud_in_transformed, 2.5, 20);
        pcl::PointCloud<PointT>::Ptr cloud_targ_filtered = pcl_tool.getSlice(cloud_targ, 2.5, 20);

        std::cout << "Applying NDT" << std::endl;
        start_timer_();
        Eigen::Matrix4f transform_matrix_ndt = pcl_tool.apply_ndt(cloud_in_transformed_filtered, cloud_targ_filtered, init_guess);
        end_timer_("NDT completed in:");

        std::cout << "Aligning scan_" << i << " to scan_" << pcl_tool.transformations[i-1].parent_id << std::endl;
        start_timer_();
        //Eigen::Matrix4f transform_matrix_icp = pcl_tool.apply_icp(cloud_in_transformed_filtered, cloud_targ_filtered, false);
        Eigen::Matrix4f transform_matrix_icp = pcl_tool.pairAlign(cloud_in_transformed_filtered, cloud_targ_filtered, false);
        end_timer_("Pair aligned in:");

        icp_result << "scan_" << i << " to scan_" << pcl_tool.transformations[i-1].parent_id << std::endl;
        icp_result << transform_matrix_icp << std::endl << std::endl;

        ndt_result << "scan_" << i << " to scan_" << pcl_tool.transformations[i-1].parent_id << std::endl;
        ndt_result << transform_matrix_ndt << std::endl << std::endl;

        stringstream output_filename;
        output_filename << "/home/mustafasezer/dataset/ICP_Results_rgb/scan_" << i << "_0.1.pcd";

        /*pcl_tools::transformation_relation current_transform;
        current_transform = pcl_tool.transformations[i-1];
        while(current_transform.is_parent==false){
            if(pcl_tool.transformations[current_transform.parent_id-1].completed==false){
                std::cerr << "Parent transformation is not completed for scan_" << i << std::endl;
                break;
            }
            pcl_tool.transformations[i-1].T = pcl_tool.transformations[i-1].T * pcl_tool.transformations[current_transform.parent_id-1].T;
            current_transform = pcl_tool.transformations[current_transform.parent_id-1];
        }*/
        if(pcl_tool.transformations[pcl_tool.transformations[i-1].parent_id-1].completed==false){
            std::cerr << "Parent transformation is not completed for scan_" << i << std::endl;
            cloud_in.reset();
            cloud_targ.reset();
            cloud_in_transformed.reset();
            continue;
        }
        pcl_tool.transformations[i-1].T = pcl_tool.transformations[pcl_tool.transformations[i-1].parent_id-1].T * transform_matrix_icp * pcl_tool.transformations[i-1].init_guess;
        pcl_tool.transformations[i-1].T_ndt = pcl_tool.transformations[pcl_tool.transformations[i-1].parent_id-1].T_ndt * transform_matrix_ndt * pcl_tool.transformations[i-1].init_guess;
        pcl_tool.transformations[i-1].completed = true;

        //pcl_tool.transformations[i-1].T = pcl_tool.transformations[i-1].T * transform_matrix_icp * pcl_tool.transformations[i-1].init_guess;

        //RGB Version
        //pcl_tool.savePCD((pcl_tool.transform_pcd(cloud_in, pcl_tool.transformations[i-1].T)), output_filename.str());
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in_rgb_transformed(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::transformPointCloud(*cloud_in_rgb, *cloud_in_rgb_transformed, pcl_tool.transformations[i-1].T);
        pcl::io::savePCDFileASCII (output_filename.str(), *cloud_in_rgb_transformed);


        if(filter_xy_range){
            cloud_in_rgb = pcl_tool.getSliceRGB(cloud_in_rgb, -50, 50, "x");
            cloud_in_rgb = pcl_tool.getSliceRGB(cloud_in_rgb, -50, 50, "y");
            stringstream filtered_rgb_filename;
            filtered_rgb_filename << "/home/mustafasezer/dataset/downsampled_scaled_rgbclouds_filtered/cloud_" << i << "_0.1.pcd";
            pcl::io::savePCDFileASCII (filtered_rgb_filename.str(), *cloud_in_rgb);
        }


        int top_parent = pcl_tool.topMostParent(i);
        if(top_parent>=1 && top_parent<=pcl_tool.MAX_NUM_SCANS){
            overall_icp << "scan_" << i << " to scan_" << top_parent << std::endl;
            overall_icp << pcl_tool.transformations[i-1].T << std::endl << std::endl;

            overall_ndt << "scan_" << i << " to scan_" << top_parent << std::endl;
            overall_ndt << pcl_tool.transformations[i-1].T_ndt << std::endl << std::endl;
        }

        cloud_in.reset();
        cloud_targ.reset();
        cloud_in_transformed.reset();
        cloud_in_rgb.reset();
        cloud_targ_rgb.reset();
    }
    icp_result.close();
    ndt_result.close();
    overall_icp.close();
    overall_ndt.close();



    //for(int i=1; i<7; i++)
    //    std::cout << pcl_tool.getTransformation(i+1, i) << std::endl << std::endl;

    return 0;
}


/*
    //Read input cloud
    std::cout << "Reading input cloud" << std::endl;
    start_timer_();
    pcl::PointCloud<PointT>::Ptr cloud_in = pcl_tool.loadPCD("/home/mustafasezer/dataset/Downsampled_Cuboids_Scaled/scan_5.pcd");
    //pcl::PointCloud<PointT>::Ptr cloud_in = pcl_tool.loadPCD("/home/mustafasezer/dataset/Downsampled/scan_1.pcd");
    end_timer_("Input cloud loaded in:");

    //Read output cloud
    std::cout << "Reading output cloud" << std::endl;
    start_timer_();
    pcl::PointCloud<PointT>::Ptr cloud_targ = pcl_tool.loadPCD("/home/mustafasezer/dataset/Downsampled_Cuboids_Scaled/scan_1.pcd");
    end_timer_("Output cloud loaded in:");


    Eigen::Vector3f translation_in(-1.649, 4.660, 0.000);
    Eigen::Matrix3f rotation_in = pcl_tool.quaternion_to_rotation(0.000, 0.000, 0.005, 1.000);
    Eigen::Matrix4f cloud_in_transformation = pcl_tool.createTransformationMatrix(rotation_in, translation_in);

    pcl::PointCloud<PointT>::Ptr cloud_in_transformed = pcl_tool.transform_pcd(cloud_in, cloud_in_transformation);

    //Eigen::Matrix4f init_guess;
    //init_guess.setIdentity();

    //Whole data
    //pcl_tool.apply_ndt(cloud_in_transformed, cloud_targ_transformed, init_guess);
    //pcl_tool.apply_icp(cloud_in_transformed, cloud_targ);

    //Only specific z range
    pcl::PointCloud<PointT>::Ptr cloud_in_transformed_filtered = pcl_tool.getSlice(cloud_in_transformed, 2.5, 15);
    pcl::PointCloud<PointT>::Ptr cloud_targ_transformed_filtered = pcl_tool.getSlice(cloud_targ, 2.5, 15);

    //pcl_tool.apply_ndt(cloud_in_transformed_filtered, cloud_targ_transformed_filtered, init_guess);
    pcl_tool.apply_icp(cloud_in_transformed_filtered, cloud_targ_transformed_filtered);

    return 0;
*/



/*pcl::PointT minPt, maxPt;
pcl::getMinMax3D (*cloud_in_transformed_filtered, minPt, maxPt);
cloud_in_transformed_filtered = pcl_tool.getSlice(cloud_in_transformed_filtered, minPt.x/1.2, maxPt.x/1.2, "x");
cloud_in_transformed_filtered = pcl_tool.getSlice(cloud_in_transformed_filtered, minPt.y/1.2, maxPt.y/1.2, "y");

pcl::getMinMax3D (*cloud_targ_transformed_filtered, minPt, maxPt);
cloud_targ_transformed_filtered = pcl_tool.getSlice(cloud_targ_transformed_filtered, minPt.x/1.2, maxPt.x/1.2, "x");
cloud_targ_transformed_filtered = pcl_tool.getSlice(cloud_targ_transformed_filtered, minPt.y/1.2, maxPt.y/1.2, "y");*/





/*
pcl::PointCloud<PointT>::Ptr cloudOut;
pcl::CropBox<PointT> cropFilter;
cropFilter.setInputCloud (cloud_in_transformed);
Eigen::Vector4f min_pt (-100.0f, -100.0f, 0.0f, 1.0f);
Eigen::Vector4f max_pt (100.0f, 100.0f, 500.0f, 1.0f);
cropFilter.setMin(min_pt);
cropFilter.setMax(max_pt);
cropFilter.filter(*cloudOut);
pcl_tool.viewPCD(cloudOut);
return 0;*/


/*pcl_tool.getPCDStatistics("/home/mustafasezer/dataset/Downsampled", 1, 1);
std::cout << "Before scaling:\n";
pcl_tool.getPCDStatistics("/home/mustafasezer/dataset/Downsampled_Cuboids", 1, 1);
pcl_tool.scale_pcd(0.2);\
std::cout << "After scaling:\n";
pcl_tool.getPCDStatistics("/home/mustafasezer/dataset/Downsampled_Cuboids_Scaled", 1, 1);*/



/*pcl::visualization::PCLVisualizer viewer ("Matrix transformation example");
pcl::PointCloud<PointT>::Ptr cloud1= pcl_tool.loadPCD("/home/mustafasezer/dataset/Downsampled/scan_1.pcd");
pcl::PointCloud<PointT>::Ptr cloud2= pcl_tool.loadPCD("/home/mustafasezer/dataset/Downsampled/scan_2.pcd");
pcl::PointCloud<PointT>::Ptr cube1= pcl_tool.loadPCD("/home/mustafasezer/dataset/Downsampled_Cuboids/scan_1.pcd");
pcl::PointCloud<PointT>::Ptr cube2= pcl_tool.loadPCD("/home/mustafasezer/dataset/Downsampled_Cuboids/scan_2.pcd");

// Define R,G,B colors for the point cloud
pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud1_color_handler(cloud1, 255, 255, 255);
//viewer.addPointCloud (cloud1, cloud1_color_handler, "cloud1");
pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud2_color_handler(cloud2, 0, 255, 0);
//viewer.addPointCloud (cloud2, cloud2_color_handler, "cloud2");
pcl::visualization::PointCloudColorHandlerCustom<PointT> cube1_color_handler(cube1, 255, 0, 0);
viewer.addPointCloud (cube1, cube1_color_handler, "cube1");
pcl::visualization::PointCloudColorHandlerCustom<PointT> cube2_color_handler(cube2, 0, 0, 255);
viewer.addPointCloud (cube2, cube2_color_handler, "cube2");


//viewer.addCoordinateSystem (1.0, name, 0);
viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud1");
viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud2");
viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cube1");
viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cube2");
//viewer.setPosition(800, 400); // Setting visualiser window position

while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
    viewer.spinOnce ();
}
return 0;*/



/*
    //Project to xy plane
    for(int i=1; i<=83; i++){
        stringstream filename;
        filename.clear();
        filename << "/home/mustafasezer/dataset/Downsampled/scan_" << i << ".pcd";
        cout << filename.str() << endl;
        string strfilename = filename.str();
        if (!file_exists (strfilename.c_str()))
        {
            std::cerr << filename << "does not exist" << std::endl;
            continue;
        }

        pcl::PointCloud<PointT>::Ptr cloud = pcl_tool.loadPCD(filename.str());

        std::cerr << "PointCloud read: " << cloud->width * cloud->height
                  << " data points (" << pcl::getFieldsList (*cloud) << ")."<< endl;

        if(cloud->points.size() == 0){
            std::cout << filename << " empty\n";
            continue;
        }


        for(float j=1.5; j<3.5; ){
            pcl::PointCloud<PointT>::Ptr cloud_in_f = pcl_tool.getSlice(cloud, 3.5+j, 4.5+j);
            stringstream filename2;
            filename2 << "/home/mustafasezer/dataset/Downsampled_Projected_" << 3.5+j << "_" << 4.5+j << "/scan_" << i << ".pcd";
            cout << filename2.str() << endl;
            pcl_tool.savePCD(pcl_tool.projectPCD(cloud_in_f, 0.0, 0.0, 1.0, 0.0), filename2.str());
            //pcl::io::savePLYFile(filename2.str(), *pcl_tool.projectPCD(cloud_in_f, 0.0, 0.0, 1.0, 0.0));
            j+=1.5;
        }

        cloud.reset();
    }

    return 0;
*/



/*
    //Passthrough filter pcd z axis
    for(int scan_no=1; scan_no<=83; scan_no++){
        if(scan_no==44)
            continue;
        stringstream in_file;
        in_file << "/home/mustafasezer/dataset/Downsampled_Cuboids_Scaled/scan_" << scan_no << ".pcd";
        pcl::PointCloud<PointT>::Ptr cloud = pcl_tool.loadPCD(in_file.str());
        float upper_limit = 5.5;
        pcl::PointCloud<PointT>::Ptr output = pcl_tool.getSlice(cloud, 5.0, upper_limit);
        if(output->points.size()<=0){
            upper_limit += 0.5;
            output = pcl_tool.getSlice(cloud, 5.0, upper_limit);
        }

        //save it to file
        stringstream out_file;
        out_file << "/home/mustafasezer/dataset/Downsampled_Cuboids/objects_" << scan_no << ".npy";
        const unsigned int shape[] = {output->points[0].x,output->points[1].y,output->points[2].z};
        cnpy::npy_save(out_file.str(), output, shape, 3, "w");


        //pcl_tool.savePCD(output, scan_no);
    }
    return 0;
*/


/*pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_inp(new pcl::PointCloud<pcl::PointXYZRGB>);
if (pcl::io::loadPCDFile<pcl::PointXYZRGB> ("/home/mustafasezer/dataset/Downsampled_Cuboids_RGB/scan_1.pcd", *cloud_inp) == -1) //* load the file
{
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    return (-1);
}
pcl_tool.rgbVis(cloud_inp);*/


/*for(int i=0; i<43; i++){
    std::cout << i+1 << " to " << pcl_tool.transformations[i].parent_id << std::endl;
    std::cout << pcl_tool.transformations[i].T << std::endl << std::endl;
    //std::cout << "Parent: " << pcl_tool.transformations[i].is_parent << std::endl << std::endl;
}
return 0;*/



/*pcl::PointCloud<pcl::PointXYZ>::Ptr cloud__(new pcl::PointCloud<pcl::PointXYZ>);
cout << "Reading" << endl;
pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/mustafasezer/dataset/Original/scan_79.pcd", *cloud__);
for(float j=1.5; j<3.5; ){
    cout << "Getting Slice" << endl;
    pcl::PointCloud<PointT>::Ptr cloud_in_f__ = pcl_tool.getSlice(cloud__, 3.5+j, 4.5+j);
    stringstream filename2_;
    filename2_ << "/home/mustafasezer/dataset/Original_Projected_" << 3.5+j << "_" << 4.5+j << "/scan_" << i << ".pcd";
    cout << filename2_.str() << endl;
    pcl_tool.savePCD(pcl_tool.projectPCD(cloud_in_f__, 0.0, 0.0, 1.0, 0.0), filename2_.str());
    //pcl::io::savePLYFile(filename2.str(), *pcl_tool.projectPCD(cloud_in_f, 0.0, 0.0, 1.0, 0.0));
    j+=1.5;
}
continue;*/



/*//Check dataset
stringstream input_filename_;
input_filename_ << "/home/mustafasezer/Desktop/downsampled_scaled_rgbclouds/cloud_" << i << "_0.1.pcd";
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in_rgb_(new pcl::PointCloud<pcl::PointXYZRGB>);
if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (input_filename_.str(), *cloud_in_rgb_) == -1) //* load the file
{
    string temp(input_filename_.str());
    PCL_ERROR ("Couldn't read %s\n", temp.c_str());
    continue;
}
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in_f = pcl_tool.getSliceRGB(cloud_in_rgb_, 3, 5);
stringstream filename2;
filename2 << "/home/mustafasezer/dataset/Downsampled_scaled_Projected_" << 3.5 << "_" << 4.5 << "/scan_" << i << "_1.75.pcd";
cout << filename2.str() << endl;
pcl::PointCloud<PointT>::Ptr cloud_in_f_(new pcl::PointCloud<PointT>);
pcl::copyPointCloud(*cloud_in_f, *cloud_in_f_);
pcl_tool.savePCD(pcl_tool.projectPCD(cloud_in_f_, 0.0, 0.0, 1.0, 0.0), filename2.str());
for(float j=0; j<1.5; ){
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in_f = pcl_tool.getSliceRGB(cloud_in_rgb_, 3.5+j, 4.5+j);
    stringstream filename2;
    filename2 << "/home/mustafasezer/dataset/Downsampled_scaled_Projected_" << 3.5+j << "_" << 4.5+j << "/scan_" << i << ".pcd";
    cout << filename2.str() << endl;
    pcl::PointCloud<PointT>::Ptr cloud_in_f_(new pcl::PointCloud<PointT>);
    pcl::copyPointCloud(*cloud_in_f, *cloud_in_f_);
    pcl_tool.savePCD(pcl_tool.projectPCD(cloud_in_f_, 0.0, 0.0, 1.0, 0.0), filename2.str());
    j+=1.5;
}
cloud_in_rgb_.reset();
continue;
*/


//Get z slice of final clouds
/*std::vector<string> filenames;
filenames.push_back("/home/mustafasezer/scan_1_18.pcd");
filenames.push_back("/home/mustafasezer/scan_28_43.pcd");
filenames.push_back("/home/mustafasezer/scan_45_53.pcd");
filenames.push_back("/home/mustafasezer/scan_54_56.pcd");
filenames.push_back("/home/mustafasezer/scan_59_66.pcd");

for(int i=2; i<filenames.size(); i++){
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in_rgb__(new pcl::PointCloud<pcl::PointXYZRGB>);
    cout << "Reading " << filenames[i] << endl;
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (filenames[i], *cloud_in_rgb__) == -1) //* load the file
    {
        string temp(filenames[i]);
        PCL_ERROR ("Couldn't read %s\n", temp.c_str());
        //continue;
    }
    pcl::PointCloud<PointT>::Ptr cloud_in__(new pcl::PointCloud<PointT>);
    pcl::copyPointCloud(*cloud_in_rgb__, *cloud_in__);
    cout << "Getting z slice" << endl;
    pcl::PointCloud<PointT>::Ptr cloud_in_f__ = pcl_tool.getSlice(cloud_in__, 3.5, 4.5);
    string myfilename(filenames[i]);
    myfilename.insert(myfilename.find_last_of(".pcd")-3, "_projected");
    cout << "Saving " << myfilename << endl;
    pcl_tool.savePCD(pcl_tool.projectPCD(cloud_in_f__, 0.0, 0.0, 1.0, 0.0), myfilename);
}*/

//for(float j=1.5; j<3.5; ){
//    pcl::PointCloud<PointT>::Ptr cloud_in_f = pcl_tool.getSlice(cloud, 3.5+j, 4.5+j);
//    stringstream filename2;
//    filename2 << "home/mustafasezer/scan_1_18.pcd";
//   cout << myfilename << endl;
//   pcl_tool.savePCD(pcl_tool.projectPCD(cloud_in__, 0.0, 0.0, 1.0, 0.0), myfilename);
//    j+=1.5;
//}
