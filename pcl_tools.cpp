#include "pcl_tools.h"
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>

//#include <tf/transform_datatypes.h>
#include <boost/thread/thread.hpp>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/passthrough.h>

#include <pcl/common/common.h>

//Projection
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>

using namespace std;
struct timespec t1_, t2_;
double elapsed_time_;
volatile long long i_;

void start_timer(){
    clock_gettime(CLOCK_MONOTONIC,  &t1_);
}

void end_timer(string message = "Elapsed time:"){
    clock_gettime(CLOCK_MONOTONIC,  &t2_);
    elapsed_time_ = (t2_.tv_sec - t1_.tv_sec) + (double) (t2_.tv_nsec - t1_.tv_nsec) * 1e-9;
    std::cout << message << " " << elapsed_time_ << " seconds" << std::endl;
}


int file_exist (const char *filename)
{
    struct stat   buffer;
    return (stat (filename, &buffer) == 0);
}


std::string int2str(int num){
    stringstream strs;
    strs << num;
    return strs.str();
}


pcl_tools::pcl_tools()
{
    /*transformations[0].parent_id = -5;
    transformations[0].T.setIdentity();
    transformations[0].completed = true;
    transformations[0].is_parent = true;*/
    for(int i=0; i<83; i++){
        transformations[i].completed = false;
        transformations[i].is_parent = false;
        transformations[i].T.setIdentity();
    }
}

Eigen::Matrix3f pcl_tools::quaternion_to_rotation(float x, float y, float z, float w){
    Eigen::Quaternionf quat(w, x, y, z);
    return quat.toRotationMatrix();
    //std::cout << rotation << std::endl;
    /*Eigen::Matrix4f Trans; // Your Transformation Matrix
    Trans.setIdentity();   // Set to Identity to make bottom row of Matrix 0,0,0,1
    Trans.block<3,3>(0,0) = rotation;
    Trans.rightCols<1>() = translation;
    std::cout << Trans << std::endl;*/
}

Eigen::Matrix4f pcl_tools::createTransformationMatrix(Eigen::Matrix3f rotation, Eigen::Vector3f translation){
    Eigen::Matrix4f Trans; // Transformation Matrix
    Trans.setIdentity();   // Set to Identity to make bottom row of Matrix 0,0,0,1
    Trans.block<3,3>(0,0) = rotation;
    Trans.block<3,1>(0,3) = translation;
    return Trans;
}

Eigen::Matrix4f pcl_tools::getInitialGuess(int input, int target){
    string tline;
    fstream fid;
    fid.open("/home/mustafasezer/Initial Guesses.txt", ios::in);
    getline(fid, tline);
    stringstream transform_pair;
    transform_pair << "scan_" << input << " to " << "scan_" << target;
    while(tline.find(transform_pair.str())==string::npos && !fid.eof()){
        getline(fid, tline);
    }
    if(fid.eof()){
        std::cout << "Transformation from scan_" << input << " to scan_" << target << " not found" << std::endl;
        fid.close();
        return Eigen::Matrix4f::Zero();
    }
    getline(fid, tline);
    float t1, t2, t3;
    float x, y, z, w;
    if(tline.find("- Translation: [")!=string::npos && !fid.eof()){
        sscanf(tline.c_str(), "- Translation: [%f, %f, %f]", &t1, &t2, &t3);
        getline(fid, tline);
    }
    else{
        cout << "Erronous file format" << endl;
        fid.close();
        return Eigen::Matrix4f::Zero();
    }
    if(tline.find("- Rotation: in Quaternion [")!=string::npos && !fid.eof()){
        sscanf(tline.c_str(), "- Rotation: in Quaternion [%f, %f, %f, %f]", &x, &y, &z, &w);
        getline(fid, tline);
    }
    else{
        cout << "Erronous file format" << endl;
        fid.close();
        return Eigen::Matrix4f::Zero();
    }
    Eigen::Vector3f translation_vector(t1, t2, t3);
    Eigen::Matrix3f rotation_matrix = quaternion_to_rotation(x, y, z, w);
    return createTransformationMatrix(rotation_matrix, translation_vector);
    //cout << "Translation: " << t1 << " " << t2 << " " << t3 << endl;
    //cout << "Rotation: " << x << " " << y << " " << z << " " << w << endl;
    //cout << endl;
    fid.close();
}


int pcl_tools::getInitialGuesses(){
    string tline;
    fstream fid;
    int input, target;
    fid.open("/home/mustafasezer/Initial Guesses.txt", ios::in);
    getline(fid, tline);
    while(!fid.eof()){
        while(tline.find("scan_")==string::npos && !fid.eof()){
            getline(fid, tline);
        }
        if(fid.eof()){
            std::cout << "Reached the end of initial guess file" << std::endl;
            fid.close();
            return 1;
        }
        sscanf(tline.c_str(), "scan_%d to scan_%d", &input, &target);
        getline(fid, tline);
        float t1, t2, t3;
        float x, y, z, w;
        if(tline.find("- Translation: [")!=string::npos && !fid.eof()){
            sscanf(tline.c_str(), "- Translation: [%f, %f, %f]", &t1, &t2, &t3);
            getline(fid, tline);
        }
        else{
            cout << "Erronous file format" << endl;
            fid.close();
            return 1;
        }
        if(tline.find("- Rotation: in Quaternion [")!=string::npos && !fid.eof()){
            sscanf(tline.c_str(), "- Rotation: in Quaternion [%f, %f, %f, %f]", &x, &y, &z, &w);
            getline(fid, tline);
        }
        else{
            cout << "Erronous file format" << endl;
            fid.close();
            return 1;
        }
        if(input==target){
            transformations[input-1].init_guess.setIdentity();
            transformations[input-1].completed = true;
            transformations[input-1].is_parent = true;
            transformations[input-1].parent_id = target;
            continue;
        }
        Eigen::Vector3f translation_vector(t1, t2, t3);
        Eigen::Matrix3f rotation_matrix = quaternion_to_rotation(x, y, z, w);
        transformations[input-1].parent_id = target;
        transformations[input-1].init_guess = createTransformationMatrix(rotation_matrix, translation_vector);
        transformations[input-1].is_parent = false;
    }
    fid.close();
    return 0;
}



Eigen::Matrix4f pcl_tools::getTransformation(int input, int target){
    string tline;
    fstream fid;
    fid.open("/home/mustafasezer/icp_result.txt", ios::in);
    getline(fid, tline);
    stringstream transform_pair;
    transform_pair << "scan_" << input << " to " << "scan_" << target;
    while(tline.find(transform_pair.str())==string::npos && !fid.eof()){
        getline(fid, tline);
    }
    if(fid.eof()){
        std::cout << "Transformation from scan_" << input << " to scan_" << target << " not found" << std::endl;
        fid.close();
        return Eigen::Matrix4f::Zero();
    }
    Eigen::Matrix4f transformation;

    for(int i=0; i<4; i++){
        getline(fid, tline);
        sscanf(tline.c_str(), "%f %f %f %f", &transformation(i,0), &transformation(i,1), &transformation(i,2), &transformation(i,3));
    }
    fid.close();
    return transformation;
}





//int pcl_tools::transform_pcd(int argc, char** argv)
//{
//    // Fetch point cloud filename in arguments | Works with PCD and PLY files
//    std::vector<int> filenames;
//    bool file_is_pcd = false;

//    filenames = pcl::console::parse_file_extension_argument (argc, argv, ".ply");

//    if (filenames.size () != 1)  {
//        filenames = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");

//        if (filenames.size () != 1) {
//            showHelp (argv[0]);
//            return -1;
//        } else {
//            file_is_pcd = true;
//        }
//    }

//    // Load file | Works with PCD and PLY files
//    pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud (new pcl::PointCloud<pcl::PointXYZ> ());

//    if (file_is_pcd) {
//        if (pcl::io::loadPCDFile (argv[filenames[0]], *source_cloud) < 0)  {
//            std::cout << "Error loading point cloud " << argv[filenames[0]] << std::endl << std::endl;
//            showHelp (argv[0]);
//            return -1;
//        }
//    } else {
//        if (pcl::io::loadPLYFile (argv[filenames[0]], *source_cloud) < 0)  {
//            std::cout << "Error loading point cloud " << argv[filenames[0]] << std::endl << std::endl;
//            showHelp (argv[0]);
//            return -1;
//        }
//    }

//    /* Reminder: how transformation matrices work :

//           |-------> This column is the translation
//    | 1 0 0 x |  \
//    | 0 1 0 y |   }-> The identity 3x3 matrix (no rotation) on the left
//    | 0 0 1 z |  /
//    | 0 0 0 1 |    -> We do not use this line (and it has to stay 0,0,0,1)

//    METHOD #1: Using a Matrix4f
//    This is the "manual" method, perfect to understand but error prone !
//  */
//    Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();

//    // Define a rotation matrix (see https://en.wikipedia.org/wiki/Rotation_matrix)
//    float theta = M_PI/4; // The angle of rotation in radians
//    theta = 0;
//    //transform_1 (0,0) = cos (theta);
//    //transform_1 (0,1) = -sin(theta);
//    //transform_1 (1,0) = sin (theta);
//    //transform_1 (1,1) = cos (theta);


//    transform_1 (0,0) = 0.99791;
//    transform_1 (0,1) = 0.0564487;
//    transform_1 (0,2) = -0.031504;
//    transform_1 (0,3) = -0.311339;
//    transform_1 (1,0) = -0.0564152;
//    transform_1 (1,1) = 0.998406;
//    transform_1 (1,2) = 0.00194948;
//    transform_1 (1,3) = 1.91249;
//    transform_1 (2,0) = 0.0315638;
//    transform_1 (2,1) = -0.000168093;
//    transform_1 (2,2) = 0.999503;
//    transform_1 (2,3) = -1.2956;
//    transform_1 (3,0) = 0;
//    transform_1 (3,1) = 0;
//    transform_1 (3,2) = 0;
//    transform_1 (3,3) = 1;


//    //    (row, column)

//    // Define a translation of 2.5 meters on the x axis.
//    //transform_1 (0,3) = 2.5;

//    // Print the transformation
//    printf ("Method #1: using a Matrix4f\n");
//    std::cout << transform_1 << std::endl;

//    /*  METHOD #2: Using a Affine3f
//    This method is easier and less error prone
//  */
//    Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();

//    // Define a translation of 2.5 meters on the x axis.
//    transform_2.translation() << 0.0, 0.0, 0.0;

//    // The same rotation matrix as before; tetha radians arround Z axis
//    transform_2.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitZ()));

//    // Print the transformation
//    printf ("\nMethod #2: using an Affine3f\n");
//    std::cout << transform_2.matrix() << std::endl;

//    // Executing the transformation
//    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
//    // You can either apply transform_1 or transform_2; they are the same
//    pcl::transformPointCloud (*source_cloud, *transformed_cloud, transform_1);




//    //Read scan_1.pcd
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);

//    if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/mustafasezer/dataset/Downsampled/scan_1.pcd", *cloud_in) == -1) //* load the file
//    {
//        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
//        return (-1);
//    }






//    // Visualization
//    printf(  "\nPoint cloud colors :  white  = original point cloud\n"
//             "                        red  = transformed point cloud\n");
//    pcl::visualization::PCLVisualizer viewer ("Matrix transformation example");

//    // Define R,G,B colors for the point cloud
//    //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler (source_cloud, 255, 255, 255);
//    // We add the point cloud to the viewer and pass the color handler
//    //viewer.addPointCloud (source_cloud, source_cloud_color_handler, "original_cloud");

//    // Define R,G,B colors for the point cloud
//    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_in_color_handler (cloud_in, 255, 255, 255);
//    // We add the point cloud to the viewer and pass the color handler
//    viewer.addPointCloud (cloud_in, cloud_in_color_handler, "original_cloud");

//    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_cloud_color_handler (transformed_cloud, 230, 20, 20); // Red
//    viewer.addPointCloud (transformed_cloud, transformed_cloud_color_handler, "transformed_cloud");

//    //viewer.addCoordinateSystem (1.0, "cloud", 0);
//    viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
//    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original_cloud");
//    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformed_cloud");
//    //viewer.setPosition(800, 400); // Setting visualiser window position

//    while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
//        viewer.spinOnce ();
//    }

//    return 0;
//}






pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_tools::transform_pcd(pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud, Eigen::Matrix4f transform_matrix)
{
    //    // Fetch point cloud filename in arguments | Works with PCD and PLY files
    //    std::vector<int> filenames;
    //    bool file_is_pcd = false;

    //    filenames = pcl::console::parse_file_extension_argument (argc, argv, ".ply");

    //    if (filenames.size () != 1)  {
    //        filenames = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");

    //        if (filenames.size () != 1) {
    //            showHelp (argv[0]);
    //            return -1;
    //        } else {
    //            file_is_pcd = true;
    //        }
    //    }

    //    // Load file | Works with PCD and PLY files
    //    pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud (new pcl::PointCloud<pcl::PointXYZ> ());

    //    if (file_is_pcd) {
    //        if (pcl::io::loadPCDFile (argv[filenames[0]], *source_cloud) < 0)  {
    //            std::cout << "Error loading point cloud " << argv[filenames[0]] << std::endl << std::endl;
    //            showHelp (argv[0]);
    //            return -1;
    //        }
    //    } else {
    //        if (pcl::io::loadPLYFile (argv[filenames[0]], *source_cloud) < 0)  {
    //            std::cout << "Error loading point cloud " << argv[filenames[0]] << std::endl << std::endl;
    //            showHelp (argv[0]);
    //            return -1;
    //        }
    //    }

    // Executing the transformation
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    // You can either apply transform_1 or transform_2; they are the same
    pcl::transformPointCloud (*source_cloud, *transformed_cloud, transform_matrix);

    return transformed_cloud;




    //    //Read scan_1.pcd
    //    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_1 = loadPCD("/home/mustafasezer/dataset/Downsampled_Cuboids/scan_3.pcd");
    //    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2 = loadPCD("/home/mustafasezer/dataset/Downsampled_Cuboids/scan_4.pcd");

    //    // Visualization
    //    printf(  "\nPoint cloud colors :  white  = original point cloud\n"
    //             "                        red  = transformed point cloud\n");
    //    pcl::visualization::PCLVisualizer viewer ("Matrix transformation example");

    //    // Define R,G,B colors for the point cloud
    //    //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler (source_cloud, 255, 255, 255);
    //    // We add the point cloud to the viewer and pass the color handler
    //    //viewer.addPointCloud (source_cloud, source_cloud_color_handler, "original_cloud");

    //    // Define R,G,B colors for the point cloud
    //    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_1_color_handler (cloud_1, 255, 255, 255);
    //    // We add the point cloud to the viewer and pass the color handler
    //    viewer.addPointCloud (cloud_1, cloud_1_color_handler, "cloud_1");
    //    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_2_color_handler (cloud_2, 0, 255, 0);
    //    viewer.addPointCloud (cloud_2, cloud_2_color_handler, "cloud_2");

    //    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_cloud_color_handler (transformed_cloud, 230, 20, 20); // Red
    //    viewer.addPointCloud (transformed_cloud, transformed_cloud_color_handler, "transformed_cloud");

    //    //viewer.addCoordinateSystem (1.0, "cloud", 0);
    //    viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
    //    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud_1");
    //    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud_2");
    //    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformed_cloud");
    //    //viewer.setPosition(800, 400); // Setting visualiser window position

    //    while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
    //        viewer.spinOnce ();
    //    }

    //    return 0;
}



void pcl_tools::rgbVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{

  pcl::visualization::PCLVisualizer viewer ("Matrix transformation example");

  viewer.setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer.addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");

  while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
      viewer.spinOnce (100);
      boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }

}


void pcl_tools::viewPCD(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, string name, int r, int g, int b){
    //*viewer= pcl::visualization::PCLVisualizer("Matrix transformation example");

    pcl::visualization::PCLVisualizer viewer ("Matrix transformation example");

    // Define R,G,B colors for the point cloud
    //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler (source_cloud, 255, 255, 255);
    // We add the point cloud to the viewer and pass the color handler
    //viewer.addPointCloud (source_cloud, source_cloud_color_handler, "original_cloud");

    // Define R,G,B colors for the point cloud
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color_handler(cloud, r, g, b);
    // We add the point cloud to the viewer and pass the color handler
    viewer.addPointCloud (cloud, cloud_color_handler, name);

    //viewer.addCoordinateSystem (1.0, name, 0);
    viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, name);
    //viewer.setPosition(800, 400); // Setting visualiser window position

    while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
        viewer.spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
}

void pcl_tools::viewICPResult(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_targ, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_aligned){
    //*viewer= pcl::visualization::PCLVisualizer("Matrix transformation example");

    pcl::visualization::PCLVisualizer viewer ("Matrix transformation example");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_in_color_handler(cloud_in, 255, 255, 255);
    viewer.addPointCloud (cloud_in, cloud_in_color_handler, "cloud_in");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_targ_color_handler(cloud_targ, 0, 0, 255);
    viewer.addPointCloud (cloud_targ, cloud_targ_color_handler, "cloud_targ");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_aligned_color_handler(cloud_aligned, 255, 0, 0);
    viewer.addPointCloud (cloud_aligned, cloud_aligned_color_handler, "cloud_aligned");

    //viewer.addCoordinateSystem (1.0, name, 0);
    viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud_in");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud_targ");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud_aligned");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud_transformed");
    //viewer.setPosition(800, 400); // Setting visualiser window position

    while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
        viewer.spinOnce ();
    }
}


Eigen::Matrix4f pcl_tools::apply_icp(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out, bool viewResult){
    start_timer();
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputCloud(cloud_in);
    icp.setInputTarget(cloud_out);

    icp.setMaxCorrespondenceDistance (1.5);
    // Set the maximum number of iterations (criterion 1)
    icp.setMaximumIterations (2000);
    // Set the transformation epsilon (criterion 2)
    icp.setTransformationEpsilon (1e-8);
    // Set the euclidean distance difference epsilon (criterion 3)
    icp.setEuclideanFitnessEpsilon(0.01);

    pcl::PointCloud<pcl::PointXYZ> Final;
    icp.align(Final);
    std::cout << "ICP has converged:" << icp.hasConverged() << " score: " <<
                 icp.getFitnessScore() << std::endl;
    Eigen::Matrix4f transformation = icp.getFinalTransformation();
    std::cout << transformation << std::endl;

    end_timer("ICP has converged in:");

    std::cout << "Iterations: " << icp.nr_iterations_ << std::endl;

    if(viewResult){
        viewICPResult(cloud_in, cloud_out, Final.makeShared());
    }
    return transformation;
}

int pcl_tools::apply_icp(string path_in, string path_out){
    //Read input cloud
    std::cout << "Reading input cloud" << std::endl;
    start_timer();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);

    if (pcl::io::loadPCDFile<pcl::PointXYZ> (path_in, *cloud_in) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
        return (-1);
    }
    end_timer("ICP: Input cloud loaded in:");

    //Read output cloud
    std::cout << "Reading output cloud" << std::endl;
    start_timer();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);

    if (pcl::io::loadPCDFile<pcl::PointXYZ> (path_out, *cloud_out) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
        return (-1);
    }
    end_timer("ICP: Output cloud loaded in:");


    start_timer();
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputCloud(cloud_in);
    icp.setInputTarget(cloud_out);
    pcl::PointCloud<pcl::PointXYZ> Final;
    icp.align(Final);
    std::cout << "ICP has converged:" << icp.hasConverged() << " score: " <<
                 icp.getFitnessScore() << std::endl;
    std::cout << icp.getFinalTransformation() << std::endl;
    end_timer("ICP has converged in:");
    return 0;
}

Eigen::Matrix4f pcl_tools::apply_ndt(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud, Eigen::Matrix4f init_guess){


    //start_timer();
    // Initializing Normal Distributions Transform (NDT).
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;

    // Setting scale dependent NDT parameters
    // Setting minimum transformation difference for termination condition.
    ndt.setTransformationEpsilon (0.00001);
    // Setting maximum step size for More-Thuente line search.
    ndt.setStepSize (0.1);
    //Setting Resolution of NDT grid structure (VoxelGridCovariance).
    ndt.setResolution (8.0);

    // Setting max number of registration iterations.
    ndt.setMaximumIterations (35);

    // Setting point cloud to be aligned.
    ndt.setInputSource (cloud_in);
    // Setting point cloud to be aligned to.
    ndt.setInputTarget (target_cloud);

    // Set initial alignment estimate found using robot odometry.
    /*Eigen::AngleAxisf init_rotation (0.6931, Eigen::Vector3f::UnitZ ());
    Eigen::Translation3f init_translation (1.79387, 0.720047, 0);
    Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix ();*/

    // Calculating required rigid transform to align the input cloud to the target cloud.
    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    ndt.align(*output_cloud, init_guess);

    std::cout << "Normal Distributions Transform has converged:" << ndt.hasConverged ()
              << " score: " << ndt.getFitnessScore () << std::endl;

    std::cout << ndt.getFinalTransformation () << std::endl;
    //end_timer("NDT has converged in:");

    // Transforming unfiltered, input cloud using found transform.
    pcl::transformPointCloud (*cloud_in, *output_cloud, ndt.getFinalTransformation ());

    // Initializing point cloud visualizer
    boost::shared_ptr<pcl::visualization::PCLVisualizer>
            viewer_final (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer_final->setBackgroundColor (0, 0, 0);

    std::cout << "Starting NDT 4" << std::endl;

    // Coloring and visualizing target cloud (red).
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
            target_color (target_cloud, 255, 0, 0);
    viewer_final->addPointCloud<pcl::PointXYZ> (target_cloud, target_color, "target cloud");
    viewer_final->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                    1, "target cloud");

    // Coloring and visualizing transformed input cloud (green).
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
            output_color (output_cloud, 0, 255, 0);
    viewer_final->addPointCloud<pcl::PointXYZ> (output_cloud, output_color, "output cloud");
    viewer_final->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                    1, "output cloud");

    // Starting visualizer
    //viewer_final->addCoordinateSystem (1.0, "global");
    viewer_final->initCameraParameters ();

    // Wait until visualizer window is closed.
    while (!viewer_final->wasStopped ())
    {
        viewer_final->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
}

pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_tools::loadPCD(string path){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

    if (pcl::io::loadPCDFile<pcl::PointXYZ> (path, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
        return (cloud);
    }
    else{
        return cloud;
    }
}



pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_tools::getSlice(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float z1, float z2, string field_name){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName (field_name);
    pass.setFilterLimits (z1, z2);
    //pass.setFilterLimitsNegative (true);
    pass.filter (*cloud_filtered);
    return cloud_filtered;
}

void pcl_tools::savePCD(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int scan_no){
    cloud->width = 1;
    cloud->height = cloud->points.size();
    stringstream filename;
    filename << "/home/mustafasezer/dataset/Cuboid_Slice/scan_" << scan_no << ".pcd";
    pcl::io::savePCDFileASCII (filename.str(), *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to pcd." << std::endl;
    std::cerr << "Cuboid size" << cloud->points.size() << std::endl;
}

void pcl_tools::savePCD(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, string path){
    cloud->width = 1;
    cloud->height = cloud->points.size();
    pcl::io::savePCDFileASCII (path, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to pcd." << std::endl;
    std::cerr << "Cuboid size" << cloud->points.size() << std::endl;
}


void pcl_tools::getPCDStatistics(string dir, int low_ind, int up_ind){
    for(int i=low_ind; i<=up_ind; i++){
        string filename = dir;
        filename.append("/scan_");
        string num = int2str(i);
        filename.append(num.c_str());
        filename.append(".pcd");
        std::cout << "Loading " << filename << std::endl;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = loadPCD(filename);
        pcl::PointXYZ minPt, maxPt;
        pcl::getMinMax3D (*cloud, minPt, maxPt);
        std::cout << "Min: " << minPt.x << " " << minPt.y << " " << minPt.z << std::endl;
        std::cout << "Max: " << maxPt.x << " " << maxPt.y << " " << maxPt.z << std::endl;
    }
}


pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_tools::projectPCD(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float a, float b, float c, float d){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZ>);

    // Create a set of planar coefficients
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    coefficients->values.resize (4);
    coefficients->values[0] = a;
    coefficients->values[1] = b;
    coefficients->values[2] = c;
    coefficients->values[3] = d;

    // Create the filtering object
    pcl::ProjectInliers<pcl::PointXYZ> proj;
    proj.setModelType (pcl::SACMODEL_PLANE);
    proj.setInputCloud (cloud);
    proj.setModelCoefficients (coefficients);
    proj.filter (*cloud_projected);

    return cloud_projected;
}


int pcl_tools::scale_pcd(float scaling_factor){
    //struct timespec t1, t2;
    //double elapsed_time;
    //volatile long long i;

    for(int i=65; i<=83; i++){
        /*stringstream check_label_filename;
        check_label_filename.clear();
        check_label_filename << "/home/mustafasezer/dataset/Original/final_labeled_images/scan_" << i << "_label.png";

        string check_filename = check_label_filename.str();
        if (!file_exist (check_filename.c_str()))
        {
            std::cerr << check_label_filename.str() << "does not exist" << std::endl;
            continue;
        }*/

        stringstream filename;
        filename.clear();
        filename << "//home/mustafasezer/dataset/Downsampled_Cuboids/scan_" << i << ".pcd";
        cout << filename.str() << endl;
        string strfilename = filename.str();
        if (!file_exist (strfilename.c_str()))
        {
            std::cerr << filename << "does not exist" << std::endl;
            continue;
        }


        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = loadPCD(filename.str());

        std::cerr << "PointCloud read: " << cloud->width * cloud->height
                  << " data points (" << pcl::getFieldsList (*cloud) << ")."<< endl;

        if(cloud->points.size() == 0){
            std::cout << filename << " empty\n";
            continue;
        }

        for (size_t j = 0; j < cloud->points.size (); ++j){
            cloud->points[j].x = scaling_factor*cloud->points[j].x;
            cloud->points[j].y = scaling_factor*cloud->points[j].y;
            cloud->points[j].z = scaling_factor*cloud->points[j].z;
        }

        stringstream filename2;
        filename2 << "/home/mustafasezer/dataset/Downsampled_Cuboids_Scaled/scan_" << i << ".pcd";
        cout << filename2.str() << endl;
        savePCD(cloud, filename2.str());

        cloud.reset();
    }

    return (0);
}



int pcl_tools::downsample_pcd(){
    //struct timespec t1, t2;
    //double elapsed_time;
    //volatile long long i;

    for(int i=68; i<=83; i++){

        stringstream check_label_filename;
        check_label_filename.clear();
        check_label_filename << "/home/mustafasezer/dataset/Original/final_labeled_images/scan_" << i << "_label.png";

        string check_filename = check_label_filename.str();
        if (!file_exist (check_filename.c_str()))
        {
            std::cerr << check_label_filename.str() << "does not exist" << std::endl;
            continue;
        }



        //clock_gettime(CLOCK_MONOTONIC,  &t1);

        pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2 ());
        pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());

        // Fill in the cloud data
        pcl::PCDReader reader;
        // Replace the path below with the path where you saved your file
        stringstream filename;
        filename.clear();
        filename << "/home/mustafasezer/dataset/Original/scan_" << i << ".pcd";
        cout << filename.str() << endl;
        reader.read (filename.str(), *cloud);

        std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height
                  << " data points (" << pcl::getFieldsList (*cloud) << ")."<< endl;

        // Create the filtering object
        pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
        sor.setInputCloud (cloud);
        sor.setLeafSize (0.2f, 0.2f, 0.2f);
        sor.filter (*cloud_filtered);

        std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height
                  << " data points (" << pcl::getFieldsList (*cloud_filtered) << ")." << endl;

        pcl::PCDWriter writer;
        stringstream filename2;
        filename2 << "/home/mustafasezer/dataset/Downsampled_new/scan_" << i << ".pcd";
        cout << filename2.str() << endl;
        writer.write (filename2.str(), *cloud_filtered,
                      Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), false);

        //clock_gettime(CLOCK_MONOTONIC,  &t2);
        //elapsed_time = (t2.tv_sec - t1.tv_sec) + (double) (t2.tv_nsec - t1.tv_nsec) * 1e-9;

        //std::cout << "Point cloud subsampled in " << elapsed_time << " seconds" << std::endl;

        cloud.reset();
        cloud_filtered.reset();
    }

    return (0);
}
