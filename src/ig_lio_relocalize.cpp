
// #include <geometry_msgs/PoseStamped.h>
// #include <nav_msgs/Odometry.h>
// #include <nav_msgs/Path.h>
// #include <ros/package.h>
// #include <ros/ros.h>
// #include <sensor_msgs/Imu.h>
// #include <tf/transform_broadcaster.h>

//c++
#include <boost/filesystem.hpp>
#include <csignal>
#include <fstream>
#include <mutex>
#include <thread>
#include <atomic>
#include <chrono>


//ig lio
#include "ig_lio/lio.h"
#include "ig_lio/logger.hpp"
#include "ig_lio/pointcloud_preprocess.h"
#include "ig_lio/timer.h"

//ros2 
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include "techshare_ros_pkg2/msg/cloud_info.hpp"
#include "techshare_ros_pkg2/msg/score_info.hpp"
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>  // For NormalDistributionsTransform
#include <pcl/registration/icp.h>  // For IterativeClosestPoint
// using symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)
// using symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
// using symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)
// using symbol_shorthand::G; // GPS pose

namespace fs = boost::filesystem;
constexpr double kAccScale = 9.80665;
// Global flag for clean shutdown
std::atomic<bool> FLAG_EXIT(false);
using std::placeholders::_1;


/*
    * A point cloud type that has 6D pose info ([x,y,z,roll,pitch,yaw] intensity is time stamp)
    */
struct PointXYZIRPYT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY                  // preferred way of adding a XYZ+padding
    float roll;
    float pitch;
    float yaw;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
} EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment

struct TrasformPose
{
    double x;
    double y;
    double z;
    double roll;
    double yaw;
    double pitch;
};

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRPYT,
(float, x, x) (float, y, y)
(float, z, z) (float, intensity, intensity)
(float, roll, roll) (float, pitch, pitch) (float, yaw, yaw)
(double, time, time))


typedef PointXYZIRPYT  PointTypePose;


class IG_LIO_RELOCALIZATION_NODE : public rclcpp::Node {
public:
  IG_LIO_RELOCALIZATION_NODE(const rclcpp::NodeOptions & options,std::string package_path) : Node("ig_lio_node"){
    // Setup signal handler
    // signal(SIGINT, IG_LIO_RELOCALIZATION_NODE::SigHandle);
    DeclareParams();
    GetParams();
    // package_path_ = package_path;

    //Initialize variables 
    Initialize();
    //Register pub/sub
    allocateMemory();
    Topics();
    cloudGlobalLoad();
    // Start the loop in a separate thread
    // processing_thread_ = std::thread(&IG_LIO_RELOCALIZATION_NODE::processingLoop, this);
  }

  ~IG_LIO_RELOCALIZATION_NODE() {
    // Set exit flag and join the thread on destruction
    // if (processing_thread_.joinable()) {
    //     processing_thread_.join();
    // }
  }


  void reLocalizeThread()
  {
      while (rclcpp::ok())
      // while (switchflag == false)
      {

          if(switchflag == true){
              sleep(1.0);
          }
          //avoid ICP using the same initial guess for many times
          else if((initializedFlag == NonInitialized)&&(initialized_Flag == true))
          {
              // InitializedType = Unknow;
              ICPLocalizeInitialize();
          }
          else if(initializedFlag == Initializing)
          {
              std::cout << "Offer A New Guess Please " << std::endl;//do nothing, wait for a new initial guess
              sleep(1.0);
          }
          else if(initializedFlag == Initialized){
              sleep(3.0);
              RCLCPP_INFO(this->get_logger(),"tyring to do icp");
              ICPscanMatchGlobal();
          }
        // Add sleep here to avoid high CPU usage
        sleep(0.1);
      }
  }

private:

  void DeclareParams(){
      declare_parameter("switchflag", false);
      declare_parameter("historyKeyframeFitnessScore", 0.3);
      declare_parameter("useImuHeadingInitialization", false);
      declare_parameter("mappingProcessInterval", 0.15);
      declare_parameter("map/map_name", "test");
      declare_parameter("map/map_location", "/Downloads/LOAM/");
      declare_parameter("odometryKeyFrameLeafSize", 0.2);
      declare_parameter("odometryCurrentScanLeafSize", 0.4);
      declare_parameter("surroundingKeyframeDensity", 1.0);
      declare_parameter("numberOfCores", 4);
      declare_parameter("keyframeFeatureMinValidNum", 100);
      declare_parameter("currentscanFeatureMinValidNum", 1000);
  }

  void GetParams(){
      get_parameter("switchflag", switchflag);
      get_parameter("historyKeyframeFitnessScore", historyKeyframeFitnessScore);  
      get_parameter("useImuHeadingInitialization", useImuHeadingInitialization);
      get_parameter("mappingProcessInterval", mappingProcessInterval);
      get_parameter("map/map_name", map_name);
      get_parameter("map/map_location", map_location);
      get_parameter("odometryKeyFrameLeafSize", odometryKeyFrameLeafSize);
      get_parameter("odometryCurrentScanLeafSize", odometryCurrentScanLeafSize);
      get_parameter("surroundingKeyframeDensity", surroundingKeyframeDensity);    
      get_parameter("numberOfCores", numberOfCores);    
      get_parameter("keyframeFeatureMinValidNum", keyframeFeatureMinValidNum);
      get_parameter("currentscanFeatureMinValidNum", currentscanFeatureMinValidNum);

 
  }
  void Initialize(){

  }
  void Topics(){
        subLaserCloudInfo = this->create_subscription<techshare_ros_pkg2::msg::CloudInfo>("halna/feature/cloud_info", 500, 
                                            std::bind(&IG_LIO_RELOCALIZATION_NODE::laserCloudInfoHandler,this,std::placeholders::_1));
        subIniPoseFromRviz = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 8, 
                                      std::bind(&IG_LIO_RELOCALIZATION_NODE::initialpose_callback,this,std::placeholders::_1));
        subRelocationOdom = this->create_subscription<nav_msgs::msg::Odometry>("halna/relocation/robot_to_map", 10, 
                                      std::bind(&IG_LIO_RELOCALIZATION_NODE::relocation_odom_callback,this,std::placeholders::_1));
        
        rclcpp::QoS qos_profile(10);
        qos_profile.durability(rclcpp::DurabilityPolicy::TransientLocal);
        rclcpp::PublisherOptionsWithAllocator<std::allocator<void>> pub_options;
        pub_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;
        globalMap_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("global_map", qos_profile, pub_options);
        pubOdomAftMappedROS = this->create_publisher<nav_msgs::msg::Odometry> ("halna/test/odometry", 1);
        pubPath = this->create_publisher<nav_msgs::msg::Path>("halna/mapping/path", 1);
        pubOdomToMapPose = this->create_publisher<geometry_msgs::msg::PoseStamped>("halna/mapping/pose_odomTo_map", 1);
        pudInitialFlag = this->create_publisher<std_msgs::msg::Bool>("lio/relocalization/InitialFlag",1);
        temp_pubOdomToMapPose = this->create_publisher<geometry_msgs::msg::PoseStamped>("halna/mapping/temp_pose_odomTo_map", 1);
        pubFitnessScore = this->create_publisher<techshare_ros_pkg2::msg::ScoreInfo>("halna/mapping/FitnessScore", 1);
        pub_temp_score = this->create_publisher<std_msgs::msg::Float64>("temp_score", 1);
        pubwincloudKeyPoses6D = this->create_publisher<geometry_msgs::msg::PoseStamped>("halna/mapping/win_cloudKey_Poses6D", 1);
        pubHistoryKeyFrames = this->create_publisher<sensor_msgs::msg::PointCloud2>("halna/mapping/icp_loop_closure_history_cloud", 1);
        pubIcpKeyFrames = this->create_publisher<sensor_msgs::msg::PointCloud2>("halna/mapping/icp_loop_closure_corrected_cloud", 1);
        pubRecentKeyFrames = this->create_publisher<sensor_msgs::msg::PointCloud2>("halna/mapping/map_local", 1);
        pubRecentKeyFrame = this->create_publisher<sensor_msgs::msg::PointCloud2>("halna/mapping/cloud_registered", 1);
        pubCloudRegisteredRaw = this->create_publisher<sensor_msgs::msg::PointCloud2>("halna/mapping/cloud_registered_raw", 1);
        pub_pcd_config = this->create_publisher<std_msgs::msg::Float64MultiArray>("halna/pcdmap/config", 1);

        
  }

  void ICPLocalizeInitialize(){
      CloudPtr laserCloudIn(new CloudType());
      mtx_general.lock();
      *laserCloudIn += *cloudScanForInitialize; //must be the keyframe from ig_lio
      mtx_general.unlock();

      if(laserCloudIn->points.size() == 0){
          RCLCPP_INFO(this->get_logger(), "the cloud is empty");
          return;
      }else{
          RCLCPP_INFO(this->get_logger(), "trying to match a keyframe with a map");       
      }
      pcl::NormalDistributionsTransform<PointType, PointType> ndt;
      ndt.setTransformationEpsilon(0.01);
      ndt.setResolution(1.0);

      pcl::IterativeClosestPoint<PointType, PointType> icp;
      icp.setMaxCorrespondenceDistance(100);
      icp.setMaximumIterations(100);
      icp.setTransformationEpsilon(1e-6);
      icp.setEuclideanFitnessEpsilon(1e-6);
      icp.setRANSACIterations(0);
      ndt.setInputSource(laserCloudIn);
      ndt.setInputTarget(cloudGlobalMapDS);
      CloudPtr unused_result_0(new CloudType());
      PointTypePose thisPose6DInWorld = trans2PointTypePose(transformInTheWorld);
      Eigen::Affine3f T_thisPose6DInWorld = pclPointToAffine3f(thisPose6DInWorld);
      ndt.align(*unused_result_0, T_thisPose6DInWorld.matrix());
      //use the outcome of ndt as the initial guess for ICP
      icp.setInputSource(laserCloudIn);
      icp.setInputTarget(cloudGlobalMapDS);
      CloudPtr unused_result(new CloudType());
      icp.align(*unused_result, ndt.getFinalTransformation());
      std::cout << "the pose before initializing is: x" << transformInTheWorld[3] << " y" << transformInTheWorld[4]
                << " z" << transformInTheWorld[5] <<std::endl;
      std::cout << "the pose in odom before initializing is: x" << tranformOdomToWorld[3] << " y" << tranformOdomToWorld[4]
                << " z" << tranformOdomToWorld[5] <<std::endl;
      std::cout << "the icp score in initializing process is: " << icp.getFitnessScore() << std::endl;
      std::cout << "the pose after initializing process is: "<< icp.getFinalTransformation() << std::endl;
      PointTypePose thisPose6DInOdom = trans2PointTypePose(transformTobeMapped);
	    std::cout<< "transformTobeMapped X_Y_Z: " << transformTobeMapped[3] << " " << transformTobeMapped[4] << " " << transformTobeMapped[5] << std::endl;
      Eigen::Affine3f T_thisPose6DInOdom = pclPointToAffine3f(thisPose6DInOdom);
      Eigen::Affine3f T_thisPose6DInMap;
      T_thisPose6DInMap = icp.getFinalTransformation();
      float x_g, y_g, z_g, R_g, P_g, Y_g;
      pcl::getTranslationAndEulerAngles (T_thisPose6DInMap, x_g, y_g, z_g, R_g, P_g, Y_g);
      transformInTheWorld[0] = R_g;
      transformInTheWorld[1] = P_g;
      transformInTheWorld[2] = Y_g;
      transformInTheWorld[3] = x_g;
      transformInTheWorld[4] = y_g;
      transformInTheWorld[5] = z_g;
      Eigen::Affine3f transOdomToMap = T_thisPose6DInMap * T_thisPose6DInOdom.inverse();
      float deltax, deltay, deltaz, deltaR, deltaP, deltaY;
      pcl::getTranslationAndEulerAngles (transOdomToMap, deltax, deltay, deltaz, deltaR, deltaP, deltaY);
      mtxtranformOdomToWorld.lock();
          //renew tranformOdomToWorld
      tranformOdomToWorld[0] = deltaR;
      tranformOdomToWorld[1] = deltaP;
      tranformOdomToWorld[2] = deltaY;
      tranformOdomToWorld[3] = deltax;
      tranformOdomToWorld[4] = deltay;
      tranformOdomToWorld[5] = deltaz;
      mtxtranformOdomToWorld.unlock();
      std::cout << "the pose of odom relative to Map: x" << tranformOdomToWorld[3] << " y" << tranformOdomToWorld[4]
                  << " z" << tranformOdomToWorld[5] <<std::endl;
      std::cout<<"before icp score : " << icp.getFitnessScore() <<std::endl;
      if (icp.hasConverged() == false || icp.getFitnessScore() > historyKeyframeFitnessScore) 
      {
        std::cout<<"after Fail icp score : " << icp.getFitnessScore() <<std::endl;
        initializedFlag = Initializing;
        std_msgs::msg::Bool InitialFlag;
        InitialFlag.data = false;
        pudInitialFlag->publish(InitialFlag);
        std::cout << "Initializing Fail" << std::endl;
        return;
      } else{
        initializedFlag = Initialized;
    
        std_msgs::msg::Bool InitialFlag ;
        InitialFlag.data = true;
        pudInitialFlag->publish(InitialFlag);
        std::cout<<"after Succeed icp score : " << icp.getFitnessScore() <<std::endl;
        std::cout << "Initializing Succeed" << std::endl;
        geometry_msgs::msg::PoseStamped pose_odomTo_map;
        tf2::Quaternion q_odomTo_map;
        q_odomTo_map.setRPY(deltaR, deltaP, deltaY);

        pose_odomTo_map.header.stamp = timeLaserInfoStamp;
        pose_odomTo_map.header.frame_id = "map";
        pose_odomTo_map.pose.position.x = deltax; 
        pose_odomTo_map.pose.position.y = deltay; 
        pose_odomTo_map.pose.position.z = deltaz;//deltaz;
        pose_odomTo_map.pose.orientation.x = q_odomTo_map.x();
        pose_odomTo_map.pose.orientation.y = q_odomTo_map.y();
        pose_odomTo_map.pose.orientation.z = q_odomTo_map.z();
        pose_odomTo_map.pose.orientation.w = q_odomTo_map.w();
        pubOdomToMapPose->publish(pose_odomTo_map);
        T_odom_to_map = poseToMatrix(pose_odomTo_map);
      }
  }
  void cloudGlobalLoad(){
        //todo
        // if(multiple_floors == true){
        //     // map_location = "/home/unitree/graph_pid_ws/src/task/maps/pcd/"+to_string(level)+"/";
        //     // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\033[1;32m----> Open multiple floors load.\033[0m");

        // }
      std::string map_dir = map_location + "/" + map_name;
      pcl::io::loadPCDFile(map_dir + "/GlobalMap.pcd", *cloudGlobalMap);
      CloudPtr cloud_temp(new CloudType());
      downSizeFilterICP.setInputCloud(cloudGlobalMap);
      downSizeFilterICP.filter(*cloud_temp);
      *cloudGlobalMapDS = *cloud_temp;
      // Create a VoxelGrid filter object
      pcl::VoxelGrid<PointType> voxelFilter;
      voxelFilter.setInputCloud(cloudGlobalMap);
      
      // Set the voxel grid leaf size (this value can be adjusted according to your needs)
      voxelFilter.setLeafSize(0.1f, 0.1f, 0.1f);
      // Create a new point cloud to store the filtered output
      CloudPtr cloudFiltered(new pcl::PointCloud<PointType>);
      voxelFilter.filter(*cloudFiltered);
      sensor_msgs::msg::PointCloud2 output;
      pcl::toROSMsg(*cloudFiltered, output);
      output.header.frame_id = "map";
      output.header.stamp = this->now();
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\033[1;31m----> Publish the global map.\033[0m");
      globalMap_pub_->publish(output);
      double min_x = 0,max_x = 0,min_y = 0, max_y = 0, min_z = 0, max_z =0;

      int cloud_size = cloudGlobalMapDS->points.size();
      
      for(int i = 0; i < cloud_size; ++i){
          auto pointFrom = cloudGlobalMapDS->points[i];
          auto x = pointFrom.x;
          auto y = pointFrom.y;
          auto z = pointFrom.z;

          if(x < min_x)
              min_x = x;
          if(x > max_x)
              max_x = x;
          if(y < min_y)
              min_y = y;
          if(y > max_y)
              max_y = y;
          if(z < min_z)
              min_z = z;
          if(z > max_z)
              max_z = z;
      }
        auto width = abs(min_y) + abs(max_y);
        auto height = abs(min_x) + abs(max_x);

        std::cout << "test 0.01  the size of global cloud: " << cloudGlobalMap->points.size() << std::endl;
        std::cout << "test 0.02  the size of global map after filter: " << cloudGlobalMapDS->points.size() << std::endl;
        
        std::cout << "min_y: " << min_y << std::endl;
        std::cout << "max_y: " << max_y << std::endl;

        std::cout << "width: " << width << std::endl;
        std::cout << "---------------------------"<<std::endl;

        std::cout << "min_x: " << min_x << std::endl;
        std::cout << "max_x: " << max_x << std::endl;
        std::cout << "height: " << height << std::endl;

        std_msgs::msg::Float64MultiArray pcd_config;
        pcd_config.data.push_back(min_x);
        pcd_config.data.push_back(min_y);
        pcd_config.data.push_back(min_z);
        pcd_config.data.push_back(max_x);
        pcd_config.data.push_back(max_y);
        pcd_config.data.push_back(max_z);
        pcd_config.data.push_back(width);
        pcd_config.data.push_back(height);
        
        pub_pcd_config->publish(pcd_config);

  }
    void ICPscanMatchGlobal()
    {
      // Perform the matrix multiplication
      // Eigen::Matrix4d T_map_to_base_link = pose_estimation * T_odom_mat;
      

      /******************added by gc************************/
        mtxWin.lock();
        downsampleCurrentScan();
        int latestFrameIDGlobalLocalize;
        // Convert odometry to Eigen matrix
        // Eigen::Matrix4d T_odom = odomToMatrix(latest_odom);
        CloudPtr latestCloudIn(new CloudType());
        *latestCloudIn += *laserCloudKeyFrameLastDS;
        mtxWin.unlock();

        // // Combine T_odom with T_odom_to_map
        // T_odom_to_map = T_odom_to_map * T_odom;
        // RCLCPP_INFO(this->get_logger(), "odom to map conversion successfull");
        // Convert the transformation matrix back to odometry

        // nav_msgs::msg::Odometry odom_msg;// = matrixToOdom(T_odom_to_map);
        // mtxtranformOdomToWorld.lock();
        // tranformOdomToWorld[0] = roll_global;
        // tranformOdomToWorld[1] = pitch_global;
        // tranformOdomToWorld[2] = yaw_global;
        // tranformOdomToWorld[3] = x;
        // tranformOdomToWorld[4] = y;
        // tranformOdomToWorld[5] = z;
        // mtxtranformOdomToWorld.unlock();
        // Publish the odometry message
        // pubOdomAftMappedROS->publish(odom_msg);
        // * += *transformPointCloud(win_keyframeCloudKeyFrames[latestFrameIDGlobalLocalize], &win_cloudKeyPoses6D[latestFrameIDGlobalLocalize]);
        // *latestCloudIn += *transformPointCloud(win_currentScanCloudKeyFrames[latestFrameIDGlobalLocalize],   &win_cloudKeyPoses6D[latestFrameIDGlobalLocalize]);

        
        // return;
        //add by zzr 20220806
        // geometry_msgs::msg::PoseStamped win_cloudKey_Poses6D;
        // win_cloudKey_Poses6D.header.stamp = timeLaserInfoStamp;
        // win_cloudKey_Poses6D.header.frame_id = "world";
        // win_cloudKey_Poses6D.pose.position.x = (&win_cloudKeyPoses6D[latestFrameIDGlobalLocalize])->x; 
        // win_cloudKey_Poses6D.pose.position.y =  (&win_cloudKeyPoses6D[latestFrameIDGlobalLocalize])->y; 
        // win_cloudKey_Poses6D.pose.position.z =  (&win_cloudKeyPoses6D[latestFrameIDGlobalLocalize])->z;
        // win_cloudKey_Poses6D.pose.orientation.x =  (&win_cloudKeyPoses6D[latestFrameIDGlobalLocalize])->roll;
        // win_cloudKey_Poses6D.pose.orientation.y =  (&win_cloudKeyPoses6D[latestFrameIDGlobalLocalize])->pitch;
        // win_cloudKey_Poses6D.pose.orientation.z =  (&win_cloudKeyPoses6D[latestFrameIDGlobalLocalize])->yaw;
        // pubwincloudKeyPoses6D->publish(win_cloudKey_Poses6D);

        /******************added by gc************************/

        pcl::NormalDistributionsTransform<PointType, PointType> ndt;
        ndt.setTransformationEpsilon(0.01);
        ndt.setResolution(1.0);


        pcl::IterativeClosestPoint<PointType, PointType> icp;
        icp.setMaxCorrespondenceDistance(100);
        icp.setMaximumIterations(100);
        icp.setTransformationEpsilon(1e-6);
        icp.setEuclideanFitnessEpsilon(1e-6);
        icp.setRANSACIterations(0);

        // Align cloud
        //3. calculate the tranform of odom relative to world
	      //Eigen::Affine3f transodomToWorld_init = pcl::getTransformation(0,0,0,0,0,0);
        mtxtranformOdomToWorld.lock();
        Eigen::Affine3f transodomToWorld_init = pcl::getTransformation(tranformOdomToWorld[3], tranformOdomToWorld[4],tranformOdomToWorld[5],tranformOdomToWorld[0],tranformOdomToWorld[1],tranformOdomToWorld[2]);
        mtxtranformOdomToWorld.unlock();

        Eigen::Matrix4f matricInitGuess = transodomToWorld_init.matrix();
	    //std::cout << "matricInitGuess: " << matricInitGuess << std::endl;
        //Firstly perform ndt in coarse resolution
        ndt.setInputSource(latestCloudIn);
        ndt.setInputTarget(cloudGlobalMapDS);
        pcl::PointCloud<PointType>::Ptr unused_result_0(new pcl::PointCloud<PointType>());
        ndt.align(*unused_result_0, matricInitGuess);
        //use the outcome of ndt as the initial guess for ICP
        icp.setInputSource(latestCloudIn);
        icp.setInputTarget(cloudGlobalMapDS);
        pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
        icp.align(*unused_result, ndt.getFinalTransformation());

        // //add by zzr 20220802
        // lio_sam::msg::ScoreInfo scoreInfo;
        // scoreInfo.header.stamp =  timeLaserInfoStamp;
        // scoreInfo.score = icp.getFitnessScore();
        // pubFitnessScore->publish(scoreInfo);

        Eigen::Affine3f transodomToWorld_New;
        transodomToWorld_New = icp.getFinalTransformation();
        float x, y, z, roll, pitch, yaw;
        pcl::getTranslationAndEulerAngles (transodomToWorld_New, x, y, z, roll, pitch, yaw);
        //std::cout << " in test 0.03: deltaX = " << x << " deltay = " << y << " deltaZ = " << z << std::endl;

        // 1. icp匹配得分大于0.3时，认为匹配失败
        // 2.本次匹配与上次匹配位置相差大于1m，角度和大于10度时，认为匹配失败
 
        double score = icp.getFitnessScore() ;
        double err_position;
        double err_position_z;
        double err_angle;
        double err_x = x -tranformOdomToWorld[3];
        double err_y = y -tranformOdomToWorld[4];
        double err_z = z - tranformOdomToWorld[5];

        err_position = sqrt(err_x*err_x + err_y*err_y);
        err_position_z = abs(err_z);
        err_angle = abs(roll-tranformOdomToWorld[0]) + abs(pitch-tranformOdomToWorld[1]) +  abs(yaw-tranformOdomToWorld[2]);
        double err_yaw = abs(abs(yaw) - abs(tranformOdomToWorld[2]));
        // if ( err_position > re_position_tollerance || err_angle >re_angle_tollerance || score > re_score_tollerance || err_position_z > 1.0)
        // if ( err_position > re_position_tollerance || ((err_angle >re_angle_tollerance)&&(err_yaw >re_angle_tollerance)) || score > re_score_tollerance)
        // {
        //     // initializedFlag = NonInitialized;
        //     std::cout << "relocation failed , jump this match.  err_position = " << err_position <<"  ,  err_position_z =  " << err_position_z<<"  ,  err_angle =  " << err_angle <<"  ,  Score =  " << score << "  ,  re_angle_tollerance =  " << re_angle_tollerance <<std::endl;

        //     roll = tranformOdomToWorld[0];
        //     pitch = tranformOdomToWorld[1];
        //     yaw = tranformOdomToWorld[2];
        //     x = tranformOdomToWorld[3];
        //     y = tranformOdomToWorld[4];
        //     z = tranformOdomToWorld[5];
            
        // }
        std::cout << "ICP converg flag: " << icp.hasConverged() << "  Fitness score: " << icp.getFitnessScore() <<" err_position: " << err_position << "err_position_z : " << err_position_z <<" err_angle :  " << err_angle  <<std::endl;
        std::cout << "err_position_z : " << err_position_z <<std::endl;
        /***********************重定位失效判断 end***********************/
        
        //add by zzr 20220802
        techshare_ros_pkg2::msg::ScoreInfo scoreInfo;
        scoreInfo.header.stamp =  timeLaserInfoStamp;
        scoreInfo.score = icp.getFitnessScore();
        scoreInfo.err_position = err_position;
        scoreInfo.err_position_z = err_position_z;
        scoreInfo.err_angle = err_angle;
        pubFitnessScore->publish(scoreInfo);


        std_msgs::msg::Float64 temp_score;
        temp_score.data = icp.getFitnessScore();
        pub_temp_score->publish(temp_score);
        

/******************add by zzr 20221104 RANSAC******************/
        TrasformPose temp_pose;
        temp_pose.x     = x;
        temp_pose.y     = y;
        temp_pose.z     = z;
        temp_pose.roll  = roll;
        temp_pose.pitch = pitch;
        temp_pose.yaw   = yaw;
        deque_trasformpose.push_back(temp_pose);

        //只取8组历史数据   
        if (deque_trasformpose.size() >8)  //20230526 15
            deque_trasformpose.pop_front();

        double distance = 0;
        
        if(deque_trasformpose.size() == 8){
            for(int i = 0; i < deque_trasformpose.size(); ++i){
                int count = 0;
                 for(int j = 0; j < deque_trasformpose.size(); ++j){
                     double err_x = deque_trasformpose[i].x - deque_trasformpose[j].x;
                     double err_y = deque_trasformpose[i].y - deque_trasformpose[j].y;
                     double err_z = deque_trasformpose[i].z - deque_trasformpose[j].z;
                     distance =  sqrt(err_x*err_x + err_y*err_y);
                     if(distance < 0.05)
                        count +=1 ;
                     if(count >6){
                        /*************************************************************/
                        x = deque_trasformpose[i].x;
                        y = deque_trasformpose[i].y; 
                        z = deque_trasformpose[i].z;
                        roll = deque_trasformpose[i].roll;
                        pitch = deque_trasformpose[i].pitch;
                        yaw = deque_trasformpose[i].yaw;
                        /*************************************************************/
                        geometry_msgs::msg::PoseStamped temp_pose_odomTo_map;
                        tf2::Quaternion temp_q_odomTo_map;
                        temp_q_odomTo_map.setRPY(deque_trasformpose[i].roll, deque_trasformpose[i].pitch, deque_trasformpose[i].yaw);
                        temp_pose_odomTo_map.header.stamp = timeLaserInfoStamp;
                        temp_pose_odomTo_map.header.frame_id = "map";
                        temp_pose_odomTo_map.pose.position.x = deque_trasformpose[i].x; 
                        temp_pose_odomTo_map.pose.position.y = deque_trasformpose[i].y; 
                        temp_pose_odomTo_map.pose.position.z = deque_trasformpose[i].z;
                        temp_pose_odomTo_map.pose.orientation.x = temp_q_odomTo_map.x();
                        temp_pose_odomTo_map.pose.orientation.y = temp_q_odomTo_map.y();
                        temp_pose_odomTo_map.pose.orientation.z = temp_q_odomTo_map.z();
                        temp_pose_odomTo_map.pose.orientation.w = temp_q_odomTo_map.w();
                        temp_pubOdomToMapPose->publish(temp_pose_odomTo_map);
                        break;
                     }
                }
            if(count >6) break;  
              
            }
        }
/******************end RANSAC******************/


     /*************************************************************/
        mtxtranformOdomToWorld.lock();
        tranformOdomToWorld[0] = roll;
        tranformOdomToWorld[1] = pitch;
        tranformOdomToWorld[2] = yaw;
        tranformOdomToWorld[3] = x;
        tranformOdomToWorld[4] = y;
        tranformOdomToWorld[5] = z;
        mtxtranformOdomToWorld.unlock();
        //publish the laserpointcloud in world frame

        //publish global map
        // publishCloud(pubMapWorld, cloudGlobalMapDS, timeLaserInfoStamp, "map");//publish world map

        if (icp.hasConverged() == true && icp.getFitnessScore() < historyKeyframeFitnessScore)
        {
            geometry_msgs::msg::PoseStamped pose_odomTo_map;
            tf2::Quaternion q_odomTo_map;
            q_odomTo_map.setRPY(roll, pitch, yaw);

            pose_odomTo_map.header.stamp = timeLaserInfoStamp;
            pose_odomTo_map.header.frame_id = "map";
            pose_odomTo_map.pose.position.x = x; 
            pose_odomTo_map.pose.position.y = y; 
            pose_odomTo_map.pose.position.z = z;
            pose_odomTo_map.pose.orientation.x = q_odomTo_map.x();
            pose_odomTo_map.pose.orientation.y = q_odomTo_map.y();
            pose_odomTo_map.pose.orientation.z = q_odomTo_map.z();
            pose_odomTo_map.pose.orientation.w = q_odomTo_map.w();
            pubOdomToMapPose->publish(pose_odomTo_map);
        }
        //publish the trsformation between map and odom
    }



  void allocateMemory()
  {
      T_odom_to_map =  Eigen::Matrix4d::Identity();
      keyframeCloudKeyFrames.clear();
      surfCloudKeyFrames.clear();
      deque_trasformpose.clear();
      win_cloudKeyPoses3D.clear();
      win_cloudKeyPoses6D.clear();
      win_keyframeCloudKeyFrames.clear();
      win_currentScanCloudKeyFrames.clear();

      timeLastProcessing = -1;
      isDegenerate = false;
      winSize = 30;
      laserCloudKeyFrameFromMapDSNum = 0;
      laserCloudCurrentScanFromMapDSNum = 0;
      laserCloudKeyFrameLastDSNum = 0;
      laserCloudCurrentScanLastDSNum = 0;
      aLoopIsClosed = false;
      initialized_Flag = false;
      imuPreintegrationResetId = 0;

      cloudGlobalMap.reset(new CloudType());//addded by gc
      cloudGlobalMapDS.reset(new CloudType());//added
      cloudScanForInitialize.reset(new CloudType());
      laserCloudKeyFrameLast.reset(new CloudType());
      laserCloudCurrentScanLast.reset(new CloudType());
      laserCloudKeyFrameLastDS.reset(new CloudType());
      laserCloudCurrentScanLastDS.reset(new CloudType());
      resetLIO();
      for (int i = 0; i < 6; ++i){
          transformInTheWorld[i] = 0;
      }

      for (int i = 0; i < 6; ++i){
          tranformOdomToWorld[i] = 0;
      }
      initializedFlag = NonInitialized;
      InitializedType = Manual;  //needs to have an operator
      downSizeFilterKeyFrame.setLeafSize(odometryKeyFrameLeafSize, odometryKeyFrameLeafSize, odometryKeyFrameLeafSize);
      downSizeFilterCurrentScan.setLeafSize(odometryCurrentScanLeafSize, odometryCurrentScanLeafSize, odometryCurrentScanLeafSize);
      downSizeFilterICP.setLeafSize(odometryCurrentScanLeafSize, odometryCurrentScanLeafSize, odometryCurrentScanLeafSize);
      downSizeFilterSurroundingKeyPoses.setLeafSize(surroundingKeyframeDensity, surroundingKeyframeDensity, surroundingKeyframeDensity); // for surrounding key poses of scan-to-map optimization

  }

    void resetLIO()
    {
        cloudKeyPoses3D.reset(new CloudType());
        cloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());

        laserCloudKeyFrameLast.reset(new CloudType()); // corner feature set from odoOptimization
        laserCloudCurrentScanLast.reset(new CloudType()); // surf feature set from odoOptimization
        laserCloudKeyFrameLastDS.reset(new CloudType()); // downsampled corner featuer set from odoOptimization
        laserCloudCurrentScanLastDS.reset(new CloudType()); // downsampled surf featuer set from odoOptimization

        laserCloudOri.reset(new CloudType());
        coeffSel.reset(new CloudType());

        laserCloudKeyFrameFromMap.reset(new CloudType());
        laserCloudCurrentScanFromMap.reset(new CloudType());
        laserCloudKeyFrameFromMapDS.reset(new CloudType());
        laserCloudCurrentScanFromMapDS.reset(new CloudType());

        kdtreeKeyFrameFromMap.reset(new pcl::KdTreeFLANN<PointType>());
        kdtreeCurrentScanFromMap.reset(new pcl::KdTreeFLANN<PointType>());

        for (int i = 0; i < 6; ++i){
            transformTobeMapped[i] = 0;
        }
        matP.setZero();
    }


  void laserCloudInfoHandler(const techshare_ros_pkg2::msg::CloudInfo::SharedPtr msgIn)
  {
    
    if(switchflag == false){
        // extract time stamp
        timeLaserInfoStamp = msgIn->header.stamp;
        timeLaserCloudInfoLast = msgIn->header.stamp.sec + msgIn->header.stamp.nanosec * 1e-9;
        // extract info and feature cloud
        cloudInfo = *msgIn;
        mtxWin.lock();
        // RCLCPP_INFO(this->get_logger(), "convert from ros msg to pcl");
        pcl::fromROSMsg(msgIn->cloud_keyframe,  *laserCloudKeyFrameLast);
        pcl::fromROSMsg(msgIn->cloud_current_scan, *laserCloudCurrentScanLast);
        latest_odom = cloudInfo.current_odom;
        mtxWin.unlock();
        /************************************added by gc*****************************/
        //if the sysytem is not initialized after the first scan for the system to initialize
        //the LIO system start working only when the localization initializing is finished
        // RCLCPP_INFO(this->get_logger(), "initialization check");
        if(!initialized_Flag) return;
        if(initializedFlag == NonInitialized || initializedFlag == Initializing)
        {   
          if(cloudScanForInitialize->points.size() == 0)
            {
              // RCLCPP_INFO(this->get_logger(), "down sampling");
              downsampleCurrentScan();
              mtx_general.lock();
              *cloudScanForInitialize += *laserCloudKeyFrameLastDS;
              *cloudScanForInitialize += *laserCloudCurrentScanLastDS;
              RCLCPP_INFO(this->get_logger(), "The size of cloudScanForInitialize is %d",cloudScanForInitialize->points.size());
              mtx_general.unlock();
              laserCloudKeyFrameLastDS->clear();
              laserCloudCurrentScanLastDS->clear();
              laserCloudKeyFrameLastDSNum = 0;
              laserCloudCurrentScanLastDSNum = 0;

              transformTobeMapped[0] = cloudInfo.imu_roll_init;//imu_0706
              transformTobeMapped[1] = cloudInfo.imu_pitch_init;
              transformTobeMapped[2] = cloudInfo.imu_yaw_init;
              if (!useImuHeadingInitialization)//gc: if not use the heading of init_IMU as Initialization
                  transformTobeMapped[2] = 0;
          }
          return;
        }
        // RCLCPP_INFO(this->get_logger(), "Received odometry");
        
        

    }else{
      return;
    }
  }


  void downsampleCurrentScan()
  {
      // Downsample cloud from current scan
      laserCloudKeyFrameLastDS->clear();
      downSizeFilterKeyFrame.setInputCloud(laserCloudKeyFrameLast);
      downSizeFilterKeyFrame.filter(*laserCloudKeyFrameLastDS);
      laserCloudKeyFrameLastDSNum = laserCloudKeyFrameLastDS->size();

      laserCloudCurrentScanLastDS->clear();
      downSizeFilterCurrentScan.setInputCloud(laserCloudCurrentScanLast);
      downSizeFilterCurrentScan.filter(*laserCloudCurrentScanLastDS);
      laserCloudCurrentScanLastDSNum = laserCloudCurrentScanLastDS->size();
  }


  void allocate_relio_Memory()
  {
      cloudScanForInitialize.reset(new CloudType());
      // resetLIO();
      //added by gc
      for (int i = 0; i < 6; ++i){
          transformInTheWorld[i] = 0;
      }

      for (int i = 0; i < 6; ++i){
          tranformOdomToWorld[i] = 0;
      }
      initializedFlag = NonInitialized;
      initialized_Flag = false;

  }

  void relocation_odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom_to_mapMsg){
      if (initializedFlag != Initialized) return;

      float x = odom_to_mapMsg->pose.pose.position.x;
      float y = odom_to_mapMsg->pose.pose.position.y;
      float z = odom_to_mapMsg->pose.pose.position.z;

      //roll-pitch-yaw
      tf2::Quaternion q_global;
      double roll_global; double pitch_global; double yaw_global;

      q_global.setX(odom_to_mapMsg->pose.pose.orientation.x);
      q_global.setY(odom_to_mapMsg->pose.pose.orientation.y);
      q_global.setZ(odom_to_mapMsg->pose.pose.orientation.z);
      q_global.setW(odom_to_mapMsg->pose.pose.orientation.w);

      tf2::Matrix3x3(q_global).getRPY(roll_global, pitch_global, yaw_global);
      //global transformation
      mtxtranformOdomToWorld.lock();
      tranformOdomToWorld[0] = roll_global;
      tranformOdomToWorld[1] = pitch_global;
      tranformOdomToWorld[2] = yaw_global;
      tranformOdomToWorld[3] = x;
      tranformOdomToWorld[4] = y;
      tranformOdomToWorld[5] = z;
      mtxtranformOdomToWorld.unlock();
  }
  

  void initialpose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pose_msg)
  {
      //first calculate global pose
      //x-y-z
      RCLCPP_INFO(this->get_logger(),"Received initialflag");
      if((InitializedType == ScanContext)||(InitializedType == Unknow)){
          return ;
      }

      // if(initializedFlag == Initialized){
      allocate_relio_Memory();
      // }

      float x = pose_msg->pose.pose.position.x;
      float y = pose_msg->pose.pose.position.y;
      float z = pose_msg->pose.pose.position.z;

      //roll-pitch-yaw
      tf2::Quaternion q_global;
      double roll_global; double pitch_global; double yaw_global;

      q_global.setX(pose_msg->pose.pose.orientation.x);
      q_global.setY(pose_msg->pose.pose.orientation.y);
      q_global.setZ(pose_msg->pose.pose.orientation.z);
      q_global.setW(pose_msg->pose.pose.orientation.w);

      tf2::Matrix3x3(q_global).getRPY(roll_global, pitch_global, yaw_global);
      //global transformation
      transformInTheWorld[0] = roll_global;
      transformInTheWorld[1] = pitch_global;
      transformInTheWorld[2] = yaw_global;
      transformInTheWorld[3] = x;
      transformInTheWorld[4] = y;
      transformInTheWorld[5] = z;
      PointTypePose thisPose6DInWorld = trans2PointTypePose(transformInTheWorld);
      Eigen::Affine3f T_thisPose6DInWorld = pclPointToAffine3f(thisPose6DInWorld);
      //Odom transformation
      PointTypePose thisPose6DInOdom = trans2PointTypePose(transformTobeMapped);
      Eigen::Affine3f T_thisPose6DInOdom = pclPointToAffine3f(thisPose6DInOdom);
      //transformation: Odom to Map
      Eigen::Affine3f T_OdomToMap = T_thisPose6DInWorld * T_thisPose6DInOdom.inverse();
      float delta_x, delta_y, delta_z, delta_roll, delta_pitch, delta_yaw;
      pcl::getTranslationAndEulerAngles (T_OdomToMap, delta_x, delta_y, delta_z, delta_roll, delta_pitch, delta_yaw);

      mtxtranformOdomToWorld.lock();
      //keep for co-operate the initializing and relio, not useful for the present
      tranformOdomToWorld[0] = delta_roll;
      tranformOdomToWorld[1] = delta_pitch;
      tranformOdomToWorld[2] = delta_yaw;
      tranformOdomToWorld[3] = delta_x;
      tranformOdomToWorld[4] = delta_y;
      tranformOdomToWorld[5] = delta_z;

      mtxtranformOdomToWorld.unlock();
      initializedFlag = NonInitialized;
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\033[1;32m----> Made initial flag true.\033[0m");
      initialized_Flag = true;
  }


  //transoform functions --------------------------------------------------------------------------------------------------------------------
  PointTypePose trans2PointTypePose(float transformIn[])
  {
      PointTypePose thisPose6D;
      thisPose6D.x = transformIn[3];
      thisPose6D.y = transformIn[4];
      thisPose6D.z = transformIn[5];
      thisPose6D.roll  = transformIn[0];
      thisPose6D.pitch = transformIn[1];
      thisPose6D.yaw   = transformIn[2];
      return thisPose6D;
  }
  Eigen::Affine3f pclPointToAffine3f(PointTypePose thisPoint)
  {
      return pcl::getTransformation(thisPoint.x, thisPoint.y, thisPoint.z, thisPoint.roll, thisPoint.pitch, thisPoint.yaw);
  }

  Eigen::Affine3f trans2Affine3f(float transformIn[])
  {
      return pcl::getTransformation(transformIn[3], transformIn[4], transformIn[5], transformIn[0], transformIn[1], transformIn[2]);
  }
  
  CloudPtr transformPointCloud(CloudPtr cloudIn, PointTypePose* transformIn)
  {
      CloudPtr cloudOut(new CloudType());

      PointType *pointFrom;

      int cloudSize = cloudIn->size();
      cloudOut->resize(cloudSize);

      Eigen::Affine3f transCur = pcl::getTransformation(transformIn->x, transformIn->y, transformIn->z, transformIn->roll, transformIn->pitch, transformIn->yaw);

      for (int i = 0; i < cloudSize; ++i){

          pointFrom = &cloudIn->points[i];
          cloudOut->points[i].x = transCur(0,0) * pointFrom->x + transCur(0,1) * pointFrom->y + transCur(0,2) * pointFrom->z + transCur(0,3);
          cloudOut->points[i].y = transCur(1,0) * pointFrom->x + transCur(1,1) * pointFrom->y + transCur(1,2) * pointFrom->z + transCur(1,3);
          cloudOut->points[i].z = transCur(2,0) * pointFrom->x + transCur(2,1) * pointFrom->y + transCur(2,2) * pointFrom->z + transCur(2,3);
          cloudOut->points[i].intensity = pointFrom->intensity;
      }
      return cloudOut;
  }
  Eigen::Matrix4d poseToMatrix(const geometry_msgs::msg::PoseStamped& pose_msg) {
    // Initialize a 4x4 transformation matrix
    Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();

    // Extract the translation components from the pose
    transformation_matrix(0, 3) = pose_msg.pose.position.x;
    transformation_matrix(1, 3) = pose_msg.pose.position.y;
    transformation_matrix(2, 3) = pose_msg.pose.position.z;

    // Extract the rotation components from the pose
    tf2::Quaternion q(
        pose_msg.pose.orientation.x,
        pose_msg.pose.orientation.y,
        pose_msg.pose.orientation.z,
        pose_msg.pose.orientation.w
    );

    // Convert quaternion to rotation matrix
    tf2::Matrix3x3 rotation_matrix(q);

    // Fill the upper-left 3x3 block of the transformation matrix with the rotation matrix
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            transformation_matrix(i, j) = rotation_matrix[i][j];
        }
    }

    return transformation_matrix;
    }
    Eigen::Matrix4d odomToMatrix(const nav_msgs::msg::Odometry& odom_msg) {
      // Initialize a 4x4 transformation matrix to identity
      Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();

      // Extract the translation components from the odometry
      transformation_matrix(0, 3) = odom_msg.pose.pose.position.x;
      transformation_matrix(1, 3) = odom_msg.pose.pose.position.y;
      transformation_matrix(2, 3) = odom_msg.pose.pose.position.z;

      // Extract the rotation components from the odometry
      tf2::Quaternion q(
          odom_msg.pose.pose.orientation.x,
          odom_msg.pose.pose.orientation.y,
          odom_msg.pose.pose.orientation.z,
          odom_msg.pose.pose.orientation.w
      );

      // Convert quaternion to rotation matrix
      tf2::Matrix3x3 rotation_matrix(q);

      // Fill the upper-left 3x3 block of the transformation matrix with the rotation matrix
      for (int i = 0; i < 3; ++i) {
          for (int j = 0; j < 3; ++j) {
              transformation_matrix(i, j) = rotation_matrix[i][j];
          }
      }

      return transformation_matrix;
  }
  nav_msgs::msg::Odometry matrixToOdom(const Eigen::Matrix4d& transformation_matrix) {
      nav_msgs::msg::Odometry odom_msg;

      // Set the position
      odom_msg.pose.pose.position.x = transformation_matrix(0, 3);
      odom_msg.pose.pose.position.y = transformation_matrix(1, 3);
      odom_msg.pose.pose.position.z = transformation_matrix(2, 3);

      // Set the orientation
      tf2::Matrix3x3 rotation_matrix(
          transformation_matrix(0, 0), transformation_matrix(0, 1), transformation_matrix(0, 2),
          transformation_matrix(1, 0), transformation_matrix(1, 1), transformation_matrix(1, 2),
          transformation_matrix(2, 0), transformation_matrix(2, 1), transformation_matrix(2, 2)
      );
      tf2::Quaternion q;
      rotation_matrix.getRotation(q);

      odom_msg.pose.pose.orientation.x = q.x();
      odom_msg.pose.pose.orientation.y = q.y();
      odom_msg.pose.pose.orientation.z = q.z();
      odom_msg.pose.pose.orientation.w = q.w();

      // Set the header (you may need to adjust the frame_id and timestamp)
      odom_msg.header.frame_id = "map";
      odom_msg.child_frame_id = "base_link";

      return odom_msg;
  }
    // ----------------------------------------------------------------------------------------------------------------------------------------------




  std::mutex mutex_;
  Eigen::MatrixXd poseCovariance;
  Eigen::Matrix4d T_odom_to_map;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subSwitchFlag;
  bool reLocalize_flag = false;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftMappedROS;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubKeyPoses;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr globalMap_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubHistoryKeyFrames;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubIcpKeyFrames;    
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubRecentKeyFrames;   
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubRecentKeyFrame;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubCloudRegisteredRaw;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudInWorld;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubMapWorld;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pubOdomToMapPose;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pubtranformOdomToWorld;//add by zzr 20220806   
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pubtranformOdomToWorld_2;//add by zzr 20220806   
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pubtranformOdomToWorld_3;//add by zzr 20220806   
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pubwincloudKeyPoses6D;//add by zzr 20220806   
  rclcpp::Publisher<techshare_ros_pkg2::msg::ScoreInfo>::SharedPtr pubFitnessScore;//add by zzr 20220802
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr temp_pubOdomToMapPose;//add by zzr 20221026 for 重定位结果均值
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pudInitialFlag;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_temp_score;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_pcd_config;




  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subGPS;
  rclcpp::Subscription<techshare_ros_pkg2::msg::CloudInfo>::SharedPtr subLaserCloudInfo;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subIniPoseFromRviz;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subRelocationOdom;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subInitializedType;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subIniPoseFromSc;
  nav_msgs::msg::Odometry latest_odom;
  techshare_ros_pkg2::msg::CloudInfo cloudInfo;
  std::vector<PointType> keyframeCloudKeyFrames;
  std::vector<PointType> surfCloudKeyFrames;
  std::deque<TrasformPose> deque_trasformpose;
  CloudPtr cloudKeyPoses3D;//gc: can be used to illustrate the path of odometry // keep
  pcl::PointCloud<PointTypePose>::Ptr cloudKeyPoses6D;//gc: can be used to illustrate the path of odometry //keep

  std::mutex mtxWin;
  std::vector<PointType> win_cloudKeyPoses3D;
  std::vector<PointTypePose> win_cloudKeyPoses6D;

  std::vector<CloudPtr> win_keyframeCloudKeyFrames;
  std::vector<CloudPtr> win_currentScanCloudKeyFrames;

  CloudPtr laserCloudKeyFrameLast; // keyframe feature set from odoOptimization
  CloudPtr laserCloudCurrentScanLast; // surf feature set from odoOptimization
  CloudPtr laserCloudKeyFrameLastDS; // downsampled keyframe featuer set from odoOptimization
  CloudPtr laserCloudCurrentScanLastDS; // downsampled surf featuer set from odoOptimization

  CloudPtr laserCloudOri;
  CloudPtr coeffSel;

  std::vector<PointType> laserCloudOriKeyFrameVec; // keyframe point holder for parallel computation
  std::vector<PointType> coeffSelKeyFrameVec;
  std::vector<bool> laserCloudOriKeyFrameFlag;
  std::vector<PointType> laserCloudOriCurrentScanVec; // surf point holder for parallel computation
  std::vector<PointType> coeffSelCurrentScanVec;
  std::vector<bool> laserCloudOriCurrentScanFlag;

  CloudPtr laserCloudKeyFrameFromMap;
  CloudPtr laserCloudCurrentScanFromMap;
  CloudPtr laserCloudKeyFrameFromMapDS;
  CloudPtr laserCloudCurrentScanFromMapDS;

  pcl::KdTreeFLANN<PointType>::Ptr kdtreeKeyFrameFromMap;
  pcl::KdTreeFLANN<PointType>::Ptr kdtreeCurrentScanFromMap;

  pcl::VoxelGrid<PointType> downSizeFilterKeyFrame;
  pcl::VoxelGrid<PointType> downSizeFilterCurrentScan;
  pcl::VoxelGrid<PointType> downSizeFilterICP;
  pcl::VoxelGrid<PointType> downSizeFilterSurroundingKeyPoses; // for surrounding key poses of scan-to-map optimization

  rclcpp::Time timeLaserInfoStamp;
  double timeLaserCloudInfoLast;

  float transformTobeMapped[6];
  std::mutex mtx;

  double timeLastProcessing = -1;

  bool isDegenerate = false;
  Eigen::Matrix<float, 6, 6> matP;

  int winSize = 30;
  int laserCloudKeyFrameFromMapDSNum = 0;
  int laserCloudCurrentScanFromMapDSNum = 0;
  int laserCloudKeyFrameLastDSNum = 0;
  int laserCloudCurrentScanLastDSNum = 0;
  int keyframeFeatureMinValidNum;
  int currentscanFeatureMinValidNum;

  bool aLoopIsClosed = false;
  bool initialized_Flag = false;
  int imuPreintegrationResetId = 0;
  float historyKeyframeFitnessScore;
  float odometryKeyFrameLeafSize;
  float odometryCurrentScanLeafSize;
  float surroundingKeyframeDensity;
  bool useImuHeadingInitialization;
  double mappingProcessInterval;
  std::string map_location;
  std::string map_name;
  int numberOfCores;
  nav_msgs::msg::Path globalPath;

  Eigen::Affine3f transPointAssociateToMap;

  CloudPtr cloudGlobalMap;
  CloudPtr cloudGlobalMapDS;
  CloudPtr cloudScanForInitialize;

  float transformInTheWorld[6];// the pose in the world, i.e. the prebuilt map
  float tranformOdomToWorld[6];
  std::mutex mtxtranformOdomToWorld;
  std::mutex mtx_general;
  int level = 0;
  bool switchflag;
  enum InitializedFlag
  {
      NonInitialized,
      Initializing,
      Initialized
  };
  InitializedFlag initializedFlag;
  enum InitializedType
  {   
      Unknow,
      Manual,
      ScanContext
  };
  InitializedType InitializedType;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.use_intra_process_comms(true);
  std::string package_path = ament_index_cpp::get_package_share_directory("ig_lio");
  Logger logger(argv, package_path);
  // auto node = std::make_shared<IG_LIO_RELOCALIZATION_NODE>(package_path);

  rclcpp::executors::SingleThreadedExecutor exec; 
  auto node = std::make_shared<IG_LIO_RELOCALIZATION_NODE>(options ,package_path);
  exec.add_node(node);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\033[1;32m----> RELOCALIZATION Started.\033[0m");

  std::thread relocalizeInWorldThread(&IG_LIO_RELOCALIZATION_NODE::reLocalizeThread, node);
  exec.spin();

  relocalizeInWorldThread.join();
  rclcpp::shutdown();
  return 0;
}
