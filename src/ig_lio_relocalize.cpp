
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
#include <geometry_msgs/msg/point_stamped.hpp>
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
#include <pcl/filters/crop_box.h>
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
          else if((currentRelocStatus == NonInitialized)&&(initialized_Flag == true))
          {
              // InitializedType = Unknow;
              ICPLocalizeInitialize();
            // ICPContinuousLocalize();
          }
          else if(currentRelocStatus == Initializing)
          {
              std::cout << "Offer A New Guess Please " << std::endl;//do nothing, wait for a new initial guess
              sleep(1.0);
          }
          else if(currentRelocStatus == Initialized){
              sleep(3.0);
              RCLCPP_INFO(this->get_logger(),"tyring to do icp");
            //   ICPscanMatchGlobal();
            // performICPAfterLocalization();
            ICPContinuousLocalize();
          }
        // Add sleep here to avoid high CPU usage
        sleep(0.1);
      }
  }

private:

  void DeclareParams(){
      declare_parameter("switchflag", false);
      declare_parameter("reloc/history_keyframe_fitness_score", 0.2);
      declare_parameter("map/map_name", "test");
      declare_parameter("map/map_location", "/Downloads/LOAM/");
      declare_parameter("odom/keyframe_leafsize", 0.1);
      declare_parameter("map/map_leafsize", 0.1);
      declare_parameter("reloc/max_translation_diff", 0.5);
      declare_parameter("reloc/max_rotation_diff", 0.1); //(in radians)
  }

  void GetParams(){
      get_parameter("switchflag", switchflag);
      get_parameter("reloc/history_keyframe_fitness_score", historyKeyframeFitnessScore);  
      get_parameter("map/map_name", map_name);
      get_parameter("map/map_location", map_location);
      get_parameter("odom/keyframe_leafsize", odometryKeyFrameLeafSize);
      get_parameter("map/map_leafsize", mapLeafSize);
      get_parameter("reloc/max_translation_diff", max_translation_diff);
      get_parameter("reloc/max_rotation_diff", max_rotation_diff);

 
  }
  void allocateMemory()
  {

      initialized_Flag = false;

      cloudGlobalMap.reset(new CloudType());//addded by gc
      cloudGlobalMapDS.reset(new CloudType());//added
      cloudScanForInitialize.reset(new CloudType());
      laserCloudKeyFrameLast.reset(new CloudType());
      laserCloudKeyFrameLastDS.reset(new CloudType());
      laserCloudKeyFrameLastCropped.reset(new CloudType());
      laserCloudKeyFrameLastCroppedDS.reset(new CloudType());
      subMap.reset(new CloudType());
      subMapDS.reset(new CloudType());
      resetLIO();
      for (int i = 0; i < 6; ++i){
          transformInTheWorld[i] = 0;
      }

    //   for (int i = 0; i < 6; ++i){
    //       tranformOdomToWorld[i] = 0;
    //   }
      currentRelocStatus = NonInitialized;
      InitializedType = Manual;  //needs to have an operator
      downSizeFilterKeyFrame.setLeafSize(odometryKeyFrameLeafSize, odometryKeyFrameLeafSize, odometryKeyFrameLeafSize);
      downSizeFilterMap.setLeafSize(mapLeafSize, mapLeafSize, mapLeafSize); // for surrounding key poses of scan-to-map optimization

    }

    void resetLIO()
    {
        laserCloudKeyFrameLast.reset(new CloudType()); // corner feature set from odoOptimization
        laserCloudKeyFrameLastDS.reset(new CloudType()); // downsampled corner featuer set from odoOptimization
        laserCloudKeyFrameLastCropped.reset(new CloudType()); // downsampled corner featuer set from odoOptimization
        laserCloudKeyFrameLastCroppedDS.reset(new CloudType()); // downsampled corner featuer set from odoOptimization


        for (int i = 0; i < 6; ++i){
            transformTobeMapped[i] = 0;
        }
        matP.setZero();
    }

  void Topics(){
        subLaserCloudInfo = this->create_subscription<techshare_ros_pkg2::msg::CloudInfo>("halna/feature/cloud_info", 500, 
                                            std::bind(&IG_LIO_RELOCALIZATION_NODE::laserCloudInfoHandler,this,std::placeholders::_1));
        subIniPoseFromRviz = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 8, 
                                      std::bind(&IG_LIO_RELOCALIZATION_NODE::initialpose_callback,this,std::placeholders::_1));

        rclcpp::QoS qos_profile(10);
        qos_profile.durability(rclcpp::DurabilityPolicy::TransientLocal);
        rclcpp::PublisherOptionsWithAllocator<std::allocator<void>> pub_options;
        pub_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;
        globalMap_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("global_map", qos_profile, pub_options);
        subMap_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("lio/sub_map", 10);
        laserInFOV_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("lio/laser_in_FOV", 10);
        pubOdomToMapPose = this->create_publisher<geometry_msgs::msg::PoseStamped>("halna/mapping/pose_odomTo_map", 1);
        pudInitialFlag = this->create_publisher<std_msgs::msg::Bool>("lio/relocalization/InitialFlag",1);
        pubFitnessScore = this->create_publisher<techshare_ros_pkg2::msg::ScoreInfo>("halna/mapping/FitnessScore", 1);
        pub_pcd_config = this->create_publisher<std_msgs::msg::Float64MultiArray>("halna/pcdmap/config", 1);

        //debug
        current_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("current_pose_in_map", 10);

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
      downSizeFilterMap.setInputCloud(cloudGlobalMap);
      downSizeFilterMap.filter(*cloud_temp);
      *cloudGlobalMapDS = *cloud_temp;
      // Number of points in the point cloud
      int num_points = cloudGlobalMapDS->points.size();
      // Create a matrix to hold the points in homogeneous coordinates
      sensor_msgs::msg::PointCloud2 output;
      pcl::toROSMsg(*cloudGlobalMapDS, output);
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
  void initialpose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pose_msg)
  {
      //first calculate global pose
      //x-y-z
      RCLCPP_INFO(this->get_logger(),"Received initialflag");
      if((InitializedType == ScanContext)||(InitializedType == Unknow)){
          return ;
      }


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
      mtxWin.lock();
      allocate_relio_Memory();
      transformInTheWorld[0] = roll_global;
      transformInTheWorld[1] = pitch_global;
      transformInTheWorld[2] = yaw_global;
      transformInTheWorld[3] = x;
      transformInTheWorld[4] = y;
      transformInTheWorld[5] = z;
      PointTypePose thisPose6DInWorld = trans2PointTypePose(transformInTheWorld);
      currentRelocStatus = NonInitialized;
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\033[1;32m----> Made initial flag true.\033[0m");
      initialized_Flag = true;
      mtxWin.unlock();
      
      Eigen::Affine3f T_thisPose6DInWorld = pclPointToAffine3f(thisPose6DInWorld);
      //Odom transformation
      PointTypePose thisPose6DInOdom = trans2PointTypePose(transformTobeMapped);
      Eigen::Affine3f T_thisPose6DInOdom = pclPointToAffine3f(thisPose6DInOdom);
      //transformation: Odom to Map
      Eigen::Affine3f T_OdomToMap = T_thisPose6DInWorld * T_thisPose6DInOdom.inverse();
      float delta_x, delta_y, delta_z, delta_roll, delta_pitch, delta_yaw;
      pcl::getTranslationAndEulerAngles (T_OdomToMap, delta_x, delta_y, delta_z, delta_roll, delta_pitch, delta_yaw);

    //   mtxtranformOdomToWorld.lock();
    //   //keep for co-operate the initializing and relio, not useful for the present
    //   tranformOdomToWorld[0] = delta_roll;
    //   tranformOdomToWorld[1] = delta_pitch;
    //   tranformOdomToWorld[2] = delta_yaw;
    //   tranformOdomToWorld[3] = delta_x;
    //   tranformOdomToWorld[4] = delta_y;
    //   tranformOdomToWorld[5] = delta_z;

    //   mtxtranformOdomToWorld.unlock();
  }



    void filterPointsWithinFOV(const CloudPtr &global_map_in_base_link, float FOV_FAR, CloudPtr &global_map_in_FOV) {
        // Filter points within the FOV
        for (const auto& point : global_map_in_base_link->points) {
            if ((point.x * point.x + point.y * point.y) < (FOV_FAR * FOV_FAR)) {
                global_map_in_FOV->points.push_back(point);
            }
        }
    }


  void ICPContinuousLocalize(){
    // convert initalize pose to matrix ----------------
    mtxWin.lock();
    if(!keyFrameAvailable){
        RCLCPP_WARN(this->get_logger(), "Key frame is not available...");
        mtxWin.unlock();
        return;
    }
    keyFrameAvailable = false;
    downsampleKeyFrame();
    
    PointTypePose initialpose_in_6D = trans2PointTypePose(transformInTheWorld);
    geometry_msgs::msg::Pose latest_odom_pose = latest_odom.pose.pose;

    

    Eigen::Matrix4f T_map_to_odom = pclPointToMatrix4f(initialpose_in_6D);
    std::cout << "T_map_to_odom:\n" << T_map_to_odom << std::endl;
    //--------------------------------------------------
    //convert the latest odom to matrix -------------
    
    PointTypePose latest_odom_pose_struct;
    latest_odom_pose_struct.x = latest_odom_pose.position.x;
    latest_odom_pose_struct.y = latest_odom_pose.position.y;
    latest_odom_pose_struct.z = latest_odom_pose.position.z;
    double temp_roll, temp_pitch, temp_yaw;
    tf2::Quaternion q(
        latest_odom_pose.orientation.x,
        latest_odom_pose.orientation.y,
        latest_odom_pose.orientation.z,
        latest_odom_pose.orientation.w);
    tf2::Matrix3x3 m(q);
    m.getRPY(temp_roll, temp_pitch, temp_yaw);
    latest_odom_pose_struct.roll = static_cast<float>(temp_roll);
    latest_odom_pose_struct.pitch = static_cast<float>(temp_pitch);
    latest_odom_pose_struct.yaw = static_cast<float>(temp_yaw);

    Eigen::Matrix4f T_odom_to_base_link = pclPointToMatrix4f(latest_odom_pose_struct);
        std::cout << "T_odom_to_base_link:\n" << T_odom_to_base_link << std::endl;


    Eigen::Matrix4f current_position_in_map = T_map_to_odom * T_odom_to_base_link;
    std::cout << "current_position_in_map:\n" << current_position_in_map << std::endl;

    // Extract translation components
    // Eigen::Vector3f translation = current_position_in_map.block<3, 1>(0, 3);

    // Create and publish PointStamped message
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.frame_id = "map";
    pose_msg.header.stamp = latest_odom.header.stamp;
    pose_msg.pose.position.x = current_position_in_map(0, 3);
    pose_msg.pose.position.y = current_position_in_map(1, 3);
    pose_msg.pose.position.z = current_position_in_map(2, 3);

    // Extract rotation matrix
    Eigen::Matrix3f rotation_matrix = current_position_in_map.block<3, 3>(0, 0);

    // Convert rotation matrix to quaternion
    tf2::Matrix3x3 tf2_rotation_matrix(
        rotation_matrix(0, 0), rotation_matrix(0, 1), rotation_matrix(0, 2),
        rotation_matrix(1, 0), rotation_matrix(1, 1), rotation_matrix(1, 2),
        rotation_matrix(2, 0), rotation_matrix(2, 1), rotation_matrix(2, 2)
    );
    tf2::Quaternion _q;
    tf2_rotation_matrix.getRotation(_q);

    // Set quaternion to pose message
    pose_msg.pose.orientation.x = _q.x();
    pose_msg.pose.orientation.y = _q.y();
    pose_msg.pose.orientation.z = _q.z();
    pose_msg.pose.orientation.w = _q.w();
    current_pose_pub_->publish(pose_msg);
    //----------------------------------------------
    float FOV_FAR = 10.0;  // Example FOV distance
    CloudPtr cloud_global_map_in_base_link = transformPointCloud(cloudGlobalMapDS, current_position_in_map.inverse());
    CloudPtr cloud_keyframe_in_base_link = transformPointCloud(laserCloudKeyFrameLastDS, T_odom_to_base_link.inverse());
    CloudPtr globalMapInFOV(new CloudType()), laserCloudKeyFrameInFOV(new CloudType());
    filterPointsWithinFOV(cloud_global_map_in_base_link, FOV_FAR, globalMapInFOV);
    filterPointsWithinFOV(cloud_keyframe_in_base_link, FOV_FAR, laserCloudKeyFrameInFOV);
    subMapDS = transformPointCloud(globalMapInFOV, current_position_in_map);
    laserCloudKeyFrameLastCroppedDS = transformPointCloud(laserCloudKeyFrameInFOV, T_odom_to_base_link);
    CloudPtr laserCloudIn(new CloudType()), targetCloud(new CloudType());
    *laserCloudIn += *laserCloudKeyFrameLastDS;
    *targetCloud += *cloudGlobalMapDS;
    publishPointClouds();
    mtxWin.unlock();

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
    ndt.setInputTarget(targetCloud);
    CloudPtr unused_result_0(new CloudType());
    ndt.align(*unused_result_0, T_map_to_odom.matrix());
    //use the outcome of ndt as the initial guess for ICP
    icp.setInputSource(laserCloudIn);
    icp.setInputTarget(targetCloud);
    CloudPtr unused_result(new CloudType());
    icp.align(*unused_result, ndt.getFinalTransformation());
    Eigen::Matrix4f new_T_map_to_odom = icp.getFinalTransformation();
    double fitnessScore = icp.getFitnessScore();


    // Convert the transformation matrix to a string
    std::ostringstream transformationStream;
    transformationStream << new_T_map_to_odom;
    RCLCPP_INFO(
        this->get_logger(),
        "\033[1;34mThe Final transformation is:\n%s\nwith the score: %lf\033[0m",
        transformationStream.str().c_str(),
        fitnessScore
    );


    // // Calculate new T_map_to_odom
    // Eigen::Matrix4f T_base_link_to_odom = T_odom_to_base_link.inverse();
    // Eigen::Matrix4f new_T_map_to_odom = T_map_to_base_link * T_base_link_to_odom;
    std::cout << "A new map to odom :\n" << new_T_map_to_odom << std::endl;



    if(isTransformationReasonable(T_map_to_odom, new_T_map_to_odom)){
        RCLCPP_INFO(this->get_logger(), "\033[1;32mThis transformation is reasonable so update the trasnfromInTheWorld\033[0m");
    }else{
        RCLCPP_INFO(this->get_logger(), "\033[1;35mThis transformation is not reasonable\033[0m");
        return;
    }
    Eigen::Affine3f T_map_to_odom_new(icp.getFinalTransformation());
    float x_g, y_g, z_g, R_g, P_g, Y_g;
    pcl::getTranslationAndEulerAngles (T_map_to_odom_new, x_g, y_g, z_g, R_g, P_g, Y_g);   
    transformInTheWorld[0] = R_g;
    transformInTheWorld[1] = P_g;
    transformInTheWorld[2] = Y_g;
    transformInTheWorld[3] = x_g;
    transformInTheWorld[4] = y_g;
    transformInTheWorld[5] = z_g;

    // 
    if (icp.hasConverged() == false || icp.getFitnessScore() > historyKeyframeFitnessScore) 
    {
        std::cout<<"after Fail icp score : " << icp.getFitnessScore() <<std::endl;
        return;
    }
    else{
        RCLCPP_INFO(this->get_logger(), "\033[1;32m---------> ICP is converged with a good score so update map to odom tf\033[0m");
        PointTypePose poseInMap = trans2PointTypePose(transformInTheWorld);
        Eigen::Affine3f T_thisPose6DInWorld = pclPointToAffine3f(poseInMap);
        //Odom transformation
        PointTypePose thisPose6DInOdom = trans2PointTypePose(transformTobeMapped);
        Eigen::Affine3f T_thisPose6DInOdom = pclPointToAffine3f(thisPose6DInOdom);
        //transformation: Odom to Map
        Eigen::Affine3f T_OdomToMap = T_thisPose6DInWorld * T_thisPose6DInOdom.inverse();
        float delta_x, delta_y, delta_z, delta_roll, delta_pitch, delta_yaw;
        pcl::getTranslationAndEulerAngles (T_thisPose6DInWorld, delta_x, delta_y, delta_z, delta_roll, delta_pitch, delta_yaw);
        geometry_msgs::msg::PoseStamped pose_odomTo_map;
        tf2::Quaternion q_odomTo_map;
        q_odomTo_map.setRPY(delta_roll, delta_pitch, delta_yaw);
        pose_odomTo_map.header.stamp = timeLaserInfoStamp;
        pose_odomTo_map.header.frame_id = "map";
        pose_odomTo_map.pose.position.x = delta_x; 
        pose_odomTo_map.pose.position.y = delta_y; 
        pose_odomTo_map.pose.position.z = delta_z;//deltaz;
        pose_odomTo_map.pose.orientation.x = q_odomTo_map.x();
        pose_odomTo_map.pose.orientation.y = q_odomTo_map.y();
        pose_odomTo_map.pose.orientation.z = q_odomTo_map.z();
        pose_odomTo_map.pose.orientation.w = q_odomTo_map.w();
        pubOdomToMapPose->publish(pose_odomTo_map);
        // transformInTheWorld[0] = delta_roll;
        // transformInTheWorld[1] = delta_pitch;
        // transformInTheWorld[2] = delta_yaw;
        // transformInTheWorld[3] = delta_x;
        // transformInTheWorld[4] = delta_y;
        // transformInTheWorld[5] = delta_z;
    }
  }
  
    // Function to check if the new transformation is reasonable
    bool isTransformationReasonable(const Eigen::Matrix4f& current_transform, const Eigen::Matrix4f& new_transform) {
        // Calculate the difference in translation
        Eigen::Vector3f current_translation = current_transform.block<3, 1>(0, 3);
        Eigen::Vector3f new_translation = new_transform.block<3, 1>(0, 3);
        float translation_diff = (current_translation - new_translation).norm();

        // Calculate the difference in rotation
        Eigen::Matrix3f current_rotation = current_transform.block<3, 3>(0, 0);
        Eigen::Matrix3f new_rotation = new_transform.block<3, 3>(0, 0);
        Eigen::AngleAxisf rotation_diff_matrix(current_rotation.inverse() * new_rotation);
        float rotation_diff = std::abs(rotation_diff_matrix.angle());

        // Log the differences
        RCLCPP_INFO(this->get_logger(),
                    "\033[1;34mTranslation difference: %f, Rotation difference: %f\033[0m",
                    translation_diff, rotation_diff);

        // Check if the differences are within the thresholds
        bool is_reasonable = (translation_diff <= max_translation_diff && rotation_diff <= max_rotation_diff);

        if (!is_reasonable) {
            if (translation_diff > max_translation_diff) {
                RCLCPP_WARN(this->get_logger(),
                            "\033[1;31mTranslation difference exceeds threshold: %f > %f\033[0m",
                            translation_diff, max_translation_diff);
            }
            if (rotation_diff > max_rotation_diff) {
                RCLCPP_WARN(this->get_logger(),
                            "\033[1;31mRotation difference exceeds threshold: %f > %f\033[0m",
                            rotation_diff, max_rotation_diff);
            }
        } else {
            RCLCPP_INFO(this->get_logger(),
                        "\033[1;32mTransformation is reasonable.\033[0m");
        }

        return is_reasonable;
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
    //   std::cout << "the pose in odom before initializing is: x" << tranformOdomToWorld[3] << " y" << tranformOdomToWorld[4]
    //             << " z" << tranformOdomToWorld[5] <<std::endl;
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
    //   mtxtranformOdomToWorld.lock();
    //       //renew tranformOdomToWorld
    //   tranformOdomToWorld[0] = deltaR;
    //   tranformOdomToWorld[1] = deltaP;
    //   tranformOdomToWorld[2] = deltaY;
    //   tranformOdomToWorld[3] = deltax;
    //   tranformOdomToWorld[4] = deltay;
    //   tranformOdomToWorld[5] = deltaz;
    //   mtxtranformOdomToWorld.unlock();
      std::cout << "the pose of odom relative to Map: x" << deltax << " y" <<deltay
                  << " z" << deltaz <<std::endl;
      std::cout<<"before icp score : " << icp.getFitnessScore() <<std::endl;
      if (icp.hasConverged() == false || icp.getFitnessScore() > historyKeyframeFitnessScore) 
      {
        std::cout<<"after Fail icp score : " << icp.getFitnessScore() <<std::endl;
        currentRelocStatus = Initializing;
        std_msgs::msg::Bool InitialFlag;
        InitialFlag.data = false;
        pudInitialFlag->publish(InitialFlag);
        std::cout << "Initializing Fail" << std::endl;
        return;
      } else{
        currentRelocStatus = Initialized;
    
        std_msgs::msg::Bool InitialFlag ;
        InitialFlag.data = true;
        pudInitialFlag->publish(InitialFlag);
        std::cout<<"after Succeed icp score : " << icp.getFitnessScore() <<std::endl;
        std::cout << "Initializing Succeed" << std::endl;
        geometry_msgs::msg::PoseStamped pose_odomTo_map;
        tf2::Quaternion q_odomTo_map;
        q_odomTo_map.setRPY(R_g, P_g, Y_g);

        pose_odomTo_map.header.stamp = timeLaserInfoStamp;
        pose_odomTo_map.header.frame_id = "map";
        pose_odomTo_map.pose.position.x = x_g; 
        pose_odomTo_map.pose.position.y = y_g; 
        pose_odomTo_map.pose.position.z = z_g;//deltaz;
        pose_odomTo_map.pose.orientation.x = q_odomTo_map.x();
        pose_odomTo_map.pose.orientation.y = q_odomTo_map.y();
        pose_odomTo_map.pose.orientation.z = q_odomTo_map.z();
        pose_odomTo_map.pose.orientation.w = q_odomTo_map.w();
        pubOdomToMapPose->publish(pose_odomTo_map);
      }
  }





  void publishPointClouds(){
        sensor_msgs::msg::PointCloud2 laser_to_odom_msg;
        pcl::toROSMsg(*laserCloudKeyFrameLastDS, laser_to_odom_msg);
        laser_to_odom_msg.header.frame_id = latest_odom.header.frame_id;
        laser_to_odom_msg.header.stamp = latest_odom.header.stamp;
        laserInFOV_pub_->publish(laser_to_odom_msg);
        // Convert the sub-map to ROS message
        sensor_msgs::msg::PointCloud2 subMapMsg;
        pcl::toROSMsg(*subMapDS, subMapMsg);

        subMapMsg.header.frame_id = "map";
        subMapMsg.header.stamp = latest_odom.header.stamp;

        // Publish the sub-map
        subMap_pub_->publish(subMapMsg);    
  }

  void laserCloudInfoHandler(const techshare_ros_pkg2::msg::CloudInfo::SharedPtr msgIn)
  {
    
    if(switchflag == false){
        // extract time stamp

        // extract info and feature cloud
        mtxWin.lock();

        keyFrameAvailable = true;
        cloudInfo = *msgIn;
        timeLaserInfoStamp = msgIn->header.stamp;
        timeLaserCloudInfoLast = msgIn->header.stamp.sec + msgIn->header.stamp.nanosec * 1e-9;
        
        if (msgIn->keyframe_available){
            pcl::fromROSMsg(msgIn->cloud_keyframe,  *laserCloudKeyFrameLast);
            latest_odom = cloudInfo.current_odom;
            keyFrameAvailable = true;
            tf2::Quaternion q(
                latest_odom.pose.pose.orientation.x,
                latest_odom.pose.pose.orientation.y,
                latest_odom.pose.pose.orientation.z,
                latest_odom.pose.pose.orientation.w);
            tf2::Matrix3x3 m(q);
            double temp_roll, temp_pitch, temp_yaw;
            m.getRPY(temp_roll, temp_pitch, temp_yaw);
            transformTobeMapped[0] = temp_roll;
            transformTobeMapped[1] = temp_pitch;
            transformTobeMapped[2] = temp_yaw;
            transformTobeMapped[3] = latest_odom.pose.pose.position.x;
            transformTobeMapped[4] = latest_odom.pose.pose.position.y;
            transformTobeMapped[5] = latest_odom.pose.pose.position.z;
        }
        mtxWin.unlock();
        /************************************added by gc*****************************/
        //if the sysytem is not initialized after the first scan for the system to initialize
        //the LIO system start working only when the localization initializing is finished
        // RCLCPP_INFO(this->get_logger(), "initialization check");
        if(!initialized_Flag) return;
        if(currentRelocStatus == NonInitialized || currentRelocStatus == Initializing)
        {   
          if(cloudScanForInitialize->points.size() <= 1000)
            {
              // RCLCPP_INFO(this->get_logger(), "down sampling");
              downsampleKeyFrame();
              mtx_general.lock();
              *cloudScanForInitialize += *laserCloudKeyFrameLastDS;
              RCLCPP_INFO(this->get_logger(), "The size of cloudScanForInitialize is %d",cloudScanForInitialize->points.size());
              mtx_general.unlock();
              laserCloudKeyFrameLastDS->clear();
          }
          return;
        }
        // RCLCPP_INFO(this->get_logger(), "Received odometry");
        
        

    }else{
      return;
    }
  }


  void downsampleKeyFrame()
  {
      // Downsample cloud from current scan
      laserCloudKeyFrameLastDS->clear();
      downSizeFilterKeyFrame.setInputCloud(laserCloudKeyFrameLast);
      downSizeFilterKeyFrame.filter(*laserCloudKeyFrameLastDS);
      laserCloudKeyFrameLastCroppedDS->clear();

  }

//   void downsampleSubMapAndLaserToMap()
//   {
//       // Downsample cloud from current scan
//       laserCloudKeyFrameLastDS->clear();
//       downSizeFilterCurrentScan.setInputCloud(subMap);
//       downSizeFilterCurrentScan.filter(*subMapDS);

//       downSizeFilterKeyFrame.setInputCloud(laserCloudKeyFrameLastCropped);
//       downSizeFilterKeyFrame.filter(*laserCloudKeyFrameLastCroppedDS);
//       laserCloudKeyFrameLastDSNum = laserCloudKeyFrameLastDS->size();
//   }

  void allocate_relio_Memory()
  {
      cloudScanForInitialize.reset(new CloudType());
      // resetLIO();
      //added by gc
      for (int i = 0; i < 6; ++i){
          transformInTheWorld[i] = 0;
      }

    //   for (int i = 0; i < 6; ++i){
    //       tranformOdomToWorld[i] = 0;
    //   }
      currentRelocStatus = NonInitialized;
      initialized_Flag = false;

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
    Eigen::Matrix4f pclPointToMatrix4f(PointTypePose thisPoint) {
        // Use pcl::getTransformation to get an Eigen::Affine3f transformation
        Eigen::Affine3f affine3f = pcl::getTransformation(thisPoint.x, thisPoint.y, thisPoint.z, thisPoint.roll, thisPoint.pitch, thisPoint.yaw);
        
        // Convert Eigen::Affine3f to Eigen::Matrix4f
        Eigen::Matrix4f matrix4f = affine3f.matrix();
        
        return matrix4f;
    }
 // Function to create an Eigen::Affine3f transformation from PointTypePose (added by wataru 6/20/2024)
  Eigen::Affine3f poseToAffine3f(const PointTypePose &pose) {
    Eigen::Translation3f translation(pose.x, pose.y, pose.z);
    Eigen::AngleAxisf rollAngle(pose.roll, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf pitchAngle(pose.pitch, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf yawAngle(pose.yaw, Eigen::Vector3f::UnitZ());
    Eigen::Quaternionf quaternion = yawAngle * pitchAngle * rollAngle;
    return translation * quaternion;
  }

  Eigen::Affine3f trans2Affine3f(float transformIn[])
  {
      return pcl::getTransformation(transformIn[3], transformIn[4], transformIn[5], transformIn[0], transformIn[1], transformIn[2]);
  }
  
    CloudPtr transformPointCloud(CloudPtr cloudIn, const PointTypePose& transformIn) {
        CloudPtr cloudOut(new CloudType());
        int cloudSize = cloudIn->size();
        cloudOut->resize(cloudSize);

        Eigen::Affine3f transCur = pcl::getTransformation(
            transformIn.x, transformIn.y, transformIn.z,
            transformIn.roll, transformIn.pitch, transformIn.yaw
        );

        for (int i = 0; i < cloudSize; ++i) {
            const PointType& pointFrom = cloudIn->points[i];
            PointType& pointTo = cloudOut->points[i];

            Eigen::Vector3f pointTransformed = transCur * Eigen::Vector3f(pointFrom.x, pointFrom.y, pointFrom.z);

            pointTo.x = pointTransformed.x();
            pointTo.y = pointTransformed.y();
            pointTo.z = pointTransformed.z();
            pointTo.intensity = pointFrom.intensity;
        }
        return cloudOut;
    }
    CloudPtr transformPointCloud(const CloudPtr& input_cloud, const Eigen::Matrix4f& transformation_matrix) {
        CloudPtr transformed_cloud(new CloudType);

        for (const auto& point : input_cloud->points) {
            Eigen::Vector4f homogeneous_point(point.x, point.y, point.z, 1.0f);
            Eigen::Vector4f transformed_point = transformation_matrix * homogeneous_point;

            pcl::PointXYZINormal new_point;
            new_point.x = transformed_point.x();
            new_point.y = transformed_point.y();
            new_point.z = transformed_point.z();
            new_point.normal_x = point.normal_x;
            new_point.normal_y = point.normal_y;
            new_point.normal_z = point.normal_z;
            new_point.curvature = point.curvature;

            transformed_cloud->push_back(new_point);
        }

        return transformed_cloud;
    }

    // ----------------------------------------------------------------------------------------------------------------------------------------------




  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subSwitchFlag;
  bool reLocalize_flag = false;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr globalMap_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr subMap_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr laserInFOV_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pubOdomToMapPose;
  rclcpp::Publisher<techshare_ros_pkg2::msg::ScoreInfo>::SharedPtr pubFitnessScore;//add by zzr 20220802
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pudInitialFlag;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_pcd_config;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_pub_;




  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subGPS;
  rclcpp::Subscription<techshare_ros_pkg2::msg::CloudInfo>::SharedPtr subLaserCloudInfo;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subIniPoseFromRviz;
  nav_msgs::msg::Odometry latest_odom;
  techshare_ros_pkg2::msg::CloudInfo cloudInfo;


  

  CloudPtr laserCloudKeyFrameLast; // keyframe feature set from odoOptimization
  CloudPtr laserCloudKeyFrameLastDS; // downsampled keyframe featuer set from odoOptimization
  CloudPtr laserCloudKeyFrameLastCropped;// downsampled keyframe featuer set from odoOptimization
  CloudPtr laserCloudKeyFrameLastCroppedDS; // downsampled keyframe featuer set from odoOptimization
  CloudPtr subMap;
  CloudPtr subMapDS;
  CloudPtr cloudGlobalMap;
  CloudPtr cloudGlobalMapDS;
  CloudPtr cloudScanForInitialize;


  pcl::VoxelGrid<PointType> downSizeFilterKeyFrame;
  pcl::VoxelGrid<PointType> downSizeFilterMap; // for surrounding key poses of scan-to-map optimization

  rclcpp::Time timeLaserInfoStamp;
  double timeLaserCloudInfoLast;

  float transformTobeMapped[6];
  std::mutex mtxWin;
  std::mutex mtx;
  std::mutex mtxtranformOdomToWorld;
  std::mutex mtx_general;

  bool keyFrameAvailable = false;
  Eigen::Matrix<float, 6, 6> matP;

  bool initialized_Flag = false;
  float historyKeyframeFitnessScore;
  float odometryKeyFrameLeafSize;
  float mapLeafSize;
  // Global thresholds for translation and rotation differences
  float max_translation_diff;
  float max_rotation_diff;     

  std::string map_location;
  std::string map_name;

  float transformInTheWorld[6];// the pose in the world, i.e. the prebuilt map
//   float tranformOdomToWorld[6];

  int level = 0;
  bool switchflag;

  enum InitializedFlag
  {
      NonInitialized,
      Initializing,
      Initialized
  };
  InitializedFlag currentRelocStatus;
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
