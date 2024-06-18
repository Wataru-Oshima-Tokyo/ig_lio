
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
    Topics();

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
              sleep(10.0);
              ICPscanMatchGlobal();
          }
      }
  }

private:

  void DeclareParams(){
      declare_parameter("switchflag", false);
      declare_parameter("historyKeyframeFitnessScore", 0.3);
      declare_parameter("useImuHeadingInitialization", false);
      declare_parameter("mappingProcessInterval", 0.15);
  }

  void GetParams(){
      get_parameter("switchflag", switchflag);
      get_parameter("historyKeyframeFitnessScore", historyKeyframeFitnessScore);  
      get_parameter("useImuHeadingInitialization", useImuHeadingInitialization);
      get_parameter("mappingProcessInterval", mappingProcessInterval);
  
  }
  void Initialize(){

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
        pubOdomAftMappedROS = this->create_publisher<nav_msgs::msg::Odometry> ("halna/mapping/odometry", 1);
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


        allocateMemory();
  }

  void ICPLocalizeInitialize(){
      pcl::PointCloud<PointType>::Ptr laserCloudIn(new pcl::PointCloud<PointType>());
      mtx_general.lock();
      *laserCloudIn += *cloudScanForInitialize; //must be the keyframe from ig_lio
      mtx_general.unlock();

      if(laserCloudIn->points.size() == 0){
          RCLCPP_INFO(this->get_logger(), "the cloud is empty");
          return;
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
      pcl::PointCloud<PointType>::Ptr unused_result_0(new pcl::PointCloud<PointType>());
      PointTypePose thisPose6DInWorld = trans2PointTypePose(transformInTheWorld);
      Eigen::Affine3f T_thisPose6DInWorld = pclPointToAffine3f(thisPose6DInWorld);
      ndt.align(*unused_result_0, T_thisPose6DInWorld.matrix());
      //use the outcome of ndt as the initial guess for ICP
      icp.setInputSource(laserCloudIn);
      icp.setInputTarget(cloudGlobalMapDS);
      pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
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
        pose_odomTo_map.pose.position.x = deltax; pose_odomTo_map.pose.position.y = deltay; pose_odomTo_map.pose.position.z = deltaz;
        pose_odomTo_map.pose.orientation.x = q_odomTo_map.x();
        pose_odomTo_map.pose.orientation.y = q_odomTo_map.y();
        pose_odomTo_map.pose.orientation.z = q_odomTo_map.z();
        pose_odomTo_map.pose.orientation.w = q_odomTo_map.w();
        pubOdomToMapPose->publish(pose_odomTo_map);

      }
  }

  void ICPscanMatchGlobal(){

  }



  void allocateMemory()
  {
      // gtSAMgraph.resize(0);
      // initialEstimate.clear();

      // ISAM2Params parameters;
      // parameters.relinearizeThreshold = 0.1;
      // parameters.relinearizeSkip = 1;
      // isam = new ISAM2(parameters);

      // isamCurrentEstimate.clear();
      keyframeCloudKeyFrames.clear();
      surfCloudKeyFrames.clear();
      deque_trasformpose.clear();
      win_cloudKeyPoses3D.clear();
      win_cloudKeyPoses6D.clear();
      win_keyframeCloudKeyFrames.clear();
      win_surfCloudKeyFrames.clear();

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

      cloudGlobalMap.reset(new pcl::PointCloud<PointType>());//addded by gc
      cloudGlobalMapDS.reset(new pcl::PointCloud<PointType>());//added
      cloudScanForInitialize.reset(new pcl::PointCloud<PointType>());
      // resetLIO();
      //added by gc
      for (int i = 0; i < 6; ++i){
          transformInTheWorld[i] = 0;
      }

      for (int i = 0; i < 6; ++i){
          tranformOdomToWorld[i] = 0;
      }
      initializedFlag = NonInitialized;
      // InitializedType = Manual;  //needs to have an operator
      // cloudGlobalLoad();//added by gc
      //added by gc
  }

  void laserCloudInfoHandler(const techshare_ros_pkg2::msg::CloudInfo::SharedPtr msgIn)
  {
    if(switchflag == false){
        // extract time stamp
        timeLaserInfoStamp = msgIn->header.stamp;
        timeLaserCloudInfoLast = msgIn->header.stamp.sec + msgIn->header.stamp.nanosec * 1e-9;
        // extract info and feature cloud
        cloudInfo = *msgIn;
        pcl::fromROSMsg(msgIn->cloud_keyframe,  *laserCloudKeyFrameLast);
        pcl::fromROSMsg(msgIn->cloud_current_scan, *laserCloudCurrentScanLast);
        /************************************added by gc*****************************/
        //if the sysytem is not initialized after the first scan for the system to initialize
        //the LIO system start working only when the localization initializing is finished
        if(initializedFlag == NonInitialized || initializedFlag == Initializing)
        {   
          if(cloudScanForInitialize->points.size() == 0)
            {
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
        /************************************added by gc*****************************/
        std::lock_guard<std::mutex> lock(mtx);

        if (timeLaserCloudInfoLast - timeLastProcessing >= mappingProcessInterval) {//gc:control the rate of mapping process

            // timeLastProcessing = timeLaserCloudInfoLast;

            // updateInitialGuess(); //gc: update initial value for states
            // RCLCPP_INFO(this->get_logger(), "Initial guess updated.");

            // extractSurroundingKeyFrames(); //gc:
            // RCLCPP_INFO(this->get_logger(), "Surrounding keyframes extracted.");

            // downsampleCurrentScan(); //gc: down sample the current corner points and surface points
            // RCLCPP_INFO(this->get_logger(), "Current scan downsampled.");

            // scan2MapOptimization(); //gc: calculate the transformation using lidar measurement with the IMU preintegration as initial values
            // RCLCPP_INFO(this->get_logger(), "Scan-to-map optimization complete.");

            // // and then interpolate roll and pitch angle using IMU measurement and above measurement
            // saveKeyFramesAndFactor(); //gc: save corner cloud and surface cloud of this scan, and add odom and GPS factors
            // RCLCPP_INFO(this->get_logger(), "Keyframes and factors saved.");

            // publishOdometry();
            // RCLCPP_INFO(this->get_logger(), "Odometry published.");

            // publishFrames();
            // RCLCPP_INFO(this->get_logger(), "Frames published.");
        }
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
      // cloudScanForInitialize.reset(new pcl::PointCloud<PointType>());
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

  void initialpose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pose_msg)
  {
      //first calculate global pose
      //x-y-z
      RCLCPP_INFO(this->get_logger(),"Received initialflag");
      if((InitializedType == ScanContext)||(InitializedType == Unknow)){
          return ;
      }

      if(initializedFlag == Initialized){
          allocate_relio_Memory();
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
    // ----------------------------------------------------------------------------------------------------------------------------------------------





  Eigen::MatrixXd poseCovariance;

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
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subInitializedType;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subIniPoseFromSc;

  techshare_ros_pkg2::msg::CloudInfo cloudInfo;
  std::vector<pcl::PointCloud<PointType>::Ptr> keyframeCloudKeyFrames;
  std::vector<pcl::PointCloud<PointType>::Ptr> surfCloudKeyFrames;
  std::deque<TrasformPose> deque_trasformpose;
  pcl::PointCloud<PointType>::Ptr cloudKeyPoses3D;//gc: can be used to illustrate the path of odometry // keep
  pcl::PointCloud<PointTypePose>::Ptr cloudKeyPoses6D;//gc: can be used to illustrate the path of odometry //keep

  std::mutex mtxWin;
  std::vector<PointType> win_cloudKeyPoses3D;
  std::vector<PointTypePose> win_cloudKeyPoses6D;

  std::vector<pcl::PointCloud<PointType>::Ptr> win_keyframeCloudKeyFrames;
  std::vector<pcl::PointCloud<PointType>::Ptr> win_surfCloudKeyFrames;

  pcl::PointCloud<PointType>::Ptr laserCloudKeyFrameLast; // keyframe feature set from odoOptimization
  pcl::PointCloud<PointType>::Ptr laserCloudCurrentScanLast; // surf feature set from odoOptimization
  pcl::PointCloud<PointType>::Ptr laserCloudKeyFrameLastDS; // downsampled keyframe featuer set from odoOptimization
  pcl::PointCloud<PointType>::Ptr laserCloudCurrentScanLastDS; // downsampled surf featuer set from odoOptimization

  pcl::PointCloud<PointType>::Ptr laserCloudOri;
  pcl::PointCloud<PointType>::Ptr coeffSel;

  std::vector<PointType> laserCloudOriKeyFrameVec; // keyframe point holder for parallel computation
  std::vector<PointType> coeffSelKeyFrameVec;
  std::vector<bool> laserCloudOriKeyFrameFlag;
  std::vector<PointType> laserCloudOriCurrentScanVec; // surf point holder for parallel computation
  std::vector<PointType> coeffSelCurrentScanVec;
  std::vector<bool> laserCloudOriCurrentScanFlag;

  pcl::PointCloud<PointType>::Ptr laserCloudKeyFrameFromMap;
  pcl::PointCloud<PointType>::Ptr laserCloudCurrentScanFromMap;
  pcl::PointCloud<PointType>::Ptr laserCloudKeyFrameFromMapDS;
  pcl::PointCloud<PointType>::Ptr laserCloudCurrentScanFromMapDS;

  // pcl::KdTreeFLANN<PointType>::Ptr kdtreeKeyFrameFromMap;
  // pcl::KdTreeFLANN<PointType>::Ptr kdtreeCurrentScanFromMap;

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

  bool aLoopIsClosed = false;
  bool initialized_Flag = false;
  int imuPreintegrationResetId = 0;
  float historyKeyframeFitnessScore;
  bool useImuHeadingInitialization;
  double mappingProcessInterval;
  nav_msgs::msg::Path globalPath;

  Eigen::Affine3f transPointAssociateToMap;

  pcl::PointCloud<PointType>::Ptr cloudGlobalMap;
  pcl::PointCloud<PointType>::Ptr cloudGlobalMapDS;
  pcl::PointCloud<PointType>::Ptr cloudScanForInitialize;

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
