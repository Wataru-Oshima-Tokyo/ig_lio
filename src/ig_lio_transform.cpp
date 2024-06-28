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
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <Eigen/Dense>
// #include <pcl/point_types.h>
// #include <pcl/common/transforms.h>



class RobotOdomProcess : public rclcpp::Node 
{
    public:
    struct Extrinsics {
        struct SE3 {
        Eigen::Vector3f t;
        Eigen::Matrix3f R;
        };
        SE3 robot2imu;
        SE3 robot2lidar;
        Eigen::Matrix4f robot2imu_T;
        Eigen::Matrix4f robot2lidar_T;
    }; Extrinsics extrinsics;
    std::mutex mtx;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subSwitchFlag;
    bool robotOdomForReloc_flag = false;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subLaserOdometry;
    // add sub robot_odom
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subRobotOdom;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subPoseOdom_To_Map;
    tf2::Transform t_map_to_odom;
    tf2::Transform t_odom_to_baselink;

    //用于发布RobotOdomGlobal，由于该变量在两个回调函数中均涉及，故有局部变量改为全局变量
    nav_msgs::msg::Odometry odometry;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubRobotOdomeGlobal;
    // add by zzr 20221210 To test whether the original data upload is disconnected
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_datasuccess_Flag;     
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publasertomap_Re;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubrobot_to_map_relocation;


    double lastrobotodomT_robotodom = -1;

    // add  RobotOdomQueue  队列存储 话题
    std::deque<nav_msgs::msg::Odometry> RobotOdomQueue;
    std::deque<nav_msgs::msg::Odometry> LaserOdomQueue;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tfMap_To_Odom;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tfOdom_To_baselink;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tfBase_link_to_imu;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tfBase_link_to_lidar;

    rclcpp::CallbackGroup::CallbackGroup::SharedPtr callback_group_sub1_;
    rclcpp::CallbackGroup::CallbackGroup::SharedPtr callback_group_sub2_;
    std::vector<double> robot2imu_t, robot2imu_r, robot2lidar_t, robot2lidar_r; 
    std::string package_path_;
    // Set default values for t_imu_lidar and R_imu_lidar
    std::vector<double> default_t_imu_lidar = {0.0, 0.0, 0.0};
    std::vector<double> default_R_imu_lidar = {1.0, 0.0, 0.0,  // First row of identity matrix
                                                0.0, 1.0, 0.0,  // Second row
                                                0.0, 0.0, 1.0}; // Third row
    std::vector<double> t_imu_lidar_v, R_imu_lidar_v; 
    Eigen::Matrix4d T_imu_lidar;
    std::string odom_frame;
    std::string robot_frame;
    std::string imu_frame;
    std::string lidar_frame;
    std::string map_frame;
    bool switchflag;
    RobotOdomProcess(const rclcpp::NodeOptions & options):Node("halna_robotOdomForReloc", options)
    {     
        RCLCPP_INFO(this->get_logger(), "ig lio transfrom node started");

        DeclareParams();
        GetParams();
        allocateMemory();
        setTopics();


        RCLCPP_INFO(this->get_logger(), "Done setting the pub/sub");
        
        RCLCPP_INFO(this->get_logger(), "Done allocating memory");
    }
    void DeclareParams(){

        this->declare_parameter<std::string>("common.odomFrame","odom");
        this->declare_parameter<std::string>("common.robotFrame","base_link");
        this->declare_parameter<std::string>("common.imuFrame", "imu_link");
        this->declare_parameter<std::string>("common.lidarFrame","velodyne");
        this->declare_parameter<std::string>("common.mapFrame","map" );

        // For vector parameters like extrinsic, it's a bit more complex
        // Declare and get extrinsic parameters (vectors)
        this->declare_parameter<std::vector<double>>("ig_lio_config.extrinsics.imu2lidar/t", default_t_imu_lidar);
        this->declare_parameter<std::vector<double>>("ig_lio_config.extrinsics.imu2lidar/r", default_R_imu_lidar);

        this->declare_parameter<std::vector<double>>("ig_lio_config.extrinsics.robot2imu/t", default_t_imu_lidar);
        this->declare_parameter<std::vector<double>>("ig_lio_config.extrinsics.robot2imu/r", default_R_imu_lidar);

        this->declare_parameter<std::vector<double>>("ig_lio_config.extrinsics.robot2lidar/t", default_t_imu_lidar);
        this->declare_parameter<std::vector<double>>("ig_lio_config.extrinsics.robot2lidar/r", default_R_imu_lidar);

    }


        void GetParams(){
            this->get_parameter("common.odomFrame", odom_frame);
            this->get_parameter("common.robotFrame", robot_frame);    
            this->get_parameter("common.imuFrame", imu_frame);
            this->get_parameter("common.lidarFrame", lidar_frame);
            this->get_parameter("common.mapFrame", map_frame);

            this->get_parameter("ig_lio_config.extrinsics.imu2lidar/t", t_imu_lidar_v);
            this->get_parameter("ig_lio_config.extrinsics.imu2lidar/r", R_imu_lidar_v);
            this->get_parameter("ig_lio_config.extrinsics.robot2imu/t", robot2imu_t);
            this->get_parameter("ig_lio_config.extrinsics.robot2imu/r", robot2imu_r);
            this->get_parameter("ig_lio_config.extrinsics.robot2lidar/t", robot2lidar_t);
            this->get_parameter("ig_lio_config.extrinsics.robot2lidar/r", robot2lidar_r);

        }
    void setTopics(){
        callback_group_sub1_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        callback_group_sub2_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        auto sub1_opt = rclcpp::SubscriptionOptions();
        sub1_opt.callback_group = callback_group_sub1_;
        auto sub2_opt = rclcpp::SubscriptionOptions();
        sub2_opt.callback_group = callback_group_sub2_;

        tfOdom_To_baselink = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        tfMap_To_Odom = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        tfBase_link_to_imu = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        tfBase_link_to_lidar = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        subSwitchFlag = this->create_subscription<std_msgs::msg::Bool>("halna/mapping/switchflag", 1, 
                                                    std::bind(&RobotOdomProcess::SwitchFlagHandler,this,std::placeholders::_1));


        subPoseOdom_To_Map = this->create_subscription<geometry_msgs::msg::PoseStamped>("halna/mapping/pose_odomTo_map", 5, 
                                                    std::bind(&RobotOdomProcess::odomToMapPoseHandler,this,std::placeholders::_1), sub1_opt);

        //订阅激光里程计，来自mapOptimization,用两帧之间的的足式里程计得到状态方程，和两帧之间的激光里程记的相对位资（这个位姿用于更新位/姿bias、速度/角速度bias），并更新
        subLaserOdometry = this->create_subscription<nav_msgs::msg::Odometry>("/halna/odometry/robot_odom", 5, 
                                                    std::bind(&RobotOdomProcess::LaserOdometryHandler,this,std::placeholders::_1), sub2_opt);

        // add by zzr 20221210 To test whether the original data upload is disconnected
        pub_datasuccess_Flag = this->create_publisher<std_msgs::msg::Bool>("halna/mapping/robotodomdatasuccess_Flag",1);
        //  发布里程计
        pubRobotOdomeGlobal = this->create_publisher<nav_msgs::msg::Odometry>("halna/odometry/robot_odom", 2000);
    
        publasertomap_Re = this->create_publisher<nav_msgs::msg::Odometry> ("halna/relocation/lidar_to_map", 2000);

        pubrobot_to_map_relocation = this->create_publisher<nav_msgs::msg::Odometry> ("halna/relocation/robot_to_map", 2000);
    }

    void allocateMemory(){
        lastrobotodomT_robotodom = -1;
        RobotOdomQueue.clear();
        LaserOdomQueue.clear();


        tf2::Quaternion q_map_to_odom_;
	    q_map_to_odom_.setRPY(0, 0, 0);

        t_map_to_odom = tf2::Transform(q_map_to_odom_, tf2::Vector3(0, 0, 0));
        t_odom_to_baselink = tf2::Transform(q_map_to_odom_, tf2::Vector3(0, 0, 0));
         RCLCPP_INFO(this->get_logger(), "Done setting setting map to base link tf");


        T_imu_lidar = Eigen::Matrix4d::Identity();
        RCLCPP_INFO(this->get_logger(), "Set identty matrix");
        T_imu_lidar.block<3, 1>(0, 3) =
            Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(
                t_imu_lidar_v.data(), 3, 1);
        T_imu_lidar.block<3, 3>(0, 0) =
            Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(
                R_imu_lidar_v.data(), 3, 3);
         RCLCPP_INFO(this->get_logger(), "Done setting imu to lidar memory");
        this->extrinsics.robot2imu.t = Eigen::Vector3f(robot2imu_t[0], robot2imu_t[1], robot2imu_t[2]);
        this->extrinsics.robot2imu.R = Eigen::Map<const Eigen::Matrix<float, -1, -1, Eigen::RowMajor>>(std::vector<float>(robot2imu_r.begin(), robot2imu_r.end()).data(), 3, 3);
        this->extrinsics.robot2imu_T = Eigen::Matrix4f::Identity();
        this->extrinsics.robot2imu_T.block(0, 3, 3, 1) = this->extrinsics.robot2imu.t;
        this->extrinsics.robot2imu_T.block(0, 0, 3, 3) = this->extrinsics.robot2imu.R;

        this->extrinsics.robot2lidar.t = Eigen::Vector3f(robot2lidar_t[0], robot2lidar_t[1], robot2lidar_t[2]);
        this->extrinsics.robot2lidar.R = Eigen::Map<const Eigen::Matrix<float, -1, -1, Eigen::RowMajor>>(std::vector<float>(robot2lidar_r.begin(), robot2lidar_r.end()).data(), 3, 3);

        this->extrinsics.robot2lidar_T = Eigen::Matrix4f::Identity();
        this->extrinsics.robot2lidar_T.block(0, 3, 3, 1) = this->extrinsics.robot2lidar.t;
        this->extrinsics.robot2lidar_T.block(0, 0, 3, 3) = this->extrinsics.robot2lidar.R;
        RCLCPP_INFO(this->get_logger(), "Done setting all the tf");
    }






    void SwitchFlagHandler(const std_msgs::msg::Bool::SharedPtr switch_flag)
    {
        switchflag = switch_flag->data;
        if(switchflag == false)
            robotOdomForReloc_flag = true;
        if((switchflag == false)&&(robotOdomForReloc_flag == true)){
            robotOdomForReloc_flag = false;
            allocateMemory();
        }    
    }

    template<typename T>
    double ROS_TIME(T msg)
    {
        return msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
    }

    void LaserOdometryHandler(const nav_msgs::msg::Odometry::SharedPtr odomMsg)
    {   
        if(switchflag == false){

            std::lock_guard<std::mutex> lock(mtx);
            double currentCorrectionTime = ROS_TIME(odomMsg);
            nav_msgs::msg::Odometry  thisLaserOdom;

            LaserOdomQueue.push_back(*odomMsg);
            thisLaserOdom = LaserOdomQueue.back();
            if(LaserOdomQueue.size()>1)
            {
                LaserOdomQueue.pop_front();
            }
            odometry = LaserOdomQueue.front();

            //将重定位过程中的tf发布移动到此处，发布频率有400hz降为了5hz
            //publish tf( map to odom,this is adynamic tf,it represents the transformation from lidar odometry coordinate to relocalization coordinate(or mapping coordinate) )
            geometry_msgs::msg::Pose pose_map_to_odom;
            tf2::toMsg(t_map_to_odom, pose_map_to_odom);
            geometry_msgs::msg::TransformStamped map_to_odom;
            map_to_odom.header.stamp = odometry.header.stamp;
            map_to_odom.header.frame_id = this->map_frame;
            map_to_odom.child_frame_id = this->odom_frame;
            map_to_odom.transform.translation.x = pose_map_to_odom.position.x;
            map_to_odom.transform.translation.y = pose_map_to_odom.position.y;
            map_to_odom.transform.translation.z = pose_map_to_odom.position.z;
            map_to_odom.transform.rotation.x = pose_map_to_odom.orientation.x;
            map_to_odom.transform.rotation.y = pose_map_to_odom.orientation.y;
            map_to_odom.transform.rotation.z = pose_map_to_odom.orientation.z;
            map_to_odom.transform.rotation.w = pose_map_to_odom.orientation.w;
            tfMap_To_Odom->sendTransform(map_to_odom);

            // Publish TF( odom to baselink)
            tf2::Quaternion q_tem;
            q_tem.setX(odometry.pose.pose.orientation.x);
            q_tem.setY(odometry.pose.pose.orientation.y);
            q_tem.setZ(odometry.pose.pose.orientation.z);
            q_tem.setW(odometry.pose.pose.orientation.w);
            tf2::Vector3 p_tem(odometry.pose.pose.position.x, odometry.pose.pose.position.y, odometry.pose.pose.position.z);
            t_odom_to_baselink  = tf2::Transform(q_tem, p_tem);

            geometry_msgs::msg::Pose pose_odom_to_baselink;
            tf2::toMsg(t_odom_to_baselink, pose_odom_to_baselink);
            geometry_msgs::msg::TransformStamped trans_odom_to_baselink;

            trans_odom_to_baselink.header.stamp = odometry.header.stamp;
            trans_odom_to_baselink.header.frame_id = this->odom_frame;
            trans_odom_to_baselink.child_frame_id = this->robot_frame;
            trans_odom_to_baselink.transform.translation.x = odometry.pose.pose.position.x;
            trans_odom_to_baselink.transform.translation.y = odometry.pose.pose.position.y;
            trans_odom_to_baselink.transform.translation.z = odometry.pose.pose.position.z;
            trans_odom_to_baselink.transform.rotation.x = odometry.pose.pose.orientation.x;
            trans_odom_to_baselink.transform.rotation.y = odometry.pose.pose.orientation.y;
            trans_odom_to_baselink.transform.rotation.z = odometry.pose.pose.orientation.z;
            trans_odom_to_baselink.transform.rotation.w = odometry.pose.pose.orientation.w;
            tfOdom_To_baselink->sendTransform(trans_odom_to_baselink);

            geometry_msgs::msg::TransformStamped base_link_to_imu;
            base_link_to_imu.header.stamp = odometry.header.stamp;
            base_link_to_imu.header.frame_id = this->robot_frame;
            base_link_to_imu.child_frame_id = this->imu_frame;
            base_link_to_imu.transform.translation.x = this->extrinsics.robot2imu.t[0];
            base_link_to_imu.transform.translation.y = this->extrinsics.robot2imu.t[1];
            base_link_to_imu.transform.translation.z = this->extrinsics.robot2imu.t[2];

            Eigen::Quaternionf q(this->extrinsics.robot2imu.R);
            base_link_to_imu.transform.rotation.w = q.w();
            base_link_to_imu.transform.rotation.x = q.x();
            base_link_to_imu.transform.rotation.y = q.y();
            base_link_to_imu.transform.rotation.z = q.z();

            tfBase_link_to_imu->sendTransform(base_link_to_imu);

            // transform: robot to lidar
            geometry_msgs::msg::TransformStamped base_link_to_lidar;
            base_link_to_lidar.header.stamp = odometry.header.stamp;
            base_link_to_lidar.header.frame_id = this->robot_frame;
            base_link_to_lidar.child_frame_id = this->lidar_frame;

            base_link_to_lidar.transform.translation.x = this->extrinsics.robot2lidar.t[0];
            base_link_to_lidar.transform.translation.y = this->extrinsics.robot2lidar.t[1];
            base_link_to_lidar.transform.translation.z = this->extrinsics.robot2lidar.t[2];
            

            Eigen::Quaternionf qq(this->extrinsics.robot2lidar.R);
            base_link_to_lidar.transform.rotation.w = qq.w();
            base_link_to_lidar.transform.rotation.x = qq.x();
            base_link_to_lidar.transform.rotation.y = qq.y();
            base_link_to_lidar.transform.rotation.z = qq.z();

            tfBase_link_to_lidar->sendTransform(base_link_to_lidar);

            /***********************pub_re_location************************/
            // map to odom  求对应的位姿(x,y,z,roll,yaw,pitch)
            tf2::Quaternion q_map_to_odom;
            double roll_map_to_odom; double pitch_map_to_odom; double yaw_map_to_odom;

            q_map_to_odom.setX(pose_map_to_odom.orientation.x);
            q_map_to_odom.setY(pose_map_to_odom.orientation.y);
            q_map_to_odom.setZ(pose_map_to_odom.orientation.z);
            q_map_to_odom.setW(pose_map_to_odom.orientation.w);
    
            tf2::Matrix3x3(q_map_to_odom).getRPY(roll_map_to_odom, pitch_map_to_odom, yaw_map_to_odom);
            //求对应的变换矩阵
            Eigen::Affine3f T_pose_Odom_Map = pcl::getTransformation(pose_map_to_odom.position.x, pose_map_to_odom.position.y, pose_map_to_odom.position.z, 
                                                roll_map_to_odom, pitch_map_to_odom, yaw_map_to_odom);

            // odom to lidar  求对应的位姿(x,y,z,roll,yaw,pitch)
            tf2::Quaternion q_odom_to_lidar;
            double roll_odom_to_lidar; double pitch_odom_to_lidar; double yaw_odom_to_lidar;

            q_odom_to_lidar.setX(pose_odom_to_baselink.orientation.x);
            q_odom_to_lidar.setY(pose_odom_to_baselink.orientation.y);
            q_odom_to_lidar.setZ(pose_odom_to_baselink.orientation.z);
            q_odom_to_lidar.setW(pose_odom_to_baselink.orientation.w);

            tf2::Matrix3x3(q_odom_to_lidar).getRPY(roll_odom_to_lidar, pitch_odom_to_lidar, yaw_odom_to_lidar);
            //求对应的变换矩阵
            Eigen::Affine3f T_thisPose6DInOdom = pcl::getTransformation(pose_odom_to_baselink.position.x, pose_odom_to_baselink.position.y, pose_odom_to_baselink.position.z, 
                                                roll_odom_to_lidar, pitch_odom_to_lidar, yaw_odom_to_lidar);

            //求lidar在map下的位姿
            Eigen::Affine3f T_poseInMap = T_pose_Odom_Map * T_thisPose6DInOdom;
            float delta_x1, delta_y1, delta_z1, delta_roll1, delta_pitch1, delta_yaw1;
            pcl::getTranslationAndEulerAngles (T_poseInMap, delta_x1, delta_y1, delta_z1, delta_roll1, delta_pitch1, delta_yaw1);
            // Publish re_location lidar_to_map
            nav_msgs::msg::Odometry lasertomap_Re;
            lasertomap_Re.header.stamp = odometry.header.stamp;
            lasertomap_Re.header.frame_id = this->map_frame;
            lasertomap_Re.child_frame_id = this->lidar_frame;
            lasertomap_Re.pose.pose.position.x = delta_x1 ;
            lasertomap_Re.pose.pose.position.y = delta_y1 ;
            lasertomap_Re.pose.pose.position.z = delta_z1 ;

            tf2::Quaternion q1_1;
            q1_1.setRPY(delta_roll1 , delta_pitch1 , delta_yaw1 );
        
            lasertomap_Re.pose.pose.orientation.x = q1_1.getX();
            lasertomap_Re.pose.pose.orientation.y = q1_1.getY();
            lasertomap_Re.pose.pose.orientation.z = q1_1.getZ();
            lasertomap_Re.pose.pose.orientation.w = q1_1.getW();

            publasertomap_Re->publish(lasertomap_Re);

            //base_link to map
            Eigen::Affine3f T_odom_to_base_link = pcl::getTransformation(-this->extrinsics.robot2lidar.t[0],-this->extrinsics.robot2lidar.t[1],-this->extrinsics.robot2lidar.t[2], 0, 0, 0);
            Eigen::Affine3f T_base_linkInMap = T_poseInMap * T_odom_to_base_link;

            float delta_x, delta_y, delta_z, delta_roll, delta_pitch, delta_yaw;
            pcl::getTranslationAndEulerAngles (T_base_linkInMap, delta_x, delta_y, delta_z, delta_roll, delta_pitch, delta_yaw);
            // Publish re_location lidar_to_map
            nav_msgs::msg::Odometry robot_to_map_relocation;
            robot_to_map_relocation.header.stamp = odometry.header.stamp;
            robot_to_map_relocation.header.frame_id = this->map_frame;
            robot_to_map_relocation.child_frame_id = this->robot_frame;
            robot_to_map_relocation.pose.pose.position.x = delta_x ;
            robot_to_map_relocation.pose.pose.position.y = delta_y ;
            robot_to_map_relocation.pose.pose.position.z = delta_z ;

            tf2::Quaternion q1_;
            q1_.setRPY(delta_roll , delta_pitch , delta_yaw );
        
            robot_to_map_relocation.pose.pose.orientation.x = q1_.getX();
            robot_to_map_relocation.pose.pose.orientation.y = q1_.getY();
            robot_to_map_relocation.pose.pose.orientation.z = q1_.getZ();
            robot_to_map_relocation.pose.pose.orientation.w = q1_.getW();

            pubrobot_to_map_relocation->publish(robot_to_map_relocation);
            /***********************pub_re_location end************************/
            // /***********************pub_re_location tf************************/
            // tf2::Quaternion q_odom_to_robotcenter_1;
            // q_odom_to_robotcenter_1.setRPY(delta_roll , delta_pitch , delta_yaw );

            // tf2::Transform t_odom_to_robotcenter_ = tf2::Transform(q_odom_to_robotcenter_1, tf2::Vector3(delta_x ,
            // delta_y , delta_z ));

            // geometry_msgs::msg::Pose pose_odom_to_robotcenter__;
            // tf2::toMsg(t_odom_to_robotcenter_, pose_odom_to_robotcenter__);
            // geometry_msgs::msg::TransformStamped odom_to_robotCenter_;

            // odom_to_robotCenter_.header.stamp = odometry.header.stamp;
            // odom_to_robotCenter_.header.frame_id = mapFrame;
            // odom_to_robotCenter_.child_frame_id = "relocation";
            // odom_to_robotCenter_.transform.translation.x = pose_odom_to_robotcenter__.position.x;
            // odom_to_robotCenter_.transform.translation.y = pose_odom_to_robotcenter__.position.y;
            // odom_to_robotCenter_.transform.translation.z = pose_odom_to_robotcenter__.position.z;
            // odom_to_robotCenter_.transform.rotation.x = pose_odom_to_robotcenter__.orientation.x;
            // odom_to_robotCenter_.transform.rotation.y = pose_odom_to_robotcenter__.orientation.y;
            // odom_to_robotCenter_.transform.rotation.z = pose_odom_to_robotcenter__.orientation.z;
            // odom_to_robotCenter_.transform.rotation.w = pose_odom_to_robotcenter__.orientation.w;
            // tfBase_link_to_imu->sendTransform(odom_to_robotCenter_);
            /***********************************************/
        }else{
            return; 
        }
    }

    /********************publish tf (map to odom) dynamic*******************/
    void odomToMapPoseHandler(const geometry_msgs::msg::PoseStamped::SharedPtr poseOdomToMapmsg)
    {   
        if(switchflag == false){
            std::lock_guard<std::mutex> lock(mtx);
            tf2::Quaternion q_tem;
            q_tem.setX(poseOdomToMapmsg->pose.orientation.x);
            q_tem.setY(poseOdomToMapmsg->pose.orientation.y);
            q_tem.setZ(poseOdomToMapmsg->pose.orientation.z);
            q_tem.setW(poseOdomToMapmsg->pose.orientation.w);
            tf2::Vector3 p_tem(poseOdomToMapmsg->pose.position.x, poseOdomToMapmsg->pose.position.y, poseOdomToMapmsg->pose.position.z);
            t_map_to_odom    = tf2::Transform(q_tem, p_tem);
        }else{
            return;
        }
      
    }

    // void RobotOdometryHandler(const nav_msgs::msg::Odometry::SharedPtr odometryMsg)
    // {
    //     if(switchflag == false){

    //         std::lock_guard<std::mutex> lock(mtx);
    //         // 当前帧足式里程计
    //         nav_msgs::msg::Odometry  thisRobotOdom;
    //         RobotOdomQueue.push_back(*odometryMsg);
    //         thisRobotOdom = RobotOdomQueue.back();

    //         // add by zzr 20221210 To test whether the original data upload is disconnected
    //         double current_robotodomTime = ROS_TIME(&thisRobotOdom);
    //         double dt_ = (lastrobotodomT_robotodom < 0) ? (1.0 / 500.0) : (current_robotodomTime - lastrobotodomT_robotodom);
    //         if(dt_ > 0.1 ){
    //             std_msgs::msg::Bool InitialFlag;
    //             InitialFlag.data = false;
    //             pub_datasuccess_Flag->publish(InitialFlag);
    //         }
    //     lastrobotodomT_robotodom = current_robotodomTime;


    //         if (!LaserOdomQueue.empty())
    //         {
    //             // pop and integrate imu data that is between two optimizations
    //             nav_msgs::msg::Odometry  *LastLaserOdom= &LaserOdomQueue.front();
        
    //             double LastLaserTime = ROS_TIME(LastLaserOdom);

    //             while (!RobotOdomQueue.empty())
    //             {
    //                 if ((RobotOdomQueue.front().header.stamp.sec + RobotOdomQueue.front().header.stamp.nanosec * 1e-9) < (LastLaserTime - 0.01))
    //                     RobotOdomQueue.pop_front();
    //                 else
    //                     break;
    //             }
    //         }
    //         else 
    //             return;

    //         // 激光帧对应的帧足式里程计
    //         nav_msgs::msg::Odometry  LaserOdom= LaserOdomQueue.front();
    //         nav_msgs::msg::Odometry  BeforeRobotOdom;
    //         BeforeRobotOdom = RobotOdomQueue.front(); 

    //         // 激光帧对应的帧足式里程计
    //         Eigen::Matrix3d before_R;
    //         Eigen::Matrix3d this_R;
    //         Eigen::Matrix3d Laser_R;
    //         Eigen::Vector3d P1,P2,P21;
    //         Eigen::Vector3d LaserP1,LaserP2;
    //         Eigen::Matrix3d Laser_R2;

    //         Eigen::Vector3d delat_p;
    //         delat_p(0)=imu_to_lidar[0];
    //         delat_p(1)=imu_to_lidar[1];
    //         delat_p(2)=imu_to_lidar[2];

    //         LaserP1(0) = LaserOdom.pose.pose.position.x;
    //         LaserP1(1) = LaserOdom.pose.pose.position.y;
    //         LaserP1(2) = LaserOdom.pose.pose.position.z;

    //         P1(0)= BeforeRobotOdom.pose.pose.position.x;
    //         P1(1)= BeforeRobotOdom.pose.pose.position.y;
    //         P1(2)= BeforeRobotOdom.pose.pose.position.z;

    //         P2(0)= thisRobotOdom.pose.pose.position.x;
    //         P2(1)= thisRobotOdom.pose.pose.position.y;
    //         P2(2)= thisRobotOdom.pose.pose.position.z;

    //         P21=P2-P1;
    
    //         Eigen::Quaterniond before_q1(BeforeRobotOdom.pose.pose.orientation.w, BeforeRobotOdom.pose.pose.orientation.x, BeforeRobotOdom.pose.pose.orientation.y, BeforeRobotOdom.pose.pose.orientation.z);
    //         Eigen::Quaterniond this_q2(thisRobotOdom.pose.pose.orientation.w, thisRobotOdom.pose.pose.orientation.x, thisRobotOdom.pose.pose.orientation.y, thisRobotOdom.pose.pose.orientation.z);
    //         Eigen::Quaterniond Laser_q1(LaserOdom.pose.pose.orientation.w, LaserOdom.pose.pose.orientation.x, LaserOdom.pose.pose.orientation.y, LaserOdom.pose.pose.orientation.z);
    //         //Eigen::Quaterniond  q21;
    //         Eigen::Quaterniond Laser_q2;   
    //         //before_q1.matrix()
    //         Laser_R2 = Laser_q1.matrix() * before_q1.matrix().transpose() * this_q2.matrix();
    //         // Laser_R2 = this_q2.matrix() * before_q1.matrix().transpose() * Laser_q1.matrix();
    //         Laser_q2 = Laser_R2;
    //         LaserP2 = LaserP1 + Laser_q1.matrix() * before_q1.matrix().transpose() * P21 + Laser_q1.matrix() * delat_p - Laser_R2 * delat_p;
    //         // publish odometry
    //         // nav_msgs::msg::Odometry odometry;
    //         odometry.header.stamp = thisRobotOdom.header.stamp;
    //         odometry.header.frame_id = odometryFrame;
    //         odometry.child_frame_id = baseFrame;
    //         odometry.pose.pose.position.x = LaserP2(0);
    //         odometry.pose.pose.position.y = LaserP2(1);
    //         odometry.pose.pose.position.z = LaserP2(2);
    //         odometry.pose.pose.orientation.x = Laser_q2.x();
    //         odometry.pose.pose.orientation.y = Laser_q2.y();
    //         odometry.pose.pose.orientation.z = Laser_q2.z();
    //         odometry.pose.pose.orientation.w = Laser_q2.w();

    //         odometry.twist.twist.linear.x = thisRobotOdom.twist.twist.linear.x;
    //         odometry.twist.twist.linear.y = thisRobotOdom.twist.twist.linear.y;
    //         odometry.twist.twist.linear.z = thisRobotOdom.twist.twist.linear.z;
    //         odometry.twist.twist.angular.x = thisRobotOdom.twist.twist.angular.x;
    //         odometry.twist.twist.angular.y = thisRobotOdom.twist.twist.angular.y;
    //         odometry.twist.twist.angular.z = thisRobotOdom.twist.twist.angular.z;      

    //         pubRobotOdomeGlobal->publish(odometry);
    //     }else{
    //         return;
    //     }
    // }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    options.use_intra_process_comms(true);

    size_t num_threads = 3; // Example: Setting the desired number of threads.
    rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), num_threads);
    auto RobotP = std::make_shared<RobotOdomProcess>(options);
    exec.add_node(RobotP);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\033[1;32m----> Transform started.\033[0m");
    exec.spin();
    rclcpp::shutdown();
    return 0;
}
