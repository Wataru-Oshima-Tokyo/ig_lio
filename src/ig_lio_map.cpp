#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include "std_srvs/srv/trigger.hpp"
#include <boost/filesystem.hpp>

//ig lio
#include "ig_lio/lio.h"
#include "ig_lio/logger.hpp"

class IgLioMapNode : public rclcpp::Node {
public:
    IgLioMapNode() : Node("ig_lio_map_node") {
        this->getParams();
        
        this->keyframe_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "keyframe_scan", 10, std::bind(&IgLioMapNode::keyframeCallback, this, std::placeholders::_1));
        
        this->save_map_service_ = this->create_service<std_srvs::srv::Trigger>(
            "/lio/save_map", std::bind(&IgLioMapNode::saveMapService, this, std::placeholders::_1, std::placeholders::_2));
        
        this->subFloorInfo = this->create_subscription<techshare_ros_pkg2::msg::FloorInfo>(
            "halna/mapping/level", 8,
            std::bind(&IgLioMapNode::floor_info_callback, this, std::placeholders::_1));

    this->world_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("world_map", 100);

        this->ig_lio_map = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    }

private:
    void getParams() {
        this->declare_parameter<std::string>("common.mapLocation", "");
        this->get_parameter("common.mapLocation", this->map_location_);
        this->declare_parameter<std::string>("common.mapName", "ig_lio_map");
        this->get_parameter("common.mapName", this->map_name_);
        this->declare_parameter<std::string>("common.mapFrame", "odom");
        this->get_parameter("common.mapFrame", this->map_frame_);
        this->declare_parameter<int>("common.startBuildingNumber", 0);
        this->get_parameter("common.startBuildingNumber", this->building_number);
        this->declare_parameter<int>("common.startFloorLevel", 0);
        this->get_parameter("common.startFloorLevel", this->floor_level);
        this->declare_parameter<double>("ig_lio_config.map.map_leafsize", 0.5);
        this->get_parameter("ig_lio_config.map.map_leafsize", this->leaf_size_);
    }

    void keyframeCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*msg, *cloud);
        // voxel filter
        this->voxelgrid.setLeafSize(this->leaf_size_, this->leaf_size_, this->leaf_size_);
        this->voxelgrid.setInputCloud(cloud);
        this->voxelgrid.filter(*cloud);
        *this->ig_lio_map += *cloud;

        sensor_msgs::msg::PointCloud2 world_ros;
        pcl::toROSMsg(*this->ig_lio_map, world_ros);
        world_ros.header.stamp = msg->header.stamp;
        world_ros.header.frame_id = this->map_frame_;
        this->world_pub->publish(world_ros);
    }

    void floor_info_callback(const techshare_ros_pkg2::msg::FloorInfo::SharedPtr msg)
    {
        this->building_number = msg->building_number;
        this->floor_level = msg->floor_level;
    }

    void saveMapService(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        std::string save_dir = this->map_location_ + "/" + this->map_name_ + "/floor_" + std::to_string(this->building_number) +"_" + std::to_string(this->floor_level) +"/";
        int unused = system((std::string("mkdir -p ") + save_dir).c_str());
        std::string filePath = save_dir +  "GlobalMap.pcd";
        std::string message="";
        if (boost::filesystem::exists(filePath)) {
            filePath = save_dir + "GlobalMap_new.pcd";
            if (boost::filesystem::exists(filePath)) {
                // Rename the existing file
                std::string newFileName = save_dir + "/GlobalMap_" + std::to_string(save_counter) + ".pcd";
                boost::filesystem::rename(filePath, newFileName);
                save_counter++;
            }
            message = "A new map for "  +  this->map_name_ + " is saved separately since there is a pre-built map already";
        }else{
            
            message = "The first map for " + this->map_name_ + "is saved as GlobalMap";
        }
        int ret = pcl::io::savePCDFileBinary(filePath, *this->ig_lio_map);
        if (ret == 0) {
            std::cout << "File " << filePath << " saved successfully." << std::endl;
            response->success = true;
        } else {
            std::cerr << "Error saving file " << filePath << std::endl;
            response->success = false;
        }

        response->message = message;
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr keyframe_sub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_map_service_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr world_pub;
    rclcpp::Subscription<techshare_ros_pkg2::msg::FloorInfo>::SharedPtr subFloorInfo;
    pcl::PointCloud<pcl::PointXYZ>::Ptr ig_lio_map;

    pcl::VoxelGrid<pcl::PointXYZ> voxelgrid;
    std::string map_location_;
    std::string map_frame_;
    std::string map_name_;
    double leaf_size_;
    int building_number;
    int floor_level;
    int save_counter=0;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<IgLioMapNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
