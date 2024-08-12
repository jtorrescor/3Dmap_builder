#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <tf/transform_listener.h>
#include <vector>
#include <std_msgs/UInt32.h>

class Constructor
{
public:
  Constructor() : nh_("~"), step_x(0.001)

  {

    cloud_sub_ = nh_.subscribe("/in_cloud", 1, &Constructor::cloudCallback, this);
    odom_sub_ = nh_.subscribe("/gazebo/controllers/diff_drive/odom", 1, &Constructor::odomCallback, this);
    cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("elevated_cloud", 1);
    map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("elevated_map", 1);
    
    // Create timer to publish elevated clouds periodically
    publish_timer_ = nh_.createTimer(ros::Duration(0.1), &Constructor::publishElevatedMap, this);
    
    // Configuración de parámetros
    nh_.param<double>("map_resolution", map_resolution_, 0.001); 	// Resolución del mapa en metros
    nh_.param<double>("x_axis_resolution", step_x, 0.001);		//Resolución en ele eje X

    // Crear objeto voxel grid para reducir el tamaño del mapa
    voxel_grid_.setLeafSize(map_resolution_, map_resolution_, map_resolution_);
    

  }

  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud_msg, *cloud);
    
        // Add elevation to each point
    if (std::abs(elevation_ - prev_elevation) >= step_x)
    {
      for (auto& point : cloud->points)
      {
        point.x += elevation_;
      }
    prev_elevation = elevation_;
    
    // Add elevated cloud to vector
    //elevated_clouds_.push_back(*cloud);
    elevated_clouds_.push_back(boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>(*cloud));
    
    // Publish elevated cloud
    sensor_msgs::PointCloud2 elevated_cloud_msg;
    pcl::toROSMsg(*cloud.get(), elevated_cloud_msg);
    elevated_cloud_msg.header = cloud_msg->header;
    cloud_pub_.publish(elevated_cloud_msg);
    }
  }
  
  void odomCallback(const nav_msgs::OdometryConstPtr& odom_msg)
  {
    // Get elevation from odometry
    elevation_ = odom_msg->pose.pose.position.x;
  }
  
  void publishElevatedMap(const ros::TimerEvent&)
  {
    // Check if there are any clouds to process
    if (elevated_clouds_.empty())
    {
      return;
    }

    // Merge clouds
    pcl::PointCloud<pcl::PointXYZ>::Ptr merged_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (auto& cloud : elevated_clouds_)
    {
      *merged_cloud += *cloud;
    }

    // Filter cloud using voxel grid
    voxel_grid_.setInputCloud(merged_cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    voxel_grid_.filter(*filtered_cloud);

    // Publish elevated map
    sensor_msgs::PointCloud2 elevated_map_msg;
    pcl::toROSMsg(*filtered_cloud, elevated_map_msg);
    elevated_map_msg.header.frame_id = "map";
    elevated_map_msg.header.stamp = ros::Time::now();
    map_pub_.publish(elevated_map_msg);
    
    // Add elevated cloud to final point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr final_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::concatenate(*filtered_cloud, *merged_cloud, *final_cloud);

    // Save final point cloud to PCD file
    pcl::io::savePCDFileBinary("3D_map.pcd", *final_cloud);

    }

private:
  ros::NodeHandle nh_;
  ros::Subscriber cloud_sub_;
  ros::Subscriber odom_sub_;
  ros::Publisher cloud_pub_;
  ros::Publisher map_pub_;
  double map_resolution_;
  pcl::VoxelGrid<pcl::PointXYZ> voxel_grid_;
  //std::vector<pcl::PointCloud<pcl::PointXYZ>> elevated_clouds_;
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> elevated_clouds_;
  ros::Timer publish_timer_;
  tf::TransformListener tf_listener_;
  double elevation_ = 0.0;
  double prev_elevation = 0.0;
  double step_x;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "elevator_node");
  Constructor constructor;
  
  ros::spin();
  return 0;
}

