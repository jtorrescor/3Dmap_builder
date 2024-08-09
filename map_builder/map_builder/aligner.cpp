#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/icp.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>


class Aligner
{
public:
  Aligner()
  {

    cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("aligned_cloud", 10);
    target_pub = nh_.advertise<sensor_msgs::PointCloud2>("target_cloud", 10);
    in_pub = nh_.advertise<sensor_msgs::PointCloud2>("in_cloud", 10);
    scan_sub_ = nh_.subscribe("scan", 10, &Aligner::scanCallback, this);
    
    cloud_target_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    
    // Create a new point cloud with random points
    cloud_target_->width = 100;
    cloud_target_->height = 1;
    cloud_target_->is_dense = false;
    cloud_target_->points.resize(cloud_target_->width * cloud_target_->height);
    for (size_t i = 0; i < cloud_target_->points.size (); ++i)
    {
        //cloud_target_->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
        cloud_target_->points[i].x = 0;
        cloud_target_->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
        cloud_target_->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
    }
    
    //cloud_target_->points.push_back(pcl::PointXYZ(0, 0, 0));

  }

  void scanCallback(const sensor_msgs::LaserScanConstPtr& scan)
  {

    
    // Convert laser scan to point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
    for (size_t i = 0; i < scan->ranges.size(); i++)
    {
      float range = scan->ranges[i];
      if (range < scan->range_min || range > scan->range_max)
        continue;
      float angle = scan->angle_min + i * scan->angle_increment;
      pcl::PointXYZ point;
      //point.x = range * cos(angle);
      point.x = 0;
      point.z = range * cos(angle);
      point.y = range * sin(angle);
      //point.z = 0;
      cloud_in->points.push_back(point);
    }
    cloud_in->width = cloud_in->points.size();
    cloud_in->height = 1;

    // Align point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(cloud_in);
    icp.setInputTarget(cloud_target_);
    icp.align(*cloud_out);
    Eigen::Matrix4f transformation = icp.getFinalTransformation();

    // Update target cloud
    //cloud_target_ = cloud_out;

    // Publish aligned point cloud
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*cloud_out, msg);
    msg.header.frame_id = "map";
    cloud_pub_.publish(msg);
    
    // Publicar el cloud_target
    sensor_msgs::PointCloud2 target_msg;
    pcl::toROSMsg(*cloud_target_, target_msg);
    target_msg.header.frame_id = "map"; // Cambia "base_link" por el frame_id deseado
    target_pub.publish(target_msg);

    // Publicar el cloud_out
    sensor_msgs::PointCloud2 in_msg;
    pcl::toROSMsg(*cloud_in, in_msg);
    in_msg.header.frame_id = "map"; // Cambia "base_link" por el frame_id deseado
    in_pub.publish(in_msg);
  }
  


private:
  ros::NodeHandle nh_;
  ros::Subscriber scan_sub_;
  ros::Publisher cloud_pub_;
  ros::Publisher target_pub;
  ros::Publisher in_pub;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target_;
};

int main(int argc, char** argv)
{
ros::init(argc, argv, "aligner_node");
Aligner aligner;
ros::spin();
return 0;
}
