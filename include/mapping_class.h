
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <iostream>
#include "std_msgs/Header.h"
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include "std_msgs/Int8.h"
#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

class mapping_class {

public:
  bool publish;
  bool debug;
  int SLAM_state;
  int error;
  int localisation;
  pcl::PointCloud<pcl::PointXYZ> map;
  std::vector <geometry_msgs::Point> map_points;
  std::string leg_name; /**< Current leg Name */
  std::string cid; /**< Cloud Frame ID */
  
  
  const tf::TransformListener transform_current; /**< Required to apply the base transform to the planar logic*/

  void mapping_publisher();
  void camera_control();
  void world_init();
  void world_update();
  void cloud_storage (const pcl::PCLPointCloud2ConstPtr& cloud_input); /**< Primary function. Takes in a point cloud, saves to SLAMmap */

  void storage_management();
};