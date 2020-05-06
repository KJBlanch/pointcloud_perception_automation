#include "mapping_class.h"

void mapping_class::world_init () {
//If no other function has created the /world frame, create one. 
/*
  ros::init(argc, argv, "world");
  ros::NodeHandle node;

  tf::TransformBroadcaster br;
  tf::Transform transform;

  ros::Rate rate(10.0);

  while (node.ok()){
    transform.setOrigin( tf::Vector3(0.0, 2.0, 0.0) );
    transform.setRotation( tf::Quaternion(0, 0, 0, 1) );
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "turtle1", "carrot1"));
    rate.sleep();
  }
  return 0;
*/
}

void mapping_class::world_update() {
//listen for Pose Change and update accordingly. 
    

    //Update JSON File. 
}

void mapping_class::camera_control() {
  ros::NodeHandle nh;
  ros::Subscriber point1;
  if (this->leg_name == "AL") point1 = nh.subscribe ("/pico_flexx_AL_leg/points", 1, &mapping_class::cloud_storage, this);
  if (this->leg_name == "AR") point1 = nh.subscribe ("/pico_flexx_AR_leg/points", 1, &mapping_class::cloud_storage, this);
  if (this->leg_name == "BL") point1 = nh.subscribe ("/pico_flexx_BL_leg/points", 1, &mapping_class::cloud_storage, this);
  if (this->leg_name == "BR") point1 = nh.subscribe ("/pico_flexx_BR_leg/points", 1, &mapping_class::cloud_storage, this);
  
  //subscribe to publish node
  while(this->publish == 0) ros::spinOnce();
}

void mapping_class::cloud_storage (const pcl::PCLPointCloud2ConstPtr& cloud_input) { //Takes input of PointCloud2 Datatype.
  //std::cout << "Testing Diagnosis Text - Cloud_CB Start" << std::endl;

  //Check to see which frame to transform to. 
  std::string transform_frame;
  
  if (this->localisation == 0) {
      transform_frame = "/map"; //World init, transform to world
  } else if (this->localisation == 1) {
      transform_frame = "/odom_ideal"; //No world init, transform to base_link
  } else {
      transform_frame = "/base_link"; //For further dev options.
  }
  
  
  pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloud_input);
  sor.setLeafSize (0.005f, 0.005f, 0.005f);
  sor.filter (*cloud_filtered);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tf (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(*cloud_filtered, *cloud_tf);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sla (new pcl::PointCloud<pcl::PointXYZ>);
  ros::Time header_time = pcl_conversions::fromPCL(cloud_tf->header.stamp);
  this->transform_current.waitForTransform(transform_frame, cloud_tf->header.frame_id, header_time, ros::Duration(0.1));
  pcl_ros::transformPointCloud(transform_frame, *cloud_tf, *cloud_sla, this->transform_current);
  this->cid = cloud_sla->header.frame_id;

  if (this->map.points.size() == 0) {
    this->map = *cloud_sla;
  } else {
    this->map = this->map + *cloud_sla;
  }

  cloud_filtered.reset(new pcl::PCLPointCloud2 ());
  cloud_tf.reset(new pcl::PointCloud<pcl::PointXYZ>);
  cloud_sla.reset(new pcl::PointCloud<pcl::PointXYZ>);

}

void mapping_class::storage_management() {
  //Downsample all points.
  pcl::PCLPointCloud2::Ptr cloud_s_m (new pcl::PCLPointCloud2 ());
  pcl::PCLPointCloud2::Ptr cloud_s_m_update (new pcl::PCLPointCloud2 ());
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_s_m_update_conv (new pcl::PointCloud<pcl::PointXYZ>);
  
  pcl::toPCLPointCloud2(this->map, *cloud_s_m);
  
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloud_s_m);
  sor.setLeafSize (0.005f, 0.005f, 0.005f);
  sor.filter (*cloud_s_m_update);
  
  pcl::fromPCLPointCloud2(*cloud_s_m_update, *cloud_s_m_update_conv);

  this->map = *cloud_s_m_update_conv;

  cloud_s_m.reset(new pcl::PCLPointCloud2 ());
  cloud_s_m_update.reset(new pcl::PCLPointCloud2 ());
  cloud_s_m_update_conv.reset(new pcl::PointCloud<pcl::PointXYZ>);


    //Need to read how to handle storage. Thinking JSON??
}