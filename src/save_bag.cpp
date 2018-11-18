// part of Stage-1 of real-time 
// Saves all the markers contained in the topic /aak_aruco_poses to a bag file
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <rosbag/bag.h>
#include <aruco_mapping.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Pose.h>
#include <boost/lexical_cast.hpp>


void poseCallback_aruco(const aruco_mapping::ArucoMarker& msg){
  static tf::TransformBroadcaster br;

  aruco_mapping::ArucoMarker mod_msg;

  rosbag::Bag bag;
  bag.open("/home/fauzan/bags/aruco.bag", rosbag::bagmode::Write);

  tf::TransformListener listener;
  tf::StampedTransform aruco_camera;
  listener.waitForTransform("/map", "/world", ros::Time(0), ros::Duration(5.0));
  try{
    listener.lookupTransform("/map", "/world", ros::Time(0), aruco_camera); // B
  }
  catch (tf::TransformException &ex) {
    ROS_ERROR("%s",ex.what());
  }

  tf::Transform cam_transform;
  int k;
  k = msg.num_of_visible_markers;

  tf::Transform marker_transform;
  geometry_msgs::Pose tmp;
  for(int i=0;i<k;i++){
  

    marker_transform.setOrigin(tf::Vector3(msg.global_marker_poses[i].position.x, msg.global_marker_poses[i].position.y, msg.global_marker_poses[i].position.z) );
    marker_transform.setRotation(tf::Quaternion(msg.global_marker_poses[i].orientation.x, msg.global_marker_poses[i].orientation.y, msg.global_marker_poses[i].orientation.z, msg.global_marker_poses[i].orientation.w));
    marker_transform = aruco_camera * marker_transform;
    
    tmp.position.x = marker_transform.getOrigin().getX();
    tmp.position.y = marker_transform.getOrigin().getY();
    tmp.position.z = marker_transform.getOrigin().getZ();
    tmp.orientation.x = marker_transform.getRotation().getX();
    tmp.orientation.y = marker_transform.getRotation().getY();
    tmp.orientation.z = marker_transform.getRotation().getZ();
    tmp.orientation.w = marker_transform.getRotation().getW();

    mod_msg.global_marker_poses.push_back(tmp);

  }
  mod_msg.num_of_visible_markers = msg.num_of_visible_markers;
  mod_msg.marker_ids = msg.marker_ids;

  bag.write("map_aruco_pose", ros::Time::now(), mod_msg);

  bag.close();

  ros::shutdown();
}


int main(int argc, char** argv){
  ros::init(argc, argv, "saving_aruco_wrt_map");

  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe("/aak_aruco_poses", 100, &poseCallback_aruco);


  ros::spin();
  return 0;

}
