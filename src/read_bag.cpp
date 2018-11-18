// Part of Stage-2 
// Calculates the transform between aruco_mapping "/world" and hector_mapping "/map"
// Basically, bridges the aruco_mapping and hector_slam package 
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <aruco_mapping.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Pose.h>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

aruco_mapping::ArucoMarker bag_msg;
geometry_msgs::Pose marker_tf;

void poseCallback_aruco(const aruco_mapping::ArucoMarker& msg){
  int total_no_markers = bag_msg.num_of_visible_markers;
  int no_markers = msg.num_of_visible_markers;
  int origin_marker = -1;
  for (int i = 0; i < no_markers; ++i)
  {
    geometry_msgs::Pose tmp = msg.global_marker_poses[i];
    if(tmp.position.x == 0 && tmp.position.y == 0 && tmp.position.z == 0 && tmp.orientation.x == 0 && tmp.orientation.y == 0 && tmp.orientation.z == 0 && tmp.orientation.w == 1){
      origin_marker = msg.marker_ids[i];
      break;
    }
  }
  // ROS_INFO_STREAM(total_no_markers);
  if(origin_marker != -1){
    for(int i=0; i < total_no_markers; i++){
      if(bag_msg.marker_ids[i] == origin_marker){
        marker_tf = bag_msg.global_marker_poses[i];
        // ROS_INFO_STREAM(bag_msg.marker_ids[i]);
        break;
      }
    }
  }
  else{
    ROS_ERROR("Could not find ORIGIN MARKER in /aak_aruco_poses");
  }
  // std::cout << marker_tf.position.x << " " << marker_tf.position.y << " " << marker_tf.position.z << " " << std::endl;
  ROS_INFO_STREAM("Publishing transform from /map to /world");
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(marker_tf.position.x, marker_tf.position.y, marker_tf.position.z));
  transform.setRotation(tf::Quaternion(marker_tf.orientation.x, marker_tf.orientation.y, marker_tf.orientation.z, marker_tf.orientation.w));
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/map", "/world"));
  // ROS_INFO_STREAM("Publishing transform.");

}

int main(int argc, char** argv){
  ros::init(argc, argv, "read_bag");

  rosbag::Bag bag;
  bag.open("/home/fauzan/bags/aruco.bag");
  ROS_INFO_STREAM("Bag opened.");
  std::vector<std::string> topics;
  topics.push_back(std::string("map_aruco_pose"));
  rosbag::View view(bag, rosbag::TopicQuery(topics));
  foreach(rosbag::MessageInstance const m, view){
    aruco_mapping::ArucoMarker::ConstPtr tmp = m.instantiate<aruco_mapping::ArucoMarker>();
    bag_msg.marker_visibile = tmp->marker_visibile;
    bag_msg.num_of_visible_markers = tmp->num_of_visible_markers;
    // std::cout << "lol: " << bag_msg.num_of_visible_markers << std::endl;
    bag_msg.global_camera_pose = tmp->global_camera_pose;
    bag_msg.marker_ids = tmp->marker_ids;
    bag_msg.global_marker_poses = tmp->global_marker_poses;
    if(tmp == NULL)
      break;
  }
  ROS_INFO_STREAM("Read bag file.");

  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe("/aak_aruco_poses", 100, &poseCallback_aruco);
  
  ros::spin();
  bag.close();
  return 0;

}
