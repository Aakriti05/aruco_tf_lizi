#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <aruco_mapping.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/Marker.h>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

aruco_mapping::ArucoMarker bag_msg;
ros::Publisher marker_visualization_pub_;

// void poseCallback_aruco(const aruco_mapping::ArucoMarker& msg){
//   int total_no_markers = bag_msg.num_of_visible_markers;
//   int no_markers = msg.num_of_visible_markers;
//   int origin_marker = -1;
//   for (int i = 0; i < no_markers; ++i)
//   {
//     geometry_msgs::Pose tmp = msg.global_marker_poses[i];
//     if(tmp.position.x == 0 && tmp.position.y == 0 && tmp.position.z == 0 && tmp.orientation.x == 0 && tmp.orientation.y == 0 && tmp.orientation.z == 0 && tmp.orientation.w == 1){
//       origin_marker = msg.marker_ids[i];
//       break;
//     }
//   }

//   // ROS_INFO_STREAM(origin_marker);
//   if(origin_marker != -1){
//     for(int i=0; i < total_no_markers; i++){
//       if(bag_msg.marker_ids[i] == origin_marker){
//         marker_tf = bag_msg.global_marker_poses[i];
//         break;
//       }
//     }
//   }
//   // std::cout << marker_tf.position.x << " " << marker_tf.position.y << " " << marker_tf.position.z << " " << std::endl;
//   ROS_INFO_STREAM("Publishing transform...");
//   static tf::TransformBroadcaster br;
//   tf::Transform transform;
//   transform.setOrigin(tf::Vector3(marker_tf.position.x, marker_tf.position.y, marker_tf.position.z));
//   transform.setRotation(tf::Quaternion(marker_tf.orientation.x, marker_tf.orientation.y, marker_tf.orientation.z, marker_tf.orientation.w));
//   br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/map", "/world"));
//   // ROS_INFO_STREAM("Publishing transform.");
// }

void publishMarker(geometry_msgs::Pose marker_pose, int marker_id)
{
  static visualization_msgs::Marker vis_marker;
  
  vis_marker.header.frame_id = "map";
  // std::cout << "lol" << std::endl;
  vis_marker.header.stamp = ros::Time::now();
  vis_marker.ns = "basic_shapes";
  vis_marker.id = marker_id;
  vis_marker.type = visualization_msgs::Marker::CUBE;
  vis_marker.action = visualization_msgs::Marker::ADD;
  vis_marker.pose = marker_pose;

  vis_marker.scale.x = 0.189;
  vis_marker.scale.y = 0.189;
  vis_marker.scale.z = 0.01;

  vis_marker.color.r = 1.0;
  vis_marker.color.g = 1.0;
  vis_marker.color.b = 1.0;
  vis_marker.color.a = 1.0;

  vis_marker.lifetime = ros::Duration();

  marker_visualization_pub_.publish(vis_marker);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "view_markers");
  ros::NodeHandle node;

  marker_visualization_pub_ = node.advertise<visualization_msgs::Marker>("view_all_markers",1);

  rosbag::Bag bag;
  bag.open("/home/lizi/bags/aruco.bag");
  ROS_INFO_STREAM("Bag opened.");
  std::vector<std::string> topics;
  topics.push_back(std::string("map_aruco_pose"));
  rosbag::View view(bag, rosbag::TopicQuery(topics));
  // std::cout << view.size() << std::endl;
  foreach(rosbag::MessageInstance const m, view){
    aruco_mapping::ArucoMarker::ConstPtr tmp = m.instantiate<aruco_mapping::ArucoMarker>();
    bag_msg.marker_visibile = tmp->marker_visibile;
    bag_msg.num_of_visible_markers = tmp->num_of_visible_markers;
    // std::cout << "lol: " << tmp->num_of_visible_markers << std::endl;
    bag_msg.global_camera_pose = tmp->global_camera_pose;
    bag_msg.marker_ids = tmp->marker_ids;
    bag_msg.global_marker_poses = tmp->global_marker_poses;
    if(tmp == NULL)
      break;
  }
  ROS_INFO_STREAM("Read bag file.");

  geometry_msgs::Pose tmp;
  tf::Transform marker_transform;
  tf::TransformListener listener;
  tf::StampedTransform aruco_camera;
  // listener.waitForTransform("/map", "/world", ros::Time(0), ros::Duration(1.0));
  // try{
  //   listener.lookupTransform("/map", "/world", ros::Time(0), aruco_camera); // B
  // }
  // catch (tf::TransformException &ex) {
  //   ROS_ERROR("%s",ex.what());
  // }

  // std::cout << bag_msg.num_of_visible_markers << std::endl;

  while(ros::ok()){
    for(int i=0; i<bag_msg.num_of_visible_markers; i++){
    // marker_transform.setOrigin(tf::Vector3(bag_msg.global_marker_poses[i].position.x, bag_msg.global_marker_poses[i].position.y, bag_msg.global_marker_poses[i].position.z) );
    // marker_transform.setRotation(tf::Quaternion(bag_msg.global_marker_poses[i].orientation.x, bag_msg.global_marker_poses[i].orientation.y, bag_msg.global_marker_poses[i].orientation.z, bag_msg.global_marker_poses[i].orientation.w));

    // marker_transform = aruco_camera * marker_transform;

    // tmp.position.x = marker_transform.getOrigin().getX();
    // tmp.position.y = marker_transform.getOrigin().getY();
    // tmp.position.z = marker_transform.getOrigin().getZ();
    // tmp.orientation.x = marker_transform.getRotation().getX();
    // tmp.orientation.y = marker_transform.getRotation().getY();
    // tmp.orientation.z = marker_transform.getRotation().getZ();
    // tmp.orientation.w = marker_transform.getRotation().getW(); 


      publishMarker(bag_msg.global_marker_poses[i], bag_msg.marker_ids[i]);
    }
  }
  
  ros::spin();
  bag.close();
  return 0;

}
