// part of stage 1 of gazebo simulation 
// Reading the marker poses from a text file(marker_position.txt) and saving it as an ArucoMessage in a bagfile (aruco.bag)
#include <vector>
#include <string>
#include <fstream>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <rosbag/bag.h>
#include <aruco_mapping.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Pose.h>
#include <boost/lexical_cast.hpp>


int main(int argc, char** argv){

  ros::init(argc, argv, "saving_aruco_wrt_map_from_data_file");

  std::string bag_file_name = "/home/fauzan/bags/aruco_from_data.bag";
  std::string data_file_name = "marker_positions.txt";

  ros::NodeHandle node("~");


  node.getParam("bag_file_name", bag_file_name);
  node.getParam("data_file_name", data_file_name);


  std::ifstream data_file(data_file_name.c_str());


  aruco_mapping::ArucoMarker aruco_msg;
  std::vector<int> marker_ids_vector;

  int marker_id;
  double d_x, d_y, d_z;
  double r_r, r_p, r_y;
  double q_x, q_y, q_z, q_w;
  geometry_msgs::Pose marker_pose;


  /**
   * marker_positions.txt can have Euler or Quarternion angles.
   * Uncomment the required format
   */


  int count = 0;
  
  // Euler
  // while(data_file >> marker_id >> d_x >> d_y >> d_z >> r_r >> r_p >> r_y){

  // Quarterion
  while(data_file >> marker_id >> d_x >> d_y >> d_z >> q_x >> q_y >> q_z >> q_w){

    marker_pose.position.x = d_x;
    marker_pose.position.y = d_y;
    marker_pose.position.z = d_z;

    // Euler
    // marker_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(r_r, r_p, r_y);
    // std::cout << marker_id << "\t";
    // std::cout << marker_pose.orientation.x << "\t";
    // std::cout << marker_pose.orientation.y << "\t";
    // std::cout << marker_pose.orientation.z << "\t";
    // std::cout << marker_pose.orientation.w << std::endl;

    // Quarternion
    marker_pose.orientation.x = q_x;
    marker_pose.orientation.y = q_y;
    marker_pose.orientation.z = q_z;
    marker_pose.orientation.w = q_w;

    aruco_msg.global_marker_poses.push_back(marker_pose);
    marker_ids_vector.push_back(marker_id);

    count++;
  }

  std::cout << "Found " << count << " markers" << std::endl;

  aruco_msg.num_of_visible_markers = count;
  aruco_msg.marker_ids = marker_ids_vector;

  rosbag::Bag bag;
  bag.open(bag_file_name, rosbag::bagmode::Write);
  bag.write("map_aruco_pose", ros::Time::now(), aruco_msg);
  bag.close();

  std::cout << "Done" << std::endl;

  return 0;
}
