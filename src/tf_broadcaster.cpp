#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include <aruco_mapping.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Pose.h>
#include <boost/lexical_cast.hpp>


void poseCallback_aruco(){
  static tf::TransformBroadcaster br;
  static tf::TransformListener listener;

  tf::StampedTransform aruco_camera;
  // listener.waitForTransform("/camera_position", "/world", ros::Time(0), ros::Duration(1));
  try{
    listener.lookupTransform("/camera_position", "/world", ros::Time(0), aruco_camera); // B
  }
  catch (tf::TransformException &ex) {
    ROS_ERROR("%s",ex.what());
  }

  tf::StampedTransform lizi_camera;
  // listener.waitForTransform( "/camera_rgb_optical_frame", "/map", ros::Time(0), ros::Duration(1));
  try{
    listener.lookupTransform("/camera_rgb_optical_frame", "/map", ros::Time(0), lizi_camera); // A
  }
  catch (tf::TransformException &ex) {
    ROS_ERROR("%s",ex.what());
  }

  // tf::Transform final_pub = lizi_camera.inverse() * aruco_camera;
  tf::Transform final_pub = aruco_camera.inverse() * lizi_camera;

  std::cout << "Publishing..." << std::endl;
  br.sendTransform(tf::StampedTransform(final_pub, ros::Time::now(), "/world", "/map"));
}


int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_broadcaster");
  ros::NodeHandle node;


  tf::TransformBroadcaster br;
  tf::TransformListener listener;

  tf::StampedTransform aruco_camera;
  tf::StampedTransform lizi_camera;

  listener.waitForTransform("/camera_position", "/world", ros::Time(0), ros::Duration(1));
  listener.waitForTransform( "/camera_rgb_optical_frame", "/map", ros::Time(0), ros::Duration(1));

  ros::Rate rate(50.0);
  while(node.ok()){
    // poseCallback_aruco();

    // std::cout << "Publishing..." << std::endl;

    try{
      listener.lookupTransform("/camera_position", "/world", ros::Time(0), aruco_camera); // B
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
    }

    try{
      listener.lookupTransform("/camera_rgb_optical_frame", "/map", ros::Time(0), lizi_camera); // A
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
    }

    tf::Transform final_pub = aruco_camera.inverse() * lizi_camera;
    br.sendTransform(tf::StampedTransform(final_pub, ros::Time::now(), "/world", "/map"));

    rate.sleep();
  }

  ros::spin();
  return 0;

}
