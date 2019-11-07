
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/Imu.h"
//#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Twist.h"
#include "tf/transform_broadcaster.h"

class pose_to_odom_node
{
public:
  pose_to_odom_node() // Class constructor
  {
    ros::NodeHandle nh_; // Public nodehandle for pub-sub
    ros::NodeHandle nh_private_("~"); // Private nodehandle for handling parameters

    uint32_t seq_ = 0;

    // Init subscribers
    pose_sub_ = nh_.subscribe("/vrpn_client_node/crazyflie2/pose", 100, &pose_to_odom_node::odom_callback, this);

    // Init publishers
    pose_stamped_pub_ = nh_.advertise<nav_msgs::Odometry>("/crazyflie/odom", 100, false);

    // You must provide the static transforms for these in a launch file!
    odom_out_.header.frame_id = "odom";
    odom_out_.child_frame_id = "";

    // Init covariances grabbed from the parameter server
    init_covariances(nh_private_);
  }

  void odom_callback(const geometry_msgs::PoseStamped::ConstPtr& pose_msg)
  {
    // Populate header
    odom_out_.header.stamp = ros::Time::now();
    //odom_out_.header.seq = seq_;
    //seq_ += 1;


    // Populate position data
    odom_out_.pose.pose.position.x = pose_msg->pose.position.x;
    odom_out_.pose.pose.position.y = pose_msg->pose.position.x;
    odom_out_.pose.pose.position.z = pose_msg->pose.position.z;

    // Populate orientation data
    odom_out_.pose.pose.orientation = pose_msg->pose.orientation;

    geometry_msgs::Twist twist_empty;
    odom_out_.twist.twist = twist_empty;

    // Publish the geometry_msgs/PoseWithCovarianceStamped message
    pose_stamped_pub_.publish(odom_out_);
  }

  // Handy function for initialising covariance matrices from parameters
  void init_covariances(ros::NodeHandle &nh_private_)
  {
    // Create the vectors to store the covariance matrix arrays
    std::vector<double> orientation_covar;
    std::vector<double> ang_vel_covar;
    std::vector<double> linear_accel_covar;
    std::vector<double> pose_covar;
    std::vector<double> twist_covar;

    // Grab the parameters and populate the vectors
    nh_private_.getParam("imu_orientation_covariance", orientation_covar);
    nh_private_.getParam("imu_angular_velocity_covariance", ang_vel_covar);
    nh_private_.getParam("imu_linear_acceleration_covariance", linear_accel_covar);
    nh_private_.getParam("pose_covariance", pose_covar);
    nh_private_.getParam("twist_covariance", twist_covar);

    // Iterate through each vector and populate the respective message fields
    // for (int i = 0; i < orientation_covar.size(); i++)
    //   imu_out_.orientation_covariance[i] = orientation_covar.at(i);

    // for (int i = 0; i < ang_vel_covar.size(); i++)
    //   imu_out_.angular_velocity_covariance[i] = ang_vel_covar.at(i);

    // for (int i = 0; i < linear_accel_covar.size(); i++)
    //   imu_out_.linear_acceleration_covariance[i] = linear_accel_covar.at(i);

    for (int i = 0; i < pose_covar.size(); i++)
      odom_out_.pose.covariance[i] = pose_covar.at(i);

    for (int i = 0; i < twist_covar.size(); i++)
      odom_out_.twist.covariance[i] = twist_covar.at(i);
  }

protected:
  // Subscriber objects
  //ros::Subscriber imu_fusion_sub_;
  ros::Subscriber pose_sub_;

  // Publisher objects
  ros::Publisher pose_stamped_pub_;
  //ros::Publisher imu_pub_;

  // Message objects
  //sensor_msgs::Imu imu_out_;
  nav_msgs::Odometry odom_out_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pose_to_odom");

  pose_to_odom_node adapter;

  ros::spin();
}
