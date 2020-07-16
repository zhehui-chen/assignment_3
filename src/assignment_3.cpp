#include <math.h>
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/Imu.h>
#include <eigen3/Eigen/Dense>
#include <visualization_msgs/Marker.h>


// initial value:
ros::Publisher pub;
Eigen::Vector3f omega, acc_b, acc_g, vel_g, pos_g(0.0, 0.0, 0.0), gravity_g(0.0, 0.0, 9.8);
Eigen::Matrix3f B, C = Eigen::Matrix3f::Identity();
double now_time, last_time=0, delta_t;
visualization_msgs::Marker line_strip;

//last variables:
Eigen::Vector3f last_acc_g, last_vel_g;
Eigen::Matrix3f last_B;

class IMU{
public:
//  time:
  double imu_time;
//  angular_velocity:
  float angu_vel_x;
  float angu_vel_y;
  float angu_vel_z;
//  linear_acceleration:
  float line_acc_x;
  float line_acc_y;
  float line_acc_z;
} imu;

void pub_to_rviz(Eigen::Vector3f pos_g){
  ros::Rate r(30);

  line_strip.header.frame_id = "map";
  line_strip.header.stamp = ros::Time::now();
  line_strip.ns = "points_and_lines";
  line_strip.action = visualization_msgs::Marker::ADD;
  line_strip.pose.orientation.w = 1.0;

  line_strip.id = 1;

  line_strip.type = visualization_msgs::Marker::LINE_STRIP;

  // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
  line_strip.scale.x = 0.1;

  // Line strip is blue
  line_strip.color.b = 1.0;
  line_strip.color.a = 1.0;

  geometry_msgs::Point p;
  p.x = pos_g(0);
  p.y = pos_g(1);
  p.z = pos_g(2);

  line_strip.points.push_back(p);
  pub.publish(line_strip);
  r.sleep();
}


void callback(const sensor_msgs::Imu::ConstPtr& msg)
{
  float sigma;

//  get info from topic:
  imu.angu_vel_x = msg->angular_velocity.x;
  imu.angu_vel_y = msg->angular_velocity.y;
  imu.angu_vel_z = msg->angular_velocity.z;
  imu.line_acc_x = msg->linear_acceleration.x;
  imu.line_acc_y = msg->linear_acceleration.y;
  imu.line_acc_z = msg->linear_acceleration.z;
  imu.imu_time = msg->header.stamp.toSec();

//  calc the position:
  now_time = imu.imu_time;

  if (last_time != 0){

    delta_t = now_time - last_time;

    acc_b <<  imu.line_acc_x, imu.line_acc_y, imu.line_acc_z;

    B <<  0, -1*imu.angu_vel_z*delta_t, imu.angu_vel_y*delta_t,
          imu.angu_vel_z*delta_t, 0, -1*imu.angu_vel_x*delta_t,
          -1*imu.angu_vel_y*delta_t, imu.angu_vel_x*delta_t, 0;
    B = 0.5*(B + last_B); // midpoint

    omega <<  imu.angu_vel_x, imu.angu_vel_y, imu.angu_vel_z;

    sigma = (omega*delta_t).norm();

    C = C*(Eigen::Matrix3f::Identity() + sin(sigma)/sigma*B + (1-cos(sigma))/(sigma*sigma)*B*B);

    acc_g = C*acc_b;

    vel_g = vel_g + delta_t*(0.5*(acc_g + last_acc_g) - gravity_g); // midpoint

    pos_g = pos_g + delta_t*(0.5*(vel_g + last_vel_g)); // midpoint

    pub_to_rviz(pos_g);
  }

  last_B = B;
  last_acc_g = acc_g;
  last_vel_g = vel_g;
  last_time = now_time;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "assignment_3");
  ros::NodeHandle nh;
  pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1000);
  ros::Subscriber sub = nh.subscribe("imu/data", 1000, callback);
  ros::spin();
}



