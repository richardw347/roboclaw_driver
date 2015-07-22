#include <ros/ros.h>
#include <numeric>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <std_srvs/Empty.h>
#include <std_srvs/EmptyRequest.h>
#include <std_srvs/EmptyResponse.h>
#include "RoboClaw.h"
#include <boost/thread/mutex.hpp>
#include <roboclaw_driver/RoboClawState.h>

#define address 0x80
#define DIAGNOSTICS_DELAY 2.0
#define TWIST_CMD_TIMEOUT 3.0
#define MAX_ERRORS 5

class RoboclawNode{
public:
  RoboclawNode(): nh(), priv_nh("~") {
    boost::mutex::scoped_lock lock(claw_mutex_);
    priv_nh.param<std::string>("port", port, "/dev/roboclaw");
    priv_nh.param<std::string>("base_frame_id", base_frame_id, "base_footprint");
    if(!priv_nh.getParam("baud_rate", baud_rate)){
        baud_rate = 1000000;
      }
    if(!priv_nh.getParam("rate", update_rate)){
        update_rate = 20;
      }
    if(!priv_nh.getParam("base_width", base_width)){
        base_width = 0.265;
      }
    if(!priv_nh.getParam("ticks_per_metre", ticks_per_m)){
        ticks_per_m = 21738;
      }
    if(!priv_nh.getParam("robot_direction", robot_dir)){
        robot_dir = 0;
    }
    if(!priv_nh.getParam("max_acceleration", max_accel)){
        max_accel = 1.0;
    }


    ROS_INFO_STREAM("Starting roboclaw node with params:");
    ROS_INFO_STREAM("Port:\t" << port);
    ROS_INFO_STREAM("Baud rate:\t" << baud_rate);
    ROS_INFO_STREAM("Base Width:\t" << base_width);
    ROS_INFO_STREAM("Ticks Per Metre:\t" << ticks_per_m);

    serial_errs = 0;
    ser.reset(new USBSerial());
    open_usb();
    claw.reset(new RoboClaw(ser.get()));

    max_accel_qpps = max_accel * ticks_per_m;
    last_motor = ros::Time::now();
    target_left_qpps = target_right_qpps = 0;
    vx = vth = 0;
    last_odom  = ros::Time::now();

    quaternion.x = 0.0;
    quaternion.y = 0.0;

    odom.header.frame_id = "odom";
    odom.child_frame_id = base_frame_id;
    odom.pose.pose.position.z = 0.0;

    cmd_vel_sub = nh.subscribe("cmd_vel", 1, &RoboclawNode::twistCb, this);
    odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 10);
    joint_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 1);
    state_pub = nh.advertise<roboclaw_driver::RoboClawState>("roboclaw_state", 1);

    try{
      claw->ReadVersion(address, &roboclaw_version);
      ROS_INFO_STREAM("Connected to: " << roboclaw_version);
    } catch (USBSerial::Exception &e) {
      ROS_WARN("Problem setting PID constants (error=%s)", e.what());
      serial_error();
    }

    js.name.push_back("base_l_wheel_joint");
    js.name.push_back("base_r_wheel_joint");
    js.position.push_back(0.0);
    js.position.push_back(0.0);
    js.effort.push_back(0.0);
    js.effort.push_back(0.0);
    js.header.frame_id = "base_link";

    calib_server = nh.advertiseService("motor_calibrate", &RoboclawNode::calib_callback, this);

  }

  void open_usb() {
    ROS_INFO("Connecting to %s...", port.c_str());
    ros::Time start = ros::Time::now();
    double notify_every = 10.0;
    double check_every = 0.25;
    std::string last_msg;
    while (ros::ok()) {
        try {
          ser->Open(port.c_str());
          ROS_INFO("Connected to %s", port.c_str());
          break;
        } catch (USBSerial::Exception &e) {
          last_msg = e.what();
        }
        ros::Duration(check_every).sleep();
        double dur = (ros::Time::now() - start).toSec();
        if (dur > notify_every) {
            ROS_WARN_THROTTLE(notify_every,
                              "Haven't connected to %s in %.2f seconds."
                              "  Last error=\n%s",
                              port.c_str(), dur, last_msg.c_str());
          }
      }
  }

  void serial_error() {
    serial_errs += 1;
    if (serial_errs == MAX_ERRORS) {
        ROS_ERROR("Several errors from roboclaw, restarting");
        roboclaw_restart_usb();
        open_usb();
        serial_errs = 0;
      }
  }

  bool calib_callback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response){
    ROS_INFO_STREAM("calibrating motors");
    ros::Duration wait(1.0);
    geometry_msgs::Twist calib;
    calib.angular.z = 0.3;
    this->twistCb(calib);
    wait.sleep();
    calib.angular.z = -0.3;
    this->twistCb(calib);
    wait.sleep();
    calib.angular.z = 0.0;
    this->twistCb(calib);
    return true;
  }

  void upateOdom(){

    boost::mutex::scoped_lock lock(claw_mutex_);

    ros::Time now = ros::Time::now();
    double dt = (now-last_odom).toSec();

    if(dt > 10.0) {
        last_odom = now;
        return;
      }

    last_odom = now;


    uint8_t status;
    int32_t speed;
    bool valid;
    int32_t left_qpps, right_qpps = 0;

    try {
      speed = claw->ReadISpeedM1(address, &status, &valid) * 125;
    } catch (USBSerial::Exception &e) {
      ROS_WARN("Problem reading motor 1 speed (error=%s)", e.what());
      serial_error();
      return;
    }

    if (valid && (status == 0 || status == 1)) {
        left_qpps = speed;
      } else {
        ROS_INFO("M1 valid: %d, status: %d", valid, status);
	ROS_WARN("Invalid data from motor 1");
        serial_error();
        return;
      }

    try {
      speed = claw->ReadISpeedM2(address, &status, &valid) * 125;
    } catch(USBSerial::Exception &e) {
      ROS_WARN("Problem reading motor 2 speed (error=%s)", e.what());
      serial_error();
      return;
    }

    if (valid && (status == 0 || status == 1)) {
        right_qpps = speed;
      } else {
	ROS_INFO("M1 valid: %d, status: %d", valid, status);
        ROS_WARN("Invalid data from motor 2");
        serial_error();
        return;
      }

    double left_speed, right_speed = 0.0;

    if (robot_dir){
    	left_speed = left_qpps / ticks_per_m;
	right_speed = right_qpps / ticks_per_m;
    } else {
    	right_speed = left_qpps / ticks_per_m;
	left_speed = right_qpps / ticks_per_m;
    }
    vx = (left_speed + right_speed) / 2.0;
    vth = (right_speed - left_speed) / base_width;

    state.linear_velocity = vx;
    state.angular_velocity = vth;

    double dx, dy, dth;
    dx = vx * dt * cos(theta);
    dy = vx * dt * sin(theta);
    dth = vth * dt;

    x += dx;
    y += dy;
    theta += dth;

    tf::Transform transform;
    transform.setOrigin(tf::Vector3(x,y,0.0));
    tf::Quaternion q;
    q.setRPY(0.0,0.0,theta);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, now, "odom", base_frame_id));

    odom.header.stamp = now;
    odom.header.frame_id = "odom";
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.orientation.x = odom.pose.pose.orientation.y = 0;
    odom.pose.pose.orientation.z = sin(theta/2);
    odom.pose.pose.orientation.w = cos(theta/2);
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.angular.z = vth;
    odom_pub.publish(odom);

    js.header.stamp = ros::Time::now();
    joint_pub.publish(js);
  }

  void updateSpeeds(int left_speed, int right_speed){
    boost::mutex::scoped_lock lock(claw_mutex_);
    try{
      if (robot_dir){
      	claw->SpeedAccelM1M2(address, max_accel_qpps, left_speed, right_speed);
      } else {
	claw->SpeedAccelM1M2(address, max_accel_qpps, right_speed, left_speed);
      }
    }  catch(USBSerial::Exception &e) {
      ROS_WARN("Error setting motor speeds (error=%s)", e.what());
      serial_error();
      return;
    }

  }

  void updateState(){
    boost::mutex::scoped_lock lock(claw_mutex_);

    bool valid = false;
    try{
      double battery = 0.0;
      battery = claw->ReadMainBatteryVoltage(address, &valid);
      if (valid){
          state.battery_voltage = (double)battery/10.0;
      }
    }  catch(USBSerial::Exception &e) {
      ROS_WARN("Error reading battery voltage (error=%s)", e.what());
      serial_error();
      return;
    }

    try{
      int16_t m1cur, m2cur;
      valid = claw->ReadCurrents(address, m1cur, m2cur);
      if (valid){
          state.left_motor_current = (double)m1cur/100.0;
          state.right_motor_current = (double)m2cur/100.0;
      }
    }  catch(USBSerial::Exception &e) {
      ROS_WARN("Error reading motor currents (error=%s)", e.what());
      serial_error();
      return;
    }
    state_pub.publish(state);
    last_state = ros::Time::now();
  }

  void twistCb(geometry_msgs::Twist msg){
    last_motor = ros::Time::now();
    double lin = msg.linear.x;
    double ang = msg.angular.z;
    double left = 1.0 * lin - ang * base_width / 2.0;
    double right = 1.0 * lin + ang * base_width / 2.0;
    target_left_qpps = left * ticks_per_m;
    target_right_qpps = right * ticks_per_m;
 }
 void shutdown(){

  }

  void spin(){
    ros::Rate r(update_rate);
    while (ros::ok()){
        ros::spinOnce();
        if (ros::Time::now() > (last_state + ros::Duration(DIAGNOSTICS_DELAY))){
            this->updateState();
          }
        if (ros::Time::now() > (last_motor + ros::Duration(TWIST_CMD_TIMEOUT))){
            target_left_qpps = target_right_qpps = 0;
          }
        this->upateOdom();
        this->updateSpeeds(target_left_qpps, target_right_qpps);
        r.sleep();
      }
    this->updateSpeeds(0, 0);
  }


private:
  ros::NodeHandle nh, priv_nh;
  ros::Subscriber cmd_vel_sub;
  ros::Publisher odom_pub, joint_pub, state_pub;
  std::string port;
  int baud_rate;
  int update_rate;
  double base_width;
  double ticks_per_m;
  std::string base_frame_id;
  ros::Time last_motor;
  boost::scoped_ptr<RoboClaw> claw;
  boost::scoped_ptr<USBSerial> ser;
  boost::mutex claw_mutex_;

  roboclaw_driver::RoboClawState state;
  int target_left_qpps, target_right_qpps;
  double x, y, theta, vx, vth;
  ros::Time last_odom;
  nav_msgs::Odometry odom;
  geometry_msgs::Quaternion quaternion;
  ros::Time last_state;
  std::string roboclaw_version;
  tf::TransformBroadcaster br;
  sensor_msgs::JointState js;
  ros::ServiceServer calib_server;
  int serial_errs;
  int robot_dir;
  double max_accel;
  int max_accel_qpps;
};


int main(int argc, char **argv){
  ros::init(argc, argv, "roboclaw_driver");
  try{
    RoboclawNode node;
    node.spin();
  } catch (boost::system::system_error e){
    std::cout << "\nError starting node: " << e.code() << " " << e.what() << std::endl;
  }


  return 0;
}
