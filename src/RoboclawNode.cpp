#include <ros/ros.h>
#include <numeric>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <std_srvs/Empty.h>
#include <std_srvs/EmptyRequest.h>
#include <std_srvs/EmptyResponse.h>
#include <std_msgs/Float64.h>
#include "RoboClaw.h"
#include <boost/thread/mutex.hpp>

#define address 0x80
#define DIAGNOSTICS_DELAY 2.0
#define TWIST_CMD_TIMEOUT 3.0

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
    if(!priv_nh.getParam("KP", KP)){
        KP = 0.1;
      }
    if(!priv_nh.getParam("KI", KI)){
        KI = 0.5;
      }
    if(!priv_nh.getParam("KD", KD)){
        KD = 0.25;
      }
    if(!priv_nh.getParam("QPPS", QPPS)){
        QPPS = 11600;
      }
    if(!priv_nh.getParam("last_odom_x", x)){
        x=0.0;
      } else {
        ROS_INFO_STREAM("setting X postion to " << x);
      }
    if(!priv_nh.getParam("last_odom_y", y)){
        y=0.0;
      } else {
        ROS_INFO_STREAM("setting Y position to " << y);
      }
    if(!priv_nh.getParam("last_odom_theta", theta)){
        theta=0.0;
      }
    if(!priv_nh.getParam("timeout", timeout)){
        timeout=3.0;
      }

    ROS_INFO_STREAM("Starting roboclaw node with params:");
    ROS_INFO_STREAM("Port:\t" << port);
    ROS_INFO_STREAM("Baud rate:\t" << baud_rate);
    ROS_INFO_STREAM("Base Width:\t" << base_width);
    ROS_INFO_STREAM("Ticks Per Metre:\t" << ticks_per_m);
    ROS_INFO_STREAM("KP:\t" << KP);
    ROS_INFO_STREAM("KI:\t" << KI);
    ROS_INFO_STREAM("KD:\t" << KD);
    ROS_INFO_STREAM("QPPS:\t" << QPPS);

    serial_errs = 0;
    ser.reset(new USBSerial());
    open_usb();
    claw.reset(new RoboClaw(ser.get()));

    last_motor = ros::Time::now();

    //x = y = theta = 0.0;
    vx = vth = 0;
    last_odom  = ros::Time::now();

    quaternion.x = 0.0;
    quaternion.y = 0.0;

    odom.header.frame_id = "odom";
    odom.child_frame_id = base_frame_id;
    odom.pose.pose.position.z = 0.0;

    cmd_vel_sub = nh.subscribe("cmd_vel", 1, &RoboclawNode::twistCb, this);
    diag_pub = nh.advertise<diagnostic_msgs::DiagnosticArray>("diagnostics", 1);
    odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 10);
    joint_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 1);
    battery_pub = nh.advertise<std_msgs::Float64>("battery_voltage", 1);

    try{
      claw->SetM1Constants(address,KD,KP,KI,QPPS);
      claw->SetM2Constants(address,KD,KP,KI,QPPS);
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
    if (serial_errs == 5) {
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
        left_qpps = left_dir * speed;
      } else {
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
        right_qpps = right_dir * speed;
      } else {
        ROS_WARN("Invalid data from motor 2");
        serial_error();
        return;
      }

    double left_speed, right_speed = 0.0;

    left_speed = left_qpps / ticks_per_m;
    right_speed = right_qpps / ticks_per_m;

    vx = (left_speed + right_speed) / 2.0;
    vth = (left_speed - right_speed) / base_width;

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

  void updateDiagnostics(){
    boost::mutex::scoped_lock lock(claw_mutex_);
    last_diag = ros::Time::now();
    int error = -1;
    bool valid = true;
    error = claw->ReadError(address, &valid);
    std::vector<std::string> messages;
    diagnostic_msgs::DiagnosticArray diag_array;
    diag_array.header.stamp = ros::Time::now();
    if (error != 0){
        // parse the error string
        if ((error & RoboClaw::ERR_M1_CURRENT) == RoboClaw::ERR_M1_CURRENT){
            messages.push_back("Motor1 OverCurrent");
            ROS_WARN("Motor1 OverCurrent");
          }
        else if ((error & RoboClaw::ERR_M2_CURRENT) == RoboClaw::ERR_M2_CURRENT){
            messages.push_back("Motor2 OverCurrent");
            ROS_WARN("Motor2 OverCurrent");
          }
        else if ((error & RoboClaw::ERR_E_STOP) == RoboClaw::ERR_E_STOP)
          messages.push_back("Emergency Stop");
        else if ((error & RoboClaw::ERR_TEMP) == RoboClaw::ERR_TEMP)
          messages.push_back("Temperature");
        else if ((error & RoboClaw::ERR_MAIN_BATT_HIGH) == RoboClaw::ERR_MAIN_BATT_HIGH)
          messages.push_back("Main Battery High");
        else if ((error & RoboClaw::ERR_MAIN_BATT_LOW) == RoboClaw::ERR_MAIN_BATT_LOW)
          messages.push_back("Main Battery Low");
        else if ((error & RoboClaw::ERR_LOGIC_BATT_HIGH) == RoboClaw::ERR_LOGIC_BATT_HIGH)
          messages.push_back("Logic Battery High");
        else if ((error & RoboClaw::ERR_LOGIC_BATT_HIGH) == RoboClaw::ERR_LOGIC_BATT_HIGH)
          messages.push_back("Logic Battery Low");

        diagnostic_msgs::DiagnosticStatus stat;
        stat.name = "Roboclaw";
        stat.hardware_id = roboclaw_version;
        stat.level = stat.ERROR;
        stat.message = std::accumulate(messages.begin(), messages.end(), std::string(""));
        diag_array.status.push_back(stat);
        diag_pub.publish(diag_array);
      } else {
        bool valid = true;
        int16_t m1cur, m2cur;
        double battery, temp = 0.0;
        diagnostic_msgs::DiagnosticStatus stat;
        stat.name = "Roboclaw";
        stat.hardware_id = roboclaw_version;
        stat.level = stat.OK;
        stat.message = "Running";
        /*temp = claw->Read ReadTemperature(&valid);
        ros::Duration(COMMAND_DELAY).sleep();
        if (valid){
            diagnostic_msgs::KeyValue kv;
            kv.key = "Temperature (Degrees C)";
            kv.value = boost::lexical_cast<std::string>((double)temp/10.0);
            stat.values.push_back(kv);
        }*/
        battery = claw->ReadMainBatteryVoltage(address, &valid);
        if (valid){
            diagnostic_msgs::KeyValue kv;
            kv.key = "Voltage (V)";
            battery_voltage = (double)battery/10.0;
            std_msgs::Float64 flt;
            flt.data = battery_voltage;
            battery_pub.publish(flt);
            kv.value = boost::lexical_cast<std::string>(battery_voltage);
            stat.values.push_back(kv);
          }
        if (claw->ReadCurrents(address, m1cur, m2cur)){
            diagnostic_msgs::KeyValue kv;
            kv.key = "Motor1 Current (A)";
            kv.value = boost::lexical_cast<std::string>((double)m1cur/100.0);
            stat.values.push_back(kv);

            kv.key = "Motor2 Current (A)";
            kv.value = boost::lexical_cast<std::string>((double)m2cur/100.0);
            stat.values.push_back(kv);
          }

        diag_array.status.push_back(stat);
        diag_pub.publish(diag_array);
      }

  }

  void twistCb(geometry_msgs::Twist msg){
    boost::mutex::scoped_lock lock(claw_mutex_);
    last_motor = ros::Time::now();
    double lin = msg.linear.x;
    double ang = msg.angular.z;
    if (lin == last_lin_speed && ang == last_ang_speed){
        return;
      } else {
        last_lin_speed = lin;
        last_ang_speed = ang;
      }
    double left = 1.0 * lin - ang * base_width / 2.0;
    double right = 1.0 * lin + ang * base_width / 2.0;
    int16_t left_qpps = left * ticks_per_m;
    int16_t right_qpps = right * ticks_per_m;
    try{
      claw->SpeedM1M2(address, left_qpps, right_qpps);
    }  catch(USBSerial::Exception &e) {
      ROS_WARN("Error setting motor speeds (error=%s)", e.what());
      serial_error();
      return;
    }

  }

  void shutdown(){
    try{
      claw->SpeedM1M2(address, 0, 0);
    }  catch(USBSerial::Exception &e) {
      ROS_WARN("Error setting motor speeds (error=%s)", e.what());
      serial_error();
      return;
    }
  }

  void spin(){
    ros::Rate r(update_rate);
    while (ros::ok()){
        ros::spinOnce();
        this->upateOdom();
        if (ros::Time::now() > (last_diag + ros::Duration(DIAGNOSTICS_DELAY))){
            try{
              this->updateDiagnostics();

            } catch(USBSerial::Exception &e) {
              ROS_WARN("Error setting motor speeds (error=%s)", e.what());
              serial_error();
            }
          }
        if (ros::Time::now() > (last_motor + ros::Duration(TWIST_CMD_TIMEOUT))){
            try{
              claw->SpeedM1M2(address, 0, 0);
            }  catch(USBSerial::Exception &e) {
              ROS_WARN("Error setting motor speeds (error=%s)", e.what());
              serial_error();
              return;
            }
            last_lin_speed = 0;
            last_ang_speed = 0;
          }

        r.sleep();
      }

    this->shutdown();
  }


private:
  ros::NodeHandle nh, priv_nh;
  ros::Subscriber cmd_vel_sub;
  ros::Publisher odom_pub, diag_pub, joint_pub, battery_pub;
  std::string port;
  int baud_rate;
  int update_rate;
  double base_width;
  double ticks_per_m;
  double KP;
  double KI;
  double KD;
  int QPPS;
  std::string base_frame_id;
  ros::Time last_motor;
  boost::scoped_ptr<RoboClaw> claw;
  boost::scoped_ptr<USBSerial> ser;
  boost::mutex claw_mutex_;

  double x, y, theta, vx, vth;
  int left_dir, right_dir;
  ros::Time last_odom;
  nav_msgs::Odometry odom;
  geometry_msgs::Quaternion quaternion;
  ros::Time last_diag;
  std::string roboclaw_version;
  tf::TransformBroadcaster br;
  sensor_msgs::JointState js;
  ros::ServiceServer calib_server;
  double battery_voltage;
  double timeout;
  double last_lin_speed, last_ang_speed;
  int serial_errs;
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
