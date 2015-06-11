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
#include "Roboclaw.h"
#include <boost/thread/mutex.hpp>

#define address 0x80
#define MAX_ERRORS 10

class RoboclawNode{
	public:
		RoboclawNode(): nh(), priv_nh("~"), debug(false), error_count(0){
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

			ROS_INFO_STREAM("Starting roboclaw node with params:");
			ROS_INFO_STREAM("Port:\t" << port);
			ROS_INFO_STREAM("Baud rate:\t" << baud_rate);
			ROS_INFO_STREAM("Base Width:\t" << base_width);
			ROS_INFO_STREAM("Ticks Per Metre:\t" << ticks_per_m);
			ROS_INFO_STREAM("KP:\t" << KP);
			ROS_INFO_STREAM("KI:\t" << KI);
			ROS_INFO_STREAM("KD:\t" << KD);
			ROS_INFO_STREAM("QPPS:\t" << QPPS);

			claw = new Roboclaw(port, baud_rate, address);
			last_motor = ros::Time::now();

			//x = y = theta = 0.0;
			vx = vth = 0;
			last_enc_left = last_enc_right = 0;
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

			if(!claw->SetM1VelocityPID(KD,KP,KI,QPPS)){
			    ROS_ERROR("Error setting M1 params");
			    ros::shutdown();
			}
			if(!claw->SetM2VelocityPID(KD,KP,KI,QPPS)){
			    ROS_ERROR("Error setting M2 params");
			    ros::shutdown();
			}
			
			roboclaw_version = claw->ReadVersion();
			ROS_INFO_STREAM("Connected to: " << roboclaw_version);

			js.name.push_back("base_l_wheel_joint");
			js.name.push_back("base_r_wheel_joint");
			js.position.push_back(0.0);
			js.position.push_back(0.0);
			js.effort.push_back(0.0);
			js.effort.push_back(0.0);
			js.header.frame_id = "base_link";

			calib_server = nh.advertiseService("motor_calibrate", &RoboclawNode::calib_callback, this);

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

		void upateOdom() throw (boost::system::system_error){

			boost::mutex::scoped_lock lock(claw_mutex_);
			//ROS_INFO_STREAM("updating odom");
			// calcuate time elapsed since last update
			ros::Time now = ros::Time::now();
			ros::Duration elapsed_t = now - last_odom;
			last_odom = now;
			double elapsed = elapsed_t.toSec();

			uint8_t status;
			bool valid1, valid2;
			valid1 = valid2 = true;
			long encoder_left = last_enc_left;
			long encoder_right = last_enc_right;

			// read the encoder counts
			encoder_left = claw->ReadEncoderM1(&status, &valid1);
			encoder_right = claw->ReadEncoderM2(&status, &valid2);

			if (!valid1 && !(status == 0 || status == 1)){
				ROS_WARN("Invalid encoder data on motor 1");
				return;
			} 
			if (!valid2 && !(status == 0 || status == 1)){
				ROS_WARN("Invalid encoder data on motor 2");
				return;
			} 

			error_count = 0;
			// calculate distance travelled by each wheel
			double dist_left = (float)(encoder_left - last_enc_left) / ticks_per_m;
			double dist_right = (float)(encoder_right - last_enc_right) / ticks_per_m;

			last_enc_left = encoder_left;
			last_enc_right = encoder_right;

			// distance robot has travelled is the average of the distance travelled by both
			// wheels
			double dist_travelled = (dist_left + dist_right) / 2;
			// calculate appoximate heading change (radians), this works for small angles
			double delta_th = (dist_right - dist_left) / base_width;
			vx = dist_travelled / elapsed;
			vth = delta_th / elapsed;

			if (dist_travelled != 0){
				double delta_x = cos(delta_th) * dist_travelled;
				double delta_y = -sin(delta_th) * dist_travelled;

				x += (cos(theta) * delta_x - sin(theta) * delta_y);
				y += (sin(theta) * delta_x + cos(theta) * delta_y);
			}

			if (delta_th != 0){
				theta += delta_th;
			}

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
			//ROS_INFO_STREAM("odom updated");
		}
		void resetEncoders() {
			return;
			boost::mutex::scoped_lock lock(claw_mutex_);
			if (vx == 0 && vth == 0){
				claw->ResetEncoders();
				last_enc_left = 0;
				last_enc_right = 0;
			}
		}

		void updateDiagnostics() throw (boost::system::system_error){
			boost::mutex::scoped_lock lock(claw_mutex_);
			last_diag = ros::Time::now();
			int error = -1;
			bool valid = true;
			error = claw->ReadErrorState(&valid);
			std::vector<std::string> messages;
			diagnostic_msgs::DiagnosticArray diag_array;
			diag_array.header.stamp = ros::Time::now();
			if (error != 0){
				// parse the error string
				if ((error & Roboclaw::ERR_M1_CURRENT) == Roboclaw::ERR_M1_CURRENT){
					messages.push_back("Motor1 OverCurrent");
					ROS_WARN("Motor1 OverCurrent");
				}
				else if ((error & Roboclaw::ERR_M2_CURRENT) == Roboclaw::ERR_M2_CURRENT){
					messages.push_back("Motor2 OverCurrent");
					ROS_WARN("Motor2 OverCurrent");
				}
				else if ((error & Roboclaw::ERR_E_STOP) == Roboclaw::ERR_E_STOP)
					messages.push_back("Emergency Stop");
				else if ((error & Roboclaw::ERR_TEMP) == Roboclaw::ERR_TEMP)
					messages.push_back("Temperature");
				else if ((error & Roboclaw::ERR_MAIN_BATT_HIGH) == Roboclaw::ERR_MAIN_BATT_HIGH)
					messages.push_back("Main Battery High");
				else if ((error & Roboclaw::ERR_MAIN_BATT_LOW) == Roboclaw::ERR_MAIN_BATT_LOW)
					messages.push_back("Main Battery Low");
				else if ((error & Roboclaw::ERR_LOGIC_BATT_HIGH) == Roboclaw::ERR_LOGIC_BATT_HIGH)
					messages.push_back("Logic Battery High");
				else if ((error & Roboclaw::ERR_LOGIC_BATT_HIGH) == Roboclaw::ERR_LOGIC_BATT_HIGH)
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
				double temp, battery = 1;
				diagnostic_msgs::DiagnosticStatus stat;
				stat.name = "Roboclaw";
				stat.hardware_id = roboclaw_version;
				stat.level = stat.OK;
				stat.message = "Running";
				ROS_INFO_COND(debug, "getting temp");
				temp = claw->ReadTemperature(&valid);
				if (valid){
					diagnostic_msgs::KeyValue kv;
					kv.key = "Temperature (Degrees C)";
					kv.value = boost::lexical_cast<std::string>((double)temp/10.0);
					stat.values.push_back(kv);
				}
				ROS_INFO_COND(debug, "getting battery");
				battery = claw->ReadMainBatteryVoltage(&valid);
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
				ROS_INFO_COND(debug, "getting currents");
				if (claw->ReadCurrents(m1cur, m2cur)){
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

		void twistCb(geometry_msgs::Twist msg)throw (boost::system::system_error){
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
			ROS_INFO("left speed %f qpps: %d", left, left_qpps);
			claw->SetMixedSpeed(left_qpps, right_qpps);
			ros::Duration(0.2).sleep();
		}

		void shutdown(){
			claw->SetMixedSpeed(0, 0);
		}

		void spin(){
			ros::Rate r(update_rate);
			while (ros::ok()){
				ros::spinOnce();
				this->upateOdom();
				if (ros::Time::now() > (last_diag + ros::Duration(2.0))){
					this->updateDiagnostics();
					this->resetEncoders();
				}
				if (ros::Time::now() > (last_motor + ros::Duration(3.0))){
					claw->SetMixedSpeed(0,0);
					last_lin_speed = 0;
					last_ang_speed = 0;
				}
				if (error_count > MAX_ERRORS){
					priv_nh.setParam("last_odom_x", x);
					priv_nh.setParam("last_odom_y", y);
					priv_nh.setParam("last_odom_theta", theta);
					ROS_ERROR("Error limit reached shutting down");
					break;
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
		Roboclaw* claw;
		boost::mutex claw_mutex_;

		double x, y, theta, vx, vth;
		long last_enc_left, last_enc_right;
		ros::Time last_odom;
		nav_msgs::Odometry odom;
		geometry_msgs::Quaternion quaternion;
		ros::Time last_diag;
		std::string roboclaw_version;
		tf::TransformBroadcaster br;
		sensor_msgs::JointState js;
		ros::ServiceServer calib_server;
		bool debug;
		int error_count;
		double battery_voltage;
		double last_lin_speed, last_ang_speed;
};


int main(int argc, char **argv){
	ros::init(argc, argv, "roboclaw_driver");
	try{
		RoboclawNode node;
		node.spin();
	} catch(const boost::system::system_error& ex){
		ROS_ERROR_STREAM("IO error " << ex.what());
		return -1;
	} catch (std::exception &ex){
		ROS_ERROR_STREAM("General error " << ex.what());
		return -1;
	}
	return 0;
}
