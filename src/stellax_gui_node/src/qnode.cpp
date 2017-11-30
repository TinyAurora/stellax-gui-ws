/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "qnode.hpp"

#include <geometry_msgs/Twist.h>
#include "cglobal.hpp"

// Define
#define max(a, b) (a > b ? a : b)
#define min(a, b) (a < b ? a : b)

#define NO_OPERATION -1
#define STOP 0
#define UP 1
#define DOWN 2
#define LEFT 3
#define RIGHT 4

double speed = 0.5, turn = 2.0;
double x = 0.0, th = 0.0;
int count = 0;
double target_speed = 0.0, target_turn = 0.0;
double control_speed = 0.0, control_turn = 0.0;

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace stellax_gui_node {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
	{}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

bool QNode::init() {
	ros::init(init_argc,init_argv,"stellax_gui_node");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	// Add your ros communications here.
    twist_publisher = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
    start();
	return true;
}

bool QNode::init(const std::string &master_url, const std::string &host_url) {
	std::map<std::string,std::string> remappings;
	remappings["__master"] = master_url;
	remappings["__hostname"] = host_url;
	ros::init(remappings,"stellax_gui_node");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	// Add your ros communications here.
    twist_publisher = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
    start();
	return true;
}

void QNode::run() {
    ros::Rate loop_rate(50);
        while(ros::ok()) {
            switch (CGlobal::key_value) {
                case UP:
                    x = 1.0;
                    th = 0.0;
                    count = 0;
                    break;
                case DOWN:
                    x = -1.0;
                    th = 0.0;
                    count = 0;
                    break;
                case LEFT:
                    x = 0.0;
                    th = 1.0;
                    count = 0;
                    break;
                case RIGHT:
                    x = 0.0;
                    th = -1.0;
                    count = 0;
                    break;
                case STOP:
                    x = 0.0;
                    th = 0.0;
                    control_speed = 0.0;
                    control_turn = 0.0;
                    break;
                case NO_OPERATION:
                    count = count + 1;
                    if (count > 4) {
                        x = 0;
                        th = 0;
                    }
                    break;
            }

            target_speed = speed * x;
            target_turn = turn * th;

            if (target_speed > control_speed)
                control_speed = min(target_speed, control_speed + 0.02);
            else if (target_speed < control_speed)
                control_speed = max(target_speed, control_speed - 0.02);
            else
                control_speed = target_speed;

            if (target_turn > control_turn)
                control_turn = min(target_turn, control_turn + 0.1);
            else if (target_turn < control_turn)
                control_turn = max(target_turn, control_turn - 0.1);
            else
                control_turn = target_turn;

            geometry_msgs::Twist twist;
            twist.linear.x = control_speed; twist.linear.y = 0; twist.linear.z = 0;
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = control_turn;
            twist_publisher.publish(twist);

            ros::spinOnce();
            loop_rate.sleep();
        }

	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}


void QNode::log( const LogLevel &level, const std::string &msg) {
	logging_model.insertRows(logging_model.rowCount(),1);
	std::stringstream logging_model_msg;
	switch ( level ) {
		case(Debug) : {
				ROS_DEBUG_STREAM(msg);
				logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Info) : {
				ROS_INFO_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Warn) : {
				ROS_WARN_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Error) : {
				ROS_ERROR_STREAM(msg);
				logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Fatal) : {
				ROS_FATAL_STREAM(msg);
				logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
				break;
		}
	}
	QVariant new_row(QString(logging_model_msg.str().c_str()));
	logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
	Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

}  // namespace stellax_gui_node
