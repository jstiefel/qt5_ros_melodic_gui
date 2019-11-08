#include <ros/ros.h>
#include <std_msgs/String.h>
#include "nav_ui/qnode.hpp"
#include <string>

namespace nav_ui {

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
  ros::init(init_argc,init_argv,"qt5_ros_melodic_gui");
  if ( ! ros::master::check() ) {
    return false;
  }
  ros::start(); // explicitly needed since our nodehandle is going out of scope.
  ros::NodeHandle nh;
  // Add your ros communications here.
  log_subscriber = nh.subscribe("/rosout_agg", 5, &QNode::logTopicCallback, this);

  //sleep(15); //wait 7 seconds before starting that whole log is not shown at startup
  start(); //start a Qthread, which calls run()
  std::cout << "Successfully initialized node." << std::endl;
  return true;
}

void QNode::run() {
  //QThread function
  ros::Rate loop_rate(1); //too fast loop rate crashes the GUI
  std::cout << "Node is running." << std::endl;
  while ( ros::ok() ) {

    ros::spinOnce();
    loop_rate.sleep();
  }
  std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
  Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

bool QNode::homing(){
  if (global_homing_client.call(homing_client)){
    ROS_INFO_STREAM("global homing client successfully called. Success: " << (int)homing_client.response.success);
  } else {
    ROS_ERROR("Failed to call service global_homing_client");
    return false;
  }
  return true;
}

bool QNode::ee_target_pos(float ee_y, float ee_x, float ee_z,  float ee_phi, float ee_theta){
  return true;
}

bool QNode::joint_target_pos(float q_y, float q_x, float q_alpha, float q_beta, float q_d){
  return true;
}

bool QNode::make_injection(){
  if (injection_client.call(injection)){
    ROS_INFO_STREAM("injection client successfully called. Success: " << (int)injection.response.success);
  } else {
    ROS_ERROR("Failed to call service injection_client");
    return false;
  }
  return true;
}


//Logging:
void QNode::logTopicCallback(const rosgraph_msgs::Log::ConstPtr& log){
  logging_model.insertRows(logging_model.rowCount(),1);
  std::stringstream logging_model_msg;
  int level = log->level;
  switch (level){
    case(1) : {
        logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << log->msg << " [" << log->name << "]";
        break;
    }
    case(2) : {
        logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << log->msg << " [" << log->name << "]";
        break;
    }
    case(4) : {
        logging_model_msg << "[WARN] [" << ros::Time::now() << "]: " << log->msg << " [" << log->name << "]";
        break;
    }
    case(8) : {
        logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << log->msg << " [" << log->name << "]";
        break;
    }
    case(16) : {
        logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << log->msg << " [" << log->name << "]";
        break;
    }
  }
  QVariant new_row(QString(logging_model_msg.str().c_str()));
  logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
  Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}


}  // namespace nav_ui
