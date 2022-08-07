#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/Empty.h>
#include "turtlelib/rigid2d.hpp"
#include "turtlelib/diff_drive.hpp"
#include "nuturtle_control/Control.h"

/// \file
/// \brief This node publishes a twist message to send the robot in a circle
///
/// PARAMETERS:
/// frequency (int): frequency of node
/// wheel_radius (float): radius of wheel
/// track_width (float): wheel distance 
///     
/// PUBLISHES:
///     cmd_vel (geometry_msgs::Twist): twist object
/// SUBSCRIBES:
///     None
/// SERVICES:
///     control: set the radius of circle and speed of robot
///     reverse: reverse direction of robot
///     stop: stop robot

using std::cout;


namespace{
    int f = 0;

    double wheel_radius = 0.0;
    double track_width = 0.0;

    geometry_msgs::Twist cmd;

    double circle_radius = 0.0;
    double theta_dot = 0.0;
    double x_dot = 0.0; 

    int state = 1;

}


    /// \brief control the circle radius and speed to move
    /// \param request - radius (m) of circle and velociy (m/s) to move at
    /// \returns None
bool control(nuturtle_control::Control::Request& request, nuturtle_control::Control::Response& ){
    
    circle_radius = request.radius;
    theta_dot = request.velocity; 
    
    x_dot = theta_dot*(circle_radius);

    state = 1;

    return true;

}

    /// \brief reverse direction of circle
    /// \param request - None
    /// \returns None
bool reverse(std_srvs::Empty::Request& , std_srvs::Empty::Response& ){

    theta_dot = -theta_dot;
    x_dot = -x_dot;

    return true;

}


    /// \brief stop the robot
    /// \param request - None
    /// \returns None
bool stop(std_srvs::Empty::Request& , std_srvs::Empty::Response& ){

    theta_dot = 0.0;
    x_dot = 0.0;
    state = 2;

    return true;

}

int main(int argc, char * argv[]){
    
    //Start ROS
    ros::init(argc,argv,"circle");
    ros::NodeHandle n("~");


    //Load parameters
    n.param("frequency",f, 100);
    n.param("wheel_radius",wheel_radius,0.033);
    n.param("track_width",track_width,0.16);


    //Initialize the twist to 0 at the start
    cmd.linear.x = 0.0;
    cmd.linear.y = 0.0;
    cmd.linear.z = 0.0;

    cmd.angular.x = 0.0;
    cmd.angular.y = 0.0;
    cmd.angular.z = 0.0;


    //Publishers
    ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    

    //Services
    ros::ServiceServer srv_control = n.advertiseService("control", control);
    ros::ServiceServer srv_reverse = n.advertiseService("reverse", reverse);
    ros::ServiceServer srv_stop = n.advertiseService("stop", stop);

    ros::Rate rate(f);

    while(ros::ok()){

        //Publish cmd_vel
        if(state==1){
            cmd.linear.x = x_dot;
            cmd.linear.y = 0.0;
            cmd.linear.z = 0.0;
            cmd.angular.x = 0.0;
            cmd.angular.y = 0.0;
            cmd.angular.z = theta_dot;
            cmd_vel_pub.publish(cmd);
        }  

        if(state==2){
            cmd.linear.x = 0.0;
            cmd.linear.y = 0.0;
            cmd.linear.z = 0.0;
            cmd.angular.x = 0.0;
            cmd.angular.y = 0.0;
            cmd.angular.z = 0.0;
            cmd_vel_pub.publish(cmd);
            state = 0;
        }



        ros::spinOnce();
        rate.sleep(); 
    }

    return 0;
}