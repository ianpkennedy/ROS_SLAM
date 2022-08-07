#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/TransformStamped.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <geometry_msgs/Quaternion.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf/transform_broadcaster.h>
#include <vector>
#include <cmath>
#include "turtlelib/rigid2d.hpp"
#include "turtlelib/diff_drive.hpp"
#include "nuturtle_control/SetPose.h"
#include "geometry_msgs/TransformStamped.h"
#include "nuturtle_control/SetPoseRequest.h"
#include "nuturtle_control/SetPoseResponse.h"


/// \file
/// \brief This node computes odometry
///
/// PARAMETERS:
///     rate (integer): publishing rate in Hz
///     wheel_radius (float): radius of turtlebot wheel in meters
///     track_width (float): wheel distance of turtlebot in meters
///     encoder_ticks_to_rad (float): conversion factor from encoder ticks to radians
///     motor_cmd_to_radsec (float): conversion factor from motor command units to rad/sec
///     body_id (string): body frame name
///     odom_id (string): odom frame name
///          
/// PUBLISHES:
///     odom (nav_msgs::Odometry): robot odometry
///     path (nav_msgs::Path): path of odometry robot
/// SUBSCRIBES:
///     joint_states (sensor_msgs::JointState): joint state publisher
/// SERVICES:
///     set_pose: service that sets the pose of the robot

namespace{
    turtlelib::Diff_Drive diff1(0.0,0.0,0.0,0.0,0.0);
    turtlelib::Twist2D twist1;
    turtlelib::Transform2D T_wb;
    
    std::vector<double> v_wheel = {0.0,0.0};
    std::vector<double> pos_wheel = {0.0,0.0};
    std::vector<double> pos_wheel_old = {0.0,0.0};

    sensor_msgs::JointState wheel_states;

    double encoder_ticks_to_rad = 0.0;
    double motor_cmd_to_radsec = 0.0;

    std::string wheel_left = "undefined";
    std::string wheel_right = "undefined";

    double xorigin = 0.0;
    double theta0 = 0.0;
    double yorigin = 0.0;

    double wheel_radius = 0.0;
    double track_width = 0.0;

    int motor_cmd_max = 0;

    geometry_msgs::TransformStamped ts;

    double initial;

}

    /// \brief joint states callback to computes wheel velocities and positions
    /// \param msg - geometry_msgs::Twist data
    /// \returns None
void joint_statesCallback(const sensor_msgs::JointState& msg){
     
    int i = 0; 

    //Assign wheel positions
    for(i=0;i<2;i++){
            v_wheel.at(i) = msg.velocity[i];
            pos_wheel.at(i) =  msg.position[i];
    }


    //Compute the twist
    twist1.tw.at(0) = -wheel_radius*v_wheel.at(0)/track_width + wheel_radius*v_wheel.at(1)/track_width; 
    twist1.tw.at(1) = wheel_radius*v_wheel.at(0)/2.0 + wheel_radius*v_wheel.at(1)/2.0; 
    twist1.tw.at(2) = 0.0;

        T_wb = diff1.fkin(pos_wheel.at(0),pos_wheel.at(1));
        pos_wheel_old.at(0) = pos_wheel.at(0);
        pos_wheel_old.at(1) = pos_wheel.at(1);

        if((ros::Time::now().toSec()-initial) < 10.0){ //wait for encoder data to load so as to avoid erroneous measurements
            std::cout<<"PLEASE WAIT TILL TIMER IS 0 BEFORE MOVING: " << (10.0 - (ros::Time::now().toSec()-initial))<<"\r\n";
            turtlelib::Vector2D vtemp;
            vtemp.x = xorigin;
            vtemp.y = yorigin;
            T_wb = turtlelib::Transform2D(vtemp,theta0);
            diff1 = turtlelib::Diff_Drive(track_width/2.0,wheel_radius,xorigin,yorigin,theta0,pos_wheel.at(0),pos_wheel.at(1));
            T_wb = diff1.fkin(pos_wheel.at(0),pos_wheel.at(1));
            T_wb = turtlelib::Transform2D(vtemp,theta0);

        }



}


    /// \brief move the robot in the odom frame
    /// \param request - x,y, theta coordinates to set the odometry at
    /// \returns None
bool set_pose(nuturtle_control::SetPose::Request& request, nuturtle_control::SetPose::Response& ){


    //Set the pose
    diff1 = turtlelib::Diff_Drive(track_width/2.0,wheel_radius,request.x,request.y,request.t);
    T_wb = diff1.fkin(pos_wheel.at(0),pos_wheel.at(1));

    return true;

}

int main(int argc, char * argv[]){
    
    // long int k=0;

    int trail_count = 0;
    int f = 0;

    std::string body_id = "undefined";
    std::string odom_id = "odom";
    std::string undefined = "undefined";

    ros::init(argc,argv,"odometry");
    ros::NodeHandle n;

    //Load in parameters
    n.param("rate", f, 500); //Source (01/29): https://answers.ros.org/question/253797/no-matching-function-for-call-to-rosnodehandleparam/
    n.param("wheel_radius", wheel_radius, 0.033);
    n.param("track_width", track_width, 0.16);
    n.param("encoder_ticks_to_rad",encoder_ticks_to_rad,-1.0);
    n.param("motor_cmd_to_radsec",motor_cmd_to_radsec,-1.0);
    n.param("odometry/x0", xorigin, 0.0);
    n.param("odometry/theta0", theta0, 0.0);
    n.param("odometry/y0",yorigin,0.0);
    
    n.param("body_id",body_id,undefined);
    n.param("odom_id",odom_id,odom_id);
    n.param("wheel_left",wheel_left,undefined);
    n.param("wheel_right",wheel_right,undefined);
    n.param("motor_cmd_max", motor_cmd_max, 256);

    if(!(n.hasParam("body_id")) ||   !(n.hasParam("wheel_left")) ||  !(n.hasParam("wheel_right")) ){
        ROS_ERROR_STREAM("Undefined parameter - terminating!\n");
        throw;
    }

    if(n.hasParam("odometry_slam/odom_id")){
        n.getParam("odometry_slam/odom_id", odom_id);
    }
    
    
    //Subscribers
    ros::Subscriber joint_state_sub = n.subscribe("joint_states", 10, joint_statesCallback);

    //Publishers
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 100);
    ros::Publisher trail_pub = n.advertise<nav_msgs::Path>("pathodom", 100);


    //Services
    ros::ServiceServer srv_set_pose = n.advertiseService("set_pose", set_pose);

    ros::Rate rate(f);

    //Throw if parameters undefined in parameter server
    if(track_width < 0 || wheel_radius < 0 || motor_cmd_to_radsec < 0 || encoder_ticks_to_rad < 0 || body_id == "undefined" || wheel_left == "undefined" || wheel_right == "undefined"){
        ROS_ERROR_STREAM("Undefined parameter - terminating!\n");
        throw;
    }

    //Differential drive model
    diff1 = turtlelib::Diff_Drive(track_width/2.0,wheel_radius,xorigin,yorigin,theta0);

    T_wb = diff1.fkin(pos_wheel.at(0),pos_wheel.at(1));


    //Declare transform
    tf2_ros::TransformBroadcaster b;

    initial = ros::Time::now().toSec();

    //Declare odometry
    nav_msgs::Odometry odom;
    odom.header.frame_id = odom_id;
    odom.child_frame_id = body_id;

    //Declare path
    nav_msgs::Path trail;
    trail.header.frame_id = odom_id;

    geometry_msgs::PoseStamped pose_stamp;
    pose_stamp.header.frame_id = odom_id;



    while(ros::ok()){

        //Publish odometry
        odom.header.stamp = ros::Time::now();
        // odom.header.stamp = wheel_states.header.stamp;
        odom.twist.twist.angular.z = twist1.tw.at(0);
        odom.twist.twist.linear.x = twist1.tw.at(1);
        geometry_msgs::Quaternion orientation_quaternion = tf::createQuaternionMsgFromYaw(T_wb.rotation()); //source (02/01): http://wiki.ros.org/navigation/Tutorials/RobotSetup/Odom
        odom.pose.pose.position.x = T_wb.translation().x;
        odom.pose.pose.position.y = T_wb.translation().y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = orientation_quaternion;
        odom_pub.publish(odom);



        pose_stamp.header.stamp = ros::Time::now();
        pose_stamp.pose.position = odom.pose.pose.position;
        pose_stamp.pose.orientation = orientation_quaternion;



        if(trail_count % 100 == 0){
            trail.header.stamp = ros::Time::now();
            trail.poses.push_back(pose_stamp);
            trail_pub.publish(trail);
        }

        trail_count++;


        //Broadcast transform
        ts.header.stamp = ros::Time::now();
        // ts.header.stamp = wheel_states.header.stamp;
        ts.header.frame_id = odom_id;
        ts.child_frame_id = body_id;
        ts.transform.translation.x = T_wb.translation().x;
        ts.transform.translation.y = T_wb.translation().y;
        ts.transform.translation.z = 0;
        tf2::Quaternion ang;
        ang.setRPY(0,0,T_wb.rotation());
        ts.transform.rotation.x = ang.x();
        ts.transform.rotation.y = ang.y();
        ts.transform.rotation.z = ang.z();;
        ts.transform.rotation.w = ang.w();
        b.sendTransform(ts);

    
        ros::spinOnce();
  
        rate.sleep(); 
    }

    return 0;
}