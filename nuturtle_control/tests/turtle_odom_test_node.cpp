/// \file
/// \brief Testing file for turtle_interface class

#include "ros/ros.h"
#include <catch_ros/catch.hpp>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include "turtlelib/rigid2d.hpp"
#include "turtlelib/diff_drive.hpp"
#include "nuturtle_control/SetPose.h"
#include "nuturtle_control/SetPoseRequest.h"
#include "nuturtle_control/SetPoseResponse.h"


namespace{

    std::vector<double> states = {-1.0,-1.0,-1.0}; //theta,x,y
    
}


    /// \brief subscribes to odometry
    /// \param msg - nav_msgs::Odometry
    /// \returns None
void odomCallback(const nav_msgs::Odometry& msg){

    states.at(0) = msg.pose.pose.orientation.z;
    states.at(1) = msg.pose.pose.position.x;
    states.at(2) = msg.pose.pose.position.y;

    
}


/// \brief test odometry transform
TEST_CASE("Odom transform","[wheel_command]"){


    //Source for listener in this test case (02/03): https://www.codetd.com/en/article/6418629
    //Declare variables and listener
    int i = 0;
    ros::NodeHandle n; 
    
    tf2_ros::Buffer b;
    tf2_ros::TransformListener tf_l(b);
    
    geometry_msgs::TransformStamped odom;

    ros::Rate rate(100);

    //Wait for result
    while(i<100){
        odom = b.lookupTransform("base_footprint","world",ros::Time(0),ros::Duration(1));
        i++;
        rate.sleep(); 
     }

     //Check result
     REQUIRE(odom.transform.translation.x==Approx(0).margin(.01));
     REQUIRE(odom.transform.translation.y==Approx(0).margin(.01));
     REQUIRE(odom.transform.translation.z==Approx(0).margin(.01));

     REQUIRE(odom.transform.rotation.x==Approx(0).margin(.01));
     REQUIRE(odom.transform.rotation.y==Approx(0).margin(.01));
     REQUIRE(odom.transform.rotation.z==Approx(0).margin(.01));
     REQUIRE(odom.transform.rotation.w==Approx(1.0).margin(.01));


}


/// \brief test odometry set pose
TEST_CASE("Odom setpose","[wheel_command]"){
    int i = 0;
    ros::NodeHandle n; 
    ros::Rate rate(50);
    

    
    //Subscribers
    ros::Subscriber odom_sub = n.subscribe("odom", 100, odomCallback);


    //Declare service proxy
    ros::ServiceClient proxy = n.serviceClient<nuturtle_control::SetPose>("set_pose");


    nuturtle_control::SetPose::Request request;
    nuturtle_control::SetPose::Response response;

    
    request.x = 1.5;
    request.y = 1.5;
    request.t = 0.0;


    //call service
    ros::service::waitForService("/set_pose", ros::Duration(5));
    bool result = proxy.call(request,response);


    //Wait for result
    while(i<100){
        ros::spinOnce();
        i++;
        rate.sleep(); 
     }


    CHECK(states.at(0)==Approx(0.0).margin(.01));
    CHECK(states.at(1)==Approx(1.5).margin(.01));
    CHECK(states.at(2)==Approx(1.5).margin(.01));
    CHECK(result==Approx(1).margin(.01));

}