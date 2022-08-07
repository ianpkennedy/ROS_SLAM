


#include "ros/ros.h"
#include <nuturtlebot_msgs/WheelCommands.h>
#include <nuturtlebot_msgs/SensorData.h>
#include <geometry_msgs/Twist.h>
#include <catch_ros/catch.hpp>
#include "turtlelib/rigid2d.hpp"
#include "turtlelib/diff_drive.hpp"
#include "sensor_msgs/JointState.h"
#include <vector>
#include <cmath>

/// \file
/// \brief Testing file for turtle_interface class


namespace{
    std::vector<int> wheel_test = {0,0};
    std::vector<double> states = {0.0,0.0};
    
}


    /// \brief subscribes to wheel commands
    /// \param msg - nuturtlebot_msgs::WheelCommands
    /// \returns None
void wheel_commandCallback(const nuturtlebot_msgs::WheelCommands& msg){

    ROS_INFO_STREAM("In test callback\n");

    //Update wheel velocities
    if(msg.left_velocity != 0){
        wheel_test[0] = msg.left_velocity;
        wheel_test[1] =  msg.right_velocity;
    }
}


    /// \brief subscribes to joint states
    /// \param msg - nuturtlebot_msgs::WheelCommands
    /// \returns None
void joint_statesCallback(const sensor_msgs::JointState& msg){
    
    states.at(0) = msg.position.at(0);
    states.at(1) = msg.position.at(1);

    
}

/// \brief test pure translation
TEST_CASE("Translation wheel command","[wheel_command]"){

    int i = 0;
    ros::NodeHandle n; 

    //Subscribers
    ros::Subscriber wheel_cmd_sub = n.subscribe("wheel_cmd", 100, wheel_commandCallback);

    //Publishers
    ros::Publisher twist_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 100);
    
    //Declare twist
    geometry_msgs::Twist test;
    test.linear.x = 1.0;
    test.linear.y = 0.0;
    test.linear.z = 0.0;
    test.angular.x = 0.0;
    test.angular.y = 0.0;
    test.angular.z = 0.0;


    ros::Rate rate(100);
    double initial=ros::Time::now().toSec();
    



    //Wait for result
    while(i<100){
        twist_pub.publish(test);
        ros::spinOnce();
        i++;
        rate.sleep(); 
     }

    while((ros::Time::now().toSec()-initial) < 11.0){
        twist_pub.publish(test);
        ros::spinOnce();
        i++;
        rate.sleep();
    }


    REQUIRE(wheel_test[0]==wheel_test[1]);
    REQUIRE(abs(wheel_test[0]) > 0);

}

/// \brief test pure rotation
TEST_CASE("Rotation wheel command","[wheel_command]"){

    int i = 0;
    ros::NodeHandle n; 

    //Subscribers
    ros::Subscriber wheel_cmd_sub = n.subscribe("wheel_cmd", 100, wheel_commandCallback);

    //Publishers
    ros::Publisher twist_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 100);
    
    //declare twist
    geometry_msgs::Twist test;
    test.linear.x = 0.0;
    test.linear.y = 0.0;
    test.linear.z = 0.0;
    test.angular.x = 0.0;
    test.angular.y = 0.0;
    test.angular.z = 1.0;

    
    ros::Rate rate(100);
    double initial=ros::Time::now().toSec();
    

    //Wait for result
    while(i<100){
        twist_pub.publish(test);
        ros::spinOnce();
        i++;
        rate.sleep(); 

     }

    while((ros::Time::now().toSec()-initial) < 11.0){
        twist_pub.publish(test);
        ros::spinOnce();
        i++;
        rate.sleep();
    }

    REQUIRE(-wheel_test[0]==wheel_test[1]);
    REQUIRE(abs(wheel_test[0]) > 0);

}

/// \brief test joint states conversion
TEST_CASE("Joint states conversion verification","[joint_states]"){

    int i = 0;
    ros::NodeHandle n; 

    //Subscribers
    ros::Subscriber states_sub = n.subscribe("joint_states", 100, joint_statesCallback);

    //Publishers
    ros::Publisher sensor_pub = n.advertise<nuturtlebot_msgs::SensorData>("sensor_data", 100);


    nuturtlebot_msgs::SensorData sense;
    sense.left_encoder = 2048/3; //pi/3
    sense.right_encoder = 3072; //1.5pi

    ros::Rate rate(100);
    
    //Wait for result
    while(i<100){
        sense.stamp = ros::Time::now();
        sensor_pub.publish(sense);
        ros::spinOnce();
        i++;
        rate.sleep(); 
    }


    REQUIRE(turtlelib::PI/3==Approx(states.at(0)).margin(.01));
    REQUIRE(turtlelib::PI*1.5==Approx(states.at(1)).margin(.01));

    
}