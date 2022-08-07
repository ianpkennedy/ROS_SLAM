
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <nuturtlebot_msgs/SensorData.h>
#include <nuturtlebot_msgs/WheelCommands.h>
#include "sensor_msgs/JointState.h"
#include "turtlelib/rigid2d.hpp"
#include "turtlelib/diff_drive.hpp"
#include <vector>



/// \file
/// \brief This node sets up the odometry calculation interface with turtlebot data. It subscribes to cmd_vel and sensor_data and publishes wheel_command and joint_states topics
///
/// PARAMETERS:
///     rate (integer): publishing rate in Hz
///     wheel_radius (float): radius of turtlebot wheel in meters
///     track_width (float): wheel distance of turtlebot in meters
///     encoder_ticks_to_rad (float): conversion factor from encoder ticks to radians
///     motor_cmd_to_radsec (float): conversion factor from motor command units to rad/sec
///     wheel_left (string): left wheel name
///     wheel_right (string): right wheel name
///     color (string): color of robot
///     motor_cmd_max (int): max wheel command
/// PUBLISHES:
///     wheel_command (nuturtlebot_msgs::WheelCommands): commands wheel velocities
///     joint_states (sensor_msgs::JointState): turtlebot joint states
///     red/joint_states (sensor_msgs::JointState): turtlebot simulation joint states
///     green/joint_states (sensor_msgs::JointState): turtlebot slam joint states

/// SUBSCRIBES:
///     cmd_vel (geometry_msgs::Twist): turtlebot twist vector
///     sensor_data (nuturtlebot_msgs::SensorData): turtlebot sensor data
/// SERVICES:
///     None



namespace{
    turtlelib::Diff_Drive diff1(0.0,0.0,0.0,0.0,0.0);
    // turtlelib::Diff_Drive diff1;
    turtlelib::Twist2D twist1;
    nuturtlebot_msgs::WheelCommands u_wheel;
    int f = 0;
    std::vector<double> v_wheel = {0.0,0.0};
    sensor_msgs::JointState wheel_states;

    sensor_msgs::JointState wheel_states_sim;

    sensor_msgs::JointState wheel_states_slam;



    // int left_old;
    // int right_old;
    // int left_new;
    // int right_new;

    double encoder_ticks_to_rad = 0.0;
    double motor_cmd_to_radsec = 0.0;

    int motor_cmd_max = 0;
    double initial = 0.0;


    std::string wheel_left = "undefined";
    std::string wheel_right = "undefined";


}

    /// \brief cmd_vel callback and computes wheel velocities
    /// \param msg - geometry_msgs::Twist data
    /// \returns None
void cmd_velCallback(const geometry_msgs::Twist& msg) //souce (01/29): https://answers.ros.org/question/259708/subscribing-and-publishing-geometrytwist-messages-from-turtlesim/
{   
    
    if((ros::Time::now().toSec()-initial) > 10.0){ // wait for all nodes to load, avoid erroneous encoder data for odometry 

    twist1.tw[0] = msg.angular.z;
    twist1.tw[1] = msg.linear.x;
    twist1.tw[2] = 0.0;
    
    v_wheel = diff1.inv_kin(twist1);

    wheel_states.velocity.at(0) = v_wheel.at(0);
    wheel_states.velocity.at(1) = v_wheel.at(1);;

    wheel_states_sim.velocity.at(0) = wheel_states.velocity.at(0);
    wheel_states_sim.velocity.at(1) = wheel_states.velocity.at(1);

    wheel_states_slam.velocity.at(0) = wheel_states.velocity.at(0);
    wheel_states_slam.velocity.at(1) = wheel_states.velocity.at(1);

    u_wheel.left_velocity = (int) (v_wheel.at(0)/motor_cmd_to_radsec); // Need to convert to a motor command unit (divide by 0.024)
    u_wheel.right_velocity = (int) (v_wheel.at(1)/motor_cmd_to_radsec);

    if(u_wheel.left_velocity<-motor_cmd_max){
        u_wheel.left_velocity = -motor_cmd_max;
    }else if(u_wheel.left_velocity>motor_cmd_max){
        u_wheel.left_velocity = motor_cmd_max;
    }

    if(u_wheel.right_velocity<-motor_cmd_max){
        u_wheel.right_velocity = -motor_cmd_max;
    }else if(u_wheel.right_velocity>motor_cmd_max){
        u_wheel.right_velocity = motor_cmd_max;
    }

    ROS_INFO_STREAM("wheel commands are [left right]: "<<u_wheel.left_velocity<<" "<<u_wheel.right_velocity<<"\n");
    }
}

    /// \brief sensor_data callback and computes joint_states
    /// \param msg - nuturtlebot_msgs::Sensor msg
    /// \returns None
void sensor_dataCallback(const nuturtlebot_msgs::SensorData& msg)
{


    wheel_states.position[0] = ((float) encoder_ticks_to_rad * msg.left_encoder);
    wheel_states.position[1] = ((float) encoder_ticks_to_rad * msg.right_encoder);

    wheel_states_sim.position[0] = wheel_states.position[0];
    wheel_states_sim.position[1] = wheel_states.position[1];

    wheel_states_slam.position[0] = wheel_states.position[0];
    wheel_states_slam.position[1] = wheel_states.position[1];

}

int main(int argc, char * argv[]){
    
    double wheel_radius = 0;
    double track_width = 0;

    std::string undefined = "undefined";
    std::string color = "";


    ros::init(argc,argv,"turtle_interface");
    ros::NodeHandle n;

    //Load in parameters
    n.param("turtle_interface/rate", f, 500); //Source (01/29): https://answers.ros.org/question/253797/no-matching-function-for-call-to-rosnodehandleparam/
    n.param("wheel_radius", wheel_radius, -1.0);
    n.param("track_width", track_width, -1.0);
    n.param("encoder_ticks_to_rad",encoder_ticks_to_rad,0.001534);
    n.param("motor_cmd_to_radsec",motor_cmd_to_radsec,0.024);

    n.param("wheel_left",wheel_left,undefined);
    n.param("wheel_right",wheel_right,undefined);
    n.param("turtle_interface/color",color,color);
    n.param("motor_cmd_max", motor_cmd_max, 256);

    if(!(n.hasParam("turtle_interface/rate")) ||   !(n.hasParam("wheel_radius")) ||  !(n.hasParam("track_width")) ||  !(n.hasParam("encoder_ticks_to_rad")) ||  !(n.hasParam("motor_cmd_to_radsec")) ){
        ROS_ERROR_STREAM("Undefined parameter - terminating!\n");
        throw;
    }

    if(!(n.hasParam("wheel_left")) ||   !(n.hasParam("wheel_right")) ||  !(n.hasParam("turtle_interface/color")) ||  !(n.hasParam("motor_cmd_max"))  ){
        ROS_ERROR_STREAM("Undefined parameter - terminating!\n");
        throw;
    }

    //Subscribers
    ros::Subscriber s_twist = n.subscribe("cmd_vel", 100, cmd_velCallback);
    ros::Subscriber s_sens = n.subscribe("sensor_data", 100, sensor_dataCallback);

    //Publishers
    ros::Publisher wheel_cmd_pub = n.advertise<nuturtlebot_msgs::WheelCommands>("wheel_cmd", 100);
    ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 100);
    ros::Publisher joint_pub_sim = n.advertise<sensor_msgs::JointState>("red/joint_states", 100);
    ros::Publisher joint_pub_slam = n.advertise<sensor_msgs::JointState>("green/joint_states", 100);


    ros::Rate rate(f);

    //Throw is parameters undefined in parameter server
    if(track_width < 0 || wheel_radius < 0 || motor_cmd_to_radsec < 0 || encoder_ticks_to_rad < 0){
        ROS_ERROR_STREAM("Undefined parameter - terminating!\n");
        throw;
    }

    //Constructing differential drive robot
    diff1 = turtlelib::Diff_Drive(track_width/2.0,wheel_radius,0.0,0.0,0.0);
    

    //Declare joint states
    wheel_states.name =  {color+"wheel_left_joint",color+"wheel_right_joint"};
    wheel_states.position = {1.0,0.0};
    wheel_states.velocity = {0.0,0.0};
    wheel_states.effort = {0.0,0.0};


    wheel_states_sim.name =  {"red-wheel_left_joint","red-wheel_right_joint"};
    wheel_states_sim.position = {0.0,0.0};
    wheel_states_sim.velocity = {0.0,0.0};
    wheel_states_sim.effort = {0.0,0.0};

    wheel_states_slam.name =  {"green-wheel_left_joint","green-wheel_right_joint"};
    wheel_states_slam.position = {0.0,0.0};
    wheel_states_slam.velocity = {0.0,0.0};
    wheel_states_slam.effort = {0.0,0.0};

    initial = ros::Time::now().toSec();


    while(ros::ok()){

        //Publish and spin
        wheel_cmd_pub.publish(u_wheel);

        wheel_states.header.stamp = ros::Time::now();
        wheel_states_sim.header.stamp = ros::Time::now();
        wheel_states_slam.header.stamp = ros::Time::now();

        
        joint_pub.publish(wheel_states);
        joint_pub_sim.publish(wheel_states_sim);
        joint_pub_slam.publish(wheel_states_slam);

        ros::spinOnce();
        rate.sleep(); 
    }

    return 0;
}