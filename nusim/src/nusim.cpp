

#include "ros/ros.h"
#include "std_msgs/UInt64.h"
#include "std_srvs/Empty.h"
#include "nusim/Teleport.h"
#include "nusim/TeleportRequest.h"
#include "nusim/TeleportResponse.h"
#include <nav_msgs/Path.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/transform_broadcaster.h>

#include "sensor_msgs/JointState.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/TransformStamped.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "nuturtlebot_msgs/WheelCommands.h"
#include "nuturtlebot_msgs/SensorData.h"
#include "sensor_msgs/LaserScan.h"

#include "turtlelib/rigid2d.hpp"
#include "turtlelib/diff_drive.hpp"
#include<random>
#include <cmath>

/// \file
/// \brief This node runs the nusimulator. It loads in robot and obstacles into RVIZ
///
/// PARAMETERS:
///     rate (integer): publishing rate
///     x0 (double): x origin of robot
///     y0 (double): y origin of robot
///     theta0 (double): orientation origin of robot
///     wheel_radius (double): radius of wheels
///     track_width (double): wheel spacing
///     motor_cmd_to_radsec (double): wheel cmd conversion factor
///     encoder_ticks_to_rad (double): encoder ticks to angle conversion factor
///     rviz_color (string): color of robot
///     gauss_noise (double): robot noise variance
///     slip_min (double): minimum slip
///     slip_max (double): maximum slip
///     basic_sensor_variance (double): noise on LiDAR sensor
///     max_range (double): maximum range of LiDAR
///     min_range (double): minimum range of LiDAR
///     collision_radius (double): collision radius of robot
///     resolution (int): sensor resolution
///     angle_increment (int): angular increment for sensor
/// PUBLISHES:
///     ~timestep (std_msgs::UInt64): simulation timestep
///     transform_broadcaster between world and red-base_footprint
///     red/sensor_data (nuturtlebot_msgs::SensorData): sensor data for the turtlebot simulation
///     obstacles (visualization_msgs::MarkerArray): cylinders and walls in the simulation in RVIZ 
///     /fake_sensor (visualization_msgs::MarkerArray): LiDAR detection of obstacles
///     laser_sim (sensor_msgs::LaserScan): LiDAR range and bearing data
///     pathreal (nav_msgs::Path): odometry path    
/// SUBSCRIBES:
///     /red/wheel_cmd: (nuturtlebot_msgs::WheelCommands): motion commands received for the nusimulation
/// SERVICES:
///     reset (std_srvs::Empty): This service resets the simulation
///     teleport (nusim::Teleport): This service teleports the robot to x,y,theta defined by user


using std::vector;
using std::cout;

namespace{
    turtlelib::Diff_Drive diff1(0.0,0.0,0.0,0.0,0.0);
    turtlelib::Transform2D T;
    sensor_msgs::LaserScan laser_sim;

    int f = 0;

    int counter = 0;
   
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;
    vector<double> v_wheel = {0.0,0.0};
    vector<double> pos_wheel = {0.0,0.0};

    vector<double> pos_wheel_noslip = {0.0,0.0};

    
    double xorigin = 0.0;
    double theta0 = 0.0;
    double yorigin = 0.0;

    double encoder_ticks_to_rad = 0.0;
    double motor_cmd_to_radsec = 0.0;

    double wheel_radius = 0.0;
    double track_width = 0.0;

    double now = 0.0;
    double before  = 0.0;

    bool use_sim = false;

    double gauss_noise = 0.0;
    double basic_sensor_variance = 0.0;


    double slip_min = 0.0;
    double slip_max = 0.0;

    //Obstacle coordinates
    vector<double> o_r;
    vector<double> o_x;
    vector<double> o_y;

    double x_length = 0;
    double y_length = 0;
    double thickness = 0;

    //Lidar range
    double max_range = 0.0;

    int resolution = 0;


    double angle_increment = 0.0;

    bool pubflag = false;


}
    /// \brief seed a randome variable
    /// \returns boolean true upon completion of service
 std::mt19937 & get_random()
 {
     // static variables inside a function are created once and persist for the remainder of the program
     static std::random_device rd{}; 
     static std::mt19937 mt{rd()};
     // we return a reference to the pseudo-random number genrator object. This is always the
     // same object every time get_random is called
     return mt;
 }

    /// \brief reset simulation to start
    /// \returns boolean true upon completion of service
bool reset(std_srvs::Empty::Request& , std_srvs::Empty::Response& ){

    counter = 0;

    diff1 = turtlelib::Diff_Drive(track_width/2.0,wheel_radius,xorigin,yorigin,theta0);
    T = diff1.fkin(pos_wheel[0],pos_wheel[1]);


    return true;
}

    /// \brief teleport the robot to an x,y,theta position in the world frame
    /// \param request - x,y, theta coordinates to teleport to
    /// \returns None
bool teleport(nusim::Teleport::Request& request, nusim::Teleport::Response& ){


    diff1 = turtlelib::Diff_Drive(track_width/2.0,wheel_radius,request.x,request.y,request.t);
    T = diff1.fkin(pos_wheel[0],pos_wheel[1]);


    return true;
}

    /// \brief wheel_cmd callback to computes wheel velocities and positions
    /// \param msg - nuturtlebot_msgs::WheelCommands data
    /// \returns None
void wheel_cmdCallback(const nuturtlebot_msgs::WheelCommands& msg)
{
    
    if(use_sim){
        double dtheta_l = 0.0;
        double dtheta_r = 0.0;



        //Update velocities
        v_wheel[0] =  motor_cmd_to_radsec * msg.left_velocity;    
        v_wheel[1] =  motor_cmd_to_radsec * msg.right_velocity;


        //Add gaussian noise to wheel velocities . take out noise if it is 0
        std::normal_distribution<> vlr(0.0, gauss_noise); //02/15: https://en.cppreference.com/w/cpp/numeric/random/normal_distribution
        std::normal_distribution<> vrr(0.0, gauss_noise);


        double rand_v_l = vlr(get_random());
        double rand_v_r = vrr(get_random());

        if(msg.left_velocity == 0){
            rand_v_l = 0.0;
        }


        if(msg.right_velocity == 0){
            rand_v_r = 0.0;
        }
        //calculate instaneous wheel velocity;
        double v_left = v_wheel[0] + rand_v_l;
        double v_right = v_wheel[1] + rand_v_r;



        //Update wheel positions
        now = ros::Time::now().toSec();

        dtheta_l = (float) (v_left * (now - before));
        dtheta_r = (float) (v_right *  (now - before));

        before = now;

        pos_wheel.at(0) = pos_wheel.at(0)+dtheta_l;              
        pos_wheel.at(1)= pos_wheel.at(1)+dtheta_r;

        pos_wheel_noslip.at(0) = pos_wheel_noslip.at(0) + dtheta_l;
        pos_wheel_noslip.at(1) = pos_wheel_noslip.at(1) + dtheta_r;

       //add new pos_wheel  


        //Update world transform to body 
        // T = diff1.fkin(pos_wheel[0],pos_wheel[1]);
        T = diff1.fkin(pos_wheel_noslip[0],pos_wheel_noslip[1]);


        //Add noise to wheel positions
        std::uniform_real_distribution<> pr(slip_min,slip_max); //02/15: https://en.cppreference.com/w/cpp/numeric/random/uniform_real_distribution

        double rand_p = pr(get_random());

        pos_wheel.at(0) += rand_p*v_left;             
        pos_wheel.at(1) += rand_p*v_right;

    }
}


    /// \brief controls a to publish laser and fake_sensor data at 5 Hz
    /// \param None
    /// \returns None
void timerCallback(const ros::TimerEvent& event){
    pubflag = true;
    std::cout<<"expected " << event.current_expected<<"\r\n";

}

    /// \brief determines whether to and teleports robot
    /// \param x_coord - x coordinate of obstacle
    /// \param y_coord - y coordinate of obstacle
    /// \returns None
void collision_avoid(double x_coord, double y_coord, double diameter, double collision_param){
    
    double r0 = diameter/2; //Radius of posts
    double r1 = collision_param; //read in parameter

    double m = (y_coord-T.translation().y)/(x_coord-T.translation().x);
    // double b = y_coord - m*x_coord;
    double d = sqrt((T.translation().x - x_coord)*(T.translation().x - x_coord) + (T.translation().y - y_coord)*(T.translation().y - y_coord) );
    
    if (d < (r0 + r1)){

        double delta = (r0+r1) - d;
        // double x1 = (-2*b*m+sqrt( 2*b*m*2*b*m - 4*(1+m*m)*b*b))/(2*(1+m*m));
        // double x2 = (-2*b*m-sqrt( 2*b*m*2*b*m - 4*(1+m*m)*b*b))/(2*(1+m*m));
        double x1 = + sqrt(delta*delta/(1+m*m));
        double x2 = - sqrt(delta*delta/(1+m*m));

        double y1 = m*x1;
        double y2 = m*x2;

        double xworld1 = x1 + T.translation().x;
        double xworld2 = x2 + T.translation().x;

        double yworld1 = y1 + T.translation().y;
        double yworld2 = y2 + T.translation().y;



        double ang = T.rotation();
        turtlelib::Vector2D v_temp;

        if(  (x_coord - T.translation().x) > 0  ){

                diff1 = turtlelib::Diff_Drive(track_width/2.0,wheel_radius,xworld2,yworld2,ang,pos_wheel.at(0),pos_wheel.at(1));
                T = diff1.fkin(pos_wheel[0],pos_wheel[1]);
        } else{


                    diff1 = turtlelib::Diff_Drive(track_width/2.0,wheel_radius,xworld1,yworld1,ang,pos_wheel.at(0),pos_wheel.at(1));
                    T = diff1.fkin(pos_wheel[0],pos_wheel[1]);
  
        }
    } 
}

    /// \brief compute the intersection with a circle
    /// \param beams  - number of beams per spin
    /// \param range - range of laser
    /// \returns None
void intersect(double beams, double range){

    long unsigned int k = 0;
    long unsigned int n;
    int j = 0;
    double m = 0.0;
    double ang = 0.0;

    double k_d = 0.0;

    //In robot frame
    double x1 = 0.0;
    double y1 = 0.0;
    double x2 = range;
    double y2 = 0.0;


    //robot frame - for circles
    vector<double> x_intercept = {};
    vector<double> y_intercept = {};

    //robot frame - for walls
    vector<double> x_intercept_wall = {};
    vector<double> y_intercept_wall = {};




    turtlelib::Vector2D v_obst; // translation vector in world for each obstacle
    turtlelib::Transform2D T_wo;
    turtlelib::Transform2D T_bw;
    turtlelib::Transform2D T_bo;
    turtlelib::Transform2D T_ob;
    turtlelib::Vector2D v1;
    turtlelib::Vector2D v2;
    turtlelib::Vector2D v1o;
    turtlelib::Vector2D v2o;

    turtlelib::Vector2D v_wall_w;
    turtlelib::Vector2D v_wall_b;
    
    turtlelib::Vector2D v_int_o;
    turtlelib::Vector2D v_int_o1;
    turtlelib::Vector2D v_int_o2;

    turtlelib::Vector2D v_int_b;
    turtlelib::Vector2D v_int_b1;
    turtlelib::Vector2D v_int_b2;


    for(j=0;j<beams;j++){

        x1 = 0.0;
        y1 = 0.0;
        x2 = range/1.0;//3.5
        y2 = 0.0;

       laser_sim.ranges.at(j) = range;
       x_intercept = {};
       y_intercept = {};

        x_intercept_wall = {};
        y_intercept_wall = {};

       ang = turtlelib::deg2rad(j);
       m=tan(ang);
       y2 = m*x2;


       double dr = sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1)); //Distance is the same regardless of frame



       for(k=0;k<o_x.size();k++){

            v_obst.x = o_x.at(k);
            v_obst.y = o_y.at(k);

            T_bw = T.inv(); //T is T_wb
            T_wo = turtlelib::Transform2D(v_obst); //pure translation
            T_bo = T_bw*T_wo;
            T_ob = T_bo.inv();  
            
            v1.x = x1; //body of robot frame
            v1.y = y1;
            v2.x = x2;
            v2.y = y2;

            v1o = T_ob(v1);
            v2o = T_ob(v2);


            double r_c = o_r.at(k)/2;
            // double discrim = r_c*r_c*dr*dr - (x1*y2 - x2*y1);
            double D = (v1o.x*v2o.y - v2o.x*v1o.y);
            double discrim = r_c*r_c*dr*dr - D*D;

            double dx = v2o.x - v1o.x; //obstacle frame
            double dy = v2o.y - v1o.y;
            

            if((turtlelib::almost_equal(discrim,0.0,1.0e-2))){
                //Calculate intersection points
                double x_i0 = (D*dy)/(dr*dr);
                double y_i0 = (-D*dx)/(dr*dr);

                v_int_o.x = x_i0;
                v_int_o.y = y_i0;

                //Transform back to robot coordinates
                v_int_b = T_bo(v_int_o);
                x_intercept.push_back(v_int_b.x);
                y_intercept.push_back(v_int_b.y);

            }


            if(discrim > 0.0001){
                //Calculate intersection points - the x values are not behaving correctly
                double x_i1 = ((D*dy)+(abs(dy)/dy)*dx*sqrt(discrim))/(dr*dr);
                double x_i2 = ((D*dy)-(abs(dy)/dy)*dx*sqrt(discrim))/(dr*dr);
                
                double y_i1 = ((-D*dx)+abs(dy)*sqrt(discrim))/(dr*dr);
                double y_i2 = ((-D*dx)-abs(dy)*sqrt(discrim))/(dr*dr);

                v_int_o1.x = x_i1;
                v_int_o1.y = y_i1;

                v_int_o2.x = x_i2;
                v_int_o2.y = y_i2;

                //Transform back to robot coordinates
                v_int_b1 = T_bo(v_int_o1);
                v_int_b2 = T_bo(v_int_o2);
                                
                x_intercept.push_back(v_int_b1.x);
                y_intercept.push_back(v_int_b1.y);
                
                x_intercept.push_back(v_int_b2.x);
                y_intercept.push_back(v_int_b2.y);


            } if(discrim<-0.1){



            }

            
        }

        // cout << " x_length " << x_length <<"\r \n";
        

        //Top wall
        for(k_d = -x_length/2 ; k_d<x_length/2; k_d +=0.02 ){

            v_wall_w.x = k_d;
            v_wall_w.y = y_length/2-thickness/2;

            v_wall_b = T_bw(v_wall_w);

            if(turtlelib::almost_equal((v_wall_b.y/v_wall_b.x),(y2/x2), 0.02)){
                x_intercept_wall.push_back(v_wall_b.x);
                y_intercept_wall.push_back(v_wall_b.y);
            }
        }


        //Bottom wall
        for(k_d = -x_length/2 ; k_d<x_length/2; k_d +=0.02 ){

            v_wall_w.x = k_d;
            v_wall_w.y = -y_length/2+thickness/2;

            v_wall_b = T_bw(v_wall_w);

            if(turtlelib::almost_equal((v_wall_b.y/v_wall_b.x),(y2/x2), 0.02)){
                x_intercept_wall.push_back(v_wall_b.x);
                y_intercept_wall.push_back(v_wall_b.y);
            }
        }

        //Left wall
        for(k_d = -y_length/2 ; k_d<y_length/2; k_d +=0.02 ){

            v_wall_w.x = -x_length/2+thickness/2;
            v_wall_w.y = k_d;

            v_wall_b = T_bw(v_wall_w);

            if(turtlelib::almost_equal((v_wall_b.y/v_wall_b.x),(y2/x2), 0.02)){
                x_intercept_wall.push_back(v_wall_b.x);
                y_intercept_wall.push_back(v_wall_b.y);
            }
        }

        //right wall
        for(k_d = -y_length/2 ; k_d<y_length/2; k_d +=0.02){

            v_wall_w.x = x_length/2-thickness/2;
            v_wall_w.y = k_d;

            v_wall_b = T_bw(v_wall_w);

            if(turtlelib::almost_equal((v_wall_b.y/v_wall_b.x),(y2/x2), 0.02)){
                x_intercept_wall.push_back(v_wall_b.x);
                y_intercept_wall.push_back(v_wall_b.y);
            }
        }


        double result = range; //3.5 lidar range for turtlebot
        double resultx = 0.0;
        double resulty = 0.0;


        for(n=0;n<x_intercept.size();n++){
            
            if(sqrt(x_intercept.at(n)*x_intercept.at(n)+y_intercept.at(n)*y_intercept.at(n)) < result ){
                result = sqrt(x_intercept.at(n)*x_intercept.at(n)+y_intercept.at(n)*y_intercept.at(n));
                resultx = x_intercept.at(n);
                resulty = y_intercept.at(n);

            }
        }



        //Reset if bad quadrant - these ranges change based on the resolution of the sensor. HEre they are assuming 360 positions.

        if( (0<j  && j < 90)  && (resultx<0 || resulty<0)  ){
            result = range;

        }

        if( (90<j  && j < 180)  && (resultx>0 || resulty<0)  ){
            result = range;

        }

        if( (180<j  && j < 270)  && (resultx>0 || resulty>0)  ){
            result = range;
        }

        if( (270<j  && j < 360)  && (resultx<0 || resulty>0)  ){
            result = range;
        }

        //Add walls - same note for the j values ranges here with regards to j ranges. Should be update if resolution is changed.
        for(n=0;n<x_intercept_wall.size();n++){
            
            if( (0<j  && j < 90)  && (x_intercept_wall.at(n)>0 && y_intercept_wall.at(n)>0)  ){
                if(sqrt(x_intercept_wall.at(n)*x_intercept_wall.at(n)+y_intercept_wall.at(n)*y_intercept_wall.at(n)) < result ){
                    result = sqrt(x_intercept_wall.at(n)*x_intercept_wall.at(n)+y_intercept_wall.at(n)*y_intercept_wall.at(n));
                    resultx = x_intercept_wall.at(n);
                    resulty = y_intercept_wall.at(n);

                }
            }

            if( (90<j  && j < 181)  && (x_intercept_wall.at(n)<0 && y_intercept_wall.at(n)>0)  ){
                if(sqrt(x_intercept_wall.at(n)*x_intercept_wall.at(n)+y_intercept_wall.at(n)*y_intercept_wall.at(n)) < result ){
                    result = sqrt(x_intercept_wall.at(n)*x_intercept_wall.at(n)+y_intercept_wall.at(n)*y_intercept_wall.at(n));
                    resultx = x_intercept_wall.at(n);
                    resulty = y_intercept_wall.at(n);

                }
            }
            if( (180<j  && j < 271)  && (x_intercept_wall.at(n)<0 && y_intercept_wall.at(n)<0)  ){
                if(sqrt(x_intercept_wall.at(n)*x_intercept_wall.at(n)+y_intercept_wall.at(n)*y_intercept_wall.at(n)) < result ){
                    result = sqrt(x_intercept_wall.at(n)*x_intercept_wall.at(n)+y_intercept_wall.at(n)*y_intercept_wall.at(n));
                    resultx = x_intercept_wall.at(n);
                    resulty = y_intercept_wall.at(n);

                }
            }
            if( (270<j  && j < 361)  && (x_intercept_wall.at(n)>0 && y_intercept_wall.at(n)<0)  ){
                if(sqrt(x_intercept_wall.at(n)*x_intercept_wall.at(n)+y_intercept_wall.at(n)*y_intercept_wall.at(n)) < result ){
                    result = sqrt(x_intercept_wall.at(n)*x_intercept_wall.at(n)+y_intercept_wall.at(n)*y_intercept_wall.at(n));
                    resultx = x_intercept_wall.at(n);
                    resulty = y_intercept_wall.at(n);

                }
            }

        }


        std::normal_distribution<> laser_seed(0.0, basic_sensor_variance);
        double laser_random = laser_seed(get_random());

        laser_sim.ranges.at(j) = result;

        if (turtlelib::almost_equal(laser_sim.ranges.at(j),range, 0.01)){
            laser_sim.ranges.at(j) = 0.0;
        } else{
            //Update list
            laser_sim.ranges.at(j) = result + laser_random;
        }

    }




}

int main(int argc, char * argv[]){

    int trail_count = 0;

    long unsigned int i = 0;

    // int period = 0;

    double min_range = 0.0;

    double collision_radius = 0.0;


    std::string color="r";

    ros::init(argc,argv,"nusim");

    ros::NodeHandle nh("~");

    visualization_msgs::Marker m;
    visualization_msgs::Marker m_sense;


    visualization_msgs::MarkerArray m_array;
    visualization_msgs::MarkerArray m_array_sense;



    //Load in parameters from basic_world.yaml
    nh.param("rate", f, 500);
    nh.param("x0", xorigin, 0.0);
    nh.param("theta0", theta0, 0.0);
    nh.param("y0",yorigin,0.0);
    nh.param("wheel_radius", wheel_radius, 0.033);
    nh.param("track_width", track_width, 0.16);
    nh.param("motor_cmd_to_radsec",motor_cmd_to_radsec,0.024);
    nh.param("encoder_ticks_to_rad",encoder_ticks_to_rad,0.001534);
    nh.param("rviz_color", color,color);
    nh.param("gauss_noise",gauss_noise,0.0);
    nh.param("slip_min",slip_min,0.0);
    nh.param("slip_max",slip_max,0.0);
    nh.param("basic_sensor_variance",basic_sensor_variance,0.0);
    nh.param("max_range",max_range,3.5);
    nh.param("min_range", min_range, 0.12);
    nh.param("collision_radius", collision_radius, 0.25);
    nh.param("resolution", resolution, 360); //In degrees
    nh.param("angle_increment", angle_increment, 1.0); // In degrees

    // int period = f / 5.56; //Publish at 5 Hz


    if(!(nh.getParam("use_sim",use_sim))){
        throw;
    }

    nh.getParam("obstacles/x",o_x); //noservice needed, just read from the yaml file and create from the yaml
    nh.getParam("obstacles/y",o_y);
    nh.getParam("obstacles/r",o_r);        
    nh.getParam("obstacles/x_length",x_length);        
    nh.getParam("obstacles/y_length",y_length);        
    nh.getParam("obstacles/thickness",thickness);        


    //Define differential model
    diff1 = turtlelib::Diff_Drive(track_width/2.0,wheel_radius,xorigin,yorigin,theta0);

    ros::Duration life(0);
    ros::Duration life_sense(0.5);

    // Add obstacles
    for(i=0;i<o_x.size();i++){
        m.header.frame_id = "world";
        m.header.stamp = ros::Time::now();
        m.ns = "obstacles";
        m.action = visualization_msgs::Marker::ADD;
        m.id = i;
        m.type = visualization_msgs::Marker::CYLINDER;
        
        m.scale.x = o_r[i];
        m.scale.y = o_r[i];
        m.scale.z = 0.25;
        
        m.color.r = 0.0;
        m.color.g = 0.0;
        m.color.b = 0.0;

        if(color=="r"){
            m.color.r = 1.0;
        }else if(color=="g"){
            m.color.g = 1.0;
        }else if(color=="b"){
            m.color.b = 1.0;
        }

        m.color.a = 1.0;

        m.pose.orientation.x = 0.0;
        m.pose.orientation.y = 0.0;
        m.pose.orientation.z = 0.0;
        m.pose.orientation.w = 1.0;
        
        m.pose.position.x = o_x[i];
        m.pose.position.y = o_y[i];
        m.pose.position.z = 0.25;

        m.lifetime = life;
        m_array.markers.push_back(m); //source (01/18): https://answers.ros.org/question/35246/add-markers-to-markerarray-in-c/

    }

    //Create the walls
    for(i=o_x.size();i<o_x.size()+4;i++){
        m.header.frame_id = "world";
        m.header.stamp = ros::Time::now();
        m.ns = "obstacles";
        m.action = visualization_msgs::Marker::ADD;
        m.id = i;
        m.type = visualization_msgs::Marker::CUBE;
        
        m.color.r = 0.0;
        m.color.g = 0.0;
        m.color.b = 0.0;

        if(color=="r"){
            m.color.r = 1.0;
        }else if(color=="g"){
            m.color.g = 1.0;
        }else if(color=="b"){
            m.color.b = 1.0;
        }


        if(i==o_x.size()){
           m.scale.x = x_length;
           m.scale.y = thickness;
           m.scale.z = 0.01;        

           m.pose.orientation.x = 0.0;
           m.pose.orientation.y = 0.0;
           m.pose.orientation.z = 0.0;
           m.pose.orientation.w = 1.0;
        
           m.pose.position.x = 0;
           m.pose.position.y = y_length/2;
           m.pose.position.z = 0.0;
        }
        if(i==o_x.size()+1){
           m.scale.x = x_length;
           m.scale.y = thickness;
           m.scale.z = 0.01;        

           m.pose.orientation.x = 0.0;
           m.pose.orientation.y = 0.0;
           m.pose.orientation.z = 0.0;
           m.pose.orientation.w = 1.0;
        
           m.pose.position.x = 0;
           m.pose.position.y = -y_length/2;
           m.pose.position.z = 0.0;
        }

      if(i==o_x.size()+2){
           m.scale.x = thickness;
           m.scale.y = y_length;
           m.scale.z = 0.01;        

           m.pose.orientation.x = 0.0;
           m.pose.orientation.y = 0.0;
           m.pose.orientation.z = 0.0;
           m.pose.orientation.w = 1.0;
        
           m.pose.position.x = x_length/2;
           m.pose.position.y = 0;
           m.pose.position.z = 0.0;
        }

        if(i==o_x.size()+3){
           m.scale.x = thickness;
           m.scale.y = y_length;
           m.scale.z = 0.01;        

           m.pose.orientation.x = 0.0;
           m.pose.orientation.y = 0.0;
           m.pose.orientation.z = 0.0;
           m.pose.orientation.w = 1.0;
        
           m.pose.position.x = -x_length/2;
           m.pose.position.y = 0;
           m.pose.position.z = 0.0;
        }
        m.lifetime = life;

        m_array.markers.push_back(m); //source (01/18): https://answers.ros.org/question/35246/add-markers-to-markerarray-in-c/

    }



    x = xorigin;
    y = yorigin;
    theta = theta0;


    //Declare publishers
    ros::Publisher m_pub;
    m_pub = nh.advertise<visualization_msgs::MarkerArray>("obstacles", 100, true);

    m_pub.publish(m_array);

    ros::Publisher count_pub;
    count_pub = nh.advertise<std_msgs::UInt64>("timestep", 100);

    ros::Publisher sensor_data_pub = nh.advertise<nuturtlebot_msgs::SensorData>("red/sensor_data", 100);
    ros::Subscriber s_w_cmd = nh.subscribe("/red/wheel_cmd", f, wheel_cmdCallback);

    ros::Publisher lidar_pub = nh.advertise<visualization_msgs::MarkerArray>("/fake_sensor", 100);
    ros::Publisher sensor_pub = nh.advertise<sensor_msgs::LaserScan>("laser_sim", 100);
    ros::Publisher trail_pub_red = nh.advertise<nav_msgs::Path>("pathreal", 100);


    ros::Timer timer_pub = nh.createTimer(ros::Duration(0.2), timerCallback);

    //Define sensed obstacles from lidar
    for(i=0;i<o_x.size();i++){
        m_sense.header.frame_id = "red-base_footprint";
        m_sense.header.stamp = ros::Time::now();
        m_sense.ns = "sense";
        m_sense.action = visualization_msgs::Marker::ADD;
        m_sense.id = i;
        m_sense.type = visualization_msgs::Marker::SPHERE;

        m_sense.scale.x = o_r[i];
        m_sense.scale.y = o_r[i];
        m_sense.scale.z = o_r[i]; 

        m_sense.color.r = 1.0;
        m_sense.color.g = 1.0;
        m_sense.color.b = 0.0;
        m_sense.color.a = 1.0;

        m_sense.pose.orientation.x = 0.0;
        m_sense.pose.orientation.y = 0.0;
        m_sense.pose.orientation.z = 0.0;
        m_sense.pose.orientation.w = 1.0;
        
        m_sense.pose.position.x = o_x[i];
        m_sense.pose.position.y = o_y[i];
        m_sense.pose.position.z = 0.5;

        m_sense.frame_locked = true;

        m_sense.lifetime = life_sense;

        m_array_sense.markers.push_back(m_sense);

    }

    if((use_sim)){
    }

    //Declare services
    ros::ServiceServer srv_reset;
    srv_reset = nh.advertiseService("reset", reset);

    ros::ServiceServer srv_tele;
    srv_tele = nh.advertiseService("teleport", teleport);

    //Set up broadcaster
    tf2_ros::TransformBroadcaster b;
    geometry_msgs::TransformStamped ts;

    ros::Rate rate(f);

    //Initiate position
    T = diff1.fkin(pos_wheel[0],pos_wheel[1]);



    //Update with parameters
    laser_sim.header.frame_id = "red-base_scan";
    laser_sim.angle_min = 0.0;
    laser_sim.angle_increment = 2*turtlelib::PI /resolution;
    laser_sim.angle_max = 2*turtlelib::PI;
    laser_sim.range_min = min_range;
    laser_sim.range_max = max_range;
    laser_sim.scan_time = 0.2; //0.2 or 0.0
    // laser_sim.scan_time = 1/6.0; //1 Hz more than the publication rate, otherwise there is a transform failure
    laser_sim.time_increment = laser_sim.scan_time / 360; //Update with parameter


    for(i=0;i<360;i++){
        laser_sim.ranges.push_back(0.5);
    }

    //Define path
    nav_msgs::Path trail;
    trail.header.frame_id = "world"; 
    geometry_msgs::PoseStamped pose_stamp;
    pose_stamp.header.frame_id = "world"; 

    while(ros::ok()){


        //Continuously broadcast transform
        ts.header.stamp = ros::Time::now();
        ts.header.frame_id = "world";
        ts.child_frame_id = "red-base_footprint";
        ts.transform.translation.x = T.translation().x;
        ts.transform.translation.y = T.translation().y;
        ts.transform.translation.z = 0;
        tf2::Quaternion ang;
        ang.setRPY(0,0,T.rotation());
        ts.transform.rotation.x = ang.x();
        ts.transform.rotation.y = ang.y();
        ts.transform.rotation.z = ang.z();;
        ts.transform.rotation.w = ang.w();
        b.sendTransform(ts);




        std_msgs::UInt64 num;
        num.data = counter;
        
        //Publish joint states and timer
        count_pub.publish(num);
        counter++;

        geometry_msgs::Quaternion orientation_quaternion = tf::createQuaternionMsgFromYaw(T.rotation()); //source (02/01): http://wiki.ros.org/navigation/Tutorials/RobotSetup/Odom


        pose_stamp.header.stamp = ros::Time::now();
        pose_stamp.pose.position.x = T.translation().x;
        pose_stamp.pose.position.y = T.translation().y;
        pose_stamp.pose.position.z = 0.0;

        pose_stamp.pose.orientation = orientation_quaternion;

        
        if(trail_count % 100 == 0){
            trail.header.stamp = ros::Time::now();
            trail.poses.push_back(pose_stamp);
            trail_pub_red.publish(trail);

        }
        trail_count++;



        if(use_sim){
            //Publish laser scan
            laser_sim.header.stamp = ros::Time::now();

            //Compute Lidar Data and avoid obstacles
            for(i=0;i<o_x.size();i++){
                
                m_array_sense.markers[i].header.stamp = ros::Time::now();

                std::normal_distribution<> pos_r(0.0, basic_sensor_variance);
                std::normal_distribution<> pos_phi(0.0, basic_sensor_variance);

                double rand_pos_r = pos_r(get_random());
                double rand_pos_phi = pos_phi(get_random());

                turtlelib::Transform2D T_inv;
                T_inv = T.inv();

                turtlelib::Vector2D v_world;
                v_world.x = o_x[i];
                v_world.y = o_y[i];

                turtlelib::Vector2D v_robot;//Transform to robot frame
                v_robot = T_inv(v_world);

                double r_sense = sqrt(v_robot.x*v_robot.x + v_robot.y*v_robot.y);
                double phi_sense = atan2(v_robot.y,v_robot.x);

                r_sense += rand_pos_r;
                phi_sense += rand_pos_phi;

                //convert to polar add noise, convert back to x y
                m_array_sense.markers[i].pose.position.x = r_sense*cos(phi_sense);
                m_array_sense.markers[i].pose.position.y =  r_sense*sin(phi_sense);


                if(sqrt((T.translation().x - o_x[i])*(T.translation().x - o_x[i])+(T.translation().y - o_y[i])*(T.translation().y - o_y[i])) > max_range){
                    m_array_sense.markers[i].action = visualization_msgs::Marker::DELETE;
                }

                if(sqrt((T.translation().x - o_x[i])*(T.translation().x - o_x[i])+(T.translation().y - o_y[i])*(T.translation().y - o_y[i])) < max_range){
                    m_array_sense.markers[i].action = visualization_msgs::Marker::ADD;
                }

                collision_avoid(o_x.at(i),o_y.at(i),o_r.at(i), collision_radius);
            }

            //Publish at 5 hz
            // if(counter%period == 0){
            //     lidar_pub.publish(m_array_sense);
            //     intersect(resolution, max_range);
            //     sensor_pub.publish(laser_sim);
            // }

            if(pubflag == true){
                lidar_pub.publish(m_array_sense);
                intersect(resolution, max_range);
                sensor_pub.publish(laser_sim);
                pubflag = false;
            }

            nuturtlebot_msgs::SensorData sense;       
            //Normalize angle
            if(pos_wheel[0]<turtlelib::PI && pos_wheel[0]>0){
                sense.left_encoder = (int) (pos_wheel[0]/encoder_ticks_to_rad);
            }

            if(pos_wheel[0]>turtlelib::PI){
                sense.left_encoder = (int) (turtlelib::normalize_angle(pos_wheel[0])/encoder_ticks_to_rad);
                if(sense.left_encoder<0){
                    sense.left_encoder+=4096;
                }
            }

            if(pos_wheel[0]<0 && pos_wheel[0]>-2*turtlelib::PI){
                sense.left_encoder = (int) (pos_wheel[0]/encoder_ticks_to_rad)+4096;
            }

            if(pos_wheel[0]<-2*turtlelib::PI){
                sense.left_encoder = (int) (turtlelib::normalize_angle(pos_wheel[0])/encoder_ticks_to_rad);
                if(sense.left_encoder<0){
                    sense.left_encoder+=4096;
                }
            }

            if(pos_wheel[1]<turtlelib::PI && pos_wheel[1]>0){
                sense.right_encoder = (int) (pos_wheel[1]/encoder_ticks_to_rad);
            }

            if(pos_wheel[1]>turtlelib::PI){
                sense.right_encoder = (int) (turtlelib::normalize_angle(pos_wheel[1])/encoder_ticks_to_rad);
                if(sense.right_encoder<0){
                    sense.right_encoder+=4096;
                }
            }

            if(pos_wheel[1]<0 && pos_wheel[1]>-2*turtlelib::PI){
                sense.right_encoder = (int) (pos_wheel[1]/encoder_ticks_to_rad)+4096;
            }

            if(pos_wheel[1]<-2*turtlelib::PI){
                sense.right_encoder = (int) (turtlelib::normalize_angle(pos_wheel[1])/encoder_ticks_to_rad);
                if(sense.right_encoder<0){
                    sense.right_encoder+=4096;
                }
            }

            sense.stamp = ros::Time::now();
            sensor_data_pub.publish(sense);
        }  
        ros::spinOnce();
        rate.sleep();

    }
    
    return 0;
}