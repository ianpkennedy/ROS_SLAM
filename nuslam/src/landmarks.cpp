#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "nuslam/Unknown.h"
#include "lidar/lidarml.hpp"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include <nav_msgs/Odometry.h>
#include "geometry_msgs/Quaternion.h"
#include "tf/LinearMath/Matrix3x3.h"
#include "tf/transform_datatypes.h"
#include <cmath>
#include "turtlelib/rigid2d.hpp"


/// \file
/// \brief Apply survised and sunsupervised learning to generate landmark XY positions
///
/// PARAMETERS:
///
/// PUBLISHES:
/// /cluster (visualization::msgs MarkerArray): location of clusters
/// /mystery (nuslam::Unknown): x,y,radius values of the clusters
/// 
/// SUBSCRIBES:
/// /nusim/laser_sim (sensor::msgs LaserScan): simuated laser data that can be remapped for real turtlebot
using std::vector;

namespace{
    lidar::Lidar2D l(0,0.1);
    vector<vector<double>> x;
    vector<vector<double>> y;
    vector<vector<double>> params;
    vector<bool> is_landmark;
    nuslam::Unknown unknown;



}


/// \brief lidar scan callback
/// \param msg - LiDAR x,y,radius
/// \returns None
void lidarptsCallback(const sensor_msgs::LaserScan& msg){
    l.cluster(msg.ranges); //cluster data
    x = l.return_x_cluster();
    y = l.return_y_cluster();    
    is_landmark = l.classify(x,y); //check if is circle
    params = l.circle(x,y); //extract parameters

    for(long unsigned int i=0;i<params.size();i++){
        
        if(is_landmark.at(i)){
    
            if(turtlelib::almost_equal(params.at(i).at(2), 0.04,0.04 ) ) {
                unknown.x.push_back(params.at(i).at(0));
                unknown.y.push_back(params.at(i).at(1));
                unknown.r.push_back(params.at(i).at(2));

            }

        }    
        
    }


}


int main(int argc, char * argv[]){

    ros::init(argc,argv,"landmarks");
    ros::NodeHandle n;
    
    //publishing object
    visualization_msgs::Marker m_sense;
    visualization_msgs::MarkerArray m_array_sense;

    ros::Subscriber lidarpts = n.subscribe("/nusim/laser_sim", 5, lidarptsCallback);
    
    ros::Publisher cluster_pub;
    cluster_pub = n.advertise<visualization_msgs::MarkerArray>("/cluster", 100, true);
    
    ros::Publisher data_pub = n.advertise<nuslam::Unknown>("/mystery",100);

    visualization_msgs::Marker cluster;
    visualization_msgs::MarkerArray cluster_array;


    ros::Rate rate(5);

    l = lidar::Lidar2D(360,0.5);

    
    while(ros::ok()){


        // l.print_cluster();
        //Publish cluster markers
        for(long unsigned int i=0;i<x.size();i++){

            cluster_array.markers.clear();

            for(long unsigned int j=0;j<x.at(i).size();j++){
                
                cluster.header.frame_id = "blue-base_scan"; //red-base_footprint
                // cluster.header.stamp = ros::Time::now();
                cluster.ns = "cluster";
                cluster.action = visualization_msgs::Marker::ADD;
                cluster.id = j;
                cluster.type = visualization_msgs::Marker::SPHERE;

                cluster.scale.x = 0.1;
                cluster.scale.y = 0.1;
                cluster.scale.z = 0.1; 

                cluster.color.r = 0.0;
                cluster.color.g = 1.0;
                cluster.color.b = 0.0;
                cluster.color.a = 1.0;

                cluster.pose.orientation.x = 0.0;
                cluster.pose.orientation.y = 0.0;
                cluster.pose.orientation.z = 0.0;
                cluster.pose.orientation.w = 1.0;

                cluster.pose.position.x = x.at(i).at(j);
                cluster.pose.position.y = y.at(i).at(j);
                cluster.pose.position.z = 0.0;

                cluster.frame_locked = true;
                ros::Duration cluster_life(5.0); //was 0.2 or 0.1

                cluster.lifetime = cluster_life;
                cluster_array.markers.push_back(cluster);


            }

            cluster_pub.publish(cluster_array);

            

        }


        //Clear all vectors
        l.clear_cluster();
        x.clear();
        y.clear();
        params.clear();

        unknown.x.clear();
        unknown.y.clear();
        unknown.r.clear();

        ros::spinOnce();

        data_pub.publish(unknown);


        rate.sleep(); 
    }


    return 0;
}