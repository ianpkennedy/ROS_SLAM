#include <vector>
#include <cmath>
#include "lidar/lidarml.hpp"
#include <catch_ros/catch.hpp>

using std::vector;

TEST_CASE("First circle test", "circle regression"){
    //Start the robot at 0,0 and theta = 0


    lidar::Lidar2D l(10,2.0);
    vector<vector<double>> x;
    x.push_back({1.0,2.0,5.0,7.0,9.0,3.0});
    
    vector<vector<double>> y;
    y.push_back({7.0,6.0,8.0,7.0,5.0,7.0});
  
    vector<vector<double>> results = l.circle(x,y);


    REQUIRE(results.at(0).at(0)==Approx(4.615482).margin(.0001));
    REQUIRE(results.at(0).at(1)==Approx(2.807354).margin(.0001));
    REQUIRE(results.at(0).at(2)==Approx(4.8275).margin(.0001));



}

TEST_CASE("Second circle test", "circle regression"){
    //Start the robot at 0,0 and theta = 0


    lidar::Lidar2D l(10,2.0);
    vector<vector<double>> x;
    x.push_back({-1.0,-0.3,0.3,1.0});
    
    vector<vector<double>> y;
    y.push_back({0.0,-0.06,0.1,0.0});
  
    vector<vector<double>> results = l.circle(x,y);


    REQUIRE(results.at(0).at(0)==Approx(0.4908357).margin(.0001));
    REQUIRE(results.at(0).at(1)==Approx(-22.15212).margin(.0001));
    REQUIRE(results.at(0).at(2)==Approx(22.17979).margin(.0001));



}
TEST_CASE("Third circle test", "circle regression"){
    //Start the robot at 0,0 and theta = 0


    lidar::Lidar2D l(10,2.0);
    vector<vector<double>> x;
    x.push_back({0.0,1.0,0.0,-1.0,-1.0,-1.0,0.0});
    
    vector<vector<double>> y;
    y.push_back({1.0,0.0,-1.0,0.0,0.0,0.0,-1.0});
  
    vector<vector<double>> results = l.circle(x,y);


    REQUIRE(results.at(0).at(0)==Approx(0.0).margin(.0001));
    REQUIRE(results.at(0).at(1)==Approx(0.0).margin(.0001));
    REQUIRE(results.at(0).at(2)==Approx(1.0).margin(.0001));



}