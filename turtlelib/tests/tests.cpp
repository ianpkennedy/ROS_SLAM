/// \file
/// \brief Testing file for Transform2D class


#include<sstream>
#include<string>
#include<iostream>
#include "turtlelib/rigid2d.hpp"
#include "turtlelib/diff_drive.hpp"
#include <catch_ros/catch.hpp>


using namespace std;

/// \brief test the identity constructor
TEST_CASE("identity constructor","[transform]"){
    //Declare variables
    turtlelib::Transform2D eye;
    double rot = eye.rotation();
    turtlelib::Vector2D tr = eye.translation();

    //Compare
    REQUIRE(0==Approx(rot).margin(.01));
    REQUIRE(0==Approx(tr.x).margin(.01));
    REQUIRE(0==Approx(tr.y).margin(.01));

}

/// \brief test the rotation constructor
TEST_CASE("rotation constructor","[transform]"){
    //Declare variables
    turtlelib::Transform2D rotate(turtlelib::PI);
    double rot = rotate.rotation();
    turtlelib::Vector2D tr = rotate.translation();


    //Compare results
    REQUIRE(turtlelib::PI==Approx(rot).margin(.01));
    REQUIRE(0==Approx(tr.x).margin(.01));
    REQUIRE(0==Approx(tr.y).margin(.01));

}

/// \brief test the translation constructor
TEST_CASE("translation constructor","[transform]"){
    //Declare variables
    turtlelib::Vector2D pos;
    pos.x = 1.5;
    pos.y = 2.5;

    //Extract translation and rotation parameters
    turtlelib::Transform2D trans(pos);
    double rot = trans.rotation();
    turtlelib::Vector2D tr = trans.translation();

    //Compare
    REQUIRE(0==Approx(rot).margin(.01));
    REQUIRE(pos.x==Approx(tr.x).margin(.01));
    REQUIRE(pos.y==Approx(tr.y).margin(.01));

}


/// \brief test the translation and rotation constructor
TEST_CASE("translation and rotation constructor","[transform]"){
    
    //Declare initial variables
    turtlelib::Vector2D pos;
    pos.x = 5.5;
    pos.y = 7.5;
    double ang = 1.2;


    //Extract rotation and translation parameters
    turtlelib::Transform2D trans(pos, ang);
    double rot = trans.rotation();
    turtlelib::Vector2D tr = trans.translation();


    //Compare
    REQUIRE(ang==Approx(rot).margin(.01));
    REQUIRE(pos.x==Approx(tr.x).margin(.01));
    REQUIRE(pos.y==Approx(tr.y).margin(.01));

}


/// \brief test the operator() on a Vector2D object
TEST_CASE("operator() Vector2D","[transform]"){

    //Declare initial Vector2D variables
    turtlelib::Vector2D pos;
    pos.x = 5.5;
    pos.y = 7.5;
    turtlelib::Vector2D translation;
    translation.x = 1.0;
    translation.y = 2.0;

    //Apply transformations
    turtlelib::Transform2D t_transform(translation);
    turtlelib::Vector2D tr = t_transform(pos);

    //Generate resulting truth
    turtlelib::Vector2D result;
    result.x = pos.x+translation.x;
    result.y = pos.y+translation.y;

    //Compare
    REQUIRE(result.x==Approx(tr.x).margin(.01));
    REQUIRE(result.y==Approx(tr.y).margin(.01));

}


/// \brief test the operator() on a Twist2D object
TEST_CASE("operator() Twist2D","[transform]"){
    //Declare a Twist2D variable
    turtlelib::Twist2D twist;
    twist.tw[0] = 10;
    twist.tw[1] = 1;
    twist.tw[2] = 0;


    //Create transform and new twist
    turtlelib::Transform2D t_transform((turtlelib::PI)/2);
    turtlelib::Twist2D twist_new = t_transform(twist);

    //Create resultant Twist2D vector
    turtlelib::Twist2D result;
    result.tw[0] = 10;
    result.tw[1] = 0;
    result.tw[2] = 1;

    //Compare calculated vector to desired truth
    REQUIRE(result.tw[0]==Approx(twist_new.tw[0]).margin(.01));
    REQUIRE(result.tw[1]==Approx(twist_new.tw[1]).margin(.01));
    REQUIRE(result.tw[2]==Approx(twist_new.tw[2]).margin(.01));

}


/// \brief test the operator*= on a Transform2D object
TEST_CASE("operator*=","[transform]"){
   //Create two starting vectors
    turtlelib::Vector2D v1;
    v1.x=1.0;
    v1.y=1.0;

    turtlelib::Vector2D v2;
    v2.x=1.4;
    v2.y=2.5;    

    //Create two transforms
    turtlelib::Transform2D t_transform1(v1);
    turtlelib::Transform2D t_transform2(v2);

    //Perform calculation
    t_transform2*=t_transform1;

    //Compare to expected values
    REQUIRE(0==Approx(t_transform2.rotation()).margin(.01));
    REQUIRE(2.4==Approx(t_transform2.translation().x).margin(.01));
    REQUIRE(3.5==Approx(t_transform2.translation().y).margin(.01));

}


/// \brief testing the inv() method
TEST_CASE("inv()","[transform]"){
    //Create initial variables
    turtlelib::Vector2D v1;
    v1.x=1.2;
    v1.y=10.2;   

    //Create transform and inverse
    turtlelib::Transform2D t_transform1(v1);
    turtlelib::Transform2D t_transform2 = t_transform1.inv();

   //Compare the resultant transforms
    REQUIRE(-t_transform1.rotation()==Approx(t_transform2.rotation()).margin(.01));
    REQUIRE(-t_transform1.translation().x==Approx(t_transform2.translation().x).margin(.01));
    REQUIRE(-t_transform1.translation().y==Approx(t_transform2.translation().y).margin(.01));

}

/// \brief test the translation() method
TEST_CASE("translation()","[transform]"){
    
    //Create initial vector
    turtlelib::Vector2D v1;
    v1.x=1.2;
    v1.y=10.2;   

    //Create vector
    turtlelib::Transform2D t_transform1(v1);

    //Exctract and compare translation information
    REQUIRE(1.2==Approx(t_transform1.translation().x).margin(.01));
    REQUIRE(10.2==Approx(t_transform1.translation().y).margin(.01));

}

/// \brief testing the rotation() method
TEST_CASE("rotation()","[transform]"){

    //Create transformation matrix
    double angle = 1.57;
    turtlelib::Transform2D t_transform1(angle);


    //Compare angle to input
    REQUIRE(angle==Approx(t_transform1.rotation()).margin(.01));

}

/// \brief testing the operator<< overload
TEST_CASE("operator<<","[transform]"){

    //Create input variables and string
    stringstream output;
    turtlelib::Vector2D v1;
    v1.x=1.2;
    v1.y=10.2;   
    turtlelib::Transform2D t_transform1(v1);
    

    //Read in transform information to string
    output << t_transform1; // (01/14)): https://stackoverflow.com/questions/5193173/getting-cout-output-to-a-stdstring

    //Compare output string to expected result
    REQUIRE("deg: 0 x: 1.2 y: 10.2\n"==output.str());

}

/// \brief testing the operator>> overload
TEST_CASE("operator>>","[transform]"){

    //feed input string into transform
    stringstream input;
    input.str("deg: 45 x: 9.4 y: 4.5\n");
    turtlelib::Transform2D t_transform1;
    input >> t_transform1;

    //Extract transformation parameters
    double rotation = t_transform1.rotation();
    turtlelib::Vector2D v = t_transform1.translation();

    //Compare resultant transform to expected inputs
    REQUIRE(rotation==Approx((turtlelib::PI)/4).margin(.01));
    REQUIRE(v.x==Approx(9.4).margin(.01));
    REQUIRE(v.y==Approx(4.5).margin(.01));

}

/// \brief testing the operator* overload
TEST_CASE("operator*","[transform]"){

    //Declare initial vectors
    turtlelib::Vector2D v1;
    v1.x=1.0;
    v1.y=1.0;
    turtlelib::Vector2D v2;
    v2.x=1.4;
    v2.y=2.5;    

    //Create transforms
    turtlelib::Transform2D t_transform1(v1);
    turtlelib::Transform2D t_transform2(v2);

    t_transform2 = t_transform2*t_transform1;

    //Compare transform parameters to expected output
    REQUIRE(0==Approx(t_transform2.rotation()).margin(.01));
    REQUIRE(2.4==Approx(t_transform2.translation().x).margin(.01));
    REQUIRE(3.5==Approx(t_transform2.translation().y).margin(.01));

}
/// \brief test normalize angle
TEST_CASE("normalize angle","[double]"){


    //Check each case
    REQUIRE(turtlelib::PI==Approx(turtlelib::normalize_angle(turtlelib::PI)).margin(.01));
    REQUIRE(-turtlelib::PI==Approx(turtlelib::normalize_angle(-turtlelib::PI)).margin(.01));

    REQUIRE(0.0==Approx(turtlelib::normalize_angle(0.0)).margin(.01));
    REQUIRE((-0.25)*turtlelib::PI==Approx(turtlelib::normalize_angle(-(0.25)*turtlelib::PI)).margin(.01));
    REQUIRE((-0.5)*turtlelib::PI==Approx(turtlelib::normalize_angle((1.5)*turtlelib::PI)).margin(.01));
    REQUIRE((0.5)*turtlelib::PI==Approx(turtlelib::normalize_angle((2.5)*turtlelib::PI)).margin(.01));
    REQUIRE((-0.1)*turtlelib::PI==Approx(turtlelib::normalize_angle((1.9)*turtlelib::PI)).margin(.01));
    
    REQUIRE(0.99*turtlelib::PI==Approx(turtlelib::normalize_angle((2.99)*turtlelib::PI)).margin(.01));

    REQUIRE(-0.99*turtlelib::PI==Approx(turtlelib::normalize_angle((-2.99)*turtlelib::PI)).margin(.01));

    REQUIRE(-0.99*turtlelib::PI==Approx(turtlelib::normalize_angle((-2.99)*turtlelib::PI)).margin(.01));
    REQUIRE(-0.083==Approx(turtlelib::normalize_angle(3.1 - (-3.1))).margin(.01));

}    



/// \brief test integrate twist
TEST_CASE("integrate twist","[transform2d]"){
    turtlelib::Twist2D V1;
    turtlelib::Twist2D V2;
    turtlelib::Twist2D V3;

    turtlelib::Transform2D T1;
    turtlelib::Transform2D T2;
    turtlelib::Transform2D T3;

    V1.tw[0] = 0.0;
    V1.tw[1] = 1.0;
    V1.tw[2] = 0.0;

    V2.tw[0] = 1.0;
    V2.tw[1] = 0.0;
    V2.tw[2] = 0.0;

    V3.tw[0] = 1.0;
    V3.tw[1] = 1.0;
    V3.tw[2] = 0.0;

    T1 = integrate_twist(V1);
    T2 = integrate_twist(V2);
    T3 = integrate_twist(V3);

    //pure translation
    REQUIRE(1.0==Approx(integrate_twist(V1).translation().x).margin(.01));
    REQUIRE(0.0==Approx(integrate_twist(V1).translation().y).margin(.01));
    REQUIRE(0.0==Approx(integrate_twist(V1).rotation()).margin(.01));


    //pure rotation
    REQUIRE(1.0==Approx(integrate_twist(V2).rotation()).margin(.01));
    REQUIRE(0.0==Approx(integrate_twist(V2).translation().x).margin(.01));
    REQUIRE(0.0==Approx(integrate_twist(V2).translation().y).margin(.01));

    //Simultaneous translation and rotation
    REQUIRE(0.84 == Approx(integrate_twist(V3).translation().x).margin(.01));
    REQUIRE(0.46==Approx(integrate_twist(V3).translation().y).margin(.01));
    REQUIRE(1.0==Approx(integrate_twist(V3).rotation()).margin(.01));


}

/// \brief test forward translation
TEST_CASE("diff_drive forward translation","[diff_drive]"){
    turtlelib::Transform2D t_test1;
    turtlelib::Diff_Drive diff1(0.0,0.0,0.0,0.0,0.0);
    turtlelib::Twist2D twist;
    twist.tw[0] = 0.0;
    twist.tw[1] = 1.0;
    twist.tw[2] = 0.0;
    diff1 = turtlelib::Diff_Drive(1.0,1.0,0.0,0.0,0.0);
    vector<double> v_wheel = {0.0,0.0};

    t_test1 = diff1.fkin(1.0,1.0);
    v_wheel = diff1.inv_kin(twist);

    //Check forward kinematics
    REQUIRE(1.0==Approx(t_test1.translation().x).margin(.01));
    REQUIRE(0.0==Approx(t_test1.translation().y).margin(.01));
    REQUIRE(0.0==Approx(t_test1.rotation()).margin(.01));

    //Check inverse kinematics
    REQUIRE(1.0==Approx(v_wheel[0]).margin(.01));
    REQUIRE(1.0==Approx(v_wheel[1]).margin(.01));

    //More forward kinematics testing
    t_test1 = diff1.fkin(2.0,2.0);
    REQUIRE(2.0==Approx(t_test1.translation().x).margin(.01));
    REQUIRE(0.0==Approx(t_test1.translation().y).margin(.01));
    REQUIRE(0.0==Approx(t_test1.rotation()).margin(.01));


}

/// \brief test reverse translation
TEST_CASE("diff_drive reverse translation","[diff_drive]"){
   //Initiate variables
    turtlelib::Transform2D t_test1;
    turtlelib::Diff_Drive diff1(0.0,0.0,0.0,0.0,0.0);
    diff1 = turtlelib::Diff_Drive(1.0,1.0,0.0,0.0,0.0);
    vector<double> v_wheel = {0.0,0.0};
    turtlelib::Twist2D twist;
    twist.tw[0] = 0.0;
    twist.tw[1] = -1.0;
    twist.tw[2] = 0.0;

    //Calcualte results
    t_test1 = diff1.fkin(-1.0,-1.0);
    v_wheel = diff1.inv_kin(twist);

    //Check forward kinematics
    REQUIRE(-1.0==Approx(t_test1.translation().x).margin(.01));
    REQUIRE(0.0==Approx(t_test1.translation().y).margin(.01));
    REQUIRE(0.0==Approx(t_test1.rotation()).margin(.01));

    //Check inverse kinematics
    REQUIRE(-1.0==Approx(v_wheel[0]).margin(.01));
    REQUIRE(-1.0==Approx(v_wheel[1]).margin(.01));

}


/// \brief test forward rotation
TEST_CASE("diff_drive forward rotation","[diff_drive]"){
    
    //Declare variablse
    turtlelib::Transform2D t_test1;
    turtlelib::Diff_Drive diff1(0.0,0.0,0.0,0.0,0.0);
    diff1 = turtlelib::Diff_Drive(1.0,1.0,0.0,0.0,0.0);
    vector<double> v_wheel = {0.0,0.0};
    turtlelib::Twist2D twist;
    twist.tw[0] = 1.0;
    twist.tw[1] = 0.0;
    twist.tw[2] = 0.0;

    //Calculate results
    v_wheel = diff1.inv_kin(twist);
    t_test1 = diff1.fkin(-turtlelib::PI/2,turtlelib::PI/2);

    //Check forward kinematics
    REQUIRE(0.0==Approx(t_test1.translation().x).margin(.01));
    REQUIRE(0.0==Approx(t_test1.translation().y).margin(.01));
    REQUIRE(turtlelib::PI/2==Approx(t_test1.rotation()).margin(.01));

    //Check inverse kinematics
    REQUIRE(-1.0==Approx(v_wheel[0]).margin(.01));
    REQUIRE(1.0==Approx(v_wheel[1]).margin(.01));

}

/// \brief test reverse rotation
TEST_CASE("diff_drive reverse rotation","[diff_drive]"){
    
    //Declare variablse
    turtlelib::Transform2D t_test1;
    turtlelib::Diff_Drive diff1(0.0,0.0,0.0,0.0,0.0);
    diff1 = turtlelib::Diff_Drive(1.0,1.0,0.0,0.0,0.0);
    vector<double> v_wheel = {0.0,0.0};
    turtlelib::Twist2D twist;
    twist.tw[0] = -1.0;
    twist.tw[1] = 0.0;
    twist.tw[2] = 0.0;

    //Calculate results
    v_wheel = diff1.inv_kin(twist);
    t_test1 = diff1.fkin(turtlelib::PI/2,-turtlelib::PI/2);

    //Check forward kinematics
    REQUIRE(0.0==Approx(t_test1.translation().x).margin(.01));
    REQUIRE(0.0==Approx(t_test1.translation().y).margin(.01));
    REQUIRE(-turtlelib::PI/2==Approx(t_test1.rotation()).margin(.01));

    //Check inverse kinematics
    REQUIRE(1.0==Approx(v_wheel[0]).margin(.01));
    REQUIRE(-1.0==Approx(v_wheel[1]).margin(.01));

}


/// \brief test forward kinematics, arc circle
TEST_CASE("diff_drive arc circle forward kinematics","[diff_drive]"){

    //Check forward kinematics

    //Declare variables
    turtlelib::Transform2D t_test1;
    double r = 2.0;
    double w = 1.0;
    double Radius = 1.0;

    turtlelib::Diff_Drive diff1(0.0,0.0,0.0,0.0,0.0);
    diff1 = turtlelib::Diff_Drive(r,w,0.0,0.0,0.0);
    vector<double> v_wheel = {0.0,0.0};
    turtlelib::Twist2D twist;
    twist.tw[0] = 1.0/(w);
    twist.tw[1] = 1.0;
    twist.tw[2] = 0.0;

    //Calculate results
    t_test1 = diff1.fkin(1.0*(turtlelib::PI/3.0)*(Radius - w)/r,1.0*(turtlelib::PI/3.0)*(Radius + w)/r); //Source (01/31): https://www.societyofrobots.com/robotforum/index.php?topic=13052.0


    //Check results
    REQUIRE(0.52==Approx(t_test1.translation().x).margin(.01));
    REQUIRE(0.07==Approx(t_test1.translation().y).margin(.01));
    REQUIRE(0.26==Approx(t_test1.rotation()).margin(.01));


}

/// \brief test inverse kinematics, arc circle
TEST_CASE("diff_drive arc circle inverse kinematics","[diff_drive]"){

    //Check forward functionality
    
    //Declare variables
    double r = 1.0;
    double w = 1.0;
    turtlelib::Diff_Drive diff1(0.0,0.0,0.0,0.0,0.0);
    diff1 = turtlelib::Diff_Drive(r,w,0.0,0.0,0.0);
    vector<double> v_wheel = {0.0,0.0};
    turtlelib::Twist2D twist;
    twist.tw[0] = 1.0/(w);
    twist.tw[1] = 1.0;
    twist.tw[2] = 0.0;


    //Calculate
    v_wheel = diff1.inv_kin(twist);


    //Check
    REQUIRE(0.0==Approx(v_wheel[0]).margin(.01));
    REQUIRE(2.0==Approx(v_wheel[1]).margin(.01));


}


/// \brief check bad twist
TEST_CASE("diff_drive bad twist","[diff_drive]"){
  
    turtlelib::Diff_Drive diff1(0.0,0.0,0.0,0.0,0.0);
    diff1 = turtlelib::Diff_Drive(1.0,1.0,0.0,0.0,0.0);
    vector<double> v_wheel = {0.0,0.0};
    std::string s = "Cannot have non-zero y velocity!\n";
    turtlelib::Twist2D twist;
    twist.tw[0] = 0.0;
    twist.tw[1] = 0.0;
    twist.tw[2] = 1.0;



    //Check if the member function is thrown properly
    try{
        diff1.inv_kin(twist);
   }catch(const std::logic_error& e){
       cout<<e.what()<<"\n";
       REQUIRE(s==e.what());

   }

   CHECK_THROWS(diff1.inv_kin(twist));

}