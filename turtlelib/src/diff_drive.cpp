#include <iostream>
#include "turtlelib/diff_drive.hpp"
#include "turtlelib/rigid2d.hpp"
#include <cmath>


using namespace std;

/// \file
/// \brief Implementation of a model for a differential drive robot


namespace turtlelib
{


    Diff_Drive::Diff_Drive(double width, double radius, double x, double y, double theta){
        
        // Declare physical parameters
        w = width;
        r = radius;
        
        //Declare initial position in world frame
        x0 = x;
        y0 = y;
        theta0 = theta;

        //Declare wheel position
        phi_l = 0.0;
        phi_r = 0.0;

        //Current position in world frame
        x_current = x0;
        y_current = y0;
        theta_current = theta0;
        

        //Configuration, initial and current
        q0 = {theta0, x0, y0};
        q = q0;

        //Create world to body transform
        Vector2D V_init;

        V_init.x = x0;
        V_init.y = y0;

        T_wb = Transform2D(V_init, theta0);
        
    }

        Diff_Drive::Diff_Drive(double width, double radius, double x, double y, double theta, double left, double right){
        
        // Declare physical parameters
        w = width;
        r = radius;
        
        //Declare initial position in world frame
        x0 = x;
        y0 = y;
        theta0 = theta;

        //Declare wheel position
        phi_l = left;
        phi_r = right;

        //Current position in world frame
        x_current = x0;
        y_current = y0;
        theta_current = theta0;
        

        //Configuration, initial and current
        q0 = {theta0, x0, y0};
        q = q0;

        //Create world to body transform
        Vector2D V_init;

        V_init.x = x0;
        V_init.y = y0;

        T_wb = Transform2D(V_init, theta0);
        
    }

  

    

    Transform2D Diff_Drive::fkin(double tl, double tr){
        

        double dtheta_l = tl - phi_l;
        double dtheta_r = tr - phi_r; 



        phi_l = tl;
        phi_r = tr; 
          
        //Avoid artifically large angles  
        // if((abs(dtheta_l) < turtlelib::PI) && ( abs(dtheta_r) < turtlelib::PI)){
        //     Twist2D V;
        //     Transform2D Tbbp;
        //     V.tw[0] = -(r/(2*w))*dtheta_l + (r/(2*w))*dtheta_r; //Integrate a constant twist, to create a new transform, per equation 10.a of docs/Kinematics.pdf
        //     V.tw[1] = r*dtheta_l/2 + r*dtheta_r/2;
        //     V.tw[2] = 0.0;

        //     Tbbp = integrate_twist(V);
        //     T_old = Tbbp;

        //     T_wb*=Tbbp;        //Update transform


        // }else{
        //     T_wb*=T_old;         //Update transform

        // } 

        dtheta_l = turtlelib::normalize_angle (dtheta_l);
        dtheta_r = turtlelib::normalize_angle(dtheta_r);

        Twist2D V;
        Transform2D Tbbp;
        V.tw[0] = -(r/(2*w))*dtheta_l + (r/(2*w))*dtheta_r; //Integrate a constant twist, to create a new transform, per equation 10.a of docs/Kinematics.pdf
        V.tw[1] = r*dtheta_l/2 + r*dtheta_r/2;
        V.tw[2] = 0.0;

        Tbbp = integrate_twist(V);
        T_old = Tbbp;

        T_wb*=Tbbp;        //Update transform


        //Update configuration
        q[0] = T_wb.rotation();
        q[1] = T_wb.translation().x;
        q[2] = T_wb.translation().y;

        x_current = T_wb.rotation();
        y_current = T_wb.translation().x;
        theta_current = T_wb.translation().y;

        return T_wb;

    }

    vector<double> Diff_Drive::inv_kin(Twist2D twist){
          
        vector<double> v_wheel = {0.0,0.0};  //[w_left, w_right]

        if(almost_equal(0.0, twist.tw[2],1.0e-6)!=true){
            throw logic_error("Cannot have non-zero y velocity!\n"); //source(01/29): https://riptutorial.com/cplusplus/example/9212/best-practice--throw-by-value--catch-by-const-reference https://en.cppreference.com/w/cpp/error/logic_error
        }  

        v_wheel[0] = (-w*twist.tw[0]+twist.tw[1])/r; //According to our inverse kinematics equations, our left (0) and right (1) wheel velocities are as follows
        v_wheel[1] = (+w*twist.tw[0]+twist.tw[1])/r; //Per equations 14 and 15 of inverse kinematics  equations (per docs/Kinematics.pdf)
        
        cout<<"vl:"<<v_wheel[0]<<" "<<"vr: " << v_wheel[1]; 
    
        cout<<"\n";


        return v_wheel;
    }  








}