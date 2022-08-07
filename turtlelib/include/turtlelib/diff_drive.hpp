#ifndef DIFF_DRIVE_INCLUDE_GUARD_HPP
#define DIFF_DRIVE_INCLUDE_GUARD_HPP

#include<vector>
#include "turtlelib/rigid2d.hpp" //source (01/28): https://stackoverflow.com/questions/3608305/class-name-does-not-name-a-type-in-c/43848478

/// \file
/// \brief Differential drive model for robot

namespace turtlelib
{

    /// \brief a rigid body differential drive robot model
    class Diff_Drive
    {
    
       public:


       /// \brief Create the differential drive robot model with parameters
       Diff_Drive(double width, double radius, double x0, double y0, double theta0);

              /// \brief Create the differential drive robot model with parameters and previous wheel
       Diff_Drive(double width, double radius, double x0, double y0, double theta0, double left, double right);
       

     
        /// \brief update the position of the robot in the world frame
        /// \param tl- left wheel angle
        /// \param tr - right wheel angle
        /// \return Transform2D object (Tbbprime)
       Transform2D fkin(double tl, double tr);

        /// \brief calculate wheel velocities
        /// \param twist- twist in the body frame of robot
        /// \return angular velocity vector: {w_left, w_right}
       std::vector<double> inv_kin(Twist2D twist);


        /// \brief update the position of the robot in the world frame
        /// \param twist- twist in the body frame of robot
        /// \return angular velocity vector: {w_left, w_right}
       std::vector<double> update(Twist2D twist);


       private:

       std::vector<double> q;  //Current pose
       double x_current;
       double y_current;
       double theta_current;

       double phi_l;
       double phi_r;


          double x0;
          double y0;
          double theta0;
          std::vector<double> q0; //Initial pose
          double w;
          double r;
          turtlelib::Transform2D T_wb;
          turtlelib::Transform2D T_old;

    };

}



#endif
