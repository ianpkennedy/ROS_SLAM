#include <armadillo>
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
#include "nuslam/SetPoseSlam.h"
#include "geometry_msgs/TransformStamped.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "nuslam/Unknown.h"


/// \file
/// \brief This node runs SLAM computations
///
/// PARAMETERS:
///     rate (integer): publishing rate in Hz
///     wheel_radius (float): radius of turtlebot wheel in meters
///     track_width (float): wheel distance of turtlebot in meters
///     encoder_ticks_to_rad (float): conversion factor from encoder ticks to radians
///     motor_cmd_to_radsec (float): conversion factor from motor command units to rad/sec
///     body_id (string): body frame name
///     odom_id (string): odom frame name
///     wheel_left(string): left wheel name
///     wheel_right (string): right wheel name
///     use_known (bool): whether to use known data association
///     use_unknown (bool): whether to use unknown
///     
///     
/// PUBLISHES:
///     odom_slam (nav_msgs::Odometry): robot odometry in SLAM reference frame
///     pathodom_slam (nav_msgs::Path): robot SLAM path
///     map_obstacles (visualization_msgs::MarkerArray): map obstacles detected by robot
///     
/// SUBSCRIBES:
///     joint_states (sensor_msgs::JointState): joint state publisher
///     fake_sensor (visualization_msgs::MarkerArray): fake sensor data (in marker form)
///     mystery (nuslam::Unknown): extracted positions of circles using for unknown data association
/// SERVICES:
///     set_pose_slam: service that sets the pose of the SLAM robot


using std::cout;
namespace{
    turtlelib::Transform2D T_ob;
    turtlelib::Transform2D T_mo;
    int timestep = 0;
    int debugger = 0;
    bool use_unknown = false;
    bool use_known = false;
    std::vector<double> o_r;
    double initial   ;


}




/// \brief two dimensional extended Kalman filter
class Kalman2D
{
    public:
        /// \brief Initiate Kalman filter object
        /// \param x0 - starting x point
        /// \param y0 - starting y point
        /// \param t0 - starting orientation
        /// \param landmarks - number of landmarks
        Kalman2D(double x, double y, double t, int landmarks){
            
            map_dim = landmarks;
            discover_unknown = 0;

            mah_lo = 0.01; // only associate with an existing landmark for state update if within certain threshold
            mah_hi = 0.07; // if measurement is further away than all existing measurements by this amount, it is new
            
            //Assign initial positions
            x_old = x;
            y_old = y;
            t_old = t;

            x_new = x;
            y_new = y;
            t_new = t;

            //Create EKF algorithm equations

            A = arma::mat((3 + 2 * landmarks), (3 + 2* landmarks));

            A.fill(0.0);
            // A((3 + 2 * landmarks), (3 + 2* landmarks),arma::fill::zeros);
            A.print("A: ");



            for(int i = 0; i< landmarks;i++){
                arma::mat temp_h = arma::mat(2,3+2*landmarks);
                H.push_back(temp_h);
                x_sense.push_back(0.0);
                y_sense.push_back(0.0);
                discover.push_back(true);
            }

            H_master = H.at(0);

            for(int i = 1; i< landmarks;i++){
                H_master = arma::join_cols(H_master, H.at(i));

            }

            H_individual = arma::mat(2,3+2*map_dim);

            K = arma::mat(3+2*landmarks,6);
            K.fill(0.0);
            K.print("K: ");

            K_ind = arma::mat(3+2*landmarks,2);



            VRVt = 0.001*arma::mat(2*landmarks,2*landmarks,arma::fill::eye); //Will need to be tuned with new values 
            VRVt_ind = 10*arma::mat(2,2,arma::fill::eye); //Will need to be tuned with new values 
            


            VRVt.print("VRVt:       ");


            Q = arma::mat(3,3,arma::fill::eye);

            arma::mat temp = arma::mat(3,2*landmarks,arma::fill::zeros);
            Q = arma::join_rows(Q, temp);
            temp = arma::mat(2*landmarks,3+2*landmarks);
            Q = arma::join_cols(Q, temp);
            Q = 0.1*Q; 
            Q.print("Q: ");

            sigma_q = arma::mat(3,3,arma::fill::zeros);
            sigma_q.print("sigmaq: ");

            sigma_m = 1000000.0*arma::mat(2*landmarks,2*landmarks,arma::fill::eye);
            sigma_m.print("sigma_m: ");

            
            temp = arma::mat(3,2*landmarks,arma::fill::zeros);
            sigma = arma::join_rows(sigma_q,temp);

            arma::mat temp1 = temp.t();
            temp1.print("temp1: ");

            temp1 = arma::join_rows(temp1,sigma_m);
            temp1.print("temp1new: ");

            sigma = arma::join_cols(sigma,temp1);
            sigma.print("sigma: ");

            H_master.print("H_master: ");

            state_vector = arma::mat(3+2*landmarks,1, arma::fill::zeros);
            state_vector(0,0) = t;
            state_vector(1,0) = x;
            state_vector(2,0) = y;

            state_vector.print("state_vector initial: ");


        }

        /// \brief Return the state of the environment
        /// \param  None
        /// \returns  arma::mat state_vector
        arma::mat returnState(){
            return state_vector;
        } 


        /// \brief Generate prediction model for the differential robot
        /// \param ang_now orientation of robot
        /// \param x_now x position of robot in map
        /// \param y_now y position of robot in map

        void predict(double ang_now, double x_now, double y_now){

            x_new = x_now;
            y_new = y_now;
            t_new = ang_now;

            x_now /= 5.0; //Sampling at 5Hz
            y_now /= 5.0;
            ang_now /= 5.0;


            t_old = state_vector(0,0);

            if(turtlelib::almost_equal(ang_now,0.0,0.0001)){ //pure translation

                // state_vector(0,0) = state_vector(0,0); // No rotation set it to itself
                state_vector(1,0) = state_vector(1,0) + x_now*cos(t_old); //x
                state_vector(2,0) = state_vector(2,0) + x_now*sin(t_old); //y

                A.fill(0.0);
 

                A = arma::eye(3+2*map_dim, 3+2*map_dim) + A;

                A(1,0) = -x_now*sin(t_old); 
                A(2,0) = x_now*cos(t_old);


            }else{

                state_vector(0,0) =   state_vector(0,0) + ang_now; //turtlelib::normalize_angle((state_vector(0,0) + dtheta)); //theta
                state_vector(1,0) = state_vector(1,0) + (-(x_now/ang_now)*sin(t_old) + (x_now/ang_now)*sin(t_old+ang_now)); //x
                state_vector(2,0) = state_vector(2,0) + ((x_now/ang_now)*cos(t_old) - (x_now/ang_now)*cos(t_old+ang_now)); //y

                A.fill(0.0);

                A = arma::eye(3+2*map_dim, 3+2*map_dim) + A;

                A(1,0) = -(x_now/ang_now)*cos(t_old) + (x_now/ang_now)*cos(t_old+ang_now);
                A(2,0) = -(x_now/ang_now)*sin(t_old) + (x_now/ang_now)*sin(t_old+ang_now);

            }

            sigma = A*sigma*A.t()+Q;


        }

        /// \brief Do a measurement update
        /// \param  sensed - Markerarray data
        /// \returns  None
        void update(visualization_msgs::MarkerArray sensed){

            arma::mat z_meas = arma::mat(2*map_dim,1);
            arma::mat z_ideal = arma::mat(2*map_dim,1);

            H_master.fill(0.0);
            z_meas.fill(0.0);
            z_ideal.fill(0.0);

            // turtlelib::Transform2D T_bw = T_wb.inv();

            std::cout<<"timestep:  " << timestep++ <<"\r \n";

            
            std::cout<<"before: ";


            for(int j=0;j<2*map_dim+3;j++){
                std::cout<<" state " << j << " " << state_vector(j,0) << "\r\n";

            }

            for(int j = 0;j<map_dim;j++){

                x_sense.at(j) = 0.0;
                y_sense.at(j) = 0.0;
                
                if(sensed.markers.at(j).action == 0){


                    x_sense.at(j) = sensed.markers.at(j).pose.position.x; //receive robot frame x and y positions
                    y_sense.at(j) = sensed.markers.at(j).pose.position.y;


                    if(discover.at(j)==true){
                        std::cout<<"discovering landmark: " << j << "\r\n";

                        discover.at(j) = false;
                        double r = sqrt( (x_sense.at(j))*(x_sense.at(j)) + (y_sense.at(j))*(y_sense.at(j)) );
                        double phi = atan2((y_sense.at(j)),(x_sense.at(j)));

                        state_vector(3+2*j,0) = state_vector(1,0) + r*cos(phi + state_vector(0,0) );
                        state_vector(3+2*j+1,0) = state_vector(2,0) + r*sin(phi + state_vector(0,0) );

                    }



                    std::cout << " xsense ysense j  " << x_sense.at(j) << " " << y_sense.at(j) << " " << j << " \n\r";

                    double r = sqrt( (x_sense.at(j))*(x_sense.at(j)) + (y_sense.at(j))*(y_sense.at(j)) );
                    // double phi = turtlelib::normalize_angle(atan2((y_sense.at(j)),(x_sense.at(j))) );
                    double phi = atan2((y_sense.at(j)),(x_sense.at(j)));



                    z_meas(2*j,0) = r; //Column vector of polar coordinate measurements
                    z_meas(2*j+1,0) = phi;
                    
                    std::cout << " r phi  " << r << " " << phi << " "  << " \n\r";
                    std::cout << " state theta x y  " << state_vector(0,0) << " " << state_vector(1,0) << " " << state_vector(2,0)  << " \n\r";

                    
                    double delta_x = state_vector(3+2*(j),0) - state_vector(1,0);//x               //These are the estimated states within the state vector
                    double delta_y = state_vector(4+2*(j),0) - state_vector(2,0);//y

                    double d = delta_x*delta_x + delta_y*delta_y;

                    //Compute H matrix
                    H_master(2*j+1,0) = -1.0;

                    H_master(2*j,1) = -delta_x/sqrt(d);
                    H_master(2*j+1,1) = delta_y/d;

                    H_master(2*j, 2) = -delta_y/sqrt(d);
                    H_master(2*j+1,2) = -delta_x/d;  

                    H_master(2*j,3+2*j) =   delta_x / sqrt(d);
                    H_master(2*j+1,3+2*j) = -delta_y/d;

                    H_master(2*j,3+2*j+1) = delta_y/sqrt(d);
                    H_master(2*j+1, 3+2*j+1) = delta_x/d;

                    z_ideal(2*j,0) = sqrt(   (state_vector(3+2*j,0)-(state_vector(1,0)))*(state_vector(3+2*j,0)-(state_vector(1,0))) + (state_vector(3+2*j+1,0)-(state_vector(2,0)))*(state_vector(3+2*j+1,0)-(state_vector(2,0)))  );
                    z_ideal(2*j+1,0) = atan2( (state_vector(3+2*j+1,0)-(state_vector(2,0))), (state_vector(3+2*j,0)-(state_vector(1,0))) ) - state_vector(0,0);


                }

            }

            arma::mat inverted = (H_master*sigma*H_master.t()+VRVt).i();
            K = sigma * H_master.t()*inverted;
            
            arma::mat zdelta = z_meas - z_ideal;

            for(int j=0;j<map_dim;j++){
                zdelta(2*j+1,0) = turtlelib::normalize_angle(zdelta(2*j+1,0));
            }
            
            state_vector = state_vector+K*(zdelta);

            arma::mat i_99 = arma::mat(3+2*map_dim,3+2*map_dim);
            i_99.eye();
            sigma = ( i_99 - K*H_master)*sigma;

    
            std::cout<<"\r \n";

            std::cout<<"after: ";
            for(int j=0;j<2*map_dim;j++){

               std::cout<<" z_ideal " << j << " " << z_ideal(j,0) << "\r\n";

            }

            for(int j=0;j<2*map_dim;j++){
                std::cout<<" z_meas " << j << " " << z_meas(j,0) << "\r\n";

            }

            for(int j=0;j<2*map_dim;j++){
                std::cout<<" z_delta " << j << " " << zdelta(j,0) << "\r\n";

            }

            for(int j=0;j<2*map_dim+3;j++){
                std::cout<<" state " << j << " " << state_vector(j,0) << "\r\n";

            }

            
            H_master.print("H: ");
                        std::cout<<"\r \n\n";

            A.print("A: ");
                        std::cout<<"\r \n\n";

            sigma.print("sigma: ");
                        std::cout<<"\r \n\n";
            K.print("K: ");
                        std::cout<<"\r \n\n";

            std::cout<<"\r \n\n";
            std::cout<<"\r \n\n";

        }

        /// \brief Do a measurement update
        /// \param  sensed - Markerarray data
        /// \returns  None
        void update_individual(visualization_msgs::MarkerArray sensed){

            arma::mat z_meas = arma::mat(2,1);
            arma::mat z_ideal = arma::mat(2,1);

            for(long unsigned int j = 0;j<(long unsigned int) map_dim;j++){
                H_individual.fill(0.0);
                z_meas.fill(0.0);
                z_ideal.fill(0.0);
                x_sense_ind = 0.0;
                y_sense_ind = 0.0;

                if(j< o_r.size()){//represents number of data points possible
                    if(sensed.markers.at(j).action == 0){ //Meaning in range

                        x_sense_ind = sensed.markers.at(j).pose.position.x;
                        y_sense_ind = sensed.markers.at(j).pose.position.y;

                        if(discover.at(j)==true){
                            std::cout<<"discovering landmark: " << j << "\r\n";

                            discover.at(j) = false;
                            double r = sqrt( (x_sense_ind)*(x_sense_ind) + (y_sense_ind)*(y_sense_ind)) ;
                            double phi = atan2((y_sense_ind),(x_sense_ind));

                            state_vector(3+2*j,0) = state_vector(1,0) + r*cos(phi + state_vector(0,0) );
                            state_vector(3+2*j+1,0) = state_vector(2,0) + r*sin(phi + state_vector(0,0) );

                        }

                        double r = sqrt( (x_sense_ind)*(x_sense_ind) + (y_sense_ind)*(y_sense_ind)) ;
                        double phi = atan2((y_sense_ind),(x_sense_ind));

                        z_meas(0,0) = r; //Column vector of polar coordinate measurements
                        z_meas(1,0) = phi;
                        
                        double delta_x = state_vector(3+2*(j),0) - state_vector(1,0);//x               //These are the estimated states within the state vector
                        double delta_y = state_vector(4+2*(j),0) - state_vector(2,0);//y
                        double d = delta_x*delta_x + delta_y*delta_y;

                        //Compute H matrix
                        H_individual(1,0) = -1.0;
                        H_individual(0,1) = -delta_x/sqrt(d);
                        H_individual(1,1) = delta_y/d;
                        H_individual(0, 2) = -delta_y/sqrt(d);
                        H_individual(1,2) = -delta_x/d;  
                        H_individual(0,3+2*j) =   delta_x / sqrt(d);
                        H_individual(1,3+2*j) = -delta_y/d;
                        H_individual(0,3+2*j+1) = delta_y/sqrt(d);
                        H_individual(1, 3+2*j+1) = delta_x/d;

                        z_ideal(0,0) = sqrt(   (state_vector(3+2*j,0)-(state_vector(1,0)))*(state_vector(3+2*j,0)-(state_vector(1,0))) + (state_vector(3+2*j+1,0)-(state_vector(2,0)))*(state_vector(3+2*j+1,0)-(state_vector(2,0)))  );
                        z_ideal(1,0) =atan2( (state_vector(3+2*j+1,0)-(state_vector(2,0))), (state_vector(3+2*j,0)-(state_vector(1,0))) ) - state_vector(0,0);

                        arma::mat inverted = (H_individual*sigma*H_individual.t()+VRVt_ind).i();

                        K_ind = sigma * H_individual.t()*inverted;
                        
                        arma::mat zdelta = z_meas - z_ideal;
                        zdelta(1,0) = turtlelib::normalize_angle(zdelta(1,0));

                        arma::mat state_update = K_ind*zdelta;
                        state_vector = state_vector+ K_ind*zdelta;

                        arma::mat i_99 = arma::mat(3+2*map_dim,3+2*map_dim);
                        i_99.eye();

                        sigma = ( i_99 - K_ind*H_individual)*sigma;


                    }else{
                        //Already filled H with 0s if no read

                    }

                }
            }
            

            

        }

        void update_ind_mah(nuslam::Unknown data){
            arma::mat z_meas = arma::mat(2,1);
            arma::mat z_ideal = arma::mat(2,1);
            int remove = -1;
           
            if(data.x.size()>1){ //filter out repeated measurements
                for(long unsigned int j=0;j<data.x.size()-1;j++){
                    if(turtlelib::almost_equal(data.x.at(j),data.x.at(j+1),0.1)){
                        remove = j;
                    }
                }
            }

            if(remove>-1){
                data.x.erase(data.x.begin()+remove);
                data.y.erase(data.y.begin()+remove);
            }


            for(long unsigned int j = 0;j<data.x.size();j++){
                //Reset H, z_meas, and z_ideal
                H_individual.fill(0.0);
                z_meas.fill(0.0);
                z_ideal.fill(0.0);
                x_sense_ind = 0.0;
                y_sense_ind = 0.0;

                // std::cout<<"debugger in data loop " << debugger++ <<"\r\n";

                if(discover_unknown == 0){//you initialize and make an update based on 1st measurement . Initialized 1st landmark.
                    // std::cout<<"debugger in first discovery " << debugger++ <<"\r\n";

                    x_sense_ind = data.x.at(j); //This should be only be fore the first measurement, first index
                    y_sense_ind = data.y.at(j);

                    double r = sqrt( (x_sense_ind)*(x_sense_ind) + (y_sense_ind)*(y_sense_ind)) ;
                    double phi = atan2((y_sense_ind),(x_sense_ind));

                    //Initializing  very first landmark, so it starts at landmark index = 0
                    state_vector(3+2*0,0) = state_vector(1,0) + r*cos(phi + state_vector(0,0) );
                    state_vector(3+2*0+1,0) = state_vector(2,0) + r*sin(phi + state_vector(0,0) );

                    z_meas(0,0) = r; //Column vector of polar coordinate measurements
                    z_meas(1,0) = phi;

                    double delta_x = state_vector(3+2*(0),0) - state_vector(1,0);//x               //These are the estimated states within the state vector
                    double delta_y = state_vector(4+2*(0),0) - state_vector(2,0);//y
                    double d = delta_x*delta_x + delta_y*delta_y;                    

                    //Compute H matrix // this becomes the first landmark by default, so its at index=0
                    H_individual(1,0) = -1.0;
                    H_individual(0,1) = -delta_x/sqrt(d);
                    H_individual(1,1) = delta_y/d;
                    H_individual(0, 2) = -delta_y/sqrt(d);
                    H_individual(1,2) = -delta_x/d;  
                    H_individual(0,3+2*0) =   delta_x / sqrt(d);
                    H_individual(1,3+2*0) = -delta_y/d;
                    H_individual(0,3+2*0+1) = delta_y/sqrt(d);
                    H_individual(1, 3+2*0+1) = delta_x/d;

                    z_ideal(0,0) = sqrt(   (state_vector(3+2*0,0)-(state_vector(1,0)))*(state_vector(3+2*0,0)-(state_vector(1,0))) + (state_vector(3+2*0+1,0)-(state_vector(2,0)))*(state_vector(3+2*0+1,0)-(state_vector(2,0)))  );
                    z_ideal(1,0) =atan2( (state_vector(3+2*0+1,0)-(state_vector(2,0))), (state_vector(3+2*0,0)-(state_vector(1,0))) ) - state_vector(0,0);

                    arma::mat inverted = (H_individual*sigma*H_individual.t()+VRVt_ind).i();

                    K_ind = sigma * H_individual.t()*inverted;
                    
                    arma::mat zdelta = z_meas - z_ideal;
                    zdelta(1,0) = turtlelib::normalize_angle(zdelta(1,0));

                    arma::mat state_update = K_ind*zdelta;
                    state_vector = state_vector + K_ind*zdelta;

                    arma::mat i_99 = arma::mat(3+2*map_dim,3+2*map_dim);
                    i_99.eye();

                    sigma = ( i_99 - K_ind*H_individual)*sigma;

                    discover_unknown++;
                    discover.at(0) = false; //now arbitrary landmark 0 has been discovered

                }else{ 
                        arma::mat H_temp = arma::mat(2,3+2*(map_dim),arma::fill::zeros); // you need a temp R and sigma matrix, sigma temp and R temp
                        arma::mat sigma_temp = arma::mat(3+2*(map_dim),3+2*(map_dim),arma::fill::zeros); //This needs to be filled with appropairate values
                        sigma_temp = sigma;

                        arma::mat phi_k = arma::mat(2,2,arma::fill::zeros);
                        
                        double x_temp = state_vector(1,0) + data.x.at(j); //these are intended to be the map x y states for the temporary landmark
                        double y_temp = state_vector(2,0) + data.y.at(j);
                        std::cout<<" temp x state " << x_temp << " temp y state " << y_temp << "\r\n";

                        double d_temp = 1000.0; //Create an arbitrarily large starting distance to compare to
                        arma::mat d_k = arma::mat(1,1,arma::fill::zeros); //mahalanobis distance variable

                        int tracker_idx = -1; //THis is the tracker idx to know which state_vector we associate data with

                        x_sense_ind = data.x.at(j);
                        y_sense_ind = data.y.at(j); 


                        double r = sqrt( (x_sense_ind)*(x_sense_ind) + (y_sense_ind)*(y_sense_ind)) ;
                        double phi = atan2((y_sense_ind),(x_sense_ind));

                        z_meas(0,0) = r; //Column vector of polar coordinate measurements
                        z_meas(1,0) = phi;  



                        for(int k=0; k<map_dim;k++){ //You also need to look at the temporary landmark
                            
                            double d,delta_x,delta_y;
                            phi_k = arma::mat(2,2,arma::fill::zeros); // Reset the phik matrix for each iteration through landmarks
                            H_temp = arma::mat(2,3+2*(map_dim),arma::fill::zeros);
                            
                            
                            if((k==discover_unknown) && (discover.at(discover_unknown) == true)){

                                d_k(0,0) = mah_hi; // we set d_k to a threshold for temporary landmark

                                std::cout<<"checking new landmark number : " << k << " d_k = " << d_k << "\r\n";
                                // H_temp.print("H_temp: ");


                            }else if(k<discover_unknown && discover.at(k) == false){ // you only make this update if there is a landmark discovered at this k position
                                z_ideal(0,0) = sqrt(   (state_vector(3+2*k,0)-(state_vector(1,0)))*(state_vector(3+2*k,0)-(state_vector(1,0))) + (state_vector(3+2*k+1,0)-(state_vector(2,0)))*(state_vector(3+2*k+1,0)-(state_vector(2,0)))  );
                                z_ideal(1,0) = atan2( (state_vector(3+2*k+1,0)-(state_vector(2,0))), (state_vector(3+2*k,0)-(state_vector(1,0))) ) - state_vector(0,0);

                                delta_x = state_vector(3+2*(k),0) - state_vector(1,0);//x               //These are the estimated states within the state vector
                                delta_y = state_vector(4+2*(k),0) - state_vector(2,0);//y
                                d = delta_x*delta_x + delta_y*delta_y;  

                                //Compute H matrix
                                H_temp(1,0) = -1.0;
                                H_temp(0,1) = -delta_x/sqrt(d);
                                H_temp(1,1) = delta_y/d;
                                H_temp(0, 2) = -delta_y/sqrt(d);
                                H_temp(1,2) = -delta_x/d;  
                                H_temp(0,3+2*k) =   delta_x / sqrt(d);
                                H_temp(1,3+2*k) = -delta_y/d;
                                H_temp(0,3+2*k+1) = delta_y/sqrt(d);
                                H_temp(1, 3+2*k+1) = delta_x/d;

                                phi_k = H_temp*sigma_temp*H_temp.t() + VRVt_ind;
                                
                                arma::mat zdelta = z_meas - z_ideal;
                                zdelta(1,0) = turtlelib::normalize_angle(zdelta(1,0));

                                d_k = zdelta.t()*phi_k.i()*zdelta;

                            }


                            if(d_k(0,0)<d_temp){ // Identify smallest d_k value and index
                                d_temp = d_k(0,0);
                                tracker_idx = k;
                            }   
                        }
                        std::cout<< "d_temp track_idx   " << d_temp << " " << tracker_idx << "\r\n\r\n\r\n";
                        if((tracker_idx==discover_unknown) && (d_temp > (mah_hi - 0.01))){ //In this case we have a new landmark. We would arrive at a d_temp = to mah_hi
                            std::cout<<"\r\n\r\n DISCOVERING NEW LANDMARK!!!!! discover_unknown = " << discover_unknown<< "\r\n\r\n\n";
                            discover.at(discover_unknown) = false; //set a new landmark to discovered

                            double r = sqrt( (x_sense_ind)*(x_sense_ind) + (y_sense_ind)*(y_sense_ind)) ;
                            double phi = atan2((y_sense_ind),(x_sense_ind));

                            //Initializing  new landmark at index = tracker_idx
                            state_vector(3+2*discover_unknown,0) = state_vector(1,0) + r*cos(phi + state_vector(0,0) );
                            state_vector(3+2*discover_unknown+1,0) = state_vector(2,0) + r*sin(phi + state_vector(0,0) );

                            z_meas(0,0) = r; //Column vector of polar coordinate measurements
                            z_meas(1,0) = phi;

                            double delta_x = state_vector(3+2*(discover_unknown),0) - state_vector(1,0);//x               //These are the estimated states within the state vector
                            double delta_y = state_vector(4+2*(discover_unknown),0) - state_vector(2,0);//y
                            double d = delta_x*delta_x + delta_y*delta_y; 


                            //Compute H matrix // this becomes the first landmark by default, so its at index=0
                            H_individual(1,0) = -1.0;
                            H_individual(0,1) = -delta_x/sqrt(d);
                            H_individual(1,1) = delta_y/d;
                            H_individual(0, 2) = -delta_y/sqrt(d);
                            H_individual(1,2) = -delta_x/d;  
                            H_individual(0,3+2*discover_unknown) =   delta_x / sqrt(d);
                            H_individual(1,3+2*discover_unknown) = -delta_y/d;
                            H_individual(0,3+2*discover_unknown+1) = delta_y/sqrt(d);
                            H_individual(1, 3+2*discover_unknown+1) = delta_x/d;

                            z_ideal(0,0) = sqrt(   (state_vector(3+2*discover_unknown,0)-(state_vector(1,0)))*(state_vector(3+2*discover_unknown,0)-(state_vector(1,0))) + (state_vector(3+2*discover_unknown+1,0)-(state_vector(2,0)))*(state_vector(3+2*discover_unknown+1,0)-(state_vector(2,0)))  );
                            z_ideal(1,0) =atan2( (state_vector(3+2*discover_unknown+1,0)-(state_vector(2,0))), (state_vector(3+2*discover_unknown,0)-(state_vector(1,0))) ) - state_vector(0,0);

                            arma::mat inverted = (H_individual*sigma*H_individual.t()+VRVt_ind).i();

                            K_ind = sigma * H_individual.t()*inverted;
                            
                            arma::mat zdelta = z_meas - z_ideal;
                            zdelta(1,0) = turtlelib::normalize_angle(zdelta(1,0));

                            arma::mat state_update = K_ind*zdelta;
                            state_vector = state_vector+ K_ind*zdelta;

                            arma::mat i_99 = arma::mat(3+2*map_dim,3+2*map_dim);
                            i_99.eye();

                            sigma = ( i_99 - K_ind*H_individual)*sigma;

                            discover_unknown++; //iterate for next discovered landmark

                        }if((tracker_idx<discover_unknown) && (d_temp<mah_lo)){ // in this case, we are associated with an existing landmark
                            
                            std::cout<<" associating with landmark tracker_idx = " << tracker_idx<< "\r\n\r\n\n";

                            double r = sqrt( (x_sense_ind)*(x_sense_ind) + (y_sense_ind)*(y_sense_ind)) ;
                            double phi = atan2((y_sense_ind),(x_sense_ind));

                            z_meas(0,0) = r; //Column vector of polar coordinate measurements
                            z_meas(1,0) = phi;

                            double delta_x = state_vector(3+2*(tracker_idx),0) - state_vector(1,0);//x               //These are the estimated states within the state vector
                            double delta_y = state_vector(4+2*(tracker_idx),0) - state_vector(2,0);//y
                            double d = delta_x*delta_x + delta_y*delta_y;

                            //Compute H matrix
                            H_individual(1,0) = -1.0;
                            H_individual(0,1) = -delta_x/sqrt(d);
                            H_individual(1,1) = delta_y/d;
                            H_individual(0, 2) = -delta_y/sqrt(d);
                            H_individual(1,2) = -delta_x/d;  
                            H_individual(0,3+2*tracker_idx) =   delta_x / sqrt(d);
                            H_individual(1,3+2*tracker_idx) = -delta_y/d;
                            H_individual(0,3+2*tracker_idx+1) = delta_y/sqrt(d);
                            H_individual(1, 3+2*tracker_idx+1) = delta_x/d;

                            z_ideal(0,0) = sqrt(   (state_vector(3+2*tracker_idx,0)-(state_vector(1,0)))*(state_vector(3+2*tracker_idx,0)-(state_vector(1,0))) + (state_vector(3+2*tracker_idx+1,0)-(state_vector(2,0)))*(state_vector(3+2*tracker_idx+1,0)-(state_vector(2,0)))  );
                            z_ideal(1,0) =atan2( (state_vector(3+2*tracker_idx+1,0)-(state_vector(2,0))), (state_vector(3+2*tracker_idx,0)-(state_vector(1,0))) ) - state_vector(0,0);

                            arma::mat inverted = (H_individual*sigma*H_individual.t()+VRVt_ind).i();

                            K_ind = sigma * H_individual.t()*inverted;
                            
                            arma::mat zdelta = z_meas - z_ideal;
                            zdelta(1,0) = turtlelib::normalize_angle(zdelta(1,0));

                            arma::mat state_update = K_ind*zdelta;
                            state_vector = state_vector+ K_ind*zdelta;

                            arma::mat i_99 = arma::mat(3+2*map_dim,3+2*map_dim);
                            i_99.eye();

                            sigma = ( i_99 - K_ind*H_individual)*sigma;


                        }



                }



            }

        
            
        }


        /// \brief print states
        /// \param  None
        /// \returns  None
        void printStates(){
            for(int i=0; i<3+2*map_dim;i++){        
                std::cout << " state " << i<< "  " << state_vector(i,0) << "\r \n";

            }
            std::cout << "\r \n";
            // H_master.print("H: ");
            // K.print("K: ");
            std::cout << "\r \n";

        }

        /// \brief print states
        /// \param  j - obstacle index to check
        /// \returns  bool true/false
        bool returnDiscover(int j){

            return discover.at(j);
        }


        /// \brief set robot positional states
        /// \param  theta - new angle
        /// \param  x - new x position
        /// \param  y - new y position       
        /// \returns  void
        void setRobot(double theta,double x, double y){
            
            state_vector(0,0) = theta;
            state_vector(1,0) = x;
            state_vector(2,0) = y;

        }


    private:

        int map_dim;
        int discover_unknown;

        double x_old;
        double y_old;
        double t_old;

        double y_new;
        double x_new;
        double t_new;

        double mah_lo;
        double mah_hi;

        arma::mat A;
        

        std::vector<arma::mat> H;

        arma::mat H_master; // THis is the concatentation of the individual H matrices
        arma::mat H_individual;


        arma::mat K;
        arma::mat K_ind;


        arma::mat sigma; //covariance matrix
        arma::mat sigma_q;
        arma::mat sigma_m;

        arma::mat Q;

        arma::mat VRVt;
        arma::mat VRVt_ind;

        arma::mat state_vector;

        std::vector<double> x_sense;
        std::vector<double> y_sense;
        std::vector<bool> discover;
        

        double x_sense_ind;
        double y_sense_ind;



};


namespace{
    Kalman2D k(0,0,0,16);
    turtlelib::Diff_Drive diff1(0.0,0.0,0.0,0.0,0.0);
    turtlelib::Twist2D twist1;
    
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
    

}

/// \brief fake sensor callback
/// \param msg - LiDAR data of each marker
/// \returns None
void fake_sensorCallback(const visualization_msgs::MarkerArray& msg){
    if(use_known){
        std::cout << "using known data association \r\n";
        k.predict(twist1.tw.at(0),twist1.tw.at(1),0.0);
        k.update_individual(msg);

        arma::mat state = k.returnState();

        turtlelib::Vector2D v_temp;
        v_temp.x = state(1,0);
        v_temp.y = state(2,0);

        turtlelib::Transform2D T_mb = turtlelib::Transform2D(v_temp, state(0,0));
        T_mo = T_mb*T_ob.inv();




        if(debugger%10 == 0){
            k.printStates();

        }
        debugger++;


        if((ros::Time::now().toSec()-initial) < 10.0){
            turtlelib::Vector2D vtemp;
            vtemp.x = xorigin;
            vtemp.y = yorigin;
            T_ob = turtlelib::Transform2D(vtemp,theta0);
            diff1 = turtlelib::Diff_Drive(track_width/2.0,wheel_radius,xorigin,yorigin,theta0,pos_wheel.at(0),pos_wheel.at(1));
            T_ob = diff1.fkin(pos_wheel.at(0),pos_wheel.at(1));
            T_ob = turtlelib::Transform2D(vtemp,theta0);
            T_mo = T_mb*T_ob.inv();

        }

    }


}


/// \brief LiDAR sensor callback
/// \param msg - LiDAR x,y,radius
/// \returns None
void mysteryCallback(const nuslam::Unknown& msg){
    
    
    if(use_unknown){
        std::cout << "using unknown data association \r\n";

        k.predict(twist1.tw.at(0),twist1.tw.at(1),0.0);

        k.update_ind_mah(msg);

        arma::mat state = k.returnState();


        turtlelib::Vector2D v_temp;
        v_temp.x = state(1,0);
        v_temp.y = state(2,0);

        turtlelib::Transform2D T_mb = turtlelib::Transform2D(v_temp, state(0,0));
        T_mo = T_mb*T_ob.inv();

        if(debugger%10 == 0){
            k.printStates();

        }



        if((ros::Time::now().toSec()-initial) < 10.0){

            turtlelib::Vector2D vtemp;
            vtemp.x = xorigin;
            vtemp.y = yorigin;
            T_ob = turtlelib::Transform2D(vtemp,theta0);
            diff1 = turtlelib::Diff_Drive(track_width/2.0,wheel_radius,xorigin,yorigin,theta0,pos_wheel.at(0),pos_wheel.at(1));
            T_ob = diff1.fkin(pos_wheel.at(0),pos_wheel.at(1));
            T_ob = turtlelib::Transform2D(vtemp,theta0);
            T_mo = T_mb*T_ob.inv();

        }
        debugger++;
    }

}


    /// \brief joint states callback to computes wheel velocities and positions
    /// \param msg - geometry_msgs::Twist data
    /// \returns None
void joint_statesCallback(const sensor_msgs::JointState& msg){
     
    int i = 0; 

    //Assign wheel positions
    for(i=0;i<2;i++){
        v_wheel.at(i) = msg.velocity[i]; //Alternative: calculate delta angle and divide by delta time
        pos_wheel.at(i) =  msg.position[i];
    }


    //Compute the twist
    twist1.tw.at(0) = -wheel_radius*v_wheel.at(0)/track_width + wheel_radius*v_wheel.at(1)/track_width; 
    twist1.tw.at(1) = wheel_radius*v_wheel.at(0)/2.0 + wheel_radius*v_wheel.at(1)/2.0; 
    twist1.tw.at(2) = 0.0;

    T_ob = diff1.fkin(pos_wheel.at(0),pos_wheel.at(1));
    if((ros::Time::now().toSec()-initial) < 10.0){
            std::cout<<"PLEASE WAIT TILL TIMER IS 0 BEFORE MOVING: " << (10.0 - (ros::Time::now().toSec()-initial))<<"\r\n";


            turtlelib::Vector2D vtemp;
            vtemp.x = xorigin;
            vtemp.y = yorigin;
            T_ob = turtlelib::Transform2D(vtemp,theta0);
            diff1 = turtlelib::Diff_Drive(track_width/2.0,wheel_radius,xorigin,yorigin,theta0,pos_wheel.at(0),pos_wheel.at(1));
            T_ob = diff1.fkin(pos_wheel.at(0),pos_wheel.at(1));
            T_ob = turtlelib::Transform2D(vtemp,theta0);

            
            arma::mat state = k.returnState();


            turtlelib::Vector2D v_temp;
            v_temp.x = state(1,0);
             v_temp.y = state(2,0);

            turtlelib::Transform2D T_mb = turtlelib::Transform2D(v_temp, state(0,0));
            T_mo = T_mb*T_ob.inv();

        }


}


    /// \brief move the robot in the odom frame
    /// \param request - x,y, theta coordinates to set the odometry at
    /// \returns None
bool set_pose_slam(nuslam::SetPoseSlam::Request& request, nuslam::SetPoseSlam::Response& ){


    //Set the pose
    k.setRobot(request.t,request.x,request.y);
    diff1 = turtlelib::Diff_Drive(track_width/2.0,wheel_radius,request.x,request.y,request.t);
    T_ob = diff1.fkin(pos_wheel.at(0),pos_wheel.at(1));

    return true;

}

int main(int argc, char * argv[]){
    
    // long int k=0;

    int f = 0;

    std::string body_id = "undefined";
    std::string odom_id = "odom";
    std::string undefined = "undefined";

    ros::init(argc,argv,"slam");
    ros::NodeHandle n;

    visualization_msgs::Marker m_sense;
    visualization_msgs::MarkerArray m_array_sense;


    //Load in parameters
    n.param("rate", f, 500); //Source (01/29): https://answers.ros.org/question/253797/no-matching-function-for-call-to-rosnodehandleparam/
    n.param("wheel_radius", wheel_radius, 0.033);
    n.param("track_width", track_width, 0.16);
    n.param("encoder_ticks_to_rad",encoder_ticks_to_rad,-1.0);
    n.param("motor_cmd_to_radsec",motor_cmd_to_radsec,-1.0);
    n.param("slam/x0", xorigin, 0.0);
    n.param("slam/theta0", theta0, 0.0);
    n.param("slam/y0",yorigin,0.0);
    n.getParam("slam/obstacles/r",o_r);
    
    n.param("/slam/body_id",body_id,undefined);
    n.param("/slam/odom_id",odom_id,odom_id);
    n.param("wheel_left",wheel_left,undefined);
    n.param("wheel_right",wheel_right,undefined);
    n.param("motor_cmd_max", motor_cmd_max, 256);
    n.param("use_known",use_known,false);
    n.param("use_unknown",use_unknown,false);

    if(!(n.hasParam("body_id")) ||   !(n.hasParam("wheel_left")) ||  !(n.hasParam("wheel_right")) ){
        ROS_ERROR_STREAM("Undefined parameter - terminating!\n");
        throw;
    }
    
    
    //Kalman filter
    k = Kalman2D(xorigin,yorigin,theta0,16);

    //Subscribers
    ros::Subscriber joint_state_sub = n.subscribe("joint_states", 10, joint_statesCallback);
    ros::Subscriber marker_sub = n.subscribe("/fake_sensor", 10, fake_sensorCallback);
    ros::Subscriber unknown_sub = n.subscribe("/mystery", 10, mysteryCallback);


    //Publishers
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom_slam", 100);
    ros::Publisher trail_pub = n.advertise<nav_msgs::Path>("pathodom_slam", 100);

    ros::Publisher marker_pub;
    marker_pub = n.advertise<visualization_msgs::MarkerArray>("map_obstacles", 100);
    //Services
    ros::ServiceServer srv_set_pose = n.advertiseService("set_pose_slam", set_pose_slam);

    ros::Rate rate(500); //CHANGE BACK TO PARAMETER FREQUENCY

    //Throw if parameters undefined in parameter server
    if(track_width < 0 || wheel_radius < 0 || motor_cmd_to_radsec < 0 || encoder_ticks_to_rad < 0 || body_id == "undefined" || wheel_left == "undefined" || wheel_right == "undefined"){
        ROS_ERROR_STREAM("Undefined parameter - terminating!\n");
        throw;
    }

    //Differential drive model
    diff1 = turtlelib::Diff_Drive(track_width/2.0,wheel_radius,xorigin,yorigin,theta0);

    T_mo = turtlelib::Transform2D();
    T_ob= turtlelib::Transform2D();
    T_ob = diff1.fkin(pos_wheel.at(0),pos_wheel.at(1));

    //Declare transform
    tf2_ros::TransformBroadcaster b;
    geometry_msgs::TransformStamped ts;
 
    tf2_ros::TransformBroadcaster b_mo;
    geometry_msgs::TransformStamped ts_mo;


    //Declare odometry
    nav_msgs::Odometry odom;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "green-base_footprint";

    //Declare path
    nav_msgs::Path trail;
    trail.header.frame_id = odom_id;

    geometry_msgs::PoseStamped pose_stamp;
    pose_stamp.header.frame_id = odom_id;


    ros::Duration life_sense(0.5);

    for(long unsigned int i=0;i<o_r.size();i++){
        m_sense.header.frame_id = "map";
        m_sense.header.stamp = ros::Time::now();
        m_sense.ns = "slam";
        m_sense.action = visualization_msgs::Marker::DELETE;
        m_sense.id = i;
        m_sense.type = visualization_msgs::Marker::CYLINDER;

        m_sense.scale.x = o_r[i];
        m_sense.scale.y = o_r[i];
        m_sense.scale.z = 0.25; 

        m_sense.color.r = 0.0;
        m_sense.color.g = 1.0;
        m_sense.color.b = 0.0;
        m_sense.color.a = 1.0;

        m_sense.pose.orientation.x = 0.0;
        m_sense.pose.orientation.y = 0.0;
        m_sense.pose.orientation.z = 0.0;
        m_sense.pose.orientation.w = 1.0;
        
        m_sense.pose.position.x = 0.0;
        m_sense.pose.position.y = 0.0;
        m_sense.pose.position.z = 0.125;

        m_sense.frame_locked = true;

        m_sense.lifetime = life_sense;

        m_array_sense.markers.push_back(m_sense);

    }
    initial = ros::Time::now().toSec();

    while(ros::ok()){


        //Publish odometry
        odom.header.stamp = ros::Time::now();
        odom.twist.twist.angular.z = twist1.tw.at(0);
        odom.twist.twist.linear.x = twist1.tw.at(1);
        geometry_msgs::Quaternion orientation_quaternion = tf::createQuaternionMsgFromYaw(T_ob.rotation()); //source (02/01): http://wiki.ros.org/navigation/Tutorials/RobotSetup/Odom
        odom.pose.pose.position.x = T_ob.translation().x;
        odom.pose.pose.position.y = T_ob.translation().y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = orientation_quaternion;
        odom_pub.publish(odom);



        pose_stamp.header.stamp = ros::Time::now();
        pose_stamp.pose.position = odom.pose.pose.position;
        pose_stamp.pose.orientation = orientation_quaternion;

        trail.header.stamp = ros::Time::now();
        trail.poses.push_back(pose_stamp);
        
        trail_pub.publish(trail);
        ros::spinOnce();

        //Broadcast transform map odom
        ts_mo.header.stamp = ros::Time::now();
        ts_mo.header.frame_id = "map";
        ts_mo.child_frame_id = "odom";
        ts_mo.transform.translation.x = T_mo.translation().x;
        ts_mo.transform.translation.y = T_mo.translation().y;
        ts_mo.transform.translation.z = 0;
        tf2::Quaternion ang_mo;
        ang_mo.setRPY(0,0,T_mo.rotation());
        ts_mo.transform.rotation.x = ang_mo.x();
        ts_mo.transform.rotation.y = ang_mo.y();
        ts_mo.transform.rotation.z = ang_mo.z();
        ts_mo.transform.rotation.w = ang_mo.w();
        b_mo.sendTransform(ts_mo);



        //Broadcast transform odom base
        ts.header.stamp = ros::Time::now();
        ts.header.frame_id = "odom";
        ts.child_frame_id = "green-base_footprint";
        ts.transform.translation.x = T_ob.translation().x;
        ts.transform.translation.y = T_ob.translation().y;
        ts.transform.translation.z = 0;
        tf2::Quaternion ang;
        ang.setRPY(0,0,T_ob.rotation());
        ts.transform.rotation.x = ang.x();
        ts.transform.rotation.y = ang.y();
        ts.transform.rotation.z = ang.z();
        ts.transform.rotation.w = ang.w();
        b.sendTransform(ts);


        if(use_known){
            for(long unsigned int i=0;i<o_r.size();i++){
                m_array_sense.markers[i].header.stamp = ros::Time::now();
                arma::mat current = k.returnState();
                // std::cout<<" obstacle array size " << o_r.size() << "\r\n";

                if( k.returnDiscover(i) == false){
                    // std::cout<<"i: " << i << "\r\n";
                    m_array_sense.markers.at(i).pose.position.x = current(3+2*i);
                    m_array_sense.markers.at(i).pose.position.y = current(4+2*i);
                    m_array_sense.markers[i].action = visualization_msgs::Marker::ADD;

                }

            }

            marker_pub.publish(m_array_sense);
        }

        if(use_unknown){
            for(long unsigned int i=0;i<o_r.size();i++){
                m_array_sense.markers[i].header.stamp = ros::Time::now();
                arma::mat current = k.returnState();
                if( k.returnDiscover(i) == false){
                    m_array_sense.markers.at(i).pose.position.x = current(3+2*i);
                    m_array_sense.markers.at(i).pose.position.y = current(4+2*i);
                    m_array_sense.markers[i].action = visualization_msgs::Marker::ADD;

                }

            }

            marker_pub.publish(m_array_sense);
        }  

  
        rate.sleep(); 
    }

    return 0;
}
