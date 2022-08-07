
#include <iostream>
#include <cmath>
#include "lidar/lidarml.hpp"
#include <armadillo>

#define PI 3.14159265358979323846

using std::vector;
using std::cout;

namespace lidar{



    Lidar2D::Lidar2D(int num_pts, double d){
        resolution = num_pts;
        cluster_threshold = d;

    }

    void Lidar2D::cluster(std::vector<float> cloud){
        vector<double> x_temp;
        vector<double> y_temp;
        vector<int> idx_temp;

        for(int i=0;i<resolution;i++){
            // std::cout<<"i: " << i << "\r\n";
            // std::cout<<" one \r\n";
            int k = 0; //this is the position that the temp vectors are starting at for current loop iteration

            x_temp.clear();
            y_temp.clear();
            idx_temp.clear();
            double ang = (double) (i%resolution) * PI / 180.0; //might need to use remainder operator
            
            //Initialize new cluster

            x_temp.push_back(cloud.at(i%resolution)*cos(ang));
            y_temp.push_back(cloud.at(i%resolution)*sin(ang));
            idx_temp.push_back(i);

            i++;//next position
            k++;
            ang = (double) (i%resolution) * PI / 180.0;

            //Iterate through possible cluster
            double x_current = cloud.at(i%resolution)*cos(ang);
            double y_current = cloud.at(i%resolution)*sin(ang);


            while( (sqrt(((x_current-x_temp.at(k-1))*(x_current-x_temp.at(k-1)) + (y_current-y_temp.at(k-1))*(y_current-y_temp.at(k-1)) ) ) < cluster_threshold)  && (sqrt(((x_current-x_temp.at(k-1))*(x_current-x_temp.at(k-1)) + (y_current-y_temp.at(k-1))*(y_current-y_temp.at(k-1)) ) ) > 0.01)      ) {
                // std::cout<<" i k " << i << " " << k << "\r\n";

                x_temp.push_back(x_current);
                y_temp.push_back(y_current);
                idx_temp.push_back(k);

                i++;
                k++;

                 //next position, reassign current x,y

                ang = (double) (i%resolution) * PI / 180.0;

                // std::cout<<"throwing here \r\n";
                x_current = cloud.at(i%resolution)*cos(ang);
                y_current = cloud.at(i%resolution)*sin(ang);
                // std::cout<<"not here \r\n";

                // }

            }

            if(x_temp.size()>3){//only add if you are above a cluster count of 3 pts
                // std::cout<<" greater than critical size \r\n";
                x_cluster.push_back(x_temp);
                y_cluster.push_back(y_temp);
                idx.push_back(idx_temp);
                std::cout<<"temp size " << x_temp.size() << " \r\n";

            }
        }



        //checking for wrap around
        bool flag = false;
        // cout<<"alpha\r\n";
        if(x_temp.size()>3){

            for(long unsigned int k=1;k<idx.at(idx.size()-1).size() ;k++  ){
                // cout<<" k: " << k << "\r\n";
                // cout<<"begin index: " << idx.at(0).at(0) << "\r\n";
                // cout<<"compared to : "<<idx.at(idx.size()-1).at(k)%resolution<<"\r\n";

                if( (idx.at(idx.size()-1).at(k) %resolution) == idx.at(0).at(0)){
                    flag = true;
                    // std::cout<<"setting flag to true \r\n";
                }

            }
            // cout<<"beta\r\n";

            if(flag){ // if wrap around happened, pop the first  element to not double count
                idx.erase(idx.begin());
                x_cluster.erase(x_cluster.begin());
                y_cluster.erase(y_cluster.begin());
                // cout<<"erased\r\n";
            }
            cout<<"size of cluster:  " <<idx.size()<<"\r\n";
            
        }




    }

    void Lidar2D::print_cluster(){ //print clusters for debugging
        for(long unsigned int i=0;i<idx.size();i++){
            std::cout<<" cluster number i = " << i <<"\r\n";

            for(long unsigned int j=0;j<idx.at(i).size();j++){
                std::cout<<" subposition j = " << j << "\r\n";
                std::cout<<" idx " << idx.at(i).at(j) << " x " << x_cluster.at(i).at(j) << " y " << y_cluster.at(i).at(j) << "\r\n";
            }
            std::cout<<"\r\n";

        }

    }

    void Lidar2D::clear_cluster(){
        //clear clusters
        x_cluster.clear();
        y_cluster.clear();
        idx.clear();


    }

    vector<vector<double>> Lidar2D::return_x_cluster(){
        return x_cluster;
    }
   
    vector<vector<double>> Lidar2D::return_y_cluster(){
        return y_cluster;
    }

    vector<bool> Lidar2D::classify(vector<vector<double>> xdata, vector<vector<double>> ydata){
        // std::cout<<" one \r\n";
        std::vector<bool> detect;

        for(long unsigned int i =0;i<xdata.size();i++){ //check angles in clusterd circle
            double ang_sum = 0.0;
            double ang_var_sum = 0.0;
            // std::cout<<" two \r\n";
            int idx_last = xdata.at(i).size()-1;

            for(long unsigned int j=1;j<xdata.at(i).size()-1;j++){
                // std::cout<< " j:  " << j << "\r\n";
                double a = sqrt( (xdata.at(i).at(0) - xdata.at(i).at(j)) * (xdata.at(i).at(0) - xdata.at(i).at(j)) + (ydata.at(i).at(0) - ydata.at(i).at(j))*(ydata.at(i).at(0) - ydata.at(i).at(j)) ) ;
                double b = sqrt( (xdata.at(i).at(idx_last) - xdata.at(i).at(j)) * (xdata.at(i).at(idx_last) - xdata.at(i).at(j)) + (ydata.at(i).at(idx_last) - ydata.at(i).at(j))*(ydata.at(i).at(idx_last) - ydata.at(i).at(j)) );
                double c = sqrt( (xdata.at(i).at(idx_last) - xdata.at(i).at(0)) * (xdata.at(i).at(idx_last) - xdata.at(i).at(0)) + (ydata.at(i).at(idx_last) - ydata.at(i).at(0))*(ydata.at(i).at(idx_last) - ydata.at(i).at(0)) );
                // std::cout<<" a b c   " << a << " " << b << " " << " " << c << "\r\n";
                // std::cout<< " ang " << acos((a*a+b*b-c*c)/(2*a*b)) << " \r\n";
                ang_sum+=acos((a*a+b*b-c*c)/(2*a*b));
            }

            double ang_mean = ang_sum / (xdata.at(i).size() - 2);
                        
                        
            for(long unsigned int j=1;j<xdata.at(i).size()-1;j++){
                int idx_last = xdata.at(i).size()-1;

                double a = sqrt( (xdata.at(i).at(0) - xdata.at(i).at(j)) * (xdata.at(i).at(0) - xdata.at(i).at(j)) + (ydata.at(i).at(0) - ydata.at(i).at(j))*(ydata.at(i).at(0) - ydata.at(i).at(j)) ) ;
                double b = sqrt( (xdata.at(i).at(idx_last) - xdata.at(i).at(j)) * (xdata.at(i).at(idx_last) - xdata.at(i).at(j)) + (ydata.at(i).at(idx_last) - ydata.at(i).at(j))*(ydata.at(i).at(idx_last) - ydata.at(i).at(j)) );
                double c = sqrt( (xdata.at(i).at(idx_last) - xdata.at(i).at(0)) * (xdata.at(i).at(idx_last) - xdata.at(i).at(0)) + (ydata.at(i).at(idx_last) - ydata.at(i).at(0))*(ydata.at(i).at(idx_last) - ydata.at(i).at(0)) );
               

                ang_var_sum += (acos((a*a+b*b-c*c)/(2*a*b)) - ang_mean)*(acos((a*a+b*b-c*c)/(2*a*b)) - ang_mean);     
            }

            double ang_var = ang_var_sum / (xdata.at(i).size() - 2);

            // std::cout<<"mean: " << ang_mean << "\r\n";
            // std::cout<<"std dev: " << sqrt(ang_var) << "\r\n";

            if((sqrt(ang_var) < 0.15) && (abs(ang_mean) < 0.8*PI) && (abs(ang_mean) > 0.5*PI)){
                // is_circle.at(i) = true;
                detect.push_back(true);
                std::cout<<"Cluster i= " << i << " is a circle !!!!\r\n";
            }else{
                detect.push_back(false);
                std::cout<<"Cluster i= " << i << " is not a circle !!!!\r\n";

            }

        }

        return detect;
    }

    vector<vector<double>> Lidar2D::circle(vector<vector<double>> xdata, vector<vector<double>> ydata){
        // std::cout<<"one\r\n";
        // perform circle regression to extract x,y,radius information
        //Create vector to return x,y,radius information for kalman filter
        vector<vector<double>> parameters;
        parameters.clear(); // make sure it is empty

        for(long unsigned int i=0;i<xdata.size();i++){
            vector<double> temp = {0.0,0.0,0.0};
            parameters.push_back(temp);
            // is_circle.push_back(false); // 

            double x_sum = 0.0;
            double y_sum = 0.0;
            double z_sum = 0.0;


            for(long unsigned int j=0;j<xdata.at(i).size();j++){
                x_sum+=xdata.at(i).at(j);
                y_sum+=ydata.at(i).at(j);
            }

            double x_mean = x_sum / xdata.at(i).size();
            double y_mean = y_sum / ydata.at(i).size();


            vector<double> xi;
            vector<double> yi;
            vector <double> zi;


            for(long unsigned int j=0;j<xdata.at(i).size();j++){
                xi.push_back(xdata.at(i).at(j) - x_mean);
                yi.push_back(ydata.at(i).at(j) - y_mean);
                zi.push_back(xi.at(j)*xi.at(j) + yi.at(j)*yi.at(j));
                z_sum += zi.at(j);

            }

            double z_mean = z_sum / zi.size();

            arma::mat Z = arma::mat(zi.size(),4,arma::fill::zeros);

            for(long unsigned int j=0;j<zi.size();j++){
                Z(j,0) = zi.at(j);
                Z(j,1) = xi.at(j);
                Z(j,2) = yi.at(j);
                Z(j,3) = 1.0;

            }

            arma::mat M = arma::mat(4,4,arma::fill::zeros);
            arma::mat Zt = Z.t();
            M = (1/zi.size())*Zt*Z;

            arma::mat H = arma::mat(4,4,arma::fill::zeros);


            H(0,0) = 8*z_mean;
            H(0,3) = 2.0;
            H(3,0) = 2.0;
            H(1,1) = 1.0;
            H(2,2) = 1.0;

            arma::mat Hinv = arma::mat(4,4,arma::fill::zeros);
            Hinv(1,1) = 1.0;
            Hinv(2,2) = 1.0;
            Hinv(3,0) = 0.5;
            Hinv(0,3) = 0.5;
            Hinv(3,3) = -2*z_mean;  

            // arma::mat X(5, 5, arma::fill::randu);
            arma::mat U;
            arma::vec sigma; //Source (03/06): https://stackoverflow.com/questions/20112490/why-does-svd-result-of-armadillo-differ-from-numpy
            arma::mat V ;

            arma::svd(U,sigma,V,Z);
            double a=0.0,b=0.0,rsquare=0.0;

            if(sigma(3) > 1e-12){
                arma::mat Y = V*arma::diagmat(sigma)*V.t();
                arma::mat Q = Y*Hinv*Y;

                arma::vec eigval;
                arma::mat eigvec;

                arma::eig_sym(eigval,eigvec,Q);
                int j = eigval.n_elem - 1; //source (03/06): https://stackoverflow.com/questions/20149427/armadillo-how-to-grow-a-vector-and-get-his-size
                double pos_eig = eigval(j);
                int idx;


                while ( (eigval(j) > 1e-19) || (j>0) ){
                    
                    if((eigval(j)<pos_eig) && (eigval(j) > 0 )  ){
                        pos_eig = eigval(j);
                        idx = j;
                     }
                    
                    j--;
                }

                if((eigval(0)<pos_eig) && eigval(0)>0){
                    pos_eig = eigval(0);                    
                    idx = 0;
                }

                arma::mat A = Y.i()*eigvec.col(idx);
                a = -A(1)/(2*A(0));
                b = -A(2)/(2*A(0));
                rsquare = (A(1)*A(1)+A(2)*A(2) - 4*A(0)*A(3))/(4*A(0)*A(0));

            } else{
                arma::mat A = V.col(3);
                a = -A(1)/(2*A(0));
                b = -A(2)/(2*A(0));
                rsquare = (A(1)*A(1)+A(2)*A(2) - 4*A(0)*A(3))/(4*A(0)*A(0));
            }
            std::cout<<"index i:  " << i << "\r\n";
            std::cout<< "x y radius " << (a+x_mean) << " " << (b+y_mean) << " " << " " << (sqrt(rsquare)) << "\r\n"; 
            parameters.at(i).at(0) = (a+x_mean);
            parameters.at(i).at(1) = (b+y_mean);
            parameters.at(i).at(2) = (sqrt(rsquare));


        }

        return parameters;

    }



}