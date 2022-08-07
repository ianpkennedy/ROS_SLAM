#include <iostream>
#include "turtlelib/rigid2d.hpp"
#include <cmath>
#include <sstream>
using namespace std;

/// \file
/// \brief Implementation file for rigid2d library

namespace turtlelib
{
    std::ostream & operator<<(std::ostream & os, const Vector2D & v){
        //code within this method adapted from this source (11/09): https://www.reddit.com/r/cpp_questions/comments/gld2oq/no_match_for_operator_operand_types_are/ https://stackoverflow.com/questions/46328422/c-ostream-operator-overload-odd-undefined-reference-when-building/46328665 https://docs.microsoft.com/en-us/cpp/standard-library/overloading-the-output-operator-for-your-own-classes?view=msvc-170 https://www.geeksforgeeks.org/overloading-stream-insertion-operators-c/

        //Output in expected format
        os << "[" << v.x << " " << v.y << "]";

        return os;
    }


    std::istream & operator>>(std::istream & is, Vector2D & v){
        //code within this method adapted from this source (11/11): https://en.cppreference.com/w/cpp/io/basic_istream/get https://en.cppreference.com/w/cpp/io/basic_istream/peek https://www.tutorialspoint.com/cplusplus/input_output_operators_overloading.htm
        
        //Check for input format
        if(is.peek()=='['){
            is.get();
            
            is >> v.x >> v.y;
            
            return is;

        }else {
        
        is >> v.x >> v.y;
        return is;
        }
    }

    Transform2D::Transform2D(){
        // code within this method adapted from this source (11/10): https://stackoverflow.com/questions/36815643/c-error-invalid-use-of-applefarmerapplefarmer-when-ca

        //Create identity 2D matrix
        t = {
            {1.0,0.0,0.0},
            {0.0,1.0,0.0},
            {0.0,0.0,1.0}
        };
    
    }

    Transform2D::Transform2D(double radians) {
        
        // code within this method adapted from this source (11/10): https://www.tutorialspoint.com/c_standard_library/math_h.htm
        
        //Input angle
        t = {
            {cos(radians),-sin(radians),0.0},
            {sin(radians),cos(radians),0.0},
            {0.0,0.0,1.0}
        };

    }

    Transform2D::Transform2D(Vector2D trans){
        
        //code within this method adapted from this source (11/10): https://www.tutorialspoint.com/c_standard_library/math_h.htm
        
        //input angle and translation
        t = {
            {cos(0.0),-sin(0.0),trans.x},
            {sin(0.0),cos(0.0),trans.y},
            {0.0,0.0,1.0}
        };


    }

    Transform2D::Transform2D(Vector2D trans, double radians){
        
        //code within this method adapted from this source (11/10): https://www.tutorialspoint.com/c_standard_library/math_h.htm

        //Input angle and translation
        t = {
            {cos(radians),-sin(radians),trans.x},
            {sin(radians),cos(radians),trans.y},
            {0.0,0.0,1.0}
        };
    }

    Vector2D Transform2D::operator()(Vector2D v) const{
        //code in this method adapted from this source(11/10): http://msl.cs.uiuc.edu/~lavalle/cs497_2001/book/geom/node9.html
        
        Vector2D v_new;
        //Compute and return new vector
        v_new.x = v.x*t[0][0] + v.y*t[0][1] + t[0][2];
        v_new.y = v.x*t[1][0] + v.y*t[1][1] + t[1][2];

        return v_new;

    }

    Twist2D Transform2D::operator()(Twist2D twist) const{
        
        
        Twist2D t_new;
        
        // Extract translation and rotation
        double ang = rotation();
        Vector2D vec = translation();
        
        //Compute new parameters and return
        t_new.tw[0] = twist.tw[0];
        t_new.tw[1] = vec.y*twist.tw[0] + twist.tw[1]*cos(ang) - sin(ang)*twist.tw[2];
        t_new.tw[2] = -vec.x*twist.tw[0] + twist.tw[1]*sin(ang) + twist.tw[2]*cos(ang); 

        return t_new;

    }

    Transform2D & Transform2D::operator*=(const Transform2D & rhs){
        // code within this method adapted from this source for this operator overload implementation (11/10): https://stackoverflow.com/questions/25898434/overload-operator-for-matrices-c

        
        // This operator overload function creates a temporary variable copy for the lefthand side matrix. 
        // Then the matrix multiplication is performed on the two transforms, and the lhs transform is updated, and return into the higher scope of the program
        
        int i=0;
        int j=0;
        int k=0;


        vector<vector<double>> temp = {
            {1.0,0.0,0.0},
            {0.0,1.0,0.0},
            {0.0,0.0,1.0}
        };

        for(i=0;i<3;i++){
            for(j=0;j<3;j++){
                temp[i][j] = t[i][j]; //temp is used as placeholder for the operation
                t[i][j] = 0;
            }    
        }

        for(i=0;i<3;i++){
            for(j=0;j<3;j++){
                for(k=0;k<3;k++){
                    t[i][j] = t[i][j] + temp[i][k]*rhs.t[k][j];
                }
            }    
        }

        return *this;
    }


    Transform2D Transform2D::inv() const{
        // The inverse of the homogeneous transform is computed with this function. This is referenced here: https://nu-msr.github.io/navigation_site/lectures/rigid2d.html
        //A new copy of the inverted transform object is returned to the higher program scope. 
        
        Transform2D t_inv; 
        t_inv.t[0][0] = t[0][0];
        t_inv.t[1][1] = t[1][1];
        t_inv.t[0][1] = t[1][0];
        t_inv.t[1][0] = t[0][1];
        
        t_inv.t[0][2] = -(t[0][0]*t[0][2] + t[1][0]*t[1][2]);
        t_inv.t[1][2] = -(t[1][1]*t[1][2] + t[0][1]*t[0][2] );
        
        t_inv.t[2][0] = 0;
        t_inv.t[2][1] = 0;
        t_inv.t[2][2] = 1;

        return t_inv;
    }

    Vector2D Transform2D::translation() const{

        Vector2D v_tr;

        //Assign values to vector object
        v_tr.x = t[0][2];
        v_tr.y = t[1][2];

        return v_tr;
    }

    double Transform2D::rotation() const{
        double angle;

        //Returns an angle in radians
        angle = atan2(t[1][0],t[0][0]);

        return angle;
    }

    std::ostream & operator<<(std::ostream & os, const Transform2D & tf){
        //code within this method adapted from this source (11/09): https://www.reddit.com/r/cpp_questions/comments/gld2oq/no_match_for_operator_operand_types_are/ https://stackoverflow.com/questions/46328422/c-ostream-operator-overload-odd-undefined-reference-when-building/46328665 https://docs.microsoft.com/en-us/cpp/standard-library/overloading-the-output-operator-for-your-own-classes?view=msvc-170 https://www.geeksforgeeks.org/overloading-stream-insertion-operators-c/
        
        
        //Ouput parameter information in expected format
        Vector2D v = tf.translation();
        os << "deg: " << rad2deg(tf.rotation()) << " x: " << v.x << " y: "<< v.y << "\n";

        return os;
    }


    std::istream & operator>>(std::istream & is, Transform2D & tf){
       
        double angle;
        double num;
        Vector2D v;
        int i=0;
        stringstream line;
        string temp;

        // Filter out entered numbers and store them in the Transform2D object
        if(is.peek() == 'd'){
            
            while( i<3){ //source (11/12): https://www.geeksforgeeks.org/extract-integers-string-c/

                is>>temp;
                if(stringstream(temp)>>num){
                    if(i==0){
                        angle = num;
                    }
                    if(i==1){
                        v.x = num;
                    }
                    if(i==2){
                        v.y = num;
                    }
                    i++;
                }
            } //source end

            tf = Transform2D(v, deg2rad(angle));

        }else{ // Plain numbers
            is>>angle >> v.x >> v.y;
            tf = Transform2D(v, deg2rad(angle));
        }

        return is;
        
    }
       
       
    Transform2D operator*(Transform2D lhs, const Transform2D & rhs){
        //code within this method adapted from this source(11/11): https://en.cppreference.com/w/cpp/language/operators
       
       //Store multiplied matrix and return
        lhs*=rhs;
        return lhs;

    }

    Vector2D operator+(Vector2D lhs,  const Vector2D rhs){

        lhs+=rhs;
        return lhs; 
    }


    std::ostream & operator<<(std::ostream & os, const Twist2D & twist){
        
        //output twist parameters
        cout<<"["<<twist.tw[0]<<" "<<twist.tw[1]<<" "<<twist.tw[2]<<"]";
        return os;
    
    }



    std::istream & operator>>(std::istream & is, Twist2D & t){


        //Input twist parameters
        is >> t.tw[0] >> t.tw[1] >> t.tw[2];
        return is;
        
    }  
       
       
    Vector2D Vector2D::normalize(){
        
        Vector2D v;
        
        //Compute and return normalized Vector2D object
        v.x = sqrt((x*x)/(x*x+y*y));
        v.y = sqrt((y*y)/(x*x+y*y));
        
        return v;

    }

    Vector2D & Vector2D::operator*=(const double & rhs){
         
         x = rhs*x;
         y = rhs*y;

        return *this;
    }

    Vector2D  operator*(Vector2D lhs, const double & rhs){
         
        lhs*=rhs;

        return lhs;
    }

    Vector2D  operator*(const double & lhs, Vector2D rhs){

        rhs*=lhs;

        return rhs;
    }


    Vector2D & operator*=(double gain,  Vector2D & rhs){
         
        rhs.x = rhs.x*gain;
        rhs.y = rhs.y*gain;

        return rhs;
    }

    Vector2D & Vector2D::operator+=(const Vector2D & rhs){
         
        x = rhs.x+x;
        y = rhs.y+y;

        return *this;
    }



    Vector2D & Vector2D::operator-=(const Vector2D & rhs){
         
        x = -rhs.x+x;
        y = -rhs.y+y;

        return *this;
    }




    Vector2D operator-(Vector2D lhs,  const Vector2D rhs){

        lhs-=rhs;
        return lhs; 
    }


    double normalize_angle(double rad){

       return remainder(rad,2*PI); //source (11/24): https://www.programiz.com/cpp-programming/library-function/cmath/remainder
  
    }

    double magnitude(Vector2D v){
        
        //Euclidian distance formula
        return sqrt(v.x*v.x+v.y*v.y);

    }

    double angle(Vector2D v1, Vector2D v2){

        //Use dot product of two vectors
        double left = v1.x*v2.x+v1.y*v2.y;
        double right = magnitude(v1)*magnitude(v2);

        double ang = acos(left/right);

        return ang;
    }

    double dot(Vector2D v1, Vector2D v2){

        return (v1.x*v2.x+v1.y*v2.y);
    }

    Transform2D integrate_twist(Twist2D twist){

        Transform2D T_bbpr;

        if (almost_equal(twist.tw[0],0.0,1.0e-6)){

            turtlelib::Vector2D v;
            v.x = twist.tw[1];
            v.y = twist.tw[2];

            T_bbpr = Transform2D(v);
        } else{
        //Calculate the xs, ys translation and build the transform
        double xs = (twist.tw[2])/twist.tw[0];
        double ys = -(twist.tw[1])/(twist.tw[0]);

        turtlelib::Vector2D v;
        v.x = xs;
        v.y = ys;
        
        // cout<<"twist: " << twist<<"\n";
        // cout<<"vector: " << v <<"\n";

        Transform2D T_sb;
        Transform2D T_sspr;
        Transform2D T_bs;
        Transform2D T_sprbpr;

        T_sb = Transform2D(v);

        T_sspr = Transform2D(twist.tw[0]);
        T_bs = T_sb.inv();
        T_sprbpr = T_sb;
        
        // cout <<"before: "<<"\n";

        // cout<<"T_sspr: " << T_sspr << "\n";
        // cout<<"T_sprbpr: " << T_sprbpr << "\n";
        // cout<<"T_bs: " << T_bs << "\n";

        // cout<<"T_bbpr: " << T_bbpr << "\n";
        // cout <<"after: "<<"\n";

        //Perform the transform integration
        T_bbpr = T_bs*T_sspr*T_sprbpr;
        
        // cout<<"T_sspr: " << T_sspr << "\n";
        // cout<<"T_sprbpr: " << T_sprbpr << "\n";
        // cout<<"T_bs: " << T_bs << "\n";
        // cout<<"T_bbpr: " << T_bbpr << "\n";
        }

        return T_bbpr;
    }




} 
