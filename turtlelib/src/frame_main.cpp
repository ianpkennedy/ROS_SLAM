#include<iostream>
#include<sstream>
#include "turtlelib/rigid2d.hpp"
#include "turtlelib/diff_drive.hpp"

using std::cout;
using std::cin;

//Ian Kennedy

/// \file
/// \brief A script that computes some transforms, vectors, and twists using frames a, b, c

int main(){
    

    // Declare vectors and twists 
    turtlelib::Vector2D v_a;    
    turtlelib::Vector2D v_b;
    turtlelib::Vector2D v_c;    

    turtlelib::Twist2D V_a;
    turtlelib::Twist2D V_b;
    turtlelib::Twist2D V_c;

    turtlelib::Vector2D  v_bhat;


    // Declare and create transforms
    turtlelib::Transform2D t_ab;
    cout<<"Enter transform T_{a,b}:  \n";
    cin>>t_ab;

    turtlelib::Transform2D t_bc;
    cout<<"Enter transform T_{b,c}:  \n";

    cin.get();
    cin>>t_bc;

    cout<<"T_{a,b}: "<<t_ab;
    turtlelib::Transform2D t_ba = t_ab.inv();
    cout<<"T_{b,a}: "<<t_ba;
    cout<<"T_{b,c}: "<<t_bc;

    turtlelib::Transform2D t_cb = t_bc.inv();
    cout<<"T_{c,b}: "<<t_cb;

    turtlelib::Transform2D t_ac = t_ab;
    t_ac*=t_bc;

    cout<<"T_{a,c}: "<<t_ac;
    turtlelib::Transform2D t_ca = t_ac.inv();
    cout<<"T_{c,a}: "<<t_ca;

    //Enter v_b
    cout<<"Enter v_b:\n";   
    cin>>v_b;

    //Compute vectors
    cout<<"v_b: "<<v_b<<"\n";
    v_bhat = v_b.normalize();
    cout<<"v_bhat: "<<v_bhat << "\n";

    v_a = t_ab(v_b);
    cout<<"v_a: "<<v_a<<"\n";
    cout<<"v_b: "<<v_b<<"\n";
    v_c = t_cb(v_b);
    cout<<"v_c: "<<v_c<<"\n";

    //Compute twists
    cout<<"Enter twist V_b:\n";
    cin>>V_b;
    V_c = t_cb(V_b);
    V_a = t_ab(V_b);

    cout<<"V_a "<<V_a<<"\n";
    cout<<"V_b "<<V_b<<"\n";
    cout<<"V_c "<<V_c<<"\n";
   


    return 0;
}