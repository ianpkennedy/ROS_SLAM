# Turtlelib Library
A library for handling transformations in SE(2) and other turtlebot-related math.

# Components
- rigid2d - Handles 2D rigid body transformations
- diffdrive - Handles forward and inverse kinematics of a differential drive robot
- frame_main - Perform some rigid body computations based on user input

# Conceptual Questions
1. We need to be able to ~normalize~ Vector2D objects (i.e., find the unit vector in the direction of a given Vector2D):
   - Propose three different designs for implementing the ~normalize~ functionality
     
     One possible implementation is to define an external function to the Vector2D struct that is "pass by value", and returns a new Vector2D object to main.
     A second implementation possibility would be to create an external function that is "pass by reference", where we pass a pointer to the Vector2D object that we want to operate on. There is no return (it is void) since we operate by reference inside the function. A last implementation would be create a member function similar to the external "pass by value" case mentioned first. In this case, there is no argument passed (it operates on its own member variables), but instead the Vector2D returns a normalized version of itself to the main scope. 

   - Discuss the pros and cons of each proposed method, in light of the C++ Core Guidelines.
   
   In light of guideline F.15, and F.16, a pro of this method is that it is simple and easy to understand from a syntax perspective, however a con is that it does use more memory since we create a copy of the struct object
   Also in light of guidelines F.15, and F.16, a con is that the 2nd method is more complex from a syntax perspective and harder to understand, however a pro is that it is more memory efficient.

   - Which of the methods would you implement and why?
   
   I would implement a method function contained in the function. This encapsulates and prevents unintended access to the function (guideline C.9). It returns a copy of an object to main, which is acceptable in this instance of a relatively simple object.

2. What is the difference between a class and a struct in C++?

The difference between the two is that by default, the access and inheritance of a struct is public, while a class is private.

3. Why is Vector2D a struct and Transform2DClass (refer to at least 2 specific C++ core guidelines in your answer)?

Vector2D is a struct and because it has two data members (x and y) that can vary independently (guideline C.2). In addition, since the x,y member variables
are public, a struct is more appropriate since it simplifies syntax and is more readable since the member variables are public by default (guideline C.8).Transform2D is a class for multiple reasons. It has an invariant, the transform two dimensional vector, and so is a good convention to use (guideline C.2). Also, the class has a private variable, the two dimensional vector transform, and by convention it is thus useful to define Transform2D as a class (guideline C.8).


4. Why are some of the constructors in Transform2D explicit (refer to a specific C++ core guideline in your answer)? 

In accordance with guideline C.46, single argument constructors should have an explicit prefix so as to avoid accidental variable conversions when creating an object instance of a class

5. Why is Transform2D::inv() declared const while Transform2D::operator*=() is not?
   - Refer to [[https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#con-constants-and-immutability][C++ Core Guidelines (Constants and Immutability)]] in your answer

In accordance with Con.2, since Transform2D::inv() does not change the object's state (the transform 2d vector), but rather returns a new object, it should be declared const. On the other hand, Transform2D::operator*=() does change the objet's state by modifying the member vector variable, so it is not declared const

# Sample Run of frame_main
```
Enter transform T_{a,b}:  
deg: 90 x: 0 y: 1
Enter transform T_{b,c}:  
deg: 90 x: 1 y: 0
T_{a,b}: deg: 90 x: 0 y: 1
T_{b,a}: deg: -90 x: -1 y: -6.12323e-17
T_{b,c}: deg: 90 x: 1 y: 0
T_{c,b}: deg: -90 x: -6.12323e-17 y: 1
T_{a,c}: deg: 180 x: 6.12323e-17 y: 2
T_{c,a}: deg: -180 x: -1.83697e-16 y: 2
Enter v_b:
1 1
v_b: [1 1]
v_bhat: [0.707107 0.707107]
v_a: [-1 2]
v_b: [1 1]
v_c: [1 1.11022e-16]
Enter twist V_b:
1 1 1
V_a [1 0 1]
V_b [1 1 1]
V_c [1 2 -1]
```