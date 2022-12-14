#ifndef RIGID2D_INCLUDE_GUARD_HPP
#define RIGID2D_INCLUDE_GUARD_HPP
/// \file
/// \brief Two-dimensional rigid body transformations.


#include<iosfwd> // contains forward definitions for iostream objects
#include<cmath>  // import for math helper commands
#include<vector>

namespace turtlelib
{

    /// \brief PI.  Not in C++ standard until C++20.
    constexpr double PI=3.14159265358979323846;

    /// \brief approximately compare two floating-point numbers using
    ///        an absolute comparison
    /// \param d1 - a number to compare
    /// \param d2 - a second number to compare
    /// \param epsilon - absolute threshold required for equality
    /// \return true if abs(d1 - d2) < epsilon
    /// NOTE: implement this in the header file
    /// constexpr means that the function can be computed at compile time
    /// if given a compile-time constant as input
    constexpr bool almost_equal(double d1, double d2, double epsilon=1.0e-12)
    {   
        double compare = fabs(d1-d2);

        if (compare<epsilon){
            return true;
        } else{
            return false;
        }
    }

    /// \brief convert degrees to radians
    /// \param deg - angle in degrees
    /// \returns radians
    constexpr double deg2rad(double deg)
    {
        return deg*PI/180.0;   

    }

    /// \brief convert radians to degrees
    /// \param rad - angle in radians
    /// \returns the angle in degrees
    constexpr double rad2deg(double rad)
    {
            return rad*180.0/PI;   

    
    }

    /// static_assertions test compile time assumptions.
    /// You should write at least one more test for each function
    /// You should also purposely (and temporarily) make one of these tests fail
    /// just to see what happens
    static_assert(almost_equal(0, 0), "is_zero failed");
    static_assert(almost_equal(23,23),"is 23 failed");

    static_assert(almost_equal(deg2rad(0.0), 0.0), "deg2rad failed");
    static_assert(almost_equal(deg2rad(180), PI), "deg2rad failed");


    static_assert(almost_equal(rad2deg(0.0), 0.0), "rad2deg) failed");
    static_assert(almost_equal(rad2deg(PI), 180.0), "rad2deg) failed");


    static_assert(almost_equal(deg2rad(rad2deg(2.1)), 2.1), "deg2rad failed");
    static_assert(almost_equal(deg2rad(rad2deg(7.8)), 7.8), "deg2rad failed");

    /// \brief A 2-Dimensional Vector
    struct Vector2D
    {
        /// \brief the x coordinate
        double x = 0.0;

        /// \brief the y coordinate   
        double y = 0.0;

        /// \brief normalize the Vector2D object
        /// \return a new Vector2D object that is normalized
        Vector2D normalize();

        /// \brief return vector multiplied by scalar
        /// \param rhs - the scalar to multiply
        /// \return a reference to the new Vector2D object
        Vector2D & operator*=(const double & rhs);

        /// \brief return the sum of two vectors
        /// \param rhs - the vector to add
        /// \return a reference to the new Vector2D object
        Vector2D & operator+=(const Vector2D & rhs);   

        /// \brief return the difference of two vectors
        /// \param rhs - the vector to add
        /// \return a reference to the new Vector2D object
        Vector2D & operator-=(const Vector2D & rhs);        
        
    };
    /// \brief return vector multiplied by a scalar
    /// \param rhs - the scalar to multiply
    /// \return a reference to the new Vector2D object
    Vector2D  operator*(Vector2D lhs, const double & rhs);

    /// \brief return vector multiplied by a scalar
    /// \param gain - the scalar to multiply
    /// \param rhs - the vector to multiply
    /// \return a reference to the new Vector2D object
    Vector2D & operator*=(double gain, Vector2D & rhs);


    /// \brief return vector multiplied by a scalar
    /// \param lhs - the scalar to multiply
    /// \param rhs - the vector to multiply and return
    /// \return a reference to the new Vector2D object
    Vector2D operator*(const double & lhs, Vector2D rhs);

    /// \brief return the sum of two vectors
    /// \param lhs - the first vector to add
    /// \param rhs - the second vector to add
    /// \return a reference to the new Vector2D object
    Vector2D operator+(Vector2D lhs,  const Vector2D rhs);

    /// \brief return the difference of two vectors
    /// \param lhs - the first vector 
    /// \param rhs - the second vector to subtract
    /// \return a reference to the new Vector2D object
    Vector2D operator-(Vector2D lhs,  const Vector2D rhs);

    /// \brief output a 2 dimensional vector as [xcomponent ycomponent]
    /// os - stream to output to
    /// v - the vector to print
    std::ostream & operator<<(std::ostream & os, const Vector2D & v);

    /// \brief input a 2 dimensional vector
    ///   You should be able to read vectors entered as follows:
    ///   [x y] or x y
    /// is - stream from which to read
    /// v [out] - output vector
    /// Hint: The following may be useful:
    /// https://en.cppreference.com/w/cpp/io/basic_istream/peek
    /// https://en.cppreference.com/w/cpp/io/basic_istream/get
    ///
    /// The way input works is (more or less): what the user types is stored in a buffer until the user enters
    /// a newline (by pressing enter).  The iostream methods then process the data in this buffer character by character.
    /// Typically, each character is examined and then removed from the buffer automatically.
    /// If the characters don't match what is expected (e.g., we are expecting an int but the letter 'q' is encountered)
    /// an error flag is set on the stream object (e.g., std::cin).
    ///
    /// We have lower level control however. For example:
    /// peek looks at the next unprocessed character in the buffer without removing it
    /// get removes the next unprocessed character from the buffer.
    std::istream & operator>>(std::istream & is, Vector2D & v);



    /// \brief 3 position twist vector: [theta_dot x_dot y_dot]
    struct Twist2D 
    {   //source(11/12): https://stackoverflow.com/questions/2133250/x-does-not-name-a-type-error-in-c/2133260
        std::vector<double> tw = {0,0,0};
    };

    /// \brief output a 2 dimensional twist vector as [theta_dot x_dot y_dot]   
    /// os - stream to output to
    /// t - the vector to print
    std::ostream & operator<<(std::ostream & os, const Twist2D & twist);
    
    
    /// \brief input a 2 dimensional vector as theta_dot x_dot y_dot
    std::istream & operator>>(std::istream & is, Twist2D & t);


    /// \brief a rigid body transformation in 2 dimensions
    class Transform2D
    {

    public:
        /// \brief Create an identity transformation
        Transform2D();

        /// \brief create a transformation that is a pure translation
        /// \param trans - the vector by which to translate
        explicit Transform2D(Vector2D trans);

        /// \brief create a pure rotation
        /// \param radians - angle of the rotation, in radians
        explicit Transform2D(double radians);

        /// \brief Create a transformation with a translational and rotational
        /// component
        /// \param trans - the translation
        /// \param radians - the rotation, in radians
        Transform2D(Vector2D trans, double radians);

        /// \brief apply a transformation to a Vector2D
        /// \param v - the vector to transform
        /// \return a vector in the new coordinate system
        Vector2D operator()(Vector2D v) const;


        /// \brief invert the transformation
        /// \return the inverse transformation. 
        Transform2D inv() const;

        /// \brief transform twist
        /// \return a twist in the new frame. 
        Twist2D operator()(Twist2D twist) const;

        /// \brief compose this transform with another and store the result 
        /// in this object
        /// \param rhs - the first transform to apply
        /// \return a reference to the newly transformed operator
        Transform2D & operator*=(const Transform2D & rhs);

        /// \brief the translational component of the transform
        /// \return the x,y translation
        Vector2D translation() const;

        /// \brief get the angular displacement of the transform
        /// \return the angular displacement, in radians
        double rotation() const;

        /// \brief \see operator<<(...) (declared outside this class)
        /// for a description
        friend std::ostream & operator<<(std::ostream & os, const Transform2D & tf);

    private:
        // source (11/10): https://www.geeksforgeeks.org/2d-vector-in-cpp-with-user-defined-size/      
        std::vector<std::vector<double>> t;

    };

    /// \brief should print a human readable version of the transform:
    /// An example output:
    /// deg: 90 x: 3 y: 5
    /// \param os - an output stream
    /// \param tf - the transform to print
    std::ostream & operator<<(std::ostream & os, const Transform2D & tf);

    /// \brief Read a transformation from stdin
    /// Should be able to read input either as output by operator<< or
    /// as 3 numbers (degrees, dx, dy) separated by spaces or newlines
    std::istream & operator>>(std::istream & is, Transform2D & tf);

    /// \brief multiply two transforms together, returning their composition
    /// \param lhs - the left hand operand
    /// \param rhs - the right hand operand
    /// \return the composition of the two transforms
    /// HINT: This function should be implemented in terms of *=
    Transform2D operator*(Transform2D lhs, const Transform2D & rhs);

    /// \brief should print a human readable version of the twist:
    /// An example output:
    /// [1 2 3]
    /// \param os - an output stream
    /// \param tf - the transform to print
    std::ostream & operator<<(std::ostream & os, const Transform2D & tf);
    
    /// \brief normalize an angle
    /// \param rad - angle in radians
    /// \return the normalized angle in radians
    double normalize_angle(double rad);

    /// \brief calculate the magnitude of a vector
    /// \param v - vector to calculate magnitude of
    /// \return the magnitude of the vector
    double magnitude(Vector2D v);

    /// \brief calculate an angle
    /// \param v1 - 1st vector to compute angle with
    /// \param v1 - 2nd vector to compute angle with
    /// \return the angle between the vectors in radians
    double angle(Vector2D v1, Vector2D v2);

    /// \brief integrate a twist over a unit time step
    /// \param twist - twist to integrate
    /// \return the integration transform per unit time
    Transform2D integrate_twist(Twist2D twist); 

    /// \brief calculate dot product of two vectors
    /// \param v1 - 1st vector to calculate dot product with
    /// \param v2 - 2nd vector to calculate dot product with
    /// \return the integration transform per unit time
    double dot(Vector2D v1, Vector2D v2);
}

#endif
