#ifndef LIDARML_INCLUDE_GUARD_HPP
#define LIDARML_INCLUDE_GUARD_HPP
/// \file
/// \brief Apply machine learning to detect circles
#include<vector>


using std::vector;

namespace lidar{



    /// \brief a cluster laser points and learn circles
    class Lidar2D
    {
    public:    
        /// \brief construct a lidar clustering ML object for circle detection
        /// \param num_pts - resolution of lidar sensor per scan
        /// \param d - threshold distance for clustering points
        Lidar2D(int num_pts, double d);


        /// \brief cluster points from lidar
        /// \param num_pts - resolution of lidar sensor per scan
        void cluster(std::vector<float> cloud);

        /// \brief clear cluster vectors
        /// \param None
        void clear_cluster();

        /// \brief clear cluster vectors
        /// \param None
        void print_cluster();


        /// \brief return cluster x pts
        /// \param None
        /// \return x cluster data points
        vector<vector<double>> return_x_cluster();

        /// \brief return cluster y pts
        /// \param None
        /// \return y cluster data points
        vector<vector<double>> return_y_cluster();

        /// \brief identify circle
        /// \param xdata - vector of vector doubles - xdata
        /// \param ydata - vector of vector doubles - ydata        
        /// \return x,y,radius parameters in meters (robot frame)
        vector<vector<double>> circle(vector<vector<double>> xdata, vector<vector<double>> ydata);


        /// \brief classify clusters into circle/non-circles
        /// \param None
        /// \returns None
        vector<bool> classify(vector<vector<double>> xdata, vector<vector<double>> ydata);

    private:
        int resolution;     
        double cluster_threshold;
        std::vector<std::vector<double>> x_cluster;
        std::vector<std::vector<double>> y_cluster;
        std::vector<std::vector<int>> idx;
        std::vector<bool> is_circle;

    };


}


#endif
