#ifndef GRID_MAP_H
#define GRID_MAP_H

#include <mutex>
#include <string>
#include <cmath>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_datatypes.h>

#include "MapCell.hpp"

class GridMap {

    private:

        // the frame id
        std::string frame_id;

        // the grid width, heigth and size
        int width, height, size;

        // auxiliar width2, height2
        int width2, height2;

        // the grid resolution
        double resolution, inverse_resolution;

        // the grid origin
        double origin_x, origin_y, orientation;

        // the grid Cells
        MapCell *cells;

        // our internal laser range max
        double range_max;

        // the limit displacement
        int displacement;

        // the alpha2 and beta2 parameters
        double alpha2, beta2;

        // sequence
        int seq;
        // the grid mutex
        std::mutex grid_mutex;

        // the inverse sensor model
        void inverse_sensor_model(MapCell *, const sensor_msgs::LaserScanConstPtr);


        // the angle diff
        double angle_diff(double, double);

        // pi over 2
        double M_PI2;

    public:

        // basic constructor
        GridMap(ros::NodeHandle&);

        // basic destructor
        ~GridMap();

        // the main method to update de grid map
        void updateGridMap(const geometry_msgs::Pose&, const sensor_msgs::LaserScanConstPtr&);

        // draw the map
        void exportGridMap(nav_msgs::OccupancyGrid&);

};

// i is the collum and j is the row :-/
#define MAP_INDEX(i, j) ((i) + (j)*width)

#endif
