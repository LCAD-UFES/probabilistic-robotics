#include "GridMap.hpp"
#include <cmath>
#include "wrap2pi.h"

// basic constructor
GridMap::GridMap(ros::NodeHandle &private_nh) : cells(nullptr), grid_mutex(), seq(0), M_PI2(M_PI/2) {

    // the frame id
    private_nh.param<std::string>("grid_map_frame_id", frame_id, "map");

    // the grid map dimensions
    // the width
    private_nh.param("grid_map_width", width, 4000);

    // update width2
    width2 = width/2;

    // the height
    private_nh.param("grid_map_height", height, 4000);

    // update the height2
    height2 = height/2;

    // updates the grid map array size
    size = width*height;

    // range max
    private_nh.param("grid_map_range_max", range_max, 15.0);

    // resolution
    private_nh.param("grid_map_resolution", resolution, 0.05);

    // update the limit displacement
    displacement = range_max/resolution;

    // the alpha2 parameter
    private_nh.param("grid_map_alpha_2", alpha2, resolution*0.75);

    // the beta2 parameter
    private_nh.param("grid_map_beta_2", beta2, 0.008726646259971648);

    // origin x
    private_nh.param("grid_map_origin_x", origin_x, 0.0);

    // origin y
    private_nh.param("grid_map_origin_y", origin_y, 0.0);

    // orientation
    private_nh.param("grid_map_orientation", orientation, 0.0);

    // allocate the memmory
    cells = new MapCell[size];
    if (nullptr == cells) {
        throw std::bad_alloc();
    }

    float div = 1.0/(float)width;

    for (int i = 0; i < size; i++) {
        // set the status to undefined
        cells[i].status = 0;

        // calculates the world coordinates to every cell
        cells[i].xi = (origin_x + (i%width - width2)*resolution);
        cells[i].yi = (origin_y + (i*div - height2)*resolution);

    }

}

// basic destructor
GridMap::~GridMap() {
    if (nullptr != cells) {
        delete cells;
    }
}

// update the grid map
void GridMap::updateGridMap(const geometry_msgs::Pose& pose, const sensor_msgs::LaserScanConstPtr& ls) {

    tf::Quaternion qt(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);

    tf::Matrix3x3 m(qt);
    double theta, p_roll, p_pitch;
    m.getRPY(p_roll, p_pitch, theta);

    int leftLimit, rigthLimit, downLimit, upLimit;

    int x_map = std::floor((pose.position.x - origin_x)/resolution + 0.5) + width2;
    int y_map = std::floor((pose.position.y - origin_y)/resolution + 0.5) + height2;

    leftLimit = x_map - displacement;
    if (0 > leftLimit) {
        leftLimit = 0;
    }

    rigthLimit = x_map + displacement;
    if (width < rigthLimit) {
        rigthLimit = width;
    }

    upLimit = y_map + displacement;
    if (height < upLimit) {
        upLimit = height;
    }

    downLimit = y_map - displacement;
    if (0 > downLimit) {
        downLimit = 0;
    }

    // crictical area
    grid_mutex.lock();

    for (int j = downLimit; j < upLimit; j++) {
        for (int i = leftLimit; i < rigthLimit; i++) {

            MapCell *cell = &cells[MAP_INDEX(i, j)];

            if (4 > abs(i - x_map) && 4 > abs(j - y_map) && 1 > cell->status) {

                cell->status = -100;

            } else {

                cell->dist = std::sqrt(std::pow(cell->xi - pose.position.x, 2) + std::pow(cell->yi - pose.position.y, 2));

                cell->angle = mrpt::math::angDistance(theta, mrpt::math::wrapToPi(atan2(cell->yi - pose.position.y, cell->xi - pose.position.x)));

                // verify the perceptual field
                if (-M_PI2 <= cell->angle && M_PI2 >= cell->angle && cell->dist <= range_max) {

                    inverse_sensor_model(cell, ls);

                }

            }

        }

    }

    // unlock the grid
    grid_mutex.unlock();

}

// the inverse sensor model
void GridMap::inverse_sensor_model(MapCell *cell, const sensor_msgs::LaserScanConstPtr ls) {

    double range = ls->ranges[0];
    double angle = fabs(mrpt::math::angDistance(cell->angle, ls->angle_min));

    // iterate over the laser beams to find the nearest one
    for (int i = 1; i < ls->ranges.size(); i++) {

        double tmp = fabs(mrpt::math::angDistance(cell->angle, ls->angle_min + (i * ls->angle_increment)));

        if (tmp < angle) {
            range = ls->ranges[i];
            angle = tmp;
        } else {
            // it can be stoped
            break;
        }
    }

    // verify the update cases
   if (cell->dist <= range + alpha2 && angle <= beta2 && range <= range_max) {

        if (fabs(cell->dist - range) <= alpha2) {

            if (100 > cell->status) {
                if (-20 < cell->status) {
                    cell->status += 10;
                } else {
                    cell->status += 5;
                }
            }

        } else if (cell->dist < range) {

            if (-100 < cell->status) {
                if (0 > cell->status) {
                    cell->status -= 10;
                } else {
                    cell->status -= 1;
                }
            }
        }

    }

}

// draw the map
void GridMap::exportGridMap(nav_msgs::OccupancyGrid &occupancy) {

    // lock the map
//     grid_mutex.lock();

    // set the sequence
    occupancy.header.seq = seq;

    // time stamp
    occupancy.header.stamp = ros::Time::now();

    // frame_id
    occupancy.header.frame_id = (frame_id);

    // the meta data
    occupancy.info.map_load_time = ros::Time::now();

    // resolution
    occupancy.info.resolution = resolution;

    // width
    occupancy.info.width = width;

    // height
    occupancy.info.height = height;

    // origin - position
    occupancy.info.origin.position.x = origin_x - width2*resolution;
    occupancy.info.origin.position.y = origin_y - height2*resolution;
    occupancy.info.origin.position.z = 0.0;

    // origin - orientation
    occupancy.info.origin.orientation.x = 0.0;
    occupancy.info.origin.orientation.y = 0.0;
    occupancy.info.origin.orientation.z = 0.0;
    occupancy.info.origin.orientation.w = 1.0;

    // copy the cells
    for (int i = 0; i < size; i++) {

        if (20 < cells[i].status) {

            occupancy.data[i] = 100;

        } else if (-20 > cells[i].status) {

            occupancy.data[i] = 0;

        } else {

            occupancy.data[i] = -1;

        }
    }

    // unlock the map
//     grid_mutex.unlock();

    return;
}

