#ifndef OCCUPANCY_GRID_MAPPING_H
#define OCCUPANCY_GRID_MAPPING_H

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>

#include "GridMap.hpp"

class OccupancyGridMapping {

    private:

        // our handlers
        // main handler
        ros::NodeHandle nh;

        // private node handler
        ros::NodeHandle private_nh;

        // the odometry subscriber
        message_filters::Subscriber<nav_msgs::Odometry> odom_sub;

        // the laser scan subscriber
        message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub;

        // ou sync policy
        typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::LaserScan> MySyncPolicy;

        // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
        message_filters::Synchronizer<MySyncPolicy> sync;

        // the grid map
        GridMap gm;

        // loop rate
        ros::Rate loop_rate;

        // the publisher
        ros::Publisher map_pub;

        // mininmum timestamp diff
        ros::Duration max_ol_translational_time;
        double max_ol_angular_time_factor;

        // the OccupancyGrid message to be published
        nav_msgs::OccupancyGrid occupancy;

        // the time factor
        ros::Duration timeFactor(ros::Duration&, double);

    public:
        // basic constructor
        OccupancyGridMapping();

        // update the grid
        void update(const nav_msgs::OdometryConstPtr&, const sensor_msgs::LaserScanConstPtr&);

        // the run method
        void run();

        // post the grid map
        void publishOccupancyGridMapping();

};

#endif
