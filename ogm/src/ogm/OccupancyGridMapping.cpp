#include "OccupancyGridMapping.hpp"

OccupancyGridMapping::OccupancyGridMapping() :
        nh(),
        private_nh("~"),
        gm(private_nh),
        loop_rate(20),
        odom_sub(nh, "odom", 40),
        laser_sub(nh, "scan", 10),
        sync(MySyncPolicy(100), odom_sub, laser_sub)
{

    // the max laser and odometry diff time
    double tmp;
    private_nh.param("max_diff_odom_laser_diff_time", tmp, 0.5);
    max_ol_translational_time = ros::Duration(tmp);

    // max angular time
    private_nh.param("max_ol_angular_time_factor", max_ol_angular_time_factor, 0.01);

    std::string map_topic;
    private_nh.param<std::string>("map_publishing_topic", map_topic, "map");

    // set the map_pub, the correct topic and messages
    map_pub = nh.advertise<nav_msgs::OccupancyGrid>(map_topic, 1);

    // Occupancy data
    occupancy.data.resize(4000*4000);

    // register the callback function to the current sync policy
    sync.registerCallback(boost::bind(&OccupancyGridMapping::update, this, _1, _2));

    std::cout << "Built" << std::endl;

}

// our main callback
void OccupancyGridMapping::update(const nav_msgs::OdometryConstPtr& odom, const sensor_msgs::LaserScanConstPtr& ls) {

    // verify the difference between the Odometry and the LaserScan
    if (odom->twist.twist.angular.z > 0.01) {
        if ((odom->header.stamp - ls->header.stamp) <= ros::Duration(max_ol_angular_time_factor/((int) 10*odom->twist.twist.angular.z))) {
            gm.updateGridMap(odom->pose.pose, ls);
        }
    } else {
        if ((odom->header.stamp - ls->header.stamp) <= max_ol_translational_time) {
            gm.updateGridMap(odom->pose.pose, ls);
        }
    }
}

// post the map
void OccupancyGridMapping::publishOccupancyGridMapping() {

    gm.exportGridMap(occupancy);

    map_pub.publish<nav_msgs::OccupancyGrid>(occupancy);

    return;
}

// the run method, just to spin around
void OccupancyGridMapping::run() {

    // the spinner object
    ros::AsyncSpinner spinner(1);

    // spin!
    spinner.start();

    while(ros::ok()) {

        // publish the pose array
        publishOccupancyGridMapping();

        // sleep
        loop_rate.sleep();

    }

}
