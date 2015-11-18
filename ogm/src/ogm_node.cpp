#include "OccupancyGridMapping.hpp"


int main (int argc, char **argv) {

    // init the ros package
    ros::init(argc, argv, "ogm");

    OccupancyGridMapping ogm;

    ogm.run();

    return 0;


}
