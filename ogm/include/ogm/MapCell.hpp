#ifndef MAP_CELL_H
#define MAP_CELL_H

struct MapCell {

    // the status
    // 0 == free, 1 == occupied, -1 == unknow
    int status;

    // the world coordinates  center of mass
    float xi;
    float yi;

    // the distance from a given point
    float dist;

    // the angle between (xi, yi) and a given point
    float angle;

};

#endif
