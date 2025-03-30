#ifndef FUSION_H
#define FUSION_H

#include <Arduino.h>
#include "main.h"

class fusion{
    public:
        fusion();
        void update(int cam_x, int cam_y, int front_lidar, int right_lidar, int back_lidar, int left_lidar);
};


#endif