//
// Created by bingbing on 3/1/22.
//

#ifndef ROBOT_COMMON_DIAGNOSITICS_H
#define ROBOT_COMMON_DIAGNOSITICS_H
#include <cstdio>
#include <cstdlib>

enum class RobotStatus
{
    UNINIT,
    INIT,
    RUNNING,
    SHUTDOWN,
    ERROR
};
enum class BatteryStatus
{
    CHARGING,
    FULL,
    HALF,
    LOW,
    CRITICAL_LOW
};


#endif //ROBOT_COMMON_DIAGNOSITICS_H
