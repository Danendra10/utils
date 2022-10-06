#ifndef UTILS_H_
#define UTILS_H_

#include <stdint.h>
#include <vector>
#include <math.h>

#define DEG2RAD 0.017452925
#define RAD2DEG 57.295780

/**
 * @param [0]: x
 * @param [1]: y
 * @param [2]: th
 */
int16_t pos_robot[3];

/**
 * @param [0]: vx
 * @param [1]: vy
 * @param [2]: vth
 */
int16_t vel_robot[3];

std::vector<uint8_t> obs_on_field;
uint8_t total_obs;

//-----------------Obs------------------//
//=------------------------------------=//
    typedef struct
    {
        uint8_t status;
        float angle;
        float distance;
        float pos_x;
        float pos_y;

        struct
        {
            uint8_t status;
            float pos_x;
            float pos_y;
        } detection[60];
    } ObstacleDetection;

/* Prototypes */

ObstacleDetection ObstacleCheck(float theta, float theta_thresh, float dist);
float Pythagoras(float _x1, float _y1, float _x2, float _y2);

float RobotAngletoPoint(int16_t x, int16_t y);

#endif