#ifndef UTILS_H_
#define UTILS_H_

#include <stdint.h>
#include <vector>
#include <math.h>

#define DEG2RAD 0.017452925
#define RAD2DEG 57.295780

/**
 * @brief All data is being declared in utils
 * @param [0]: x
 * @param [1]: y
 * @param [2]: th
 */
float pos_robot[3];

/**
 * @param [0]: vx
 * @param [1]: vy
 * @param [2]: vth
 */
int16_t vel_robot[3];

/**
 * @param [0]: x
 * @param [1]: y
 * @param [2]: th
 * @param [3]: dist
 */
float ball_on_field[4];

/**
 * Ball Condition
 * @param 1 ball exist
 * @param 0 ball not exist
*/
uint8_t ball_status;

uint8_t line_sensor;

uint8_t ball_sensor[2];

std::vector<uint16_t> obs_on_field;
uint8_t total_obs;

//=----------------BS----------------=//
//=----------------------------------=//


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

//---Enumeration
//==============

enum robot_state
{
    //---General Cmd
    status_iddle = 83,   // S | 0x53
    status_iddle_2 = 32, // Space | 0x20
    status_start = 115,  // s | 0x73

    //---Home Cmd
    status_preparation_kickoff_home = 75,     // K | 0x4B
    status_preparation_freekick_home = 70,    // F | 0x46
    status_preparation_goalkick_home = 71,    // G | 0x47
    status_preparation_cornerkick_home = 67,  // C | 0x43
    status_preparation_penaltykick_home = 80, // P | 0x50
    status_preparation_throwin_home = 84,     // T | 0x54

    //---All Cmd
    status_preparation_dropball = 78, // N | 0x4E
    status_callibration = 35,         // # | 0x23
    status_park = 76,                 // L | 0x4C

    //---Away Cmd
    status_preparation_kickoff_away = 107,     // k | 0x6B
    status_preparation_freekick_away = 102,    // f | 0x66
    status_preparation_goalkick_away = 103,    // g | 0x67
    status_preparation_cornerkick_away = 99,   // c | 0x63
    status_preparation_penaltykick_away = 112, // p | 0x70
    status_preparation_throwin_away = 116,     // t | 0x74

    //---Keyboard Manual
    status_keyboard_maju = 106,        // j | 0x6A
    status_keyboard_kiri = 98,         // b | 0x62
    status_keyboard_mundur = 110,      // n | 0x6E
    status_keyboard_kanan = 109,       // m | 0x6D
    status_keyboard_rotasi_kanan = 48, // 0 | 0x30
    status_keyboard_rotasi_kiri = 57,  // 9 | 0x39

};

ObstacleDetection ObstacleCheck(float theta, float theta_thresh, uint16_t dist);
float Pythagoras(float _x1, float _y1, float _x2, float _y2);

float RobotAngletoPoint(int16_t x, int16_t y);

void SetLineSensor(uint8_t _line_sensor);
void SetBallSensor(std::vector<uint8_t> _ball_sensor);

bool LeftLineSensorDetected();
bool RightLineSensorDetected();

void DribbleAcceleration(int16_t *dribble, int16_t set_point, int16_t max_acceleration);

#endif