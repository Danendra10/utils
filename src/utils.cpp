#include "utils/utils.h"
#include <iostream>

float Pythagoras(float _x1, float _y1, float _x2, float _y2) { return sqrt(pow(_x2 - _x1, 2) + pow(_y2 - _y1, 2)); }

float RobotAngletoPoint(int16_t x, int16_t y) { return atan2(y - pos_robot[1], x - pos_robot[0]) * RAD2DEG; }

//=------------------------------------=//
//--------------Sensors-----------------//
//=------------------------------------=//

void SetLineSensor(uint8_t _line_sensor) { line_sensor = _line_sensor; }

bool LeftLineSensorDetected()
{
    // printf("Line sensor: %d\n", line_sensor);
    if ((line_sensor & 0x02) != 0x02)
        return true;
    else
        return false;
}
bool RightLineSensorDetected()
{
    if ((line_sensor & 0x01) != 0x01)
        return true;
    else
        return false;
}

void SetBallSensor(std::vector<uint8_t> _ball_sensor)
{
    ball_sensor[0] = _ball_sensor[0];
    ball_sensor[1] = _ball_sensor[1];
}

//=------------------------------------=//
//--------------Hardware----------------//
//=------------------------------------=//
void DribbleAcceleration(int16_t *dribble, int16_t set_point, int16_t max_acceleration)
{
    if (*dribble < set_point)
    {
        *dribble += max_acceleration;
        if (*dribble > set_point)
            *dribble = set_point;
    }
    else if (*dribble > set_point)
    {
        *dribble -= max_acceleration;
        if (*dribble < set_point)
            *dribble = set_point;
    }
}

//-----------Obstacle Avoidance-----------//
ObstacleDetection ObstacleCheck(float theta, float theta_thresh, uint16_t dist)
{
    ObstacleDetection obs_data;
    // for(int i = 0; i < obs_on_field.size(); i++)
    //     printf("%d\n", obs_on_field[i]);

    // printf("pos robot: %f %f %f\n", pos_robot[0], pos_robot[1], pos_robot[2]);

    static uint8_t obs_counter;
    static uint8_t obs_start;
    static uint8_t obs_final;

    obs_data.status = 0;
    obs_data.distance = dist;

    // Set every 60 iteration is not an obstacle for init
    for (uint8_t i = 0; i < 60; i++)
        obs_data.detection[i].status = 0;

    uint16_t init_index = (theta - theta_thresh) * 0.166666667;
    uint16_t final_index = (theta + theta_thresh) * 0.166666667;

    while (init_index < 0)
        init_index += 60;
    while (init_index > 59)
        init_index -= 60;

    while (final_index < 0)
        final_index += 60;
    while (final_index > 59)
        final_index -= 60;
    // printf("init index: %d final index: %d\n", init_index, final_index);

    std::vector<float> obs_dist;
    std::vector<uint8_t> obs_index;

    // std::cout << obs_on_field.at(1);
    // printf("%d\n", i);

    // for(int i = init_index; i <= final_index; i++)
    // {
    //     printf("%d\n", i);
    // }

    if (init_index < final_index)
    {
        printf("Mlevu kene\n");
        obs_counter = 0;
        for (uint16_t i = init_index; i <= final_index; i++)
        {
            /* Check if the obstacle is in the range of our dist thresh */
            /**
             * Got some error in this line of code, 
             * TODO: try to use try and catch to debug 
            */
            if (obs_on_field[i] > dist && i != final_index)
            {
                printf("helo");
                obs_data.status = 1;
                obs_data.angle = i * 6;
                obs_data.distance = obs_on_field[i];
                obs_data.pos_x = pos_robot[0] + obs_on_field[i] * cos((i * 6 * DEG2RAD));
                obs_data.pos_y = pos_robot[1] + obs_on_field[i] * sin((i * 6 * DEG2RAD));

                obs_data.detection[i].status = 1;
                obs_data.detection[i].pos_x = obs_data.pos_x;
                obs_data.detection[i].pos_y = obs_data.pos_y;

                obs_counter++;

                if (obs_counter == 1)
                    obs_start = i;

                printf("All datas: %d %f %f %f %f\n", obs_data.status, obs_data.angle, obs_data.distance, obs_data.pos_x, obs_data.pos_y);
            }
            else
            {
                if (obs_counter >= 2)
                {
                    obs_final = i - 1;
                    obs_data.status = 1;
                    obs_data.angle = (obs_start + obs_final) * 6;

                    obs_data.pos_x = 0;
                    obs_data.pos_y = 0;

                    for (uint8_t j = obs_start; j <= obs_final; j++)
                    {
                        obs_data.pos_x += obs_data.detection[j].pos_x;
                        obs_data.pos_y += obs_data.detection[j].pos_y;
                    }

                    obs_data.pos_x /= (obs_final - obs_start);
                    obs_data.pos_y /= (obs_final - obs_start);

                    obs_data.pos_x += 25 * cosf(obs_data.angle * M_PI * 0.005555556);
                    obs_data.pos_y += 25 * sinf(obs_data.angle * M_PI * 0.005555556);

                    float buffer_obs_dist;

                    buffer_obs_dist = Pythagoras(pos_robot[0], pos_robot[1], obs_data.pos_x, obs_data.pos_y);

                    obs_dist.push_back(buffer_obs_dist);
                    obs_index.push_back((int)(obs_start + obs_final) * 0.5);
                }
                obs_counter = 0;
            }
        }
    }
    else if (init_index > final_index)
    {
        obs_counter = 0;
        for (uint8_t i = init_index; i <= final_index; i++)
        {
            /* Check if the obstacle is in the range of our dist thresh */
            if (obs_on_field[i] < dist && i != final_index)
            {
                obs_data.status = 1;
                obs_data.angle = i * 6;
                obs_data.distance = obs_on_field[i];
                obs_data.pos_x = pos_robot[0] + obs_on_field[i] * cos((i * 6 * DEG2RAD));
                obs_data.pos_y = pos_robot[1] + obs_on_field[i] * sin((i * 6 * DEG2RAD));

                obs_data.detection[i].status = 1;
                obs_data.detection[i].pos_x = obs_data.pos_x;
                obs_data.detection[i].pos_y = obs_data.pos_y;

                obs_counter++;

                if (obs_counter == 1)
                    obs_start = i;
            }
            else
            {
                if (obs_counter >= 2)
                {
                    obs_final = i - 1;
                    obs_data.status = 1;
                    obs_data.angle = (obs_start + obs_final) * 6;

                    obs_data.pos_x = 0;
                    obs_data.pos_y = 0;

                    for (uint8_t j = obs_start; j <= obs_final; j++)
                    {
                        obs_data.pos_x += obs_data.detection[j].pos_x;
                        obs_data.pos_y += obs_data.detection[j].pos_y;
                    }

                    obs_data.pos_x /= (obs_final - obs_start);
                    obs_data.pos_y /= (obs_final - obs_start);

                    obs_data.pos_x += 25 * cosf(obs_data.angle * M_PI * 0.005555556);
                    obs_data.pos_y += 25 * sinf(obs_data.angle * M_PI * 0.005555556);

                    float buffer_obs_dist;

                    buffer_obs_dist = Pythagoras(pos_robot[0], pos_robot[1], obs_data.pos_x, obs_data.pos_y);

                    obs_dist.push_back(buffer_obs_dist);
                    obs_index.push_back((int)(obs_start + obs_final) * 0.5);
                }
                obs_counter = 0;
            }
            if (obs_on_field[i] < dist && i == final_index)
                obs_final = i;
        }
        for (uint8_t i = init_index; i <= 59; i++)
        {
            /* Check if the obstacle is in the range of our dist thresh */
            if (obs_on_field[i] < dist && i != 59)
            {
                obs_data.status = 1;
                obs_data.angle = i * 6;
                obs_data.distance = obs_on_field[i];
                obs_data.pos_x = pos_robot[0] + obs_on_field[i] * cos((i * 6 * DEG2RAD));
                obs_data.pos_y = pos_robot[1] + obs_on_field[i] * sin((i * 6 * DEG2RAD));

                obs_data.detection[i].status = 1;
                obs_data.detection[i].pos_x = obs_data.pos_x;
                obs_data.detection[i].pos_y = obs_data.pos_y;

                obs_counter++;

                if (obs_counter == 1)
                    obs_start = i;
            }
            else
            {
                if (obs_counter >= 2)
                {
                    obs_final = i - 1;
                    obs_data.status = 1;
                    obs_data.angle = (obs_start + obs_final) * 6;

                    obs_data.pos_x = 0;
                    obs_data.pos_y = 0;

                    for (uint8_t j = obs_start; j <= obs_final; j++)
                    {
                        obs_data.pos_x += obs_data.detection[j].pos_x;
                        obs_data.pos_y += obs_data.detection[j].pos_y;
                    }

                    obs_data.pos_x /= (obs_final - obs_start);
                    obs_data.pos_y /= (obs_final - obs_start);

                    obs_data.pos_x += 25 * cosf(obs_data.angle * M_PI * 0.005555556);
                    obs_data.pos_y += 25 * sinf(obs_data.angle * M_PI * 0.005555556);

                    float buffer_obs_dist;

                    buffer_obs_dist = Pythagoras(pos_robot[0], pos_robot[1], obs_data.pos_x, obs_data.pos_y);

                    obs_dist.push_back(buffer_obs_dist);
                    obs_index.push_back((int)(obs_start + obs_final) * 0.5);
                }
                obs_counter = 0;
            }
            if (obs_on_field[i] < dist && i == 59)
                obs_final = i;
        }
    }
    // /* Full Scan */
    // else if (init_index == final_index)
    // {
    // }
    // printf("obs:  %f %f\n", obs_data.pos_x, obs_data.pos_y);

    return obs_data;
}