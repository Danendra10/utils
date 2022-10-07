#ifndef PID_H_
#define PID_H_

#ifdef _cplusplus
extern "C"
{
#endif

    float min_out;
    float max_out;
    float min_integral;
    float max_integral;
    float integral;
    float last_error;
    float proportional;
    float derivative;
    float output_speed;

    typedef struct PID_tag
    {
        float Kp;
        float Ki;
        float Kd;
    } PID_t;

    void PIDInit(PID_t *pid, float Kp, float Ki, float Kd)
    {
        pid->Kp = Kp;
        pid->Ki = Ki;
        pid->Kd = Kd;
    }
    float PIDCalculate(PID_t *pid, float error, float minmax)
    {
        min_out= min_integral = -minmax;
        max_out = max_integral = minmax;

        proportional = pid->Kp * error;
        integral += pid->Ki * error;
        derivative = pid->Kd * (error - last_error);

        last_error = error;

        if (integral > max_integral)
            integral = max_integral;
        else if (integral < min_integral)
            integral = min_integral;

        output_speed = proportional + integral + derivative;

        if(output_speed > max_out)
            output_speed = max_out;
        else if(output_speed < min_out)
            output_speed = min_out;        
        return output_speed;
    }

#ifdef _cplusplus
}
#endif

#endif // PID_H_