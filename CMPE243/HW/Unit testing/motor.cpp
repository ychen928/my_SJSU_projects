#include "motor.hpp"
#include "sensor.hpp"

int get_motor_pwm(int sensor_value)
{
    int value = read_sensor_value(sensor_value);
    if (value < 0 || value > 100)
    {
        return -1;
    }
    else if (value >= 0 && value < 50)
    {
        return 8;
    }
    else if (value >= 50 && value <= 100)
    {
        return 9;
    }
    return 0;
}
