#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "mbed.h"
#include <stdio.h>
#include <cmath>
#include "Sensors/Sensors.h"
#include "EsconDriver/EsconDriver.h"

#define HZ 40000

typedef struct
{
    float x_des, dx_des, theta_des, dtheta_des, kp_m, kd_m, kp_x, kd_x, t_ff, r, L;
} ControllerStruct;

class Controller
{
public:
    Sensors *sensors;
    EsconDriver *driver;
    ControllerStruct parameters;
    // float des_current;
    float current = 0.;
    int16_t des_cur_bytes = 0;
    // controller->driver->setCurrent(current);

    uint16_t mot_counts = 0;
    uint16_t lin_counts = 0;
    uint16_t f_sensor = 0;  // 0    ... 65535
    uint16_t m_current = 0; // 0    ... 65535

    // class constructor
    Controller(Sensors *sensors, EsconDriver *driver);
    // set the controller parameters

    void reset_param();
    float jacobian(float theta, float L, float r);
    // void motorControl();
    // void posControl();
    // void 
    
    void control();
    void updateSensors();
    void update();

    // class destructor
    ~Controller();

private:
};

#endif