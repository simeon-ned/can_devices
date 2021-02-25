#include "Controller.h"

// class constructor
Controller::Controller(Sensors *sensors, EsconDriver *driver) : sensors(sensors),
                                                                driver(driver)
{
    printf("Controller object was constructed.\n");
    reset_param();
}

void Controller::reset_param()
{
    parameters.x_des = 0.0;
    parameters.dx_des = 0.0;
    parameters.theta_des = 0.0;
    parameters.dtheta_des = 0.0;
    parameters.kp_m = 0.0;
    parameters.kd_m = 0.0;
    parameters.kp_x = 0.0;
    parameters.kd_x = 0.0;
    parameters.t_ff = 0.0;
}

// TODO implement
float Controller::jacobian(float theta, float L, float r)
{   
    return 0;
}

void Controller::control()
{
    // float p_error = parameters.p_des - sensors->getMotorPosition();
    // float v_error = parameters.v_des - sensors->getMotorSpeed(0.00005f);

    // float current_ref = parameters.kp * p_error + parameters.kd * v_error + parameters.t_ff;
    // printf(curr_dump);
    // float current_ref = 0;
    // printf("curr: %d\n", curr_dump);
    // printf("curr_ref: %.4f \n", current_ref);
    //printf("p_des: %.4f \n", parameters.p_des);
    //printf("kp: %.5f kd: %.5f e: %.4f cur_p: %.4f cur: %.4f \n", parameters.kp, parameters.kd, error, sensors->getMotorSpeed(0.025), current_ref);
    // if (fabs(current_ref) > 0.5)
    //     current_ref = 0.0;

    // current = ((float) des_cur_bytes)/100000.;
    // driver->setCurrent(current);
    driver->setCurrentU16(des_cur_bytes);

    //printf("kp: %f p_des %f m_pos %f kd %f v_des %f m_v %f t_ff %f \n", parameters.kp, parameters.p_des, sensors->getMotorPosition(), parameters.kd, parameters.v_des, sensors->getMotorSpeed(1.0 / HZ), parameters.t_ff);
}

void Controller::updateSensors(){
    mot_counts = sensors->getMotorCounter(); // 0 ... 65535
    lin_counts = sensors->getLinearCounter(); // 0 ... 65535
    f_sensor = sensors->getForceU16();     // 0    ... 65535
    m_current = driver->getFilteredCurrentU16();   // 0    ... 65535
}


void Controller::update()
{
    //sensors->readIMU();
    updateSensors();
    control();    
}


// class destructor
Controller::~Controller()
{
    printf("Controller object was destructed.\n");
}
