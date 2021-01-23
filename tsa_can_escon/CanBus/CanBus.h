#ifndef CANBUS_H
#define CANBUS_H

/* Network settings */
#define txSize 8
#define rxSize 8
#define CAN_ID 1
#define CAN_MASTER_ID 0

#include <stdio.h>
#include "mbed.h"
#include "Controller/Controller.h"

class CanBus
{
public:
    enum
    {
        MSG_MOTOR_ON = 0x88,
        MSG_MOTOR_OFF = 0x80,
        MSG_SET_CURRENT = 0xA1,
        MSG_RESET = 0x05,
        MSG_SET_KINEMATICS, // set length and radius 
        MSG_SET_MOT_GAINS, // set motor pd gains
        MSG_SET_LIN_GAINS, // set linear pd gains
        MSG_LIN_POS_CONTROL, // 
        MSG_MOT_POS_CONTROL,
        MSG_FORCE_CONTRIOL,
        MSG_
        // MSG_WRITE_PID_RAM,
        // MSG_READ_PID,
        // MSG_SET_POSITION,
        // MSG_SET_VELOCITY,
        // MSG_GET_IMU_ACCEL,
        // MSG_IMU_SENSETIVITY,
        // MSG_CALIBRATE_FS, 
        // MSG_GET_FORCE, 
        // MSG_GET_POSITION,
        // MSG_SET_KM,
    };

    // class constructor
    CanBus(PinName rd, PinName td, int hz, Controller *controller);
    // Message 
    void onMsgReceived();

    // cmd functions
    void set_motor_on(CANMessage &msg); // Turn driver in enable mode
    void set_motor_off(CANMessage &msg); // Turn driver in disable mode
    void reset_device(CANMessage &msg); 
    // void set_current_1(CANMessage &msg);
    void set_current(CANMessage &msg);
    
    // TODO:
    void write_pid_ram(CANMessage &msg);
    void read_pid_callback(CANMessage &msg);
    void set_position_callback(CANMessage &msg);
    void read_imu_accellerometer(CANMessage &msg);

    void unknown_command(CANMessage &msg);

    // class destructor
    ~CanBus();

private:
    CAN can;
    CANMessage rxMsg;
    CANMessage txMsg;

    Controller *controller;
};

#endif