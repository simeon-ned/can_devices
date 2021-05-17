#include "mbed.h"
#include "EsconDriver/EsconDriver.h"
#include "CanBus/CanBus.h"
#include "Controller/Controller.h"
#include "TimEncoders/Nucleo_Encoder_16_bits.h"
#include "MPU9250.h"

#define M_PI 3.14159265358979323846

// BufferedSerial pc(USBTX, USBRX, 115200); // tx, rx

// FileHandle *mbed::mbed_override_console(int fd)
// {
//   return &pc;
// }

//Serial usb(USBTX, USBRX, 115200);

// DRIVER //
// PinName driverDigitalPins[] = {
//     PE_2, // motor enable output: 0 - disable, 1 - enable
//     LED1, // motor enable LED
//     PE_3, // motor direction output: 0 - CW, 1 - CCW
//     LED2  // motor direction LED
// };

// PinName driverAnalogPins[] = {
//     PC_3, // motor current input
//     PC_2, // motor filtered current input
//     PA_4  // motor setpoint output
// };

// DRIVER //

PinName driverDigitalPins[] = {
    PG_0, // motor enable output: 0 - disable, 1 - enable
    LED1, // motor enable LED
    PG_1, // motor direction output: 0 - CW, 1 - CCW
    LED2  // motor direction LED
};

PinName driverAnalogPins[] = {
    PF_4, // motor current input
    PC_3, // motor filtered current input
    PA_4  // motor setpoint output
};


/* Initialize the EsconDriver */
EsconDriver driver(driverDigitalPins, driverAnalogPins);

/* Initialize the motor and linear encoders */
Nucleo_Encoder_16_bits motor_encoder(TIM4);
Nucleo_Encoder_16_bits linear_encoder(TIM3);

// ForceSensor
AnalogIn force_sensor(PF_9);
/*MPU9250 mpu9250(PF_0, PF_1);*/

Sensors sensors(&motor_encoder, &linear_encoder, &force_sensor /*, &mpu9250*/);

Controller controller(&sensors, &driver);

CanBus can(PB_8, PB_9, 1000000, &controller);

Ticker ticker;

int main()
{

  ticker.attach(callback(&controller, &Controller::update), 50us); // 50us 25us

  while (1)
  {
    // uint16_t dd = controller.parameters.t_ff;
    // printf("motor: %d\n", dd);
    // thread_sleep_for(100);
  }
}