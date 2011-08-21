//to organize A_messagePump better
//Ian split the functions into different files

//if we're assigning a pin to something, it's done here

//THESE PINS WERE CHECKED ON JULY 20 2011
//analog pins for pot reading
// Pins defined in boashield_pins.h

//Sensor Pins
const int sensorPinX[5] = {HORZ_POS_SENSOR_1, 
                          HORZ_POS_SENSOR_2, 
                          HORZ_POS_SENSOR_3, 
                          HORZ_POS_SENSOR_4, 
                          HORZ_POS_SENSOR_5};

const int sensorPinZ[5] = { VERT_POS_SENSOR_1, 
                          VERT_POS_SENSOR_2,
                          VERT_POS_SENSOR_3,
                          VERT_POS_SENSOR_4,
                          VERT_POS_SENSOR_5};

//PID pwn pins
const int actuatorPinX[5] = {HORZ_ACTUATOR_1, 
                            HORZ_ACTUATOR_2, 
                            HORZ_ACTUATOR_3, 
                            HORZ_ACTUATOR_4, 
                            HORZ_ACTUATOR_5};

const int actuatorPinZ[5] = {VERT_ACTUATOR_1, 
                            VERT_ACTUATOR_2, 
                            VERT_ACTUATOR_3, 
                            VERT_ACTUATOR_4, 
                            VERT_ACTUATOR_5};

//Valve select pins because we don't have enough PWM
const int valveSelectX[5] = {HORZ_ACTUATOR_CTRL_1, 
                            HORZ_ACTUATOR_CTRL_2, 
                            HORZ_ACTUATOR_CTRL_3, 
                            HORZ_ACTUATOR_CTRL_4, 
                            HORZ_ACTUATOR_CTRL_5};

const int valveSelectZ[5] = {VERT_ACTUATOR_CTRL_1, 
                            VERT_ACTUATOR_CTRL_2, 
                            VERT_ACTUATOR_CTRL_3, 
                            VERT_ACTUATOR_CTRL_4, 
                            VERT_ACTUATOR_CTRL_5};

//Dither will be fed into all unselected valves
const int ditherPin = DITHER;

//motor write speed
const int motorPin = MOTOR_CONTROL;

/*limit switches in series. will determine L/R through math*/
const int limit_switchX[5] = {HORZ_LIMIT_SWITCH_1, 
                             HORZ_LIMIT_SWITCH_2,
                             HORZ_LIMIT_SWITCH_3, 
                             HORZ_LIMIT_SWITCH_4, 
                             HORZ_LIMIT_SWITCH_5};
                             
const int limit_switchZ[5] = {VERT_LIMIT_SWITCH_1, 
                             VERT_LIMIT_SWITCH_2, 
                             VERT_LIMIT_SWITCH_3, 
                             VERT_LIMIT_SWITCH_4, 
                             VERT_LIMIT_SWITCH_5};

//defaul values for motor
int motorValue = 127;

//have to check battery voltage
const int batPin = BAT_LEVEL_24V;

//arduinos need to kill
