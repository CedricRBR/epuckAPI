// ------------------------------------------------------------------------------
// ! \file webotsAPI.h
// ! declares the webots API
//
// ------------------------------------------------------------------------------

#ifndef WEBOTSAPI_H
#define WEBOTSAPI_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <webots/camera.h>
#include <webots/distance_sensor.h>
#include <webots/emitter.h>
#include <webots/led.h>
#include <webots/light_sensor.h>
#include <webots/motor.h>
#include <webots/receiver.h>
#include <webots/robot.h>

// to create dir
#include <sys/stat.h>
#include <unistd.h>

#define TIME_STEP             64                                // !< number of milliseconds to compute per simulation step

#define MAX_SPEED             1000                              // !< maximum speed of the robot
#define NORM_SPEED            500                               // !< normal speed of the robot

#define LED_COUNT             4                                 // !< number of LEDs on the robot

#define PROX_SENSORS_COUNT    8                                 // !< number of proximity sensors on the robot
#define NBR_CALIB             50                                // !< number of used readings for calibration
#define OFFSET_CALIB          5                                 // !< start evaluating data for calibration after this many dummy readings

#define PROX_RIGHT_FRONT      0                                 // !< index of the front right proximity sensor
#define PROX_RIGHT_FRONT_DIAG 1                                 // !< index of the front right diagonal proximity sensor
#define PROX_RIGHT_SIDE       2                                 // !< index of the right side proximity sensor
#define PROX_RIGHT_BACK       3                                 // !< index of the back right proximity sensor
#define PROX_LEFT_BACK        4                                 // !< index of the back left proximity sensor
#define PROX_LEFT_SIDE        5                                 // !< index of the left side proximity sensor
#define PROX_LEFT_FRONT_DIAG  6                                 // !< index of the front left diagonal proximity sensor
#define PROX_LEFT_FRONT       7                                 // !< index of the front left proximity sensor
#define MAX_PROX              800                               // !< maximum value of a proximity sensor

#define GROUND_SENSORS_COUNT  3                                 // !< number of ground sensors
#define GS_LEFT               0                                 // !< index of the left ground sensor
#define GS_CENTER             1                                 // !< index of the central ground sensor
#define GS_RIGHT              2                                 // !< index of the right ground sensor

#define CAM_RATE              8                                 // !< framerate of the camera
#define CAMERA_WIDTH          160                               // !< horizontal pixel count of the camera
#define CAMERA_HEIGHT         120                               // !< vertical pixel count of the camera

#define COM_CHANNEL           1                                 // !< communication channel index
#define MSG_NONE              "ZZZZ"                            // !< dummy message
#define MSG_LENGTH            4                                 // !< byte length of the message

extern WbDeviceTag  left_motor;                                 // !< left motor
extern WbDeviceTag  right_motor;                                // !< right motor

extern const char * led_names[LED_COUNT];                       // !< led names
extern WbDeviceTag  led_tags[LED_COUNT];                        // !< Webots led link

extern const char * prox_sensors_names[PROX_SENSORS_COUNT];     // !< proximity sensor names
extern WbDeviceTag  prox_sensor_tags[PROX_SENSORS_COUNT];       // !< proximity sensor webots link
extern double       prox_corr_vals[PROX_SENSORS_COUNT];         // !< proximity sensor correction values

extern const char * ground_sensors_names[GROUND_SENSORS_COUNT]; // !< ground sensor names
extern WbDeviceTag  ground_sensor_tags[GROUND_SENSORS_COUNT];   // !< ground sensor webots link

extern WbDeviceTag  cam;                                        // !< camera webots link

extern WbDeviceTag  emitter;                                    // !< emitter webots link
extern WbDeviceTag  receiver;                                   // !< receiver webots link

/*** ROBOT CONTROL start ***/

/**
 *  Initialise the motors
 *
 *  \return void
 **/
void init_motor();

/**
 *  set the speed of the robot
 *
 *  \param left the speed of the left wheel
 *  \param right the speed of the right wheel
 *  \return void
 **/
void set_speed(double left, double right);

/**
 *  return a bounded version of the input speed
 *
 *  \param speed the unbounded speed
 *  \return the speed [-MAX_SPEED..+MAX_SPEED]
 **/
double bounded_speed(double speed);

/*** ROBOT CONTROL end ***/

/*** LEDs start ***/
/**
 *  initiliase the LEDs on the robot
 *
 *  \return void
 **/
void init_led();

/**
 *  toggle a given LED on the robot
 *
 *  \param led_position the index of the LED to toggle
 *  \return void
 **/
void toggle_led(int led_position);

/**
 *  turn a given LED on
 *
 *  \param led_position the index of the LED to turn on
 *  \return void
 **/
void enable_led(int led_position);

/**
 *  turn a given LED off
 *
 *  \param led_position the index of the LED to turn off
 *  \return void
 **/
void disable_led(int led_position);

/*** PROXIMITY SENSORS start ***/

/*** IR SENSORS start ***/

/**
 *  Initialise proximity sensors
 *
 *  \return void
 **/
void init_prox();

/**
 *  get the raw proximity sensor data
 *
 *  \param prox_values pointer to the array holding the  data
 *  \return void
 **/
void get_prox(short int *prox_values);

/**
 *  get the calibrated proximity sensor data
 *
 *  \param prox_values pointer to the array holding the data
 *  \return void
 **/
void get_prox_calibrated(short int *prox_values);

/**
 *  calibrate the proximity sensors
 *
 *  \return void
 **/
void calibrate_prox();

void get_light(short int *prox_values);
void get_light_calibrated(short int *prox_values);
void calibrate_light();

/**
 *  Disable proximity sensors
 *
 *  \return void
 **/
void disable_prox();

/**
 *  Initialise ground sensors
 *
 *  \return void
 **/
void init_ground();

/**
 *  get ground sensor data
 *
 *  \param ground_values pointer to the array holding the data
 *  \return void
 **/
void get_ground(short int *ground_values);

/**
 *  Disable ground sensors
 *
 *  \return void
 **/
void disable_ground();

/*** IR SENSORS end ***/

/*** CAMERA start ***/

/**
 *  initiliase and enable the camera
 *
 *  \return void
 **/
void init_camera();

/**
 *  disable the camera
 *
 *  \return void
 **/
void disable_camera();

/*** TEMPERATURE start ***/

#define TEMP_SENSOR_COUNT 1
void get_temp(unsigned char *temp);

/*** TEMPERATURE stop ***/

/*** TOF start ***/

#define TOF_SENSOR_COUNT 1
void get_tof(short int *tof_distance);

/*** TOF stop ***/

/*** ACCELEROMETER start ***/
#include <webots/accelerometer.h>

#define AXES_X            0
#define AXES_Y            1
#define AXES_Z            2

// instantaneous rotational speed for each axis
#define GYRO_SENSOR_COUNT 3
void get_gyro_axes( short *gyro);

// planar orientation of acceleration vector (relative to robot)
#define ORIENT_SENSOR_COUNT 1
void get_orientation(float *orientation);

// inclination to vertical of acceleration vector
#define INCLIN_SENSOR_COUNT 1
void get_inclination(float *inclination);

// magnitude of acceleration vector
#define ACC_SENSOR_COUNT 1
void get_acceleration(float *acceleration);

// raw acceleration values for each axis
#define ACC_RAW_SENSOR_COUNT 3
void get_acceleration_axes(short int *acceleration);

/*** ACCELEROMETER stop ***/

/*** SOUND start ***/

void play_sound(int sound);
void stop_sound(void);

#define MICROPHONE_COUNT 4

#define MICROPHONE_FRONT 0
#define MICROPHONE_RIGHT 1
#define MICROPHONE_BACK  2
#define MICROPHONE_LEFT  3

void get_microphones(short int *soundlevels);

/*** SOUND stop ***/

/*** COMMUNICATION start ***/

/**
 *  get an image from the camera
 *
 *  \param red pointer to the array holding the red channel data
 *  \param green pointer to the array holding the green channel data
 *  \param blue pointer to the array holding the blue channel data
 *  \return void
 **/
void get_camera(unsigned char *red, unsigned char *green, unsigned char *blue);
/*** CAMERA end ***/

/*** COMMUNICATION start ***/

/**
 *  initiliase and enable communication
 *
 *  \return void
 **/
void init_communication();

/**
 *  disable communication
 *
 *  \return void
 **/
void disable_communication();

/**
 *  send a message to the controller
 *
 *  \param snd pointer to the char array holding the data to send
 *  \return void
 **/
void send_msg(const char *snd);

/**
 *  receive a message from the controller
 *
 *  \param rcv pointer to the char array holding the received data
 *  \return void
 **/
void receive_msg(char *rcv);

/**
 *  get the ID of the robot
 *
 *  \return the ID of the robot
 **/
int get_robot_ID();

/*** COMMUNICATION end ***/

/*** ROBOT HANDLING start ***/

/**
 *  initialise the motors on the robot
 *
 *  \return void
 **/
void init_robot();

/**
 *  send the commands in the output buffer and receive the data from the sensors
 *
 *  \return always returns 1
 **/
int robot_go_on();

/**
 *  clean up the robot
 *
 *  \return void
 **/
void cleanup_robot();

/**
 *  initialise the sensors
 *
 *  \return void
 **/
void init_sensors();

/**
 *  disable the sensors
 *
 *  \return void
 **/
void disable_sensors();

/*** ROBOT HANDLING end ***/

#endif // WEBOTSAPI_H