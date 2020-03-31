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
#include <sys/stat.h>
#include <unistd.h>
#include <webots/accelerometer.h>
#include <webots/camera.h>
#include <webots/distance_sensor.h>
#include <webots/emitter.h>
#include <webots/led.h>
#include <webots/light_sensor.h>
#include <webots/motor.h>
#include <webots/receiver.h>
#include <webots/robot.h>

#define TIME_STEP             64                                // !< number of milliseconds to compute per simulation step

#define MAX_SPEED             1000                              // !< maximum speed of the robot
#define NORM_SPEED            400                               // !< normal speed of the robot

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
#define MAX_PROX              200                               // !< maximum value of a proximity sensor

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

#define TEMP_SENSOR_COUNT     1                                 // !< number of temperature sensors

#define TOF_SENSOR_COUNT      1                                 // !< number of Time of Flight sensors

#define GYRO_SENSOR_COUNT     3                                 // !< number of gyroscopic sensors
#define ORIENT_SENSOR_COUNT   1                                 // !< number of orientation sensors
#define INCLIN_SENSOR_COUNT   1                                 // !< number of inclination sensors
#define ACC_SENSOR_COUNT      1                                 // !< number of accelerometer sensors
#define ACC_RAW_SENSOR_COUNT  3                                 // !< number of raw accelerometer sensors
#define AXES_X                0                                 // !< index of the X-axis gyro / orient / inclin / acc sensor
#define AXES_Y                1                                 // !< index of the Y-axis gyro / orient / inclin / acc sensor
#define AXES_Z                2                                 // !< index of the Z-axis gyro / orient / inclin / acc sensor

#define MICROPHONE_COUNT      4                                 // !< number of microphones
#define MICROPHONE_FRONT      0                                 // !< index of the front microphone
#define MICROPHONE_RIGHT      1                                 // !< index of the right microphone
#define MICROPHONE_BACK       2                                 // !< index of the back microphone
#define MICROPHONE_LEFT       3                                 // !< index of the left microphone

extern WbDeviceTag  left_motor;                                 // !< left motor
extern WbDeviceTag  right_motor;                                // !< right motor

extern const char * led_names[LED_COUNT + 2];                   // !< LED names
extern WbDeviceTag  led_tags[LED_COUNT + 2];                    // !< Webots LED link

extern const char * rgbled_names[LED_COUNT];                    // !< RGB LED names
extern WbDeviceTag  rgbled_tags[LED_COUNT];                     // !< Webots RGB LED link

extern const char * prox_sensors_names[PROX_SENSORS_COUNT];     // !< proximity sensor names
extern WbDeviceTag  prox_sensor_tags[PROX_SENSORS_COUNT];       // !< proximity sensor webots link
extern double       prox_corr[PROX_SENSORS_COUNT];              // !< proximity sensor correction values

extern const char * ground_sensors_names[GROUND_SENSORS_COUNT]; // !< ground sensor names
extern WbDeviceTag  ground_sensor_tags[GROUND_SENSORS_COUNT];   // !< ground sensor webots link

extern const char * light_sensors_names[PROX_SENSORS_COUNT];    // !< light sensor names
extern WbDeviceTag  light_sensor_tags[PROX_SENSORS_COUNT];      // !< light sensor webots link

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

/**
 *  turn the body LED on
 *
 *  \return void
 **/
void enable_body_led(void);

/**
 *  turn the body LED off
 *
 *  \return void
 **/
void disable_body_led(void);

/**
 *  turn the front LED on
 *
 *  \return void
 **/
void enable_front_led(void);

/**
 *  turn the front LED off
 *
 *  \return void
 **/
void disable_front_led(void);

/**
 *  turn a given RGB LED on
 *
 *  \param led_position the index of the RGB LED to turn on
 *  \param led_color an RRGGBB formatted color
 **/
void enable_rgbled(int led_position, int led_color);

/**
 *  turn a given RGB LED off
 *
 *  \param led_position the index o the RGB LED to turn off
 **/
void disable_rgbled(int led_position);

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

/**
 *  <<<<<<< HEAD
 *  get the light sensor data
 *
 *  \param prox_values pointer to the array holding the data
 *  \return void
 **/
void get_light(short int *prox_values);

/**
 *  get the calibrated light sensor data
 *
 *  \param prox_values pointer to the array holding the data
 *  \return void
 **/
void get_light_calibrated(short int *prox_values);

/**
 *  calibrate the light sensors
 *
 *  \return void
 **/
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
 *  =======
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
 *  >>>>>>> 9541c2903e8a14ee3a71da48e60bd8f1ad0a6cc4
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

/**
 *  <<<<<<< HEAD
 *  get temperature data
 *
 *  \param temp pointer to the array holding the data
 **/

void get_temp(unsigned char *temp);

/**
 *  get Time of Flight data
 *
 *  \param tof_distance pointer to the array holding the data
 **/
void get_tof(short int *tof_distance);

/**
 *  get gyroscopic data
 *
 *  get the instantaneous rotational speed for each axis
 *
 *  \param gyro pointer to the array holding the data
 **/
void get_gyro_axes( short *gyro);

/**
 *  get orientation of robot
 *
 *  get the planar orientation of acceleration vector (relative to robot)
 *
 *  \param orientation pointer to the array holding the data
 **/
void get_orientation(float *orientation);
/**
 *  get inclination of robot
 *
 *  get the inclination to vertical of the acceleration vector
 *
 *  \param inclination pointer to the array holding the data
 **/
void get_inclination(float *inclination);

/**
 *  get acceleration of robot
 *
 *  get the magnitude of the acceleration vector
 *
 *  \param acceleration pointer to the array holding the data
 **/
void get_acceleration(float *acceleration);

/**
 *  get the raw acceleration fo the robot
 *
 *  get the raw acceleration values for each axis
 *
 *  \param acceleration pointer to the array holding the data
 **/
void get_acceleration_axes(short int *acceleration);

/**
 *  play a sound
 *
 *  \param sound index of the sound to play
 **/
void play_sound(int sound);

/**
 *  stop playing a sound
 *
 **/
void stop_sound(void);

/**
 *  get microphone data
 *
 *  \param soundlevels pointer to the array holding the data
 **/
void get_microphones(short int *soundlevels);

/**
 *  get an image from the camera
 *
 *  \param red pointer to the array holding the red channel data
 *  \param green pointer to the array holding the green channel data
 *  \param blue pointer to the array holding the blue channel data
 *  \return void
 **/
void get_camera(unsigned char *red, unsigned char *green, unsigned char *blue);

/**
 *  =======
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
 *  >>>>>>> 9541c2903e8a14ee3a71da48e60bd8f1ad0a6cc4
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