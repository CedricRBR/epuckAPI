#ifndef EPUCKAPI_H
#define EPUCKAPI_H

#include <stdio.h>
#include <stdlib.h>

// for opening socket to the robot
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/time.h>

// for communication API
#include <sys/ipc.h>
#include <sys/msg.h>
#include <sys/shm.h>
#include <sys/types.h>

// to create dir
#include <sys/stat.h>
#include <unistd.h>

#include <string.h>

#define DEBUG                 0                                     // !< set to 1 to enter debug mode, leave at 0 otherwise

#define MAX_SPEED             1000                                  // !< maximum speed of the robot
#define NORM_SPEED            400                                   // !< normal speed of the robot
#define LED_COUNT             4                                     // !< number of LEDs on the robot

#define PROX_SENSORS_COUNT    8                                     // !< number of proximity sensors on the robot
#define NBR_CALIB             50                                    // !< number of used readings for calibration
#define OFFSET_CALIB          5                                     // !< start evaluating data for calibration after this many dummy readings

#define PROX_RIGHT_FRONT      0                                     // !< index of the front right proximity sensor
#define PROX_RIGHT_FRONT_DIAG 1                                     // !< index of the front right diagonal proximity sensor
#define PROX_RIGHT_SIDE       2                                     // !< index of the right side proximity sensor
#define PROX_RIGHT_BACK       3                                     // !< index of the back right proximity sensor
#define PROX_LEFT_BACK        4                                     // !< index of the back left proximity sensor
#define PROX_LEFT_SIDE        5                                     // !< index of the left side proximity sensor
#define PROX_LEFT_FRONT_DIAG  6                                     // !< index of the front left diagonal proximity sensor
#define PROX_LEFT_FRONT       7                                     // !< index of the front left proximity sensor

#define MAX_PROX              200                                   // !< maximum value of a proximity sensor

#define GROUND_SENSORS_COUNT  3                                     // !< number of ground sensors
#define GS_LEFT               0                                     // !< index of the left ground sensor
#define GS_CENTER             1                                     // !< index of the central ground sensor
#define GS_RIGHT              2                                     // !< index of the right ground sensor

#define CAMERA_WIDTH          160                                   // !< horizontal pixel count of the camera
#define CAMERA_HEIGHT         120                                   // !< vertical pixel count of the camera

#define TEMP_SENSOR_COUNT     1                                     // !< number of temperature sensors

#define TOF_SENSOR_COUNT      1                                     // !< number of Time of Flight sensors

#define AXES_X                0                                     // !< index of the X axis
#define AXES_Y                1                                     // !< index of the y axis
#define AXES_Z                2                                     // !< index of the z axis

#define GYRO_SENSOR_COUNT     3                                     // !< number of gyroscopic sensors

#define ORIENT_SENSOR_COUNT   1                                     // !< number of orientation sensors

#define INCLIN_SENSOR_COUNT   1                                     // !< number of incination sensors

#define ACC_SENSOR_COUNT      1                                     // !< number of acceleration sensors

#define ACC_RAW_SENSOR_COUNT  3                                     // !< number of raw acceleration sensors

#define MICROPHONE_COUNT      4                                     // !< number of microphones

#define MICROPHONE_FRONT      0                                     // !< index of the front facing microphone
#define MICROPHONE_RIGHT      1                                     // !< index of the right facing microphone
#define MICROPHONE_BACK       2                                     // !< index of the back facing microphone
#define MICROPHONE_LEFT       3                                     // !< index of the left facing microphone

#define COM_CHANNEL           1                                     // !< communication channel
#define MSG_NONE              "ZZZZ"                                // !< dummy message

#define MSG_LENGTH            4                                     // !< length of a message

extern char *             ip;                                       // !< IP address of the robot
extern unsigned char      command[21];                              // !< char array holding the commands sent to the robot
extern unsigned char      sensor[104];                              // !< char array holding received sensor data
extern unsigned char      rgb565[CAMERA_WIDTH * CAMERA_HEIGHT * 2]; // !< holds an image in rgb565
extern unsigned char      bgr888[CAMERA_WIDTH * CAMERA_HEIGHT * 3]; // !< holds an image in bgr888
extern int                camera_updated;                           // !< 1 if the camera has been updated
extern struct sockaddr_in remote_addr;                              // !< remote address for communicating with the robot
extern int                fd;                                       // !< TCP socket
extern int                bytes_sent;                               // !< count of how many bytes were sent to the robot
extern int                bytes_recv;                               // !< count of how many bytes were received from the robot
extern unsigned char      header;                                   // !< holds the header of the data sent by the robot
extern struct timeval     start_time;                               // !< not used atm
extern struct timeval     curr_time;                                // !< not used atm
extern int32_t            time_diff_us;                             // !< not used atm
extern int32_t            refresh;                                  // !< not used atm
extern uint16_t           num_packets;                              // !< not used atm
extern short int          prox_corr[PROX_SENSORS_COUNT];            // !< correction values for the proximity sensors
extern short int          light_corr[PROX_SENSORS_COUNT];           // !< correction values for the ambient sensors
extern key_t              key;                                      // !< key for the ICP message queue
extern int                msgid;                                    // !< holds an ICP message ID
extern int                shmid;                                    // !< ICP shared memory ID
extern int *              queues;                                   // !< holds the ICP message queues for all the robots

extern struct msg_buffer  message;                                  // !< holds a message to be sent

/**
 *  initialise the robot
 *
 *  \return void
 **/
void   init_robot();
/**
 *  initialise the sensors
 *
 *  \return void
 **/
void   init_sensors();

/**
 *  disable the sensors
 *
 *  \return void
 **/
void   disable_sensors();

/**
 *  get the ID of the robot
 *
 *  \return the ID of the robot
 **/
int    get_robot_ID();

/**
 *  clean up the robot
 *
 *  \return void
 **/
void   cleanup_robot();
/**
 *  send the commands in the output buffer and receive the data from the sensors
 *
 *  \return always returns 1
 **/
int    robot_go_on();

/**
 *  set the speed of the robot
 *
 *  \param left the speed of the left wheel
 *  \param right the speed of the right wheel
 *  \return void
 **/
void   set_speed(double left, double right);

/**
 *  get the current step values
 *
 *  \param left the pointer to the int holding the left motor data
 *  \param right the pointer to the int holding the right motor data
 **/
void   get_steps(short *left, short*right);

/**
 *  return a bounded version of the input speed
 *
 *  \param speed the unbounded speed
 *  \return the speed [-MAX_SPEED..+MAX_SPEED]
 **/
double bounded_speed(double speed);

/**
 *  toggle a given LED on the robot
 *
 *  \param led_position the index of the LED to toggle
 *  \return void
 **/
void   toggle_led(int led_position);

/**
 *  turn a given LED on
 *
 *  \param led_position the index of the LED to turn on
 *  \return void
 **/
void   disable_led(int led_position);

/**
 *  turn a given LED off
 *
 *  \param led_position the index of the LED to turn off
 *  \return void
 **/
void   enable_led(int led_position);

/**
 *  turn the body LED on
 *
 *  \return void
 **/
void   enable_body_led(void);

/**
 *  turn the body LED off
 *
 *  \return void
 **/
void   disable_body_led(void);

/**
 *  turn the fonr LED on
 *
 *  \return void
 **/
void   enable_front_led(void);

/**
 *  turn the front LED pff
 *
 *  \return void
 **/
void   disable_front_led(void);

/**
 *  get the raw proximity sensor data
 *
 *  \param prox_values pointer to the array holding the  data
 *  \return void
 **/
void   get_prox(short int *prox_values);

/**
 *  get the calibrated proximity sensor data
 *
 *  \param prox_values pointer to the array holding the data
 *  \return void
 **/
void   get_prox_calibrated(short int *prox_values);

/**
 *  calibrate the proximity sensors
 *
 *  \return void
 **/
void   calibrate_prox();

/**
 *  get the raw light sensor data
 *
 *  \param prox_values pointer to the array holding the  data
 *  \return void
 **/
void   get_light(short int *prox_values);

/**
 *  get the calibrated light sensor data
 *
 *  \param prox_values pointer to the array holding the data
 *  \return void
 **/
void   get_light_calibrated(short int *prox_values);

/**
 *  calibrate the light sensors
 *
 *  \return void
 **/
void   calibrate_light();

/**
 *  get ground sensor data
 *
 *  \param ground_values pointer to the array holding the data
 *  \return void
 **/
void   get_ground(short int *ground_values);

/**
 *  get an image from the camera
 *
 *  \param red pointer to the array holding the red channel data
 *  \param green pointer to the array holding the green channel data
 *  \param blue pointer to the array holding the blue channel data
 *  \return void
 **/
void   get_camera(unsigned char *red, unsigned char *green, unsigned char *blue);

/**
 *  initiliase and enable the camera
 *
 *  \return void
 **/
void   init_camera();

/**
 *  disable the camera
 *
 *  \return void
 **/
void   disable_camera();

/**
 *  Get temperature
 *
 *  \param temp pointer to array holding the data
 *  \return void
 **/
void   get_temp(unsigned char *temp);

/**
 *  Get ToF data
 *
 *  Measures distance in mm
 *
 *  \param tof_distance pointer to array holding the data
 *  \return void
 **/
void   get_tof(short int *tof_distance);

/**
 *  get gyroscopic data
 *
 *  \param gyro pointer to array holding the data
 *  \return void
 **/
void   get_gyro_axes( short *gyro);

/**
 *  get orientation data
 *
 *  \param orientation pointer to array holding the data
 *  \return void
 **/
void   get_orientation(float *orientation);
/**
 *  get inclination data
 *
 *  \param inclination pointer to array holding the data
 *  \return void
 **/
void   get_inclination(float *inclination);
/**
 *  get acceleration data
 *
 *  \param acceleration pointer to array holding the data
 *  \return void
 **/
void   get_acceleration(float *acceleration);
/**
 *  get axes acceleration data
 *
 *  \param acceleration pointer to array holding the data
 *  \return void
 **/
void   get_acceleration_axes(short int *acceleration);
/**
 *  play a sound
 *
 *  \param sound index of the song to play
 *  \return void
 **/
void   play_sound(int sound);
/**
 *  stop playing sounds
 *
 *  \return void
 **/
void   stop_sound(void);
/**
 *  get microphone data
 *
 *  \param soundlevels pointer to array holding the data
 *  \return void
 **/
void   get_microphones(short int *soundlevels);

/**
 *  initiliase and enable communication
 *
 *  \return void
 **/
void   init_communication();

/**
 *  disable communication
 *
 *  \return void
 **/
void   disable_communication();

/**
 *  send a message to the controller
 *
 *  \param snd pointer to the char array holding the data to send
 *  \return void
 **/
void   send_msg(const char *snd);

/**
 *  receive a message from the controller
 *
 *  \param rcv pointer to the char array holding the received data
 *  \return void
 **/
void   receive_msg(char *rcv);

#endif // EPUCKAPI_H