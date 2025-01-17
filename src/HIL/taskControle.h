/**
  ******************************************************************************
  * @file    taskControle.h
  * @brief   This file contains the declarations for the FreeRTOS task 
  *          taskControle, which make a control of a autonomous vehicle by the
  *          high level control action given by Autoware or manualy controled by
  *          a physical joysitck.
  ******************************************************************************
  * @author  Gabriel Toffanetto França da Rocha 
  *          Laboratory of Autonomous Vehicles (LMA) - FEM/Unicamp
  * @date    Created:  October 21, 2024
  *          Modified: 
  ******************************************************************************
  */
#ifndef TASKCONTROLE_H_
#define TASKCONTROLE_H_

#include "microAutoware.h"
#include "utils.h"
#include "main.h"
#include "cmsis_os.h"
#include "adc.h"

// Timeout to connect micro-ROS agent (seconds)
#define TIMEOUT_MICRO_ROS_AGENT 20

// Timeout to wait control action
#define TIMEOUT_GET_CONTROL_ACTION 110

// Timeout to wait CARLA data
#define TIMEOUT_GET_CARLA_RX 100

// Max of lost messages acceptable
#define MAX_OF_LOST_MESSAGES 10

// Number of bytes of control message sent to CARLA.
#define MSG_TO_CARLA_SIZE 30

// HIL CARLA UART Handle
#define HUART_CARLA huart2

// Time period for wait between manual commands sent to CARLA.
#define MANUAL_CONTROL_TIME_COMMAND 60

// Vehicle limits to manual control
#define MAX_THROTTLE 1
#define MAX_BRAKE 1
#define MAX_STEERING_ANGLE 1 

// Joystick ADC Handler
#define HADC hadc1

// Number of ticks for JoySW debounce delay
#define DEBOUNCE_TICKS 1000

// UART RX DMA BUFFER SIZE = MSG_RX_SIZE = 22 bytes
#define UART2_DMA_BUFFER_SIZE 22


#endif  /* TASKCONTROLE_H_ */
