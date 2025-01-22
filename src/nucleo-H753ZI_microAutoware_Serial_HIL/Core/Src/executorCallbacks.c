/**
  ******************************************************************************
  * @file    executorCallbacks.c
  * @brief   This file contains all the callbacks functions from microAutoware
  *          task. Functions templates are in microAutoware.h.
  ******************************************************************************
  * @author  Gabriel Toffanetto França da Rocha 
  *          Laboratory of Autonomous Vehicles (LMA) - FEM/Unicamp
  * @date    Created:  October 9, 2024
  *          Modified: January 22, 2025
  ******************************************************************************
  */

#include "microAutoware.h"

// From microAutoware.c
extern unsigned char ucSubscribersReceived;
extern autoware_auto_vehicle_msgs__srv__ControlModeCommand_Response control_mode_response_msg_;

// Timer callbacks
/**
  * @name   timer_watchdog_agent_callback
  * @brief  ROS timer callback to test if agent is still connected.
  * @param  timer: timer handler
  * @param  last_call_time: last timestamp the timer was called
  * @retval None
  */
void timer_watchdog_agent_callback(rcl_timer_t * timer, int64_t last_call_time)
{
	
  if(RMW_RET_OK != rmw_uros_ping_agent(20, 1)) 
  {
    osEventFlagsSet(EventsMicroAutowareHandle, MA_TO_MANUAL_MODE_FLAG);
    osEventFlagsSet(EventsMicroAutowareHandle, SYS_TO_MANUAL_MODE_FLAG);
  } 

  // Try to reconnect

}

// Subscriptors callbacks


#ifdef SIMULATION_TIME
/**
  * @name   clock_callback
  * @brief  ROS topic subscriber callback to recieve current timestamp
  * @param  xMsgIn: pointer to the message recieved in the topic.
  * @retval None
  */
void clock_callback(const void * xMsgIn)
{
  const rosgraph_msgs__msg__Clock * clock_msg_ = (const rosgraph_msgs__msg__Clock * )xMsgIn;
  ucSubscribersReceived = ucSubscribersReceived | (0b1 << 0);
}
#endif

/**
  * @name   control_cmd_callback
  * @brief  ROS topic subscriber callback to recieve the control commands and save in control_cmd_msg_
  * @param  xMsgIn: pointer to the message recieved in the topic.
  * @retval None
  */
void control_cmd_callback(const void * xMsgIn)
{
  const autoware_auto_control_msgs__msg__AckermannControlCommand * control_cmd_msg_ = (const autoware_auto_control_msgs__msg__AckermannControlCommand * )xMsgIn;
  ucSubscribersReceived = ucSubscribersReceived | (0b1 << 1);
}

/**
  * @name   gear_cmd_callback
  * @brief  ROS topic subscriber callback to recieve the gear command and save in gear_cmd_msg_
  * @param  xMsgIn: pointer to the message recieved in the topic.
  * @retval None
  */
void gear_cmd_callback(const void * xMsgIn)
{
  const autoware_auto_vehicle_msgs__msg__GearCommand * gear_cmd_msg_ = (const autoware_auto_vehicle_msgs__msg__GearCommand * )xMsgIn;
  ucSubscribersReceived = ucSubscribersReceived | (0b1 << 2);
}

/**
  * @name   turn_indicators_cmd_callback
  * @brief  ROS topic subscriber callback to recieve the turn lights commands and save in turn_indicators_cmd_msg_
  * @param  xMsgIn: pointer to the message recieved in the topic.
  * @retval None
  */
void turn_indicators_cmd_callback(const void * xMsgIn)
{
  const autoware_auto_vehicle_msgs__msg__TurnIndicatorsCommand * turn_indicators_cmd_msg_ = (const autoware_auto_vehicle_msgs__msg__TurnIndicatorsCommand * )xMsgIn;
  ucSubscribersReceived = ucSubscribersReceived | (0b1 << 3);
}

/**
  * @name   hazard_lights_cmd_callback
  * @brief  ROS topic subscriber callback to recieve the hazard lights commands and save in hazard_lights_cmd_msg_
  * @param  xMsgIn: pointer to the message recieved in the topic.
  * @retval None
  */
void hazard_lights_cmd_callback(const void * xMsgIn)
{
  const autoware_auto_vehicle_msgs__msg__HazardLightsCommand * hazard_lights_cmd_msg_ = (const autoware_auto_vehicle_msgs__msg__HazardLightsCommand * )xMsgIn;
  ucSubscribersReceived = ucSubscribersReceived | (0b1 << 4);
}

/**
  * @name   actuation_cmd_callback
  * @brief  ROS topic subscriber callback to recieve the control commands and save in actuation_cmd_msg_
  * @param  xMsgIn: pointer to the message recieved in the topic.
  * @retval None
  */
void actuation_cmd_callback(const void * xMsgIn)
{
  const tier4_vehicle_msgs__msg__ActuationCommandStamped * actuation_cmd_msg_ = (const tier4_vehicle_msgs__msg__ActuationCommandStamped * )xMsgIn;
  ucSubscribersReceived = ucSubscribersReceived | (0b1 << 5);
}

/**
  * @name   emergency_callback
  * @brief  ROS topic subscriber callback to recieve the control commands and save in emergency_msg_ 
  * @param  xMsgIn: pointer to the message recieved in the topic.
  * @retval None
  */
void emergency_callback(const void * xMsgIn)
{
  const tier4_vehicle_msgs__msg__VehicleEmergencyStamped * emergency_msg_ = (const tier4_vehicle_msgs__msg__VehicleEmergencyStamped * )xMsgIn;
  ucSubscribersReceived = ucSubscribersReceived | (0b1 << 6);
}

// Service callbacks


/**
  * @name   control_mode_cmd_callback
  * @brief  ROS service server callback to change the control mode and return a status about the change
  * @param  xRequestMsg: pointer to the request made for the server.
  * @param  xResponseMsg: pointer to the response gave by the server.
  * @retval None
  */
void control_mode_cmd_callback(const void * xRequestMsg, autoware_auto_vehicle_msgs__srv__ControlModeCommand_Response * xResponseMsg)
{
  const autoware_auto_vehicle_msgs__srv__ControlModeCommand_Request * control_mode_request_msg_ = (const autoware_auto_vehicle_msgs__srv__ControlModeCommand_Request * )xRequestMsg;

  if(AUTOWARE == control_mode_request_msg_->mode)
  {
    xResponseMsg->success = true;
    osEventFlagsSet(EventsMicroAutowareHandle, MA_TO_AUTOWARE_MODE_FLAG);
    osEventFlagsSet(EventsMicroAutowareHandle, SYS_TO_AUTOWARE_MODE_FLAG);
  }
  else if(MANUAL == control_mode_request_msg_->mode)
  {
    xResponseMsg->success = true;
    osEventFlagsSet(EventsMicroAutowareHandle, MA_TO_MANUAL_MODE_FLAG);
    osEventFlagsSet(EventsMicroAutowareHandle, SYS_TO_MANUAL_MODE_FLAG);
  }
  else
  {
    xResponseMsg->success = false;
  }

}
