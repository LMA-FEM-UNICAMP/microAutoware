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
#include "taskControl.h"

// ADC1 buffer for channels 2 and 6.
unsigned int uiADCBuffer[2];

extern osThreadId_t TaskControlHandle;


// Control action struct with high level control action from MicroAutoware to TaskControle,
// for compute the vehicle control action. [From microautoware.c].
extern control_action xControlAction;

// Vehicle status struct with low level control signal from TaskControle to CARLA,
// for publish in simulator topics by micro-ros. [From microautoware.c].
extern vehicle_status xVehicleStatus;

// Data structure for data received from CARLA by UART2
extern vehicle_status xVehicleData;

// On press JoySW tick counter for JoySW debounce delay
unsigned int uiJoySWTickOnPress = 0;

// Buffer for data received from CARLA by UART2
unsigned char ucDmaBuffer[UART2_DMA_BUFFER_SIZE];

// Data structure for data received from CARLA by UART2
vehicle_status xVehicleData;

/**
  * @name   StartTaskControle
  * @brief  TaskControle task function.
  * @param  argument: not used.
  * @retval None
  */
void StartTaskControl(void * argument)
{

  // Local variables -- START
  unsigned char ucControlMode;

  // Joystick calibration -- Keep as variable for future auto-calibration implementation
  unsigned int uiX0   = 33970;
  unsigned int uiXMin = 1057;
  unsigned int uiXMax = 65535;
  unsigned int uiY0   = 33580;
  unsigned int uiYMin = 1062;
  unsigned int uiYMax = 65535;

  // Return flags
  unsigned int uiFlags = 0;

  // Deadline lost counter for control command msg
  unsigned char ucNumberOfLostMessageCtlCmd = 0;

  // Deadline lost counter for status msg
  unsigned char ucNumberOfLostMessageStatus = 0;

  // Joystick reading
  float fJoyXAxis;
  float fJoyYAxis;

  // Message buffer to CARLA by UART2
  unsigned char ucTxMsgToCarla[MSG_TO_CARLA_SIZE];

  // Local variables -- END

  // Starting ADC reading by DMA
  HAL_ADC_Start_DMA(&HADC, (uint32_t * ) uiADCBuffer, 2);

  // Initialization of DMA RX in single mode
  HAL_UART_Receive_DMA(&HUART_CARLA, ucDmaBuffer, UART2_DMA_BUFFER_SIZE);

  // Initialization of operation mode
  ucControlMode = AUTOWARE;

  // Waiting for micro-ROS connect to agent
  uiFlags = osEventFlagsGet(EventsMicroAutowareHandle);
  uiFlags = osEventFlagsWait(EventsMicroAutowareHandle, MICRO_ROS_AGENT_ONLINE_FLAG, osFlagsWaitAny, 1000 * TIMEOUT_MICRO_ROS_AGENT); // Wait 20 seconds for uROS init

  // If micro-ROS agent is unvailable, start in MANUAL mode
  if(osFlagsErrorTimeout == uiFlags)
  {
    ucControlMode = MANUAL;
  }

  // Task loop
  for(;;)
  {

    // Looking for operation mode change by Autoware -- START
	uiFlags = osEventFlagsGet(EventsMicroAutowareHandle);
    uiFlags = osEventFlagsWait(EventsMicroAutowareHandle, SYS_TO_AUTOWARE_MODE_FLAG | SYS_TO_MANUAL_MODE_FLAG, osFlagsWaitAny, 0);

    if(CHECK_FLAG(SYS_TO_AUTOWARE_MODE_FLAG, uiFlags))
    {
      ucControlMode = AUTOWARE;
    }
    else if(CHECK_FLAG(SYS_TO_MANUAL_MODE_FLAG, uiFlags))
    {
      ucControlMode = MANUAL;
    }
    else if(CHECK_FLAG((SYS_TO_AUTOWARE_MODE_FLAG | SYS_TO_MANUAL_MODE_FLAG), uiFlags))
    {
      ucControlMode = MANUAL;
    }
    // Looking for operation mode change by Autoware -- END

    // Looking for operation mode change by JoySW -- START
    uiFlags = osThreadFlagsGet();
    uiFlags = osThreadFlagsWait(JOYSW_FLAG, osFlagsWaitAll, 0);

    if(CHECK_FLAG(JOYSW_FLAG, uiFlags))
    {
      if(AUTOWARE == ucControlMode)
      {
        ucControlMode = MANUAL;
        osEventFlagsSet(EventsMicroAutowareHandle, MA_TO_MANUAL_MODE_FLAG);
      }
      else if(MANUAL == ucControlMode)
      {
        ucControlMode = AUTOWARE;
        osEventFlagsSet(EventsMicroAutowareHandle, MA_TO_AUTOWARE_MODE_FLAG);
      }
    }
    // Looking for operation mode change by JoySW -- END

    // Autonomous mode (AUTOWARE) routine -- START
    if(AUTOWARE == ucControlMode)
    {
      // Setting driving mode lights
	    vDrivingModeLights(ucControlMode);

      // WAIT for flag to sync xControlAction update
  	  uiFlags = osEventFlagsGet(EventsMicroAutowareHandle);
  	  uiFlags = osEventFlagsWait(EventsMicroAutowareHandle, AUTOWARE_DATA_UPDATED_FLAG, osFlagsWaitAll, TIMEOUT_GET_CONTROL_ACTION);

      // Timeout error -- deadline lost
      if(osFlagsErrorTimeout == uiFlags)
      {
        // Increment the lost data counter
        ucNumberOfLostMessageCtlCmd++;

        // Check if the max of data lost was got
        if(ucNumberOfLostMessageCtlCmd >= MAX_OF_LOST_MESSAGES) // If yes, change to manual
        {
          ucControlMode = MANUAL;
          osEventFlagsSet(EventsMicroAutowareHandle, MA_TO_MANUAL_MODE_FLAG);
          ucNumberOfLostMessageCtlCmd = 0;
        }
        else // If not, sends the same command again
        {
          HAL_UART_Transmit_DMA(&HUART_CARLA, ucTxMsgToCarla, MSG_TO_CARLA_SIZE);
        }
      }
      else if(CHECK_FLAG(AUTOWARE_DATA_UPDATED_FLAG, uiFlags))
      {
	    ucNumberOfLostMessageCtlCmd = 0;

    	  // Reshaping control command to array of bytes
        osMutexAcquire(MutexControlActionHandle, osWaitForever);

        // Remapping variables for CARLA Bridge
        xControlAction.xSteeringAngle.fFloat *= 1.2;
        xControlAction.xSteeringVelocity.fFloat *= 1.2;

        vGetStringFromControlAction(xControlAction, ucTxMsgToCarla);

        osMutexRelease(MutexControlActionHandle);

        // Send ucTxMsgToCarla to CARLA
        HAL_UART_Transmit_DMA(&HUART_CARLA, ucTxMsgToCarla, MSG_TO_CARLA_SIZE);

        // Wait recieve CARLA full msg xVehicleStatusRx
        uiFlags = osThreadFlagsGet();
        uiFlags = osThreadFlagsWait(UART_NEW_DATA_FLAG, osFlagsWaitAll, TIMEOUT_GET_CARLA_RX);

        // Timeout error -- deadline lost
        if(osFlagsErrorTimeout == uiFlags)
        {
          // Increment the lost data counter
          ucNumberOfLostMessageStatus++;

          // Check if the max of data lost was got
          if(ucNumberOfLostMessageStatus >= MAX_OF_LOST_MESSAGES) // If yes, change to manual
          {
        	ucControlMode = EMERGENCY;
        	osEventFlagsSet(EventsMicroAutowareHandle, MA_TO_EMERGENCY_MODE_FLAG);
      	    ucNumberOfLostMessageStatus = 0;
          }
          else // If not, sends the same command again
          {
        	osEventFlagsSet(EventsMicroAutowareHandle, VEHICLE_DATA_UPDATED_FLAG);
          }
        }
        else if(CHECK_FLAG(UART_NEW_DATA_FLAG, uiFlags))
        {
          ucNumberOfLostMessageStatus = 0;

          // Fitting CARLA recieved data in xVehicleStatus way for microAutoware
          osMutexAcquire(MutexVehicleStatusHandle, osWaitForever);

          xVehicleStatus = xVehicleData;

          osMutexRelease(MutexVehicleStatusHandle);

          // Sync new data with microAutoware
          osEventFlagsSet(EventsMicroAutowareHandle, VEHICLE_DATA_UPDATED_FLAG);
        }
      }
    }
    // Autonomous mode (AUTOWARE) routine -- END


    // Manual mode (MANUAL) routine -- START
    if(MANUAL == ucControlMode)
    {
      // Setting driving mode lights
  	  vDrivingModeLights(ucControlMode);

      // Joystick read block -- START
      fJoyXAxis = fGetJoyPostition((unsigned int) uiADCBuffer[0], uiX0, uiXMax, uiXMin);
      fJoyYAxis = fGetJoyPostition((unsigned int) uiADCBuffer[1], uiY0, uiYMax, uiYMin);

      // Assembling xControlAction with joystick data
      osMutexAcquire(MutexControlActionHandle, osWaitForever);

      xControlAction.xSteeringAngle.fFloat = -fJoyXAxis*MAX_STEERING_ANGLE; 
      xControlAction.xSteeringVelocity.fFloat = 0;
      xControlAction.xSpeed.fFloat = (fJoyYAxis > 0) ? fJoyYAxis*MAX_THROTTLE : 0.0; // Throttle
      xControlAction.xAcceleration.fFloat = (fJoyYAxis < 0) ? -fJoyYAxis*MAX_BRAKE : 0.0; // Brake
      xControlAction.xJerk.fFloat = 0;
      xControlAction.ucControlMode = MANUAL;

      // Reshaping control command to array of bytes
      vGetStringFromControlAction(xControlAction, ucTxMsgToCarla);

      osMutexRelease(MutexControlActionHandle);

      // Send ucTxMsgToCarla to CARLA
      HAL_UART_Transmit_DMA(&HUART_CARLA, ucTxMsgToCarla, MSG_TO_CARLA_SIZE);

      // Wait recieve CARLA full msg xVehicleStatusRx
  	  uiFlags = osEventFlagsGet(EventsMicroAutowareHandle);
  	  uiFlags = osEventFlagsWait(EventsMicroAutowareHandle, UART_NEW_DATA_FLAG, osFlagsWaitAll, TIMEOUT_GET_CARLA_RX);

      // Timeout error
      if(osFlagsErrorTimeout == uiFlags)
      {
      // Increment the lost data counter
      ucNumberOfLostMessageStatus++;

      // Check if the max of data lost was got
      if(ucNumberOfLostMessageStatus >= MAX_OF_LOST_MESSAGES) // If yes, change to manual
      {
        ucControlMode = EMERGENCY;
        osEventFlagsSet(EventsMicroAutowareHandle, MA_TO_EMERGENCY_MODE_FLAG);
        ucNumberOfLostMessageStatus = 0;
      }
      else // If not, sends the same command again
      {
    	osEventFlagsSet(EventsMicroAutowareHandle, VEHICLE_DATA_UPDATED_FLAG);
      }
      }
        else if(CHECK_FLAG(UART_NEW_DATA_FLAG, uiFlags))
      {
        ucNumberOfLostMessageStatus = 0;

        // Assembling xVehicleStatus
        osMutexAcquire(MutexVehicleStatusHandle, osWaitForever);

        xVehicleStatus = xVehicleData;

        osMutexRelease(MutexVehicleStatusHandle);

        osEventFlagsSet(EventsMicroAutowareHandle, VEHICLE_DATA_UPDATED_FLAG);
	    }

      // WAIT for send other joystick command
      osDelay(MANUAL_CONTROL_TIME_COMMAND);
    }
    // Manual mode (MANUAL) routine -- END

    // Emergency mode (EMERGENCY) routine -- START
    if(EMERGENCY == ucControlMode)
    {
      // Setting driving mode lights
  	  vDrivingModeLights(ucControlMode);
    
      // Assemble xControlAction to stop the vehicle
      osMutexAcquire(MutexControlActionHandle, osWaitForever);

      xControlAction.xSteeringAngle.fFloat = 0.0;
      xControlAction.xSteeringVelocity.fFloat = 0.0;
      xControlAction.xSpeed.fFloat = 0.0;
      xControlAction.xAcceleration.fFloat = 0.0;
      xControlAction.xJerk.fFloat = 0.0;
      xControlAction.ucControlMode = EMERGENCY;

      // Reshaping control command to array of bytes
      vGetStringFromControlAction(xControlAction, ucTxMsgToCarla);

      osMutexRelease(MutexControlActionHandle);

      // Send ucTxMsgToCarla to CARLA
      HAL_UART_Transmit_DMA(&HUART_CARLA, ucTxMsgToCarla, MSG_TO_CARLA_SIZE);

      // Wait recieve CARLA full msg xVehicleStatusRx
  	  uiFlags = osThreadFlagsGet();
      uiFlags = osThreadFlagsWait(UART_NEW_DATA_FLAG, osFlagsWaitAll, TIMEOUT_GET_CARLA_RX);

      // If data from CARLA arrives, control mode returns to MANUAL
      if(CHECK_FLAG(UART_NEW_DATA_FLAG, uiFlags))
      {
        ucNumberOfLostMessageCtlCmd = 0;
        ucNumberOfLostMessageStatus = 0;
        ucControlMode = MANUAL;
        osEventFlagsSet(EventsMicroAutowareHandle, MA_TO_MANUAL_MODE_FLAG);
      }
      
    }
    // Emergency mode (EMERGENCY) routine -- END

  }


}

// System ISRs -- START

/**
  * @name   HAL_GPIO_EXTI_Callback
  * @brief  ISR callback for the JoySW, switching the control mode.
  * @param  GPIO_Pin: EXTI pin.
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(JoySW_Pin == GPIO_Pin){

    unsigned int uiTick = osKernelGetTickCount();

    if(uiTick > (uiJoySWTickOnPress + DEBOUNCE_TICKS)) // DEBOUNCE_TICKS debounce
    {
      uiJoySWTickOnPress = uiTick;
      osThreadFlagsSet(TaskControlHandle, JOYSW_FLAG);
    }
  }
}

/**
  * @name   HAL_UART_RxCpltCallback
  * @brief  ISR callback for reading msg from UART with UART2_DMA_BUFFER_SIZE bytes.
  * @param  huart: Handle for serial UART
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef * huart)
{
  if(&HUART_CARLA == huart)
  {

	// State machine state
	unsigned int ucSmState = 0;

	for(unsigned char i = 0; i<UART2_DMA_BUFFER_SIZE; i++)
	{
	  switch (ucSmState)
	  {
		case 0:
		  if('#' == ucDmaBuffer[i])
		  {
			ucSmState = 1;
		  }
		  break;

		case 1:
		  switch (ucDmaBuffer[i])
		  {
			case 'A':
			  ucSmState = 10;
			  break;

			case 'B':
			  ucSmState = 20;
			  break;

			case 'C':
			  ucSmState = 30;
			  break;

			case 'D':
			  ucSmState = 40;
			  break;

			case '$':
			  ucSmState = 0;
			  // Message fully received, setting TaskControle ThreadFlag for sync.
			  osThreadFlagsSet(TaskControlHandle, UART_NEW_DATA_FLAG);
			  break;

			default:
			  ucSmState = 0;
			  break;
		  }
		  break;

		case 10:
		  xVehicleData.xLongSpeed.ucBytes[0] = ucDmaBuffer[i];
		  ucSmState = 11;
		  break;

		case 11:
		  xVehicleData.xLongSpeed.ucBytes[1] = ucDmaBuffer[i];
		  ucSmState = 12;
		  break;

		case 12:
		  xVehicleData.xLongSpeed.ucBytes[2] = ucDmaBuffer[i];
		  ucSmState = 13;
		  break;

		case 13:
		  xVehicleData.xLongSpeed.ucBytes[3] = ucDmaBuffer[i];
		  ucSmState = 1;
		  break;

		case 20:
		  xVehicleData.xLatSpeed.ucBytes[0] = ucDmaBuffer[i];
		  ucSmState = 21;
		  break;

		case 21:
		  xVehicleData.xLatSpeed.ucBytes[1] = ucDmaBuffer[i];
		  ucSmState = 22;
		  break;

		case 22:
		  xVehicleData.xLatSpeed.ucBytes[2] = ucDmaBuffer[i];
		  ucSmState = 23;
		  break;

		case 23:
		  xVehicleData.xLatSpeed.ucBytes[3] = ucDmaBuffer[i];
		  ucSmState = 1;
		  break;

		case 30:
		  xVehicleData.xHeadingRate.ucBytes[0] = ucDmaBuffer[i];
		  ucSmState = 31;
		  break;

		case 31:
		  xVehicleData.xHeadingRate.ucBytes[1] = ucDmaBuffer[i];
		  ucSmState = 32;
		  break;

		case 32:
		  xVehicleData.xHeadingRate.ucBytes[2] = ucDmaBuffer[i];
		  ucSmState = 33;
		  break;

		case 33:
		  xVehicleData.xHeadingRate.ucBytes[3] = ucDmaBuffer[i];
		  ucSmState = 1;
		  break;

		case 40:
		  xVehicleData.xSteeringStatus.ucBytes[0] = ucDmaBuffer[i];
		  ucSmState = 41;
		  break;

		case 41:
		  xVehicleData.xSteeringStatus.ucBytes[1] = ucDmaBuffer[i];
		  ucSmState = 42;
		  break;

		case 42:
		  xVehicleData.xSteeringStatus.ucBytes[2] = ucDmaBuffer[i];
		  ucSmState = 43;
		  break;

		case 43:
		  xVehicleData.xSteeringStatus.ucBytes[3] = ucDmaBuffer[i];
		  ucSmState = 1;
		  break;

		default:
			  ucSmState = 0;
		  break;
	  }
	}
	// Starting other UART reading
	HAL_UART_Receive_DMA(&HUART_CARLA, ucDmaBuffer, UART2_DMA_BUFFER_SIZE);
  }
}

// System ISRs -- END

