/**
  ******************************************************************************
  * @file    utils.c
  * @brief   This file contains utilities functions used in the project.
  ******************************************************************************
  * @author  Gabriel Toffanetto França da Rocha
  *          Laboratory of Autonomous Vehicles (LMA) - FEM/Unicamp
  * @date    Created:  October 23, 2024
  *          Modified:
  ******************************************************************************
  */

#include "utils.h"

/**
  * @name   fGetJoyPostition
  * @brief  Calculate the position of the joystick in [-1, 1] intervals.
  * @param  uiValue:  Analog value for axis
  * @param  uiRef0:   Analog zero value
  * @param  uiRefMax: Analog value for axis maximum limit.
  * @param  uiRefMin: Analog value for axis minimum limit.
  * @retval Float value with joystick position between -1 and 1.
  */
float fGetJoyPostition(unsigned int uiValue, unsigned int uiRef0, unsigned int uiRefMax, unsigned int uiRefMin)
{
  if((uiValue > (uiRef0 + JOY_DEAD_BAND)) || (uiValue < (uiRef0 - JOY_DEAD_BAND)))
  {
    return ((int)(uiValue - uiRef0) > 0) ? ((int)(uiValue - uiRef0 - JOY_DEAD_BAND))/((float)(uiRefMax - uiRef0 - JOY_DEAD_BAND)) : ((int)(uiValue - uiRef0 + JOY_DEAD_BAND))/((float)(uiRef0 - JOY_DEAD_BAND - uiRefMin));
  }
  else
  {
	return 0.0;
  }
}

/**
  * @name   ucGetStringFromControlAction
  * @brief  Converts the compressed data in control_action form to string in form
  *         #T%c%c%c%cS%c%c%c%cB%c%c%c%cH%cR%cG%cM%c$
  * @param  xControlActionTx: Compressed data with information to CARLA.
  * @param  ucTxMsg: Formated msg to be transmitted.
  * @retval None.
  */
void vGetStringFromControlAction(control_action xControlActionTx, unsigned char * ucTxMsg)
{
  ucTxMsg[0]  = (unsigned char)'#';
  ucTxMsg[1]  = (unsigned char)'S';
  ucTxMsg[2]  = xControlActionTx.xSteeringAngle.ucBytes[0];
  ucTxMsg[3]  = xControlActionTx.xSteeringAngle.ucBytes[1];
  ucTxMsg[4]  = xControlActionTx.xSteeringAngle.ucBytes[2];
  ucTxMsg[5]  = xControlActionTx.xSteeringAngle.ucBytes[3];
  ucTxMsg[6]  = (unsigned char)'W';
  ucTxMsg[7]  = xControlActionTx.xSteeringVelocity.ucBytes[0];
  ucTxMsg[8]  = xControlActionTx.xSteeringVelocity.ucBytes[1];
  ucTxMsg[9]  = xControlActionTx.xSteeringVelocity.ucBytes[2];
  ucTxMsg[10] = xControlActionTx.xSteeringVelocity.ucBytes[3];
  ucTxMsg[11] = (unsigned char)'V';
  ucTxMsg[12] = xControlActionTx.xSpeed.ucBytes[0];
  ucTxMsg[13] = xControlActionTx.xSpeed.ucBytes[1];
  ucTxMsg[14] = xControlActionTx.xSpeed.ucBytes[2];
  ucTxMsg[15] = xControlActionTx.xSpeed.ucBytes[3];
  ucTxMsg[16] = (unsigned char)'A';
  ucTxMsg[17] = xControlActionTx.xAcceleration.ucBytes[0];
  ucTxMsg[18] = xControlActionTx.xAcceleration.ucBytes[1];
  ucTxMsg[19] = xControlActionTx.xAcceleration.ucBytes[2];
  ucTxMsg[20] = xControlActionTx.xAcceleration.ucBytes[3];
  ucTxMsg[21] = (unsigned char)'J';
  ucTxMsg[22] = xControlActionTx.xJerk.ucBytes[0];
  ucTxMsg[23] = xControlActionTx.xJerk.ucBytes[1];
  ucTxMsg[24] = xControlActionTx.xJerk.ucBytes[2];
  ucTxMsg[25] = xControlActionTx.xJerk.ucBytes[3];
  ucTxMsg[26] = (unsigned char)'M';
  ucTxMsg[27] = xControlActionTx.ucControlMode;
  ucTxMsg[28] = (unsigned char)'$';
  ucTxMsg[29] = (unsigned char)'\0';
  
}

/**
  * @name   vDrivingModeLights
  * @brief  Update driving mode signaling lights
  * @param  ucDrivingMode:  Driving mode
  * @retval None
  */
void vDrivingModeLights(unsigned char ucDrivingMode)
{
  switch (ucDrivingMode)
  {
    case AUTOWARE:
      HAL_GPIO_WritePin(LD1_GPIO_Port,LD1_Pin, 0); // Green LED
      HAL_GPIO_WritePin(LD2_GPIO_Port,LD2_Pin, 1); // Ambar LED
      HAL_GPIO_WritePin(LD3_GPIO_Port,LD3_Pin, 0); // Red LED
      break;

    case MANUAL:
      HAL_GPIO_WritePin(LD1_GPIO_Port,LD1_Pin, 1); // Green LED
      HAL_GPIO_WritePin(LD2_GPIO_Port,LD2_Pin, 0); // Ambar LED
      HAL_GPIO_WritePin(LD3_GPIO_Port,LD3_Pin, 0); // Red LED
      break;

    case EMERGENCY:
      HAL_GPIO_WritePin(LD2_GPIO_Port,LD2_Pin, 0); // Green LED
      HAL_GPIO_WritePin(LD1_GPIO_Port,LD1_Pin, 0); // Ambar LED
      HAL_GPIO_WritePin(LD3_GPIO_Port,LD3_Pin, 1); // Red LED
      break;

    default:
      break;
  }

}
