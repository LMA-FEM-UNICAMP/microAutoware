/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    freertos.c
  * @brief   Base code for FreeRTOS applcation. Declaration of tasks, global
  *          variables, kernel objects, typedefs and project libraries.
  ******************************************************************************
  * @author  Gabriel Toffanetto França da Rocha 
  *          Laboratory of Autonomous Vehicles (LMA) - FEM/Unicamp
  * @date    Created:  October 9, 2024
  *          Modified: 
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include <taskControl.h>
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

// Tasks includes -- START
#include "microAutoware.h"

// Tasks includes -- END

// Libraries includes -- START
#include "light_printf.h"
#include "utils.h"

// Libraries includes -- END

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for TaskControl */
osThreadId_t TaskControlHandle;
const osThreadAttr_t TaskControl_attributes = {
  .name = "TaskControl",
  .stack_size = 1500 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for TaskMicroAutowa */
osThreadId_t TaskMicroAutowaHandle;
const osThreadAttr_t TaskMicroAutowa_attributes = {
  .name = "TaskMicroAutowa",
  .stack_size = 4500 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for MutexVehicleStatus */
osMutexId_t MutexVehicleStatusHandle;
const osMutexAttr_t MutexVehicleStatus_attributes = {
  .name = "MutexVehicleStatus"
};
/* Definitions for MutexControlAction */
osMutexId_t MutexControlActionHandle;
const osMutexAttr_t MutexControlAction_attributes = {
  .name = "MutexControlAction"
};
/* Definitions for EventsMicroAutoware */
osEventFlagsId_t EventsMicroAutowareHandle;
const osEventFlagsAttr_t EventsMicroAutoware_attributes = {
  .name = "EventsMicroAutoware"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */



/* USER CODE END FunctionPrototypes */

void StartTaskControl(void *argument);
extern void StartMicroAutoware(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* creation of MutexVehicleStatus */
  MutexVehicleStatusHandle = osMutexNew(&MutexVehicleStatus_attributes);

  /* creation of MutexControlAction */
  MutexControlActionHandle = osMutexNew(&MutexControlAction_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of TaskControl */
  TaskControlHandle = osThreadNew(StartTaskControl, NULL, &TaskControl_attributes);

  /* creation of TaskMicroAutowa */
  TaskMicroAutowaHandle = osThreadNew(StartMicroAutoware, NULL, &TaskMicroAutowa_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* creation of EventsMicroAutoware */
  EventsMicroAutowareHandle = osEventFlagsNew(&EventsMicroAutoware_attributes);

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartTaskControl */
/**
  * @brief  Function implementing the TaskControl thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartTaskControl */
__weak void StartTaskControl(void *argument)
{
  /* USER CODE BEGIN StartTaskControl */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartTaskControl */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */


/* USER CODE END Application */

