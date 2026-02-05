/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "UART_print.h"
#include "mpu.h"
#include <string.h>
#include "fatfs.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
  uint32_t t_ms;
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
} data_read;

#define chunk_size 1024
data_read chunk[chunk_size];

osStatus_t st;

TickType_t period = pdMS_TO_TICKS(5);
TickType_t b1 = pdMS_TO_TICKS(500);
TickType_t b2 = pdMS_TO_TICKS(600);

int16_t ax, ay, az;
int16_t gx, gy, gz;

HAL_StatusTypeDef debugger_gyro;
HAL_StatusTypeDef debugger_acc;

extern I2C_HandleTypeDef hi2c2;

HAL_I2C_StateTypeDef i2c_st;
uint32_t err;
uint32_t isr;

extern uint8_t retSD;    /* Return value for SD */
extern char SDPath[4];   /* SD logical drive path */
extern FATFS SDFatFS;    /* File system object for SD logical drive */
extern FIL SDFile;       /* File object for SD */

FRESULT res;
uint32_t byteswritten, bytesread;

char* filename = "SD_file.txt";

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
/* Definitions for blink01 */
osThreadId_t blink01Handle;
const osThreadAttr_t blink01_attributes = {
  .name = "blink01",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow7,
};
/* Definitions for blink02 */
osThreadId_t blink02Handle;
const osThreadAttr_t blink02_attributes = {
  .name = "blink02",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow6,
};
/* Definitions for SensorRead */
osThreadId_t SensorReadHandle;
const osThreadAttr_t SensorRead_attributes = {
  .name = "SensorRead",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal1,
};
/* Definitions for DataProcess */
osThreadId_t DataProcessHandle;
const osThreadAttr_t DataProcess_attributes = {
  .name = "DataProcess",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for SD_write */
osThreadId_t SD_writeHandle;
const osThreadAttr_t SD_write_attributes = {
  .name = "SD_write",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for data_queue */
osMessageQueueId_t data_queueHandle;
const osMessageQueueAttr_t data_queue_attributes = {
  .name = "data_queue"
};
/* Definitions for uartMutex */
osMutexId_t uartMutexHandle;
const osMutexAttr_t uartMutex_attributes = {
  .name = "uartMutex"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartBlink01(void *argument);
void StartBlink02(void *argument);
void SensorReadTask(void *argument);
void DataProcessTask(void *argument);
void SD_write_task(void *argument);

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
  /* creation of uartMutex */
  uartMutexHandle = osMutexNew(&uartMutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of data_queue */
  data_queueHandle = osMessageQueueNew (16, sizeof(data_read), &data_queue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of blink01 */
  blink01Handle = osThreadNew(StartBlink01, NULL, &blink01_attributes);

  /* creation of blink02 */
  blink02Handle = osThreadNew(StartBlink02, NULL, &blink02_attributes);

  /* creation of SensorRead */
  SensorReadHandle = osThreadNew(SensorReadTask, NULL, &SensorRead_attributes);

  /* creation of DataProcess */
  DataProcessHandle = osThreadNew(DataProcessTask, NULL, &DataProcess_attributes);

  /* creation of SD_write */
  SD_writeHandle = osThreadNew(SD_write_task, NULL, &SD_write_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartBlink01 */
/**
  * @brief  Function implementing the blink01 thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartBlink01 */
void StartBlink01(void *argument)
{
  /* USER CODE BEGIN StartBlink01 */
  /* Infinite loop */
	TickType_t lastWakeTime = xTaskGetTickCount();
  for(;;)
  {
	HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
	vTaskDelayUntil(&lastWakeTime, b1);
  }
  osThreadTerminate(NULL);
  /* USER CODE END StartBlink01 */
}

/* USER CODE BEGIN Header_StartBlink02 */
/**
* @brief Function implementing the blink02 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartBlink02 */
void StartBlink02(void *argument)
{
  /* USER CODE BEGIN StartBlink02 */
  /* Infinite loop */
	TickType_t lastWakeTime = xTaskGetTickCount();
  for(;;)
  {
	HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    //osDelay(600);
	vTaskDelayUntil(&lastWakeTime, b2);
  }
  osThreadTerminate(NULL);
  /* USER CODE END StartBlink02 */
}

/* USER CODE BEGIN Header_SensorReadTask */
/**
* @brief Function implementing the SensorRead thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_SensorReadTask */
void SensorReadTask(void *argument)
{
  /* USER CODE BEGIN SensorReadTask */
	uint32_t t = 0;
	TickType_t lastWakeTime = xTaskGetTickCount();
  /* Infinite loop */
  for(;;)
  {
	  //dont need mutex since only i2c transaction
	  data_read s;
	  s.t_ms = osKernelGetTickCount();   // RTOS tick count in ms if tick=1ms
	  /* Debugging
	  i2c_st = HAL_I2C_GetState(&hi2c2);
	  err = HAL_I2C_GetError(&hi2c2);
	  isr = hi2c2.Instance->ISR;
	  */
	  debugger_acc = mpu_read_acc(&ax, &ay, &az);
	  s.ax = ax;
	  s.ay = ay;
	  s.az = az;

	  debugger_gyro = mpu_read_gyro(&gx, &gy, &gz);
	  s.gx = gz;
	  s.gy = gy;
	  s.gz = gz;

	  st = osMessageQueuePut(data_queueHandle, &s, 0, 0);

	  if (st != osOK) {

	  }

	  t++;
	  //osDelay(5); // 200 Hz-ishosDelay(1);
	  vTaskDelayUntil(&lastWakeTime, period);
  }
  osThreadTerminate(NULL);
  /* USER CODE END SensorReadTask */
}

/* USER CODE BEGIN Header_DataProcessTask */
/**
* @brief Function implementing the DataProcess thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_DataProcessTask */
void DataProcessTask(void *argument)
{
  /* USER CODE BEGIN DataProcessTask */
  /* Infinite loop */
	data_read s;
	uint32_t count = 0;
  for(;;)
  {
	  if (osMessageQueueGet(data_queueHandle, &s, NULL, osWaitForever) == osOK)
	  {
		count++;
		if ((count % 200) == 0)
		{
			char msg[64];
			snprintf(msg, sizeof(msg), "t=%lu ax=%d az=%d\r\n", (unsigned long)s.t_ms, s.ax, s.az);

		  print_thread(msg);
		}
	  }
  }
  osThreadTerminate(NULL);
  /* USER CODE END DataProcessTask */
}

/* USER CODE BEGIN Header_SD_write_task */
/**
* @brief Function implementing the SD_write thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_SD_write_task */
void SD_write_task(void *argument)
{
  /* USER CODE BEGIN SD_write_task */
	TickType_t lastWakeTime = xTaskGetTickCount();
	uint8_t wtext[] = "Hello World\n";
	uint8_t rtext[100];

	//mount SD card
	if (f_mount(&SDFatFS, (TCHAR const*)SDPath, 0) != FR_OK) {
	  //Error
	  print_thread("Error Mounting SD card\n");
	}
	else {
	  print_thread("SD card mounted successfully\n");
	}

	vTaskDelayUntil(&lastWakeTime, b2);

	//open file for writing
	if (f_open(&SDFile, filename, FA_CREATE_ALWAYS | FA_WRITE) != FR_OK){
		print_thread("Error opening file\n");
	}
	else {
		print_thread("File opened successfully\n");

		//write data to file to test
		res = f_write(&SDFile, wtext, strlen((char*)wtext), (void *)&byteswritten);
		if (byteswritten == 0 || (res != FR_OK)){
			print_thread("Failed to write file\n");
		}
		else {
			char msg[256];
			snprintf(msg, sizeof(msg), "Write content: %s\n", wtext);

			print_thread(msg);
		}
		f_close(&SDFile);
	}

	//test read file
	f_open(&SDFile, filename, FA_READ);
	memset(rtext, 0, sizeof(rtext));
	res = f_read(&SDFile, rtext, sizeof(rtext), (UINT*)&bytesread);

	if (bytesread== 0 || (res != FR_OK)){
		print_thread("Failed to read file\n");
	}
	else {
		char msg[256];
		snprintf(msg, sizeof(msg), "Read content: %s\n", (char *)rtext);
		print_thread(msg);
	}
	f_close(&SDFile);

  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  osThreadTerminate(NULL);
  /* USER CODE END SD_write_task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

