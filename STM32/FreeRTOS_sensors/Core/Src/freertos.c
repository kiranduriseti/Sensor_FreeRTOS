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
#include "usart.h"
#include "crc.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct __attribute__((packed)) {
  uint32_t t_ms;
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
} data_read;

#define chunk_bytes 4096
#define SD_QUEUE_TIMEOUT 1000
#define max_samples (chunk_bytes/(sizeof(data_read)))
uint32_t sd_chunk_len = 0;

__attribute__((aligned(4))) uint8_t sd_chunk[chunk_bytes]; //forces it to be on 4 but boundaries

osStatus_t st;
osStatus_t st_sd;

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

char* filename = "log.bin";

uint32_t g_produced = 0;
uint32_t g_dropped  = 0;
uint32_t g_max_qdepth = 0;

uint32_t g_data_dropped = 0;

extern CRC_HandleTypeDef hcrc;

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
/* Definitions for status */
osThreadId_t statusHandle;
const osThreadAttr_t status_attributes = {
  .name = "status",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for data_queue */
osMessageQueueId_t data_queueHandle;
const osMessageQueueAttr_t data_queue_attributes = {
  .name = "data_queue"
};
/* Definitions for sd_queue */
osMessageQueueId_t sd_queueHandle;
const osMessageQueueAttr_t sd_queue_attributes = {
  .name = "sd_queue"
};
/* Definitions for uartMutex */
osMutexId_t uartMutexHandle;
const osMutexAttr_t uartMutex_attributes = {
  .name = "uartMutex"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

void sd_chunk_append_record(const data_read *rec)
{
  const uint32_t rec_sz = sizeof(*rec);
  memcpy(&sd_chunk[sd_chunk_len], rec, rec_sz);
  sd_chunk_len += rec_sz;
}

uint32_t calc_pad_length(uint32_t length){
	return ((4 - (length & 3)) & 3);
}

uint32_t calculate_crc32(uint8_t *data, uint32_t length_bytes){
	//__HAL_CRC_DR_RESET(&hcrc);

	uint32_t crc = HAL_CRC_Calculate(&hcrc, (uint32_t *)data, length_bytes);

	return crc;
}

FRESULT sd_flush_chunk(void)
{
  if (sd_chunk_len == 0) return FR_OK;

  UINT bw = 0;
  FRESULT r = f_write(&SDFile, sd_chunk, sd_chunk_len, &bw);

  if ((r != FR_OK) || (bw != sd_chunk_len)) {
    print_thread("SD flush failed: data\r\n");
    return (r != FR_OK) ? r : FR_INT_ERR;
  }

  //uint32_t pad_length = calc_pad_length(sd_chunk_len); HAL handles padding itself

//  for (int i = 0; i < pad_length; i++) {
//	  sd_chunk[sd_chunk_len + i] = 0;
//  }
  //pad_length = (sd_chunk_len + pad_length); //length in bytes because byte mode
  //uint32_t crc = calculate_crc32(sd_chunk, pad_length);

  uint32_t crc = calculate_crc32(sd_chunk, sd_chunk_len);

  uint8_t validation_arr[8];
  memcpy(&validation_arr[0], &crc, 4);
  memcpy(&validation_arr[4], &sd_chunk_len, 4);

  //writes crc and original length (python can calculate amount of padded bits and act accordingly)

  r = f_write (&SDFile, validation_arr, 8, &bw);

  if ((r != FR_OK) || (bw != 8)) {
	  print_thread("SD flush failed: crc\r\n");
	  return (r != FR_OK) ? r : FR_INT_ERR;
  }

  sd_chunk_len = 0;
  //memset(&sd_chunk[sd_chunk_len], 0, chunk_bytes);

  // f_sync(&SDFile); //force metadata update more often (slower but safer)

  return FR_OK;
}


/* USER CODE END FunctionPrototypes */

void StartBlink01(void *argument);
void StartBlink02(void *argument);
void SensorReadTask(void *argument);
void DataProcessTask(void *argument);
void SD_write_task(void *argument);
void status_task(void *argument);

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

  /* creation of sd_queue */
  sd_queueHandle = osMessageQueueNew (16, sizeof(data_read), &sd_queue_attributes);

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

  /* creation of status */
  statusHandle = osThreadNew(status_task, NULL, &status_attributes);

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
	  s.gx = gx;
	  s.gy = gy;
	  s.gz = gz;

	  st_sd = osMessageQueuePut(sd_queueHandle, &s, 0, 0);
	  st = osMessageQueuePut(data_queueHandle, &s, 0, 0);

//	  if (st == osOK) {
//	    g_produced++;
//
//	    uint32_t depth = osMessageQueueGetCount(data_queueHandle);
//	    if (depth > g_max_qdepth) g_max_qdepth = depth;
//
//	  } else {
//	    // error -> count as dropped
//	    g_dropped++;
//	  }
	  if (st != osOK) g_data_dropped++;

	  if (st_sd == osOK) {
	    g_produced++;
	    uint32_t depth = osMessageQueueGetCount(sd_queueHandle);
	    if (depth > g_max_qdepth) g_max_qdepth = depth;

	  }
	  else {
	    g_dropped++;
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
	//remove Hello World Test when logging, just to ensure proper wiring

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

	if (f_open(&SDFile, filename, FA_OPEN_ALWAYS | FA_WRITE) != FR_OK) {
	print_thread("Error opening file for append/logging\n");
	vTaskDelete(NULL);
  }

	f_lseek(&SDFile, f_size(&SDFile));
	print_thread("SD file opened (append)\r\n");

	data_read s;
	const uint32_t rec_sz = sizeof(s);

	TickType_t last_flush = xTaskGetTickCount();
	const TickType_t flush_period = pdMS_TO_TICKS(1000);
  /* Infinite loop */
  for(;;)
  {
	  osStatus_t ok = osMessageQueueGet(sd_queueHandle, &s, NULL, SD_QUEUE_TIMEOUT);

	  if (ok == osOK)
	  {
		if (sd_chunk_len + rec_sz > chunk_bytes) { //too big
		  if (sd_flush_chunk() != FR_OK) {
			// no more writting
		  }
		  last_flush = xTaskGetTickCount();
		}

		sd_chunk_append_record(&s);
	  }

	  // Periodic flush
	  if ((xTaskGetTickCount() - last_flush) >= flush_period) {
		(void)sd_flush_chunk();
		(void)f_sync(&SDFile);   // ensures data is committed periodically (slower but safer)
		last_flush = xTaskGetTickCount();
	  }

	  if (g_stop_req) {
	      print_thread("[LOG] STOP received, draining queue...\r\n");

	      // Drain remaining items quickly (non-blocking)
	      while (osMessageQueueGet(sd_queueHandle, &s, NULL, 0) == osOK) {
	        if (sd_chunk_len + rec_sz > chunk_bytes) {
	          if (sd_flush_chunk() != FR_OK) break;
	        }
	        sd_chunk_append_record(&s);
	      }

	      print_thread("[LOG] Flushing chunk...\r\n");
	      (void)sd_flush_chunk();
	      (void)f_sync(&SDFile);

	      print_thread("[LOG] Closing file...\r\n");
	      f_close(&SDFile);

	      print_thread("[LOG] Unmounting...\r\n");
	      f_mount(NULL, (TCHAR const*)SDPath, 0);

	      print_thread("[LOG] SAFE TO REMOVE / SWITCH FIRMWARE\r\n");

	      vTaskSuspend(NULL);
	    }
  }
  osThreadTerminate(NULL);
  /* USER CODE END SD_write_task */
}

/* USER CODE BEGIN Header_status_task */
/**
* @brief Function implementing the status thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_status_task */
void status_task(void *argument)
{
  /* USER CODE BEGIN status_task */
  /* Infinite loop */
	const TickType_t one_sec = pdMS_TO_TICKS(1000);
	TickType_t lastWake = xTaskGetTickCount();

	uint32_t prev_prod = 0;
	uint32_t prev_drop = 0;

	uint32_t prev_data_drop = 0;


  for(;;)
  {
	  taskENTER_CRITICAL();
	  uint32_t prod = g_produced;
	  uint32_t drop = g_dropped;
	  uint32_t maxd = g_max_qdepth;
	  uint32_t data_drop = g_data_dropped;
	  taskEXIT_CRITICAL();

	  uint32_t rate = prod - prev_prod;     // samples/sec produced
	  uint32_t dps  = drop - prev_drop;     // drops/sec

	  uint32_t data_dps = data_drop - prev_data_drop;

	  prev_prod = prod;
	  prev_drop = drop;
	  prev_data_drop = data_drop;

	  uint32_t qd_count = osMessageQueueGetCount(data_queueHandle);
	  uint32_t qd_cap   = osMessageQueueGetCapacity(data_queueHandle);

	  uint32_t qs_count = osMessageQueueGetCount(sd_queueHandle);
	  uint32_t qs_cap   = osMessageQueueGetCapacity(sd_queueHandle);


	  size_t heap_free = xPortGetFreeHeapSize();

	  UBaseType_t sw_sensor  = uxTaskGetStackHighWaterMark((TaskHandle_t)SensorReadHandle);
	  UBaseType_t sw_proc    = uxTaskGetStackHighWaterMark((TaskHandle_t)DataProcessHandle);
	  UBaseType_t sw_sd      = uxTaskGetStackHighWaterMark((TaskHandle_t)SD_writeHandle);
	  UBaseType_t sw_status  = uxTaskGetStackHighWaterMark((TaskHandle_t)statusHandle);

	  char msg[256];
//	  snprintf(msg, sizeof(msg),
//		"[STAT] rate=%lu/s drops=%lu/s  q=%lu/%lu (max=%lu)  heap_free=%u  stack_wm(w): SR=%lu DP=%lu SD=%lu ST=%lu\r\n",
//		(unsigned long)rate,
//		(unsigned long)dps,
//		(unsigned long)q_count,
//		(unsigned long)q_cap,
//		(unsigned long)maxd,
//		(unsigned)heap_free,
//		(unsigned long)sw_sensor,
//		(unsigned long)sw_proc,
//		(unsigned long)sw_sd,
//		(unsigned long)sw_status
//	  );
	  snprintf(msg, sizeof(msg),
	    "[STAT] rate=%lu/s  sdDrops=%lu/s  dataDrops=%lu/s  "
	    "dataQ=%lu/%lu  sdQ=%lu/%lu (sdQmax=%lu)  "
	    "heap_free=%u  stack_wm(w): SR=%lu DP=%lu SD=%lu ST=%lu\r\n",

	    (unsigned long)rate,
	    (unsigned long)dps,
	    (unsigned long)data_dps,

	    (unsigned long)qd_count,
	    (unsigned long)qd_cap,

	    (unsigned long)qs_count,
	    (unsigned long)qs_cap,
	    (unsigned long)maxd,

	    (unsigned)heap_free,

	    (unsigned long)sw_sensor,
	    (unsigned long)sw_proc,
	    (unsigned long)sw_sd,
	    (unsigned long)sw_status
	  );

	  print_thread(msg);

	  vTaskDelayUntil(&lastWake, one_sec);
}
  osThreadTerminate(NULL);
  /* USER CODE END status_task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

