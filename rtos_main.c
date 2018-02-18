/**
 * rtos_main.c
 * created by
 * Luis Roberto Lomeli IE700093
 * Jorge Mizael Rodriguez IE698983
 */
#include "board.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_debug_console.h"

#include "rtos.h"

/*!
 * @brief Start its counter and increments it every 2 seconds
 *
 * @param none
 * @retval none
 */
void dummy_task1(void);

/*!
 * @brief Start its counter and increments it every second
 *
 * @param none
 * @retval none
 */
void dummy_task2(void);

/*!
 * @brief Start its counter and increments it every 4 seconds
 *
 * @param none
 * @retval none
 */
void dummy_task3(void);

/*!
 * @brief Creates all tasks and initialize the scheduler
 *
 * @param none
 * @retval none
 */
void init_operating_system(void);

int main(void)
{
	BOARD_InitPins();
	BOARD_BootClockRUN();
	BOARD_InitDebugConsole();

	init_operating_system();

	for (;;)
	{
		__asm("NOP");
	}
}

/*!
 * @brief Creates all tasks and initialize the scheduler
 *
 * @param none
 * @retval none
 */
void init_operating_system(void)
{

	static uint16_t error;

	error += (uint16_t) rtos_create_task(dummy_task1, PRIORITY1, kAutoStart);
	error += (uint16_t) rtos_create_task(dummy_task2, PRIORITY2, kAutoStart);
	error += (uint16_t) rtos_create_task(dummy_task3, PRIORITY1, kAutoStart);
	if(ALL_TASKS_CREATED_CORRECTLY < error)
	{
		PRINTF("Error while starting OS try to delete a task\n");
		return;
	}

	rtos_start_scheduler();

}

/*!
 * @brief Start its counter and increments it every 2 seconds
 *
 * @param none
 * @retval none
 */
void dummy_task1(void)
{
	uint8_t counter = 0;
	for (;;)
	{
		PRINTF("IN TASK 1: %i +++++++++++++++\r\n", counter);
		counter++;
		rtos_delay(2000);
	}
}

/*!
 * @brief Start its counter and increments it every second
 *
 * @param none
 * @retval none
 */
void dummy_task2(void)
{
	uint8_t counter = 0;
	for (;;)
	{
		PRINTF("IN TASK 2: %i ***************\r\n", counter);
		counter++;
		rtos_delay(1000);
	}
}

/*!
 * @brief Start its counter and increments it every 4 seconds
 *
 * @param none
 * @retval none
 */
void dummy_task3(void)
{
	uint8_t counter = 0;
	for (;;)
	{
		PRINTF("IN TASK 3: %i ---------------\r\n", counter);
		counter++;
		rtos_delay(4000);
	}
}
