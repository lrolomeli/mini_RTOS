/**
 * @file rtos.h
 * @author ITESO
 * @date Feb 2018
 * @brief Implementation of rtos API
 * created by
 * Luis Roberto Lomeli IE700093
 * Jorge Mizael Rodriguez IE698983
 * This is the implementation of the rtos module for the
 * embedded systems II course at ITESO
 */
#ifndef SOURCE_RTOS_H_
#define SOURCE_RTOS_H_

#include "rtos_config.h"
#include "stdint.h"

/**********************************************************************************/
/* Module defines																  */
/**********************************************************************************/

#define FORCE_INLINE 	__attribute__((always_inline)) inline

#define STACK_PSR_DEFAULT			0x01000000
#define ALL_TASKS_CREATED_CORRECTLY 66
#define RESERVED_MEMORY				10
#define STACK_OFFSET_ISR_AND_EXEC	9
#define STACK_FRAME_SIZE			8
#define STACK_PC_OFFSET				2
#define STACK_PSR_OFFSET			1
#define TASK_IDLE					1
#define END_OF_STACK				1
#define VALUE_START_PERIOD			1
#define RTOS_INVALID_TASK			(-1)

/**********************************************************************************/
/* Type definitions																  */
/**********************************************************************************/

/*! @brief type of task state */
typedef enum
{
    S_READY = 0, S_RUNNING, S_WAITING, S_SUSPENDED

} task_state_e;

/*! @brief type of variable start */
typedef enum
{
    START_ZERO = 0, START_ONE

} start_values_type_e;

/*! @brief Type of dispatcher call */
typedef enum
{
    kFromISR = 0, kFromNormalExec

} task_switch_type_e;

/*! @brief AutoStart state type */
typedef enum
{
    kAutoStart, kStartSuspended
} rtos_autostart_e;

/*! @brief priority of tasks */
typedef enum
{
    LOWEST_PRIORITY = -1, PRIORITY0 = 0, PRIORITY1, PRIORITY2
} task_priority_type_e;

/*! @brief led state */
typedef enum
{
    ON = 0, OFF
} led_state_type_e;

/*! @brief Task handle type, used to identify a task */
typedef int8_t rtos_task_handle_t;

/*! @brief Tick type, used for time measurement */
typedef uint64_t rtos_tick_t;

/**********************************************************************************/
/* Global (static) TCB															  */
/**********************************************************************************/
typedef struct
{
    uint8_t priority;
    task_state_e state;
    uint32_t *sp;
    void (*task_body) ();
    rtos_tick_t local_tick;
    uint32_t reserved[RESERVED_MEMORY]; /** leave this field as it is */
    uint32_t stack[RTOS_STACK_SIZE];

} rtos_tcb_t;

/**********************************************************************************/
// Global (static) task list													  */
/**********************************************************************************/

typedef struct
{
    uint8_t nTasks;
    rtos_task_handle_t current_task;
    rtos_task_handle_t next_task;
    /** total of tasks that can be created*/
    rtos_tcb_t tasks[RTOS_MAX_NUMBER_OF_TASKS + TASK_IDLE];
    rtos_tick_t global_tick;
} task_list_t;

/*!
 * @brief Starts the scheduler, from this point the RTOS takes control
 * on the processor
 *
 * @param none
 * @retval none
 */
void rtos_start_scheduler (void);

/*!
 * @brief Create task API function
 *
 * @param task_body pointer to the body of the task
 * @param priority number for the RMS algorithm
 * @param autostart either autostart or start suspended
 * @retval task_handle of the task created
 */
rtos_task_handle_t rtos_create_task (void (*task_body) (), uint8_t priority,
        rtos_autostart_e autostart);

/*!
 * @brief Suspends the task calling this function
 *
 * @param none
 * @retval none
 */
void rtos_suspend_task (void);

/*!
 * @brief Activates the task identified by the task handle
 *
 * @param task handle of the task to be activated
 * @retval none
 */
void rtos_activate_task (rtos_task_handle_t task);

/*!
 * @brief Returns the rtos global tick
 *
 * @param none
 * @retval clock value
 */
rtos_tick_t rtos_get_clock (void);

/*!
 * @brief Suspends the task calling this function by a certain
 * amount of time specified by the parameter ticks
 *
 * @param ticks amount of ticks for the delay
 * @retval none
 */
void rtos_delay (rtos_tick_t ticks);

#endif /* SOURCE_RTOS_H_ */
