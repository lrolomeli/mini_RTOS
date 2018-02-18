/**
 * @file rtos.c
 * @author ITESO
 * @date Feb 2018
 * @brief Implementation of rtos API
 * created by
 * Luis Roberto Lomeli IE700093
 * Jorge Mizael Rodriguez IE698983
 * This is the implementation of the rtos module for the
 * embedded systems II course at ITESO
 */

#include "rtos.h"
#include "rtos_config.h"
#include "clock_config.h"

#ifdef RTOS_ENABLE_IS_ALIVE
#include "fsl_gpio.h"
#include "fsl_port.h"
#endif

/**********************************************************************************/
// IS ALIVE definitions
/**********************************************************************************/

#ifdef RTOS_ENABLE_IS_ALIVE
#define CAT_STRING(x,y)  		x##y
#define alive_GPIO(x)			CAT_STRING(GPIO,x)
#define alive_PORT(x)			CAT_STRING(PORT,x)
#define alive_CLOCK(x)			CAT_STRING(kCLOCK_Port,x)
static void init_is_alive(void);
static void refresh_is_alive(void);
#endif
/**********************************************************************************/
// Local methods prototypes
/**********************************************************************************/

static void reload_systick(void);
static void dispatcher(task_switch_type_e type);
static void activate_waiting_tasks();
FORCE_INLINE static void context_switch(task_switch_type_e type);
static void idle_task(void);

/**********************************************************************************/
// Global Variables
/**********************************************************************************/

static task_list_t  task_list = { 0 };

/**********************************************************************************/
// API implementation
/**********************************************************************************/

/**********************************************************************************/
//@ brief Starts the scheduler, from this point the RTOS takes control
//@ on the processor
//@ param none
//@ retval none
/**********************************************************************************/
void rtos_start_scheduler(void)
{
#ifdef RTOS_ENABLE_IS_ALIVE

	/** configures OS I/O ports  */
	init_is_alive();

	/** sets the systick timer */
	task_list.global_tick = START_ZERO;

	/** creates default task*/
	rtos_create_task(idle_task, PRIORITY0, kAutoStart);

	/** presets and prepare the system when task cant be created*/
	task_list.current_task = RTOS_INVALID_TASK;

#endif

	/**Configure systick interrupt*/
	SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk
	        | SysTick_CTRL_ENABLE_Msk;

	/**gives the first count value number of ticks to count*/
	reload_systick();

	for (;;); /**everything configure waiting for interrupt*/
}

/**********************************************************************************/
//@param task_body pointer to the body of the task
//@param priority number for the RMS algorithm
//@param autostart either autostart or start suspended
//@retval task_handle of the task created
/**********************************************************************************/
rtos_task_handle_t rtos_create_task(void (*task_body)(), uint8_t priority,
		rtos_autostart_e autostart)
{
	rtos_task_handle_t retval = RTOS_INVALID_TASK;
	/** checks if there are enough space to create another task*/
	if(RTOS_MAX_NUMBER_OF_TASKS > task_list.nTasks)
	{
		/** according to kauto change of state of task to ready or suspended */
		task_list.tasks[task_list.nTasks].state = (kAutoStart == autostart) ? S_READY : S_SUSPENDED;

		/** Sets all fields of the tasks */
		task_list.tasks[task_list.nTasks].priority = priority;
		task_list.tasks[task_list.nTasks].local_tick = START_ZERO;
		task_list.tasks[task_list.nTasks].task_body = task_body;

		/** stack pointer points out the first parameter of task context*/
		task_list.tasks[task_list.nTasks].sp =
				&(task_list.tasks[task_list.nTasks].stack[RTOS_STACK_SIZE - END_OF_STACK - STACK_FRAME_SIZE]);

		/** stores the function address and the PSR default value*/
		task_list.tasks[task_list.nTasks].stack[RTOS_STACK_SIZE - STACK_PSR_OFFSET] = STACK_PSR_DEFAULT;
		task_list.tasks[task_list.nTasks].stack[RTOS_STACK_SIZE - STACK_PC_OFFSET] = (uint32_t) task_body;

		retval = task_list.nTasks;
		task_list.nTasks++;

		return retval;	/** returns task ID */

	}
	else
	{
		return retval;	/** error while creating task*/
	}

	return retval;
}

/**********************************************************************************/
//@brief Returns the rtos global tick
//@param none
//@retval clock value
/**********************************************************************************/
rtos_tick_t rtos_get_clock(void)
{
	/** returns the rtos global tick*/
	return task_list.global_tick;
}

/**********************************************************************************/
//@brief Suspends the task calling this function by a certain
//amount of time specified by the parameter ticks
//@param ticks amount of ticks for the delay
//@retval none
/**********************************************************************************/
void rtos_delay(rtos_tick_t ticks)
{
	/** This function rest the task so that another task can be executed */
	task_list.tasks[task_list.current_task].state = S_WAITING;
	task_list.tasks[task_list.current_task].local_tick = ticks;
	dispatcher(kFromNormalExec);

}

/**********************************************************************************/
//@brief Suspends the task calling this function
//@param none
//@retval none
/**********************************************************************************/
void rtos_suspend_task(void)
{
	/** when this function is called the task goes to sleep mode */
	task_list.tasks[task_list.current_task].state = S_SUSPENDED;
	dispatcher(kFromNormalExec);

}

/**********************************************************************************/
//@brief Activates the task identified by the task handle
//@param task handle of the task to be activated
//@retval none
/**********************************************************************************/
void rtos_activate_task(rtos_task_handle_t task)
{
	/** this activates the task which was suspended */
	task_list.tasks[task].state = S_READY;
	dispatcher(kFromNormalExec);

}

/**********************************************************************************/
// Local methods implementation
/**********************************************************************************/

/**********************************************************************************/
//@brief modify the systick
//@param none
//@retval none
/**********************************************************************************/
static void reload_systick(void)
{
	/**  */
	SysTick->LOAD = USEC_TO_COUNT(RTOS_TIC_PERIOD_IN_US,
	        CLOCK_GetCoreSysClkFreq());
	SysTick->VAL =START_ZERO;
}

/**********************************************************************************/
//@brief This funcion choose the task depend of her priority and change the context
//@param task_switch_type check the priority of all tasks
//@retval none
/**********************************************************************************/
static void dispatcher(task_switch_type_e type)
{
	rtos_task_handle_t next_task = RTOS_INVALID_TASK;
	uint8_t index;
	int8_t highest_priority = LOWEST_PRIORITY;

	/** this selects an active task with highest priority to be executed */
	for (index = 0; index < task_list.nTasks; index++)
	{
		if ((highest_priority < task_list.tasks[index].priority)
		        && (S_RUNNING == task_list.tasks[index].state
		                || S_READY == task_list.tasks[index].state))
		{
			next_task = index;
			highest_priority = task_list.tasks[index].priority;
		}
	}


	//** if the next task is the same as the new one then stop doing context switch */
	if (task_list.current_task != next_task)
	{
		task_list.next_task = next_task;
		context_switch(type);
	}

}

/**********************************************************************************/
//@brief Activates the task identified by the task handle
//@param task_switch_type,
//@retval none
/**********************************************************************************/
FORCE_INLINE static void context_switch(task_switch_type_e type)
{
	static uint8_t first_run = START_ONE;
	register uint32_t *sp asm("sp");

	if(first_run)
	{
		/**Stack pointer doesn't have to be stored because there is no context switch*/
		first_run = START_ZERO;
	}
	else
	{
		/**Saves the stack of the current task evaluating the stack frame*/
		task_list.tasks[task_list.current_task].sp = (kFromNormalExec == type) ? sp - STACK_OFFSET_ISR_AND_EXEC
				: sp + STACK_OFFSET_ISR_AND_EXEC;
	}

	/**Execute next task make the context switch*/
	task_list.current_task = task_list.next_task;
	task_list.tasks[task_list.current_task].state = S_RUNNING;
	SCB->ICSR |= SCB_ICSR_PENDSVSET_Msk;

}


/**********************************************************************************/
//@brief change the state of tasks of waiting to active
//@param none
//@retval none
/**********************************************************************************/
static void activate_waiting_tasks()
{
	 uint8_t index;

	 for(index = START_ZERO ; index < task_list.nTasks; index++)
	 {
		 /** Searches for the task which has left sleep mode (done delay)*/
		 if(S_WAITING == task_list.tasks[index].state)
		 {
			 task_list.tasks[index].local_tick--;

			 /** And turns it ready (ON)*/
			 if(START_ZERO == task_list.tasks[index].local_tick)
			 {
				 task_list.tasks[index].state = S_READY;
			 }
		 }
	 }

}

/**********************************************************************************/
// IDLE TASK
/**********************************************************************************/

/**********************************************************************************/
//@brief it is default task
//@param none
//@retval none
/**********************************************************************************/
static void idle_task(void)
{
	for (;;)
	{

	}
}

/**********************************************************************************/
// ISR implementation
/**********************************************************************************/

/**********************************************************************************/
//@brief increment the ticks each task and change the context
//@param none
//@retval none
/**********************************************************************************/
void SysTick_Handler(void)
{
#ifdef RTOS_ENABLE_IS_ALIVE

	refresh_is_alive();			/* alive led toggling **/

#endif
	task_list.global_tick++;
	activate_waiting_tasks();	/** search for a task to run*/
	dispatcher(kFromISR);		/** try to switch context*/
	reload_systick();			/** recharges the value to interrupt again*/

}

/**********************************************************************************/
//@brief moves the actual task stack pointer to the global one on processor
//@param none
//@retval none
/**********************************************************************************/
void PendSV_Handler(void)
{
	register uint32_t *sp asm("r0");
	SCB->ICSR |= SCB_ICSR_PENDSVCLR_Msk;	/**saves the context cortex M-4*/
	sp = task_list.tasks[task_list.current_task].sp;
	asm("MOV r7, r0");	/** preserves the original stack pointer as it was*/
}

/**********************************************************************************/
// IS ALIVE SIGNAL IMPLEMENTATION
/**********************************************************************************/
#ifdef RTOS_ENABLE_IS_ALIVE
static void init_is_alive(void)
{
	gpio_pin_config_t gpio_config =
	{ kGPIO_DigitalOutput, 1, };

	port_pin_config_t port_config =
	{ kPORT_PullDisable, kPORT_FastSlewRate, kPORT_PassiveFilterDisable,
	        kPORT_OpenDrainDisable, kPORT_LowDriveStrength, kPORT_MuxAsGpio,
	        kPORT_UnlockRegister, };
	CLOCK_EnableClock(alive_CLOCK(RTOS_IS_ALIVE_PORT));
	PORT_SetPinConfig(alive_PORT(RTOS_IS_ALIVE_PORT), RTOS_IS_ALIVE_PIN,
	        &port_config);
	GPIO_PinInit(alive_GPIO(RTOS_IS_ALIVE_PORT), RTOS_IS_ALIVE_PIN,
	        &gpio_config);
}

/**********************************************************************************/
//
/**********************************************************************************/
static void refresh_is_alive(void)
{
	static uint8_t state = START_ZERO;
	static uint32_t count =START_ZERO;
	SysTick->LOAD = USEC_TO_COUNT(RTOS_TIC_PERIOD_IN_US,
	        CLOCK_GetCoreSysClkFreq());
	SysTick->VAL = START_ZERO;
	if (RTOS_IS_ALIVE_PERIOD_IN_US / RTOS_TIC_PERIOD_IN_US - VALUE_START_PERIOD == count)
	{
		GPIO_WritePinOutput(alive_GPIO(RTOS_IS_ALIVE_PORT), RTOS_IS_ALIVE_PIN,
		        state);
		state = state == ON ? OFF : ON;
		count = START_ZERO;
	} else
	{
		count++;
	}
}
#endif
/**********************************************************************************/
//
/**********************************************************************************/
