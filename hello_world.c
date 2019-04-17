#include "system.h"                     // to use the symbolic names
#include "altera_avalon_pio_regs.h" 	// to use PIO functions
#include "alt_types.h"                 	// alt_u32 is a kind of alt_types
#include "sys/alt_irq.h"              	// to register interrupts
#include <stdio.h>
#include <stdlib.h>
#include "io.h"
#include "altera_up_avalon_ps2.h"
#include "altera_up_ps2_keyboard.h"
#include "altera_up_avalon_video_character_buffer_with_dma.h"
#include "altera_up_avalon_video_pixel_buffer_dma.h"
#include "math.h"

#include "FreeRTOS/FreeRTOS.h"
#include "FreeRTOS/task.h"
#include "FreeRTOS/queue.h"
#include "FreeRTOS/semphr.h"
#include "FreeRTOS/timers.h"

#define MIN_FREQ 45.0 //minimum frequency to draw

#define CONTROLLER_TASK_P (tskIDLE_PRIORITY+2)
#define FREQUENCY_TASK_P (tskIDLE_PRIORITY+4)
#define STATES_TASK_P (tskIDLE_PRIORITY+3)


typedef enum FSM {Stable, Unstable, All_connected} State;
typedef enum analyser_request {Connect, Shed} Request;

int connected_flag = 0;
int maintenance_flag = 0;
int timer_finished = 0;
int fully_connected = 0;
int connect = 0;
double threshold_freq = 70.0;
double threshold_ROC = 100.0;
int red;
int green;
int uiSwitchValue;
int uiOldSwitchValue = 0;

static TimerHandle_t timer;
// task handles
static TaskHandle_t H_StabilityAnalyser;
static TaskHandle_t frequency_task_handle;
static TaskHandle_t load_controller_task_handle;
static TaskHandle_t states_task_handle;

// semaphore handles
static SemaphoreHandle_t freq_sem;
static SemaphoreHandle_t threshold_sem;

// communication queue handles
static QueueHandle_t Q_key_data;
static QueueHandle_t Q_freq_data;
static QueueHandle_t Q_ROC_TimeInstant;
static QueueHandle_t Q_load_manage;

double freq, dfreq;
double graph_freq[100], graph_dfreq[100];
int i = 99, j = 0;

TickType_t PeakDetection_Time[100];

// timer interrupt function
void timer_isr(xTimerHandle timer)
{
	timer_finished = 1;
}

// button interrupt function
void button_isr(void* context, alt_u32 id)
{
	// need to cast the context first before using it
	int* temp = (int*) context;
	*temp = IORD_ALTERA_AVALON_PIO_EDGE_CAP(PUSH_BUTTON_BASE);

	if (IORD_ALTERA_AVALON_PIO_EDGE_CAP(PUSH_BUTTON_BASE) == 4){
		if (maintenance_flag == 0)
		{
			printf("Maintenance mode ON\n");
			maintenance_flag = 1;
		}
		else
		{
			printf("Maintenance mode OFF\n");
			maintenance_flag = 0;
		}
	}
	// clears the edge capture register
	IOWR_ALTERA_AVALON_PIO_EDGE_CAP(PUSH_BUTTON_BASE, 0x7);
}

void keyboard_isr(void* ps2_device, alt_u32 id)
{
	char ascii;
	int status = 0;
	unsigned char key = 0;
	KB_CODE_TYPE decode_mode;
	// blocking function call
	status = decode_scancode (ps2_device, &decode_mode , &key , &ascii) ;
	if ( status == 0 ) //success
	{
		// print out the result
		switch ( decode_mode )
		{
		case KB_ASCII_MAKE_CODE :
			printf ( "ASCII   : %x\n", key ) ;
			xQueueSendToBackFromISR(Q_key_data, &key, pdFALSE);
			break ;
			//		case KB_BINARY_MAKE_CODE :
			//			printf ( "MAKE CODE : %x\n", key ) ;
			//			break ;
		default :
			//			printf ( "DEFAULT   : %x\n", key ) ;
			break ;
		}
		IOWR(SEVEN_SEG_BASE, 0 ,key);
		//xQueueSendtoBackFromISR(Q_keydata, &key, pdFALSE);
	}
}

void frequency_isr()
{
#define SAMPLING_FREQ 16000.0
	double temp = SAMPLING_FREQ/(double)IORD(FREQUENCY_ANALYSER_BASE, 0);
	//printf("%f\n", temp);
	xQueueSendToBackFromISR( Q_freq_data, &temp, pdFALSE );
}

void frequency_task()
{
	while(1) {
		xSemaphoreTake(freq_sem, portMAX_DELAY);
		//receive frequency data from queue
		while(uxQueueMessagesWaiting( Q_freq_data ) != 0){
			xQueueReceive( Q_freq_data, graph_freq+i, 0 );
			freq = graph_freq[i];
			//calculate frequency RoC
			if(i==0){
				graph_dfreq[0] = (graph_freq[0] - graph_freq[99]) * 2.0 * graph_freq[0] * graph_freq[99] / (graph_freq[0] + graph_freq[99]);
			}
			else{
				graph_dfreq[i] = (graph_freq[i] - graph_freq[i-1]) * 2.0 * graph_freq[i] * graph_freq[i-1] / (graph_freq[i] + graph_freq[i-1]);
			}

			if (graph_dfreq[i] > 100.0){
				graph_dfreq[i] = 100.0;
			}
			dfreq = graph_dfreq[i];
			i =	++i%100; //point to the next data (oldest) to be overwritten
		}
		//printf("Frequency : %f, RoC : %f\n", freq, dfreq);
		xSemaphoreGive(freq_sem);
		vTaskDelay(10);
	}
}

void load_states_task()
{
	State current_state = All_connected;
	State next_state = All_connected;
	int connect;
	printf("load_states\n");

	while(1){
		if (maintenance_flag == 1)
		{
			current_state = All_connected;
		}
		else
		{
			current_state = next_state;
		}

		if(xSemaphoreTake(threshold_sem, (TickType_t) 5))
		{
			if(xSemaphoreTake(freq_sem, (TickType_t) 5))
			{
				key_detection();
				switch(current_state)
				{
				case (Stable):
																																	if ((freq < threshold_freq) || (dfreq > threshold_ROC))
																																	{
																																		next_state = Unstable;
																																	}
																																	else if (timer_finished == 1)
																																	{
																																		printf("stable!\n");
																																		connect = 1;
																																		timer_finished = 0;
																																		//key_detection();
																																		xQueueSendToBack(Q_load_manage, &connect, 2);
																																		if (fully_connected == 1)
																																		{
																																			next_state = All_connected;
																																		}
																																		else
																																		{
																																			printf("here");
																																			next_state = Stable;
																																			xTimerReset(timer, 5);
																																		}
																																	}
				break;
				//	else
				//	{
				//		next_state = Stable;
				//	}

				case (Unstable):
																																	//printf("%f, %f\n", freq, dfreq);
																																	if ((freq > threshold_freq && dfreq < threshold_ROC))
																																	{
																																		next_state = Stable;
																																	}
																																	else if (timer_finished == 1)
																																	{
																																		printf("unstable!\n");
																																		next_state = Unstable;
																																		connect = 0;
																																		timer_finished = 0;
																																		//key_detection();
																																		xQueueSendToBack(Q_load_manage, &connect, 2);
																																		xTimerReset(timer, 5);
																																	}
				break;
				//	else
				//	{
				//		next_state = Unstable;
				//	}
				case (All_connected):
																															connected_flag = 1;
				if ((freq < threshold_freq || dfreq > threshold_ROC))
				{
					next_state = Unstable;
					connected_flag = 0;
					connect = 0;
					//key_detection();
					xQueueSendToBack(Q_load_manage, &connect, 2);
					xTimerReset(timer, 5);
				}
				else
				{
					next_state = All_connected;
				}
				break;
				}
				xSemaphoreGive(freq_sem);
			}
			xSemaphoreGive(threshold_sem);
		}
	}
}

void key_detection(void *pvParameters)
{
	uiSwitchValue = 0;
	int led_value = 0;
	red = 0;
	green = 0;
	int a;

	red = IORD_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE);
	green = IORD_ALTERA_AVALON_PIO_DATA(GREEN_LEDS_BASE);
	uiSwitchValue = IORD_ALTERA_AVALON_PIO_DATA(SLIDE_SWITCH_BASE) & 0x1F;
	if (maintenance_flag == 1)
	{
		IOWR_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE, (uiSwitchValue & 0x1F));
		IOWR_ALTERA_AVALON_PIO_DATA(GREEN_LEDS_BASE, 0);
		fully_connected = 1;
		xQueueReset(Q_load_manage);
	}
	else{
		if (connected_flag == 1)
		{
//			printf("all connected!\n");
			IOWR_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE, 0x1F);
			IOWR_ALTERA_AVALON_PIO_DATA(GREEN_LEDS_BASE, 0);
		}
		else
		{
			led_value = uiSwitchValue;
			if (uiOldSwitchValue != uiSwitchValue){
				a = log2(uiOldSwitchValue^uiSwitchValue);
//				printf("a before: %d\n",a);
			}

//			printf("OldSwitchValue : %d, SwitchValue : %d\n", uiOldSwitchValue, uiSwitchValue);
//			printf("red before:%d\n", red);
//			printf("red after:%d\n", red & ~(1 << a));
			if(a)
			{
				if ((uiOldSwitchValue) < (uiSwitchValue) && uiOldSwitchValue)
				{
//					printf("switched on\n");
					//IOWR_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE, uiOldSwitchValue | (1 << a));
				}
				else if((uiOldSwitchValue) > (uiSwitchValue))
				{
//					printf("switched off\n");
					red = red & ~(1 << a);
					IOWR_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE, red);
//					printf("LED Value : %d\n", IORD_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE));
				}
				if(uiOldSwitchValue != uiSwitchValue)
				{
					uiOldSwitchValue = uiSwitchValue;
				}
			}
		}
	}
}

void load_controller_task(void *pvParameters)
{
	TickType_t last_wake_time;
	const TickType_t task_period = 1;
	//	int connect = 0;
	//	int uiSwitchValue = 0;
	//	int uiOldSwitchValue = IORD_ALTERA_AVALON_PIO_DATA(SLIDE_SWITCH_BASE) & 0x1F;
	//	int led_value = 0;
	//	int red = 0;
	//	int green = 0;
	//	int a;
	while(1){
		printf("here\n");
//		last_wake_time = xTaskGetTickCount();
//		vTaskDelayUntil(&last_wake_time, task_period);
		//		uiSwitchValue = IORD_ALTERA_AVALON_PIO_DATA(SLIDE_SWITCH_BASE) & 0x1F;
		//		if (maintenance_flag == 1)
		//		{
		//			IOWR_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE, (uiSwitchValue & 0x1F));
		//			IOWR_ALTERA_AVALON_PIO_DATA(GREEN_LEDS_BASE, 0);
		//			fully_connected = 1;
		//			xQueueReset(Q_load_manage);
		//		}
		//		else
		//		{
		//			red = IORD_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE);
		//			green = IORD_ALTERA_AVALON_PIO_DATA(GREEN_LEDS_BASE);
		//
		//			if (connected_flag == 1)
		//			{
		//				printf("all connected!\n");
		//				IOWR_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE, 0x1F);
		//				IOWR_ALTERA_AVALON_PIO_DATA(GREEN_LEDS_BASE, 0);
		//			}
		//			else
		//			{
		//				red = IORD_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE);
		//				green = IORD_ALTERA_AVALON_PIO_DATA(GREEN_LEDS_BASE);
		//				led_value = uiSwitchValue;
		//				if (uiOldSwitchValue != uiSwitchValue){
		//					a = log2(uiOldSwitchValue^uiSwitchValue);
		//					printf("a: %d\n",a);
		//				}
		//
		//				printf("SwitchValue : %d, OldSwitchValue : %d, %d\n", uiSwitchValue, uiOldSwitchValue, led_value);
		//
		//				if(a)
		//				{
		//					if ((uiOldSwitchValue) < (uiSwitchValue) && uiOldSwitchValue)
		//					{
		//						printf("switched on\n");
		//						IOWR_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE, uiOldSwitchValue | (1 << a));
		//					}
		//					else if((uiOldSwitchValue) > (uiSwitchValue))
		//					{
		//						printf("switched off\n");
		//						IOWR_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE, red & ~(1 << a));
		//						printf("LED Value : %d\n", IORD_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE));
		//					}
		//					if(uiOldSwitchValue != uiSwitchValue)
		//					{
		//						uiOldSwitchValue = uiSwitchValue;
		//					}
		//				}

		//				for (int i=0 ; i<5 ; i++)
		//				{
		//					if ((uiOldSwitchValue & (1 << i)) < (uiSwitchValue & (1 << i)))
		//					{
		//						printf("switched on");
		//						//IOWR_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE, uiSwitchValue & ~(1 << i));
		//						IOWR_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE, uiOldSwitchValue);
		//						IOWR_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE, uiSwitchValue & ~(1 << i));
		//						//						IOWR_ALTERA_AVALON_PIO_DATA(GREEN_LEDS_BASE, led_value & green);
		//						//led_value = IORD_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE);
		//					}
		//					else if((uiOldSwitchValue & (1 << i)) > (uiSwitchValue & (1 << i)))
		//					{
		//						printf("switched off");
		//						//led_value = IORD_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE);
		//						IOWR_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE, uiSwitchValue | (1 << i));
		//						IOWR_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE, uiSwitchValue);
		//					}
		//					printf("something turned on!\n");
		//					led_value = uiSwitchValue;
		//					IOWR_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE, led_value & red);
		//					IOWR_ALTERA_AVALON_PIO_DATA(GREEN_LEDS_BASE, led_value & green);
		//				}
		//				led_value = ~((uiSwitchValue^uiOldSwitchValue) & uiSwitchValue) & uiSwitchValue;
		//				IOWR_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE, led_value & red);
		//				IOWR_ALTERA_AVALON_PIO_DATA(GREEN_LEDS_BASE, led_value & green);
		//				printf("red: %d\n", red);

		if (uxQueueMessagesWaiting(Q_load_manage) != 0)
		{
			xQueueReceive(Q_load_manage, &connect, 5);
			printf("connect : %d\n",connect);
			switch(connect)
			{

			case 0:
				for (i=0 ; i<5 ; i++)
				{
					if(red & (1 << i))
					{
						IOWR_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE, red & ~(1 << i));
						IOWR_ALTERA_AVALON_PIO_DATA(GREEN_LEDS_BASE, green | (1 << i));
						break;
					}
				}
				break;

			case 1:
				for (i=4 ; i>=0 ; i--)
				{
					if ((uiSwitchValue & (1 << i)) ^ (red & (1 << i)))
					{
						IOWR_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE, red | (1 << i));
						IOWR_ALTERA_AVALON_PIO_DATA(GREEN_LEDS_BASE, green & ~(1 << i));
						break;
					}
				}
				break;

			default:
				printf("Something went wrong!\n");
				break;
			}
		}
		if (uiSwitchValue & 0x1F ^ IORD_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE & 0x1F))
		{
			fully_connected = 0;
		}
		else
		{
			fully_connected = 1;
		}
		//		}
		//	}
	vTaskDelay(1);
	}
}

int update_frequency_threshold()
{
	char key;
	int pressed_key = 0;
	while(1)
	{
		//xQueueReceive(Q_keydata, &key, pdFALSE);
		if (pressed_key == 1)
		{
			pressed_key == 0;
		}
		else
		{
			pressed_key == 1;
		}
		threshold_freq = atoi(key);
	}
}

// initialising buttons
void button_initialise()
{
	// clears the edge capture register. Writing 1 to bit clears pending interrupt for corresponding button.
	IOWR_ALTERA_AVALON_PIO_EDGE_CAP(PUSH_BUTTON_BASE, 0x7);

	// enable interrupts for all buttons
	IOWR_ALTERA_AVALON_PIO_IRQ_MASK(PUSH_BUTTON_BASE, 0x7);

	// register the ISR
	alt_irq_register(PUSH_BUTTON_IRQ, NULL , button_isr);
}

// initialising keyboard inputs
int keyboard_initialise()
{
	// creating queue
	//Q_keydata = xQueueCreate(5, sizeof(char));

	// setting up device
	alt_up_ps2_dev * ps2_device = alt_up_ps2_open_dev(PS2_NAME);

	if(ps2_device == NULL)
	{
		printf("can't find PS/2 device\n");
		return 1;
	}

	alt_up_ps2_clear_fifo (ps2_device);

	// registering ISR
	alt_irq_register(PS2_IRQ, ps2_device, keyboard_isr);

	//alt_up_ps2_enable_read_interrupt(ps2_device);
	IOWR_8DIRECT(PS2_BASE,4,1);
}

int main(void)
{
	//for frequency isr
	Q_freq_data = xQueueCreate(100, sizeof(double));
	alt_irq_register(FREQUENCY_ANALYSER_IRQ, 0, frequency_isr);

	// for keyboard isr
	Q_key_data = xQueueCreate(5, sizeof(unsigned char));

	// for load management task
	Q_load_manage = xQueueCreate(5, sizeof(Request));

	// initialisations
	button_initialise();
	keyboard_initialise();

	// creating semaphores and making them available
	freq_sem = xSemaphoreCreateMutex();
	threshold_sem = xSemaphoreCreateMutex();

	xSemaphoreGive(freq_sem);
	xSemaphoreGive(threshold_sem);

	timer = xTimerCreate("timer_isr", pdMS_TO_TICKS(500), pdFALSE, NULL, timer_isr);

	// creating tasks
	xTaskCreate(frequency_task, "frequency_task", configMINIMAL_STACK_SIZE, NULL, FREQUENCY_TASK_P, frequency_task_handle);
	xTaskCreate(load_states_task, "load_states_task", configMINIMAL_STACK_SIZE, NULL, STATES_TASK_P, states_task_handle);
	xTaskCreate(load_controller_task, "load_controller_task", configMINIMAL_STACK_SIZE, NULL, CONTROLLER_TASK_P, load_controller_task_handle);

	vTaskStartScheduler();

	// need this to keep the program alive
	while(1){}

	return 0;
}
