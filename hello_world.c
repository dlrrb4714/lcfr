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

#include "FreeRTOS/FreeRTOS.h"
#include "FreeRTOS/task.h"
#include "FreeRTOS/queue.h"
#include "FreeRTOS/semphr.h"

#define MIN_FREQ 45.0 //minimum frequency to draw

#define FREQUENCY_TASK_P (tskIDLE_PRIORITY+4)

TaskHandle_t frequency_task_handle;

int maintenanceFlag = 0;
double threshold_freq = 50.0;

// task handles
static TaskHandle_t H_StabilityAnalyser;

// semaphore handles
SemaphoreHandle_t freq_sem;
// communication queue handles
static QueueHandle_t Q_keydata;
static QueueHandle_t Q_freq_data;
static QueueHandle_t Q_ROC_TimeInstant;

double freq, dfreq;
double graph_freq[100], graph_dfreq[100];
int i = 99, j = 0;

TickType_t PeakDetection_Time[100];

// button interrupt function
void button_isr(void* context, alt_u32 id)
{
	// need to cast the context first before using it
	int* temp = (int*) context;
	*temp = IORD_ALTERA_AVALON_PIO_EDGE_CAP(PUSH_BUTTON_BASE);

	if (IORD_ALTERA_AVALON_PIO_EDGE_CAP(PUSH_BUTTON_BASE) == 4){
		if (maintenanceFlag == 0)
		{
			printf("Maintenance mode ON\n");
			maintenanceFlag = 1;
		}
		else
		{
			printf("Maintenance mode OFF\n");
			maintenanceFlag = 0;
		}
	}
	// clears the edge capture register
	IOWR_ALTERA_AVALON_PIO_EDGE_CAP(PUSH_BUTTON_BASE, 0x7);
}

int keyboard_isr(void* ps2_device, alt_u32 id)
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
			break ;
		case KB_BINARY_MAKE_CODE :
			printf ( "MAKE CODE : %x\n", key ) ;
			break ;
		default :
			printf ( "DEFAULT   : %x\n", key ) ;
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
			i =	++i%100; //point to the next data (oldest) to be overwritten
		}
		printf("Frequency : %f, RoC : %f", freq, dfreq);
		vTaskDelay(10);
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
	// for frequency isr
	Q_freq_data = xQueueCreate(100, sizeof(double));
	alt_irq_register(FREQUENCY_ANALYSER_IRQ, 0, frequency_isr);

//	button_initialise();
//	keyboard_initialise();
	freq_sem = xSemaphoreCreateMutex();

	xTaskCreate(frequency_task, "frequency_task", configMINIMAL_STACK_SIZE, NULL, FREQUENCY_TASK_P, frequency_task_handle);

	vTaskStartScheduler();

	// need this to keep the program alive
	while(1){}

	return 0;
}
