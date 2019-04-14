#include <stdio.h>
#include <stdlib.h>
#include "sys/alt_irq.h"
#include "system.h"
#include "io.h"
#include "altera_up_avalon_video_character_buffer_with_dma.h"
#include "altera_up_avalon_video_pixel_buffer_dma.h"




#include "FreeRTOS/FreeRTOS.h"
#include "FreeRTOS/task.h"
#include "FreeRTOS/queue.h"


#define MIN_FREQ 45.0 //minimum frequency to draw


//Task Handle
static TaskHandle_t H_StabilityAnalyser;

//Communication Queue Handles
static QueueHandle_t Q_freq_data;
static QueueHandle_t Q_ROC_TimeInstant;

//Samaphores handles
static SemaphoreHandle_t xFreq_Data_Mutex


unsigned double freq[100], ROC[100];
TickType_t PeakDetection_Time[100];

//FreqAnalyser interrupt function definition
void freq_analyser_isr(void *pvParameters)
{
#define SAMPLING_FREQ 16000.0
	double Signal_iFreq = SAMPLING_FREQ/(double)IORD(FREQUENCY_ANALYSER_BASE, 0);
 
//Sending i/p freq on freq data queue
	xQueueSendToBackFromISR( Q_freq_data, &Signal_iFreq , pdFALSE );
	
//obtaining tickcount from peak detection and sending onQ_ROC_TimeInstant 
	
	TickType_t Tick_Count = xTaskGetTickCountFromISR();
	xQueueSendToBackFromISR( Q_ROC_TimeInstant, &Tick_Count, pdFALSE );

int i = 99,j=99;
	for(;;){
		xSemaphoreTake(xFreq_Data_Mutex,portMAX_Delay);

		//receive frequency data from queue
		while(uxQueueMessagesWaiting( Q_freq_data ) != 0){
			xQueueReceive( Q_freq_data, freq+i, portMAX_DELAY );

			//calculate frequency RoC

			if(i==0){
				ROC[0] = (freq[0]-freq[99]) * 2.0 * freq	[0] * freq[99] / (freq[0]+freq[99]);
			}
			else{
				ROC[i] = (freq[i]-freq[i-1]) * 2.0 * 	freq[i]* freq[i-1] / (freq[i]+freq[i-1]);
			}

			if (ROC[i] > 100.0){
				ROC[i] = 100.0;
			}
		

			i =	++i%100; //point to the next data (oldest) 	to be overwritten
			
		}
	while(uxQueueMessagesWaiting( Q_ROC_TimeInstant) != 0){
			xQueueReceive( Q_ROC_TimeInstant, 	PeakDetection_Time+j, portMAX_DELAY );
		j =	++j%100; 
			}
			xSemaphoreGive(xFreq_Data_Mutex);
	
	//Notifying Stability analyser task from FreqAnalyserISR
	
	vTaskNotifyGiveFromISR( H_StabilityAnalyser, pdFALSE );

			
		
		}
		vTaskDelay(10);

	}
	}
}



int main()
{
	Q_freq_data = xQueueCreate( 100, sizeof(double) );
	Q_ROC_TimeInstant=xQueueCreate(100, sizeof(TickType_t));
     //register frequency amalyser as ISR
	alt_irq_register(FREQUENCY_ANALYSER_IRQ, 0, freq_analyser_isr);





	vTaskStartScheduler();

	for(;;)

  return 0;
}



