#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "sys/alt_irq.h"
#include "system.h"
#include "io.h"



#include "FreeRTOS/FreeRTOS.h"
#include "FreeRTOS/task.h"
#include "FreeRTOS/queue.h"
#include "FreeRTOS/timers.h"
#include "FreeRTOS/semphr.h"

enum FSM {Stable,Unstable,Normal};

enum Analyser_Request {L_Connect, L_Shed};





//Task handles
static TaskHandle_t H_StabilityAnalyser;
static TaskHandle_t H_load_ctroller;
static TimerHandle_t timer;

//Communication Queue Handles
static QueueHandle_t Q_freq_data;
static QueueHandle_t Q_ROC_TimeInstant;
static QueueHandle_t Q_loadCtrl_Msg;

//Samaphores handles
static SemaphoreHandle_t xFreq_Data_Mutex
static SemaphoreHandle_t StabilityAnalyser_Normal_sem;  //Normal state without performing task (IDLE STATE)
static SemaphoreHandle_t xthreshold_Data_Mutex;  





unsigned char maintenance_mode = 0;
unsigned char timer_expired = 0;
unsigned char all_Loadconnected = 0;

double f_threshold = 49.5;
double ROC_threshold = 10.0;

int f_old=99, f_new=98;
int shed_index




//stabilityanalyser task definition
void StabilityAnalyser(void *pvParamaters) {
	enum FSM current_S = Normal;
	enum FSM next_S = Normal;
	enum Analyser_Request req;

	for(;;){
		ulTaskNotifyTake(pdFALSE, portMAX_DELAY);

		// check whether maintainance mode is enabled
		if (maintenance_mode==0){
			current_S = Normal;
		} else {
			current_S = next_S;
		}

		// take frequency data and threshold data mutex semaphores
		if (xSemaphoreTake(xFreq_Data_Mutex, (TickType_t) 4)==pdTRUE){
			if (xSemaphoreTake(xthreshold_Data_Mutex, (TickType_t) 4)==pdTRUE){

				switch(current_S){
				
				case 0: //stable state
					if ((freq[f_new] < f_threshold) || fabs((ROC[f_new]) > ROC_threshold)) {
						
						next_S = Unstable;
						xTimerReset(timer, 5);
					} else if (timer_expired==pdTRUE){
						timer_expired = 0;

						Ctrl_Req = L_Connect;
						xQueueSendToBack(Q_loadCtrl_Msg, &Ctrl_Req, 2);

						if (all_Loadconnected){
							next_S = Normal;
						} else {
							xTimerReset(timer, 1);
							next_S = Stable;
						}

					} else {
						next_S = Stable;
					}

					break;
					
					case 1: //unstable state
					if ((freq[f_new] > f_threshold) && (fabs(ROC[f_new]) < ROC_threshold)){
						xTimerReset(timer, 5);
						next_S = Stable;

					} else if (timer_expired){
						timer_expired = 0;

						shed_index = f_new;

						Ctrl_Req = L_Shed;
						xQueueSendToBack(Q_loadCtrl_Msg, &Ctrl_Req, 2);

						xTimerReset(timer, 5);
						next_S = Unstable;

					} else {
						next_S = Unstable;
					}

					break;
				case 2: // normal state
					xSemaphoreGive(StabilityAnalyser_Normal_sem); // give the managing semaphonre to control task

					if ((freq[f_new] < f_threshold) || (fabs(ROC[f_new]) > ROC_threshold)) {
						shed_index = f_new;

						Ctrl_Req = L_Shed;
						xQueueSendToBack(Q_loadCtrl_Msg, &Ctrl_Req, 2);

						xTimerReset(timer, 5);

						next_S = Unstable;
					} else {
						next_S = Normal;
					}
					break;

				

				

				default:
					printf("Stability analyser critical");
					break;
				}

				xSemaphoreGive(xthreshold_Data_Mutex);
			}

			xSemaphoreGive(xFreq_Data_Mutex);
		}
	}
}


