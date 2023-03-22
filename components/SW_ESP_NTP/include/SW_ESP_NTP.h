

#ifndef MAIN_NTP_H_
#define MAIN_NTP_H_

#include "esp_sntp.h"
#include <stdio.h>
#include <time.h>
#include <sys/time.h>


extern SemaphoreHandle_t UART_Jeton;
esp_err_t SW_SafePrint(SemaphoreHandle_t* Jeton,const char* fmt, ...);
void NTP_Init();
struct tm NTP_GetTime();
void time_sync_notification_cb();


#endif /* MAIN_NTP_H_ */
