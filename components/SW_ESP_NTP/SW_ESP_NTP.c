

#include "SW_ESP_NTP.h"

void NTP_Init()
{

		setenv("TZ", "CET-1CEST,M3.5.0,M10.5.0/3", 1); //France Time zone
		tzset(); // update C library runtime data for the new timezone.
		//sntp_set_sync_interval(15000); //testing sync time every 15 seconds
		sntp_set_time_sync_notification_cb(time_sync_notification_cb);
		sntp_setoperatingmode(SNTP_OPMODE_POLL);
		sntp_setservername(0, "pool.ntp.org");
		sntp_init();
		int retry = 0;
		const int retry_count = 10;
		while (sntp_get_sync_status() == SNTP_SYNC_STATUS_RESET && ++retry < retry_count) {
			printf("Waiting for system time to be set... (%d/%d)\n", retry, retry_count);
			vTaskDelay(2000 / portTICK_PERIOD_MS);
		}
}

struct tm NTP_GetTime()
{
	time_t now = 0;
	struct tm timeinfo = { 0 };
	time(&now); //retrieves the current time in seconds since the Epoch (January 1st, 1970, 00:00:00 UTC)
	localtime_r(&now, &timeinfo); //  converts the time in &now to a local time
	return timeinfo;
}

//TIME TM DEJA INSTALLER DANS ESPRESSIF!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
void time_sync_notification_cb()
{
	SW_SafePrint(&UART_Jeton,"Here We Synchronize external RTC\n\r");
}


