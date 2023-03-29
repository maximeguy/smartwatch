#include <stdio.h>
#include <math.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "esp_freertos_hooks.h"
#include "esp_timer.h"
#include "esp_sleep.h"
#include "esp_system.h"
#include "sdkconfig.h"
#include "esp_wifi.h"
#include "SW_ESP_NTP.h"
#include "SW_IMU_Driver.h"
#include "nvs_flash.h"
#include "lvgl/lvgl.h"
#include "lvgl_helpers.h"

#include "images/bluetooth.c"
#include "images/ellipse.c"
#include "images/temperature.c"

#include "images/humidity.c"
#include "images/pressure.c"
#include "images/steps.c"

/*********************
 *      DEFINES
 *********************/
#define BTN_GPIO 0
#define LV_TICK_PERIOD_MS 100
#define N_SCREENS 3
#define COMPASS_RADIUS 42

/********I2c Specific*********/
#define I2C_MASTER_SCL_IO 26//19
#define I2C_MASTER_SDA_IO 2
#define LSM6DSO_INT1 25 //GPIO25 => RTC_GPIO6
/********I2c Specific*********/

#define deg_to_rad(angleInDegrees) ((angleInDegrees) * M_PI / 180.0)
#define rad_to_deg(angleInRadians) ((angleInRadians) * 180.0 / M_PI)

/* The combined length of the queues and binary semaphore that will be
added to the queue set. */
#define COMBINED_LENGTH 18

#define EXAMPLE_ESP_WIFI_SSID      CONFIG_WIFI_SSID
#define EXAMPLE_ESP_WIFI_PASS      CONFIG_WIFI_PASSWORD

/**************************************Begin Handlers**********************************************/
TaskHandle_t blink_task_handle = NULL;
TaskHandle_t state_machine_handle = NULL;
TaskHandle_t Lsm6dso_TASK_Handler ,StepCounter_Handler ;
TaskHandle_t LIS2MDL_TASK_Handler,Weather_TASK_Handler;
SemaphoreHandle_t gui_smphr;
SemaphoreHandle_t btn_smphr;
SemaphoreHandle_t clock_smphr;
QueueSetHandle_t smphr_qs;
QueueHandle_t StepsQ , WeatherQ ,North_DirQ;
/**
 * @note Use SW_SafePrint for thread safe UART communication
 */
SemaphoreHandle_t UART_Jeton =NULL;

/*
 * @note Use SW_I2c_Driver for thread safe i2c transaction
 */
SemaphoreHandle_t I2c_Jeton =NULL;
/**************************************End Handlers**********************************************/


/**************************************Begin Globals********************************************/
lv_obj_t * label;
lv_obj_t * time_lbl;
lv_obj_t * compass_time_lbl;
lv_obj_t * weather_time_lbl;

lv_obj_t * day_lbl;
lv_obj_t * date_lbl;
lv_obj_t * steps_lbl;

stmdev_ctx_t Lsm6dso_dev_ctx;
stmdev_ctx_t Lis2mdl_dev_ctx;
stmdev_ctx_t hts221_dev_ctx;
stmdev_ctx_t lps22hh_dev_ctx;

/************LIS2MDL / LSM6DSO Variables*********/
typedef struct Steps {
	uint16_t CurrentSteps; //Real time Steps
	uint16_t DailySteps; // Steps/24h
} Steps;

/************LIS2MDL / LSM6DSO Variables*********/

uint8_t current_screen = 0;
lv_obj_t ** screens;

uint8_t screen_init[3] = {0,0,0};
lv_obj_t * main_screen;
lv_obj_t * compass_screen;
lv_obj_t * weather_screen;
lv_obj_t * ellipse_img;
lv_obj_t * compass_lbl;
lv_obj_t * temp_lbl;
lv_obj_t * hum_lbl;
lv_obj_t * press_lbl;
lv_obj_t * compass_canvas;
lv_obj_t * canvas;
lv_draw_line_dsc_t line_dsc;

static lv_color_t c_a;
static lv_color_t c_b;
static lv_color_t c_c;
/**************************************End Globals********************************************/


/*******************************Begin Interrupt service routine ISR******************************/

// Wake up after boot button was pressed
static void IRAM_ATTR btn_isr_handler(void* arg){
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(btn_smphr, &xHigherPriorityTaskWoken);
}

// Wake up after 2s from timer isr
static bool IRAM_ATTR timer_isr_handler(gptimer_handle_t timer, const gptimer_alarm_event_data_t * edata, void * user_ctx){
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(clock_smphr, &xHigherPriorityTaskWoken);
    return true;
}

// Wake up after 100ms from timer isr
static bool IRAM_ATTR timer_display_isr_handler(gptimer_handle_t timer, const gptimer_alarm_event_data_t * edata, void * user_ctx){
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(gui_smphr, &xHigherPriorityTaskWoken);
    return true;
}

static void IRAM_ATTR  Inactivity_Activity_IRQ(void * args){
	vTaskResume(Lsm6dso_TASK_Handler);
}

static void oneshot_timer_callback(void* arg){
	esp_light_sleep_start(); //when we reach here, it means that 20 seconds are already passed without any motion detected
}
/*******************************End Interrupt service routine ISR******************************/

/*******************************Begin Function Prototypes***********************************/
void init_clock_timer(uint64_t delay);

void init_timer_display(uint64_t delay);

static void lv_tick_task(void *arg);
static void state_machine();
static void create_screen(uint8_t screen_id);
void Wifi_Init();

/***************Thread Safe Print****************/
esp_err_t SW_SafePrint(SemaphoreHandle_t* Jeton,const char* fmt, ...);
/***************Thread Safe Print****************/

/*******************************End Function Prototypes***********************************/





void Lsm6dso_TASK(void * pvParameters ){
	/**************One shot timer for entering sleep mode*************/
	esp_timer_handle_t oneshot_timer;
	const esp_timer_create_args_t oneshot_timer_args = {
			.callback = &oneshot_timer_callback,
			.name = "one-shot"
	};
	esp_timer_create(&oneshot_timer_args, &oneshot_timer);
	/**************One shot timer for entering sleep mode*************/

	lsm6dso_all_sources_t all_source;
	for(;;){
		lsm6dso_all_sources_get(&Lsm6dso_dev_ctx, &all_source);
		if(all_source.sleep_state){
			esp_timer_start_once(oneshot_timer, 20000000); //20 seconds
			vTaskSuspend(StepCounter_Handler); //Stop step counting task
			SW_SafePrint(&UART_Jeton,"INACTIVITY Detected\n\r");
		}else if(!all_source.sleep_state){
			esp_timer_stop(oneshot_timer);
			SW_SafePrint(&UART_Jeton,"ACTIVITY Detected\n\r");
			vTaskResume(StepCounter_Handler);//Resume Step Counting
		}
		SW_SafePrint(&UART_Jeton,"LSM6dso Suicide\n\r");
		vTaskSuspend(Lsm6dso_TASK_Handler);//suspend Lsm6dso_Task (suicide)
	}
}

void StepCounter(void * pvParameters ){

	Steps steps;
	lsm6dso_emb_sens_t emb_sens;
	lsm6dso_steps_reset(&Lsm6dso_dev_ctx);
	/* Enable pedometer */
	lsm6dso_pedo_sens_set(&Lsm6dso_dev_ctx, LSM6DSO_FALSE_STEP_REJ_ADV_MODE);
	emb_sens.step = PROPERTY_ENABLE;
	emb_sens.step_adv = PROPERTY_ENABLE;
	lsm6dso_embedded_sens_set(&Lsm6dso_dev_ctx, &emb_sens);
	for(;;){
		lsm6dso_number_of_steps_get(&Lsm6dso_dev_ctx, &steps.CurrentSteps); //step Counting
		xQueueSend(StepsQ,&steps,100)==pdTRUE?SW_SafePrint(&UART_Jeton,"steps :%d\r\n", steps.CurrentSteps)
				:SW_SafePrint(&UART_Jeton,"StepsQ Not Sent\n\r");

		vTaskDelay(500/portTICK_PERIOD_MS);
	}
}


void LIS2MDL_TASK(void * pvParameters){
	uint8_t reg;
	int16_t data_raw_magnetic[3];
	double angle_degree;
	for(;;){
		/* Read output only if new value is available */
		lis2mdl_mag_data_ready_get(&Lis2mdl_dev_ctx, &reg);
		if (reg) {
			/* Read magnetic field data */
			memset(data_raw_magnetic, 0x00, 3 * sizeof(int16_t));
			lis2mdl_magnetic_raw_get(&Lis2mdl_dev_ctx, data_raw_magnetic);
			angle_degree= SW_North_Dir(data_raw_magnetic);

			xQueueSend(North_DirQ,&angle_degree,100)==pdTRUE?SW_SafePrint(&UART_Jeton, "The North direction relative to the sensor: %5.1f deg\r\n", angle_degree)
					:SW_SafePrint(&UART_Jeton,"North_DirQ Not Sent\n\r");
		}
		vTaskDelay(200/portTICK_PERIOD_MS);
	}
}


void Weather_TASK(void * pvParameters){
Weather weather;
	for(;;){
		weather=SW_Get_Weather(hts221_dev_ctx, lps22hh_dev_ctx);
		xQueueSend(WeatherQ,&weather,100)==pdTRUE?SW_SafePrint(&UART_Jeton, "Temp : %5.1f Hum : %5.1f Press : %5.1f\r\n", weather.Temperature,weather.Humdity,weather.Pressure)
				:SW_SafePrint(&UART_Jeton,"WeatherQ Not Sent\n\r");
		vTaskDelay(3000/portTICK_PERIOD_MS);
	}
}

void create_screen(uint8_t screen_id){
		/*********************
		 *      MAIN SCREEN
		 *********************/
		struct tm time = NTP_GetTime();
		char time_buf[16];

		char date_buf[16];
		strftime(date_buf,16, "%a %d/%m",&time);
		strftime(time_buf,16, "%R",&time);

		lv_obj_t * main_screen = lv_obj_create(NULL, NULL);

		lv_obj_t * bt_icon = lv_img_create(main_screen, NULL);
		lv_img_set_src(bt_icon,&bluetooth);
		lv_obj_align(bt_icon, NULL, LV_ALIGN_CENTER, 140, -100);

		time_lbl =  lv_label_create(main_screen, NULL);
		lv_label_set_text(time_lbl, time_buf);
		lv_obj_align(time_lbl, NULL, LV_ALIGN_CENTER, 0, 0);

		date_lbl = lv_label_create(main_screen, NULL);
		lv_label_set_text(date_lbl, date_buf);
		lv_obj_align(date_lbl, NULL, LV_ALIGN_CENTER, 0, -105);

		lv_obj_t * steps_icon = lv_img_create(main_screen, NULL);
		lv_img_set_src(steps_icon,&steps);
		lv_obj_align(steps_icon, NULL, LV_ALIGN_CENTER, -30, 80);

		steps_lbl = lv_label_create(main_screen, NULL);
		lv_label_set_text(steps_lbl, "432");
		lv_obj_align(steps_lbl, NULL, LV_LABEL_ALIGN_LEFT, 5, 80); // @suppress("Symbol is not resolved")

		lv_draw_line_dsc_init(&line_dsc);
		line_dsc.color=c_a;
		line_dsc.width = 8;
		line_dsc.round_end = 1;
		line_dsc.round_start = 1;

		static lv_color_t cbuf[LV_CANVAS_BUF_SIZE_TRUE_COLOR(100, 100)];

		canvas = lv_canvas_create(main_screen, NULL);
		lv_canvas_set_buffer(canvas, cbuf, 100, 100, LV_IMG_CF_TRUE_COLOR_ALPHA);
		lv_obj_align(canvas, NULL, LV_ALIGN_CENTER, 0, 0);
		lv_canvas_draw_arc(canvas, 50, 50, 50, 360-90, time.tm_hour >= 0 && time.tm_hour <=6 ? ((360)*time.tm_hour)/24 + 270 : ((360)*time.tm_hour)/24 + -90, &line_dsc);
		line_dsc.color=c_b;
		line_dsc.width = 6;
		lv_canvas_draw_arc(canvas, 50, 50, 40, 360-90, time.tm_min >= 0 && time.tm_min <=15 ? ((360)*time.tm_min)/60 + 270 : ((360)*time.tm_min)/60 + -90, &line_dsc);
		line_dsc.color=c_c;
		line_dsc.width = 4;
		lv_canvas_draw_arc(canvas, 50, 50, 32, 360-90, time.tm_sec >= 0 && time.tm_sec <=15 ? ((360)*time.tm_sec)/60 + 270 : ((360)*time.tm_sec)/60 + -90, &line_dsc);

		screens[0] = main_screen;


		/*********************
		 *      COMPASS SCREEN
		 *********************/
		static lv_style_t line_style;
		lv_style_init(&line_style);
		lv_style_set_line_width(&line_style, LV_STATE_DEFAULT, 2);
		lv_style_set_line_color(&line_style, LV_STATE_DEFAULT, LV_COLOR_BLUE);
		lv_style_set_line_rounded(&line_style, LV_STATE_DEFAULT, true);

		lv_obj_t * compass_screen = lv_obj_create(NULL, NULL);

		compass_time_lbl =  lv_label_create(compass_screen, NULL);
		lv_label_set_text(compass_time_lbl, time_buf);
		lv_obj_align(compass_time_lbl, NULL, LV_ALIGN_CENTER, 0, -105);


		static lv_color_t cbuf2[LV_CANVAS_BUF_SIZE_TRUE_COLOR(90, 90)];
		lv_obj_t * compass_canvas = lv_canvas_create(compass_screen, NULL);
		lv_canvas_set_buffer(compass_canvas, cbuf2, 90, 90, LV_IMG_CF_TRUE_COLOR_ALPHA);
		lv_obj_align(compass_canvas, NULL, LV_ALIGN_CENTER, 0, 0);
		lv_canvas_draw_arc(compass_canvas, 45, 45, 45, 0, 360, &line_dsc);


		compass_lbl =  lv_label_create(compass_screen, NULL);
		lv_label_set_text(compass_lbl, "    ");
		lv_obj_align(compass_lbl, NULL, LV_ALIGN_CENTER, 0, 0);

		ellipse_img = lv_img_create(compass_screen, NULL);
		lv_img_set_src(ellipse_img,&ellipse);
		lv_obj_align(ellipse_img, NULL, LV_ALIGN_CENTER, 0, 0);
		lv_obj_set_pos(ellipse_img, cos(deg_to_rad(0))*COMPASS_RADIUS+165,sin(deg_to_rad(0))*COMPASS_RADIUS+120);

		screens[1] = compass_screen;

		/*********************
		 *      METEO SCREEN
		 *********************/
		lv_obj_t * weather_screen = lv_obj_create(NULL, NULL);

		weather_time_lbl =  lv_label_create(weather_screen, NULL);
		lv_label_set_text(weather_time_lbl, time_buf);
		lv_obj_align(weather_time_lbl, NULL, LV_ALIGN_CENTER, 0, -105);

		lv_obj_t * temp_icon = lv_img_create(weather_screen, NULL);
		lv_img_set_src(temp_icon,&temperature);
		lv_obj_align(temp_icon, NULL, LV_ALIGN_CENTER, -100, -20);

		lv_obj_t * temp_lbl =  lv_label_create(weather_screen, NULL);
		lv_label_set_text(temp_lbl, "20.4°C");
		lv_obj_align(temp_lbl, NULL, LV_ALIGN_CENTER, -100, 15);

		lv_obj_t * hum_icon = lv_img_create(weather_screen, NULL);
		lv_img_set_src(hum_icon,&humidity);
		lv_obj_align(hum_icon, NULL, LV_ALIGN_CENTER, 100, -20);

		lv_obj_t * hum_lbl =  lv_label_create(weather_screen, NULL);
		lv_label_set_text(hum_lbl, "55%");
		lv_obj_align(hum_lbl, NULL, LV_ALIGN_CENTER, 100, 15);

		lv_obj_t * press_icon = lv_img_create(weather_screen, NULL);
		lv_img_set_src(press_icon,&pressure);
		lv_obj_align(press_icon, NULL, LV_ALIGN_CENTER, 0, 40);

		lv_obj_t * press_lbl =  lv_label_create(weather_screen, NULL);
		lv_label_set_text(press_lbl, "1007hPa");
		lv_obj_align(press_lbl, NULL, LV_ALIGN_CENTER, 0, 75);


		screens[2] = weather_screen;

		lv_scr_load(main_screen);
}

static void init_lvgl() {

    lv_init();
    
    /* Initialize SPI or I2C bus used by the drivers */
    lvgl_driver_init();

    static lv_color_t buf1[DISP_BUF_SIZE];

    /* Use double buffered when not working with monochrome displays */
    static lv_color_t *buf2 = NULL;

    static lv_disp_buf_t disp_buf;

    uint32_t size_in_px = DISP_BUF_SIZE;

    /* Initialize the working buffer depending on the selected display.
     * NOTE: buf2 == NULL when using monochrome displays. */
    lv_disp_buf_init(&disp_buf, buf1, buf2, size_in_px);

    lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.flush_cb = disp_driver_flush;

    disp_drv.buffer = &disp_buf;
    lv_disp_drv_register(&disp_drv);

    // Create and start periodic timer interrupt to call lv_tick_inc
    const esp_timer_create_args_t periodic_timer_args ={
    		.callback = &lv_tick_task,
			.name = "periodic_gui"
    };
    
    esp_timer_handle_t periodic_timer;
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, LV_TICK_PERIOD_MS * 1000));
}

void state_machine()
{
	uint8_t display_state = 0;
	double North_Dir=0.0;
	Steps steps;
	Weather weather__;
	QueueSetMemberHandle_t received_member;

	char temp_buf[16];
	char hum_buf[16];
	char press_buf[16];
	char deg_buf[16];
	char step_buf[16];
	char time_buf[16];

	while(1){
		received_member = xQueueSelectFromSet(smphr_qs, portMAX_DELAY);
		if (received_member == btn_smphr){
			xSemaphoreTake(btn_smphr,0);
			current_screen++;
			SW_SafePrint(&UART_Jeton, "#SM BOOT button toggle : current screen : %d\n",current_screen);
			if (current_screen >= N_SCREENS) current_screen = 0;
			//create_screen(current_screen);
			lv_scr_load(screens[current_screen]);
			current_screen == 1 ? vTaskResume(LIS2MDL_TASK_Handler) : vTaskSuspend(LIS2MDL_TASK_Handler);
			current_screen == 2 ? vTaskResume(Weather_TASK_Handler) : vTaskSuspend(Weather_TASK_Handler);
		}
		else if (received_member == clock_smphr){
			SW_SafePrint(&UART_Jeton, "#SM Tick (second).\n");
			xSemaphoreTake(clock_smphr,0);
			display_state = !display_state;

			struct tm time = NTP_GetTime();

			if(display_state == 1){
				sprintf(time_buf, "%02d:%02d",time.tm_hour, time.tm_min);
				lv_label_set_text(time_lbl, time_buf);
				lv_label_set_text(compass_time_lbl, time_buf);
				lv_label_set_text(weather_time_lbl, time_buf);
			}
			else {
				sprintf(time_buf, "%02d %02d",time.tm_hour, time.tm_min);
				lv_label_set_text(time_lbl, time_buf);
				lv_label_set_text(compass_time_lbl, time_buf);
				lv_label_set_text(weather_time_lbl, time_buf);
			}
			if (current_screen == 0){
				lv_canvas_fill_bg(canvas, LV_COLOR_WHITE, 1);
				line_dsc.color=c_a;
				line_dsc.width = 8;
				lv_canvas_draw_arc(canvas, 50, 50, 50, 360-90, time.tm_hour >= 0 && time.tm_hour <=6 ? ((360)*time.tm_hour)/24 + 270 : ((360)*time.tm_hour)/24 + -90, &line_dsc);
				line_dsc.color=c_b;
				line_dsc.width = 6;
				lv_canvas_draw_arc(canvas, 50, 50, 40, 360-90, time.tm_min >= 0 && time.tm_min <=15 ? ((360)*time.tm_min)/60 + 270 : ((360)*time.tm_min)/60 + -90, &line_dsc);
				line_dsc.color=c_c;
				line_dsc.width = 4;
				lv_canvas_draw_arc(canvas, 50, 50, 32, 360-90, time.tm_sec >= 0 && time.tm_sec <=15 ? ((360)*time.tm_sec)/60 + 270 : ((360)*time.tm_sec)/60 + -90, &line_dsc);
			}
		}
		else if (received_member == gui_smphr){
			//SW_SafePrint(&UART_Jeton, "#SM Display update.\n");
			xSemaphoreTake(gui_smphr,0);
			lv_task_handler();
		}else if (received_member == North_DirQ){
			if(xQueueReceive(North_DirQ, &North_Dir,100)== pdPASS){
				SW_SafePrint(&UART_Jeton, "#SM Received North_DirQ : %f.\n",fabs(North_Dir));
				double deg = (North_Dir+180) >= 0 && (North_Dir+180) <=90 ? (North_Dir+180) + 270 : (North_Dir+180)-90;
				double x = cos(deg_to_rad(deg))*COMPASS_RADIUS+155;
				double y = sin(deg_to_rad(deg))*COMPASS_RADIUS+115;
				lv_obj_set_pos(ellipse_img, x, y);
				sprintf(deg_buf, "%3.0f°", deg);
				lv_label_set_text(compass_lbl, deg_buf);
			}
		}else if(received_member == StepsQ){
			if(xQueueReceive(StepsQ, &steps, 100)== pdPASS){
				sprintf(step_buf, "%u", steps.CurrentSteps);
				lv_label_set_text(steps_lbl, step_buf);
			}
		}else if(received_member == WeatherQ){
			if(xQueueReceive(WeatherQ, &weather__, 100)== pdPASS){
				SW_SafePrint(&UART_Jeton, "#SM Received WeatherQ :temp = %f, hum = %f, press = %f.\n",weather__.Temperature,weather__.Humdity,weather__.Pressure);
				//Not working for god knows why.
//				sprintf(temp_buf,"%f°C", weather__.Temperature);
//				lv_label_set_text(temp_lbl, temp_buf);
//				sprintf(hum_buf,"%f", weather__.Humdity);
//				lv_label_set_text(hum_lbl, hum_buf);
//				sprintf(press_buf,"%fhPa", weather__.Pressure);
//				lv_label_set_text(press_lbl, press_buf);

				lv_label_set_text_fmt(temp_lbl, "%f°C", weather__.Temperature);
				lv_label_set_text_fmt(hum_lbl, "%f%%", weather__.Humdity);
				lv_label_set_text_fmt(press_lbl, "%fhPa", weather__.Pressure);
			}
		}
	}
}

void app_main(void)
{
	/******Checking Flash Mem***************/
	esp_err_t ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		ESP_ERROR_CHECK(nvs_flash_erase());
		ret = nvs_flash_init();
	}
	ESP_ERROR_CHECK(ret);
	/******Checking Flash Mem***************/

	/**********Thread Safe Peripherals*********/
	//UART Semaphore creation
	UART_Jeton=xSemaphoreCreateBinary();
	xSemaphoreGive(UART_Jeton);
	//I2c0 Semaphore Creation
	I2c_Jeton=xSemaphoreCreateBinary();
	xSemaphoreGive(I2c_Jeton);
	/**********Thread Safe Peripherals*********/

	c_a = lv_color_make(80, 97, 191);
	c_b = lv_color_make(95, 114, 217);
	c_c = lv_color_make(148, 162, 242);

	SW_I2c_Master_Init(I2C_NUM_0,I2C_MASTER_SCL_IO,I2C_MASTER_SDA_IO);
	Lsm6dso_dev_ctx = SW_Mems_Interface_Init(I2C_NUM_0,0); //0=>Lsm6dso
	Lis2mdl_dev_ctx = SW_Mems_Interface_Init(I2C_NUM_0,1); //1=>Lis2mdl
	lps22hh_dev_ctx = SW_Mems_Interface_Init(I2C_NUM_0,2); //2=>lps22hh
	hts221_dev_ctx=   SW_Mems_Interface_Init(I2C_NUM_0,3); //3=>hts221
	SW_Lsm6dso6_Init_Config(Lsm6dso_dev_ctx);
	SW_Lis2mdl_Init_Config(Lis2mdl_dev_ctx);
	SW_Lps22hh_Init_Config(lps22hh_dev_ctx);
	SW_Hts221_Init_Config(hts221_dev_ctx);

	Wifi_Init();
	NTP_Init();
	vTaskDelay(2000/portTICK_PERIOD_MS);
	xTaskCreate(StepCounter, "StepCounter", 10000, NULL, 1, &StepCounter_Handler);
	vTaskSuspend(StepCounter_Handler);
	xTaskCreate(Lsm6dso_TASK, "Lsm6dso_TASK", 10000, NULL, 3, &Lsm6dso_TASK_Handler);
	//vTaskSuspend(Lsm6dso_TASK_Handler);
	xTaskCreate(LIS2MDL_TASK, "LIS2MDL_TASK", 10000, NULL, 2, &LIS2MDL_TASK_Handler);
	vTaskSuspend(LIS2MDL_TASK_Handler);
	xTaskCreate(Weather_TASK, "Weather_TASK", 10000, NULL, 2, &Weather_TASK_Handler);
	vTaskSuspend(Weather_TASK_Handler);
	/**************LSM6DSO_INT1 ISR / Wake-up Trigger *****************/
	gpio_set_direction(LSM6DSO_INT1, GPIO_MODE_INPUT);
	gpio_set_intr_type(LSM6DSO_INT1,GPIO_INTR_POSEDGE);
	gpio_install_isr_service(0);
	gpio_isr_handler_add(LSM6DSO_INT1, Inactivity_Activity_IRQ,NULL);
	gpio_wakeup_enable(LSM6DSO_INT1, GPIO_INTR_HIGH_LEVEL);
	esp_sleep_enable_gpio_wakeup();
	/**************LSM6DSO_INT1 ISR / Wake-up Trigger *****************/
	gpio_reset_pin(BTN_GPIO);
	gpio_set_direction(BTN_GPIO, GPIO_MODE_INPUT);
	gpio_set_intr_type(BTN_GPIO, GPIO_INTR_NEGEDGE);
	gpio_install_isr_service(0);
	gpio_isr_handler_add(BTN_GPIO, btn_isr_handler, (void*)GPIO_NUM_0);

	gpio_reset_pin(4);
	gpio_set_direction(4, GPIO_MODE_OUTPUT);
	gpio_set_level(4, 0);

	StepsQ = xQueueCreate(5, sizeof(Steps));
	WeatherQ = xQueueCreate(5, sizeof(Weather));
	North_DirQ = xQueueCreate(5, sizeof(double));

	btn_smphr = xSemaphoreCreateBinary();
	clock_smphr = xSemaphoreCreateBinary();
	gui_smphr = xSemaphoreCreateBinary();

    smphr_qs = xQueueCreateSet(COMBINED_LENGTH); //Must update the COMBINED_LENGTH if new Q or Sem are added
    xQueueAddToSet(StepsQ,smphr_qs);
    xQueueAddToSet(WeatherQ,smphr_qs);
    xQueueAddToSet(North_DirQ,smphr_qs);
    xQueueAddToSet(btn_smphr,smphr_qs);
    xQueueAddToSet(clock_smphr,smphr_qs);
    xQueueAddToSet(gui_smphr,smphr_qs);

    init_lvgl();
    // Allocate ui screens array
	screens = malloc(N_SCREENS * sizeof(lv_obj_t));
    create_screen(current_screen);

    xTaskCreate(state_machine, "state_machine", 10000, NULL, 6, &state_machine_handle);

    init_clock_timer(1000 * 1000);
    init_timer_display(200 * 1000);
}





/**
 * @brief Prints a formatted string to the console using a semaphore to protect against concurrent access.
 *
 * @param Jeton Pointer to the semaphore handle used to protect access to the console
 *
 * @param ... Additional arguments to be printed according to the format string.
 *
 * @return ESP_ERR_TIMEOUT if the semaphore could not be acquired within 1000ms, otherwise returns 0.
 */
esp_err_t SW_SafePrint(SemaphoreHandle_t* Jeton,const char* fmt, ...){
	int ret_rtos= xSemaphoreTake(*Jeton,(TickType_t)1000/portTICK_PERIOD_MS);
	if(!ret_rtos)
		return ESP_ERR_TIMEOUT;
	va_list arg;
	va_start(arg, fmt);
	vprintf(fmt, arg);
	va_end(arg);
	return !(xSemaphoreGive(*Jeton)); //return 0 if ok
}


void init_clock_timer(uint64_t delay){
	gptimer_handle_t timer_handle = NULL;

	gptimer_config_t timer_config ={
			.clk_src = GPTIMER_CLK_SRC_DEFAULT,
			.direction = GPTIMER_COUNT_UP,
			.resolution_hz = 1000 * 1000 // 1 tick = 1us
	};
	ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &timer_handle));

	// Alarm config

	gptimer_alarm_config_t alarm_config = {
			.alarm_count = delay,
			.reload_count = 0,
			.flags.auto_reload_on_alarm = true,
	};
	ESP_ERROR_CHECK(gptimer_set_alarm_action(timer_handle, &alarm_config));

	gptimer_event_callbacks_t cbs_config = {
			.on_alarm = timer_isr_handler
	};
	ESP_ERROR_CHECK(gptimer_register_event_callbacks(timer_handle, &cbs_config, NULL));
	ESP_ERROR_CHECK(gptimer_enable(timer_handle));
	ESP_ERROR_CHECK(gptimer_start(timer_handle));
}


void init_timer_display(uint64_t delay){
	gptimer_handle_t timer_handle = NULL;

	gptimer_config_t timer_config ={
			.clk_src = GPTIMER_CLK_SRC_DEFAULT,
			.direction = GPTIMER_COUNT_UP,
			.resolution_hz = 1000 * 1000 // 1 tick = 1us
	};
	ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &timer_handle));

	// Alarm config

	gptimer_alarm_config_t alarm_config = {
			.alarm_count = delay,
			.reload_count = 0,
			.flags.auto_reload_on_alarm = true,
	};
	ESP_ERROR_CHECK(gptimer_set_alarm_action(timer_handle, &alarm_config));

	gptimer_event_callbacks_t cbs_config = {
			.on_alarm = timer_display_isr_handler
	};
	ESP_ERROR_CHECK(gptimer_register_event_callbacks(timer_handle, &cbs_config, NULL));
	ESP_ERROR_CHECK(gptimer_enable(timer_handle));
	ESP_ERROR_CHECK(gptimer_start(timer_handle));
}


void Wifi_Init(){
	ESP_ERROR_CHECK(esp_netif_init());
	ESP_ERROR_CHECK(esp_event_loop_create_default());
	esp_netif_create_default_wifi_sta();
	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT() ;
	esp_wifi_init(&cfg);
	esp_wifi_set_mode(WIFI_MODE_STA);
	wifi_config_t wifi_config = {
			.sta = {
					.ssid = EXAMPLE_ESP_WIFI_SSID,
					.password = EXAMPLE_ESP_WIFI_PASS,
					.threshold.authmode = WIFI_AUTH_WPA2_PSK,
					.sae_pwe_h2e = WPA3_SAE_PWE_BOTH,
			},
	};
	esp_wifi_set_config(WIFI_IF_STA,&wifi_config);
	esp_err_t result = esp_wifi_start();
	if(!result)
		esp_wifi_connect();
}

static void lv_tick_task(void *arg) {
    (void) arg;

    lv_tick_inc(LV_TICK_PERIOD_MS);
}
