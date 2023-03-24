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
#include "SW_I2c_Driver.h"
#include "lsm6dso_reg.h"
#include "lis2mdl_reg.h"
#include "lps22hh_reg.h"
#include "hts221_reg.h"
#include "SW_IMU_Driver.h"

#include "lvgl/lvgl.h"
#include "lvgl_helpers.h"

//#include "images/battery.c"
//#include "images/news.c"
#include "images/bluetooth.c"
#include "images/ellipse.c"
//#include "images/temperature.c"
//#include "images/weather.c"
//#include "images/humidity.c"
//#include "images/pressure.c"
#include "images/steps.c"

/*********************
 *      DEFINES
 *********************/
#define TAG "lvgl_rtos"
#define BLINK_GPIO 4
#define BTN_GPIO 0
#define LV_TICK_PERIOD_MS 100
#define BLINK_PERIOD_MS 200
#define TXT_PERIOD_MS 2000
#define N_SCREENS 3
#define COMPASS_RADIUS 70

/********I2c Specific*********/
#define I2C_MASTER_SCL_IO 22//19
#define I2C_MASTER_SDA_IO 21//18
#define LSM6DSO_INT1 25 //GPIO25 => RTC_GPIO6
/********I2c Specific*********/

#define deg_to_rad(angleInDegrees) ((angleInDegrees) * M_PI / 180.0)
#define rad_to_deg(angleInRadians) ((angleInRadians) * 180.0 / M_PI)

/* The combined length of the queues and binary semaphorse that will be
added to the queue set. */
#define COMBINED_LENGTH 18

#define EXAMPLE_ESP_WIFI_SSID      "HUAWEI Y6p"
#define EXAMPLE_ESP_WIFI_PASS      "majdi123"

/***************Thread Safe Print****************/
esp_err_t SW_SafePrint(SemaphoreHandle_t* Jeton,const char* fmt, ...);
/***************Thread Safe Print****************/

/**********************
 *  STATIC PROTOTYPES
 **********************/
static void lv_tick_task(void *arg);
static void blink_task(void *arg);
static void state_machine();
static void create_screen(uint8_t screen_id);


void Wifi_Init();
/**********************
 *  HANDLERS
 **********************/
TaskHandle_t blink_task_handle = NULL;
TaskHandle_t state_machine_handle = NULL;
TaskHandle_t Lsm6dso_TASK_Handler ,StepCounter_Handler ;
TaskHandle_t LIS2MDL_TASK_Handler;
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

/**********************
 *  GLOBAL
 **********************/
lv_obj_t * label;
lv_obj_t * time_lbl;
lv_obj_t * day_lbl;
lv_obj_t * date_lbl;
lv_obj_t * steps_lbl;

stmdev_ctx_t Lsm6dso_dev_ctx;
stmdev_ctx_t Lis2mdl_dev_ctx;
/************LIS2MDL / LSM6DSO Variables*********/
typedef struct Steps {
	uint16_t CurrentSteps; //Real time Steps
	uint16_t DailySteps; // Steps/24h
} Steps;

typedef struct Weather {
	float Temperature; //Real time Steps
	float Pressure; // Steps/24h
	float Humdity;
} Weather;

/************LIS2MDL / LSM6DSO Variables*********/


uint8_t current_screen = 0;
lv_obj_t ** screens;
lv_obj_t * main_screen;
lv_obj_t * compass_screen;
lv_obj_t * weather_screen;
lv_obj_t * ellipse_img;
static lv_color_t c_a;
static lv_color_t c_b;
static lv_color_t c_c;


/**********************
 *  ISR
 **********************/

// Wake up after boot button was pressed
static void IRAM_ATTR btn_isr_handler(void* arg)
{

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(btn_smphr, &xHigherPriorityTaskWoken);
}

// Wake up after 2s from timer isr
static bool IRAM_ATTR timer_isr_handler(gptimer_handle_t timer, const gptimer_alarm_event_data_t * edata, void * user_ctx)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(clock_smphr, &xHigherPriorityTaskWoken);
    return true;
}

// Wake up after 100ms from timer isr
static bool IRAM_ATTR timer_display_isr_handler(gptimer_handle_t timer, const gptimer_alarm_event_data_t * edata, void * user_ctx)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(gui_smphr, &xHigherPriorityTaskWoken);
    return true;
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


static void IRAM_ATTR  Inactivity_Activity_IRQ(void * args){
	vTaskResume(Lsm6dso_TASK_Handler);
}


static void oneshot_timer_callback(void* arg){
	esp_light_sleep_start(); //when we reach here, it means that 20 seconds are already passed without any motion detected
}

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
			SW_SafePrint(&UART_Jeton,"INACTIVITY ==> Sleep_State: %u\n\r",all_source.sleep_state);
		}else if(!all_source.sleep_state){
			esp_timer_stop(oneshot_timer);
			SW_SafePrint(&UART_Jeton,"ACTIVITY ==> Sleep_State : %u\n\r",all_source.sleep_state);
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

		vTaskDelay(1000/portTICK_PERIOD_MS);
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
		vTaskDelay(2000/portTICK_PERIOD_MS);
	}
}

void create_screen(uint8_t screen_id){
		/*********************
		 *      MAIN SCREEN
		 *********************/
		lv_obj_t * main_screen = lv_obj_create(NULL, NULL);

		lv_obj_t * bt_icon = lv_img_create(main_screen, NULL);
		lv_img_set_src(bt_icon,&bluetooth);
		lv_obj_align(bt_icon, NULL, LV_ALIGN_CENTER, 140, -100);

		time_lbl =  lv_label_create(main_screen, NULL);
		lv_label_set_text(time_lbl, "10:46");
		lv_obj_align(time_lbl, NULL, LV_ALIGN_CENTER, 0, 0);

		day_lbl =  lv_label_create(main_screen, NULL);
		lv_label_set_text(day_lbl, "Wednesday");
		lv_obj_align(day_lbl, NULL, LV_ALIGN_CENTER, 0, -85);

		date_lbl = lv_label_create(main_screen, NULL);
		lv_label_set_text(date_lbl, "18/02/23");
		lv_obj_align(date_lbl, NULL, LV_ALIGN_CENTER, 0, -105);

		lv_obj_t * steps_icon = lv_img_create(main_screen, NULL);
		lv_img_set_src(steps_icon,&steps);
		lv_obj_align(steps_icon, NULL, LV_ALIGN_CENTER, -30, 80);

//		steps_lbl = lv_label_create(main_screen, NULL);
//		lv_label_set_text(steps_lbl, "432");
//		lv_obj_align(steps_lbl, NULL, LV_LABEL_ALIGN_LEFT, 5, 80);

		lv_draw_line_dsc_t line_dsc;
		lv_draw_line_dsc_init(&line_dsc);
		line_dsc.color=c_a;
		line_dsc.width = 8;
		line_dsc.round_end = 1;
		line_dsc.round_start = 1;

		static lv_color_t cbuf[LV_CANVAS_BUF_SIZE_TRUE_COLOR(120, 120)];

		lv_obj_t * canvas = lv_canvas_create(main_screen, NULL);
		lv_canvas_set_buffer(canvas, cbuf, 120, 120, LV_IMG_CF_TRUE_COLOR_ALPHA);
		lv_obj_align(canvas, NULL, LV_ALIGN_CENTER, 0, 0);
		lv_canvas_draw_arc(canvas, 60, 60, 60, 360-90, 300-90, &line_dsc);
		line_dsc.color=c_b;
		line_dsc.width = 6;
		lv_canvas_draw_arc(canvas, 60, 60, 50, 360-90, 190-90, &line_dsc);
		line_dsc.color=c_c;
		line_dsc.width = 4;
		lv_canvas_draw_arc(canvas, 60, 60, 42, 360-90, 260-90, &line_dsc);

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

		time_lbl =  lv_label_create(compass_screen, NULL);
		lv_label_set_text(time_lbl, "10:46");
		lv_obj_align(time_lbl, NULL, LV_ALIGN_CENTER, 0, -105);

		uint8_t compass_tick_len = 5;
		for (int i = 0 ; i < 36; i ++){
			float rad = deg_to_rad(i*10);
			float x = cos(rad)*COMPASS_RADIUS;
			float y = sin(rad)*COMPASS_RADIUS;
			float xx = cos(rad)*(COMPASS_RADIUS+compass_tick_len);
			float yy = sin(rad)*(COMPASS_RADIUS+compass_tick_len);
			//ESP_LOGI(TAG, "deg : %d°; rad : %f; x : %f; y : %f", i*10, rad, x, y);
			lv_point_t  compass_tick[] = {{x, y}, {xx, yy}};
			lv_obj_t * line1= lv_line_create(compass_screen, NULL);
			lv_line_set_points(line1, compass_tick, 5);     /*Set the points*/
			lv_obj_add_style(line1, LV_OBJ_PART_MAIN, &line_style);     /*Set the points*/
			lv_obj_set_pos(line1, x+160,y+125);
		}

		lv_obj_t * north_lbl =  lv_label_create(compass_screen, NULL);
		lv_label_set_text(north_lbl, "N");
		lv_obj_align(north_lbl, NULL, LV_ALIGN_CENTER, 0, -COMPASS_RADIUS-10);

		lv_obj_t * east_lbl =  lv_label_create(compass_screen, NULL);
		lv_label_set_text(east_lbl, "E");
		lv_obj_align(east_lbl, NULL, LV_ALIGN_CENTER, -COMPASS_RADIUS-20,10 );

		lv_obj_t * south_lbl =  lv_label_create(compass_screen, NULL);
		lv_label_set_text(south_lbl, "S");
		lv_obj_align(south_lbl, NULL, LV_ALIGN_CENTER, 0, COMPASS_RADIUS+20);

		lv_obj_t * west_lbl =  lv_label_create(compass_screen, NULL);
		lv_label_set_text(west_lbl, "W");
		lv_obj_align(west_lbl, NULL, LV_ALIGN_CENTER, COMPASS_RADIUS+20, 10);

		lv_obj_t * compass_lbl =  lv_label_create(compass_screen, NULL);
		lv_label_set_text(compass_lbl, "COMPASS");
		lv_obj_align(compass_lbl, NULL, LV_ALIGN_CENTER, 0, 10);

		ellipse_img = lv_img_create(compass_screen, NULL);
		lv_img_set_src(ellipse_img,&ellipse);
		lv_obj_align(ellipse_img, NULL, LV_ALIGN_CENTER, 0, 0);
		lv_obj_set_pos(ellipse_img, cos(deg_to_rad(0))*COMPASS_RADIUS+155,sin(deg_to_rad(0))*COMPASS_RADIUS+120);

		screens[1] = compass_screen;

		/*********************
		 *      METEO SCREEN
		 *********************/
		lv_obj_t * weather_screen = lv_obj_create(NULL, NULL);

		time_lbl =  lv_label_create(weather_screen, NULL);
		lv_label_set_text(time_lbl, "10:46");
		lv_obj_align(time_lbl, NULL, LV_ALIGN_CENTER, 0, -105);

//		lv_obj_t * weather_icon = lv_img_create(weather_screen, NULL);
//		lv_img_set_src(weather_icon,&weather);
//		lv_obj_align(weather_icon, NULL, LV_ALIGN_CENTER, 0, -60);

//		lv_obj_t * temp_icon = lv_img_create(weather_screen, NULL);
//		lv_img_set_src(temp_icon,&temperature);
//		lv_obj_align(temp_icon, NULL, LV_ALIGN_CENTER, 100, -20);

		lv_obj_t * temp_lbl =  lv_label_create(weather_screen, NULL);
		lv_label_set_text(temp_lbl, "12.5°C");
		lv_obj_align(temp_lbl, NULL, LV_ALIGN_CENTER, 100, 15);

//		lv_obj_t * hum_icon = lv_img_create(weather_screen, NULL);
//		lv_img_set_src(hum_icon,&humidity);
//		lv_obj_align(hum_icon, NULL, LV_ALIGN_CENTER, -100, -20);

		lv_obj_t * hum_lbl =  lv_label_create(weather_screen, NULL);
		lv_label_set_text(hum_lbl, "55%");
		lv_obj_align(hum_lbl, NULL, LV_ALIGN_CENTER, -100, 15);

//		lv_obj_t * press_icon = lv_img_create(weather_screen, NULL);
//		lv_img_set_src(press_icon,&pressure);
//		lv_obj_align(press_icon, NULL, LV_ALIGN_CENTER, 0, 40);

		lv_obj_t * press_lbl =  lv_label_create(weather_screen, NULL);
		lv_label_set_text(press_lbl, "12.5°C");
		lv_obj_align(press_lbl, NULL, LV_ALIGN_CENTER, 0, 75);

		screens[2] = weather_screen;

		lv_scr_load(main_screen);
}

//static void create_screen(uint8_t screen_id){
//
//
//	switch(screen_id) {
//		case 0:
//		/*********************
//		 *      MAIN SCREEN
//		 *********************/
//		lv_obj_t * main_screen = lv_obj_create(NULL, NULL);
//
//		lv_obj_t * bt_icon = lv_img_create(main_screen, NULL);
//		lv_img_set_src(bt_icon,&bluetooth);
//		lv_obj_align(bt_icon, NULL, LV_ALIGN_CENTER, 140, -100);
//
//		time_lbl =  lv_label_create(main_screen, NULL);
//		lv_label_set_text(time_lbl, "10:46");
//		lv_obj_align(time_lbl, NULL, LV_ALIGN_CENTER, 0, 0);
//
//		day_lbl =  lv_label_create(main_screen, NULL);
//		lv_label_set_text(day_lbl, "Wednesday");
//		lv_obj_align(day_lbl, NULL, LV_ALIGN_CENTER, 0, -85);
//
//		date_lbl = lv_label_create(main_screen, NULL);
//		lv_label_set_text(date_lbl, "18/02/23");
//		lv_obj_align(date_lbl, NULL, LV_ALIGN_CENTER, 0, -105);
//
//		lv_obj_t * steps_icon = lv_img_create(main_screen, NULL);
//		lv_img_set_src(steps_icon,&steps);
//		lv_obj_align(steps_icon, NULL, LV_ALIGN_CENTER, -30, 80);
//
//		steps_lbl = lv_label_create(main_screen, NULL);
//		lv_label_set_text(steps_lbl, "432");
//		lv_obj_align(steps_lbl, NULL, LV_LABEL_ALIGN_LEFT, 5, 80);
//
//		lv_draw_line_dsc_t line_dsc;
//		lv_draw_line_dsc_init(&line_dsc);
//		line_dsc.color=c_a;
//		line_dsc.width = 8;
//		line_dsc.round_end = 1;
//		line_dsc.round_start = 1;
//
//		static lv_color_t cbuf[LV_CANVAS_BUF_SIZE_TRUE_COLOR(120, 120)];
//
//		lv_obj_t * canvas = lv_canvas_create(main_screen, NULL);
//		lv_canvas_set_buffer(canvas, cbuf, 120, 120, LV_IMG_CF_TRUE_COLOR_ALPHA);
//		lv_obj_align(canvas, NULL, LV_ALIGN_CENTER, 0, 0);
//		lv_canvas_draw_arc(canvas, 60, 60, 60, 360-90, 300-90, &line_dsc);
//		line_dsc.color=c_b;
//		line_dsc.width = 6;
//		lv_canvas_draw_arc(canvas, 60, 60, 50, 360-90, 190-90, &line_dsc);
//		line_dsc.color=c_c;
//		line_dsc.width = 4;
//		lv_canvas_draw_arc(canvas, 60, 60, 42, 360-90, 260-90, &line_dsc);
//
//		screens[0] = main_screen;
//		break;
//		case 1:
//		/*********************
//		 *      COMPASS SCREEN
//		 *********************/
//		static lv_style_t line_style;
//		lv_style_init(&line_style);
//		lv_style_set_line_width(&line_style, LV_STATE_DEFAULT, 2);
//		lv_style_set_line_color(&line_style, LV_STATE_DEFAULT, LV_COLOR_BLUE);
//		lv_style_set_line_rounded(&line_style, LV_STATE_DEFAULT, true);
//
//		lv_obj_t * compass_screen = lv_obj_create(NULL, NULL);
//
//		time_lbl =  lv_label_create(compass_screen, NULL);
//		lv_label_set_text(time_lbl, "10:46");
//		lv_obj_align(time_lbl, NULL, LV_ALIGN_CENTER, 0, -105);
//
//		uint8_t compass_tick_len = 5;
//		for (int i = 0 ; i < 36; i ++){
//			float rad = deg_to_rad(i*10);
//			float x = cos(rad)*COMPASS_RADIUS;
//			float y = sin(rad)*COMPASS_RADIUS;
//			float xx = cos(rad)*(COMPASS_RADIUS+compass_tick_len);
//			float yy = sin(rad)*(COMPASS_RADIUS+compass_tick_len);
//			//ESP_LOGI(TAG, "deg : %d°; rad : %f; x : %f; y : %f", i*10, rad, x, y);
//			lv_point_t  compass_tick[] = {{x, y}, {xx, yy}};
//			lv_obj_t * line1= lv_line_create(compass_screen, NULL);
//			lv_line_set_points(line1, compass_tick, 5);     /*Set the points*/
//			lv_obj_add_style(line1, LV_OBJ_PART_MAIN, &line_style);     /*Set the points*/
//			lv_obj_set_pos(line1, x+160,y+125);
//		}
//
//		lv_obj_t * north_lbl =  lv_label_create(compass_screen, NULL);
//		lv_label_set_text(north_lbl, "N");
//		lv_obj_align(north_lbl, NULL, LV_ALIGN_CENTER, 0, -COMPASS_RADIUS-10);
//
//		lv_obj_t * east_lbl =  lv_label_create(compass_screen, NULL);
//		lv_label_set_text(east_lbl, "E");
//		lv_obj_align(east_lbl, NULL, LV_ALIGN_CENTER, -COMPASS_RADIUS-20,10 );
//
//		lv_obj_t * south_lbl =  lv_label_create(compass_screen, NULL);
//		lv_label_set_text(south_lbl, "S");
//		lv_obj_align(south_lbl, NULL, LV_ALIGN_CENTER, 0, COMPASS_RADIUS+20);
//
//		lv_obj_t * west_lbl =  lv_label_create(compass_screen, NULL);
//		lv_label_set_text(west_lbl, "W");
//		lv_obj_align(west_lbl, NULL, LV_ALIGN_CENTER, COMPASS_RADIUS+20, 10);
//
//		lv_obj_t * compass_lbl =  lv_label_create(compass_screen, NULL);
//		lv_label_set_text(compass_lbl, "COMPASS");
//		lv_obj_align(compass_lbl, NULL, LV_ALIGN_CENTER, 0, 10);
//
//		ellipse_img = lv_img_create(compass_screen, NULL);
//		lv_img_set_src(ellipse_img,&ellipse);
//		lv_obj_align(ellipse_img, NULL, LV_ALIGN_CENTER, 0, 0);
//		lv_obj_set_pos(ellipse_img, cos(deg_to_rad(0))*COMPASS_RADIUS+155,sin(deg_to_rad(0))*COMPASS_RADIUS+120);
//
//		screens[1] = compass_screen;
//		break;
//		case 2:
//		/*********************
//		 *      METEO SCREEN
//		 *********************/
//		lv_obj_t * weather_screen = lv_obj_create(NULL, NULL);
//
//		time_lbl =  lv_label_create(weather_screen, NULL);
//		lv_label_set_text(time_lbl, "10:46");
//		lv_obj_align(time_lbl, NULL, LV_ALIGN_CENTER, 0, -105);
//
//		lv_obj_t * weather_icon = lv_img_create(weather_screen, NULL);
//		lv_img_set_src(weather_icon,&weather);
//		lv_obj_align(weather_icon, NULL, LV_ALIGN_CENTER, 0, -60);
//
//		lv_obj_t * temp_icon = lv_img_create(weather_screen, NULL);
//		lv_img_set_src(temp_icon,&temperature);
//		lv_obj_align(temp_icon, NULL, LV_ALIGN_CENTER, 100, -20);
//
//		lv_obj_t * temp_lbl =  lv_label_create(weather_screen, NULL);
//		lv_label_set_text(temp_lbl, "12.5°C");
//		lv_obj_align(temp_lbl, NULL, LV_ALIGN_CENTER, 100, 15);
//
//		lv_obj_t * hum_icon = lv_img_create(weather_screen, NULL);
//		lv_img_set_src(hum_icon,&humidity);
//		lv_obj_align(hum_icon, NULL, LV_ALIGN_CENTER, -100, -20);
//
//		lv_obj_t * hum_lbl =  lv_label_create(weather_screen, NULL);
//		lv_label_set_text(hum_lbl, "55%");
//		lv_obj_align(hum_lbl, NULL, LV_ALIGN_CENTER, -100, 15);
//
//		lv_obj_t * press_icon = lv_img_create(weather_screen, NULL);
//		lv_img_set_src(press_icon,&pressure);
//		lv_obj_align(press_icon, NULL, LV_ALIGN_CENTER, 0, 40);
//
//		lv_obj_t * press_lbl =  lv_label_create(weather_screen, NULL);
//		lv_label_set_text(press_lbl, "12.5°C");
//		lv_obj_align(press_lbl, NULL, LV_ALIGN_CENTER, 0, 75);
//
//		screens[2] = weather_screen;
//		break;
//	}
//	SW_SafePrint(&UART_Jeton, "WTL screen %d\n",current_screen);
//	lv_scr_load(screens[screen_id]);
////	for (uint8_t i = 0; i< N_SCREENS; i++){
////		if (i != screen_id) lv_obj_clean(screens[i]);
////	}
//}


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

// Toggle led
void blink_task(void *arg){

    while(1){
        /* Blink off (output high) */
        gpio_set_level(BLINK_GPIO, 1);
        vTaskDelay(500 / portTICK_PERIOD_MS);
        /* Blink on (output low) */
        gpio_set_level(BLINK_GPIO, 0);
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

void state_machine()
{
	//uint8_t blink_state = 1;
	uint8_t display_state = 0;
	double North_Dir=0.0;
	Steps steps;
	QueueSetMemberHandle_t received_smphr;

	//vTaskResume(blink_task_handle);

	while(1){
		received_smphr = xQueueSelectFromSet(smphr_qs, portMAX_DELAY);
		if (received_smphr == btn_smphr){
			xSemaphoreTake(btn_smphr,0);
			current_screen++;
			SW_SafePrint(&UART_Jeton, "#SM BOOT button toggle : current screen : %d\n",current_screen);
			if (current_screen >= N_SCREENS) current_screen = 0;
			//create_screen(current_screen);
			lv_scr_load(screens[current_screen]);
			if(current_screen == 1){
				vTaskResume(LIS2MDL_TASK_Handler);
			}
			else vTaskSuspend(LIS2MDL_TASK_Handler);
//			blink_state = !blink_state;
//			if(blink_state == 1)vTaskResume(blink_task_handle);
//			else vTaskSuspend(blink_task_handle);
		}
		else if (received_smphr == clock_smphr){
			SW_SafePrint(&UART_Jeton, "#SM Tick (second).\n");
			xSemaphoreTake(clock_smphr,0);
			display_state = !display_state;
			if(display_state == 1)lv_label_set_text(time_lbl, "10:46");
			else lv_label_set_text(time_lbl, "10 46");
		}
		else if (received_smphr == gui_smphr){
			SW_SafePrint(&UART_Jeton, "#SM Display update.\n");
			xSemaphoreTake(gui_smphr,0);
			lv_task_handler();
		}else if (received_smphr == North_DirQ){
			if( North_DirQ != NULL ){
				if(xQueueReceive(North_DirQ, &North_Dir,100)== pdPASS){
					SW_SafePrint(&UART_Jeton, "#SM Received North_DirQ : %f.\n",North_Dir);
					//lv_obj_set_pos(ellipse_img, cos(deg_to_rad(North_Dir))*COMPASS_RADIUS+155,sin(deg_to_rad(North_Dir))*COMPASS_RADIUS+120);
				}
			}
		}else if(received_smphr == StepsQ){
			xQueueReceive(StepsQ, &steps, 100);
		}
	}

}

void app_main(void)
{

	c_a = lv_color_make(80, 97, 191);
	c_b = lv_color_make(95, 114, 217);
	c_c = lv_color_make(148, 162, 242);
	gpio_reset_pin(BTN_GPIO);
	gpio_set_direction(BTN_GPIO, GPIO_MODE_INPUT);
	gpio_set_intr_type(BTN_GPIO, GPIO_INTR_NEGEDGE);
	gpio_install_isr_service(0);
	gpio_isr_handler_add(BTN_GPIO, btn_isr_handler, (void*)GPIO_NUM_0);

	gpio_reset_pin(4);
	gpio_set_direction(4, GPIO_MODE_OUTPUT);
	gpio_set_level(4, 0);



	//UART Semaphore creation
	UART_Jeton=xSemaphoreCreateBinary();
	xSemaphoreGive(UART_Jeton);
	//I2c0 Semaphore Creation
	I2c_Jeton=xSemaphoreCreateBinary();
	xSemaphoreGive(I2c_Jeton);

	SW_I2c_Master_Init(I2C_NUM_0,I2C_MASTER_SCL_IO,I2C_MASTER_SDA_IO);
	Lsm6dso_dev_ctx = SW_Mems_Interface_Init(I2C_NUM_0,0); //0=>Lsm6dso
	Lis2mdl_dev_ctx = SW_Mems_Interface_Init(I2C_NUM_0,1);//1=>Lis2mdl
	//SW_Lsm6dso6_Init_Config(Lsm6dso_dev_ctx);
	SW_Lis2mdl_Init_Config(Lis2mdl_dev_ctx);

	xTaskCreate(StepCounter, "StepCounter", 10000, NULL, 1, &StepCounter_Handler);
	vTaskSuspend(StepCounter_Handler);
	xTaskCreate(Lsm6dso_TASK, "Lsm6dso_TASK", 10000, NULL, 2, &Lsm6dso_TASK_Handler);
	vTaskSuspend(Lsm6dso_TASK_Handler);
	xTaskCreate(LIS2MDL_TASK, "LIS2MDL_TASK", 10000, NULL, 1, &LIS2MDL_TASK_Handler);
	vTaskSuspend(LIS2MDL_TASK_Handler);

	/**************LSM6DSO_INT1 ISR / Wake-up Trigger *****************/
//	gpio_set_direction(LSM6DSO_INT1, GPIO_MODE_INPUT);
//	gpio_set_intr_type(LSM6DSO_INT1,GPIO_INTR_POSEDGE);
//	gpio_install_isr_service(0);
//	//gpio_isr_handler_add(LSM6DSO_INT1, Inactivity_Activity_IRQ,NULL);
//	gpio_wakeup_enable(LSM6DSO_INT1, GPIO_INTR_HIGH_LEVEL);
//	esp_sleep_enable_gpio_wakeup();
	/**************LSM6DSO_INT1 ISR / Wake-up Trigger *****************/



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

    xTaskCreate(blink_task, "blink_task", 10000, NULL, 3, &blink_task_handle);
    vTaskSuspend(blink_task_handle);
    xTaskCreate(state_machine, "state_machine", 10000, NULL, 6, &state_machine_handle);

    init_clock_timer(1000 * 1000);
    init_timer_display(500 * 1000);
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
