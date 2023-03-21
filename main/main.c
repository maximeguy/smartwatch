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
#include "esp_log.h"
#include "sdkconfig.h"

#include "SW_I2c_Driver.h"
#include "lsm6dso_reg.h"
#include "lis2mdl_reg.h"
#include "SW_IMU_Driver.h"

#include "lvgl/lvgl.h"
#include "lvgl_helpers.h"

#include "images/bluetooth.c"

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

/********I2c Specific*********/
#define I2C_MASTER_SCL_IO 19
#define I2C_MASTER_SDA_IO 18
#define LSM6DSO_INT1 25 //GPIO25 => RTC_GPIO6
/********I2c Specific*********/

#define deg_to_rad(angleInDegrees) ((angleInDegrees) * M_PI / 180.0)
#define rad_to_deg(angleInRadians) ((angleInRadians) * 180.0 / M_PI)


/***************Thread Safe Print****************/
esp_err_t SW_SafePrint(SemaphoreHandle_t* Jeton,const char* fmt, ...);
/***************Thread Safe Print****************/

/**********************
 *  STATIC PROTOTYPES
 **********************/
static void lv_tick_task(void *arg);
static void blink_task(void *arg);
static void state_machine();
static void create_lvgl_gui(void);

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
/************LIS2MDL Variables*********/
static int16_t data_raw_magnetic[3];
static int16_t data_raw_temperature;
static float magnetic_mG[3];
static float temperature_degC;
/************LIS2MDL Variables*********/

uint8_t current_screen = 0;
// Don't forget to free memory
lv_obj_t ** screens;


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
	uint16_t steps;
	lsm6dso_emb_sens_t emb_sens;
	lsm6dso_steps_reset(&Lsm6dso_dev_ctx);
	/* Enable pedometer */
	lsm6dso_pedo_sens_set(&Lsm6dso_dev_ctx, LSM6DSO_FALSE_STEP_REJ_ADV_MODE);
	emb_sens.step = PROPERTY_ENABLE;
	emb_sens.step_adv = PROPERTY_ENABLE;
	lsm6dso_embedded_sens_set(&Lsm6dso_dev_ctx, &emb_sens);
	for(;;){
		lsm6dso_number_of_steps_get(&Lsm6dso_dev_ctx, &steps); //step Counting
		SW_SafePrint(&UART_Jeton,"steps :%d\r\n", steps);
		vTaskDelay(1000/portTICK_PERIOD_MS);
	}
}


void LIS2MDL_TASK(void * pvParameters){
	uint8_t reg;
	double norm_x,norm_y,angle,angle_degree;
	for(;;){
		/* Read output only if new value is available */
		lis2mdl_mag_data_ready_get(&Lis2mdl_dev_ctx, &reg);
		if (reg) {
			/* Read magnetic field data */
			memset(data_raw_magnetic, 0x00, 3 * sizeof(int16_t));
			lis2mdl_magnetic_raw_get(&Lis2mdl_dev_ctx, data_raw_magnetic);
			magnetic_mG[0] = lis2mdl_from_lsb_to_mgauss( data_raw_magnetic[0]);
			magnetic_mG[1] = lis2mdl_from_lsb_to_mgauss( data_raw_magnetic[1]);
			magnetic_mG[2] = lis2mdl_from_lsb_to_mgauss( data_raw_magnetic[2]);
			//SW_SafePrint(&UART_Jeton,"Mag field [mG]:%4.2f\t%4.2f\t%4.2f\r\n",magnetic_mG[0], magnetic_mG[1], magnetic_mG[2]);
			 norm_x=magnetic_mG[0]/sqrt(magnetic_mG[0]*magnetic_mG[0] + magnetic_mG[1]*magnetic_mG[1]+magnetic_mG[2]*magnetic_mG[2]);
			 norm_y=magnetic_mG[1]/sqrt(magnetic_mG[0]*magnetic_mG[0] + magnetic_mG[1]*magnetic_mG[1]+magnetic_mG[2]*magnetic_mG[2]);
			 angle = atan2(norm_x,norm_y);
			 angle_degree = angle*(180/M_PI)-1.1333; // 1.1333 poitiers-declination

			// print the result
			SW_SafePrint(&UART_Jeton, "The North direction relative to the sensor: %5.1f deg\r\n", angle_degree);
			/* Read temperature data */
			/*
			memset(&data_raw_temperature, 0x00, sizeof(int16_t));
			lis2mdl_temperature_raw_get(&Lis2mdl_dev_ctx, &data_raw_temperature);
			temperature_degC =lis2mdl_from_lsb_to_celsius(data_raw_temperature);
			SW_SafePrint(&UART_Jeton,"Temperature [degC]:%6.2f\r\n",temperature_degC);*/
		}
		vTaskDelay(2000/portTICK_PERIOD_MS);
	}
}

static void create_lvgl_gui(void)
{
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
	lv_obj_align(day_lbl, NULL, LV_ALIGN_CENTER, 0, 20);

	date_lbl =  lv_label_create(main_screen, NULL);
	lv_label_set_text(date_lbl, "18/02/23");
	lv_obj_align(date_lbl, NULL, LV_ALIGN_CENTER, 0, 40);

	steps_lbl =  lv_label_create(main_screen, NULL);
	lv_label_set_text(steps_lbl, "432");
	lv_obj_align(steps_lbl, NULL, LV_ALIGN_CENTER, 0, 80);

	screens[0] = main_screen;
	lv_scr_load(main_screen);

	/*********************
	 *      COMPASS SCREEN
	 *********************/
	static lv_style_t style_line;
	lv_style_init(&style_line);
	lv_style_set_line_width(&style_line, LV_STATE_DEFAULT, 2);
	lv_style_set_line_color(&style_line, LV_STATE_DEFAULT, LV_COLOR_BLUE);
	lv_style_set_line_rounded(&style_line, LV_STATE_DEFAULT, true);

	lv_obj_t * compass_screen = lv_obj_create(NULL, NULL);
	uint8_t compass_radius = 80;
	uint8_t compass_tick_len = 5;
	for (int i = 0 ; i < 36; i ++){
		float rad = deg_to_rad(i*10);
		float x = cos(rad)*compass_radius;
		float y = sin(rad)*compass_radius;
		float xx = cos(rad)*(compass_radius+compass_tick_len);
		float yy = sin(rad)*(compass_radius+compass_tick_len);
		ESP_LOGI(TAG, "deg : %d°; rad : %f; x : %f; y : %f", i*10, rad, x, y);
		lv_point_t  compass_tick[] = {{x, y}, {xx, yy}};
		lv_obj_t * line1= lv_line_create(compass_screen, NULL);
		lv_line_set_points(line1, compass_tick, 5);     /*Set the points*/
		lv_obj_add_style(line1, LV_OBJ_PART_MAIN, &style_line);     /*Set the points*/
		//lv_obj_align(line1, NULL, LV_ALIGN_CENTER, x,y);
		lv_obj_set_pos(line1, x+160,y+120);
	}

	lv_obj_t * north_lbl =  lv_label_create(compass_screen, NULL);
	lv_label_set_text(north_lbl, "N");
	lv_obj_align(north_lbl, NULL, LV_ALIGN_CENTER, 0, -compass_radius-20);

	lv_obj_t * east_lbl =  lv_label_create(compass_screen, NULL);
	lv_label_set_text(east_lbl, "E");
	lv_obj_align(east_lbl, NULL, LV_ALIGN_CENTER, -compass_radius-20,0 );

	lv_obj_t * south_lbl =  lv_label_create(compass_screen, NULL);
	lv_label_set_text(south_lbl, "S");
	lv_obj_align(south_lbl, NULL, LV_ALIGN_CENTER, 0, compass_radius+20);

	lv_obj_t * west_lbl =  lv_label_create(compass_screen, NULL);
	lv_label_set_text(west_lbl, "W");
	lv_obj_align(west_lbl, NULL, LV_ALIGN_CENTER, compass_radius+20, 0);

	lv_obj_t * compass_lbl =  lv_label_create(compass_screen, NULL);
	lv_label_set_text(compass_lbl, "COMPASS");
	lv_obj_align(compass_lbl, NULL, LV_ALIGN_CENTER, 0, 0);

	screens[1] = compass_screen;

	/*********************
	 *      METEO SCREEN
	 *********************/
	lv_obj_t * weather_screen = lv_obj_create(NULL, NULL);

	lv_obj_t * weather_lbl =  lv_label_create(weather_screen, NULL);
	lv_label_set_text(weather_lbl, "METEO");
	lv_obj_align(weather_lbl, NULL, LV_ALIGN_CENTER, 0, 0);

	screens[2] = weather_screen;

//	lv_style_t my_style;
//	lv_style_init(&my_style);
//	lv_style_set_text_font(&my_style, LV_STATE_DEFAULT, &lv_font_montserrat_28);
//	label =  lv_label_create(lv_scr_act(), NULL);
//	lv_label_set_text(label, "Hello world!");
//	lv_obj_add_style(label, LV_OBJ_PART_MAIN, &my_style);
//	lv_obj_align(label, NULL, LV_ALIGN_CENTER, 0, -30);
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
	uint8_t blink_state = 1;
	uint8_t display_state = 0;

	QueueSetMemberHandle_t received_smphr;

	vTaskResume(blink_task_handle);

	while(1){
		received_smphr = xQueueSelectFromSet(smphr_qs, portMAX_DELAY);
		if (received_smphr == btn_smphr){
			ESP_LOGE(TAG, "BOOT button toggle : change screen");
			xSemaphoreTake(btn_smphr,0);
			current_screen++;
			if (current_screen >= N_SCREENS) current_screen = 0;
			ESP_LOGI(TAG, "Current screen : %d", current_screen);
			lv_scr_load(screens[current_screen]);
			blink_state = !blink_state;
			if(blink_state == 1)vTaskResume(blink_task_handle);
			else vTaskSuspend(blink_task_handle);
		}
		else if (received_smphr == clock_smphr){
			ESP_LOGI(TAG, "Tick (second).");
			xSemaphoreTake(clock_smphr,0);
			display_state = !display_state;
			if(display_state == 1)lv_label_set_text(time_lbl, "10:46");
			else lv_label_set_text(time_lbl, "10 46");
		}
		else if (received_smphr == gui_smphr){
			ESP_LOGI(TAG, "Display update.");
			xSemaphoreTake(gui_smphr,0);
			lv_task_handler();
		}
	}

}

void app_main(void)
{
    /* Configure the IOMUX register for pad BLINK_GPIO (some pads are
       muxed to GPIO on reset already, but some default to other
       functions and need to be switched to GPIO. Consult the
       Technical Reference for a list of pads and their default
       functions.)
    */
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
	SW_Lsm6dso6_Init_Config(Lsm6dso_dev_ctx);
	SW_Lis2mdl_Init_Config(Lis2mdl_dev_ctx);

	xTaskCreate(StepCounter, "StepCounter", 10000, NULL, 1, &StepCounter_Handler);
	vTaskSuspend(StepCounter_Handler);
	xTaskCreate(Lsm6dso_TASK, "Lsm6dso_TASK", 10000, NULL, 1, &Lsm6dso_TASK_Handler);
	vTaskSuspend(Lsm6dso_TASK_Handler);

	xTaskCreate(LIS2MDL_TASK, "LIS2MDL_TASK", 10000, NULL, 1, &LIS2MDL_TASK_Handler);
	vTaskSuspend(LIS2MDL_TASK_Handler);
	/**************LSM6DSO_INT1 ISR / Wake-up Trigger *****************/
	gpio_set_direction(LSM6DSO_INT1, GPIO_MODE_INPUT);
	gpio_set_intr_type(LSM6DSO_INT1,GPIO_INTR_POSEDGE);
	gpio_install_isr_service(0);
	gpio_isr_handler_add(LSM6DSO_INT1, Inactivity_Activity_IRQ,NULL);
	gpio_wakeup_enable(LSM6DSO_INT1, GPIO_INTR_HIGH_LEVEL);
	esp_sleep_enable_gpio_wakeup();
	/**************LSM6DSO_INT1 ISR / Wake-up Trigger *****************/




	btn_smphr = xSemaphoreCreateBinary();
	clock_smphr = xSemaphoreCreateBinary();
	gui_smphr = xSemaphoreCreateBinary();

    smphr_qs = xQueueCreateSet(2);
    xQueueAddToSet(btn_smphr,smphr_qs);
    xQueueAddToSet(clock_smphr,smphr_qs);
    xQueueAddToSet(gui_smphr,smphr_qs);

    init_lvgl();
    // Allocate ui screens array
	screens = malloc(N_SCREENS * sizeof(lv_obj_t));
    create_lvgl_gui();

    xTaskCreate(blink_task, "blink_task", 10000, NULL, 3, &blink_task_handle);
    vTaskSuspend(blink_task_handle);
    xTaskCreate(state_machine, "state_machine", 10000, NULL, 6, &state_machine_handle);

    init_clock_timer(1000 * 1000);
    init_timer_display(50 * 1000);

    ESP_LOGI(TAG, "End app_main");
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

static void lv_tick_task(void *arg) {
    (void) arg;

    lv_tick_inc(LV_TICK_PERIOD_MS);
}
