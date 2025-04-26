#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_http_client.h"
#include "esp_https_ota.h"
#include "protocol_examples_common.h"
#include "string.h"
#include "nvs.h"
#include "nvs_flash.h"

#include <time.h>
#include <sys/time.h>
#include "freertos/event_groups.h"
#include "esp_attr.h"
#include "esp_sleep.h"
#include "esp_sntp.h"

#include "driver/gpio.h"

#include <driver/spi_master.h>
#include <stdio.h>
#include <u8g2.h>

#include "sdkconfig.h"
#include "u8g2_esp32_hal.h"

#include "ds18b20.h" 

#include <stdint.h>
#include <stddef.h>
#include "esp_wifi.h"

#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"

#include "mqtt_client.h"
#include "esp_smartconfig.h"
#include "lora.h"
#include "math.h"



#include "ACS71020.h"


/*#define WIFI_SSID "ZONG MBB-E8372-6714"
#define WIFI_PASS "15625888"*/

#define Controller_Name "Watering"
#define BROKER_URL "mqtt://szabolcskiss.ddns.net:1883"


const esp_app_desc_t *app_desc;
char new_Firmware_version[16];

//extern int read_ACS71020(int chip_addr, int data_addr, int *X0, int *X1,int *X2,int *X3);
extern double MeasuredValue(int ACS71020_address, int reg_address, long mask, int shiftleft,int shiftright,int fractional, float fullscale );
//extern void readEeprom(int ACS71020_address_default);
//extern void readShadow(int ACS71020_address_default);
extern int write_ACS71020(int chip_addr, int data_addr, int regValue);
extern long read_ACS71020_register(int ACS71020_address, int reg_address, long mask, int shiftleft,int shiftright);
void read_ACS71020_register2(int reg_addr,long value);

#define ACS71020_address_default 0x66
#define Rs 1000.0
#define R1_4 2000000.0


typedef enum {MAN_ON,PROG_STARTED,MAN_RESUMED,PROG_RESUMED,INIT,ENABLED,DISABLED,MAN_OFF,SUSPENDED,DELAY,PROG_FINISHED,PROG_END,IDLE,REBOOTED,NOREQUEST} T_states;
char* str_states[15]={"MAN_ON","PROG_STARTED","MAN_RESUMED","PROG_RESUMED","INIT","ENABLED","DISABLED","MAN_OFF","SUSPENDED","DELAY","PROG_FINISHED","PROG_END","IDLE","REBOOTED","NOREQUEST"};

T_states channel_states_before_suspend[3]={INIT,INIT,INIT};
T_states channel_states[3]={INIT,INIT,INIT};
T_states manual_change_request[3]={NOREQUEST,NOREQUEST,NOREQUEST};

static const int CONNECTED_BIT = BIT0;
static const int ESPTOUCH_DONE_BIT = BIT1;
static const int MQTT_CONNECTED_BIT = BIT2;
static EventGroupHandle_t s_wifi_event_group;
int smartconfig_started=0;

int measure_mode=0;
int pump_restart_delay=10;
int pump_suspended=0;
time_t pump_protection_started_at;

int prev_daily_ontime[3]={0,0,0};
int SNTP_synchronized=0;
int FW_update_available=0;
#define OTA_URL_SIZE 256 
//char url_buf[OTA_URL_SIZE]="https://github.com/szabolcskiss70/watering_controller/raw/main/release/watering_controller.bin";
char url_buf[OTA_URL_SIZE]="http://szabolcskiss.ddns.net/watering_controller.bin";
void ota_update_task(void *pvParameter);
void switch_channel(int ch, T_states status);
void Save_data_to_NVS();
static time_t turn_on_time[3]={-1,-1,-1};
time_t last_pump_on_time=0;
time_t sink_time=0;
time_t fill_time=0;
int channel_disabled[3]={0,0,0};

static SemaphoreHandle_t mutex;

static const char *TAG = "watering";
static int prevhour=0;
float water_level=0;
float protection_level_off=0;
float protection_level_on=0;
int wifi_retry_count=0;
int lastloopnow=0;

const int DS_PIN = 17; //GPIO where you connected ds18b20


#define GPIO_OUTPUT_OUT_1    23
#define GPIO_OUTPUT_OUT_2    12
#define GPIO_OUTPUT_OUT_3    2
#define GPIO_OUTPUT_OUT_4    25
#define GPIO_OUTPUT_RELAY    21

#define ISOLATED_INPUT_1    36
#define ISOLATED_INPUT_2    39

#define PRG_BUTTON 0



#define GPIO_OUTPUT_PIN_SEL  (1ULL<<GPIO_OUTPUT_OUT_1 | 1ULL<<GPIO_OUTPUT_OUT_2 | 1ULL<<GPIO_OUTPUT_OUT_3 | 1ULL<<GPIO_OUTPUT_OUT_4 | 1ULL<<GPIO_OUTPUT_RELAY)
#define GPIO_INPUT_PIN_SEL (1ULL<<ISOLATED_INPUT_1 | 1ULL<<ISOLATED_INPUT_2)


float temperature=0;







int on_time[3]={0,0,0};
double powerconsumptionWs=0;


esp_mqtt_client_handle_t mqtt_client;
int mqtt_connected = 0;



// SDA - GPIO21
#define PIN_SDA 4

// SCL - GPIO22
#define PIN_SCL 15

#define PIN_OLED_RESET 16

#include "driver/adc.h"
#include "esp_adc_cal.h"

#define DEFAULT_VREF    1100        //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   64          //Multisampling

static esp_adc_cal_characteristics_t *adc_chars;
static const adc_channel_t channel = ADC_CHANNEL_2;  
static const adc_atten_t atten = ADC_ATTEN_DB_11;
static const adc_unit_t unit = ADC_UNIT_1;

static void check_efuse()
{
    //Check TP is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
        printf("eFuse Two Point: Supported\n");
    } else {
        printf("eFuse Two Point: NOT supported\n");
    }

    //Check Vref is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) {
        printf("eFuse Vref: Supported\n");
    } else {
        printf("eFuse Vref: NOT supported\n");
    }
}

static void print_char_val_type(esp_adc_cal_value_t val_type)
{
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        printf("Characterized using Two Point Value\n");
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        printf("Characterized using eFuse Vref\n");
    } else {
        printf("Characterized using Default Vref\n");
    }
}




typedef struct{
	int32_t on_time;
	int32_t off_time;
	char weekdays[8];
	int32_t duration;
} T_chedule_data;


T_chedule_data Chedule_array[3][10]; 



int watering=0;

/* Variable holding number of times ESP32 restarted since first boot.
 * It is placed into RTC memory using RTC_DATA_ATTR and
 * maintains its value when ESP32 wakes from deep sleep.
 */
RTC_DATA_ATTR static int boot_count = 0;

static void obtain_time(void);
static void initialize_sntp(void);

#ifdef CONFIG_SNTP_TIME_SYNC_METHOD_CUSTOM
void sntp_sync_time(struct timeval *tv)
{
   settimeofday(tv, NULL);
   ESP_LOGI(TAG, "Time is synchronized from custom code");
   sntp_set_sync_status(SNTP_SYNC_STATUS_COMPLETED);
}
#endif

void time_sync_notification_cb(struct timeval *tv)
{
 //   ESP_LOGI(TAG, "Notification of a time synchronization event");
	SNTP_synchronized=1;
	ESP_LOGI(TAG, "SNTP_synchronized");
}

   

void app_main_SNTP()
{
    ++boot_count;
    ESP_LOGI(TAG, "Boot count: %d", boot_count);

    time_t now;
    struct tm timeinfo;
    time(&now);
    localtime_r(&now, &timeinfo);
    // Is time set? If not, tm_year will be (1970 - 1900).
 //   if (timeinfo.tm_year < (2016 - 1900)) {
        ESP_LOGI(TAG, "Time is not set yet. Connecting to WiFi and getting time over NTP.");
        obtain_time();
        // update 'now' variable with current time
        time(&now);
 //   }

    char strftime_buf[64];

    // Set timezone to Eastern Standard Time and print local time
    setenv("TZ", "CET-1CEST,M3.5.0,M10.5.0/3", 1);
    tzset();
    localtime_r(&now, &timeinfo);
    strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
    ESP_LOGI(TAG, "The current date/time in Hungary is: %s", strftime_buf);


    if (sntp_get_sync_mode() == SNTP_SYNC_MODE_SMOOTH) {
        struct timeval outdelta;
        while (sntp_get_sync_status() == SNTP_SYNC_STATUS_IN_PROGRESS) {
            adjtime(NULL, &outdelta);
            ESP_LOGI(TAG, "Waiting for adjusting time ... outdelta = %li sec: %li ms: %li us",
                        outdelta.tv_sec,
                        outdelta.tv_usec/1000,
                        outdelta.tv_usec%1000);
            vTaskDelay(2000 / portTICK_PERIOD_MS);
        }
    }


}

static void obtain_time(void)
{
    initialize_sntp();

    // wait for time to be set
    time_t now = 0;
    struct tm timeinfo = { 0 };
    int retry = 0;
    const int retry_count = 10;
    while (sntp_get_sync_status() == SNTP_SYNC_STATUS_RESET && ++retry < retry_count) {
        ESP_LOGI(TAG, "Waiting for system time to be set... (%d/%d)", retry, retry_count);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
    time(&now);
    localtime_r(&now, &timeinfo);

}

static void initialize_sntp(void)
{
    ESP_LOGI(TAG, "Initializing SNTP");
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, "pool.ntp.org");
    sntp_set_time_sync_notification_cb(time_sync_notification_cb);
#ifdef CONFIG_SNTP_TIME_SYNC_METHOD_SMOOTH
    sntp_set_sync_mode(SNTP_SYNC_MODE_SMOOTH);
#endif
    sntp_init();
}

u8g2_t u8g2; // a structure which will contain all the data for one display
void Write_Msg_toDisplay(int line, char *Msg)
{
	static char messages[4][32]={"","","",""};
	strcpy(messages[line],Msg);
int position1[4]={0,0,0,0};
	int position2[4]={15,30,45,60};
	int i;
	
	u8g2_ClearBuffer(&u8g2);
	for(i=0;i<4;i++)
	{
   // ESP_LOGI(TAG, "u8g2_ClearBuffer");
	
	//ESP_LOGI(TAG, "u8g2_DrawBox");
	//u8g2_DrawBox(&u8g2, 0, 26, 80,6);
	//u8g2_DrawFrame(&u8g2, 0,26,100,6);

  //ESP_LOGI(TAG, "u8g2_SetFont");
    u8g2_SetFont(&u8g2, u8g2_font_ncenB10_tr);
	//ESP_LOGI(TAG, "u8g2_DrawStr");
    u8g2_DrawStr(&u8g2, position1[i],position2[i],messages[i]);
//	ESP_LOGI(TAG, "u8g2_SendBuffer");
	}
	u8g2_SendBuffer(&u8g2);
}


void init_display()
{
	u8g2_esp32_hal_t u8g2_esp32_hal = U8G2_ESP32_HAL_DEFAULT;
	u8g2_esp32_hal.sda   = PIN_SDA;
	u8g2_esp32_hal.scl  = PIN_SCL;
	u8g2_esp32_hal.reset  = PIN_OLED_RESET;
	u8g2_esp32_hal_init(u8g2_esp32_hal);


	
	u8g2_Setup_ssd1306_i2c_128x64_noname_f(
		&u8g2,
		U8G2_R0,
		//u8x8_byte_sw_i2c,
		u8g2_esp32_i2c_byte_cb,
		u8g2_esp32_gpio_and_delay_cb);  // init u8g2 structure
	u8x8_SetI2CAddress(&u8g2.u8x8,0x78);

	ESP_LOGI(TAG, "u8g2_InitDisplay");
	u8g2_InitDisplay(&u8g2); // send init sequence to the display, display is in sleep mode after this,

	ESP_LOGI(TAG, "u8g2_SetPowerSave");
	u8g2_SetPowerSave(&u8g2, 0); // wake up display
	Write_Msg_toDisplay(0,"Initialized");
	
}


void report_scheduling(int ch)
{
 int period;
 char message[128];
 char topicname[64];	
 int msg_id;
 sprintf(topicname, "Watering/SWITCH%d/cheduled",ch+1); 
 for(period=0;period<10;period++)
 {
	if(Chedule_array[ch][period].off_time>Chedule_array[ch][period].on_time)
	{		
	sprintf(message,"P%d:%2.2d:%2.2d-%2.2d:%2.2d [%c%c%c%c%c%c%c] %dmin", 
	 period+1,
	 (int)(Chedule_array[ch][period].on_time/3600),
	 (int)(Chedule_array[ch][period].on_time%3600/60),
	 (int)(Chedule_array[ch][period].off_time/3600),
	 (int)(Chedule_array[ch][period].off_time%3600/60),
	 Chedule_array[ch][period].weekdays[0],
	 Chedule_array[ch][period].weekdays[1],
	 Chedule_array[ch][period].weekdays[2],
	 Chedule_array[ch][period].weekdays[3],
	 Chedule_array[ch][period].weekdays[4],
	 Chedule_array[ch][period].weekdays[5],
	 Chedule_array[ch][period].weekdays[6],
	 Chedule_array[ch][period].duration);
	 
	 if(mqtt_connected)
     {	
	  msg_id = esp_mqtt_client_publish(mqtt_client,  topicname, message, 0, 0, 0);   //Qos=0; retain=0
	  ESP_LOGI(TAG, "publish successful, msg_id=%d", msg_id);
	 }
    }
 }	 
	
}



int is_channel_active(int ch)
{
	switch (channel_states[ch])
	{
	 case MAN_ON:
	 case PROG_STARTED:
	 case MAN_RESUMED:
	 case PROG_RESUMED: return 1;
     default: return 0;	 
	}
}


void Publish_ontime(int ch)
{
	 int msg_id;
			 char topicname[64];
		     char message[16];		
			 sprintf(topicname, "Watering/SWITCH%d/on_time",ch);
			 ch--;	
			 
			 
			 if(!is_channel_active(ch))  sprintf(message,"%d",on_time[ch]);
			 else 
			 {
			  time_t now;
			  time(&now);
			  now-=1658700000;
			  now%=86400; 
			  sprintf(message,"%ld",on_time[ch]+now-turn_on_time[ch]);
			 }
			
	 if(mqtt_connected)
     {				
			 msg_id = esp_mqtt_client_publish(mqtt_client, topicname, message, 0, 0, 0);   //Qos=1; retain=0
			 ESP_LOGI(TAG, "publish successful, msg_id=%d", msg_id);	
	 }
}





void reboot_WIFI_STICK()
{
	ESP_LOGI(TAG, "reboot wifi stick");
	gpio_set_level(GPIO_OUTPUT_RELAY, 1);
	vTaskDelay(5*1000 / portTICK_PERIOD_MS);	
	gpio_set_level(GPIO_OUTPUT_RELAY, 0);
}





int update_item(char * ldata,char* item,int data_addr)
{
	char formatstring[128];
	int regValue;
	strcpy(formatstring,item);
	strcat(formatstring,"=%d");
	if(sscanf(ldata,formatstring,&regValue)==1)
			 {
				 eeprom_reg_t reg; 
				 xSemaphoreTake(mutex, portMAX_DELAY);
	     		  reg.frame.value = read_ACS71020_register(ACS71020_address_default, data_addr, 0xffffffff, 0,0);
				 xSemaphoreGive(mutex); 
				 
				 switch(data_addr)
				 {
					case 0x0B:  
								{
									eeprom_0x0B_t addr_0x0B;
									addr_0x0B.eeprom_data = reg.frame.fields.eeprom_data; 
									if (strcmp(item,"qvo_fine")==0) addr_0x0B.fields.qvo_fine=regValue;	
									else if (strcmp(item,"sns_fine")==0) addr_0x0B.fields.sns_fine=regValue;	
									else if (strcmp(item,"crs_sns")==0) addr_0x0B.fields.crs_sns=regValue;	
									else if (strcmp(item,"iavgselen")==0) addr_0x0B.fields.iavgselen=regValue;	
									reg.frame.fields.eeprom_data=addr_0x0B.eeprom_data;
								}
								break;
					case 0x0C:  
								{
									eeprom_0x0C_t addr_0x0C;
									addr_0x0C.eeprom_data = reg.frame.fields.eeprom_data; 
									if (strcmp(item,"rms_avg_1")==0) addr_0x0C.fields.rms_avg_1=regValue;	
									else if (strcmp(item,"rms_avg_2")==0) addr_0x0C.fields.rms_avg_2=regValue;	
									reg.frame.fields.eeprom_data=addr_0x0C.eeprom_data;
								}
								break;
					case 0x0D:  
								{
									eeprom_0x0D_t addr_0x0D;
									addr_0x0D.eeprom_data = reg.frame.fields.eeprom_data; 
									if (strcmp(item,"pacc_trim")==0) addr_0x0D.fields.pacc_trim=regValue;	
									else if (strcmp(item,"ichan_del_en")==0) addr_0x0D.fields.ichan_del_en=regValue;	
									else if (strcmp(item,"chan_del_sel")==0) addr_0x0D.fields.chan_del_sel=regValue;	
									else if (strcmp(item,"fault")==0) addr_0x0D.fields.fault=regValue;	
									else if (strcmp(item,"fltdly")==0) addr_0x0D.fields.fltdly=regValue;	
									else if (strcmp(item,"halfcycle_en")==0) addr_0x0D.fields.halfcycle_en=regValue;	
									else if (strcmp(item,"squarewave_en")==0) addr_0x0D.fields.squarewave_en=regValue;	
									reg.frame.fields.eeprom_data=addr_0x0D.eeprom_data;
								}
								break;
					case 0x0E:  
								{
									eeprom_0x0E_t addr_0x0E;
									addr_0x0E.eeprom_data = reg.frame.fields.eeprom_data; 
									if (strcmp(item,"delaycnt_sel")==0) addr_0x0E.fields.delaycnt_sel=regValue;	
									else if (strcmp(item,"undervreg")==0) addr_0x0E.fields.undervreg=regValue;	
									else if (strcmp(item,"overvreg")==0) addr_0x0E.fields.overvreg=regValue;	
									else if (strcmp(item,"vadc_rate_set")==0) addr_0x0E.fields.vadc_rate_set=regValue;	
									else if (strcmp(item,"vevent_cycs")==0) addr_0x0E.fields.vevent_cycs=regValue;	
									reg.frame.fields.eeprom_data=addr_0x0E.eeprom_data;
								}
								break;
					case 0x0F:  
								{
									eeprom_0x0F_t addr_0x0F;
									addr_0x0F.eeprom_data = reg.frame.fields.eeprom_data; 
									if (strcmp(item,"dio_1_sel")==0) addr_0x0F.fields.dio_1_sel=regValue;	
									else if (strcmp(item,"dio_0_sel")==0) addr_0x0F.fields.dio_0_sel=regValue;	
									else if (strcmp(item,"i2c_dis_slv_addr")==0) addr_0x0F.fields.i2c_dis_slv_addr=regValue;	
									else if (strcmp(item,"i2c_slv_addr")==0) addr_0x0F.fields.i2c_slv_addr=regValue;										
									reg.frame.fields.eeprom_data=addr_0x0F.eeprom_data;
								}
					default:	break;			
									
				 }
				 
				 xSemaphoreTake(mutex, portMAX_DELAY);
				  write_ACS71020(ACS71020_address_default, 0x2F, 0x4f70656E); //enter to customer mode
				  write_ACS71020(ACS71020_address_default, data_addr+0x10, reg.frame.fields.eeprom_data); //write shadow
				  write_ACS71020(ACS71020_address_default, data_addr, reg.frame.fields.eeprom_data); //write EEPROM	
				 xSemaphoreGive(mutex); 
				 esp_mqtt_client_publish(mqtt_client,  "Watering/ACS71020", "shadow + eeprom writen", 0, 0, 0);   //Qos=0; retain=0	 
				 return 1;
			 }
	else return 0;		 
}



static esp_err_t mqtt_event_handler(esp_mqtt_event_handle_t event)
{
    esp_mqtt_client_handle_t client = event->client;
    mqtt_client = event->client;
    int msg_id;
	int ch,period;
	char ltopic[128];
	char ldata[128];
	char str[32];
	char mark;
	    // your_context_t *context = event->context;
    switch (event->event_id) {
		case MQTT_EVENT_BEFORE_CONNECT:
			ESP_LOGI(TAG, "MQTT_EVENT_BEFORE_CONNECT");
		
		break;
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
             mqtt_connected = 1;
			 xEventGroupSetBits(s_wifi_event_group, MQTT_CONNECTED_BIT);
			 
			// Write_Msg_toDisplay(0,"MQTT connected.");
			 
		/*	 msg_id = esp_mqtt_client_subscribe(client, "Watering/#", 1); 
             ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);
		 */

             msg_id = esp_mqtt_client_subscribe(client, "Watering/FIRMWARE/URL", 1); 
             ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);
			 
			 msg_id = esp_mqtt_client_subscribe(client, "Watering/FIRMWARE/VERSION", 1); 
             ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);
			 
			 msg_id = esp_mqtt_client_subscribe(client, "Watering/FIRMWARE/ROLLBACK", 1); 
             ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);		
			 
			 if(mqtt_connected)
			 {		 
			  msg_id = esp_mqtt_client_publish(mqtt_client,  "Watering/FIRMWARE/RUNNING_VERSION", app_desc->version, 0, 0, 1);   //Qos=1; retain=1
			  ESP_LOGI(TAG, "publish successful, msg_id=%d", msg_id);
			  
			   msg_id = esp_mqtt_client_publish(mqtt_client,  "Watering/FIRMWARE/ROLLBACK", "", 0, 0, 0);   //Qos=1; retain=0
			   ESP_LOGI(TAG, "publish successful, msg_id=%d", msg_id);
			 }
			 
			 msg_id = esp_mqtt_client_subscribe(client, "Watering/SWITCH1/request", 1); 
             ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);
			 
			 msg_id = esp_mqtt_client_subscribe(client, "Watering/SWITCH2/request", 1); 
             ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);
			 
			 msg_id = esp_mqtt_client_subscribe(client, "Watering/SWITCH3/request", 1); 
             ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);
			 
			 msg_id = esp_mqtt_client_subscribe(client, "Watering/+/statistic", 1); 
             ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);
			 
			
			 
			 
	/* if(mqtt_connected)
     {	
			 msg_id = esp_mqtt_client_publish(mqtt_client,  "Watering/SWITCH1/status", "INIT", 0, 0, 1);   //Qos=1; retain=1
			 ESP_LOGI(TAG, "publish successful, msg_id=%d", msg_id);
			 
			 msg_id = esp_mqtt_client_publish(mqtt_client,  "Watering/SWITCH2/status", "INIT", 0, 0, 1);   //Qos=1; retain=1
			 ESP_LOGI(TAG, "publish successful, msg_id=%d", msg_id);
			 
			 msg_id = esp_mqtt_client_publish(mqtt_client,  "Watering/SWITCH3/status", "INIT", 0, 0, 1);   //Qos=1; retain=1
			 ESP_LOGI(TAG, "publish successful, msg_id=%d", msg_id);
	 }*/
			 
			 msg_id = esp_mqtt_client_subscribe(client, "Watering/+/schedule/#", 1); 
             ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);
			 
			 msg_id = esp_mqtt_client_subscribe(client, "Watering/restart", 1); 
             ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);
			 
			 msg_id = esp_mqtt_client_subscribe(client, "Watering/pump_restart_delay", 1); 
             ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);
			 
			 msg_id = esp_mqtt_client_subscribe(client, "Watering/measure_mode", 1); 
             ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);
			 
			 msg_id = esp_mqtt_client_subscribe(client, "Watering/LEVEL/?", 1); 
             ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);
			 
			 msg_id = esp_mqtt_client_subscribe(client, "Watering/TIME/?", 1); 
             ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);
			 			 
				 
			 msg_id = esp_mqtt_client_subscribe(client, "Watering/ACS71020/#", 1); 
             ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);
			 
			 msg_id = esp_mqtt_client_subscribe(client, "Watering/temp/?", 1); 
             ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);
			 
			 msg_id = esp_mqtt_client_subscribe(client, "Watering/help", 1); 
             ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);			 
						 
			  msg_id = esp_mqtt_client_subscribe(client, "Watering/life", 1); 
             ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
		//	Write_Msg_toDisplay(0,"MQTT disconnected.");
            mqtt_connected = 0;
			xEventGroupClearBits(s_wifi_event_group, MQTT_CONNECTED_BIT);
            break;

        case MQTT_EVENT_SUBSCRIBED:
            ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_UNSUBSCRIBED:
            ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_PUBLISHED:
            ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_DATA:
            ESP_LOGI(TAG, "MQTT_EVENT_DATA");
            printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
            printf("DATA=%.*s\r\n", event->data_len, event->data);
			strncpy(ltopic,event->topic,event->topic_len);
			ltopic[event->topic_len]=0;
			strncpy(ldata,event->data,event->data_len);
			ldata[event->data_len]=0;
			ESP_LOGI(TAG, "%s",ltopic);
			ESP_LOGI(TAG, "%s",ldata);

			
			if(strncmp(event->topic,"Watering/FIRMWARE/URL",strlen("Watering/FIRMWARE/URL"))==0) 
			{
			 ESP_LOGI(TAG, "FIRMWARE/URL");
			 if (event->data_len)  strcpy(url_buf,ldata);
			 if(mqtt_connected) esp_mqtt_client_publish(mqtt_client,  "Watering/FIRMWARE/URL/value", url_buf, 0, 0, 0);			 
			}	
			else if(strcmp(ltopic,"Watering/FIRMWARE/VERSION")==0) 
			{
				ESP_LOGI(TAG, "FIRMWARE/VERSION");
				if(strcmp(ldata,app_desc->version)!=0) 
				{
				  strcpy(new_Firmware_version,ldata);
				  ESP_LOGI(TAG, "NEW FIRMWARE Name:%s",new_Firmware_version);
				  if(mqtt_connected) esp_mqtt_client_publish(mqtt_client,  "Watering/FIRMWARE/UPDATE", "new version available", 0, 0, 0);   //Qos=0; retain=0	
				  if(mqtt_connected) esp_mqtt_client_publish(mqtt_client,  "Watering/FIRMWARE/UPDATE", new_Firmware_version, 0, 0, 0);   //Qos=0; retain=0	
				  FW_update_available=1;
				}
			}
			
		
				
			else if((sscanf(ltopic,"Watering/SWITCH%d/reques%c",&ch,&mark)==2) && (mark=='t'))
			{
				int i;
				ESP_LOGI(TAG, "SWITCHx/request");
				ch--;
				if((ch>=0) && (ch<3))
				{
				 for(i=0;i<sizeof(str_states);i++)
				 {
				  if(strcmp(ldata,str_states[i])==0) break;				  
				 }
				 if(i<sizeof(str_states)) manual_change_request[ch]=i; 
			     else manual_change_request[ch]=NOREQUEST;
				 if(manual_change_request[ch]==ENABLED) {channel_disabled[ch]=0;Save_data_to_NVS();}
				 else if (manual_change_request[ch]==DISABLED) {channel_disabled[ch]=1;Save_data_to_NVS();}
				 
				}					 
			}
			
			else if(strcmp(ltopic,"Watering/help")==0) 
			{
				char *message="-Watering/help show the available commands -Topic {Message}\n\
-Watering/FIRMWARE/URL {URL} -set new URL for OTA\n\
-Watering/FIRMWARE/VERSION  {version} -set new version for OTA\n\
-Watering/FIRMWARE/ROLLBACK {ROLLBACK:CANCEL_ROLLBACK} -keep or rollback OTA update\n\
-Watering/SWITCHx/request {MAN_ON,PROG_STARTED,MAN_RESUMED,PROG_RESUMED,INIT,ENABLED,DISABLED,MAN_OFF,SUSPENDED,DELAY,PROG_FINISHED,PROG_END,IDLE,REBOOTED,NOREQUEST} -set new state\n\
-Watering/SWITCHx/schedule/periodx {10:00-12:00 [+++++++] 30} -add new schedule period\n\
-Watering/SWITCHx/schedule/? {}   -list all programmed periods\n\
-Watering/SWITCHx/ontime {} -query ontime of switch in sec}\n\
-Watering/SWITCHx/statistic {} -get statistic}";						
		
		         if(mqtt_connected)
				 {	
				  esp_mqtt_client_publish(mqtt_client,  "Watering/MEASURE/commands", message, 0, 0, 0);   //Qos=0; retain=0	
				 }
				 vTaskDelay(3*1000 / portTICK_PERIOD_MS);	
message="-Watering/pump_restart_delay {10min} -set pump restart delay\n\
-Watering/LEVEL/? {} -query water level\n\
-Watering/TIME/? {} -query TIME\n\
-Watering/temp/? {} -query temperature sensor\n\
-Watering/measure_mode {POWER:LEVEL:STACK:CT:LOG:OFF}\n\
-Watering/restart {ESP:WIFI} -force restart of ESP32 or WIFI dongle\n\
-Watering/ACS71020/read {0xhex_address} \n\
-Watering/ACS71020/write {0xhex_address=0xhex_value}";	
				if(mqtt_connected)
				 {	
				  esp_mqtt_client_publish(mqtt_client,  "Watering/MEASURE/commands", message, 0, 0, 0);   //Qos=0; retain=0	
				 }			 
				
			}
			
			
			
			
			else if(strcmp(ltopic,"Watering/LEVEL/?")==0) 
			{
				char message[128];
				sprintf(message,"%0.1fcm off=%0.1fcm on=%0.1fcm fill=%ds sink=%ds ",water_level,protection_level_off,protection_level_on,(int)fill_time,(int)sink_time); 
		        if(mqtt_connected)
				{	
				 esp_mqtt_client_publish(mqtt_client,  "Watering/MEASURE/LEVEL", message, 0, 0, 0);   //Qos=0; retain=0	
				}
			}
			
			else if(strcmp(ltopic,"Watering/TIME/?")==0) 
			{
				char strftime_buf[64];
				time_t now;
				struct tm timeinfo;
				time(&now);
				localtime_r(&now, &timeinfo);
				strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
				if(SNTP_synchronized) strcat(strftime_buf," SYNC");
		        if(mqtt_connected)
				{	
				 esp_mqtt_client_publish(mqtt_client,  "Watering/TIME", strftime_buf, 0, 0, 0);   //Qos=0; retain=0	
				}
			}
			else if(strcmp(ltopic,"Watering/ACS71020/read")==0) 
			{
			  long val=0;	
			 char strval[32];
			 int reg,mask,shift_left,shift_right,fields;
			 fields=sscanf(ldata,"0x%x,0x%x,%d,%d",&reg,&mask,&shift_left,&shift_right);
			 switch (fields)
			 {
				
				 case 1: mask=0xffffffff;
				 case 2: shift_left=0; 
				 case 3: shift_right=0;
				 case 4: 
						xSemaphoreTake(mutex, portMAX_DELAY);
							val= read_ACS71020_register(ACS71020_address_default, reg, mask, shift_left,shift_right);
						xSemaphoreGive(mutex); 		
						 sprintf(strval,"%x:%lx",reg,val);
						 esp_mqtt_client_publish(mqtt_client,  "Watering/ACS71020", strval, 0, 0, 0);   //Qos=0; retain=0	
						 read_ACS71020_register2(reg,val);
				 break;
			 }
			 
			   
			 
			 
	
			}
			
			else if(strcmp(ltopic,"Watering/ACS71020/write")==0) 
			{	
		     int data_addr,regValue;
			 if(sscanf(ldata,"0x%x=0x%x",&data_addr,&regValue)==2)
			 {
				if ((data_addr>=0x1B) && (data_addr<=0x1F)) 
				{//write SHADOW
				    xSemaphoreTake(mutex, portMAX_DELAY);
					 write_ACS71020(ACS71020_address_default, 0x2F, 0x4f70656E); //enter to customer mode
					 write_ACS71020(ACS71020_address_default, data_addr, regValue); //write shadow
					 xSemaphoreGive(mutex); 
					esp_mqtt_client_publish(mqtt_client,  "Watering/ACS71020", "shadow writen", 0, 0, 0);   //Qos=0; retain=0	
				}	
				else if ((data_addr>=0x0B) && (data_addr<=0x0F)) 
				{//write SHADOW + EEPROM
					xSemaphoreTake(mutex, portMAX_DELAY);
					 write_ACS71020(ACS71020_address_default, 0x2F, 0x4f70656E); //enter to customer mode
					 write_ACS71020(ACS71020_address_default, data_addr+0x10, regValue); //write shadow
				     write_ACS71020(ACS71020_address_default, data_addr, regValue); //write EEPROM*/	
					xSemaphoreGive(mutex); 
					esp_mqtt_client_publish(mqtt_client,  "Watering/ACS71020", "shadow + eeprom writen", 0, 0, 0);   //Qos=0; retain=0	
				}					
			 }

/*
   qvo_fine
   sns_fine
   crs_sns
   iavgselen
   
   rms_avg_2
   rms_avg_1
   
   pacc_trim
   ichan_del_en
   chan_del_sel
   fault
   fltdly
   halfcycle_en
   squarewave_en
   
   delaycnt_sel
   undervreg
   overvreg
   vadc_rate_set
   vevent_cycs
   
   dio_1_sel
   dio_0_sel
   i2c_dis_slv_addr
   i2c_slv_addr
*/





	 else if(update_item(ldata,"qvo_fine",0x0B))	;	
	 else if(update_item(ldata,"sns_fine",0x0B))	;		
	 else if(update_item(ldata,"crs_sns",0x0B))	;	
	 else if(update_item(ldata,"iavgselen",0x0B))	;	

	 else if(update_item(ldata,"rms_avg_2",0x0C))	;	
	 else if(update_item(ldata,"rms_avg_1",0x0C))	;		 

else if(update_item(ldata,"pacc_trim",0x0D))	;	
else if(update_item(ldata,"ichan_del_en",0x0D))	;	
else if(update_item(ldata,"chan_del_sel",0x0D))	;	
else if(update_item(ldata,"fault",0x0D))	;	
else if(update_item(ldata,"fltdly",0x0D))	;	
else if(update_item(ldata,"halfcycle_en",0x0D))	;	
else if(update_item(ldata,"squarewave_en",0x0D))	;	
			 
			
else if(update_item(ldata,"delaycnt_sel",0x0E))	;	
else if(update_item(ldata,"undervreg",0x0E))	;	
else if(update_item(ldata,"overvreg",0x0E))	;	
else if(update_item(ldata,"vadc_rate_set",0x0E))	;	
else if(update_item(ldata,"vevent_cycs",0x0E))	;	
			
			
else if(update_item(ldata,"dio_1_sel",0x0F))	;		
else if(update_item(ldata,"dio_0_sel",0x0F))	;	
else if(update_item(ldata,"i2c_dis_slv_addr",0x0F))	;	
else if(update_item(ldata,"i2c_slv_addr",0x0F))	;		
			
			 
			 		
			}
			

			else if((sscanf(ltopic,"Watering/SWITCH%d/schedule/%c",&ch,&mark)==2) && (mark=='?'))
			{
			 ch--;
			 if((ch>=0) && (ch<3)) report_scheduling(ch);	
			}
			else if(sscanf(ltopic,"Watering/SWITCH%d/schedule/period%d",&ch,&period)==2)
			{
			 int HH_on,MM_on,HH_off,MM_off;
			 char weekdays[7];
			 int duration=0;
			 ch--;period--;
			 ESP_LOGI(TAG, "CH:%d,period:%d",ch,period);
			 if((ch>=0) && (ch<3) && (period>=0) && (period<10))
			 {
			  if(sscanf(ldata,"%2d:%2d-%2d:%2d [%c%c%c%c%c%c%c] %d",&HH_on,&MM_on,&HH_off,&MM_off,&weekdays[0],&weekdays[1],&weekdays[2],&weekdays[3],&weekdays[4],&weekdays[5],&weekdays[6],&duration)==12)
			  {
				Chedule_array[ch][period].on_time=HH_on*3600+MM_on*60;
				Chedule_array[ch][period].off_time=HH_off*3600+MM_off*60;
				strncpy(Chedule_array[ch][period].weekdays,weekdays,7);
				Chedule_array[ch][period].weekdays[7]=0;
				Chedule_array[ch][period].duration=duration;
				ESP_LOGI(TAG, "on_time:%d,off_time:%d",(int)Chedule_array[ch][period].on_time,(int)Chedule_array[ch][period].off_time);
				ESP_LOGI(TAG,"%s",weekdays);	
				if(Chedule_array[ch][period].on_time>=Chedule_array[ch][period].off_time)
				{
					Chedule_array[ch][period].on_time=0;
					Chedule_array[ch][period].off_time=0;
					Chedule_array[ch][period].weekdays[0]=0;
					Chedule_array[ch][period].duration=0;
				}					
			  }
			  else
			  { //delete invalid schedule 
					Chedule_array[ch][period].on_time=0;
					Chedule_array[ch][period].off_time=0;
					Chedule_array[ch][period].weekdays[0]=0;
					Chedule_array[ch][period].duration=0;
			  }
			  Save_data_to_NVS();
			 }
			 report_scheduling(ch);
			}
			else if((sscanf(ltopic,"Watering/SWITCH%d/%s",&ch,str)==2) && strcmp(str,"statistic")==0)
			{
			 Publish_ontime(ch);
			}
			else if(strcmp(ltopic,"Watering/restart")==0) 
			{
                if(strcmp(ldata,"ESP")==0) 	
				{
					prevhour=0;
					for(ch=0;ch<3;ch++) 
					{
						switch_channel(ch,DISABLED);
						vTaskDelay(1*1000 / portTICK_PERIOD_MS);
					}
					esp_restart();   
				}
				else if(strcmp(ldata,"WIFI")==0) 	 reboot_WIFI_STICK();					
			}
			else if(strcmp(ltopic,"Watering/pump_restart_delay")==0) 
			{
				int intval;
				 if(sscanf(ldata,"%d",&intval)==1) 
				 {
					 if ((intval>=0) && (intval<=60)) 
					 {
						 pump_restart_delay=intval;
						 Save_data_to_NVS();
					 }
				 }
			}
			else if(strcmp(ltopic,"Watering/measure_mode")==0) 
			{
				 if(strcmp(ldata,"LEVEL")==0)  measure_mode=1;
				 else if(strcmp(ldata,"POWER")==0)  measure_mode=2;
				 else if(strcmp(ldata,"CT")==0)  measure_mode=3;
				 else if(strcmp(ldata,"STACK")==0)  measure_mode=4;
				 else if(strcmp(ldata,"LOG")==0)  measure_mode=5;
				 else measure_mode=0;
			}
			else if(strcmp(ltopic,"Watering/FIRMWARE/ROLLBACK")==0)
			{
				 if(strcmp(ldata,"CANCEL_ROLLBACK")==0)  
				 {
					 esp_ota_mark_app_valid_cancel_rollback(); //validate the last OTA update
					if(mqtt_connected)
					{	
					 msg_id = esp_mqtt_client_publish(mqtt_client,  "Watering/FIRMWARE/ROLLBACK", "CANCELLED", 0, 0, 0);   //Qos=1; retain=1
					}
				 }
				 else if(strcmp(ldata,"ROLLBACK")==0) 
				 {
					if(mqtt_connected)
					{	
					  msg_id = esp_mqtt_client_publish(mqtt_client,  "Watering/FIRMWARE/ROLLBACK", "STARTED", 0, 0, 0);   //Qos=1; retain=1
					}
					 esp_ota_mark_app_invalid_rollback_and_reboot(); //rollback to previous FW
					 
				 }
			}
			else if(strcmp(ltopic,"Watering/life")==0) 
			{
				sscanf(ldata,"%d",&lastloopnow);
				
				//if(measure_mode==5)
				{
				 char message[16];
				 sprintf(message,"%d",lastloopnow);
				 esp_mqtt_client_publish(mqtt_client,  "Watering/life_loop", message, 0, 0, 0);   //Qos=0; retain=1
				}
			}
			else if(strcmp(ltopic,"Watering/temp/?")==0)
			{
				char msg[16];
				sprintf(msg,"%0.2f°C",temperature);
				msg_id = esp_mqtt_client_publish(mqtt_client,  "Watering/temp", msg, 0, 0, 0);   //Qos=1; retain=1
			}
			else 
			{
				esp_mqtt_client_publish(mqtt_client,  "Watering/command", "Unknown command", 0, 0, 0);   //Qos=1; retain=1
			}
			
            break;
        case MQTT_EVENT_ERROR:
            ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
            break;
        default:
            ESP_LOGI(TAG, "Other event id:%d", event->event_id);
            break;
    }
    return ESP_OK;
}




static void mqtt_app_start(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .uri = BROKER_URL,
        .event_handle = mqtt_event_handler,
        // .user_context = (void *)your_context
    };

    esp_mqtt_client_handle_t mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_start(mqtt_client);
}


int GPIO_OUTPUT_OUT_Array[2][3]={{GPIO_OUTPUT_OUT_1,GPIO_OUTPUT_OUT_1,GPIO_OUTPUT_OUT_1},{GPIO_OUTPUT_OUT_2,GPIO_OUTPUT_OUT_3,GPIO_OUTPUT_OUT_4}};

void switch_channel_relays(int channel, int status)
{
		if (GPIO_OUTPUT_OUT_Array[0][channel]) gpio_set_level(GPIO_OUTPUT_OUT_Array[0][channel], status);
		if (GPIO_OUTPUT_OUT_Array[1][channel]) gpio_set_level(GPIO_OUTPUT_OUT_Array[1][channel], status);	
}



void switch_channel(int ch, T_states status)
{
	int new_relay_status=0;
	
	char topicname[34];
	
	int msg_id;
	time_t now;
    time(&now);
			  now-=1658700000;
			  now%=86400; 
	
	if((status==MAN_ON) ||  (status==PROG_STARTED)  ||  (status==MAN_RESUMED) ||  (status==PROG_RESUMED))  new_relay_status=1; 
	
	if(channel_states[ch]!=status)
	{//statuschange	
	 sprintf(topicname,"Watering/SWITCH%1d/status",ch+1);
	 if(channel_states[ch]==DISABLED)
	 {
		new_relay_status=0;
		if((status!=ENABLED) && (status!=DISABLED)) return;
	 }		 
	 else if(((channel_states[ch]==SUSPENDED) || (channel_states[ch]==DELAY)) && ((status==MAN_ON) || (status==PROG_STARTED)))
	 {
	 	channel_states_before_suspend[ch]=status; //new turn ON request will be handled after resume
		if(mqtt_connected)
				{	
			     char message[64]; 
			     sprintf(message,"%s->%s",str_states[channel_states[ch]],str_states[channel_states_before_suspend[ch]]);
				 msg_id = esp_mqtt_client_publish(mqtt_client,topicname,message , 0, 0, 1);
				 ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
				}
		return;		
	 }

    if (new_relay_status && pump_suspended) 
	{
		if(mqtt_connected)
				{	
				 msg_id = esp_mqtt_client_publish(mqtt_client,topicname,"REFUSED_ON"  , 0, 0, 1);
				 ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
				}
		return; //not allowed to switch pump on 
	}

	 
	 if(new_relay_status) {if (!is_channel_active(ch)) {last_pump_on_time=now;turn_on_time[ch]=now;}} // from OFF to ON
	else if (is_channel_active(ch)) if (turn_on_time[ch]!=-1) {on_time[ch]+=now-turn_on_time[ch];turn_on_time[ch]=-1;} //from ON to OFF
	
	channel_states[ch]=status;		   
    switch_channel_relays(ch,new_relay_status);
				
				if(mqtt_connected)
				{	
			     if((channel_states[ch]==SUSPENDED) || (channel_states[ch]==DELAY))
				 {
				  char message[64]; 
			      sprintf(message,"%s->%s",str_states[channel_states[ch]],str_states[channel_states_before_suspend[ch]]);
				  msg_id = esp_mqtt_client_publish(mqtt_client,topicname,message , 0, 0, 1);
				 }
		         else
				 {			 
			    	 msg_id = esp_mqtt_client_publish(mqtt_client,topicname,str_states[status]  , 0, 0, 1);
				     ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
				 }
				}
	}			
}


static void smartconfig_example_task(void * parm);

static void event_handler(void* arg, esp_event_base_t event_base, 
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
	//	Write_Msg_toDisplay(0,"Start Wifi connection.");
		esp_wifi_connect();
        
/*	} else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_CONNECTED) {	
	   Write_Msg_toDisplay(0,"Wifi connected");
	*/	
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
	//	Write_Msg_toDisplay(0,"Wifi disconnected");
		ESP_LOGI(TAG, "WIFI_EVENT_STA_DISCONNECTED");
        if(!smartconfig_started)
		{
			xEventGroupClearBits(s_wifi_event_group, CONNECTED_BIT);
			if (mqtt_connected) 
			{
				esp_mqtt_client_stop(mqtt_client);
				mqtt_connected=0;
			}
			sntp_stop();
			ESP_LOGI(TAG, "WIFI reconnecting");
			wifi_retry_count++;
			ESP_LOGI(TAG, "wifi_retry_count:%d",wifi_retry_count);
			vTaskDelay(10*1000 / portTICK_PERIOD_MS);	
			if(wifi_retry_count==100)
			{
			 wifi_retry_count=0;
			 reboot_WIFI_STICK();
			 vTaskDelay(10*1000 / portTICK_PERIOD_MS);	
			}				
			else if(wifi_retry_count==150) 
			{
				wifi_retry_count=0;
				esp_restart();
			}
			esp_wifi_connect();
		}
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        
		app_main_SNTP();
		mqtt_app_start();
		wifi_retry_count=0;
		xEventGroupSetBits(s_wifi_event_group, CONNECTED_BIT);
		
    } else if (event_base == SC_EVENT && event_id == SC_EVENT_SCAN_DONE) {
        ESP_LOGI(TAG, "Scan done");
		Write_Msg_toDisplay(0,"Scan done");
    } else if (event_base == SC_EVENT && event_id == SC_EVENT_FOUND_CHANNEL) {
        ESP_LOGI(TAG, "Found channel");
		Write_Msg_toDisplay(0,"Found channel");
    } else if (event_base == SC_EVENT && event_id == SC_EVENT_GOT_SSID_PSWD) {
        ESP_LOGI(TAG, "Got SSID and password");
		Write_Msg_toDisplay(0,"Got SSID and password:");
        smartconfig_event_got_ssid_pswd_t *evt = (smartconfig_event_got_ssid_pswd_t *)event_data;
        wifi_config_t wifi_config;
        uint8_t ssid[33] = { 0 };
        uint8_t password[65] = { 0 };

        bzero(&wifi_config, sizeof(wifi_config_t));
        memcpy(wifi_config.sta.ssid, evt->ssid, sizeof(wifi_config.sta.ssid));
        memcpy(wifi_config.sta.password, evt->password, sizeof(wifi_config.sta.password));
        wifi_config.sta.bssid_set = evt->bssid_set;
        if (wifi_config.sta.bssid_set == true) {
            memcpy(wifi_config.sta.bssid, evt->bssid, sizeof(wifi_config.sta.bssid));
        }

        memcpy(ssid, evt->ssid, sizeof(evt->ssid));
        memcpy(password, evt->password, sizeof(evt->password));
        ESP_LOGI(TAG, "SSID:%s",(char*)ssid);
        ESP_LOGI(TAG, "PASSWORD:%s",(char*)password);
		Write_Msg_toDisplay(1,(char*)ssid);
		Write_Msg_toDisplay(2,(char*)password);
		vTaskDelay(3*1000 / portTICK_PERIOD_MS);
        ESP_ERROR_CHECK( esp_wifi_disconnect() );
        ESP_ERROR_CHECK( esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
		ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_FLASH)); 
        ESP_ERROR_CHECK( esp_wifi_connect() );
    } else if (event_base == SC_EVENT && event_id == SC_EVENT_SEND_ACK_DONE) {
        xEventGroupSetBits(s_wifi_event_group, ESPTOUCH_DONE_BIT);
    }
}

static void initialise_wifi(void)
{
	ESP_LOGI(TAG, "initialise_wifi");
	EventBits_t uxBits;
    const TickType_t xTicksToWait = 300000 / portTICK_PERIOD_MS;
    tcpip_adapter_init();
    s_wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );

    ESP_ERROR_CHECK( esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL) );
    ESP_ERROR_CHECK( esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL) );
    ESP_ERROR_CHECK( esp_event_handler_register(SC_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL) );

    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK( esp_wifi_start() );
	
	uxBits = xEventGroupWaitBits(s_wifi_event_group, CONNECTED_BIT, false, false, xTicksToWait); 
    if(uxBits & CONNECTED_BIT) {
            ESP_LOGI(TAG, "WiFi Connected to ap");
    }
	else{
		//timeout
		 ESP_LOGI(TAG, "WiFi timeout");
		 reboot_WIFI_STICK();
		 ESP_LOGI(TAG, "ESP restart");
		 esp_restart();
	}
	
}

static void smartconfig_example_task(void * parm)
{
    EventBits_t uxBits;
	Write_Msg_toDisplay(0,"Use ESPTouch on IOS to");
	Write_Msg_toDisplay(1,"change WIFI SSID and password.");
    ESP_ERROR_CHECK( esp_smartconfig_set_type(SC_TYPE_ESPTOUCH) );
    smartconfig_start_config_t cfg = SMARTCONFIG_START_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_smartconfig_start(&cfg) );
    while (1) {
        uxBits = xEventGroupWaitBits(s_wifi_event_group, CONNECTED_BIT | ESPTOUCH_DONE_BIT, true, false, portMAX_DELAY); 
        if(uxBits & CONNECTED_BIT) {
            ESP_LOGI(TAG, "WiFi Connected to ap");
        }
        if(uxBits & ESPTOUCH_DONE_BIT) {
            ESP_LOGI(TAG, "smartconfig over");
            esp_smartconfig_stop();
			smartconfig_started=0;
            vTaskDelete(NULL);
        }
    }
	
	
	
}


void read_ACS71020_register2(int reg_addr,long value)
{
    char strval[128];
	eeprom_reg_t reg;
    reg.address = reg_addr;
    reg.frame.value =  value;
    eeprom_0x0B_t addr_0x0B;
    eeprom_0x0C_t addr_0x0C;   
    eeprom_0x0D_t addr_0x0D;
    eeprom_0x0E_t addr_0x0E;
    eeprom_0x0F_t addr_0x0F;
   
   acs_0x20_t addr_0x20;
   acs_0x21_t addr_0x21;
   acs_0x22_t addr_0x22;
   acs_0x23_t addr_0x23;
   acs_0x24_t addr_0x24;
   acs_0x25_t addr_0x25;
   acs_0x26_t addr_0x26;
   acs_0x27_t addr_0x27;
   acs_0x28_t addr_0x28;
   acs_0x29_t addr_0x29;
   acs_0x2A_t addr_0x2A;
   acs_0x2B_t addr_0x2B;
   acs_0x2C_t addr_0x2C;
   acs_0x2D_t addr_0x2D;
  // acs_0x2E_t addr_0x2E;
   acs_0x2F_t addr_0x2F;
   acs_0x30_t addr_0x30;
   
  
   
   
   acs_reg_t req_volatile;
   req_volatile.address = reg_addr;
   req_volatile.register_value =  value;
    
    switch (reg_addr)
	{
	 case 0x0B:
	 case 0x1B:
	
     addr_0x0B.eeprom_data = reg.frame.fields.eeprom_data; 
	 sprintf(strval,"qvo_fine: %d", addr_0x0B.fields.qvo_fine);
	 esp_mqtt_client_publish(mqtt_client,  "Watering/ACS71020", strval, 0, 0, 0);   //Qos=0; retain=0	 
	 sprintf(strval,"sns_fine: %d", addr_0x0B.fields.sns_fine);
	 esp_mqtt_client_publish(mqtt_client,  "Watering/ACS71020", strval, 0, 0, 0);   //Qos=0; retain=0	 
	 sprintf(strval,"crs_sns: %d", addr_0x0B.fields.crs_sns);
	 esp_mqtt_client_publish(mqtt_client,  "Watering/ACS71020", strval, 0, 0, 0);   //Qos=0; retain=0	 
	 sprintf(strval,"iavgselen: %d", addr_0x0B.fields.iavgselen);
	 esp_mqtt_client_publish(mqtt_client,  "Watering/ACS71020", strval, 0, 0, 0);   //Qos=0; retain=0	 
	 break;
	 
	 case 0x0C:
	 case 0x1C:
	 
     addr_0x0C.eeprom_data = reg.frame.fields.eeprom_data; 
	 sprintf(strval,"rms_avg_2: %d", addr_0x0C.fields.rms_avg_2);
	 esp_mqtt_client_publish(mqtt_client,  "Watering/ACS71020", strval, 0, 0, 0);   //Qos=0; retain=0	
	 sprintf(strval,"rms_avg_1: %d", addr_0x0C.fields.rms_avg_1);
	 esp_mqtt_client_publish(mqtt_client,  "Watering/ACS71020", strval, 0, 0, 0);   //Qos=0; retain=0	 	 
	 break;
	 
	 case 0x0D:
	 case 0x1D:
	
     addr_0x0D.eeprom_data = reg.frame.fields.eeprom_data;
     sprintf(strval,"pacc_trim: %d", addr_0x0D.fields.pacc_trim);
	 esp_mqtt_client_publish(mqtt_client,  "Watering/ACS71020", strval, 0, 0, 0);   //Qos=0; retain=0	 
     sprintf(strval,"ichan_del_en: %d", addr_0x0D.fields.ichan_del_en);
	 esp_mqtt_client_publish(mqtt_client,  "Watering/ACS71020", strval, 0, 0, 0);   //Qos=0; retain=0	
     sprintf(strval,"chan_del_sel: %d", addr_0x0D.fields.chan_del_sel);
	 esp_mqtt_client_publish(mqtt_client,  "Watering/ACS71020", strval, 0, 0, 0);   //Qos=0; retain=0	
     sprintf(strval,"fault: %d", addr_0x0D.fields.fault);
	 esp_mqtt_client_publish(mqtt_client,  "Watering/ACS71020", strval, 0, 0, 0);   //Qos=0; retain=0	
     sprintf(strval,"fltdly: %d", addr_0x0D.fields.fltdly);
	 esp_mqtt_client_publish(mqtt_client,  "Watering/ACS71020", strval, 0, 0, 0);   //Qos=0; retain=0	
     sprintf(strval,"halfcycle_en: %d", addr_0x0D.fields.halfcycle_en);
	 esp_mqtt_client_publish(mqtt_client,  "Watering/ACS71020", strval, 0, 0, 0);   //Qos=0; retain=0	
     sprintf(strval,"squarewave_en: %d", addr_0x0D.fields.squarewave_en);
	 esp_mqtt_client_publish(mqtt_client,  "Watering/ACS71020", strval, 0, 0, 0);   //Qos=0; retain=0	
     break;
	 case 0x0E:
	 case 0x1E:
	 
     addr_0x0E.eeprom_data = reg.frame.fields.eeprom_data; 
	 sprintf(strval,"delaycnt_sel: %d", addr_0x0E.fields.delaycnt_sel);
	 esp_mqtt_client_publish(mqtt_client,  "Watering/ACS71020", strval, 0, 0, 0);   //Qos=0; retain=0	
	 sprintf(strval,"undervreg: %d", addr_0x0E.fields.undervreg);
	 esp_mqtt_client_publish(mqtt_client,  "Watering/ACS71020", strval, 0, 0, 0);   //Qos=0; retain=0	
	 sprintf(strval,"overvreg: %d", addr_0x0E.fields.overvreg);
	 esp_mqtt_client_publish(mqtt_client,  "Watering/ACS71020", strval, 0, 0, 0);   //Qos=0; retain=0	
	 sprintf(strval,"vadc_rate_set: %d", addr_0x0E.fields.vadc_rate_set);
	 esp_mqtt_client_publish(mqtt_client,  "Watering/ACS71020", strval, 0, 0, 0);   //Qos=0; retain=0	
	 sprintf(strval,"vevent_cycs: %d", addr_0x0E.fields.vevent_cycs);
	 esp_mqtt_client_publish(mqtt_client,  "Watering/ACS71020", strval, 0, 0, 0);   //Qos=0; retain=0	
	 break; 
	 case 0x0F:
	 case 0x1F:
	
     addr_0x0F.eeprom_data = reg.frame.fields.eeprom_data; 
	 sprintf(strval,"dio_1_sel: %d", addr_0x0F.fields.dio_1_sel);
	 esp_mqtt_client_publish(mqtt_client,  "Watering/ACS71020", strval, 0, 0, 0);   //Qos=0; retain=0	
	 sprintf(strval,"dio_0_sel: %d", addr_0x0F.fields.dio_0_sel);
	 esp_mqtt_client_publish(mqtt_client,  "Watering/ACS71020", strval, 0, 0, 0);   //Qos=0; retain=0	
	 sprintf(strval,"i2c_dis_slv_addr: %d", addr_0x0F.fields.i2c_dis_slv_addr);
	 esp_mqtt_client_publish(mqtt_client,  "Watering/ACS71020", strval, 0, 0, 0);   //Qos=0; retain=0	
	  sprintf(strval,"i2c_slv_addr: %d", addr_0x0F.fields.i2c_slv_addr);
	 esp_mqtt_client_publish(mqtt_client,  "Watering/ACS71020", strval, 0, 0, 0);   //Qos=0; retain=0	
	 break;
	 
	 case 0x20:
     addr_0x20.register_value = req_volatile.register_value; 
	 sprintf(strval,"irms: %d, %2.2fA", addr_0x20.fields.irms,addr_0x20.fields.irms*30/pow(2,14));
	 esp_mqtt_client_publish(mqtt_client,  "Watering/ACS71020", strval, 0, 0, 0);   //Qos=0; retain=0	
	  sprintf(strval,"vrms: %d, %2.2fV", addr_0x20.fields.vrms,addr_0x20.fields.vrms*550/pow(2,15));
	 esp_mqtt_client_publish(mqtt_client,  "Watering/ACS71020", strval, 0, 0, 0);   //Qos=0; retain=0	
	 break;
	 case 0x21:
	 addr_0x21.register_value = req_volatile.register_value; 
	 sprintf(strval,"pactive: %d, %2.2fW", addr_0x21.fields.pactive,addr_0x21.fields.pactive*30*0.275*(R1_4+Rs)/Rs/pow(2,15));
	 esp_mqtt_client_publish(mqtt_client,  "Watering/ACS71020", strval, 0, 0, 0);   //Qos=0; retain=0	
	 break;
	 case 0x22:
	 addr_0x22.register_value = req_volatile.register_value; 
	 sprintf(strval,"papparent: %d, %2.2fVA", addr_0x22.fields.papparent,addr_0x22.fields.papparent*30*0.275*(R1_4+Rs)/Rs/pow(2,15));
	 esp_mqtt_client_publish(mqtt_client,  "Watering/ACS71020", strval, 0, 0, 0);   //Qos=0; retain=0	 
	 break;
	  case 0x23:
	 addr_0x23.register_value = req_volatile.register_value; 
	 sprintf(strval,"pimag: %d, %2.2fVA", addr_0x23.fields.pimag,addr_0x23.fields.pimag*30*0.275*(R1_4+Rs)/Rs/pow(2,15));
	 esp_mqtt_client_publish(mqtt_client,  "Watering/ACS71020", strval, 0, 0, 0);   //Qos=0; retain=0	 
	 break;
	  case 0x24:
	 addr_0x24.register_value = req_volatile.register_value; 
	 sprintf(strval,"pfactor: %d, %2.2f", addr_0x24.fields.pfactor,addr_0x24.fields.pfactor/pow(2,9));
	 esp_mqtt_client_publish(mqtt_client,  "Watering/ACS71020", strval, 0, 0, 0);   //Qos=0; retain=0	 
	 break;
	  case 0x25:
	 addr_0x25.register_value = req_volatile.register_value; 
	 sprintf(strval,"numptsout: %d", addr_0x25.fields.numptsout);
	 esp_mqtt_client_publish(mqtt_client,  "Watering/ACS71020", strval, 0, 0, 0);   //Qos=0; retain=0	 
	 break;
	   case 0x26:
	 addr_0x26.register_value = req_volatile.register_value; 
	 sprintf(strval,"irmsavgonesec: %d, %2.2fA", addr_0x26.fields.irmsavgonesec,addr_0x26.fields.irmsavgonesec*30/pow(2,14));
	 esp_mqtt_client_publish(mqtt_client,  "Watering/ACS71020", strval, 0, 0, 0);   //Qos=0; retain=0	 
	  sprintf(strval,"vrmsavgonesec: %d, %2.2fV", addr_0x26.fields.vrmsavgonesec,addr_0x26.fields.vrmsavgonesec*0.275*(R1_4+Rs)/Rs/pow(2,15));
	 esp_mqtt_client_publish(mqtt_client,  "Watering/ACS71020", strval, 0, 0, 0);   //Qos=0; retain=0	 
	 break;
	    case 0x27:
	 addr_0x27.register_value = req_volatile.register_value; 
	 sprintf(strval,"irmsavgonemin: %d, %2.2fA", addr_0x27.fields.irmsavgonemin,addr_0x27.fields.irmsavgonemin*30/pow(2,14));
	 esp_mqtt_client_publish(mqtt_client,  "Watering/ACS71020", strval, 0, 0, 0);   //Qos=0; retain=0	
	  sprintf(strval,"vrmsavgonemin: %d, %2.2fV", addr_0x27.fields.vrmsavgonemin,addr_0x27.fields.vrmsavgonemin*0.275*(R1_4+Rs)/Rs/pow(2,15));
	 esp_mqtt_client_publish(mqtt_client,  "Watering/ACS71020", strval, 0, 0, 0);   //Qos=0; retain=0	
	 break;
	     case 0x28:
	 addr_0x28.register_value = req_volatile.register_value; 
	 sprintf(strval,"pactavgonesec: %d, %2.2fW", addr_0x28.fields.pactavgonesec,addr_0x28.fields.pactavgonesec*30*0.275*(R1_4+Rs)/Rs/pow(2,15));
	 esp_mqtt_client_publish(mqtt_client,  "Watering/ACS71020", strval, 0, 0, 0);   //Qos=0; retain=0	
	 break;
	      case 0x29:
	 addr_0x29.register_value = req_volatile.register_value; 
	 sprintf(strval,"pactavgonemin: %d, %2.2fW", addr_0x29.fields.pactavgonemin,addr_0x29.fields.pactavgonemin*30*0.275*(R1_4+Rs)/Rs/pow(2,15));
	 esp_mqtt_client_publish(mqtt_client,  "Watering/ACS71020", strval, 0, 0, 0);   //Qos=0; retain=0	
	 break;
	 case 0x2A:
	 addr_0x2A.register_value = req_volatile.register_value; 
	 sprintf(strval,"vcodes: %d, %2.2FV", addr_0x2A.fields.vcodes,addr_0x2A.fields.vcodes*0.275*(R1_4+Rs)/Rs/pow(2,16));
	 esp_mqtt_client_publish(mqtt_client,  "Watering/ACS71020", strval, 0, 0, 0);   //Qos=0; retain=0	
	 break;
	 case 0x2B:
	 addr_0x2B.register_value = req_volatile.register_value; 
	 sprintf(strval,"icodes: %d, %2.2FA", addr_0x2B.fields.icodes,addr_0x2B.fields.icodes*30/pow(2,15));
	 esp_mqtt_client_publish(mqtt_client,  "Watering/ACS71020", strval, 0, 0, 0);   //Qos=0; retain=0	 
	 break;
	 case 0x2C:
	 addr_0x2C.register_value = req_volatile.register_value; 
	 sprintf(strval,"pinstant: %d, %2.2fW", addr_0x2C.fields.pinstant,addr_0x2C.fields.pinstant*30*0.275*(R1_4+Rs)/Rs/pow(2,29));
	 esp_mqtt_client_publish(mqtt_client,  "Watering/ACS71020", strval, 0, 0, 0);   //Qos=0; retain=0	 
	 break;
	  case 0x2D:
	 addr_0x2D.register_value = req_volatile.register_value; 
	 sprintf(strval,"vzerocrossout: %d", addr_0x2D.fields.vzerocrossout);
	 esp_mqtt_client_publish(mqtt_client,  "Watering/ACS71020", strval, 0, 0, 0);   //Qos=0; retain=0	 
	 sprintf(strval,"faultout: %d", addr_0x2D.fields.faultout);
	 esp_mqtt_client_publish(mqtt_client,  "Watering/ACS71020", strval, 0, 0, 0);   //Qos=0; retain=0	 
	 sprintf(strval,"faultlatched: %d", addr_0x2D.fields.faultlatched);
	 esp_mqtt_client_publish(mqtt_client,  "Watering/ACS71020", strval, 0, 0, 0);   //Qos=0; retain=0	 
	 sprintf(strval,"overvoltage: %d", addr_0x2D.fields.overvoltage);
	 esp_mqtt_client_publish(mqtt_client,  "Watering/ACS71020", strval, 0, 0, 0);   //Qos=0; retain=0	 
	 sprintf(strval,"undervoltage: %d", addr_0x2D.fields.undervoltage);
	 esp_mqtt_client_publish(mqtt_client,  "Watering/ACS71020", strval, 0, 0, 0);   //Qos=0; retain=0	 
	 sprintf(strval,"posangle: %d", addr_0x2D.fields.posangle);
	 esp_mqtt_client_publish(mqtt_client,  "Watering/ACS71020", strval, 0, 0, 0);   //Qos=0; retain=0	 
	 sprintf(strval,"pospf: %d", addr_0x2D.fields.pospf);
	 esp_mqtt_client_publish(mqtt_client,  "Watering/ACS71020", strval, 0, 0, 0);   //Qos=0; retain=0	 
	 break;
	  case 0x2F:
	 addr_0x2F.register_value = req_volatile.register_value; 
	 sprintf(strval,"access_code: %d", addr_0x2F.fields.access_code);
	 esp_mqtt_client_publish(mqtt_client,  "Watering/ACS71020", strval, 0, 0, 0);   //Qos=0; retain=0	 
	 break;
	   case 0x30:
	 addr_0x30.register_value = req_volatile.register_value; 
	 sprintf(strval,"customer_access: %d", addr_0x30.fields.customer_access);
	 esp_mqtt_client_publish(mqtt_client,  "Watering/ACS71020", strval, 0, 0, 0);   //Qos=0; retain=0	 
	 break;
	 
	default: break;
	}
}


void mainTask(void *pvParameters){
  ds18b20_init(DS_PIN);
 
  int msg_id;
  int ch,i;
  time_t time_at_start;
  time_t runtime=0;
  
  time(&time_at_start);
  int      TimeToPublish = 600000000; //in uS
  
  xEventGroupWaitBits(s_wifi_event_group, MQTT_CONNECTED_BIT, false, false, 30*1000 / portTICK_PERIOD_MS); 
  

   for(ch=0;ch<3;ch++) 
   {
	if(channel_disabled[ch]) switch_channel(ch,DISABLED);
	else  switch_channel(ch,REBOOTED);
   }

 uint64_t TimePastPublish = esp_timer_get_time();

  while (1) {
	  if(measure_mode==4)
	  { char message[64];
	    int unused_stack=  uxTaskGetStackHighWaterMark(NULL); 
		if(mqtt_connected)
     {	
 	  wifi_ap_record_t ap;
	  esp_wifi_sta_get_ap_info(&ap);
	  sprintf(message,"unused_stack:%dbytes, runtime:%lds, RSSSI:%d",unused_stack, runtime, ap.rssi);
	  esp_mqtt_client_publish(mqtt_client,  "Watering/MEASURE/STACK", message, 0, 0, 0);   //Qos=0; retain=0
  
	 }
		
	  }
 	int isolated_inp1=gpio_get_level(ISOLATED_INPUT_1);
	int isolated_inp2=gpio_get_level(ISOLATED_INPUT_2);

	
	if(gpio_get_level(PRG_BUTTON)==0) 
	{
		vTaskDelay(3*1000 / portTICK_PERIOD_MS);
		if(gpio_get_level(PRG_BUTTON)==0) 
		{
		 ESP_LOGI(TAG, "button long pressed");
		 if(!smartconfig_started) 
		 {
			ESP_ERROR_CHECK( esp_wifi_disconnect() );
			vTaskDelay(1*1000 / portTICK_PERIOD_MS);
		  xTaskCreate(smartconfig_example_task, "smartconfig_example_task", 4096, NULL, 3, NULL);
		  smartconfig_started=1;
		 }
		 
		 while(smartconfig_started)
		 {
			vTaskDelay(1*1000 / portTICK_PERIOD_MS);   
		 } 
		} 	
	}
	
		
    for(ch=0;ch<3;ch++) //manual switches
	{		
     if(mqtt_connected && (manual_change_request[ch]!=NOREQUEST))
	 {
		switch_channel(ch,manual_change_request[ch]);
		manual_change_request[ch]=NOREQUEST;
	 }
	}	
	     
	
	if((FW_update_available) && strlen(url_buf))
	{
	 if(mqtt_connected) esp_mqtt_client_publish(mqtt_client,  "Watering/FIRMWARE/UPDATE", "started", 0, 0, 0);   //Qos=0; retain=0	
//	 if(mqtt_connected)  esp_mqtt_client_stop(mqtt_client); 
	 
	 
	 for(ch=0;ch<3;ch++) switch_channel(ch,DISABLED);
	 
	 Write_Msg_toDisplay(0,"OTA update started.");	
     xTaskCreate(&ota_update_task, "ota_update_task", 8192, NULL, 5, NULL);	
		
	 while(FW_update_available) ;		
	}
	
	  vTaskDelay(1*1000 / portTICK_PERIOD_MS);
	 
	
	 {
	 int ch;
	 char timeStr[32];
	 int dayofweek;
	 time_t now,now2;
	 time(&now);
	 now2=now;
	 runtime=now-time_at_start;
	 
		 
	 now-=1658700000;
	 static int prev_dayofweek=0;
	 
	
	dayofweek=(int)(now/86400)%7+1;
	if(prev_dayofweek!=dayofweek) 
	{
		for(ch=0;ch<=3;ch++) on_time[ch]=0; //erase ontime at daychange
		pump_protection_started_at=0;
		if(((prev_dayofweek>0) && ((now2-lastloopnow)>1800))) // reboot at every midnight if mqtt broken more tha 1/2 hour
		{
			reboot_WIFI_STICK();
			esp_restart();
		}
	}
		
	
	prev_dayofweek=dayofweek;
	
	now%=86400; 
	
	int hour=(int)now/3600;
	if(prevhour!=hour)
	{
	 char message[32];
	 Publish_ontime(1);
	 Publish_ontime(2);
	 sprintf(message,"%0.1fcm",water_level);
	 if(mqtt_connected)
     {	
	  esp_mqtt_client_publish(mqtt_client,  "Watering/MEASURE/LEVEL", message, 0, 0, 0);   //Qos=0; retain=0
	 }
	 prevhour=hour;
	}
	
	
    temperature=ds18b20_get_temp();
   
	if(SNTP_synchronized)
	{	
	 char* days[7]={"M","Tu","W","Th","F","Sa","Su"};	
     sprintf(timeStr,"%2.2d:%2.2d:%2.2d %s %0.2f°C",(int)((now)/3600),(int)(((now)%3600)/60),(int)((now)%60),days[dayofweek-1],temperature); 
	 Write_Msg_toDisplay(1,timeStr);	
	
	if ( (esp_timer_get_time() - TimePastPublish) >= TimeToPublish )
    {
       if(mqtt_connected) 
	   {	
			  char message[64];             
			  time_t now1;
			  time(&now1);
			  sprintf(message,"%ld:%s",now1,timeStr);   
			  esp_mqtt_client_publish(mqtt_client,  "Watering/life", message, 0, 0, 1);   //Qos=0; retain=1
	   }
      TimePastPublish = esp_timer_get_time(); // get next publish time
    }
	
	
	
	for(ch=0;ch<3;ch++)
	{
	 if (channel_disabled[ch]) continue; 	
	for(i=0;i<10;i++)
	{
		if(Chedule_array[ch][i].on_time<Chedule_array[ch][i].off_time)
		{	
			//ESP_LOGI(TAG,"ch=%d,period=%d,day=%d,time:%d,start%d,stop%d,day:%c",ch,i,dayofweek,(int) now,(int)Chedule_array[ch][i].on_time,(int)Chedule_array[ch][i].off_time,Chedule_array[ch][i].weekdays[dayofweek]);
		if((Chedule_array[ch][i].on_time<now) && (Chedule_array[ch][i].off_time>now) && ((Chedule_array[ch][i].weekdays[dayofweek-1]=='+') || (Chedule_array[ch][i].weekdays[dayofweek-1]=='x') || (Chedule_array[ch][i].weekdays[dayofweek-1]=='X')|| (Chedule_array[ch][i].weekdays[dayofweek-1]=='1')))
		{
			ESP_LOGI(TAG,"times:%d<%d<%d",(int)Chedule_array[ch][i].on_time,(int)now,(int)Chedule_array[ch][i].off_time);
			
			
			if(now-Chedule_array[ch][i].on_time<30) 
			{
				ESP_LOGI(TAG,"ON EVENT");
			  
				if(channel_states[ch]!=PROG_STARTED) 
				{
					prev_daily_ontime[ch]=on_time[ch];
					switch_channel(ch,PROG_STARTED);
				}
				
			}
			else if(Chedule_array[ch][i].off_time-now<30) 
			{
				if(channel_states[ch]!=PROG_END) 
				{	ESP_LOGI(TAG,"OFF EVENT");
					switch_channel(ch,PROG_END);
					if(measure_mode==5)
				{
					if(mqtt_connected)
					{
					char message[128];
					sprintf(message,"%d,%d,%ld,%ld,%ld",on_time[ch],prev_daily_ontime[ch],now,turn_on_time[ch],(on_time[ch]-prev_daily_ontime[ch]+ now-turn_on_time[ch])/60);				 
					msg_id = esp_mqtt_client_publish(mqtt_client,  "Watering/LOG", message, 0, 0, 0);   //Qos=0; retain=0
					}
	  
				}
				}
				
			}
			else if(is_channel_active(ch))
			{
			  if(measure_mode==5)
				{
					if(mqtt_connected)
					{
					char message[128];
					sprintf(message,"%d,%d,%ld,%ld,%ld",on_time[ch],prev_daily_ontime[ch],now,turn_on_time[ch],(on_time[ch]-prev_daily_ontime[ch]+ now-turn_on_time[ch])/60);				 
					msg_id = esp_mqtt_client_publish(mqtt_client,  "Watering/LOG", message, 0, 0, 0);   //Qos=0; retain=0
					}
	  
				}
     		 if (Chedule_array[ch][i].duration<=(on_time[ch]-prev_daily_ontime[ch]+ now-turn_on_time[ch])/60) 
		     {
				 ESP_LOGI(TAG,"OFF EVENT DUE TO DURATION");
				 switch_channel(ch,PROG_FINISHED);
				 if(measure_mode==5)
				{
					if(mqtt_connected)
					{
					char message[128];
					sprintf(message,"%d,%d,%ld,%ld,%ld",on_time[ch],prev_daily_ontime[ch],now,turn_on_time[ch],(on_time[ch]-prev_daily_ontime[ch]+ now-turn_on_time[ch])/60);				 
					msg_id = esp_mqtt_client_publish(mqtt_client,  "Watering/LOG", message, 0, 0, 0);   //Qos=0; retain=0
					}
				}
			
			 }
			}
		}	
		}		
	}
	}
	}
	else Write_Msg_toDisplay(1,"wait for SNTP sync.");

	if(isolated_inp1)
	{
		 if	(!pump_suspended)
		 {		
		  ESP_LOGI(TAG,"PUMP SUSPENDED");  
		  pump_protection_started_at=now;
		  protection_level_off=water_level;
		  sink_time=now-last_pump_on_time;
		 }
		 pump_suspended=1;	 
	     for(ch=0;ch<3;ch++) 
		  { 
	          if(channel_states[ch]!=SUSPENDED) 
			  {// all channels forced to suspended state
				  channel_states_before_suspend[ch]=channel_states[ch];
				  switch_channel(ch,SUSPENDED);} 	 
		      }
		 		 
		 Write_Msg_toDisplay(2,"pump suspended");
	}
	else
	{
	 if	(pump_suspended)
	 {
		if((now-pump_protection_started_at)/60>=pump_restart_delay)
	 	 {
			ESP_LOGI(TAG,"PUMP_RESUMED");
			pump_suspended=0;
			Write_Msg_toDisplay(2,"pump resumed");
			for(ch=0;ch<3;ch++)
			{ 
			 switch(channel_states[ch])
			 {
				case SUSPENDED:  
								  protection_level_on=water_level;
								  fill_time=now-pump_protection_started_at;
				case DELAY: 
			                 switch (channel_states_before_suspend[ch])
							 {
								case MAN_ON: switch_channel(ch,MAN_RESUMED); break;
								case PROG_STARTED: switch_channel(ch,PROG_RESUMED); break;
								default:switch_channel(ch,channel_states_before_suspend[ch]);
							 }	
							 break;
				default:	break;
			 } 
			}	 
		 }
		 else {
			 char message[32];
			 sprintf(message,"waiting:%lds",(int)pump_restart_delay*60+pump_protection_started_at-now); 
			 ESP_LOGI(TAG,"WAITING FOR RESTART DELAY");
			 Write_Msg_toDisplay(2,message);
			 for(ch=0;ch<3;ch++)
			 { 
			  if(channel_states[ch]==SUSPENDED) 
			   {
					 protection_level_on=water_level;
					 fill_time=now-pump_protection_started_at;				   
				     switch_channel(ch,DELAY);
				}
			 } 
		 }
	 }	
	// else Write_Msg_toDisplay(2,"");
   }
	 }
	 
	 {
	
		 char message[32];
		 sprintf(message,"%s %s %s",str_states[channel_states[0]],str_states[channel_states[1]],str_states[channel_states[2]]);
		 Write_Msg_toDisplay(0,message);
		  
	 }		

if(1)
{
char message[128];
xSemaphoreTake(mutex, portMAX_DELAY);
		
/*	
double i=    MeasuredValue(ACS71020_address_default, 0x2B, 0x0001ffff,15, 0,15,30.0);
// ESP_LOGI(TAG, "i= %lf", i);
double u=    MeasuredValue(ACS71020_address_default, 0x2A, 0x0001ffff,15, 0,16,0.275*(R1_4+Rs)/Rs; 
//ESP_LOGI(TAG, "u= %lf", u);
*/
double irms= MeasuredValue(ACS71020_address_default, 0x20, 0x7fff0000, 0,16,14,30.0);
//ESP_LOGI(TAG, "irms= %lf", irms);
double urms= MeasuredValue(ACS71020_address_default, 0x20, 0x00007fff, 0, 0,15,0.275*(R1_4+Rs)/Rs); 

/*
double urms_1s= MeasuredValue(ACS71020_address_default, 0x26, 0x00007fff, 0, 0,15,0.275*(R1_4+Rs)/Rs; 

double urms_1m= MeasuredValue(ACS71020_address_default, 0x27, 0x00007fff, 0, 0,15,0.275*(R1_4+Rs)/Rs; 


//ESP_LOGI(TAG, "urms= %lf", urms);
*/
double num=  MeasuredValue(ACS71020_address_default, 0x25, 0x000001ff, 0, 0,0,1); 
//ESP_LOGI(TAG, "num= %lf", num);


double p=    MeasuredValue(ACS71020_address_default, 0x28, 0x0001ffff,15, 0,15,30.0*0.275*(R1_4+Rs)/Rs);
//ESP_LOGI(TAG, "p= %lf", p);


xSemaphoreGive(mutex);

 sprintf(message,"%0.1lfV %0.1lfA %0.1lfW",urms,irms,p);
 Write_Msg_toDisplay(3,message);
 
 if(measure_mode==2)
 {
  sprintf(message,"Urms=%0.1lfV  Irms=%0.2lfA  P=%0.1lfW num=%0.0lf consumption:%lfkWh\n",urms,irms,p,num,powerconsumptionWs/3600/1000);
  	 if(mqtt_connected)
     {	
		msg_id = esp_mqtt_client_publish(mqtt_client,  "Watering/MEASURE/POWER", message, 0, 0, 0);   //Qos=0; retain=0
	    ESP_LOGI(TAG, "publish successful, msg_id=%d", msg_id);	
	 }
 }


}

if(1) //measure water level
{
	char message[128];
 
 
        uint32_t adc_reading = 0;
        //Multisampling
        for (int i = 0; i < NO_OF_SAMPLES; i++) {
            if (unit == ADC_UNIT_1) {
                adc_reading += adc1_get_raw((adc1_channel_t)channel);
            } else {
                int raw;
                adc2_get_raw((adc2_channel_t)channel, ADC_WIDTH_BIT_12, &raw);
                adc_reading += raw;
            }
        }
        adc_reading /= NO_OF_SAMPLES;
        //Convert adc_reading to voltage in mV
        uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
		//ESP_LOGI(TAG, "Raw: %d\tVoltage: %dmV\n", adc_reading, voltage);
		water_level=((voltage-142)/44.13*3/2);
		sprintf(message,"%2.1fcm",water_level);		
		Write_Msg_toDisplay(2,message);
		
		if(measure_mode==1)
		{
		 //sprintf(message,"Raw: %d,Voltage: %dmV level:%2.1fcm", adc_reading, voltage, (voltage-142)/44.13*3/2);
		 sprintf(message,"Level:%2.1fcm Temp:%0.2f°C", (voltage-142)/44.13*3/2, temperature);
		 if(mqtt_connected)
		 {	
		  msg_id = esp_mqtt_client_publish(mqtt_client,  "Watering/MEASURE/LEVEL", message, 0, 0, 0);   //Qos=0; retain=0
	     }
		}
		if(measure_mode==3)
		{
		uint32_t current_CT[100];
		float avg_current_CT=0;
		float rms_current_CT=0;
		int zerocross=0;
		int first_zerocross=0;
		int last_zerocross=0;
		for(i=0;i<100;i++)
		{	
		 current_CT[i] = esp_adc_cal_raw_to_voltage(adc1_get_raw((adc1_channel_t)ADC_CHANNEL_1), adc_chars)*10;
		 avg_current_CT+=current_CT[i];
		 vTaskDelay(0.1 / portTICK_PERIOD_MS);
		}
		avg_current_CT/=100;
		for(i=0;i<100;i++)
		{
			if((i<99) && (current_CT[i]<=avg_current_CT) && (current_CT[i+1]>avg_current_CT)) {zerocross++; if(zerocross==1) first_zerocross=i;}
			else if ((i<99) && (current_CT[i]>avg_current_CT) && (current_CT[i+1]<=avg_current_CT)) {zerocross++; last_zerocross=i;}
		}
		for(i=first_zerocross;i<last_zerocross;i++)
		{
			rms_current_CT+= pow(current_CT[i]-avg_current_CT,2);
		}
				
		rms_current_CT=sqrt(rms_current_CT/(last_zerocross-first_zerocross));
		
        if(mqtt_connected)
		 {
		  sprintf(message,"CT_rms=%2.1fA first:%d,last:%d,zerocross=%d, offset=%f",rms_current_CT,first_zerocross,last_zerocross,zerocross,avg_current_CT);				 
		  msg_id = esp_mqtt_client_publish(mqtt_client,  "Watering/MEASURE/CT", message, 0, 0, 0);   //Qos=0; retain=0
	     }
		
		}
		
		
		
		// ESP_LOGI(TAG, "CT voltage: %dmV\n", current_CT);
}	 
  
  
  
  
 
  
  
  
  
  } //while(1)
}

extern const uint8_t server_cert_pem_start[] asm("_binary_ca_cert_pem_start");
extern const uint8_t server_cert_pem_end[] asm("_binary_ca_cert_pem_end");

esp_err_t _http_event_handler_OTA(esp_http_client_event_t *evt)
{
    switch (evt->event_id) {
    case HTTP_EVENT_ERROR:
        ESP_LOGD(TAG, "HTTP_EVENT_ERROR");
        break;
    case HTTP_EVENT_ON_CONNECTED:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_CONNECTED");
        break;
    case HTTP_EVENT_HEADER_SENT:
        ESP_LOGD(TAG, "HTTP_EVENT_HEADER_SENT");
        break;
    case HTTP_EVENT_ON_HEADER:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_HEADER, key=%s, value=%s", evt->header_key, evt->header_value);
        break;
    case HTTP_EVENT_ON_DATA:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
        break;
    case HTTP_EVENT_ON_FINISH:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_FINISH");
        break;
    case HTTP_EVENT_DISCONNECTED:
        ESP_LOGD(TAG, "HTTP_EVENT_DISCONNECTED");
        break;
    }
    return ESP_OK;
}

esp_err_t my_esp_https_ota(const esp_http_client_config_t *config)
{
	char message[128];
	esp_app_desc_t new_app_info;
	
    if (!config) {
        ESP_LOGE(TAG, "esp_http_client config not found");
        return ESP_ERR_INVALID_ARG;
    }    

    esp_https_ota_config_t ota_config = {
        .http_config = config,
    };

    esp_https_ota_handle_t https_ota_handle = NULL;
    esp_err_t err = esp_https_ota_begin(&ota_config, &https_ota_handle);
    if (https_ota_handle == NULL) {
        return ESP_FAIL;
    }
	
	
    esp_https_ota_get_img_desc(https_ota_handle, &new_app_info);
	
	 ESP_LOGE(TAG,"FW project:Running:%s, received:%s",app_desc->project_name,new_app_info.project_name); 
	 if(strcmp(app_desc->project_name,new_app_info.project_name)!=0)
	 {
		ESP_LOGE(TAG, "wrong FIRMWARE project. Running:%s, received:%s",app_desc->project_name,new_app_info.project_name); 
		esp_restart(); 
	 }
	
	
	 ESP_LOGE(TAG, "FIRMARE version. Running:%s, Expected:%s, received:%s",app_desc->version,new_Firmware_version,new_app_info.version); 
	  sprintf(message,"Running:%s, Expected:%s, received:%s",app_desc->version,new_Firmware_version,new_app_info.version); 
	  esp_mqtt_client_publish(mqtt_client,  "Watering/FIRMWARE/RUNNING_VERSION", message, 0, 0, 1);   //Qos=1; retain=1
	 if(strcmp(new_Firmware_version,new_app_info.version)!=0) 
	 {
		 ESP_LOGE(TAG, "wrong FIRMWARE version. Running:%s, Expected:%s, received:%s",app_desc->version,new_Firmware_version,new_app_info.version); 
		 esp_restart();
	 }
 

    while (1) {
        err = esp_https_ota_perform(https_ota_handle);
        if (err != ESP_ERR_HTTPS_OTA_IN_PROGRESS) {
            break;
        }
    }

    esp_err_t ota_finish_err = esp_https_ota_finish(https_ota_handle);
    if (err != ESP_OK) {
        /* If there was an error in esp_https_ota_perform(),
           then it is given more precedence than error in esp_https_ota_finish()
         */
        return err;
    } else if (ota_finish_err != ESP_OK) {
        return ota_finish_err;
    }
    return ESP_OK;
}




void ota_update_task(void *pvParameter)
{	
 ESP_LOGI(TAG, "Starting OTA example");
int ptr=strlen(url_buf)-1;
while(url_buf[ptr]!='n') url_buf[ptr--]=0;

 ESP_LOGI(TAG, "OTA URL:%s",url_buf);
//ESP_LOGI(TAG, "OTA CERT:%s",server_cert_pem_start);

    esp_http_client_config_t config = {
		.url = url_buf,
	   .cert_pem = (char *)server_cert_pem_start,
        .event_handler = _http_event_handler_OTA,
    };
#ifdef CONFIG_EXAMPLE_SKIP_COMMON_NAME_CHECK
    config.skip_cert_common_name_check = true;
#endif


 //   esp_err_t ret = esp_https_ota(&config);
 esp_err_t ret = my_esp_https_ota(&config);
    if (ret == ESP_OK) {
        esp_restart();
    } else {
        ESP_LOGE(TAG, "Firmware upgrade failed");
    }
   
    FW_update_available=0;
}


void Load_data_from_NVS()
{
    nvs_handle_t nvs_handle;
	 int i,ch;
	 size_t buf_len;
	 int ret;
	 int intval;
    if((ret=nvs_open("Watering", NVS_READWRITE, &nvs_handle))!=ESP_OK) ESP_LOGI(TAG, "NVS open failed. %d",ret);
	
	if(nvs_get_i32(nvs_handle, "pump_delay",&intval)==ESP_OK) pump_restart_delay=intval;
	
	ESP_LOGI(TAG, "pump_restart_delay:%dmin",pump_restart_delay);
	
	for (ch=0;ch<3;ch++)
    {
	  char keyName[32];   //CHx_x 
	  sprintf(keyName,"CH%1.1d_DISABLED",ch);
	  ESP_LOGI(TAG, "KEY:%s",keyName);
	  nvs_get_i32(nvs_handle, keyName, &channel_disabled[ch]);	
		
	 
     for(i=0;i<10;i++)
     {
	  sprintf(keyName,"CH%1.1d_ON_%1.1d",ch,i);
	  ESP_LOGI(TAG, "KEY:%s",keyName);
	  nvs_get_i32(nvs_handle, keyName, &Chedule_array[ch][i].on_time);
	  ESP_LOGI(TAG, "VALUE:%d",(int)Chedule_array[ch][i].on_time);
	  sprintf(keyName,"CH%1.1d_OFF_%1.1d",ch,i);
	  ESP_LOGI(TAG, "KEY:%s",keyName);
	  nvs_get_i32(nvs_handle, keyName, &Chedule_array[ch][i].off_time);
	  ESP_LOGI(TAG, "VALUE:%d",(int)Chedule_array[ch][i].off_time);
	  sprintf(keyName,"CH%1.1d_DAY_%1.1d",ch,i);
	  ESP_LOGI(TAG, "KEY:%s",keyName);
	  buf_len = sizeof(Chedule_array[ch][i].weekdays);
	  nvs_get_str(nvs_handle, keyName, Chedule_array[ch][i].weekdays,&buf_len);
      ESP_LOGI(TAG, "VALUE:%s",Chedule_array[ch][i].weekdays);
		
	  sprintf(keyName,"CH%1.1d_DUR_%1.1d",ch,i);
	  ESP_LOGI(TAG, "KEY:%s",keyName);
	  nvs_get_i32(nvs_handle, keyName, &Chedule_array[ch][i].duration);
	  ESP_LOGI(TAG, "VALUE:%d",(int)Chedule_array[ch][i].duration);	
    } 
   }
	
    nvs_close(nvs_handle);
}


void Save_data_to_NVS()
{
    nvs_handle_t  nvs_handle;
	 int i,ch;
    if(nvs_open("Watering", NVS_READWRITE, &nvs_handle)!=ESP_OK) ESP_LOGI(TAG, "NVS OPEN FAILED");
		
   // TEST_ESP_OK(nvs_erase_all(nvs_handle));
   
    nvs_set_i32(nvs_handle, "pump_delay",pump_restart_delay);
   for (ch=0;ch<3;ch++)
   {
	 char keyName[32];   //CHx_x 
	 sprintf(keyName,"CH%1.1d_DISABLED",ch);
	 nvs_set_i32(nvs_handle, keyName, channel_disabled[ch]);   
    for(i=0;i<10;i++)
    {
 	
	 sprintf(keyName,"CH%1.1d_ON_%1.1d",ch,i);
	 nvs_set_i32(nvs_handle, keyName, Chedule_array[ch][i].on_time);
	 sprintf(keyName,"CH%1.1d_OFF_%1.1d",ch,i);
	 nvs_set_i32(nvs_handle, keyName, Chedule_array[ch][i].off_time);
	 sprintf(keyName,"CH%1.1d_DAY_%1.1d",ch,i);
	 Chedule_array[ch][i].weekdays[7]=0;
	 nvs_set_str(nvs_handle, keyName, Chedule_array[ch][i].weekdays);
	 sprintf(keyName,"CH%1.1d_DUR_%1.1d",ch,i);
	 nvs_set_i32(nvs_handle, keyName, Chedule_array[ch][i].duration); 
    } 
   }
      
   if(nvs_commit(nvs_handle)!=ESP_OK) ESP_LOGI(TAG, "NVS COMMIT FAILED");; 

    nvs_close(nvs_handle);
	ESP_LOGI(TAG, "NVS STORED");
}



void delete_all_chedules()
{
 memset(Chedule_array,0,sizeof(Chedule_array));
 Save_data_to_NVS();
}


uint8_t buf[32];

void task_rx(void *p)
{
   int x;
   for(;;) {
      lora_receive();    // put into receive mode
      while(lora_received()) {
         x = lora_receive_packet(buf, sizeof(buf));
         buf[x] = 0;
         printf("Received: %s\n", buf);
		 ESP_LOGI(TAG, "Received: %s\n", buf);
         lora_receive();
      }
      vTaskDelay(1);
   }
}

 



void PowerMeterTask(void *pvParameters){
    
	TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = (60*1000 / portTICK_PERIOD_MS);

    while(1){

        vTaskDelayUntil( &xLastWakeTime, xFrequency);
		xSemaphoreTake(mutex, portMAX_DELAY);
		 powerconsumptionWs+=    MeasuredValue(ACS71020_address_default, 0x28, 0x0001ffff,15, 0,15,30.0*0.275*(R1_4+Rs)/Rs)*60.0;
		xSemaphoreGive(mutex); 
  
    }
}


void app_main()
{
	app_desc = esp_ota_get_app_description();
	ESP_LOGI(TAG, "[APP] Startup..");
    ESP_LOGI(TAG, "[APP] Free memory: %d bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());
    ESP_LOGI(TAG, "Project name:     %s", app_desc->project_name);
    ESP_LOGI(TAG, "App version:      %s", app_desc->version);



    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("MQTT_CLIENT", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT_TCP", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT_SSL", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT", ESP_LOG_VERBOSE);
    esp_log_level_set("OUTBOX", ESP_LOG_VERBOSE);
    // Initialize NVS.
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // 1.OTA app partition table has a smaller NVS partition size than the non-OTA
        // partition table. This size mismatch may cause NVS initialization to fail.
        // 2.NVS partition contains data in new format and cannot be recognized by this version of code.
        // If this happens, we erase NVS partition and initialize NVS again.
	
        ESP_ERROR_CHECK(nvs_flash_erase());
		ESP_LOGI(TAG, "nvs_flash_erased");
        err = nvs_flash_init();
    }
	
   if (err!=ESP_OK) ESP_LOGI(TAG, "NVS INIT ERROR %d",err);
	ESP_ERROR_CHECK(err);   
	
//	delete_all_chedules();
	Load_data_from_NVS();
	
	gpio_set_level(GPIO_OUTPUT_RELAY, 0);

  init_display();
  
  

  
	 gpio_config_t io_conf;
    //disable interrupt
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);
	
	
	
	//interrupt of rising edge
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //bit mask of the pins, use GPIO4/5 here
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    //set as input mode    
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
	
	gpio_set_direction(PRG_BUTTON ,GPIO_MODE_INPUT);
	gpio_set_pull_mode(PRG_BUTTON,GPIO_PULLUP_ONLY);
	
	   //Check if Two Point or Vref are burned into eFuse
    check_efuse();

    //Configure ADC
    if (unit == ADC_UNIT_1) {
        adc1_config_width(ADC_WIDTH_BIT_12);
        adc1_config_channel_atten(channel, atten);
    } else {
        adc2_config_channel_atten((adc2_channel_t)channel, atten);
    }

    //Characterize ADC
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
    print_char_val_type(val_type);


	Write_Msg_toDisplay(0,"connect to WIFI...");
		
	initialise_wifi();
	
	
	mutex = xSemaphoreCreateMutex();

    xTaskCreatePinnedToCore(&mainTask, "mainTask", 4096, NULL, 5, NULL, 0);
	xTaskCreatePinnedToCore(&PowerMeterTask, "PowerMeterTask", 1024, NULL, 5, NULL, 0);
	
//	readEeprom(ACS71020_address_default);
//    readShadow(ACS71020_address_default);
	
 /*  lora_init();
   ESP_LOGI(TAG, "Done");
    ESP_LOGI(TAG, "lora_set_frequency");
   lora_set_frequency(433e6);
    ESP_LOGI(TAG, "Done");
   lora_enable_crc();
   xTaskCreate(&task_rx, "task_rx", 2048, NULL, 5, NULL);*/
	
	Write_Msg_toDisplay(3,"Done!");
}



