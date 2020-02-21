// ###########################################################################
// #                                                                         #
// #                                L E D - C A M                            #
// #                                                                         #
// ###########################################################################
// # Versions:                                                               #
// # 20200116: variables for FTP can now be set with WiFi-Manager            #
// #           source cleared                                                #
// # 20200117: MQTT added                                                    #
// # 20200119: MQTT commands added                                           #
// # 20200120: infomation about boards added,                                #
// #           source cleared                                                #
// # 20200121: FTP part in setup added                                       #
// # 20200124: reading of base mac address at setup added                    #
// # 20200126: MQTT commands <filename_datetime> and <info> added            #
// # 20200127: added restart, if no connection to MQTT broker is possible    #
// # 20200204: added restart, if no connection to WiFi is possible           # 
// #           param,eters for WM are only set, if WM will be used for setup #
// #                                                                         #
// ###########################################################################

// ESP_F1BF713C: Board_Node32s_WMOS_LOLIN32
// ESP_3A334FC4: Board_AI_Thinker_ESP32-Cam
 
#include <esp_wifi.h>
#include <esp_system.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <EEPROM.h>   // include library to read and write from flash memory
#include <PubSubClient.h>

// SSID and PW for Config Portal
#define ESP_getChipId()   ((uint32_t)ESP.getEfuseMac())
String ssid = "ESP_" + String(ESP_getChipId(), HEX);
const char* password = "JoE_rad800";

// SSID and PW for your Router
String Router_SSID;
String Router_Pass;

#include <ESP_WiFiManager.h>     //https://github.com/khoih-prog/ESP_WiFiManager
#include "esp_camera.h"
#include "lwip/inet.h"
#include "lwip/dns.h"

#define DEBUG
#define DEBUG_FTP false    // true or false
#define DEBUG_MTQQ false    // true or false
#define DEBUG_CAM false   // true or false

#define CODEVERSION "Web-Mini-Cam-FTP - 20200204"
#define AUTHOR "Dipl.Ing. Jochen Egbers"
#define NOTE "using ideas of Tzapu, Ken Taylor, Khoi Hoang, Lordcyber and others"

// FTP related
char cam_location[16]="Dortmund";                  // will be overwritten by Wifi-Manager usage
char cam_name[16]="LED-Cam";                       // will be overwritten by Wifi-Manager usage
bool cam_filename_datetime = true;                 // true: like 20200113_123818; false: as cam_location
char ftp_server_url[32] = "jegbers.dyndns.info";   // will be overwritten by Wifi-Manager usage
char ftp_pfad[64] = "/ExtHDD/Dortmund";            // will be overwritten by Wifi-Manager usage
char ftp_user[16] = "";                            // will be overwritten by Wifi-Manager usage
char ftp_pw[16] =  "";                             // will be overwritten by Wifi-Manager usage
String dateiname;

#define FTP_ANZAHL 4
String ftp_comm[] = {"USER "+ String(ftp_user), "PASS "+String(ftp_pw), "SYST","Type I"};
String ftp_ret[] = {"331","230","215","200"};
//IPAddress ftpserver( 192,168,0,99 );
IPAddress ftpserver;
uint16_t ftpport = 21;

// MQTT related
char mqtt_server_url[32] = "test.mosquitto.org";          // will be overwritten by Wifi-Manager usage
IPAddress mqttserver;
int  mqtt_port = 1883;
char mqtt_user[16] = "";
char mqtt_pw[16] = "";
char mqtt_message[90] = "";
char mqtt_message_save[90] = "";
char mqtt_command[24] = "NOP";
char mqtt_data[66] = "0";
char mqtt_topic[64] = "JoE_IoT/Cam";
char mqtt_pub_topic[64]="";
char mqtt_pub_info[90]="";

// EEPROM related
const int eeAddress_waiting_time = 0;                                              //Start-Location we want the data to be get from
const int eeAddress_flood = 4;
const int eeAddress_cam_location = 5;
const int eeAddress_cam_name = 21;
const int eeAddress_cam_filename_datetime = 37;
const int eeAddress_ftp_server_url = 38;
const int eeAddress_ftp_pfad = 70;
const int eeAddress_ftp_user = 134;
const int eeAddress_ftp_pw = 150;
const int eeAddress_mqtt_server_url = 166;
const int eeAddress_mqtt_topic = 198;

// LEDs
const int LED_1 = 33;    // small led1 on back of the ESP32-CAM
int floodlight = 4;      // the on-board LED flash
bool flood_use = true;   // true: use floodlight, false: don't use floodlight, 
                         // will be overwritten by Wifi-Manager usage

// NTP related
char picturePath[50];    //50 chars should be enough
char ntpServer[] = "pool.ntp.org";
const long  gmtOffset_sec = 3600;
const int   daylightOffset_sec = 3600;

void receiveLocalTime()
{
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    Serial.println("Failed to obtain time");
    return;
  }
  strftime(picturePath, sizeof(picturePath), "%Y%m%d_%H%M%S", &timeinfo);
}

// DeepSleep Timer
#define uS_TO_S_FACTOR 1000000    /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  1          /* Time ESP32 will go to sleep (in seconds) */
int delay_s = 10; // 10s division of waiting time

int Waiting_Time;
unsigned long currentMillis = 0;
unsigned long previousMillis = 0;

RTC_DATA_ATTR int bootCount = 0;

void print_wakeup_reason(void);

#define WAKE_TIMER   1
#define WAKE_PIR     2

uint16_t wake_reason = 0;                   // Grund für das Aufwecken
  
WiFiClient ftpclient;
WiFiClient dclient;
WiFiClient mqttclient;

String uploadFTP(String filename);
String ftpReceive(void);
bool ftpCommand(uint8_t index);

PubSubClient MQTTClient(mqttclient);

// Camera related
// PIR-Cam ist das Modul mit PIR und OLED, M5Cam ist die kleine M5Stack-Cam, diese mit ESP32 Devkit ohne PSRAM compilieren.
//#define PIR_CAM                           // für Modul mit PIR
//#define M5_CAM                            // für M5Stack-Cam
#define LED_CAM                             // für AI-Thinker Modul mit LEDC_CHANNEL_0

#define BUTTON  34                        // L aktiv 
#define PIR_OUT 33                        // H aktiv

#if defined (PIR_CAM)

  #define PWDN_GPIO_NUM 26
  #define RESET_GPIO_NUM -1
  #define XCLK_GPIO_NUM 32
  #define SIOD_GPIO_NUM 13
  #define SIOC_GPIO_NUM 12

  #define Y9_GPIO_NUM 39
  #define Y8_GPIO_NUM 36
  #define Y7_GPIO_NUM 23
  #define Y6_GPIO_NUM 18
  #define Y5_GPIO_NUM 15
  #define Y4_GPIO_NUM 4
  #define Y3_GPIO_NUM 14
  #define Y2_GPIO_NUM 5
  #define VSYNC_GPIO_NUM 27
  #define HREF_GPIO_NUM 25
  #define PCLK_GPIO_NUM 19

  #define XCLK_FREQUENZ  20000000
  #define RESOLUTION  FRAMESIZE_SXGA
  #define BILDNAME "PIR_CAM_"

#elif defined(LED_CAM)

  #define PWDN_GPIO_NUM     32
  #define RESET_GPIO_NUM    -1
  #define XCLK_GPIO_NUM      0
  #define SIOD_GPIO_NUM     26
  #define SIOC_GPIO_NUM     27

  #define Y9_GPIO_NUM       35
  #define Y8_GPIO_NUM       34
  #define Y7_GPIO_NUM       39
  #define Y6_GPIO_NUM       36
  #define Y5_GPIO_NUM       21
  #define Y4_GPIO_NUM       19
  #define Y3_GPIO_NUM       18
  #define Y2_GPIO_NUM        5
  #define VSYNC_GPIO_NUM    25
  #define HREF_GPIO_NUM     23
  #define PCLK_GPIO_NUM     22

  #define XCLK_FREQUENZ  20000000
  //#define RESOLUTION  FRAMESIZE_XGA     // with Board_Node32s_WMOS_LOLIN32
  #define RESOLUTION  FRAMESIZE_UXGA      // with Board_AI_Thinker_ESP32-Cam                                 
  #define BILDNAME "LED_CAM"

#elif defined(M5_CAM)

  #define PWDN_GPIO_NUM     -1
  #define RESET_GPIO_NUM    15
  #define XCLK_GPIO_NUM     27
  #define SIOD_GPIO_NUM     25
  #define SIOC_GPIO_NUM     23

  #define Y9_GPIO_NUM       19
  #define Y8_GPIO_NUM       36
  #define Y7_GPIO_NUM       18
  #define Y6_GPIO_NUM       39
  #define Y5_GPIO_NUM        5
  #define Y4_GPIO_NUM       34
  #define Y3_GPIO_NUM       35
  #define Y2_GPIO_NUM       17
  #define VSYNC_GPIO_NUM    22
  #define HREF_GPIO_NUM     26
  #define PCLK_GPIO_NUM     21

  #define XCLK_FREQUENZ  20000000
  #define RESOLUTION  FRAMESIZE_XGA
  #define BILDNAME "M5_CAM"

#else

  #error "Camera Modell not set"

#endif

static camera_config_t camera_config = {
    .pin_pwdn  = PWDN_GPIO_NUM,
    .pin_reset = RESET_GPIO_NUM,
    .pin_xclk  = XCLK_GPIO_NUM,
    .pin_sscb_sda = SIOD_GPIO_NUM,
    .pin_sscb_scl = SIOC_GPIO_NUM,

    .pin_d7 = Y9_GPIO_NUM,
    .pin_d6 = Y8_GPIO_NUM,
    .pin_d5 = Y7_GPIO_NUM,
    .pin_d4 = Y6_GPIO_NUM,
    .pin_d3 = Y5_GPIO_NUM,
    .pin_d2 = Y4_GPIO_NUM,
    .pin_d1 = Y3_GPIO_NUM,
    .pin_d0 = Y2_GPIO_NUM,
    .pin_vsync = VSYNC_GPIO_NUM,
    .pin_href = HREF_GPIO_NUM,
    .pin_pclk = PCLK_GPIO_NUM,

    //XCLK 20MHz or 10MHz for OV2640 double FPS (Experimental)
    .xclk_freq_hz = XCLK_FREQUENZ,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    .pixel_format = PIXFORMAT_JPEG,//YUV422,GRAYSCALE,RGB565,JPEGS    
    .frame_size = RESOLUTION,   //QQVGA-QXGA Do not use sizes above QVGA when not JPEG

    .jpeg_quality = 0, //12, 0-63 lower number means higher quality
    .fb_count = 1 //if more than one, i2s runs in continuous mode. Use only with JPEG
};

esp_err_t camera_init(void);
esp_err_t camera_capture(void);
void setSensordaten(void);
void grabPicture(String filename);

// ############################################ M Q T T ###############################################
void   do_mqtt_command()
{
  if (strcmp(mqtt_command, "waiting_time")==0) {
    int data_value = abs(atoi(mqtt_data));
    if (data_value <= 20) {                        // waiting_time should be >= 20s
      data_value=20;
    }
    Waiting_Time=data_value;
    // save to EEPROM                                              
    EEPROM.put(eeAddress_waiting_time, Waiting_Time);                              
    EEPROM.commit();
  }
  if (strcmp(mqtt_command, "flood")==0) {
    int data_value = abs(atoi(mqtt_data));
    if (data_value ==0) { flood_use=false;}  // only 0 and 1 are allowed for filename_datetime
    else {flood_use=true;}
    EEPROM.put(eeAddress_flood, flood_use);                              
    EEPROM.commit();  
  }  
  if (strcmp(mqtt_command, "filename_datetime")==0) {
    int data_value = abs(atoi(mqtt_data));
    if (data_value ==0) { cam_filename_datetime=false;}  // only 0 and 1 are allowed for filename_datetime
    else {cam_filename_datetime=true;}
      // save to EEPROM                                              
    EEPROM.put(eeAddress_cam_filename_datetime, cam_filename_datetime);                              
    EEPROM.commit();  
    }  
 
  if (strcmp(mqtt_command, "info")==0) {
      mqtt_pub_info[0]='\0';
      strcat(mqtt_pub_info,CODEVERSION);
      strcat(mqtt_pub_topic,mqtt_topic);
      strcat(mqtt_pub_topic,"/info");
      MQTTClient.publish(mqtt_pub_topic,mqtt_pub_info);
      delay(1);
      
      mqtt_pub_info[0]='\0';
      strcat(mqtt_pub_info,"bootCount: ");
      char help1_char[]="";
      itoa ( bootCount, help1_char, 10 ); //char *  itoa ( int value, char * str, int base );
      strcat(mqtt_pub_info,help1_char);
      MQTTClient.publish(mqtt_pub_topic,mqtt_pub_info);
      delay(1);

      mqtt_pub_info[0]='\0';
      strcat(mqtt_pub_info,"waiting_time: ");
      char help4_char[]="";
      itoa ( Waiting_Time, help4_char, 10 ); //char *  itoa ( int value, char * str, int base );
      strcat(mqtt_pub_info,help4_char);
      MQTTClient.publish(mqtt_pub_topic,mqtt_pub_info);
      delay(1);
      
      mqtt_pub_info[0]='\0';
      strcat(mqtt_pub_info,"flood_use: ");
      char help3_char[]="";
      itoa (flood_use, help3_char, 2 );                     //char *  itoa ( int value, char * str, int base );
      strcat(mqtt_pub_info,help3_char);
      MQTTClient.publish(mqtt_pub_topic,mqtt_pub_info);
      delay(1);

      mqtt_pub_info[0]='\0';
      strcat(mqtt_pub_info,"cam_location: ");
      strcat(mqtt_pub_info,cam_location);
      MQTTClient.publish(mqtt_pub_topic,mqtt_pub_info);
      delay(1);
      
      mqtt_pub_info[0]='\0';
      strcat(mqtt_pub_info,"cam_name: ");
      strcat(mqtt_pub_info,cam_name);
      MQTTClient.publish(mqtt_pub_topic,mqtt_pub_info);
      delay(1);

      mqtt_pub_info[0]='\0';
      strcat(mqtt_pub_info,"cam_filename_datetime: ");
      char help2_char[]="";
      itoa ( cam_filename_datetime, help2_char, 2 ); //char *  itoa ( int value, char * str, int base );
      strcat(mqtt_pub_info,help2_char);
      MQTTClient.publish(mqtt_pub_topic,mqtt_pub_info);
      delay(1);

      mqtt_pub_info[0]='\0';
      strcat(mqtt_pub_info,"ftp_server_url: ");
      strcat(mqtt_pub_info,ftp_server_url);
      MQTTClient.publish(mqtt_pub_topic,mqtt_pub_info);
      delay(1);
      
      mqtt_pub_info[0]='\0';
      strcat(mqtt_pub_info,"ftp_pfad: ");
      strcat(mqtt_pub_info,ftp_pfad);
      MQTTClient.publish(mqtt_pub_topic,mqtt_pub_info);
      delay(1);
      
      mqtt_pub_info[0]='\0';
      strcat(mqtt_pub_info,"ftp_user: ");
      strcat(mqtt_pub_info,ftp_user);
      MQTTClient.publish(mqtt_pub_topic,mqtt_pub_info);
      delay(1);          
  }
  if (strcmp(mqtt_command, "restart")==0) {
    Serial.println("Restarting in 10s ....");
    delay(10000);
    ESP.restart();
  }
}

void get_mqtt_command()
{
   if (DEBUG_MTQQ) Serial.println(" ... now in get_mqtt_command");
  
  //clear mqtt_command und mqtt_data
   if (DEBUG_MTQQ) Serial.println(" clearing mqtt_command and mqtt_data");
  for (int i=0; i<strlen(mqtt_command); i++)
  {
    mqtt_command[i] = '\0';
  }
  for (int i=0; i<strlen(mqtt_data); i++)
  {
    mqtt_data[i] = ' ';
  }
  
  bool separator_found = false;;
  int mqtt_command_reading_pointer = 0;
  int mqtt_data_reading_pointer = 0;
  
  for (int i=0; i<strlen(mqtt_message); i++)
  {
    if (DEBUG_MTQQ) Serial.println("i: "+String(i) +" command_reading_pointer: "+String(mqtt_command_reading_pointer) + " data_reading_pointer: " + String(mqtt_data_reading_pointer));
    if (DEBUG_MTQQ) Serial.println("mqtt_command: " + String(mqtt_command) + " mqtt_data: " + String(mqtt_data));
    
    if (mqtt_message[i] != (int)'=') // Separator betwen command and data is =
    {
      if (!separator_found)
      {
        // reading command 
        mqtt_command[mqtt_command_reading_pointer] = mqtt_message[i];  
        mqtt_command_reading_pointer++;       
      } else
      {
        // reading data
        mqtt_data[mqtt_data_reading_pointer] = mqtt_message[i];
        mqtt_data_reading_pointer++;
      }
    } 
    else 
    {
      separator_found = true;;
    }
  }
  //Serial.println();
  if (DEBUG_MTQQ) Serial.println("command_reading_pointer: "+String(mqtt_command_reading_pointer) + " data_reading_pointer: " + String(mqtt_data_reading_pointer));
  //set end of char arrays
  mqtt_command[mqtt_command_reading_pointer] = '\0';
  mqtt_data[mqtt_data_reading_pointer] = '\0';

  if (DEBUG_MTQQ) Serial.println("mqtt_command: " + String(mqtt_command) + " mqtt_data: " + String(mqtt_data));
}

void callback(char* topic, byte* payload, unsigned int length)
{
  if (DEBUG_MTQQ)Serial.println("clearing old Message");
  for (int i=0; i<strlen(mqtt_message); i++)
  {
    mqtt_message[i] = '\0';
  }
  
  if (DEBUG_MTQQ)Serial.println("Message arrived in topic: ");
  if (DEBUG_MTQQ)Serial.println(topic);
  for (int i=0; i<length; i++)
  {
    mqtt_message[i] = (char)payload[i];
  }
  Serial.print("MQTT message: <");
  Serial.print(mqtt_message);
  Serial.println("> received.");
  get_mqtt_command();
  do_mqtt_command();
}

String getMacAddress() {
  uint8_t baseMac[6];
  // Get MAC address for WiFi station
  //esp_read_mac(baseMac, ESP_MAC_WIFI_STA);
  esp_efuse_mac_get_default(baseMac);
  char baseMacChr[18] = {0};
  sprintf(baseMacChr, "%02X:%02X:%02X:%02X:%02X:%02X", baseMac[0], baseMac[1], baseMac[2], baseMac[3], baseMac[4], baseMac[5]);
  return String(baseMacChr);
}

//#######################################################################################################################
//##################################################### s e t u p #######################################################
//#######################################################################################################################

void setup(void)
{
  Serial.begin(115200);
  Serial.println("   ");
  Serial.print("Starting ");
  Serial.println(CODEVERSION);
  Serial.println(AUTHOR);
  Serial.println(NOTE);
  Serial.println();
  pinMode(floodlight, OUTPUT);                                    //Flash-LED
  digitalWrite(floodlight, LOW);                                  //Flash-LED off
  pinMode (LED_1, OUTPUT);                                        //back led
  
  unsigned long startedAt = millis();
  
//Increment boot number and print it every reboot
  ++bootCount;
  Serial.println("Boot number: " + String(bootCount));

//Print the wakeup reason for ESP32
print_wakeup_reason();

// get base mac address for STA
Serial.print("Base MAC Address: ");
Serial.println(getMacAddress());

// Get values from EEPROM
if (!EEPROM.begin(1000)) 
   {
   Serial.println("Failed to initialise EEPROM");
   Serial.println("Restarting...");
   delay(1000);
   ESP.restart();
   }

Serial.println("reading parameter from EEPROM");

int eeAddress = eeAddress_waiting_time;                                              
EEPROM.get(eeAddress, Waiting_Time);                           
Serial.println("Waiting_Time <" + String(Waiting_Time) + "> at EEPROM-Address " + String(eeAddress));    

eeAddress = eeAddress_flood;                                       
EEPROM.get(eeAddress, flood_use);                              
Serial.println("flood_use <" + String(flood_use) + "> at EEPROM-Address " + String(eeAddress));

eeAddress = eeAddress_cam_location;
EEPROM.get(eeAddress, cam_location);                           
Serial.println("cam_location <" + String(cam_location) + "> at EEPROM-Address " + String(eeAddress));

eeAddress = eeAddress_cam_name;
EEPROM.get(eeAddress, cam_name);                           
Serial.println("cam_name <" + String(cam_name) + "> at EEPROM-Address " + String(eeAddress));

eeAddress = eeAddress_cam_filename_datetime;                              
EEPROM.get(eeAddress, cam_filename_datetime);                  
Serial.println("cam_filename_datetime <" + String(cam_filename_datetime) + "> at EEPROM-Address " + String(eeAddress));

eeAddress = eeAddress_ftp_server_url;
EEPROM.get(eeAddress, ftp_server_url);                          
Serial.println("ftp_server_url <" + String(ftp_server_url) + "> at EEPROM-Address " + String(eeAddress)); 

eeAddress = eeAddress_ftp_pfad;
EEPROM.get(eeAddress, ftp_pfad);                                
Serial.println("ftp_pfad <" + String(ftp_pfad) + "> at EEPROM-Address " + String(eeAddress));

eeAddress = eeAddress_ftp_user;
EEPROM.get(eeAddress, ftp_user);                                 
Serial.println("ftp_user <" + String(ftp_user) + "> at EEPROM-Address " + String(eeAddress));

eeAddress = eeAddress_ftp_pw;
EEPROM.get(eeAddress, ftp_pw);                                  
Serial.println("ftp_pw <" + String(ftp_pw) + "> at EEPROM-Address " + String(eeAddress));

eeAddress = eeAddress_mqtt_server_url;
EEPROM.get(eeAddress, mqtt_server_url);                          
Serial.println("mqtt_server_url <" + String(mqtt_server_url) + "> at EEPROM-Address " + String(eeAddress)); 

eeAddress = eeAddress_mqtt_topic;
EEPROM.get(eeAddress, mqtt_topic);                                
Serial.println("mqtt_topic <" + String(mqtt_topic) + "> at EEPROM-Address " + String(eeAddress));

esp_sleep_enable_timer_wakeup(1* uS_TO_S_FACTOR);               // 1s for last step of waiting time

// ############################ WiFiManager ######################

ESP_WiFiManager ESP_wifiManager;


Router_SSID = ESP_wifiManager.WiFi_SSID();
Router_Pass = ESP_wifiManager.WiFi_Pass();
digitalWrite(LED_1, 1);  // Back-LED LED-1 off
  
//Remove this line if you do not want to see WiFi password printed
Serial.println("Stored: SSID = " + Router_SSID);

if (bootCount == 1)
  {
   // Just a quick hint
   ESP_WMParameter p_hint_mqtt("<small>MQTT topic is buildt with mqtt_topic/cam_location</small>");
   // cam_location
   ESP_WMParameter p_cam_location("cam_location","cam_location like CAM_LOC",cam_location,16);
   // cam_name
   ESP_WMParameter p_cam_name("cam_name","cam_name like MYCAM",cam_name,16);
   char convertedValue1[2];
   sprintf(convertedValue1, "%d", cam_filename_datetime);
   ESP_WMParameter p_cam_filename_datetime("file datetime", "  use 1 for filename as cam-location-20200113-135854.jpg and use 0 for cam-location-cam-name.jpg", convertedValue1, 2);
   // ftp_server_url
   ESP_WMParameter p_ftp_server_url("ftp_server_url","FTP Server URL like a.b.c",ftp_server_url,32);
   // ftp_pfad
   ESP_WMParameter p_ftp_pfad("ftp_pfad","FTP dfirectory like /mydir/cam",ftp_pfad,64);
   // ftp_user
   ESP_WMParameter p_ftp_user("ftp_user","FTP user",ftp_user,16);
   // ftp_pw
   ESP_WMParameter p_ftp_pw("ftp_pw","FTP pw",ftp_pw,16);
   // mqtt_server_url
   ESP_WMParameter p_mqtt_server_url("mqtt_server_url","MQTT Server URL like a.b.c",mqtt_server_url,32);
   // mqtt_topic
   ESP_WMParameter p_mqtt_topic("mqtt_topic","MQTT Topic like /myIoT/test",mqtt_topic,64);
   // Waiting Time
   char convertedValue2[6];
   sprintf(convertedValue2, "%d", Waiting_Time);
   ESP_WMParameter p_custom_waiting_time("Waiting_Time", "waiting time for next pictute in s", convertedValue2, 6);
   // flood usage
   char convertedValue3[2];
   sprintf(convertedValue3, "%d", flood_use);
   ESP_WMParameter p_flood("Flood", "  use 1 for flood LED on and 0 for off", convertedValue3, 2);

   //add all parameters here
   ESP_wifiManager.addParameter(&p_cam_location);
   ESP_wifiManager.addParameter(&p_cam_name);
   ESP_wifiManager.addParameter(&p_cam_filename_datetime);
   ESP_wifiManager.addParameter(&p_ftp_server_url);
   ESP_wifiManager.addParameter(&p_ftp_pfad);
   ESP_wifiManager.addParameter(&p_ftp_user);
   ESP_wifiManager.addParameter(&p_ftp_pw);
   ESP_wifiManager.addParameter(&p_hint_mqtt);
   ESP_wifiManager.addParameter(&p_mqtt_server_url);
   ESP_wifiManager.addParameter(&p_mqtt_topic);
   ESP_wifiManager.addParameter(&p_custom_waiting_time);
   ESP_wifiManager.addParameter(&p_flood);
   
   Serial.print("Opening configuration portal with ");
   digitalWrite(LED_1, 0);  // Back-LED LED-1 on
   if (Router_SSID != "")
   {
    Serial.println("timeout of 60s");
    ESP_wifiManager.setConfigPortalTimeout(60); //If no access point name has been previously entered disable timeout.
   }
   else
    Serial.println("no timeout");
    // SSID to uppercase 
    ssid.toUpperCase();  
    //it starts an access point and goes into a blocking loop awaiting configuration
    if (!ESP_wifiManager.startConfigPortal((const char *) ssid.c_str(), password)) 
      Serial.println("Not connected to WiFi but continuing anyway.");
    else 
      Serial.println("WiFi connected...yeah ;-)");
  
    digitalWrite(LED_1, 1); // Back-LED LED-1 off

    // saving the values to EEPROM
    Serial.println("saving new parameter to EEPROM");
    int eeAddress = eeAddress_waiting_time;                                               
    Waiting_Time = atoi(p_custom_waiting_time.getValue());
    Serial.println("waiting_time <"+String(Waiting_Time)+"> at EEPROM-Address " + String(eeAddress));
    EEPROM.put(eeAddress, Waiting_Time);                              
    
    eeAddress = eeAddress_flood;    
    Serial.println("flood: "+String(p_flood.getValue()));  
    flood_use = atoi(p_flood.getValue());
    Serial.println("flood_use <"+String(flood_use)+"> at EEPROM-Address " + String(eeAddress));
    EEPROM.put(eeAddress, flood_use);                              
    
    eeAddress = eeAddress_cam_location;
    strcpy(cam_location, p_cam_location.getValue());
    Serial.println("cam_location <"+String(cam_location)+"> at EEPROM-Address " + String(eeAddress));
    EEPROM.put(eeAddress, cam_location);                            

    eeAddress = eeAddress_cam_name;
    strcpy(cam_name, p_cam_name.getValue());
    Serial.println("cam_name >"+String(cam_name)+"> at EEPROM-Address " + String(eeAddress));
    EEPROM.put(eeAddress, cam_name);                           
    
    eeAddress = eeAddress_cam_filename_datetime;  
    cam_filename_datetime = atoi(p_cam_filename_datetime.getValue());
    Serial.println("cam_filename_datetime <"+String(cam_filename_datetime)+"> at EEPROM-Address " + String(eeAddress));
    EEPROM.put(eeAddress, cam_filename_datetime);                  

    eeAddress = eeAddress_ftp_server_url;
    strcpy(ftp_server_url, p_ftp_server_url.getValue());
    Serial.println("ftp_server_url <"+String(ftp_server_url)+"> at EEPROM-Address " + String(eeAddress));
    EEPROM.put(eeAddress, ftp_server_url);                          

    eeAddress = eeAddress_ftp_pfad;
    strcpy(ftp_pfad, p_ftp_pfad.getValue());
    Serial.println("ftp_pfad <"+String(ftp_pfad)+"> at EEPROM-Address " + String(eeAddress));
    EEPROM.put(eeAddress, ftp_pfad);                                 

    eeAddress = eeAddress_ftp_user;
    strcpy(ftp_user, p_ftp_user.getValue());
    Serial.println("ftp_user <"+String(ftp_user)+"> at EEPROM-Address " + String(eeAddress));
    EEPROM.put(eeAddress, ftp_user);                                
    
    eeAddress = eeAddress_ftp_pw;
    strcpy(ftp_pw, p_ftp_pw.getValue()); 
    Serial.println("ftp_pw <"+String(ftp_pw)+"> at EEPROM-Address " + String(eeAddress));
    EEPROM.put(eeAddress, ftp_pw);                                  

    eeAddress = eeAddress_mqtt_server_url;
    strcpy(mqtt_server_url, p_mqtt_server_url.getValue());
    Serial.println("mqtt_server_url <" + String(mqtt_server_url) + "> at EEPROM-Address " + String(eeAddress)); 
    EEPROM.put(eeAddress, mqtt_server_url);                          

    eeAddress = eeAddress_mqtt_topic;
    strcpy(mqtt_topic, p_mqtt_topic.getValue());
    Serial.println("mqtt_topic <" + String(mqtt_topic) + "> at EEPROM-Address " + String(eeAddress));
    EEPROM.put(eeAddress, mqtt_topic);                                

    EEPROM.commit();
  }
  // For some unknown reason webserver can only be started once per boot up 
  // so webserver can not be used again in the sketch.
  #define WIFI_CONNECT_TIMEOUT        30000L
  #define WHILE_LOOP_DELAY            1000L   //200L
  #define WHILE_LOOP_STEPS            (WIFI_CONNECT_TIMEOUT / ( 3 * WHILE_LOOP_DELAY ))
  
  
  while ( (WiFi.status() != WL_CONNECTED) && (millis() - startedAt < WIFI_CONNECT_TIMEOUT ) )
  {   
    WiFi.mode(WIFI_STA);
    WiFi.persistent (true);
    // We start by connecting to a WiFi network
    
    Serial.print("Connecting to ");
    Serial.println(Router_SSID);
    
    WiFi.begin(Router_SSID.c_str(), Router_Pass.c_str());

    int i = 0;
    while((!WiFi.status() || WiFi.status() >= WL_DISCONNECTED) && i++ < WHILE_LOOP_STEPS)
    {
      delay(WHILE_LOOP_DELAY);
    }    
  }

  Serial.print("After waiting ");
  Serial.print((millis()- startedAt) / 1000);
  Serial.print(" secs in setup, connection result is ");

  if (WiFi.status() == WL_CONNECTED)
  {
    Serial.print("connected! at ");
    Serial.print(WiFi.SSID());
    Serial.print(" with Local IP ");
    Serial.println(WiFi.localIP());
  }
  else
  {
    Serial.println(ESP_wifiManager.getStatus(WiFi.status()));
    Serial.println("Failed to connect to WiFi ... restarting");
    delay(1000);
    ESP.restart();
  }
  
// ################### Get IP-Adress of FTP-Server by Name ##################
WiFi.hostByName(ftp_server_url, ftpserver);
Serial.print("FTP-Server: "+String(ftp_server_url)+ " ");
Serial.println(ftpserver);

// ################### Get IP-Adress of MQTT-Server by Name ##################
WiFi.hostByName(mqtt_server_url, mqttserver);
Serial.print("MQTT-Server: "+String(mqtt_server_url)+ " ");
Serial.println(mqttserver);

// ####################### Get DateTime ###################
configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
receiveLocalTime();

// ####################### MQTT #####################
MQTTClient.setServer(mqttserver,mqtt_port);
MQTTClient.setCallback(callback);
int mqtt_connect_time = 0; //Waiting_Time

while (!MQTTClient.connected()) {
  Serial.print("Waiting count "+String(mqtt_connect_time) + ", Connecting to MQTT ... ");
//  if (MQTTClient.connect("CAM-MQTT",mqtt_user, mqtt_pw)) {
  if (MQTTClient.connect("CAM-MQTT")) {
    Serial.println(" connected.");
  } else {
    Serial.print(" failed with state ");
    Serial.println(MQTTClient.state());
    delay(2000); // wait 2 s til next try
    mqtt_connect_time++;
    if (mqtt_connect_time >= Waiting_Time/10) {
      Serial.println("Failed to connect to MQTT broker ... restarting");
      delay(1000);
      ESP.restart();
    }
  } 
} // while !MQTTClient.connected())

// subscribing to mqtt_topic
strcat(mqtt_topic,"/");
strcat(mqtt_topic,cam_location);
Serial.println("Subscribing to MQTT-Topic " + String(mqtt_topic));
MQTTClient.subscribe(mqtt_topic);



// ############################ FTP ######################
ftp_comm[0] = String("USER "+ String(ftp_user));
ftp_comm[1] = String("PASS "+ String(ftp_pw));
ftp_comm[2] = String("SYST");
ftp_comm[3] = String("Type I");

// ################### Camera init ##########################
 if (flood_use)
  {
    digitalWrite(floodlight, HIGH); //Flash-LED on
  }
  
  delay(10); //wait  10ms

  esp_err_t err = esp_camera_init(&camera_config);
  if (err == ESP_OK)
  {
    Serial.println("Cam Init ok");
  }
  else  
  {
    Serial.println("Cam Init FAIL");
  }
  setSensordaten();

  sensor_t *s = esp_camera_sensor_get();
  
#ifndef PIR_CAM
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);
#endif  
}

// #####################################################################################
// ################################ M A I N - L o o p ##################################
// #####################################################################################

void loop(void)
{
  // MQTT
  MQTTClient.loop(); 
  
  String filename = "Bild.jpg";
  Serial.println(filename);
  uploadFTP(filename);  
  digitalWrite(floodlight, LOW); //Flash-LED off

  //Serial.println("Going to sleep now for " + String(TIME_TO_SLEEP) + " Seconds");
  delay(1); // delay(10);
  Serial.flush(); 

//  WiFi.disconnect();
//  WiFi.mode(WIFI_OFF);
  btStop();  

#if defined (PIR_CAM)
  esp_sleep_enable_ext0_wakeup((gpio_num_t )BUTTON, LOW);  
  esp_sleep_enable_ext0_wakeup((gpio_num_t )PIR_OUT, HIGH); 

  pinMode(PWDN_GPIO_NUM,PULLUP);                      // Kamera aus
  digitalWrite(PWDN_GPIO_NUM, HIGH);
#endif

  Serial.println("Waiting "+ String(Waiting_Time)+"s for next picture.");
  int time_count = 1;
  while (Waiting_Time > 0)
  {    
    currentMillis = millis();
    previousMillis = currentMillis;
    while (currentMillis - previousMillis < delay_s*1000)
      {
        currentMillis = millis();
        // MQTT
        MQTTClient.loop();
      }
    digitalWrite(LED_1, 0);                     // Back-LED LED-1 on
    delay(10);                                  // 10ms delay
    MQTTClient.loop(); 
    digitalWrite(LED_1, 1);                     // Back-LED LED-1 off   
    Waiting_Time = Waiting_Time - delay_s;  
    if (time_count < 10)
      {
        Serial.print(".");
        ++time_count;
      } else
      { Serial.print("!");
        time_count=1;
      }
  }
  Serial.println();
  MQTTClient.loop(); 
  esp_deep_sleep_start();
  
  Serial.println("This will never be printed");
}

//#########################################################
//-------------------- Sleep Tools ------------------------
//#########################################################


void print_wakeup_reason()
{
  esp_sleep_wakeup_cause_t wakeup_reason;
  wakeup_reason = esp_sleep_get_wakeup_cause();
  switch(wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0:
      Serial.println("Wakeup caused by external signal using RTC_IO");
      wake_reason = WAKE_PIR;
      break;

    case ESP_SLEEP_WAKEUP_EXT1:
      Serial.println("Wakeup caused by external signal using RTC_CNTL");
      break;

    case ESP_SLEEP_WAKEUP_TIMER:
      Serial.println("Wakeup caused by timer");
      wake_reason = WAKE_TIMER;
      break;

    case ESP_SLEEP_WAKEUP_TOUCHPAD:
      Serial.println("Wakeup caused by touchpad");
      break;
 
    case ESP_SLEEP_WAKEUP_ULP:
      Serial.println("Wakeup caused by ULP program");
      break;

    default:
      Serial.printf("Wakeup was not caused by deep sleep: %d\n",wakeup_reason);
      break;
  }
}

//#########################################################
//--------------------- CAM Tools -------------------------
//#########################################################

void setSensordaten()
{
  size_t res = 0;
  sensor_t * s = esp_camera_sensor_get();

  res = s->set_framesize(s, (framesize_t) FRAMESIZE_UXGA);      // with Board_AI_Thinker_ESP32-Cam
                                                                // with Board_Node32s_WMOS_LOLIN32: //res = s->set_framesize(s, (framesize_t) FRAMESIZE_XGA);
                                                                // Framesize      FRAMESIZE_(QQVGA, HQVGA, QVGQ, CIF, VGA, SVGA, XGA, SXGA, UXGA)
  res = s->set_quality(s, 10);                                  // JPEG Quality   (10...63)  10
  res = s->set_contrast(s, 0);                                  // Kontrast       (-2...+2)   0
  res = s->set_brightness(s, 0);                                // Helligkeit     (-2...+2)   0
  res = s->set_saturation(s, 0);                                // Farbsättigung  (-2...+2)   0 
  res = s->set_gainceiling(s, (gainceiling_t) GAINCEILING_16X);  // Verstärkung    GAINCEILING_(2x, 4x, 8x, 16x, 32x, 64x, 128x)   2x
  //on: 1, 0: off
  res = s->set_colorbar(s, 0);                                  // Farbbalken     (off/on)   off 
  res = s->set_whitebal(s, 1);                                  // Weißbalance    (off/on)    on
  res = s->set_gain_ctrl(s, 1);                                 // AGC            (off/on)    on
  res = s->set_exposure_ctrl(s, 1);                             // AEC            (off/on)    on               
//  res = s->set_hmirror(s, 1);                                   // H-Mirror       (off/on) 
//  res = s->set_vflip(s, val);                                   // V-Flip         (off/on) 
  res = s->set_awb_gain(s, 1);                                // Verstärkung AWB
  res = s->set_agc_gain(s, 1);                                // Verstärkung AGC
  res = s->set_aec_value(s, 600);                               // Wert AEC       (0...1200)  
  res = s->set_aec2(s, 1);                                    // Wert AEC2
  res = s->set_dcw(s, 0);                                       // Downsize       (off/on)    on    
  res = s->set_bpc(s, 1);                                       //                (off/on)   off
  res = s->set_wpc(s, 1);                                       //                (off/on)    on
  res = s->set_raw_gma(s, 1);                                   // RAW-Gamma      (off/on)    on
  res = s->set_lenc(s, 1);                                      // Linsenkorr.    (off/on)    on 
//  res = s->set_special_effect(s, val);                            // Special Effekt 
  res = s->set_wb_mode(s, 1);                                   // WB Mode         (Auto/Sunny/..) 
  res = s->set_ae_level(s, 0);                                  // AE Level        (-2...+2)    0
}

//#########################################################
//--------------------- FTP Tools -------------------------
//#########################################################

String uploadFTP(String filename)
{ 
  for (uint8_t i = 0; i < 3; i++)
  {
    camera_fb_t * fb = esp_camera_fb_get();
    esp_camera_fb_return(fb);  
    delay(1);   //delay(10); 
  }
  camera_fb_t * fb = esp_camera_fb_get();

  digitalWrite(floodlight, LOW); //Flash-LED off

  String returnText = "";
  String meldung = "";
  bool flag_ftp_ok = true;

  String bildnummer = "00000" + String(bootCount, DEC);
  //bildnummer = bildnummer.substring(bildnummer.length() - 5);
  //String dateiname = String(wake_reason, DEC) + "_" + BILDNAME + "_" + bildnummer + ".jpg";
  //String dateiname = String(wake_reason, DEC) + "_" + BILDNAME  + ".jpg";
  //String dateiname = String(wake_reason, DEC) + "_" + BILDNAME + ".jpg";

  if (cam_filename_datetime)
     {  
      dateiname = String(cam_location) + "_" + picturePath + ".jpg";
     } 
     else
     {
      dateiname = String(cam_location) + "_" + cam_name + ".jpg";
     }
  // Serial.println(dateiname);     
  
  if (ftpclient.connect(ftpserver, ftpport)) // 21 = FTP server
  { 
    Serial.println(F("FTP Command connected"));
  }
  else
  {
    Serial.println(F("FTP Command connection failed"));
    return "FTP Error";
  }

  returnText = ftpReceive();
  meldung = returnText.substring(0,3); // erste 3 Zeichen
  if (DEBUG_FTP) Serial.println("Return: " + meldung);
  if (meldung == "220")
  {
    for (uint8_t i = 0; i < FTP_ANZAHL; i++)
    {
      if (!ftpCommand(i))
      {
        flag_ftp_ok = false;
        break;
      }  
    }  

    if (flag_ftp_ok)    // alles klar, PASV senden
    {
      uint8_t werte[6];
      uint8_t pos = 0;
      uint8_t start = 0;
      uint8_t komma = 0;
      uint16_t dport = 0;
      
      if (DEBUG_FTP) Serial.println("PASV");
      ftpclient.println("PASV");
      String returnText = ftpReceive();
      String meldung = returnText.substring(0,3); // erste 3 Zeichen
      if (DEBUG_FTP) Serial.println("Return: " + meldung); 
      if (meldung == "227")
      {
        start = returnText.indexOf("(");
        for (uint8_t i = 0; i < 5; i++)
        {
          komma = returnText.indexOf(",", start + 1);
          werte[pos] = returnText.substring(start + 1, komma).toInt();
          if (DEBUG_FTP) Serial.println(werte[pos], DEC);
          pos++; 
          start = komma;
        }       
        werte[pos] = returnText.substring(start + 1).toInt();
        if (DEBUG_FTP) Serial.println(werte[pos], DEC);
        dport = (werte[4] << 8) | werte[5];
        if (DEBUG_FTP) Serial.println("Datenport: " + String(dport, DEC));
        if (dclient.connect(ftpserver, dport))
        {
          Serial.println(F("FTP Data connected"));
        }
        else
        {
          Serial.println(F("FTP Data connection failed"));
          ftpclient.stop();
          return "Error";
        }
        if (DEBUG_FTP) Serial.println("CWD " + String(ftp_pfad));
        ftpclient.println("CWD " + String(ftp_pfad));
        String returnText = ftpReceive();
        String meldung = returnText.substring(0,3); // erste 3 Zeichen
        if (DEBUG_FTP) Serial.println("Return: " + meldung); 
      
        if (meldung == "250")
        {
          Serial.print(dateiname);
          if (DEBUG_FTP) Serial.println("STOR " + dateiname);
          ftpclient.println("STOR " + dateiname);
          String returnText = ftpReceive();
          String meldung = returnText.substring(0,3); // erste 3 Zeichen
          if (DEBUG_FTP) Serial.println("Return: " + meldung); 
          if (meldung == "150")
          {
            Serial.print(F(" ... writing "));
            dclient.write((const uint8_t *)fb->buf, fb->len);
            Serial.println(F("done"));
          }          
        }        
        dclient.stop();
        Serial.println(F("FTP Data disconnected"));
      }  
    }

  }
 
  ftpclient.println(F("QUIT"));
  ftpclient.stop();
  Serial.println(F("FTP Command disconnected"));

  esp_camera_fb_return(fb);  

  return "OK";
} 

//----------------------------------------------------------

String ftpReceive()  
{                       // gibt nur die letzte Zeile zurück
  char thisByte;
  uint8_t count = 0;
  String outText[20];
  String retText = "";
  
  while (!ftpclient.available()) delay(1);  // auf Daten warten
  while (ftpclient.available())
  {
    thisByte = ftpclient.read();
#ifdef DEBUG_FTP
    if (DEBUG_FTP) Serial.write(thisByte);
#endif
    if (thisByte == 13)          // return
    {
      count++;      // nächste Zeile
    }
    else if (thisByte != 10)          // newline
    {
      outText[count] += thisByte;
    }
  }  
  for (uint8_t i = 20; i > 0; i--)
  {
    if (outText[i - 1] > "")
    {
      retText = outText[i - 1];
      break;
    }  
  }  
  return retText;
} 

//----------------------------------------------------------

bool ftpCommand(uint8_t index)
{
  if (DEBUG_FTP) Serial.println(ftp_comm[index]);
  ftpclient.println(ftp_comm[index]);
  String returnText = ftpReceive();
  String meldung = returnText.substring(0,3); // erste 3 Zeichen
  if (DEBUG_FTP) Serial.println("Return: " + meldung);
  if (meldung == ftp_ret[index])
  {
    return true;
  }
  else  
  {
    return false;
  }
}
