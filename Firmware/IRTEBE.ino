/*IRTEBE Main firmware */


/*This code is based FreeRTOS. Some libraries are required:
 * Preqrequisites: HX711_ADC, SdFAT (v2)
 * 
 * Teensy 4.1:
 * Required:  Teensyduino, FreeRTOS_TEENSY4, NativeEthernet
 * -features the Ethernet PHY connection for transmission
 * 
 * MKR1000:
 * Required:  SAMD21, Wifi101 FreeRTOS_SAMD21.h
 * -features the WiFi connection for transmission
 * 
 * Any AVR based:
 * Required: FreeRTOS
 */


 /* Task overview:
  *  -------------
  *  Priority |   Task
  *       3   | Measurement cycling
  *       2   | Acquisition ADC, Acquisition HX711, TCP socket communication
  *       1   | SdWrite, Housekeeping, Wifi
  *       
  *       
  */

   /* Interrupt overview:
  *  -------------
  *  Timer x  | 1 kHz   | Stepper motor step()
  *       
  *       
  */



#ifdef TEENSY_CORE
  //Teensy 4.1:  7936K Flash, 1024K RAM (512K tightly coupled)
  #include <EEPROM.h>
  #include "FreeRTOS_TEENSY4.h" //Using TEENSY 4 board
#elif defined(_SAMD21_)
  //#include "mod/ADS1256/ADS1256.h"
  #include <FreeRTOS_SAMD21.h> //Using Arduino MKR1000 board with SAMD21
#else defined(__AVR___)

#endif

#include "IRTEBE.h"      // Definition of measurement file

//The following libraries are needed (install via TOOLS-Manage libraries
#include <HX711_ADC.h>
#include "SdFat.h" // SdFAT2



/* Buffer: SD Card writing */
#define SD_STREAMBUFFER_SIZE 4192
#define SD_TRIGGERLEVEL 128
/* The stream buffer that is used to send data from an interrupt to the task. */
static StreamBufferHandle_t xStreamBuffer = NULL;

#define ERROR_SD 0
#define ERROR_NETWORK 1



uint8_t run_id = 0;
uint8_t reg_status = 0;
uint8_t reg_error = 0;
sMeasurement currentMeasurement; // During one interval new measurements are written into file

const uint8_t EEPROM_ADDR = 100;

// Declare a semaphore handle.
SemaphoreHandle_t sem;

//=====================================================================================                                  
//==================================Initialisation=====================================
//=====================================================================================  
void setup() {

   // Initialize some randomndess
  randomSeed(analogRead(0));
  
  Serial.begin(115200);
  delay(1000);
  Serial.println("main::TEENSY IRTEBE firmware");

  
  portBASE_TYPE s1, s2,s3,s4,s5;

  
  //get information on how many times arduino was started. this will reflect in the filename
  #ifdef TEENSY_CORE
    run_id = EEPROM.read(EEPROM_ADDR);
    EEPROM.write(EEPROM_ADDR, run_id+1);
  #endif

  


  

   xStreamBuffer = xStreamBufferCreate( /* The buffer length in bytes. */
                     SD_STREAMBUFFER_SIZE,
                     /* The stream buffer's trigger level. */
                     SD_TRIGGERLEVEL);
  
  // initialize semaphore
  sem = xSemaphoreCreateCounting(1, 0);

  // create task at priority two
  s1 = xTaskCreate(Thread1, "LED1", configMINIMAL_STACK_SIZE, NULL, 2, NULL);

  // create task at priority one
  s2 = xTaskCreate(Thread2, "LED2", configMINIMAL_STACK_SIZE, NULL, 1, NULL);


  // Housekeeping
  s3 = xTaskCreate(pHousekeeping, "HK", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY+1, NULL);

  // SD Card write
  s4 = xTaskCreate(pSdWrite, "SD", configMINIMAL_STACK_SIZE+1000, NULL, tskIDLE_PRIORITY+1, NULL);


  #if defined(_SAMD21_)
    s5 = xTaskCreate(pWifi, "WIFI", configMINIMAL_STACK_SIZE+100, NULL, tskIDLE_PRIORITY+1, NULL);
  #endif

  // check for creation errors
  if (sem== NULL || s1 != pdPASS || s2 != pdPASS || s3 != pdPASS || s4 != pdPASS) {
    Serial.println("main::Creation problem");
  }

  Serial.println("main::Starting the scheduler !");


   delay(5000);
  // start scheduler
  vTaskStartScheduler();
  Serial.println("main::Insufficient RAM");
  while(1);
}
//------------------------------------------------------------------------------
// WARNING idle loop has a very small stack (configMINIMAL_STACK_SIZE)
// loop must never block
void loop() {
  // Not used.
}



//=====================================================================================                                  
//=================THREAD==========SD card write =====================================
//=====================================================================================  

// -- SD Card writing (low priority task)
//const uint8_t SD_CS_PIN = SDCARD_SS_PIN;
const uint8_t SD_CS_PIN = 15;

// SD_FAT_TYPE = 0 for SdFat/File as defined in SdFatConfig.h,
// 1 for FAT16/FAT32, 2 for exFAT, 3 for FAT16/FAT32 and exFAT.
#define SD_FAT_TYPE 1

static void pSdWrite( void *pvParameters )
{
uint8_t cRxBuffer[ SD_TRIGGERLEVEL ];
size_t rx_cnt; 

char filename[32];
snprintf(filename, 32,"log_%i_%i.bin", run_id,rand()%1000);
Serial.print("sd::  Filename: ");
Serial.println(filename);


SdFat32 sd;
File32 file;

TickType_t time_lastflush = xTaskGetTickCount();

if (!sd.begin(SdSpiConfig(SD_CS_PIN, DEDICATED_SPI, SD_SCK_MHZ(50)))) {
  //reg_error |= 1<<ERROR_SD;
  Serial.println("sd:: Dedicated SPI mode ERROR.");
}else  {
Serial.println("sd:: Dedicated SPI mode SUCCESS.");
}

if (!file.open(filename, O_RDWR | O_CREAT)) {
  //reg_error |= 1<<ERROR_SD;
  Serial.println("sd:: File ERROR.");
}else{
file.sync();
Serial.println("sd:: File Success.");
}




  /* Remove warning about unused parameters. */
  ( void ) pvParameters;

  /* Make sure the stream buffer has been created. */
  configASSERT( xStreamBuffer != NULL );

  /* Start with the Rx buffer in a known state. */
  memset( cRxBuffer, 0x00, sizeof( cRxBuffer ) );

  for( ;; )
  {
    /* Keep receiving characters until the end of the string is received.
    Note:  An infinite block time is used to simplify the example.  Infinite
    block times are not recommended in production code as they do not allow
    for error recovery. */
    rx_cnt = xStreamBufferReceive( /* The stream buffer data is being received from. */
                xStreamBuffer,
                /* Where to place received data. */
                ( void * ) cRxBuffer,
                /* The number of bytes to receive. */
                SD_TRIGGERLEVEL,
                /* The time to wait for the next data if the buffer
                is empty. */
                pdMS_TO_TICKS(1000) );

    //  size_t write(const void* buf, size_t count);
   file.write(cRxBuffer,rx_cnt);

   // Flush occasionally
   if (xTaskGetTickCount()-time_lastflush > pdMS_TO_TICKS(5000)){
    time_lastflush=xTaskGetTickCount();
    file.flush();
    Serial.println("sdTask:Flush.");
    }

    //This would reloop immediately and would receive any number of bytes. Add a wait.
    while(xStreamBufferBytesAvailable( xStreamBuffer ) < SD_TRIGGERLEVEL)
    {
      vTaskDelay(1);
    }
                
  }
}

void vApplicationStackOverflowHook( TaskHandle_t xTask,
                                    signed char *pcTaskName )
                                    {
                                     
Serial.print("main: RTOS Stackoverflow.");
for(;;){};
                                    }


//=====================================================================================                                  
//=================THREAD============Example Threads======================================
//=====================================================================================  

// The LED is attached to pin 13 on the Teensy 4.0
const uint8_t LED_PIN = 13;

//------------------------------------------------------------------------------
/*
 * Thread 1, turn the LED off when signalled by thread 2.
 */
// Declare the thread function for thread 1.
static void Thread1(void* arg) {
  while (1) {

    // Wait for signal from thread 2.
    xSemaphoreTake(sem, portMAX_DELAY);

    // Turn LED off.
    digitalWrite(LED_PIN, LOW);
  }
}
//------------------------------------------------------------------------------
/*
 * Thread 2, turn the LED on and signal thread 1 to turn the LED off.
 */
// Declare the thread function for thread 2.
static void Thread2(void* arg) {

  pinMode(LED_PIN, OUTPUT);

  while (1) {
    // Turn LED on.
    digitalWrite(LED_PIN, HIGH);


    // Sleep for 200 milliseconds.
    vTaskDelay((200L * configTICK_RATE_HZ) / 1000L);

    // Signal thread 1 to turn LED off.
    xSemaphoreGive(sem);

    // Sleep for 200 milliseconds.
    vTaskDelay((200L * configTICK_RATE_HZ) / 1000L);
  }
}
//=====================================================================================                                  
//=================THREAD============Housekeeping======================================
//=====================================================================================  
char bufptr1[200];
static void pHousekeeping(void* arg) {



  for(;;) {
    
    uint8_t test[21];
    xStreamBufferSend(xStreamBuffer,test,20,0);
   
    size_t sd_free =xStreamBufferSpacesAvailable( xStreamBuffer );
    size_t sd_full =xStreamBufferBytesAvailable( xStreamBuffer );
     
    Serial.print("hk::");
    Serial.print("sd");Serial.print(sd_full); Serial.print("/"); Serial.println(sd_free); 



    
   vTaskGetRunTimeStats(bufptr1);  
   Serial.write(bufptr1);


    // Sleep for 100 milliseconds.
    vTaskDelay((1000L * configTICK_RATE_HZ) / 1000L);
  }
}
                                    
//=====================================================================================                                  
//=================THREAD============WIFI on MKR1000===================================
//=====================================================================================  

#if defined(_SAMD21_)
#include <FreeRTOS_SAMD21.h>
#include <SPI.h>
#include <WiFi101.h>
#include <WiFiUdp.h>
#include "arduino_secrets.h" 

int status = WL_IDLE_STATUS;

///////please enter your sensitive data in the Secret tab/arduino_secrets.h
char ssid[] = SECRET_SSID;        // your network SSID (name)
char pass[] = SECRET_PASS;    // your network password (use for WPA, or use as key for WEP)
int keyIndex = 0;            // your network key Index number (needed only for WEP)

unsigned int localPort = 2390;      // local port to listen on
char packetBuffer[255]; //buffer to hold incoming packet
char  ReplyBuffer[255] = "acknowledged";       // a string to send back
WiFiUDP vUdp;

static void pWifi(void* arg) {

  // check for the presence of the shield:
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("wifi::WiFi shield not present");
    // don't continue:
    while (true);
  }

  // attempt to connect to WiFi network:
  while ( status != WL_CONNECTED) {
    Serial.print("wifi::Attempting to connect to SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);

    // wait 10 seconds for connection:
    vTaskDelay((10000L * configTICK_RATE_HZ) / 1000L);
  }
  Serial.println("wifi:: Connected to wifi");
  //printWiFiStatus();

  Serial.println("wifi:: Starting connection to server...");
  // if you get a connection, report back via serial:
  vUdp.begin(localPort);


uint16_t  cnt=0;

for(;;){

  vTaskDelay( (500L * configTICK_RATE_HZ) / 1000L);

  cnt++;

 unsigned long tstart= micros();
  // if there's data available, read a packet

   IPAddress broadcastIp(192,168,178,255);
   vUdp.beginPacket(broadcastIp, 2390);

   sprintf(ReplyBuffer,"%i_%i",cnt,tstart);

   
   vUdp.write(ReplyBuffer);
   vUdp.endPacket();
   unsigned long tdiff= micros()-tstart;

   Serial.println(tdiff);
   
  
  int packetSize = vUdp.parsePacket();
  if (packetSize)
  {
    Serial.print("Received packet of size ");
    Serial.println(packetSize);
    Serial.print("From ");
    IPAddress remoteIp = vUdp.remoteIP();
    Serial.print(remoteIp);
    Serial.print(", port ");
    Serial.println(vUdp.remotePort());

    // read the packet into packetBufffer
    int len = vUdp.read(packetBuffer, 255);
    if (len > 0) packetBuffer[len] = 0;
    Serial.println("Contents:");
    Serial.println(packetBuffer);

    vUdp.beginPacket(remoteIp, vUdp.remotePort());
    vUdp.write(ReplyBuffer);
    vUdp.endPacket();
    unsigned long tdur= micros()-tstart;
  }

    // 600 us for one packet
    // send a reply, to the IP address and port that sent us the packet we received

} //loop


} //thread WIFI

#endif // #ifdef defined(_SAMD21_)
