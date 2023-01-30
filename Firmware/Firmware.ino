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
	*       4   | Measurement cycle
	*       3   |
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
   //MKR1000: 32kByte RAM, 262kByte ROM
	//#include "mod/ADS1256/ADS1256.h"
	#include "FreeRTOS_SAMD21.h" //Using Arduino MKR1000 board with SAMD21, FreeRTOS 10
#else defined(__AVR___)

#endif




#include "IRTEBE.h"      // Definition of measurement file

//The following libraries are needed (install via TOOLS-Manage libraries
#include <HX711_ADC.h>
// Consider switching to : HX711-multi uses only 1 clock line for several modules.
#include "SdFat.h" // SdFAT2



/* Buffer: SD Card writing */
#define SD_STREAMBUFFER_SIZE 4192
#define SD_TRIGGERLEVEL 128
/* The stream buffer that is used to send data from an interrupt to the task. */
static StreamBufferHandle_t xStreamBuffer = NULL;


#define QUEUE_LENGTH    10
#define ITEM_SIZE       sizeof( sMeasurement_t )
static QueueHandle_t xQueue = NULL;



#define ERROR_SD 0
#define ERROR_NETWORK 1
#define ERROR_MUTEX 2
#define ERROR_QUEUE_NET 3
#define ERROR_QUEUE_SD 4
#define ERROR_ETH 5


uint8_t run_id = 0;
uint8_t reg_status = 0;
uint8_t reg_error = 0;

sMeasurement_t dCurrentMeas; // During one interval new measurements are written into file
SemaphoreHandle_t xCurrentMeas_mutex;



const uint8_t EEPROM_ADDR = 100;

// Declare a semaphore handle.
SemaphoreHandle_t sem;



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
//=================THREAD============Meassurement cycling==============================
//=====================================================================================  
byte pin_info_state = 0;

static void pMeasurement(void* arg) {

  pinMode(pin_info, OUTPUT);
  const TickType_t taskPeriod = 1000; // 20ms <--> 50Hz
  
  TickType_t xLastWakeTime = xTaskGetTickCount();
  for(;;){
    
    vTaskDelayUntil(&xLastWakeTime, taskPeriod);
        
    pin_info_state=~pin_info_state;
    digitalWrite(PIN_INFO1, pin_info_state);

      // Take the mutex for writing to current measurement
     if( xSemaphoreTake( xCurrentMeas_mutex, (1000L * configTICK_RATE_HZ) / 1000L ) == pdTRUE )
    { 

         dCurrentMeas.ts = micros();
         dCurrentMeas.cnt  ++;
         dCurrentMeas.status = reg_status;
         dCurrentMeas.error = reg_error;

         if(xQueueSend( xQueue,     /* The queue being written to. */
                &dCurrentMeas, /* The address of the data being sent. */
                0UL ) != pdPASS)
                {
                  reg_error |= ERROR_QUEUE_NET; 
                }
                
          if (xStreamBufferSend(xStreamBuffer,&dCurrentMeas,sizeof( sMeasurement_t ),0)!= pdPASS)
          {
              reg_error |= ERROR_QUEUE_SD; 
            
          }
                


        
        xSemaphoreGive( xCurrentMeas_mutex );
    }
    else
    {
        reg_error |= ERROR_MUTEX;
    }

    
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
		
		Serial.print("status::");
		Serial.print("queue_sd");Serial.print(sd_full); Serial.print("/"); Serial.println(sd_free); 

    size_t net_free =uxQueueSpacesAvailable(xQueue);
    size_t net_full =uxQueueMessagesWaiting(xQueue);

    Serial.print("status::");
    Serial.print("queue_net");Serial.print(net_full); Serial.print("/"); Serial.println(net_free); 

    Serial.print("status::ram"); Serial.println(xPortGetFreeHeapSize());

    Serial.print("status::info"); Serial.print(reg_status,BIN);Serial.print("| error"); Serial.print(reg_error,BIN);

    /* Debug functions  */
		vTaskGetRunTimeStats(bufptr1);  
		Serial.write(bufptr1);
		
		
		// Sleep for 100 milliseconds.
		vTaskDelay((1000L * configTICK_RATE_HZ) / 1000L);
	}
}





//=====================================================================================                                  
//=================THREAD============HX711=============================================
//=====================================================================================  
#include <HX711_ADC.h>

static void pLoadcell(void* arg) {
	
	//HX711 constructor:
	HX711_ADC LoadCell_1(36, 37);
	//HX711 constructor:
	HX711_ADC LoadCell_2(38, 39);
	//HX711 constructor:
	HX711_ADC LoadCell_3(40, 41);
	
	LoadCell_1.begin();
	LoadCell_2.begin();
	LoadCell_3.begin();
	
	unsigned long stabilizingtime = 2000; // tare preciscion can be improved by adding a few seconds of stabilizing time
	boolean _tare = true; //set this to false if you don't want tare to be performed in the next step
	byte loadcell_1_rdy = 0;
	byte loadcell_2_rdy = 0;
	byte loadcell_3_rdy = 0;
	while ((loadcell_1_rdy + loadcell_2_rdy+ loadcell_3_rdy) < 3) { //run startup, stabilization and tare, both modules simultaniously
		if (!loadcell_1_rdy) loadcell_1_rdy = LoadCell_1.startMultiple(stabilizingtime, _tare);
		if (!loadcell_2_rdy) loadcell_2_rdy = LoadCell_2.startMultiple(stabilizingtime, _tare);
		if (!loadcell_3_rdy) loadcell_3_rdy = LoadCell_3.startMultiple(stabilizingtime, _tare);
	}
	if (LoadCell_1.getTareTimeoutFlag()) {
		Serial.println("hx711::Timeout MCU>HX711 #1");
	}
	if (LoadCell_2.getTareTimeoutFlag()) {
		Serial.println("hx711::Timeout MCU>HX711 #2");
	}
	if (LoadCell_3.getTareTimeoutFlag()) {
		Serial.println("hx711::Timout MCU>HX711 #3");
	}
	Serial.println("hx711::init complete");
	
	
	
	const TickType_t taskPeriod = 20; // 20ms <--> 50Hz
	TickType_t xLastWakeTime = xTaskGetTickCount();
	byte newDataReady = false;
	for(;;){
		
		vTaskDelayUntil(&xLastWakeTime, taskPeriod);
		
		if (LoadCell_1.update()) newDataReady = true;
		LoadCell_2.update();
		LoadCell_3.update();
		
		if(newDataReady){
			float hx711_1 = LoadCell_1.getData();
			float hx711_2 = LoadCell_2.getData();
			float hx711_3 = LoadCell_3.getData();




        // Take the mutex for writing to current measurement
         if( xSemaphoreTake( xCurrentMeas_mutex, (1000L * configTICK_RATE_HZ) / 1000L ) == pdTRUE )
        { 
            dCurrentMeas.strain[0] =hx711_1;
            dCurrentMeas.strain[1] =hx711_2;
            dCurrentMeas.strain[2] =hx711_3;
            dCurrentMeas.strain_ts=millis();
            
            xSemaphoreGive( xCurrentMeas_mutex );
        }
        else
        {
            reg_error |= ERROR_MUTEX;
        }


      
		} // new data ready
		
	}//for
	
}


//=====================================================================================                                  
//=================THREAD============ADXL355 =========================================
//=====================================================================================  
#include <SPI.h>

// Memory register addresses:
const int XDATA3 = 0x08;
const int XDATA2 = 0x09;
const int XDATA1 = 0x0A;
const int YDATA3 = 0x0B;
const int YDATA2 = 0x0C;
const int YDATA1 = 0x0D;
const int ZDATA3 = 0x0E;
const int ZDATA2 = 0x0F;
const int ZDATA1 = 0x10;
const int RANGE = 0x2C;
const int POWER_CTL = 0x2D;
const int ODR = 0x28;

// Device values
const int RANGE_2G = 0x01;
const int RANGE_4G = 0x02;
const int RANGE_8G = 0x03;
const int MEASURE_MODE = 0x06; // Only accelerometer
const int ODR_125HZ = 0b0101;

// Operations
const int READ_BYTE = 0x01;
const int WRITE_BYTE = 0x00;

// Pins used for the connection with the sensor
const int CHIP_SELECT_PIN = 7;

/* 
	* Write registry in specific device address
*/
void spi_writeRegister(byte thisRegister, byte thisValue) {
	byte dataToSend = (thisRegister << 1) | WRITE_BYTE;
	digitalWrite(CHIP_SELECT_PIN, LOW);
	SPI.transfer(dataToSend);
	SPI.transfer(thisValue);
	digitalWrite(CHIP_SELECT_PIN, HIGH);
}

/* 
	* Read registry in specific device address
*/
unsigned int spi_readRegistry(byte thisRegister) {
	unsigned int result = 0;
	byte dataToSend = (thisRegister << 1) | READ_BYTE;
	
	digitalWrite(CHIP_SELECT_PIN, LOW);
	SPI.transfer(dataToSend);
	result = SPI.transfer(0x00);
	digitalWrite(CHIP_SELECT_PIN, HIGH);
	return result;
}

/* 
	* Read multiple registries
*/
void spi_readMultipleData(int *addresses, int dataSize, int *readedData) {
	digitalWrite(CHIP_SELECT_PIN, LOW);
	for(int i = 0; i < dataSize; i = i + 1) {
		byte dataToSend = (addresses[i] << 1) | READ_BYTE;
		SPI.transfer(dataToSend);
		readedData[i] = SPI.transfer(0x00);
	}
	digitalWrite(CHIP_SELECT_PIN, HIGH);
}


static void pAccelerometer(void* arg) {
	
	SPI.begin();
	
	// Initalize the  data ready and chip select pins:
	pinMode(CHIP_SELECT_PIN, OUTPUT);
	
	//Configure ADXL355:
	spi_writeRegister(RANGE, RANGE_2G); // 2G
	spi_writeRegister(POWER_CTL, MEASURE_MODE); // Enable measure mode
	spi_writeRegister(ODR, ODR_125HZ); // 125 HZ, no Highpass
	
	// Give the sensor time to set up:
	delay(100);
	Serial.println("acc::Startup is complete");
	
	
	
	const TickType_t taskPeriod = 20; // 20ms <--> 50Hz
	
	TickType_t xLastWakeTime = xTaskGetTickCount();
	for(;;){
		
		vTaskDelayUntil(&xLastWakeTime, taskPeriod);
		
		int axisAddresses[] = {XDATA1, XDATA2, XDATA3, YDATA1, YDATA2, YDATA3, ZDATA1, ZDATA2, ZDATA3};
		int axisMeasures[] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
		int dataSize = 9;
		
		// Read accelerometer data
		spi_readMultipleData(axisAddresses, dataSize, axisMeasures);
		
		// Split data
		int xdata = (axisMeasures[0] >> 4) + (axisMeasures[1] << 4) + (axisMeasures[2] << 12);
		int ydata = (axisMeasures[3] >> 4) + (axisMeasures[4] << 4) + (axisMeasures[5] << 12);
		int zdata = (axisMeasures[6] >> 4) + (axisMeasures[7] << 4) + (axisMeasures[8] << 12);
		
		// Apply two complement
		if (xdata >= 0x80000) {
			xdata = ~xdata + 1;
		}
		if (ydata >= 0x80000) {
			ydata = ~ydata + 1;
		}
		if (zdata >= 0x80000) {
			zdata = ~zdata + 1;
		}

    // Take the mutex for writing to current measurement
         if( xSemaphoreTake( xCurrentMeas_mutex, (1000L * configTICK_RATE_HZ) / 1000L ) == pdTRUE )
        { 
            dCurrentMeas.acc[0] =xdata;
            dCurrentMeas.acc[1] =ydata;
            dCurrentMeas.acc[2] =zdata;
            dCurrentMeas.acc_ts=millis();
            
            xSemaphoreGive( xCurrentMeas_mutex );
        }
        else
        {
            reg_error |= ERROR_MUTEX;
        }
		
		
	}//for


       
	
}






//=====================================================================================                                  
//=================THREAD============ADC on SPI0 ==================================
//=====================================================================================  


#define ADS_SPISPEED 1250000

#define ADS_RST_PIN    8 //ADS1256 reset pin
#define ADS_RDY_PIN    9 //ADS1256 data ready
#define ADS_CS_PIN    10 //ADS1256 chip select




static void pADC_ext(void* arg) {
	pinMode(ADS_CS_PIN, OUTPUT);
	
	pinMode(ADS_RDY_PIN, INPUT);
	pinMode(ADS_RST_PIN, OUTPUT);
	digitalWrite(ADS_RST_PIN, LOW);
	delay(1); // LOW at least 4 clock cycles of onboard clock. 100 microsecons is enough
	digitalWrite(ADS_RST_PIN, HIGH); // now reset to deafult values
	
	vTaskDelay((150L * configTICK_RATE_HZ) / 1000L);
	
	digitalWrite(ADS_CS_PIN, LOW); // select ADS
	delayMicroseconds(50);
	while (digitalRead(ADS_RDY_PIN)) {
		// Sleep for 100 milliseconds.
		vTaskDelay((100L * configTICK_RATE_HZ) / 1000L);
	}  // wait for ready_line to go low
	
	SPI.beginTransaction(SPISettings(ADS_SPISPEED, MSBFIRST, SPI_MODE1));
	delayMicroseconds(10);
	
	//Reset to Power-Up Values (FEh)
	SPI.transfer(0xFE);
	delayMicroseconds(100);
	
	byte status_reg = 0 ;  // address (datasheet p. 30)
	byte status_data = 0x01; //status: Most Significant Bit First, Auto-Calibration Disabled, Analog Input Buffer Disabled
	//0x03; //to activate buffer
	SPI.transfer(0x50 | status_reg);
	SPI.transfer(0x00);   // 2nd command byte, write one register only
	SPI.transfer(status_data);   // write the databyte to the register
	delayMicroseconds(10);
	
	//PGA SETTING
	//1 ±5V        000 (1)
	//2 ±2.5V      001 (2)
	//4 ±1.25V     010 (3)
	//8 ±0.625V    011 (4)
	//16 ±312.5mV  100 (5)
	//32 ±156.25mV 101 (6)
	//64 ±78.125mV 110 (7) OR 111 (8)
	byte adcon_reg = 2; //A/D Control Register (Address 02h)
	byte adcon_data = 0x20; // 0 01 00 000 => Clock Out Frequency = fCLKIN, Sensor Detect OFF, gain 1
	//0x25 for setting gain to 32, 0x27 to 64
	SPI.transfer(0x50 | adcon_reg);
	SPI.transfer(0x00);   // 2nd command byte, write one register only
	SPI.transfer(adcon_data);   // write the databyte to the register
	delayMicroseconds(10);
	
	//Set sampling rate
	byte drate_reg = 3; // Choosing Data Rate register = third register.
	byte drate_data = 0b10000010; // 100 SPS
	SPI.transfer(0x50 | drate_reg);
	SPI.transfer(0x00);   // 2nd command byte, write one register only
	SPI.transfer(drate_data);   // write the databyte to the register
	delayMicroseconds(10);
	
	//done with settings, can close SPI transaction now
	digitalWrite(ADS_CS_PIN, HIGH); //unselect ADS
	delayMicroseconds(50);
	
	Serial.println("adc_ext:: init finished.");
	
	const TickType_t taskPeriod = 20; // 20ms <--> 50Hz
	
	TickType_t xLastWakeTime = xTaskGetTickCount();
	long adc_val = 0; // unsigned long is on 32 bits
	unsigned long adc_lasttime = 0;
	for(;;){
		
		vTaskDelayUntil(&xLastWakeTime, taskPeriod);
		
		
		
		digitalWrite(ADS_CS_PIN, LOW);
		adc_lasttime = millis();
		
		delayMicroseconds(50);
		SPI.beginTransaction(SPISettings(ADS_SPISPEED, MSBFIRST, SPI_MODE1));
		delayMicroseconds(10);
		
		while (digitalRead(ADS_RDY_PIN)) {vTaskDelay((5L * configTICK_RATE_HZ) / 1000L);} ; // delay single ticks while waiting
		
		byte adc_currentChannel = 0;
		byte data = (adc_currentChannel << 4) | adc_currentChannel+1; //xxxx1000 - AINp = positiveCh, AINn = negativeCh
		SPI.transfer(0x50 | 1); // write (0x50) MUX register (0x01)
		SPI.transfer(0x00);   // number of registers to be read/written − 1, write one register only
		SPI.transfer(data);   // write the databyte to the register
		delayMicroseconds(10);
		
		//SYNC command 1111 1100
		SPI.transfer(0xFC);
		delayMicroseconds(10);
		
		//WAKEUP 0000 0000
		SPI.transfer(0x00);
		delayMicroseconds(10);
		
		SPI.transfer(0x01); // Read Data 0000  0001 (01h)
		delayMicroseconds(10);
		
		adc_val = SPI.transfer(0);
		adc_val <<= 8; //shift to left
		adc_val |= SPI.transfer(0);
		adc_val <<= 8;
		adc_val |= SPI.transfer(0);
		
		delayMicroseconds(10);
		
		digitalWrite(ADS_CS_PIN, HIGH);
		delayMicroseconds(50);
		
		if (adc_val > 0x7fffff) { //if MSB == 1
			adc_val = adc_val - 16777216; //do 2's complement, keep the sign this time!
		}

      // Take the mutex for writing to current measurement
         if( xSemaphoreTake( xCurrentMeas_mutex, (1000L * configTICK_RATE_HZ) / 1000L ) == pdTRUE )
        { 
            dCurrentMeas.adc[0] =adc_val;
            dCurrentMeas.adc_ts[0]=adc_lasttime;
            
            xSemaphoreGive( xCurrentMeas_mutex );
        }
        else
        {
            reg_error |= ERROR_MUTEX;
        }
     
		
		
	}//for
	
	
}  // static void pADC_ext(void* arg)



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



//=====================================================================================                                  
//=================THREAD============ETHERNET on TEENSY================================
//===================================================================================== 


#include <Ethernet.h>


// Enter the IP address of the server you're connecting to:
IPAddress server(192, 168, 1, 100);

EthernetClient client;
static unsigned int port_eth = 5080;

boolean alreadyConnected = false; // whether or not the client was connected previously
byte mac_eth[] = {
  0x00, 0xAA, 0x00, 0x00, 0x00, 0x02
};

static void pEthernet(void* arg) {

// initialize the ethernet device
  if (Ethernet.begin(mac_eth) == 0) {
    Serial.println("eth::Failed to configure Ethernet using DHCP");
    if (Ethernet.hardwareStatus() == EthernetNoHardware) {
      Serial.println("eth::Ethernet shield was not found.  Sorry, can't run without hardware. :(");
    } else if (Ethernet.linkStatus() == LinkOFF) {
      Serial.println("eth::Ethernet cable is not connected.");
    }
    // no point in carrying on, so do nothing forevermore:
    /*
    while (true) {
      vTaskDelay(portMAX_DELAY);
    }*/
  }
  // print your local IP address:
  Serial.print("eth:: ip: ");
  Serial.println(Ethernet.localIP());

if(client.connected()){
  

}else{ // not connected to SERVER
    if (client.connect(server, port_eth)) {
    Serial.println("eth:connected");
  } else {
    // if you didn't get a connection to the server:
    Serial.println("eth:connection failed");
  }
} // if(client.connected())

  for(;;) {

    byte tmp_meas;
    if (xQueueReceive(xQueue,&tmp_meas,10) == pdPASS)
      {
        Serial.print("eth::queue rx");
        if(client.connected()){
          reg_error = reg_error & ~(1 << ERROR_ETH);  //clear error bit
          client.write(tmp_meas);
        }else{
          reg_error = reg_error & ~(1 << ERROR_ETH) | (1 << ERROR_ETH); //set error bit
        }
      }

      /* Check for incoming command */
      if (client.available()) {
            char c = client.read();
            Serial.println("eth:: rx command:");
            Serial.print(c);
            }
      } // for(;;)
      
   
  } //thread pETH

//=====================================================================================                                  
//==INTERRUPT=====================Stepper control======================================
//=====================================================================================  


#include <AccelStepper.h>
// Create an IntervalTimer object 
IntervalTimer stepperTimer;

AccelStepper stepper_grabx1(1, 20, 21); // pin 3 = step, pin 6 = direction
AccelStepper stepper_grabx2(1, 22, 23); 
AccelStepper stepper_graby1(1, 24, 25); 
AccelStepper stepper_graby2(1, 26, 27); 


void setup_stepper() {
  stepper_grabx1.setMaxSpeed(400);
  stepper_grabx2.setMaxSpeed(400);
  stepperTimer.begin(run_steppers, 100);  // run every 100 us
}

int ledState = LOW;

void run_steppers(){
  stepper_grabx1.runSpeed();
  stepper_grabx2.runSpeed();


  if (ledState == LOW) {
    ledState = HIGH;
  } else {
    ledState = LOW;
  }
  digitalWrite(PIN_INFO2, ledState);
}
}




//=====================================================================================                                  
//==================================Initialisation=====================================
//=====================================================================================  
void setup() {
	
	// Initialize some randomness
	randomSeed(analogRead(0));
	
	Serial.begin(115200);
	delay(1000);
	Serial.println("main::TEENSY IRTEBE firmware");
	
	
	Serial.print("main::Kernel version: ");
	//Serial.println(tskKERNELVERSIONNUMBER);
	
	Serial.print("main::CPU Clock: ");
	Serial.println(configCPU_CLOCK_HZ); 
	
	Serial.print("main::Tick rate: ");
	Serial.println(configTICK_RATE_HZ);

  Serial.print("main::Free heap size ");
  Serial.println(xPortGetFreeHeapSize());
 
	
	
	 /* Create a mutex type semaphore for current measurement. */
   xCurrentMeas_mutex = xSemaphoreCreateMutex();

  if( xCurrentMeas_mutex == NULL )
    {
      reg_error |= ERROR_MUTEX;
      Serial.print("main::Mutex error.");
    }
	
	portBASE_TYPE s1, s2,s3,s4,s5;
	
	
	
	//get information on how many times arduino was started. this will reflect in the filename
	#ifdef TEENSY_CORE
		run_id = EEPROM.read(EEPROM_ADDR);
		EEPROM.write(EEPROM_ADDR, run_id+1);
	#endif
	
	
	
	
	xStreamBuffer = xStreamBufferCreate(SD_STREAMBUFFER_SIZE,	SD_TRIGGERLEVEL);
    if( xStreamBuffer == NULL )
    {
       reg_error |= ERROR_QUEUE_SD; 
    }

    xQueue = xQueueCreate( QUEUE_LENGTH,
                                 ITEM_SIZE);

    if( xStreamBuffer == NULL )
    {
       reg_error |= ERROR_QUEUE_SD; 
    }
                                 
    configASSERT( xQueue );
    configASSERT( xStreamBuffer );


 
	
	// initialize semaphore
	sem = xSemaphoreCreateCounting(1, 0);
	
	s1 = xTaskCreate(Thread1, "LED1", configMINIMAL_STACK_SIZE, NULL, 2, NULL);
	s2 = xTaskCreate(Thread2, "LED2", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
	
	s3 = xTaskCreate(pHousekeeping, "HK", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY+1, NULL);
	s4 = xTaskCreate(pSdWrite, "SD", configMINIMAL_STACK_SIZE+1000, NULL, tskIDLE_PRIORITY+1, NULL);
  s5 = xTaskCreate(pMeasurement, "MEAS", configMINIMAL_STACK_SIZE+1000, NULL, tskIDLE_PRIORITY+3, NULL);  
	
	
	#if defined(_SAMD21_)
		s5 = xTaskCreate(pWifi, "WIFI", configMINIMAL_STACK_SIZE+100, NULL, tskIDLE_PRIORITY+1, NULL);
	#endif
	
	// check for creation errors
	if (sem== NULL || s1 != pdPASS || s2 != pdPASS || s3 != pdPASS || s4 != pdPASS || s5 != pdPASS) {
		Serial.println("main::Creation problem");
	}
	
	Serial.println("main::Starting the scheduler !");
	  Serial.print("main::Free heap size ");
  Serial.println(xPortGetFreeHeapSize());
	
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
