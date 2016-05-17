
//////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                              //
//                                      SMART GARDEN PROJECT                                    //
//                                                                                              //
//                           for Pervasive Systems 2016 @ DIAG Sapienza                         //
//                                http://persys2016-diag.slack.com/                             //
//                                                                                              //
//                                 Stefano Coratti & Massimo Perri                              //
//                                                                                              //
//                                                                                              //
// MASTER program, target board: Arduino Mega 2560                                              //
//////////////////////////////////////////////////////////////////////////////////////////////////
//#define BLYNK_NO_INFO
//#define BLYNK_DEBUG
#include <BlynkSimpleSerial.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>
#include <RHReliableDatagram.h>
#include <RH_NRF24.h>
#include <SPI.h>
#include <TimeLib.h>
#include <WidgetRTC.h>



/* auth token */
char auth[ ] = "6a0cd1a204764d21a1930c8a3476c86d";

//boolean online_alerts = false; //if true online alerts via email by PushingBox will be enabled
//boolean online_logging = false; //if true ThinkSpeak logging will be enabled
boolean online_alerts = true; //if true online alerts via email by PushingBox will be enabled
boolean online_logging = true; //if true ThinkSpeak logging will be enabled

//temp & pressure sensor BMP180
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);

//DHT111
#include <dht.h>
dht DHT;
#define DHT11_PIN 4
       
#define BLYNK_PRINT Serial

WidgetRTC rtc;
BLYNK_ATTACH_WIDGET(rtc, V5);

#define TIME_HEADER  "T"   
#define TIME_REQUEST  7    

#define SERVER_ADDRESS 254
// Singleton instance of the radio driver
//RH_NRF24 driver;
// RH_NRF24 driver(8, 7);   // For RFM73 on Anarduino Mini
RH_NRF24 driver(48,49);
// Class to manage message delivery and receipt, using the driver declared above
RHReliableDatagram manager(driver, SERVER_ADDRESS);

int RECV_INTERVALS = 5; //20 receive intervals -> 20 seconds of rx
int IDLE_DELAY = 15000; //the delay between web/process and next loop in milliseconds

/* override demo values */
float  DEMO_TEMP = 46.0;
int DEMO_HUMIDITY = 10; 
boolean override_flag = false;

long debouncing_time = 15; //Debouncing Time in Milliseconds
volatile unsigned long last_micros;


//////////////// WIFI ESP8266 //////////////////

#include "ESP8266.h"

#define SSID        "TIM_ID445"
#define PASSWORD    "!@Taipei101$$"
#define HOST_NAME   "api.thingspeak.com"
#define HOST_PORT   (80)

ESP8266 wifi(Serial2,115200);

void ESP8266setup(void)
{
    Serial1.print("setup begin\r\n");

    Serial1.print("FW Version:");
    Serial1.println(wifi.getVersion().c_str());

    if (wifi.setOprToStationSoftAP()) {
        Serial1.print("to station + softap ok\r\n");
    } else {
        Serial1.print("to station + softap err\r\n");
    }

    if (wifi.joinAP(SSID, PASSWORD)) {
        Serial1.print("Join AP success\r\n");

        Serial1.print("IP:");
        Serial1.println( wifi.getLocalIP().c_str());       
    } else {
        Serial1.print("Join AP failure\r\n");
    }

    for(int x = 0; x<10; x++) //retry 10 times before giving up
    if (wifi.disableMUX()) {
        Serial1.print("single ok\r\n");
        x=10; //exit
    } else {
        Serial1.print("single err\r\n");
    }
    
    Serial1.print("ESP8266 setup end\r\n");
}

void sendNotification()
{
  uint8_t buffer[1024] = {0};

    for(int x = 0; x<10; x++) //retry 10 times before giving up
    if (wifi.createTCP(HOST_NAME, HOST_PORT)) {
        Serial1.print("create tcp ok\r\n");
        x=10; //exit
    } else {
        Serial1.print("create tcp err\r\n");
    }

    delay(500);

    //char *hello = "GET / HTTP/1.1\r\nHost: api.thingspeak.com/update?api_key=J2UBCQ8WHN4ISS1M&field8=0\r\nConnection: close\r\n\r\n";

    char *hello = "GET /update?api_key=J2UBCQ8WHN4ISS1M&field8=0\r\n\r\n"; 

    //char *hello = "GET / HTTP/1.1\r\nHost: www.google.com\r\nConnection: close\r\n\r\n";
    
    wifi.send((const uint8_t*)hello, strlen(hello));

    uint32_t len = wifi.recv(buffer, sizeof(buffer), 20000);
    if (len > 0) {
        Serial1.print("Received:[");
        for(uint32_t i = 0; i < len; i++) {
            Serial1.print((char)buffer[i]);
        }
        Serial1.print("]\r\n");
    }

    /*
    delay(100);

    if (wifi.releaseTCP()) {
        Serial1.print("release tcp ok\r\n");
    } else {
        Serial1.print("release tcp err\r\n");
    }
    */
}

void sendThingSpeakUpdate(String s)
{
  uint8_t buffer[1024] = {0};

    //for(int x = 0; x<10; x++) //retry 10 times before giving up

    delay(500);
    if (wifi.createTCP(HOST_NAME, HOST_PORT)) {
        Serial1.print("create tcp ok\r\n");
        //x=10; //exit
    } else {
        Serial1.print("create tcp err\r\n");
    }

    delay(500);

    char * hello = new char[s.length() + 1];

    strcpy(hello,s.c_str());

    wifi.send((const uint8_t*)hello, strlen(hello));

    
    uint32_t len = wifi.recv(buffer, sizeof(buffer), 2000);
    if (len > 0) {
        Serial1.print("Received:[");
        for(uint32_t i = 0; i < len; i++) {
            Serial1.print((char)buffer[i]);
        }
        Serial1.print("]\r\n");
    }
    

    /*
    delay(300);

    if (wifi.releaseTCP()) {
        Serial1.print("release tcp ok\r\n");
    } else {
        Serial1.print("release tcp err\r\n");
    }
    */
}


/* data structures */

struct dataStruct{
  time_t timestamp;

  float press_atm;
  float temp;
  byte humidity;
  int light_intensity;

  /* dafault config values for the three plants */
  int s_1_humidity_th = 200;  //max th above of this value no water will be provided (to prevent mould and root problems)
  int s_1_max_cm3_week = 400; //max water volume in the week (cm3)

  int s_2_humidity_th = 300;
  int s_2_max_cm3_week = 800;

  int s_3_humidity_th = 500;
  int s_3_max_cm3_week = 1200;

  boolean override_s = false;

}myData;

byte tx_buf[sizeof(myData)] = {0};

struct dataSlave_1{
  time_t timestamp;
  
  int soil_humidity = 0;
}slaveData_1;

struct dataSlave_2{
  time_t timestamp;
  
  int soil_humidity = 0;
}slaveData_2;

struct dataSlave_3{
  time_t timestamp;
  
  int soil_humidity = 0;
}slaveData_3;


//Lux calculation routine
double Light (int RawADC0)
{
  double Vout=RawADC0*0.0048828125;
  //int lux=500/(10*((5-Vout)/Vout));//use this equation if the LDR is in the upper part of the divider
  int lux=(2500/Vout-500)/10;
  return lux;
}

/* read weather data and store them in local data structure */
void readWeatherData()
{
   ///read sensors
    /* Get a new sensor event */ 
      sensors_event_t event;
      bmp.getEvent(&event);
     
      /* Display the results (barometric pressure is measure in hPa) */
      if (event.pressure)
      {

        float pressure_atm = 0.0;
        bmp.getPressure(&pressure_atm); 
        pressure_atm/=1000;  //convert the value to kPa
        myData.press_atm=pressure_atm;
        /* Display atmospheric pressue in hPa */
        /*
        Serial1.print("Pressure:    ");
        Serial1.print(pressure_atm);
        Serial1.println(" hPa");
        */
        
        /* First we get the current temperature from the BMP085 */
        float temperature;
        bmp.getTemperature(&temperature); //in °C
        myData.temp = temperature;
        /*
        Serial1.print("Temperature: ");
        Serial1.print(temperature);
        Serial1.println(" C");
        */

        //read air relative humidity
        int chk = DHT.read11(DHT11_PIN);
        myData.humidity = DHT.humidity;
        
        //read light intensity, pin A0 for the LDR and covert to lux
        myData.light_intensity = (int)(Light(analogRead(0)));     
      }
      else
      {
        //Serial1.println("Sensor error");
      }

}

/* put values on Blynk */
void pushWeatherData()
{    
        Blynk.virtualWrite(1, myData.press_atm);               
        Blynk.virtualWrite(2, myData.temp);
        Blynk.virtualWrite(3, myData.humidity);
        Blynk.virtualWrite(4, myData.light_intensity);
}

/* push slaves data to Blynk for e.g. display */
void pushSlavesData()
{
 Blynk.virtualWrite(6, slaveData_1.soil_humidity); 
 Blynk.virtualWrite(7, slaveData_2.soil_humidity); 
 Blynk.virtualWrite(8, slaveData_3.soil_humidity); 
}

/* process data and update web info */
void processAndSendData()
{
  //push data to Blink..
  pushWeatherData();
  //update slaves data and push to Blink
  pushSlavesData();
}

boolean boo = true;


/* handle override pin change */
void checkPin()
{
  // Invert state, since button is "Active LOW"
  if (!digitalRead(2))
  {
    boo=!boo;
    override_flag = boo;
    
    if(override_flag)
    digitalWrite(13, HIGH); else digitalWrite(13, LOW);
  }
}

void debounceInterrupt() {
  if((long)(micros() - last_micros) >= debouncing_time * 1000) {
    checkPin();
    last_micros = micros();
  }
}

////TIME

void digitalClockDisplay(){
  // digital clock display of the time
  Serial1.print(hour());
  printDigits(minute());
  printDigits(second());
  Serial1.print(" ");
  Serial1.print(day());
  Serial1.print(" ");
  Serial1.print(month());
  Serial1.print(" ");
  Serial1.print(year()); 
  Serial1.println(); 
}

void printDigits(int digits){
  // utility function for digital clock display: prints preceding colon and leading 0
  Serial1.print(":");
  if(digits < 10)
    Serial1.print('0');
  Serial1.print(digits);
}


void processSyncMessage() {
  unsigned long pctime;
  const unsigned long DEFAULT_TIME = 1462663749; // 7 maggio 2016

  if(Serial1.find(TIME_HEADER)) {
     pctime = Serial1.parseInt();
     if( pctime >= DEFAULT_TIME) { // check the integer is a valid time (greater than Jan 1 2013)
       setTime(pctime); // Sync Arduino clock to the time received on the serial port
     }
  }
}

time_t requestSync()
{
  Serial1.write(TIME_REQUEST);  
  return 0; // the time will be sent later in response to serial mesg
}

///





// the setup function runs once when you press reset or power the board
void setup() {
    Serial1.begin( 115200 );  //for debug/logging

    Serial1.println("MASTER ESP8266 WIFI config init..");

    ESP8266setup();

    //now procede on
    
    override_flag = false; 
    
    if (!manager.init()) //init the manager.. and check
    {
     //Serial1.println("nRF24L01 init failed");
    }
    /* A0 as analog input for the LDR */
    pinMode(A0, INPUT);

    //override led, on if active, pin 13
    pinMode(13, OUTPUT);
    
    /* override PIN handling */
    // Make pin 2 HIGH by default
    pinMode(2, INPUT_PULLUP);
    // Attach INT to our handler
    attachInterrupt(digitalPinToInterrupt(2), debounceInterrupt, FALLING);

    /* Initialise the BMP180 sensor */
      if(!bmp.begin())
      {
        /* There was a problem detecting the BMP085 ... check your connections */
        Serial1.print("Ooops, no BMP085 detected ... Check your wiring or I2C ADDR!");
        while(1);
      }
      
    
    Serial.begin( 115200 );
    Blynk.begin( auth,115200); 

    while (Blynk.connect() == false) {
      // Wait until connected
    }

   // Begin synchronizing time using RTC
   rtc.begin();
}


// the loop function runs over and over again until power down or reset
void loop() 
{
  Serial1.println("MASTER started: waiting for timestamp");
  while (timeStatus()== timeNotSet)  //wait until time is set..
  {
    Blynk.run();
  }
  Serial1.println("got timestamp..");
  //read weather data, store into data structure
  readWeatherData();
  
  //set actual timestamp in data structure and send broadcast with weather data, config and override
  if (timeStatus()!= timeNotSet) //here must be true
  { 
        //set timestamp
        myData.timestamp = now();

        Serial1.print("timestamp: ");
        Serial1.println(myData.timestamp);
        /* OVERRIDE HANDLING */
        if(override_flag) //give demo data and set override_s to true
        {
          myData.temp = DEMO_TEMP;
          myData.humidity = DEMO_HUMIDITY;
          myData.override_s = true;

         //override set, send notification via email by PushBox
         if(online_alerts) sendNotification();
        }
        else
           {
            myData.override_s = false; 
           }
        Serial1.print("override_s :");   
        Serial1.println(myData.override_s);  
        
        memcpy(tx_buf, &myData, sizeof(myData) );
        byte zize=sizeof(myData);
  
        //sending BC
        Serial1.println("");
        Serial1.println("sending SYNC broadcast..\n");
        //send SYNC broadcast
        manager.sendtoWait((uint8_t *)tx_buf, zize, 0xFF);

       
        /*
        if (manager.available())
        {
          if (!manager.sendtoWait((uint8_t *)tx_buf, zize, 0xFF))
              Serial1.println("sendtoWait failed");
          //reset override_flag
          
        }
        */

  }
  
  ///then it looks for msgs from slaves
  uint8_t buf[RH_NRF24_MAX_MESSAGE_LEN];
  uint8_t buflen = sizeof(buf);
  uint8_t from;

  
  for(int t = 0; t<RECV_INTERVALS; t++) //receive for RECV_INTERVALS times
  {
  if (manager.recvfromAckTimeout(buf, &buflen, 1000, &from))
  { 
        // Message with a good checksum received, dump it.
        Serial1.println("");
        Serial1.print("Got msg from slave addr: ");
        Serial1.print(from,DEC); 
        Serial1.print(" ");
        Serial1.print("msg size : ");
        Serial1.print(buflen, DEC);

        //copy data in the right data structure..
        if(from == 1) //slave 1
        {
         memcpy(&slaveData_1, buf, sizeof(slaveData_1));
         Serial1.println("");
       
         Serial1.print("timestamp: ");
         Serial1.print(slaveData_1.timestamp);

         Serial1.print(" soil_humidity: ");
         Serial1.print(slaveData_1.soil_humidity);
        }
       if(from == 2) //slave 2
        {
         memcpy(&slaveData_2, buf, sizeof(slaveData_2));
         Serial1.println("");
       
         Serial1.print("timestamp: ");
         Serial1.print(slaveData_2.timestamp);

         Serial1.print(" soil_humidity: ");
         Serial1.print(slaveData_2.soil_humidity);
        } 
        if(from == 3) //slave 3
        {
         memcpy(&slaveData_3, buf, sizeof(slaveData_3));
         Serial1.println("");
       
         Serial1.print("timestamp: ");
         Serial1.print(slaveData_3.timestamp);

         Serial1.print(" soil_humidity: ");
         Serial1.print(slaveData_3.soil_humidity);
        } 
   
   }
  }//for
  

  Serial1.println("");
  Serial1.println("process and send data..\n");
  //process data / send to the web
  //push them to Blynk
  processAndSendData();

  //push data to ThingSpeak for temporal dynamics graphs
  /*
  float press_atm;
  float temp;
  byte humidity;
  int light_intensity;
  
  int soil_humidity
  */
  String str = "GET /update?api_key=J2UBCQ8WHN4ISS1M&field1="+(String)myData.press_atm+"&field2="+(String)myData.temp+"&field3="+(String)myData.humidity+"&field4="+(String)myData.light_intensity+"&field5="+(String)slaveData_1.soil_humidity+"&field6="+(String)slaveData_2.soil_humidity+"&field7="+(String)slaveData_3.soil_humidity+"\r\n\r\n";
  
  if(online_logging) sendThingSpeakUpdate(str);

  Serial1.println("");
  Serial1.println("idle..\n");
  //wait in idle mode
  delay(IDLE_DELAY);

    
    
  Blynk.run();
}

