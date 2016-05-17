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
// SLAVE3 program , target board: custom  ATMEGA328P board (Arduino standalone @16Mhz           //
//////////////////////////////////////////////////////////////////////////////////////////////////
#include <RHReliableDatagram.h>
#include <RH_NRF24.h>
#include <SPI.h>
#include <TimeLib.h>
#include <stdlib.h>
#include "Arduino.h"

#define TIME_HEADER  "T"   
#define TIME_REQUEST  7 

/* slave specific values */
int SLAVE_ADDRESS = 3;
int DEMO_WATER_VOLUME = 30; //volume of water for the demo/override expressed in cm3
int DEMO_SLAVE_DELAY = 35000; //30 seconds

//plant specific
float CROP_COEFF = 0.9; //crop coefficient


/***************************************************/

/* common reference values  */
int PUMP_PIN = 8;
int TX433_PIN = 10;
int RX433_PIN = 9; 
float pump_cm3_s = 8.33; //water punp efficiency is 8.33 cm3/s in steady state
float cm3_to_time = 0.12; //conversion factor from cm3 to time (seconds) -> 8.33 * 0.12 = 1 sec ; 120 ms -> 1cm3
int LIGHT_TH = 500; // 1000 lux of light intensity is the threshold, watering must start only under this threshold
time_t previous_timestamp;
float previous_press_atm;

int SERVER_ADDRESS = 254;
int SDI = 1000; //slave TX delay in milliseconds from the reference SYNC msg reception

float ET0[13]={0.0, 0.6, 1.1, 2.0, 3.0, 4.4, 5.3, 5.5, 4.8, 3.1, 1.8, 0.8, 0.5}; //ET0 values for the months
float PLANTING_DENSITY = 1.0;
float EXPOSURE_FACTOR = 1.0;
float IRRIGATION_EFFICIENCY = 0.9;
float PLANTED_AREA = 0.2; //3 inches radius -> 28 inches^2 -> ~ 0.2 feet^2
float GAL_MONTLY_FACTOR = 0.623;
float GALLONS_TO_LITERS = 3.78541;

// Singleton instance of the radio driver
//RH_NRF24 driver; // 8,10
RH_NRF24 driver(6,7);   // 6 CE blu, 7 SS verde, per ATMEGA328P standalone
// RH_NRF24 driver(8, 7);   // For RFM73 on Anarduino Mini
// Class to manage message delivery and receipt, using the driver declared above
RHReliableDatagram manager(driver, SLAVE_ADDRESS);

/* data structure */

struct dataStruct{
  time_t timestamp;

  float press_atm;
  float temp;
  byte humidity;
  int light_intensity;

  int s_1_humidity_th;
  int s_1_max_cm3_week;

  int s_2_humidity_th;
  int s_2_max_cm3_week;

  int s_3_humidity_th;
  int s_3_max_cm3_week;

  boolean override_s;

}myData;

struct dataSend{
  time_t timestamp;
  
  int soil_humidity = 0;
}slaveData;

byte tx_buf[sizeof(slaveData)] = {0};

/////////////// RUNNING AVG //////////////////


class RunningAverage
{
public:
    RunningAverage(void);
    explicit RunningAverage(const uint8_t);
    ~RunningAverage();

    void clear();
    void addValue(const double);
    void fillValue(const double, const uint8_t);

    double getAverage() const;      // does iterate over all elements.
    int getSum() const;      // does iterate over all elements.
    double getFastAverage() const;  // reuses previous values.

    // returns min/max added to the data-set since last clear
    double getMin() const { return _min; };
    double getMax() const { return _max; };

    // returns min/max from the values in the internal buffer
    double GetMinInBuffer() const;
    double GetMaxInBuffer() const;

    double getElement(uint8_t idx) const;

    uint8_t getSize() const { return _size; }
    uint8_t getCount() const { return _cnt; }

protected:
    uint8_t _size;
    uint8_t _cnt;
    uint8_t _idx;
    double _sum;
    double * _ar;
    double _min;
    double _max;
};

///////////// END RUNNING AVG ///////////////

//define ring buffer variable
RunningAverage weekRingBuffer(7);  //create a ring buffer with 7 slots , to hold given water values of the last week..

/* this function collects and then send slave data to the master */
void sendData()
{
 slaveData.timestamp=now(); //current slave timestamp
 slaveData.soil_humidity = map(analogRead(A0),0,1023,1023,0); //hygrometer is on A0 pin, when 1023 means total dry, when 0 means total wet, so map is used to invert value
  
  memcpy(tx_buf, &slaveData, sizeof(slaveData) );
  byte d_size=sizeof(slaveData); 

  manager.sendtoWait((uint8_t *)tx_buf, d_size, SERVER_ADDRESS);
}

/* return soil humidity as sent.. (was read from A0) */
int getSoilHumidity()
{
  return slaveData.soil_humidity;
}

/* return water given in the seven slots, ideally last week.. */
int computeWeeklyWater()
{
  return weekRingBuffer.getSum();
  //Serial.println(weekRingBuffer.getSum(), DEC);
}

/* gives vol cm3 of water to the plant, converting the value in time ON at the water pump */
void giveWater(int vol)
{
  int pump_time_ON = vol*cm3_to_time*1000; 
  Serial.println("");
  Serial.print("giving water : ");
  Serial.print(vol, DEC);
  Serial.print(" [cm3] time_ON[ms] : ");
  Serial.print(pump_time_ON,DEC);
  Serial.print("\n");

  //start the pump
  digitalWrite(PUMP_PIN,HIGH);
  //provide the right volume by time
  delay(pump_time_ON);
  //stop the pump
  digitalWrite(PUMP_PIN,LOW);
  Serial.print("watering done..\n");

  /* if not override store relevant values */
  if(!myData.override_s) 
  {
        //store previous_timestamp with current value, before overwriting it with recv data
        previous_timestamp = myData.timestamp;
        //store previous_press_atm for forecast
        previous_press_atm = myData.press_atm;
        //store volume value into weekRingBuffer
        weekRingBuffer.addValue(vol);
  }
  
  
}

/* give coarse rain forescast basing on pressure delta */
boolean probablyRains()
{
  if(myData.press_atm < (previous_press_atm*0.965) && myData.humidity >=50) return true; //the rule is that if drops under 3.5% and RH > 50% with hi probability it will rain
  return false;
}

/* compute the volume of water using ET0 and other params/calculations */
/* depends on type of plant, type of soil, temperature, RH */
int computeVolumeWater()
{
  /*
   *  
   *  float CROP_COEFF = 0.2;
   *  float ET0[13]={0.0, 0.6, 1.1, 2.0, 3.0, 4.4, 5.3, 5.5, 4.8, 3.1, 1.8, 0.8, 0.5}; //ET0 values for the months
      float PLANTING_DENSITY = 1.0;
      float EXPOSURE_FACTOR = 1.0;
      float IRRIGATION_EFFICIENCY = 0.9;
      float PLANTED_AREA = 0.2; //3 inches radius -> 28 inches^2 -> ~ 0.2 feet^2
      float GAL_MONTLY_FACTOR = 0.623;
      float GALLONS_TO_LITERS = 3.78541;
   */
  
  
  int daily_volume = (ET0[month()]*CROP_COEFF*PLANTING_DENSITY*EXPOSURE_FACTOR*PLANTED_AREA*GAL_MONTLY_FACTOR*GALLONS_TO_LITERS)/(IRRIGATION_EFFICIENCY*4*7);
  return daily_volume;
}

/* this function processes remote received data and local data and , if requeired, enable water transfer to the plant */
void processData()
{ 
  if(!myData.override_s)  //normal behavior -> make calculations and , if needed, give water
  {
    if((previous_timestamp+86400) <= myData.timestamp) //watering can be made just with at least 24h intervals between them
    {
      if(computeWeeklyWater() < myData.s_1_max_cm3_week) //customize in other slaves..
      {
        if(getSoilHumidity() < myData.s_1_humidity_th) //customize in other slaves..
        {
          if(myData.light_intensity < LIGHT_TH) //give water only if light is under the threshold
          {
            if(!probablyRains()) //use pressure_atm and RH to make a coarse , short term, weather forecast
            {
              int volume = computeVolumeWater();
              giveWater(volume);
            }
          }
        }
      }
    }   
  }
  else //override is on -> water the plant with demo water volume if temp and humidity fall into range
     {
      if(myData.temp >= 45.0 && myData.humidity <= 20)
      {
        delay(DEMO_SLAVE_DELAY);
        giveWater(DEMO_WATER_VOLUME);
      }
     }
}




void setup() 
{
  pinMode(TX433_PIN, OUTPUT);
  pinMode(RX433_PIN,INPUT);
  pinMode(PUMP_PIN,OUTPUT);
  digitalWrite(PUMP_PIN, LOW);
  Serial.begin(115200);
  if (!manager.init())
    Serial.println("init failed");
  // Defaults after init are 2.402 GHz (channel 2), 2Mbps, 0dBm
}

void loop()
{
  uint8_t buf[RH_NRF24_MAX_MESSAGE_LEN];
  uint8_t buflen = sizeof(buf);
  uint8_t from;

  //RX MODE, wait for SYNC MSG from server
  if (manager.recvfromAckTimeout(buf, &buflen, 2000, &from))
    {
        int i;

        // Message with a good checksum received, dump it.
        //driver.printBuffer("Got :", buf, buflen);
        Serial.print("msg size : ");
        Serial.print(buflen, DEC);

        //overwrite data in memory using recv params
        memcpy(&myData, buf, sizeof(myData));
        Serial.println("");

        Serial.print("timestamp: ");
        Serial.print(myData.timestamp);

        setTime(myData.timestamp); //sync slave time to master timestamp
       
        Serial.print(" press_atm: ");
        Serial.print(myData.press_atm);
             
        Serial.print("  temp: ");
        Serial.print(myData.temp);;

        Serial.print("  humidity: ");
        Serial.print(myData.humidity);

        Serial.print("  light_intensity: ");
        Serial.print(myData.light_intensity);

        Serial.print("  s_1_humidity_th: ");
        Serial.print(myData.s_1_humidity_th);

        Serial.print("  s_1_max_cm3_week: ");
        Serial.print(myData.s_1_max_cm3_week);

        Serial.print("  s_2_humidity_th: ");
        Serial.print(myData.s_2_humidity_th);

        Serial.print("  s_2_max_cm3_week: ");
        Serial.print(myData.s_2_max_cm3_week);

        Serial.print("  s_3_humidity_th: ");
        Serial.print(myData.s_3_humidity_th);

        Serial.print("  s_3_max_cm3_week: ");
        Serial.print(myData.s_3_max_cm3_week);

        Serial.print("  override_s: ");
        Serial.print(myData.override_s);

        Serial.print("\n-----\n\n");

        
        //slave delay before tx slave data to master
        delay(SDI*SLAVE_ADDRESS);

        Serial.println("send data...");

        sendData();  //SEND SLAVE DATA TO THE MASTER

        Serial.println("process data...");

        processData(); //PROCESS DATA AND WATERING PLANT IF REQUIRED

        delay(200);

        Serial.println("back to receive..");
    }
    else
    {
      //Serial.println("No msg, is nrf24_reliable_datagram_server running?");
    }
    
 
}



////////////// RUNNING AVERAGE IMPLEMENTATION  ///////////////


RunningAverage::RunningAverage(const uint8_t size)
{
    _size = size;
    _ar = (double*) malloc(_size * sizeof(double));
    if (_ar == NULL) _size = 0;
    clear();
}

RunningAverage::~RunningAverage()
{
    if (_ar != NULL) free(_ar);
}

// resets all counters
void RunningAverage::clear()
{
    _cnt = 0;
    _idx = 0;
    _sum = 0.0;
    _min = NAN;
    _max = NAN;
    for (uint8_t i = 0; i < _size; i++)
    {
        _ar[i] = 0.0; // keeps addValue simpler
    }
}

// adds a new value to the data-set
void RunningAverage::addValue(const double value)
{
    if (_ar == NULL) return;  // allocation error
    _sum -= _ar[_idx];
    _ar[_idx] = value;
    _sum += _ar[_idx];
    _idx++;
    if (_idx == _size) _idx = 0;  // faster than %

    // handle min max
    if (_cnt == 0) _min = _max = value;
    else if (value < _min) _min = value;
    else if (value > _max) _max = value;

    // update count as last otherwise if( _cnt == 0) above will fail
    if (_cnt < _size) _cnt++;
}

// returns the average of the data-set added sofar
double RunningAverage::getAverage() const
{
    if (_cnt == 0) return NAN;
    double sum = 0;
    for (uint8_t i = 0; i < _cnt; i++)
    {
        sum += _ar[i];
    }
    return sum / _cnt;
}

int  RunningAverage::getSum() const
{
    double sum = 0;
    for (uint8_t i = 0; i < _cnt; i++)
    {
        sum += _ar[i];
    }
    return (int)sum;
}

double RunningAverage::getFastAverage() const
{
    if (_cnt == 0) return NAN;
    return _sum / _cnt;
}

// returns the max value in the buffer
double RunningAverage::GetMinInBuffer() const
{
    if (_cnt == 0) return NAN;
    double min = _ar[0];
    for (uint8_t i = 1; i < _cnt; i++)
    {
        if (min > _ar[i]) min = _ar[i];
    }
    return min;
}

double RunningAverage::GetMaxInBuffer() const
{
    if (_cnt == 0) return NAN;
    double max = _ar[0];
    for (uint8_t i = 1; i < _cnt; i++)
    {
        if (max < _ar[i]) max = _ar[i];
    }
    return max;
}

// returns the value of an element if exist, NAN otherwise
double RunningAverage::getElement(uint8_t idx) const
{
    if (idx >=_cnt ) return NAN;
    return _ar[idx];
}

// fill the average with a value
// the param number determines how often value is added (weight)
// number should preferably be between 1 and size
void RunningAverage::fillValue(const double value, const uint8_t number)
{
    clear(); // TODO conditional?  if (clr) clear();

    for (uint8_t i = 0; i < number; i++)
    {
        addValue(value);
    }
}




