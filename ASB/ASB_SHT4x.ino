// CubeSat v1 ASB Code Revision 1.0
// =================================
// ©Copyright 2023 EdgeFlyte, LLC.
// All Rights Reserved. 

#include "Arduino.h"
#include <Wire.h>
#include "Adafruit_SHT31.h"
#include "SparkFun_ENS160.h"
#include <Adafruit_BMP085.h>
#include <sps30.h>
#include <DFRobot_SCD4X.h>
#include "Adafruit_SHT4x.h"



Adafruit_SHT31 sht31 = Adafruit_SHT31();
Adafruit_SHT4x sht4 = Adafruit_SHT4x();

SparkFun_ENS160 ens; 
Adafruit_BMP085 bmp;
struct sps30_measurement m;
DFRobot_SCD4X SCD4X(&Wire, SCD4X_I2C_ADDR);


// ASB Variables
#define LED     2
bool ist[128];


struct baseSens {
  bool sht3x, bmp180, ens160, sps30, scd40;
};
baseSens bs;

struct baseData {
  float t1, t2, p1, rh, pa1;
  uint32_t aqi, tvoc, eco2;
  uint16_t co2;
  int ensFlags;
  
};
baseData bd;


void setup() {
  Wire.begin();
  Serial.begin(9600);
  pinMode(1, INPUT);
  UCSR0B &= ~_BV(TXEN0);
  
  bootLED();
  
  scani2c();

  UCSR0B |= _BV(TXEN0);

  // Check base sensors
  for(int i=0; i<128; i++){
    if(ist[i]){
      Serial.print("DTC 0x");
      if(i < 16) Serial.print("0");
      Serial.println(i, HEX);
    }
  }
  delay(100);
  UCSR0B &= ~bit (TXEN0);
  if(ist[0x77]) bs.bmp180 = true;
  if(ist[0x44]) bs.sht3x = true;

  if(ist[0x53]) bs.ens160 = true;
  

  sensirion_i2c_init();

  if(sps30_probe() != 0) {
    Serial.print("SPS sensor probing failed\n");
    delay(500);
    bs.sps30 = false;
  }
  uint16_t ret = sps30_set_fan_auto_cleaning_interval_days(4);
  if (ret) {
    Serial.print("error setting the auto-clean interval: ");
    Serial.println(ret);
  }
  ret = sps30_start_measurement();
  if (ret < 0) {
    Serial.print("error starting measurement\n");
  }

  if (bmp.begin()) {
    flashLED();
  }
  if(ens.begin() ){
    flashLED();
  }
  // if(sht31.begin()){
  //   flashLED();
  // }
  if(SCD4X.begin() ){
    flashLED();
  }
  if (sht4.begin()) {
    flashLED();
  }

  // Serial.println("Found SHT4x sensor");
  // Serial.print("Serial number 0x");
  // Serial.println(sht4.readSerial(), HEX);
  sht4.setPrecision(SHT4X_LOW_PRECISION);
  sht4.setHeater(SHT4X_NO_HEATER);



  delay(2000);
  SCD4X.enablePeriodMeasure(SCD4X_STOP_PERIODIC_MEASURE);
  SCD4X.setTempComp(4.0);
//  float temp = 0;
//  temp = SCD4X.getTempComp();
//  Serial.print("The current temperature compensation value : ");
//  Serial.print(temp);
//  Serial.println(" C");
  SCD4X.setSensorAltitude(bmp.readAltitude());
  SCD4X.enablePeriodMeasure(SCD4X_START_PERIODIC_MEASURE);
  
  
  if( ens.setOperatingMode(SFE_ENS160_RESET)){
    Serial.println("Ready.");
    delay(100);
    ens.setOperatingMode(SFE_ENS160_STANDARD);
    delay(100);

    int ensStatus = ens.getFlags();
  //  Serial.print("Gas Sensor Status Flag (0 - Standard, 1 - Warm up, 2 - Initial Start Up): ");
    Serial.println(ensStatus);
  }
}

uint32_t lowPollRate, highPollRate, txPacketTime = 0;

void loop() {
  if(lowPollRate + 1000 <= millis()){
    getSHT3x();
    getENS160();
    pollsps();
    getSCD4x();
    lowPollRate = millis();
  }

  if(highPollRate + 500 <= millis()){
    getBMP180();
    
    highPollRate = millis();
  } 

  if(txPacketTime + 1000 <= millis()){
    txMainPkt();
    txPM2Pkt();
    txPacketTime = millis();
  }
}


void bootLED(){
  for(int i=0; i<10; i++){
    digitalWrite(LED, 1);
    delay(50);
    digitalWrite(LED, 0);
    delay(50);
  }
}


void flashLED(){
  digitalWrite(LED, 1);
  delay(100);
  digitalWrite(LED, 0);
  delay(500);
  
}



void scani2c(){
  for(byte a = 1; a < 127; a++ ){
    Wire.beginTransmission(a);
    byte e = Wire.endTransmission();
    if (e == 0) ist[a] = true;
    else  ist[a] = false;
  }  
}



void txMainPkt(){
  UCSR0B |= _BV(TXEN0);
  digitalWrite(LED, 1);

  char bu[150];
  //                  | T1 | T2 | RH | P1 | PA |AQI |TVOC|ECO2|CO2
  sprintf(bu, "$ASB,0,%05d,%05d,%04d,%05d,%05d,%01d,%06d,%05d,%05d,*",
          (int)((bd.t1+40)*100), (int)((bd.t2+40)*100), (int)(bd.rh*10),
          (int)(bd.p1/10), (int)(bd.pa1),
          (int)bd.aqi, (int)bd.tvoc, (int)bd.eco2, (int)bd.co2);
  Serial.println(bu);
  delay(100);
  UCSR0B &= ~bit (TXEN0);
  // UCSR0B &= _BV(TXEN0);
  // digitalWrite(1, 0);
  // pinMode(1, INPUT);
  digitalWrite(LED, 0);
}

void txPM2Pkt(){
  digitalWrite(LED, 1);
  UCSR0B |= _BV(TXEN0);
  char bu[150];
  //                  |PM 1.0 - 10.0     |NC 0.5 - 10.0           |TYP Part Size
  sprintf(bu, "$ASB,1,%05d,%05d,%05d,%05d,%06d,%06d,%06d,%06d,%06d,%03d,*",
          (int)(m.mc_1p0 * 10),(int)(m.mc_2p5 * 10),(int)(m.mc_4p0 * 10),(int)(m.mc_10p0 * 10),
          (int)((m.nc_0p5)*10), (int)((m.nc_1p0  - m.nc_0p5)*10), (int)((m.nc_2p5  - m.nc_1p0)*10), 
          (int)((m.nc_4p0  - m.nc_2p5)*10), (int)((m.nc_10p0 - m.nc_4p0)*10),
          (int)(m.typical_particle_size * 100)
          );
  Serial.println(bu);
  delay(100);
  // UCSR0B &= _BV(TXEN0);
  UCSR0B &= ~bit (TXEN0);
  // digitalWrite(1, 0);
  // pinMode(1, INPUT);
  digitalWrite(LED, 0);
}




// Main Sens Functions
void getSHT3x(){
  // float t = sht31.readTemperature();
  // float h = sht31.readHumidity();
  // if (! isnan(t)) bd.t1 = t;
  // if (! isnan(h)) bd.rh = h;

  // UCSR0B |= _BV(TXEN0);

  // Serial.print("S31 T: ");
  // Serial.print(t);
  // Serial.print(" | H: ");
  // Serial.println(h);

  sensors_event_t humidity, temp;
  sht4.getEvent(&humidity, &temp);// populate temp and humidity objects with fresh data
  // Serial.print("Temperature: "); Serial.print(temp.temperature); Serial.println(" degrees C");
  // Serial.print("Humidity: "); Serial.print(humidity.relative_humidity); Serial.println("% rH");

  float t = (float)temp.temperature;
  float h = (float)humidity.relative_humidity;

  if (! isnan(t)) bd.t1 = t;
  if (! isnan(h)) bd.rh = h;


  // delay(100);
  // UCSR0B &= ~bit (TXEN0);

  delay(1000);
}

void getBMP180(){
  bd.t2 = bmp.readTemperature();
  bd.p1 = bmp.readPressure();
  bd.pa1 = bmp.readAltitude();

  // UCSR0B |= _BV(TXEN0);
  // Serial.print("#####   ");
  // Serial.println(bd.p1);

  // delay(100);
  // UCSR0B &= ~bit (TXEN0);

}

void getENS160(){

  if( ens.checkDataStatus() ){
//    Serial.print("Air Quality Index (1-5) : ");
//    Serial.println(ens.getAQI());
    bd.aqi = ens.getAQI();

//    Serial.print("Total Volatile Organic Compounds: ");
//    Serial.print(ens.getTVOC());
//    Serial.println("ppb");
    bd.tvoc = ens.getTVOC();

//    Serial.print("CO2 concentration: ");
//    Serial.print(ens.getECO2());
//    Serial.println("ppm");
    bd.eco2 = ens.getECO2();

//    Serial.print("Gas Sensor Status Flag (0 - Standard, 1 - Warm up, 2 - Initial Start Up): ");
//    Serial.println(ens.getFlags());
    bd.ensFlags = ens.getFlags();

//    Serial.println();
  }
}





void pollsps(){
//  if(!bs.sps30) return;
  
  
  uint16_t data_ready;
  int16_t ret;

  ret = sps30_read_data_ready(&data_ready);
  if (ret < 0) {
    Serial.print("#error reading data-ready flag: ");
    Serial.println(ret);
    return;
  } else if (!data_ready){
    Serial.print("#data not ready, no new measurement available\n");
    return;
  }
  ret = sps30_read_measurement(&m);
  if (ret < 0) {
    Serial.print("#error reading measurement\n");
    return;
  } else {

//    Serial.print("PM  1.0: ");
//    Serial.println(m.mc_1p0);
//    Serial.print("PM  2.5: ");
//    Serial.println(m.mc_2p5);
//    Serial.print("PM  4.0: ");
//    Serial.println(m.mc_4p0);
//    Serial.print("PM 10.0: ");
//    Serial.println(m.mc_10p0);
//
//    Serial.print("Typical partical size: ");
//    Serial.println(m.typical_particle_size);
//
//    Serial.print(m.nc_0p5);
//    Serial.print(" ");
//    Serial.print(m.nc_1p0  - m.nc_0p5);
//    Serial.print(" ");
//    Serial.print(m.nc_2p5  - m.nc_1p0);
//    Serial.print(" ");
//    Serial.print(m.nc_4p0  - m.nc_2p5);
//    Serial.print(" ");
//    Serial.print(m.nc_10p0 - m.nc_4p0);
//    Serial.println();
  }
}

void getSCD4x(){
  if(SCD4X.getDataReadyStatus()) {
    DFRobot_SCD4X::sSensorMeasurement_t data;
    SCD4X.readMeasurement(&data);
//
//    Serial.print("Carbon dioxide concentration : ");
//    Serial.print(data.CO2ppm);
//    Serial.println(" ppm");
    bd.co2 = data.CO2ppm;
//
//    Serial.print("Environment temperature : ");
//    Serial.print(data.temp);
//    Serial.println(" C");
//
//    Serial.print("Relative humidity : ");
//    Serial.print(data.humidity);
//    Serial.println(" RH");

//    Serial.println();
  }
}
