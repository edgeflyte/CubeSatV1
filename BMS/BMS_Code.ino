// CubeSat v1 BMS Code Revision 1.0
// =================================
// ©Copyright 2023 EdgeFlyte, LLC.
// All Rights Reserved. 


#define VSENSE    A0
#define SP1       A1
#define SP2       A2

// LEDS
#define L1        2
#define L2        3
#define L3        4

// LDO / Channel Outs
#define LDO1      5
#define LDO2      6
#define LDO3      7
#define LDO4      8

// Batt Chem
const int NUM_CELLS = 4;
const float MIN_VOLTAGE_PER_CELL = 1.2;
const float MAX_VOLTAGE_PER_CELL = 1.4;

const float vsense_offset = -2.81;
const float sp1_offset = -1.65;
const float sp2_offset = -1.65;

// Sys Variables
bool activeonBoot_L1 = true;
bool activeonBoot_L2 = true;
bool activeonBoot_L3 = false;
bool activeonBoot_L4 = true;


bool active_LDO1 = false;
bool active_LDO2 = false;
bool active_LDO3 = false;
bool active_LDO4 = false;

bool disabled_LDO1 = false;
bool disabled_LDO2 = false;
bool disabled_LDO3 = false;
bool disabled_LDO4 = false;

bool rememberStates = false;  // Write LDO States to EEPROM

int err = 0;


// System Sensor Variables
float batV, batPct;
float sol1, sol2;



void setup() {
  initPins();
  Serial.begin(9600);
  Serial.println("#BMS, Battery Management System Initializing...");
  sequenceLEDS();

  if(activeonBoot_L1) setLDO(1, 1);
  if(activeonBoot_L2) setLDO(2, 1);
  if(activeonBoot_L3) setLDO(3, 1);
  if(activeonBoot_L4) setLDO(4, 1);
  

  
  Serial.println("#BMS, Battery Management System Initialized.");
}


double sensT, updateT = 0;

void loop() {
  lights();
//  Serial.print("*");
//  Serial.print(sensT);
//  Serial.print(",");
//  Serial.print(updateT);
//  Serial.print(",");
//  Serial.print(millis());
//  Serial.println("*");
  
  
  

  if(sensT + 2000 <= millis()){
    batV = getBatV();
    batPct = getBatPct();
    sol1 = getSol(1);
    sol2 = getSol(2);
    
    
    if(batPct <= 20) err = 1;   // Do a small battery checking
    if(batPct <= 5)  err = 2;
    sensT = millis();
  }


  if(updateT + 1500 <= millis()){
    digitalWrite(L1, 1);
    char bu[20];
    Serial.print("$BMS,");
    Serial.print(err);
    Serial.print(",");
    Serial.print(active_LDO1);
    Serial.print(",");
    Serial.print(active_LDO2);
    Serial.print(",");
    Serial.print(active_LDO3);
    Serial.print(",");
    Serial.print(active_LDO4);
    Serial.print(",");
    dtostrf(batV, -1, 2, bu);
    Serial.print(bu);
    Serial.print(",");
    dtostrf(batPct, -3, 2, bu);
    Serial.print(bu);
    Serial.print(",");
    dtostrf(sol1, -1, 2, bu);
    Serial.print(bu);
    Serial.print(",");
    dtostrf(sol2, -1, 2, bu);
    Serial.print(bu);
    Serial.println("*");  
    
    digitalWrite(L1, 0);  
    updateT = millis();
  }

  
  if (Serial.available() > 0) {
    digitalWrite(L2, 1);
    String rs = Serial.readStringUntil('\n');
    if (rs.startsWith("&BMS")) {
      // Get the command section
      String es = rs.substring(5, 10);

      // Start System Commands
      if (es.equals("CH1,1")) { // Toggle LDO1 On
        Serial.println("#Set CH1 ON...");
        if(setLDO(1, 1)) Serial.println("#OK");
      }
      if (es.equals("CH1,0")) {
        Serial.println("#Set CH1 OFF...");
        if(setLDO(1, 0)) Serial.println("#OK");
      }
      if (es.equals("CH2,1")) {
        Serial.println("#Set CH2 ON...");
        if(setLDO(2, 1)) Serial.println("#OK");
      }
      if (es.equals("CH2,0")) {
        Serial.println("#Set CH2 OFF...");
        if(setLDO(2, 0)) Serial.println("#OK");
      }
      if (es.equals("CH3,1")) {
        Serial.println("#Set CH3 ON...");
        if(setLDO(3, 1)) Serial.println("#OK");
      }
      if (es.equals("CH3,0")) {
        Serial.println("#Set CH3 OFF...");
        if(setLDO(3, 0)) Serial.println("#OK");
      }
      if (es.equals("CH4,1")) {
        Serial.println("#Set CH4 ON...");
        if(setLDO(4, 1)) Serial.println("#OK");
      }
      if (es.equals("CH4,0")) {
        Serial.println("#Set CH4 OFF...");
        if(setLDO(4, 0)) Serial.println("#OK");
      }
      
      if (es.equals("RST,1")) {
        if(rst()) Serial.println("#OK");
      }

      // End System Control Commands
    } else {
      Serial.println("#BMS?");
    }
    digitalWrite(L2, 0);
  }
}

float getBatV(){
  int av = analogRead(VSENSE);
//  Serial.println(av);
  float iv = (av * 5.0) / 1024.0;
  iv = iv / .5;
  iv = iv + vsense_offset;
  iv = (iv < 0.1) ? 0.0 : iv;
//  Serial.print("V:");
//  Serial.println(iv);
  return iv;
}

float getBatPct(){
  float v = getBatV();
  int bp = map(v, MIN_VOLTAGE_PER_CELL * NUM_CELLS, MAX_VOLTAGE_PER_CELL * NUM_CELLS, 0, 100);
  bp = constrain(bp, 0, 100);
  return bp;
}


float getSol(int pan){
  float v;
  if(pan == 1){
    int av = analogRead(SP1);
    v = (av * 5.0) / 1024.0;
    v = v + sp1_offset;
    v = (v < 0.1) ? 0.0 : v;
    return v;
  }
  if(pan == 2){
    int av = analogRead(SP2);
    v = (av * 5.0) / 1024.0;
    v = v + sp2_offset;
    v = (v < 0.1) ? 0.0 : v;
    return v;
  }
}


void initPins(){
  pinMode(L1, OUTPUT);
  pinMode(L2, OUTPUT);
  pinMode(L3, OUTPUT);
  
  pinMode(LDO1, OUTPUT);
  pinMode(LDO2, OUTPUT);
  pinMode(LDO3, OUTPUT);
  pinMode(LDO4, OUTPUT);
}

void sequenceLEDS(){
  for(int i =0; i<10; i++){
    digitalWrite(L1, HIGH);
    delay(100);
    digitalWrite(L1, LOW);
    digitalWrite(L2, HIGH);
    delay(100);
    digitalWrite(L2, LOW);
    digitalWrite(L3, HIGH);
    delay(100);
    digitalWrite(L3, LOW);
  }
  digitalWrite(L1, LOW);
  digitalWrite(L2, LOW);
  digitalWrite(L3, LOW);
}

bool setLDO(int ch, bool state){
  if(ch==1){
    digitalWrite(LDO1, state);
    active_LDO1 = state;
    return true;
  }
  if(ch==2){
    digitalWrite(LDO2, state);
    active_LDO2 = state;
    return true;
  }
  if(ch==3){
    digitalWrite(LDO3, state);
    active_LDO3 = state;
    return true;
  }
  if(ch==4){
    digitalWrite(LDO4, state);
    active_LDO4 = state;
    return true;
  }
  return false;
}

bool rst(){
  // reset system somehow
  
}


double l1t, l2t, l3t; // Timers
bool l1s, l2s, l3s; // States

int lights(){
 
  if(err){  // Error code active, determine which one it is and set l3...
    
    if(err == 1){ // Low Battery Level
      if(l3t - 2000 >= millis()){
        l3s = !l3s;
        digitalWrite(L3, l3s);
        l3t = millis();
      }
    }
    
    if(err == 2){ // Critical Battery Level
      if(l3t - 500 >= millis()){
        l3s = !l3s;
        digitalWrite(L3, l3s);
        l3t = millis();
      }
    }
    
  }
}
