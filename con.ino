#include <SD.h>
#include <SPI.h>
#include <EEPROM.h>
#include <Seeed_BME280.h>
#include <TinyGPSPlus.h>
#include <TimeLib.h>
#include <Servo.h>

//set port
#define voldivpin 21//set port
#define LED1 4//set port
#define LED2 5//set port
#define buzzer 3//set port

TinyGPSPlus gps;
BME280 bme280;
time_t RTCTime;
Servo servo1;
Servo servo2;

unsigned long time1 = 0;
unsigned long time0 = 0;

bool cxON = false;
bool simEN = false;
bool simAC = false;
bool SIM = false;
int first = 0;

char FileC[100]; 
char FileP[100]; //FileS1

const int recovPkg = 0;
const int recovState = 10;
const int recovMode = 20;
const int recovAlt = 30;

float Peak = -2147483648;
int refAltitude = 0;
int state = 0;
int simPressure = 0;

String Telemetry = "";
String P1 = "";
String cm = ""; //IDK WHAT THIS IS

String teamId = "1022";
char missionTime[32] = "xx:xx:xx";
int Packet = 0;
char Mode = 'F';
char P1r = 'N'; //S1r
float Altitude = 0;
float Temp = 0;
float Voltage = 0;
char gpsTime[32] = "xx:xx:xx";
float Latitude = 0;
float Longitude = 0;
float gpsAltitude = 0;
int gpsSatellite = 0;
String State = "PRELAUNCH";
int StatePayload = 0;
String cmdEcho = "----"; //IDK what is this doing but know why its here

void setup() {
  
  Serial3.begin(9600);
  Serial4.begin(9600);

  servo1.attach(22); //set port
  servo2.attach(23); //set port
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(buzzer,OUTPUT);
  pinMode(voldivpin,INPUT);

  setSyncProvider(getTeensy3Time);
  bme280.init();

  if (!SD.begin(10)) {
    for (int i=0;i<5;i++){
    digitalWrite(buzzer,HIGH);
    delay(50);
    digitalWrite(buzzer,LOW);
    delay(50);
    }}
  Serial.println("[Install SD Card done"); //fix later
  recovery();
}

void recovery(){
  String a = EEPROM.read(recovPkg);
  String b = EEPROM.read(recovState);
  String c = EEPROM.read(recovMode);
  String d = EEPROM.read(recovAlt);
  int e = c.toInt();

  switch (e){
    case 0:  for (int i = 0 ; i < EEPROM.length() ; i++) {
                 EEPROM.write(i, 0);
              }
              cxON = false;
              break;
    case 1:  cxON = true;
              cmdEcho = "CXON";
              Mode = 'F';
              break;
    case 2: cmdEcho = "CXON";
             simEN = true;
             simAC = true;
             Mode = 'S';
             break;
  }
    if (d != '0'){
    refAltitude = d.toInt();
  }else{
    refAltitude = round(float(bme280.calcAltitude(bme280.getPressure())));}
  Packet = a.toInt();
  state = b.toInt();
  get_file();
}

void Command(String cm){
  if (cm == "CX,ON"){
    Serial.println("CXON");
    cxON = true;
    refAltitude = round(float(bme280.calcAltitude(bme280.getPressure())));
    Packet = 0;
    cmdEcho = "CXON";
    state = 0;
    Mode = 'F';
    EEPROM.write(recovMode, 1);
    EEPROM.write(recovAlt, refAltitude);
    EEPROM.write(recovState, state);
    EEPROM.write(recovPkg,0);
    get_file();
  }
  else if (cm == "CX,OFF"){
    Serial.println("CXOFF");
    cxON = false;
    cmdEcho="CXOFF";
    Mode = 'F';
    for (int i = 0 ; i < EEPROM.length() ; i++) {
      EEPROM.write(i, 0);
    }
    servo1.write(90);
    servo2.write(90);
    Serial.println("PayloadOFF");
    Serial5.print("CMD,1022,PayloadX,OFF\r");
    P1r='R';
    cmdEcho = "PayloadON";
  }
  else if (cm == "PayloadX,ON"){
    Serial5.print("CMD,1022,PayloadX,ON\r");
    P1r='R';
    cmdEcho = "PayloadON";
  }
  else if (cm == "PayloadX,OFF"){
    Serial5.print("CMD,1022,PayloadX,OFF\r");
    P1r='N';
    cmdEcho = "PayloadOFF";
  }
  else if (cm == "SIM,ENABLE"){
    simEN = true;
    cmdEcho = "SIMEN";
    Serial.println("SIM_EN");
  }
  else if (cm == "SIM,ACTIVATE" && simEN){
    simAC = true;
    cmdEcho = "SIMAC";
    Mode = 'S';
    Serial.println("SIM_AC");
    EEPROM.write(recovMode, 2);
  }
  else if (cm == "SIM,DISABLE"){
    simEN = false;
    simAC = false;
    SIM = false;
    Mode = 'F';
    EEPROM.write(recovMode, 1);
    Serial.println("SIMDIS");
    Command("CX,ON");
  }
  else if (cm.indexOf("SIMP")!=-1 && (simAC&&simEN)){
    cmdEcho = "SIMP";
    simPressure = (cm.substring(5)).toInt();
    Serial.println("sim ; "+ String(cm.substring(5)));
    Mode = 'S';
    if (first==0){
      SIM = true;
      refAltitude = round(float(bme280.calcAltitude(simPressure)));
      EEPROM.write(recovAlt, refAltitude);
      first = 1;
    }
  }
  else{
    Serial.println(cm); 
  }
}

void get_file(){
  int idx = 0;
  String C = ("CI" + String(idx) + ".txt");
  C.toCharArray(FileC,100);
  while (SD.exists(FileC))
  {
    idx++;
    C=("CI" + String(idx) + ".txt");
    C.toCharArray(FileC,100);
  }
  C=("Payload" + String(idx) + ".txt");
  C.toCharArray(FileP,100);
  Serial.println(FileC); 
  Serial.println(FileP); 
}

time_t getTeensy3Time() {
  return Teensy3Clock.get();
}

void get_gps(){
  gps.encode(Serial2.read());
  if (gps.location.isValid()){
    Latitude=gps.location.lat();
    Longitude=gps.location.lng();
    }
  sprintf(gpsTime, "%02d:%02d:%02d", gps.time.hour(), gps.time.minute(), gps.time.second());
  gpsAltitude = gps.altitude.meters();
  gpsSatellite = gps.satellites.value();
}

void get_BME_flight(){
  Temp = bme280.getTemperature();
  float a = bme280.calcAltitude(bme280.getPressure());
  Altitude = float(a-refAltitude);
  if (Altitude>=Peak){
    Peak=Altitude;
  }
}

void get_BME_simulation(){
  Temp = bme280.getTemperature();
  float a = bme280.calcAltitude(simPressure);
  Altitude = float(a-refAltitude);
  if (Altitude>=Peak){
    Peak = Altitude;
  }
}

void get_time(){
  sprintf(missionTime, "%02d:%02d:%02d", hour(), minute(), second());
}

void get_battery(){
  Voltage = (((analogRead(voldivpin) * 0.00080566406)*(4000))/1000)+0.16;
}

void inMission(){
   time1 = millis();
  if (time1-time0>=990){
    get_time();
    get_gps();
    get_battery();
    if (SIM){
      get_BME_simulation();
    }else{
      get_BME_flight();
    }
    switch(state){
      case 0:
        State = "PRELAUNCH";
        if (Altitude >= 10){
          state = 1;
        }break;
      case 1:
        State = "LAUNCH";
        if (Peak-Altitude >= 30 && Altitude > 600){
          state = 2;
        }break;
      case 2:
        State = "EJECTED Payload";
        if (Altitude <= 310 && Altitude > 290){
          state = 3;
          P1r = 'R';
          Serial5.print("CMD,1022,PAYLOAD,ON\r");
          servo1.write(90);
          //delay(1000);
          //servo2.write(0);
        }break;
      case 3:
        State = "EJECTED SECOND PARACHUTE";
        if (Altitude <= 410 && Altitude > 390){
          state = 4;
          P1r = 'R';
          Serial3.print("CMD,1022,SECOND PARACHUTE,ON\r");
          servo2.write(180);
          //delay(1000);
          //servo2.write(0);
        }break;
      case 4:
        State = "RELEASED_2";
        servo2.write(180);
        if (Altitude <= 5 && Altitude >= -5){
          state = 5;
          State = "LAND";
        }break;
      case 5:
        while (true){
          digitalWrite(buzzer,HIGH);
          delay(500);
          digitalWrite(buzzer,LOW);
          delay(500);
        }break;
    }
    Telemetry = teamId + "," + missionTime + "," 
       + String(Packet) + ",C," + Mode + "," + String(P1r) + ","
       + String(Altitude,2) + "," + String(Temp,2) + ","
       + String(Voltage) + "," + String(gpsTime) + "," 
       + String(Latitude,6) + "," + String(Longitude,6) + "," + String(gpsAltitude) + ","
       + String(gpsSatellite) + "," + State + "," + String(StatePayload) + "," + String(P1r)
       + cmdEcho + "\r";
    File file = SD.open(FileC, FILE_WRITE);
    if (file){
      file.println(Telemetry);
      file.close();
    }
    Serial3.println("CX : " + Telemetry);
    Serial4.println(Telemetry);
    EEPROM.put(recovPkg, Packet);
    EEPROM.put(recovState, state);
    Packet ++;
    time0 = time1;
  }
}

void emergency(){
  if (Serial3.available()) {  //input xbee
    while (Serial3.available ()) {
      char emergency = Serial3.read();
      switch(emergency){
       case 0:
        if (emergency == "CMD,1022,FORCE,PARADEPLOY") {
        servo1.write(90);
        //delay(1000);
        servo1.write(0);
        }break;
       case 1:
        if (emergency == "CMD,1022,FORCE,TIMEDPL") {
        servo2.write(90);
        //delay(1000);
        servo2.write(0);
        }break;
       case 2:
        if (emergency == "CMD,1022,FORCE,BEGINPL") {
        servo1.write(90);
        delay(1000);
        }break;
       case 3:
        if (emergency == "CMD,1022,FORCE,STOPPL") {
        servo1.write(0);
        }break;
       case 4:
        if (emergency == "CMD,1022,FORCE,RESETCAMPOS") {
        servo1.write(90);
        servo2.write(90);
        }break;
      }   
    }
  }
}
void loop() {
  if (cxON){
    inMission();
  }
  if (Serial3.available()) {
    while (Serial3.available()){
      char inchar = Serial3.read();
      if (inchar =='$' or inchar == '\r'){
        cm = cm.trim();
        cm = (cm.substring(9));
        Serial.println("S3:" + cm);
        Command(cm);
        cm = "";
      }
      else{
        cm += inchar;
  }}}
  if(Serial4.available()){
    while (Serial4.available()){
      char inchar=Serial4.read();
      if (inchar=='$' or inchar=='\r'){
        P1=P1.trim();   
        Serial4.println(P1+"$");
        Serial4.println("S1 : "+P1);
        StatePayload++;
        File file = SD.open(FILE_WRITE);
        if (file){
          file.println(P1);
          file.close();
        }
        P1 = "";
      }
      else{
      P1+=inchar;
    }}}
}
