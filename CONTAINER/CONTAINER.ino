#include <EEPROM.h>
#include <SD.h>
#include <SPI.h>
#include <Seeed_BME280.h>
#include <Servo.h>
#include <TimeLib.h>
#include <TinyGPSPlus.h>

// set port
#define voldivpin 21  // set port
#define LED1 4        // set port
#define LED2 5        // set port
#define buzzer 3      // set port

#define R1_Ohm 2000
#define R2_Ohm 1200

TinyGPSPlus gps;
BME280 bme280;
time_t RTCTime;
Servo servo1;
Servo servo2;

unsigned long time1 = 0;
unsigned long time0 = 0;

unsigned long poll_time1 = 0;
unsigned long poll_time0 = 0;

bool cxON = false;
bool simEN = false;
bool simAC = false;
bool SIM = false;
int first = 0;

char FileC[100];
char FileP[100];  // FileS1

const int recovPkg = 0;
const int recovState = 10;
const int recovMode = 20;
const int recovAlt = 30;

float Peak = -2147483648;
int refAltitude = 0;
int state = 0;
int simPressure = 0;

String telemetry = "";
String tp = "";
String cmd = "";  // IDK WHAT THIS IS

String teamId = "1022";
char missionTime[32] = "xx:xx:xx";
int Packet = 0;
char Mode = 'F';
char P = 'N';
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
String cmdEcho = "N/A";  // IDK what is this doing but know why its here

void recovery();
void get_file();
time_t getTeensy3Time();

void setup() {
    Serial.begin(9600);
    Serial.println("Initiating ...");
    Serial3.begin(115200);  // GCS
    Serial4.begin(115200);  // Payload

    servo1.attach(22);  // set port
    servo2.attach(23);  // set port
    pinMode(LED1, OUTPUT);
    pinMode(LED2, OUTPUT);
    pinMode(buzzer, OUTPUT);
    pinMode(voldivpin, INPUT);

    pinMode(13, OUTPUT);
    digitalWrite(13, HIGH);

    setSyncProvider(getTeensy3Time);

    Serial.println("Initializing BME280 ...");
    bme280.init();

    Serial.println("Initializing SD ...");
    if (!SD.begin(10)) {
        Serial.println("SD card failed to initialize");
        for (int i = 0; i < 5; i++) {
            digitalWrite(buzzer, HIGH);
            delay(50);
            digitalWrite(buzzer, LOW);
            delay(50);
        }
    }
    Serial.println("Install SD Card done");  // fix later
    recovery();
}

void recovery() {
    String a = EEPROM.read(recovPkg);
    String b = EEPROM.read(recovState);
    String c = EEPROM.read(recovMode);
    String d = EEPROM.read(recovAlt);
    int e = c.toInt();

    switch (e) {
        case 0:
            for (int i = 0; i < EEPROM.length(); i++) {
                EEPROM.write(i, 0);
            }
            cxON = false;
            break;
        case 1:
            cxON = true;
            cmdEcho = "CXON";
            Mode = 'F';
            break;
        case 2:
            cmdEcho = "CXON";
            simEN = true;
            simAC = true;
            Mode = 'S';
            break;
    }
    if (d != '0') {
        refAltitude = d.toInt();
    } else {
        refAltitude = round(float(bme280.calcAltitude(bme280.getPressure())));
    }
    Packet = a.toInt();
    state = b.toInt();
    get_file();
}

void doCommand(String cmd) {
    if (cmd == "CX,ON") {
        for (int i = 0; i < 2; i++) {  // omsin
            digitalWrite(buzzer, HIGH);
            delay(50);
            digitalWrite(buzzer, LOW);
            delay(50);
        }
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
        EEPROM.write(recovPkg, 0);
        get_file();
    } else if (cmd == "CX,OFF") {
        Serial.println("CXOFF");
        cxON = false;
        cmdEcho = "CXOFF";
        Mode = 'F';
        for (int i = 0; i < EEPROM.length(); i++) {
            EEPROM.write(i, 0);
        }
        servo1.write(90);
        servo2.write(90);
        Serial.println("CXOFF");
        state = 0;
    } else if (cmd == "SIM,ENABLE") {
        simEN = true;
        cmdEcho = "SIMEN";
        Serial.println("SIM_EN");
    } else if (cmd == "SIM,ACTIVATE" && simEN) {
        simAC = true;
        cmdEcho = "SIMAC";
        Mode = 'S';
        Serial.println("SIM_AC");
        EEPROM.write(recovMode, 2);
    } else if (cmd == "SIM,DISABLE") {
        simEN = false;
        simAC = false;
        SIM = false;
        Mode = 'F';
        EEPROM.write(recovMode, 1);
        Serial.println("SIMDIS");
        doCommand("CX,ON");
    } else if (cmd.indexOf("SIMP") != -1 && (simAC && simEN)) {
        cmdEcho = "SIMP";
        simPressure = (cmd.substring(5)).toInt();
        Serial.println("sim ; " + String(cmd.substring(5)));
        Mode = 'S';
        if (first == 0) {
            SIM = true;
            refAltitude = round(float(bme280.calcAltitude(simPressure)));
            EEPROM.write(recovAlt, refAltitude);
            first = 1;
        }
    } else {
        emergency(cmd);
        Serial.println(cmd);
    }
}

void get_file() {
    int idx = 0;
    String C = ("CI" + String(idx) + ".txt");
    C.toCharArray(FileC, 100);
    while (SD.exists(FileC)) {
        idx++;
        C = ("CI" + String(idx) + ".txt");
        C.toCharArray(FileC, 100);
    }
    C = ("Payload" + String(idx) + ".txt");
    C.toCharArray(FileP, 100);
    Serial.println(FileC);
    Serial.println(FileP);
}

time_t getTeensy3Time() {
    return Teensy3Clock.get();
}

void get_gps() {
    gps.encode(Serial2.read());
    if (gps.location.isValid()) {
        Latitude = gps.location.lat();
        Longitude = gps.location.lng();
    }
    sprintf(gpsTime, "%02d:%02d:%02d", gps.time.hour(), gps.time.minute(), gps.time.second());
    gpsAltitude = gps.altitude.meters();
    gpsSatellite = gps.satellites.value();
}

void get_BME_flight() {
    Temp = bme280.getTemperature();
    float a = bme280.calcAltitude(bme280.getPressure());
    Altitude = float(a - refAltitude);
    if (Altitude >= Peak) {
        Peak = Altitude;
    }
}

void get_BME_simulation() {
    Temp = bme280.getTemperature();
    float a = bme280.calcAltitude(simPressure);
    Altitude = float(a - refAltitude);
    if (Altitude >= Peak) {
        Peak = Altitude;
    }
}

void get_time() {
    sprintf(missionTime, "%02d:%02d:%02d", hour(), minute(), second());
}

void get_battery() {
    //  Voltage = (((analogRead(voldivpin) * 0.00080566406)*(4000))/1000)+0.16;
    // float detected_voltage = (map(analogRead(voldivpin), 0, 1023, 0, 3.3));
    float detected_voltage = (analogRead(voldivpin) * 0.00080566406);
    Voltage = detected_voltage * ((R1_Ohm + R2_Ohm) / R2_Ohm);
}

void inMission() {
    time1 = millis();
    if (time1 - time0 >= 990) {
        Serial.println("inMission");
        get_time();
        get_gps();
        get_battery();
        if (SIM) {
            get_BME_simulation();
        } else {
            get_BME_flight();
        }
        switch (state) {
            case 0:
                State = "PRELAUNCH";
                if (Altitude >= 10) {
                    state = 1;
                }
                break;
            case 1:
                State = "LAUNCH";
                if (Peak - Altitude >= 30 && Altitude > 600) {
                    state = 2;
                }
                break;
            case 2:
                State = "Parachute Deployed";
                if (Altitude <= 410 && Altitude > 390) {
                    state = 3;
                    P = 'R';
                    Serial3.print("CMD,1022,SECOND PARACHUTE,ON\r");
                    servo2.write(180);
                    // delay(1000);
                    // servo2.write(0);
                }
                break;
            case 3:
                State = "Payload Deployment Initiated";
                if (Altitude <= 310 && Altitude > 290) {
                    state = 4;
                    P = 'R';
                    Serial5.print("CMD,1022,PAYLOAD,ON\r");
                    servo1.write(90);
                    // delay(1000);
                    // servo2.write(0);
                }
                break;
            case 4:
                State = "RELEASED";
                servo2.write(180);
                if (Altitude <= 5 && Altitude >= -5) {
                    state = 5;
                    State = "LAND";
                }
                break;
            case 5:
                while (true) {
                    digitalWrite(buzzer, HIGH);
                    delay(500);
                    digitalWrite(buzzer, LOW);
                    delay(500);
                }
                break;
        }
        telemetry = teamId + "," + missionTime + "," + String(Packet) + ",C," + Mode + "," + String(P) + "," + String(Altitude, 2) + "," + String(Temp, 2) + "," + String(Voltage) + "," + String(gpsTime) + "," + String(Latitude, 6) + "," + String(Longitude, 6) + "," + String(gpsAltitude) + "," + String(gpsSatellite) + "," + State + "," + cmdEcho + "\r";
        File file = SD.open(FileC, FILE_WRITE);
        if (file) {
            file.println(telemetry);
            file.close();
        }
        Serial.println(telemetry);
        Serial3.print(telemetry);
        EEPROM.put(recovPkg, Packet);
        EEPROM.put(recovState, state);
        Packet++;
        time0 = time1;
    }
}

void emergency(String cmd) {
    if (cmd == "FORCE,PARADEPLOY") {
        servo1.write(90);
        // delay(1000);
        servo1.write(0);
    } else if (cmd == "FORCE,TIMEDPL") {
        servo2.write(90);
        // delay(1000);
        servo2.write(0);
    } else if (cmd == "FORCE,BEGINPL") {
        state = 3;
        servo1.write(90);
        delay(1000);
    } else if (cmd == "FORCE,STOPPL") {
        servo1.write(0);
    } else if (cmd == "FORCE,RESETCAMPOS") {
        servo1.write(90);
        servo2.write(90);
    }
}

void loop() {
    poll_time1 = millis() + 125;
    if (state >= 3 && poll_time1 - poll_time0 >= 250) {
        poll_time0 = poll_time1;
        Serial.println("Pinging payload");
        Serial4.print("CMD,1022,TP,POLL\r\r\r");
    }
    if (Serial.available()) {
        for (int i = 0; i < 1; i++) {  // debug only
            digitalWrite(buzzer, HIGH);
            delay(50);
            digitalWrite(buzzer, LOW);
            delay(50);
        }
        while (Serial.available()) {
            char inchar = Serial.read();
            if (inchar == '$' or inchar == '\r') {
                cmd = cmd.trim();
                cmd = (cmd.substring(9));
                Serial.println("GS:" + cmd);
                doCommand(cmd);
                cmd = "";
            } else {
                cmd += inchar;
            }
        }
    }
    if (Serial3.available()) {
        for (int i = 0; i < 1; i++) {  // debug only
            digitalWrite(buzzer, HIGH);
            delay(50);
            digitalWrite(buzzer, LOW);
            delay(50);
        }
        String cmd = Serial3.readStringUntil('\r');
        if (cmd == "\r") return;
        cmd = cmd.trim();
        cmd = (cmd.substring(9));
        Serial.println("GS:" + cmd);
        doCommand(cmd);
        cmd = "";
    }
    if (Serial4.available()) {
        String tp = Serial4.readStringUntil('\r');
        if (tp == "\r") return;
        tp = tp.trim();
        Serial3.print(tp + "\r");
        Serial.println("TP: " + tp);
        StatePayload++;
        File file = SD.open(FileC, FILE_WRITE);
        if (file) {
            file.println(tp);
            file.close();
        }
        tp = "";
    }
    delay(10);
    if (cxON) {
        inMission();
    }
}