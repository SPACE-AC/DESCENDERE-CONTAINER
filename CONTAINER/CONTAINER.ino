#include <EEPROM.h>
#include <SD.h>
#include <SPI.h>
#include <Seeed_BME280.h>
#include <Servo.h>
#include <TimeLib.h>
#include <TinyGPSPlus.h>

// CONFIG - START

#define LED1_PIN 4
#define LED2_PIN 5
#define BUZZER_PIN 3

#define SERVO_PARA_PIN 4
#define SERVO_BREAK_PIN 5

#define SD_CS_PIN 10

#define voldivpin 21
#define R1_OHM 3000.0F
#define R2_OHM 1740.0F

#define xbeeGS Serial3
#define xbeeTP Serial4

// CONFIG - END

TinyGPSPlus gps;
BME280 bme280;
time_t RTCTime;
Servo servoParachute;
Servo servoBreak;

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

float Apogee = -2147483648;
int refAltitude = 0;
int state = 0;
int simPressure = 0;

String telemetry = "";
String tp = "";
String cmd = "";

String teamId = "1022";
char missionTime[32] = "xx:xx:xx";
int Packet = 0;
char Mode = 'F';
char P = 'N';
float Altitude = 0;
float Temp = 0;
float Voltage = 0;
char gpsTime[32] = "xx:xx:xx";
double Latitude = 0;
double Longitude = 0;
float gpsAltitude = 0;
int gpsSatellite = 0;
String State = "PRELAUNCH";
int StatePayload = 0;
String cmdEcho = "N/A";
float breakDegree = 180;
long breakStartAt = -1;

void recovery();
void get_file();
time_t getTeensy3Time();

void beep(const unsigned int& count, const unsigned int& delay_ms = 100) {
    for (unsigned int i = 0; i < count; i++) {
        digitalWrite(BUZZER_PIN, HIGH);
        delay(delay_ms);
        digitalWrite(BUZZER_PIN, LOW);
        delay(delay_ms);
    }
}

void setup() {
    Serial.begin(9600);
    Serial.println("Initiating ...");
    Serial2.begin(9600);
    xbeeGS.begin(115200);  // GCS
    xbeeTP.begin(115200);  // Payload

    servoParachute.attach(SERVO_PARA_PIN);  // set port
    servoBreak.attach(SERVO_BREAK_PIN);     // set port
    pinMode(LED1_PIN, OUTPUT);
    pinMode(LED2_PIN, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(voldivpin, INPUT);
    pinMode(13, OUTPUT);
    digitalWrite(13, HIGH);

    digitalWrite(BUZZER_PIN, HIGH);
    delay(1000);
    digitalWrite(BUZZER_PIN, LOW);
    delay(500);

    setSyncProvider(getTeensy3Time);

    if (bme280.init())
        Serial.println("✔ SUCCEED: BME280");
    else {
        Serial.println("[FAILED] Unable to set up BME280!");
        beep(1);
    }
    delay(500);
    if (SD.begin(SD_CS_PIN))
        Serial.println("✔ SUCCEED: SD card reader");
    else {
        Serial.println("[FAILED] SD card reader initialization failed!");
        beep(5);
    }

    delay(1000);
    recovery();

    servoBreak.write(0);
    delay(3000);
    servoBreak.write(180);

    // ! TESTING ONLY
    // state = 3;
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
    beep(3);
}

void doCommand(String cmd) {
    if (cmd == "CX,ON") {
        beep(2);
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
        beep(1);
        Serial.println("CXOFF");
        cxON = false;
        cmdEcho = "CXOFF";
        Mode = 'F';
        for (int i = 0; i < EEPROM.length(); i++) {
            EEPROM.write(i, 0);
        }
        servoParachute.write(90);
        servoBreak.write(90);
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
    Latitude = gps.location.lat();
    Longitude = gps.location.lng();
    sprintf(gpsTime, "%02d:%02d:%02d", gps.time.hour(), gps.time.minute(), gps.time.second());
    gpsAltitude = gps.altitude.meters();
    gpsSatellite = gps.satellites.value();
}

void get_BME_flight() {
    Temp = bme280.getTemperature();
    float a = bme280.calcAltitude(bme280.getPressure());
    Altitude = float(a - refAltitude);
    if (Altitude >= Apogee) {
        Apogee = Altitude;
    }
}

void get_BME_simulation() {
    Temp = bme280.getTemperature();
    float a = bme280.calcAltitude(simPressure);
    Altitude = float(a - refAltitude);
    if (Altitude >= Apogee) {
        Apogee = Altitude;
    }
}

void get_time() {
    sprintf(missionTime, "%02d:%02d:%02d", hour(), minute(), second());
}

void get_battery() {
    //  Voltage = (((analogRead(voldivpin) * 0.00080566406)*(4000))/1000)+0.16;
    // float detected_voltage = (map(analogRead(voldivpin), 0, 1023, 0, 3.3));
    float apparentVoltage = analogRead(voldivpin) * 3.3 / 1023.0;
    Voltage = apparentVoltage * ((R1_OHM + R2_OHM) / R2_OHM);
    if (Voltage < 5.3) {
        beep(5, 25);
    }
}

double mapf(double x, double in_min, double in_max, double out_min, double out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void inMission() {
    time1 = millis();
    if (time1 - time0 >= 990) {
        //        Serial.println("inMission");
        get_time();
        get_gps();
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
                if (Apogee - Altitude >= 30 && Altitude >= 670) {
                    state = 2;
                }
                break;
            case 2:
                State = "APOGEE";
                if (Altitude <= 410 && Altitude > 390) {
                    state = 3;
                    xbeeGS.print("CMD,1022,SECOND PARACHUTE,ON\r");

                    // rotate 90 degrees on continuous servo using delay
                    servoParachute.write(26);
                    delay(173);
                    servoParachute.write(90);
                }
                break;
            case 3:
                State = "PARADEPLOY";
                if (Altitude <= 310 && Altitude > 290) {
                    state = 4;
                    P = 'R';
                    breakStartAt = millis();
                    xbeeTP.print("ON");
                }
                break;
            case 4:
                State = "TPDEPLOY";
                if (Altitude <= 5 && Altitude >= -5) {
                    state = 5;
                    cxON = false;
                }
                break;
            case 5:
                State = "LAND";
                digitalWrite(BUZZER_PIN, HIGH);
                delay(500);
                digitalWrite(BUZZER_PIN, LOW);
                delay(500);
                break;
        }
        telemetry = teamId + "," + missionTime + "," + String(Packet) + ",C," + Mode + "," + String(P) + "," + String(Altitude, 2) + "," + String(Temp, 2) + "," + String(Voltage) + "," + String(gpsTime) + "," + String(Latitude, 6) + "," + String(Longitude, 6) + "," + String(gpsAltitude) + "," + String(gpsSatellite) + "," + State + "," + cmdEcho + "\r";
        File file = SD.open(FileC, FILE_WRITE);
        if (file) {
            file.println(telemetry);
            file.close();
        }
        Serial.println(telemetry);
        xbeeGS.print(telemetry);
        EEPROM.update(recovPkg, Packet);
        Packet++;
        time0 = time1;
    }
}

void emergency(String cmd) {
    if (cmd == "FORCE,PARADEPLOY") {
        servoParachute.write(26);
        delay(173);
        servoParachute.write(90);
    } else if (cmd == "FORCE,SEQUENCE") {
        breakStartAt = millis();
    } else if (cmd == "FORCE,HALT") {
        breakStartAt = -1;
    } else if (cmd == "FORCE,RELEASE") {
        breakDegree = 0;
    } else if (cmd == "FORCE,BREAK") {
        breakDegree = 180;
    } else if (cmd == "FORCE,RESETCAM") {
        xbeeTP.print("CMD,1022,FORCE,RESETCAM\r");
    } else if (cmd == "FORCE,CALCAM") {
        xbeeTP.print("CMD,1022,FORCE,CALCAM\r");
    } else if (cmd == "FORCE,POLL") {
        Serial.println("Pinging payload");
        xbeeTP.print("POLL\r\r\r");
    } else if (cmd == "FORCE,STATE0")
        state = 0;
    else if (cmd == "FORCE,STATE1")
        state = 1;
    else if (cmd == "FORCE,STATE2")
        state = 2;
    else if (cmd == "FORCE,STATE3")
        state = 3;
    else if (cmd == "FORCE,STATE4")
        state = 4;
    else if (cmd == "FORCE,STATE5")
        state = 5;
}

bool reachTerminator = false;
void loop() {
    // if (xbeeTP.available()) {
    //     Serial.print(xbeeTP.readString(5));
    // }
    if (xbeeTP.available()) {
        // while (xbeeTP.available()) {
        //     char inchar = xbeeTP.read();
        //     tp += inchar;
        //     Serial.print(inchar);
        //     if (inchar == '\r') {
        //         xbeeGS.print(tp);
        //         Serial.println(tp);
        //         tp = "";
        //     }
        // }

        // while (xbeeTP.available()) {
        //     char inchar = xbeeTP.read();
        //     if (reachTerminator && inchar != '$') {
        //         xbeeGS.print(tp);
        //         Serial.println(tp);
        //         Serial.println("DONE");
        //         tp = "";
        //         tp += inchar;
        //         reachTerminator = false;
        //         continue;
        //     }
        //     if (inchar == '$') {
        //         inchar = '\r';
        //         reachTerminator = true;
        //     }
        //     tp += inchar;
        // }

        String in = xbeeTP.readStringUntil('$');
        xbeeGS.print(in + '\r');
        Serial.println(in);
    }
    poll_time1 = millis() + 125;
    while (Serial2.available())
        gps.encode(Serial2.read());
    if (state >= 3 && poll_time1 - poll_time0 >= 250) {
        poll_time0 = poll_time1;
        // Serial.println("Pinging payload");
        xbeeTP.print("POLL\r\r\r");
    }
    if (Serial.available()) {
        for (int i = 0; i < 1; i++) {  // debug only
            digitalWrite(BUZZER_PIN, HIGH);
            delay(50);
            digitalWrite(BUZZER_PIN, LOW);
            delay(50);
        }
        while (Serial.available()) {
            char inchar = Serial.read();
            if (inchar == '\n') {
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
    if (xbeeGS.available()) {
        beep(1);
        while (xbeeGS.available()) {
            String cmd = xbeeGS.readStringUntil('\r');
            if (cmd == "\r") return;
            cmd = cmd.trim();
            cmd = (cmd.substring(9));
            Serial.println("GS:" + cmd);
            doCommand(cmd);
            cmd = "";
        }
    }

    if (breakStartAt != -1) {
        float t = (millis() - breakStartAt) / 1000.0;
        if (t <= 0)
            breakDegree = 180;
        else if (t <= 0.1)
            breakDegree = mapf(t, 0, 0.1, 180, 0);
        else if (t <= 0.6)
            breakDegree = 0;
        else if (t <= 0.9)
            breakDegree = mapf(t, 0.6, 0.9, 0, 180);
        else if (t <= 1.9)
            breakDegree = 180;
        else if (t <= 20.9) {
            float t_loop = (int((t - 1.9) * 100) % 190) / 100.0;
            if (t_loop <= 1)
                breakDegree = 180;
            else if (t_loop <= 1.1)
                breakDegree = mapf(t_loop, 1, 1.1, 180, 0);
            else if (t_loop <= 1.6)
                breakDegree = 0;
            else if (t_loop <= 1.9)
                breakDegree = mapf(t_loop, 1.6, 1.9, 0, 180);
        } else {
            breakStartAt = -1;
        }
    }

    servoBreak.write(breakDegree);
    // if (xbeeTP.available()) {
    // String tp = xbeeTP.readStringUntil('\r');
    // if (tp == "\r") return;
    // tp = tp.trim();
    // Serial.println("gonna read");

    // char tpByte[10];
    // const unsigned short n_read = xbeeTP.readBytes(tpByte, 10);
    // // Serial.println("read");
    // for (unsigned short i = 0; i < n_read; i++) {
    //     Serial.print(tpByte[i]);
    //     xbeeGS.print(tpByte[i]);
    // }
    // // Serial.println("TP: " + tp);
    // // StatePayload++;
    // File file = SD.open(FileC, FILE_WRITE);
    // if (file) {
    //     file.println(tp);
    //     file.close();
    // }
    // tp = "";
    // }

    // delay(10);
    get_battery();
    if (cxON) {
        inMission();
    }
    EEPROM.update(recovState, state);
}