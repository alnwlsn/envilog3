#include <Arduino.h>
#include <EEPROM.h>
#include <SPI.h>
#include <avr/sleep.h>
#include <avr/wdt.h>

#include "BH1730.h"
#include "SHT31.h"
#include "SPIMemory/src/SPIMemory.h"
#include "SparkFunBME280.h"
#include "Wire.h"

#define intClk 2
#define int1 1  // unused
#define buttonOption 4
#define ledGreen 7
#define ledYellow 6
#define ledRed 5

struct frame_t {  // structure that stores each frame (32 bytes)
    uint32_t count;

    uint8_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;

    int16_t temperatureBME;
    int16_t temperatureSHT;
    int16_t temperatureRTC;
    int16_t humidityBME;
    int16_t humiditySHT;
    float pressureBME;
    uint16_t lightVis;
    uint16_t lightIr;
    uint16_t batteryADC;
    uint16_t avrTherm;
};
frame_t frame;
void printBin(byte temp) {
    Serial.print(F("0b"));
    for (uint8_t y = 0; y <= 7; y++) {
        if (temp & (0x80)) {
            Serial.print(F("1"));
        } else {
            Serial.print(F("0"));
        }
    }
}
void printHex(byte temp) {
    if (temp < 0x10) {
        Serial.print('0');
    }
    Serial.print(temp, HEX);
}

class DS3231 {  // my own custom lib for control of the DS3231
    uint8_t RTC_ADDR = 0x68;

    uint8_t ztimeSecond;
    uint8_t ztimeMinute;
    uint8_t ztimeHour;
    uint8_t ztimeDay;
    uint8_t ztimeMonth;
    uint8_t ztimeYear;
    uint8_t ztimeWeekday;

    uint8_t za1rmb;  // alarm register bits - set first, then set the time, day, month, etc to get this value into the clock
    uint8_t za2rmb;

   public:
    DS3231() {}

    static uint8_t bcd2bin(uint8_t val) { return val - 6 * (val >> 4); }
    static uint8_t bin2bcd(uint8_t val) { return val + 6 * (val / 10); }
    uint8_t read_register(uint8_t reg) {
        Wire.beginTransmission(RTC_ADDR);
        Wire.write((uint8_t)reg);
        Wire.endTransmission();
        Wire.requestFrom(RTC_ADDR, (uint8_t)1);
        return Wire.read();
    }
    void write_register(uint8_t reg, uint8_t val) {
        Wire.beginTransmission(RTC_ADDR);
        Wire.write((uint8_t)reg);
        Wire.write((uint8_t)val);
        Wire.endTransmission();
    }

    void setTimeSecond(uint8_t temp) { write_register(0x00, bin2bcd(temp)); }
    void setTimeMinute(uint8_t temp) { write_register(0x01, bin2bcd(temp)); }
    void setTimeHour(uint8_t temp) { write_register(0x02, bin2bcd(temp)); }
    void setTimeDay(uint8_t temp) { write_register(0x04, bin2bcd(temp)); }
    void setTimeMonth(uint8_t temp) { write_register(0x05, bin2bcd(temp)); }
    void setTimeYear(int temp) { write_register(0x06, bin2bcd(temp)); }
    void validateClock() {
        uint8_t bstat = read_register(0x0F);
        bstat &= ~0x80;
        write_register(0x0F, bstat);
    }  // Clears OSF
    bool lostPower(void) { return (read_register(0x0f) >> 7); }
    void getTime() {
        Wire.beginTransmission(RTC_ADDR);  // set to location 0 (seconds)
        Wire.write((uint8_t)0);
        Wire.endTransmission();
        Wire.requestFrom(RTC_ADDR, (uint8_t)7);
        ztimeSecond = bcd2bin(Wire.read() & 0x7F);
        ztimeMinute = bcd2bin(Wire.read());
        ztimeHour = bcd2bin(Wire.read());
        Wire.read();
        ztimeDay = bcd2bin(Wire.read());
        ztimeMonth = bcd2bin(Wire.read());
        ztimeYear = bcd2bin(Wire.read());
    }
    uint8_t csecond() { return ztimeSecond; }
    uint8_t cminute() { return ztimeMinute; }
    uint8_t chour() { return ztimeHour; }
    uint8_t cday() { return ztimeDay; }
    uint8_t cmonth() { return ztimeMonth; }
    uint8_t cyear() { return ztimeYear; }
    uint8_t cweekday() { return ztimeWeekday; }

    void setAlarm1RMB(uint8_t temp) { za1rmb = temp; }
    void setAlarm1Second(uint8_t temp) { write_register(0x07, ((1 & (za1rmb >> 0)) << 7) + bin2bcd(temp)); }  // set alarm bcd and bit 0 of alarm 1 register mask bit
    void setAlarm1Minute(uint8_t temp) { write_register(0x08, ((1 & (za1rmb >> 1)) << 7) + bin2bcd(temp)); }
    void setAlarm1Hour(uint8_t temp) { write_register(0x09, ((1 & (za1rmb >> 2)) << 7) + bin2bcd(temp)); }
    void setAlarm1Weekday(uint8_t temp) { write_register(0x0a, (0b01000000) | (((1 & (za1rmb >> 3)) << 7) + bin2bcd(temp))); }
    void setAlarm1Day(uint8_t temp) { write_register(0x0a, (~0b01000000) & (((1 & (za1rmb >> 3)) << 7) + bin2bcd(temp))); }  // if day is set, tell alarm to use it, otherwise, use weekday

    void useAlarm1onINT() {
        uint8_t bcont = read_register(0x0e);
        bcont |= 0b00000001;
        write_register(0x0e, bcont);
    }  // sets A1IE
    void dontUseAlarm1onINT() {
        uint8_t bcont = read_register(0x0e);
        bcont &= ~0b00000001;
        write_register(0x0e, bcont);
    }  // clears A1IE
    void resetAlarm1() {
        uint8_t bstat = read_register(0x0f);
        bstat &= ~0b00000001;
        write_register(0x0f, bstat);
    }  // clears alarm 1 flag
    float getTemp() {
        int8_t temp_msb, temp_lsb;
        Wire.beginTransmission(RTC_ADDR);
        Wire.write(0x11);
        Wire.endTransmission();
        Wire.requestFrom(RTC_ADDR, (uint8_t)2);
        temp_msb = Wire.read();
        temp_lsb = (Wire.read() >> 6) & 0x03;
        Wire.endTransmission();
        if (temp_msb & 0b10000000) {  // check if negative number
            temp_msb ^= 0b11111111;
            temp_msb += 0x1;
            return (-1.0 * ((float)temp_msb) + ((float)temp_lsb * 0.25));
        } else {
            return ((float)temp_msb + ((float)temp_lsb * 0.25));
        }
    }
    void printRegs() {
        Serial.print(F("DS3231 registers:\r\n"));
        Serial.print(F("ADR DAT(BIN)\r\n"));
        for (byte j = 0; j <= 0x12; j++) {
            printHex(j);
            Serial.print(' ');
            printBin(read_register(j));
            Serial.println();
        }
        Serial.print(F("\r\n"));
    }
    void printTime() {
        getTime();
        Serial.print(F("Current time: "));
        if (cyear() < 10) {
            Serial.print('0');
        }
        Serial.print(cyear());
        Serial.print('/');
        if (cmonth() < 10) {
            Serial.print('0');
        }
        Serial.print(cmonth());
        Serial.print('/');
        if (cday() < 10) {
            Serial.print('0');
        }
        Serial.print(cday());
        Serial.print(' ');
        if (chour() < 10) {
            Serial.print('0');
        }
        Serial.print(chour());
        Serial.print(':');
        if (cminute() < 10) {
            Serial.print('0');
        }
        Serial.print(cminute());
        Serial.print(':');
        if (csecond() < 10) {
            Serial.print('0');
        }
        Serial.println(csecond());
    }
};

DS3231 rtc;
BH1730 lightSensor;
#define SHT31_ADDRESS 0x44
SHT31 sht;
BME280 bme;
SPIFlash flash;

void msg() {  // reads EEPROM stored message
    Serial.print(F("*******\r\n"));
    char rxb;
    int eindex = 0;
    while (1) {
        rxb = EEPROM.read(eindex);
        if (rxb == 13) {
            Serial.write(10);
        }
        if (rxb == '\\') {
            break;
        }
        Serial.write(rxb);
        if (eindex >= 1023) {
            break;
        }
        eindex++;
    }
    Serial.print(F("\r\n*******\r\n"));
}

void wakeISR() {  // just wakes the chip and disables interrupts until next time
    sleep_disable();
    detachInterrupt(0);
    detachInterrupt(1);
}
void printFrameHeader() {
    Serial.print(F("Count\tYear\tMonth\tDay\tHour\tMinute\tSecond\tAVR Temp (ADC)\tDS3232 Temp (C)\tBME Temp (C)\tSHT Temp (C)\tBME RH (%)\tSHT RH (%)\tBME Pressure (pa)\tLight Vis (raw)\tLight IR (raw)\tBatt (ADC)\r\n"));
}
void printFrame(frame_t current) {  // prints one line of frame data
    Serial.print(current.count);
    Serial.print('\t');
    Serial.print(current.year);
    Serial.print('\t');
    Serial.print(current.month);
    Serial.print('\t');
    Serial.print(current.day);
    Serial.print('\t');
    Serial.print(current.hour);
    Serial.print('\t');
    Serial.print(current.minute);
    Serial.print('\t');
    Serial.print(current.second);
    Serial.print('\t');
    Serial.print(current.avrTherm);
    Serial.print('\t');
    Serial.print(current.temperatureRTC / 100.0);
    Serial.print('\t');
    Serial.print(current.temperatureBME / 100.0);
    Serial.print('\t');
    Serial.print(current.temperatureSHT / 100.0);
    Serial.print('\t');
    Serial.print(current.humidityBME / 100.0);
    Serial.print('\t');
    Serial.print(current.humiditySHT / 100.0);
    Serial.print('\t');
    Serial.print(current.pressureBME);
    Serial.print('\t');
    Serial.print(current.lightVis);
    Serial.print('\t');
    Serial.print(current.lightIr);
    Serial.print('\t');
    Serial.print(current.batteryADC);
    Serial.println();
}
void printProjectHeader() {
    Serial.println(F("TIME CAPSULE LONG DURATION ENVIROMENTAL LOGGER - v10 (2022-04-24) - Alan J. WIlson"));
}
frame_t wakeupMeasure() {  // wake up from sleep, initialize and do the measurements
    Wire.begin();
    SPI.begin();
    ADCSRA = 134;
    ADMUX = (_BV(REFS1) | _BV(REFS0) | _BV(MUX3));  // for AVR therm
    ADCSRA |= _BV(ADEN);
    pinMode(ledGreen, OUTPUT);
    pinMode(ledRed, OUTPUT);
    pinMode(ledYellow, OUTPUT);
    rtc.resetAlarm1();
    flash.powerUp();

    bme.settings.commInterface = I2C_MODE;
    bme.settings.I2CAddress = 0x76;
    bme.settings.runMode = 3;  // 3 for normal mode (ON), 0 for sleep mode (OFF)
    bme.settings.tStandby = 0;
    bme.settings.filter = 0;
    bme.settings.tempOverSample = 8;
    bme.settings.pressOverSample = 8;
    bme.settings.humidOverSample = 8;
    bme.begin();

    sht.begin();
    sht.reset();
    sht.read();

    ADCSRA |= _BV(ADSC);  // for AVR therm

    lightSensor.begin();
    lightSensor.reset();
    lightSensor.setGain(GAIN_X1);
    lightSensor.measure();

    while (bit_is_set(ADCSRA, ADSC))
        ;  // avr therm

    // collect sensor info
    frame_t current;
    current.avrTherm = ADCW;  // avr therm
    current.temperatureRTC = rtc.getTemp() * 100;
    current.temperatureSHT = sht.getTemperature() * 100;
    current.humiditySHT = sht.getHumidity() * 100;
    current.temperatureBME = bme.readTempC() * 100;
    current.humidityBME = bme.readFloatHumidity() * 100;
    current.pressureBME = bme.readFloatPressure();
    current.lightVis = lightSensor.readVis();
    current.lightIr = lightSensor.readIr();
    current.batteryADC = analogRead(A7);
    return current;
}
uint32_t findIndex() {  // finds next blank area in flash for next index
    uint32_t index = 0;
    while (1) {
        if (!flash.readAnything(index * sizeof(frame), frame)) {
            Serial.println(F("Read failed!"));
            break;
        }
        if (frame.count == 4294967295) {
            break;
        }
        index++;
    }
    return index;
}
bool saveData(uint32_t index, frame_t frame) {
    // Write to the flash chip. The retries are likely unnessacary, but I added them anyways.
    byte retry = 10;
    while (retry > 0) {
        retry--;
        if (flash.writeAnything(index * sizeof(frame), frame)) {
            return true;
        }
        delay(50);
    }
    return false;
}
void frameTimestamp(struct frame_t* current) {  // timestamps frame with current time
    rtc.getTime();
    current->year = rtc.cyear();
    current->month = rtc.cmonth();
    current->day = rtc.cday();
    current->hour = rtc.chour();
    current->minute = rtc.cminute();
    current->second = rtc.csecond();
}
uint32_t frameIndex = 0;
uint32_t frameCount = 0;
uint8_t nextMinuteTrigger = 0;

//***************************Wilson Command Console******************************
#include <string.h>
#define serbuflen 50             // total length of single line of input
#define paramMax 10              // number of parameters that will be split
char serbuf[serbuflen + 1];      // buffer for storing serial input text
unsigned short serbufIndex = 0;  // index pointer for serbuf
char* param[paramMax];           // stores pointers to parameters
short params;                    // stores number of parameters got
void consoleProcess() {          // splits out the parameters
    char* tmp;
    params = -1;
    tmp = strtok(serbuf, " ");
    if (tmp != NULL) {
        params = 0;
        param[params] = tmp;
    } else {
        return;
    }
    while (tmp != NULL) {
        tmp = strtok(NULL, " ");
        if (tmp != NULL) {
            params++;
            if (params >= paramMax) {
                params = paramMax - 1;
            }
            param[params] = tmp;
        }
    }
}
void consoleOperate() {                           // act on the parameters we got
    if (0 == strcmp_P(param[0], PSTR("time"))) {  // sets the RTC
        Serial.print(F("\r\n"));
        if (params <= 1) {
            rtc.printTime();
        } else {
            rtc.setTimeYear(strtol(param[1], NULL, 10));
            rtc.setTimeMonth(strtol(param[2], NULL, 10));
            rtc.setTimeDay(strtol(param[3], NULL, 10));
            rtc.setTimeHour(strtol(param[4], NULL, 10));
            rtc.setTimeMinute(strtol(param[5], NULL, 10));
            rtc.setTimeSecond(strtol(param[6], NULL, 10));
            rtc.resetAlarm1();
            rtc.validateClock();
            rtc.printTime();
        }
        // Serial.print(F("\r\n1st parameter (int/prn in Hex)"));
        //   Serial.println(strtol(param[1], NULL, 16), HEX);
        // Serial.print(F("2nd parameter (int/prn in Dec)"));
        //   Serial.println(strtol(param[2], NULL, 10));
        // Serial.print(F("3rd parameter (int/prn in Bin)"));
        //   Serial.println(strtol(param[3], NULL, 2), BIN);
    }
    if (0 == strcmp_P(param[0], PSTR("speed"))) {
        if (params >= 1) {
            Serial.begin(strtol(param[1], NULL, 10));
        }
    }
    if (0 == strcmp_P(param[0], PSTR("dump"))) {
        unsigned long start = 0;
        if (params >= 1) {
            start = strtol(param[1], NULL, 10);
        }
        while (Serial.available()) {
            Serial.read();
        }  // empty serial register
        Serial.println();
        printFrameHeader();
        for (unsigned long t = start; t < (flash.getCapacity() / (int)sizeof(frame)); t++) {
            frame_t dumpFrame;
            flash.readAnything(t * sizeof(frame), dumpFrame);
            printFrame(dumpFrame);
            if (digitalRead(buttonOption) == 0) {
                return;
            }
            if (Serial.available()) {
                return;
            }
        }
    }
    if (0 == strcmp_P(param[0], PSTR("erase"))) {
        Serial.print(F("\r\n"));
        flash.eraseChip();
        frameIndex = 0;
        frameCount = 0;
    }
    if (0 == strcmp_P(param[0], PSTR("sensors"))) {
        Serial.print(F("\r\n"));
        frame_t current = wakeupMeasure();
        frameTimestamp(&current);
        current.count = frameCount;
        printFrame(current);
    }
    if (0 == strcmp_P(param[0], PSTR("reset"))) {
        Serial.print(F("\r\n"));
        wdt_enable(WDTO_15MS);
        for (;;) {
        }
    }
    if (0 == strcmp_P(param[0], PSTR("new"))) {
        Serial.print(F("\r\n"));
        char rxb = 0;
        int eindex = 0;
        while (1) {
            if (Serial.available()) {
                rxb = Serial.read();
                if (rxb == 13) {
                    Serial.write(10);
                }
                EEPROM.write(eindex, rxb);
                if (rxb == '\\') {
                    break;
                }
                Serial.write(rxb);
                if (eindex >= 1023) {
                    break;
                }
                eindex++;
            }
        }
    }
    if (0 == strcmp_P(param[0], PSTR("msg"))) {
        Serial.print(F("\r\n"));
        msg();
    }
    if (0 == strcmp_P(param[0], PSTR("?"))) {
        Serial.print(F("\r\ntime [YY MM DD HH MM SS] - show / set current RTC time\r\nspeed [baudrate]         - set serial console baudrate\r\ndump                     - dump saved Flash data\r\nerase                    - erase flash chip\r\nsensors                  - read all sensors and print data\r\nreset                    - reset AVR\r\nnew                      - write new EEPROM message\r\nmsg                      - display current EEPROM message\r\n"));
    }
}
void consoleRun() {
    while (Serial.available()) {
        serbuf[serbufIndex] = Serial.read();
        if (serbuf[serbufIndex] == 13) {
            serbuf[serbufIndex] = 0;
            serbufIndex = 0;
            consoleProcess();  // split the parameters
            if (params >= 0) {
                consoleOperate();  // operate the console given parameters
                                   // for(int y=0; y<=params; y++){printf("%d - %s\r\n",y,param[y]);}
            }
            Serial.print(F("\r\n>"));
        } else if (serbuf[serbufIndex] == 8 || serbuf[serbufIndex] == 127) {
            if (serbufIndex != 0) {
                serbufIndex--;
                Serial.write(8);
                Serial.write(32);
                Serial.write(8);
            }
        } else {
            Serial.write(serbuf[serbufIndex]);
            serbufIndex++;
            if (serbufIndex >= serbuflen) {
                serbufIndex = serbuflen;
            }
        }
    }
}

void setup() {
    Serial.begin(115200);
    flash.begin(MB(16));
    wakeupMeasure();  // init hardware
    pinMode(buttonOption, INPUT);
    if (digitalRead(buttonOption) == 0) {
        pinMode(ledYellow, OUTPUT);
        digitalWrite(ledYellow, HIGH);
        Serial.begin(9600);
        printProjectHeader();
        Serial.print(F("Welcome to the Command Console! (? for command list)\r\n>"));
        while (1) {
            consoleRun();
        }
    }
    printProjectHeader();
    Serial.print(F("Hold Option and press Reset for Command Console\r\n"));

    Serial.print(F("Flash capacity: "));
    Serial.print(flash.getCapacity());
    Serial.print(F(" bytes, "));
    Serial.print((flash.getCapacity() / (int)sizeof(frame)));
    Serial.print(F(" frames (frame size: "));
    Serial.print((int)sizeof(frame));
    Serial.print(F(")\r\nNext frame index: "));
    frameIndex = findIndex();
    Serial.println(frameIndex);
}
void loop() {
    // wakeup here initize stuff
    frame_t current = wakeupMeasure();
    frameTimestamp(&current);
    current.count = frameCount;

    Serial.begin(115200);
    Serial.println();

    if (frameIndex <= (flash.getCapacity() / (int)sizeof(frame))) {
        bool sd = saveData(frameIndex, current);
        if (sd) {
            Serial.print(F("Saved!\r\n"));
            digitalWrite(ledGreen, HIGH);
            frameIndex++;
            frameCount++;
        } else {
            Serial.print(F("Save FAILED!\r\n"));
            digitalWrite(ledRed, HIGH);
        }
    } else {
        Serial.print(F("flash FULL!\r\n"));
        digitalWrite(ledRed, HIGH);
    }

    rtc.printTime();
    printFrame(current);

    //Set RTC alarm for the next trigger
    nextMinuteTrigger = rtc.cminute() + 6;
    if(nextMinuteTrigger >= 60){
        nextMinuteTrigger = 0;
    }
    //ALARM 1 REGISTER MASK BITS - 0b1110 trips timer when seconds match. 0b1100 trips timer when minutes and seconds match.
    rtc.setAlarm1RMB(0b1100);
    rtc.setAlarm1Day(27);
    rtc.setAlarm1Hour(11);
    rtc.setAlarm1Minute(nextMinuteTrigger);
    rtc.setAlarm1Second(0);
    rtc.useAlarm1onINT();
    rtc.resetAlarm1();

    // sleep here
    bme.setMode(0);
    sht.reset();
    lightSensor.reset();
    flash.powerDown();
    digitalWrite(ledRed, LOW);
    digitalWrite(ledGreen, LOW);
    digitalWrite(ledYellow, LOW);
    pinMode(buttonOption, INPUT);
    pinMode(ledRed, INPUT);
    pinMode(ledGreen, INPUT);
    pinMode(ledYellow, INPUT);
    Serial.end();
    Wire.end();
    SPI.end();
    pinMode(0, INPUT);
    pinMode(1, INPUT);
    delay(1);
    ADCSRA = 0;  // Shutdown procedure from the Nick Gammon fourms
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_enable();
    noInterrupts();                        // stop any interrups before sleeping
    attachInterrupt(0, wakeISR, FALLING);  // goto 'wake' on interrupt (using clock alarm)
    EIFR = bit(INTF0);                     // clear flag for interrupt 0
    interrupts();                          // start interrups and
    sleep_cpu();                           // then immediately sleep
}
