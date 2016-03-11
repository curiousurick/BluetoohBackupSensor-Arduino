//Final Project - Combination sonic distance sensor and bluetooth communication device
//                Sensor detects data on 10Hz intervals and sends the readings through SoftwareSerial (UART) to another device
//                Due to design limitations of the Bluetooth chip used, a middleman between two BlueFruit devices is required
//                In our project, we developed an iOS app based on the BlueFruit example, expanding it to accept the notifications
//                we are pushing from the arduino device, and responding accordingly. This sketch is code for the Arduino Sensor device
//
//Sensor Pin Setup - SensorPin ArduinoPin
// Hold LOW          1         22
// Send Interrupt    2         21
// Send Data         3         0
//                   6         Vcc(3.3v)
//                   7         GND


#include <Adafruit_Sensor.h>
#include <Adafruit_BLE.h>
#include <Adafruit_BluefruitLE_SPI.h>
#include <Adafruit_BluefruitLE_UART.h>
#include <Adafruit_LSM303_U.h>

#include <Arduino.h>
#include <SoftwareSerial.h>
#include "HardwareSerial.h"
#include <Wire.h> //used for accelerometer, default uses D0 and D1 for RX and TX

#include "BluefruitConfig.h"

/*  =========================================================================
    APPLICATION SETTINGS

        FACTORYRESET_ENABLE     Perform a factory reset when running this sketch
       
                                Enabling this will put your Bluefruit LE module
                            in a 'known good' state and clear any config
                            data set in previous sketches or projects, so
                                running this at least once is a good idea.
       
                                When deploying your project, however, you will
                            want to disable factory reset by setting this
                            value to 0.  If you are making changes to your
                                Bluefruit LE device via AT commands, and those
                            changes aren't persisting across resets, this
                            is the reason why.  Factory reset will erase
                            the non-volatile memory where config data is
                            stored, setting it back to factory default
                            values.
           
                                Some sketches that require you to bond to a
                            central device (HID mouse, keyboard, etc.)
                            won't work at all with this feature enabled
                            since the factory reset will clear all of the
                            bonding data stored on the chip, meaning the
                            central device won't be able to reconnect.



    -----------------------------------------------------------------------*/
#define FACTORYRESET_ENABLE         0
#define MINIMUM_FIRMWARE_VERSION    "0.5.6"
#define MODE_LED_BEHAVIOUR          "MODE"
#define PIN_TO_INTERRUPT            19
#define DEFAULT_INTERVAL            5000
#define TEMPERATURE_PIN             A1
/*=========================================================================*/

// Create the bluefruit object, either software serial...uncomment these lines

SoftwareSerial bluefruitSS = SoftwareSerial(BLUEFRUIT_SWUART_TXD_PIN, BLUEFRUIT_SWUART_RXD_PIN);

Adafruit_BluefruitLE_UART ble(bluefruitSS, BLUEFRUIT_UART_MODE_PIN,
                              BLUEFRUIT_UART_CTS_PIN, BLUEFRUIT_UART_RTS_PIN);


/* ...or hardware serial, which does not need the RTS/CTS pins. Uncomment this line */
// Adafruit_BluefruitLE_UART ble(BLUEFRUIT_HWSERIAL_NAME);

Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(12345);

// A small helper
void error(const __FlashStringHelper*err) {
    Serial.println(err);
    while (1);
}

void soundCameBack();
boolean reportAtInterval(unsigned long interval, int device);
void combineCommandAndFloat(char command[], float reading, char *buffer);
float readTemp();
float getFloat(char cString[]);
unsigned long getLong(char* start);

volatile boolean shouldRead = false;

void setup() {
    while (!Serial);  // required for Flora & Micro
    delay(500);

    Serial.begin(115200);
    Serial.println(F("Adafruit Bluefruit Command <-> Data Mode Example"));
    Serial.println(F("------------------------------------------------"));

    /* Initialise bluetooth module */
    Serial.print(F("Initialising the Bluefruit LE module: "));

    if ( !ble.begin(VERBOSE_MODE) )
    {
        error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
    }
    Serial.println( F("OK!") );

    if ( FACTORYRESET_ENABLE )
    {
        /* Perform a factory reset to make sure everything is in a known state */
        Serial.println(F("Performing a factory reset: "));
        if ( ! ble.factoryReset() ) {
            error(F("Couldn't factory reset"));
        }
    }

    /* Disable command echo from Bluefruit */
    ble.echo(false);

    Serial.println("Requesting Bluefruit info:");
    /* Print Bluefruit information */
    ble.info();

    Serial.println(F("Please use Adafruit Bluefruit LE app to connect in UART mode"));
    Serial.println(F("Then Enter characters to send to Bluefruit"));
    Serial.println();

    ble.verbose(false);  // debug info is a little annoying after this point!

    /* Wait for connection */
    ble.setMode(BLUEFRUIT_MODE_COMMAND);
    while (! ble.isConnected()) {
        delay(500);
        Serial.println("mooo");
    }

    Serial.println(F("******************************"));

    // LED Activity command is only supported from 0.6.6
    if ( ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) )
    {
        // Change Mode LED Activity
        Serial.println(F("Change LED activity to " MODE_LED_BEHAVIOUR));
        ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
    }

    // Set module to DATA mode
    Serial.println( F("Switching to DATA mode!") );
    ble.setMode(BLUEFRUIT_MODE_DATA);

    Serial.println(F("******************************"));
    Serial.println();

    pinMode(22, OUTPUT);
    digitalWrite(22, LOW);

    shouldRead = false;
    digitalWrite(22, HIGH);
    attachInterrupt(digitalPinToInterrupt(PIN_TO_INTERRUPT), soundCameBack, HIGH);

      //activate magnetometer/accelerometer
      if(!mag.begin()){
        /* There was a problem detecting the LSM303 ... check your connections */
        Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
      }

}
//end setup


volatile int lastSonarReading = 0;
volatile int sonarSensorValue = 0;

unsigned long prev[4] = { 0, 0, 0, 0};
unsigned long current[4] = { 0, 0, 0, 0 };

boolean interval[4] = {false, false, false, false};//enable or disable interval reporting
unsigned long sampleIntervals[4] = {DEFAULT_INTERVAL, DEFAULT_INTERVAL, DEFAULT_INTERVAL, DEFAULT_INTERVAL};//sample rates in Hz
boolean requested[4] = {false, false, false, false};//activates an immediate request for data
float thresh[4] = {40.0, 80.0, 30, 30};
//unsigned long tiempo [4] = {DEFAULT_INTERVAL, DEFAULT_INTERVAL, DEFAULT_INTERVAL, DEFAULT_INTERVAL};

void soundCameBack() {
    //no interrupts
    detachInterrupt(digitalPinToInterrupt(PIN_TO_INTERRUPT));
    lastSonarReading = sonarSensorValue;
    sonarSensorValue = analogRead(A0);
    //	Serial.println(sonarSensorValue);
    //pin 22 is used for reporting sensor data
    digitalWrite(22, LOW);

    shouldRead = true;
    //interrupts
}

//send char 'I' for interval and char 'R' for request, E for enable sonic, D for disable sonic
void loop() {

    delay(100); //needed for sonic sensor, sensor is only able to create accurate data a 10Hz

    char c [10];
    int index = 0;
    bool newBTMessage = false;
    // Echo received data
    //  Serial.println("Looping");
    while ( ble.available() )
    {
        c[index] = (char) ble.read();
        index++;
        newBTMessage = true;
    }
    if (newBTMessage) {
        parseCommandAndDevice(c);
    }

    handleSonarSensor();
    handleTempSensor();
    handleAccelerometerSensor();
    handleCompassSensor();


}

void handleSonarSensor() {
    if (shouldRead)
    {
        float cm = 1.69 * sonarSensorValue;

        if (reportAtInterval(sampleIntervals[0], 0) == true)
        {
            char data[20];
            combineCommandAndFloat("IA", cm, data);
            //            Serial.println("Sending interval sonar");
            //            Serial.println(data);
            ble.print(data);
        }
        if (cm < thresh[0])
        {
            char alarmData[20];
            combineCommandAndFloat("AA", cm, alarmData);
            //            Serial.println("Sending alarm sonar");
                        ble.print(alarmData);
        }
        if (requested[0] == true)
        {
            requested[0] = false;
            char requestedData[20];
            combineCommandAndFloat("RA", cm, requestedData);
            //            Serial.println("Sending requested sonar");
            ble.print(requestedData);
        }
        shouldRead = false;
        digitalWrite(22, HIGH);
        attachInterrupt(digitalPinToInterrupt(PIN_TO_INTERRUPT), soundCameBack, HIGH);
    }
}

void handleTempSensor() {
    float temp = readTemp();
    if (reportAtInterval(sampleIntervals[1], 1)) {
        char data[20];
        combineCommandAndFloat("IB", temp, data);
        ble.println(data);
    }
    if (temp > thresh[1]) {
        char alarmData[20];
        combineCommandAndFloat("AB", temp, alarmData);
                ble.println(alarmData);
    }
    if (requested[1] == true) {
        requested[1] = false;
        char requestedData[20];
        combineCommandAndFloat("RB", temp, requestedData);
        ble.print(requestedData);
    }
}

void handleAccelerometerSensor() {

//	Serial.println("Entered accelerometer");
    sensors_event_t event;
    mag.getEvent(&event);

    float abar[3];

    abar[0] = event.acceleration.x;
    abar[1] = event.acceleration.y;
    abar[2] = event.acceleration.z;
//    Serial.println("Got accelerometer");

    if (reportAtInterval(sampleIntervals[2], 2) == true) {
        char data[20];
        sendAccel(abar[0], abar[1], abar[2], data);
        String temp = "IC";
        temp += data;
        temp.toCharArray(data, 20);
        //            Serial.println("Sending interval sonar");
        //            Serial.println(data);
        ble.print(data);
    }
    if (abs(abar[0]) > thresh[2]) {
        char data[20];
        sendAccel(abar[0], abar[1], abar[2], data);
        String temp = "AC";
        temp += data;
        temp.toCharArray(data, 20);
        //            Serial.println("Sending interval sonar");
//                    Serial.println(data);
        ble.print(data);
    }
    if (requested[2] == true)
    {
    	requested[2] = false;
        char data[20];
        sendAccel(abar[0], abar[1], abar[2], data);
        String temp = "RC";
        temp += data;
        temp.toCharArray(data, 20);
        //            Serial.println("Sending interval sonar");
        //            Serial.println(data);
        ble.print(data);
    }
//    Serial.println("Exited accelerometer");
}

void handleCompassSensor() {
    sensors_event_t event;
    mag.getEvent(&event);

    float abar[3];

    abar[0] = event.magnetic.x;
    abar[1] = event.magnetic.y;
    abar[2] = event.magnetic.z;

    if (reportAtInterval(sampleIntervals[3], 3) == true) {
        char data[20];
        sendAccel(abar[0], abar[1], abar[2], data);
        String temp = "ID";
        temp += data;
        temp.toCharArray(data, 20);
        //            Serial.println("Sending interval sonar");
        //            Serial.println(data);
        ble.print(data);
    }
    if (abs(abar[0]) > thresh[3]) {
        char data[20];
        sendAccel(abar[0], abar[1], abar[2], data);
        String temp = "AD";
        temp += data;
        temp.toCharArray(data, 20);
        //            Serial.println("Sending interval sonar");
        //            Serial.println(data);
        ble.print(data);
    }
    if (requested[3] == true)
    {
    	requested[3] = false;
        char data[20];
        sendAccel(abar[0], abar[1], abar[2], data);
        String temp = "RD";
        temp += data;
        temp.toCharArray(data, 20);
        //            Serial.println("Sending interval sonar");
        //            Serial.println(data);
        ble.print(data);
    }

}

void combineCommandAndFloat(char command[], float reading, char *buffer) {


    char floatAsString[9];
    dtostrf(reading, 3, 2, floatAsString);
    sprintf(buffer, "%s%s", command, floatAsString);
    //    return buffer;

}

boolean reportAtInterval(unsigned long interv, int device) {
    current[device] = millis();

    if ((current[device] - prev[device]) >= interv && interval[device]) {
        prev[device] = current[device];
        //        Serial.print(interval[device]);
        return true;
    }
    return false;
}

float getFloat(char cString[]) {
    //while(inChar != '\n') {
    String newString = cString;
    String value = newString.substring(2, newString.length());
    Serial.print("String: ");
    Serial.println(value);
    float theFloat = value.toFloat();
    Serial.print("As float: ");
    Serial.println(theFloat);
    return theFloat;
}

unsigned long getLong(char* start) {
    String temp = start;
    unsigned long mouse = temp.toInt();
    return mouse;
}

float readTemp() {
    int reading = analogRead(TEMPERATURE_PIN);

    // converting that reading to voltage, for 3.3v arduino use 3.3
    float voltage = reading * 3.3;
    voltage /= 1024.0;

    // // print out the voltage
    // Serial.print(voltage); Serial.println(" volts");

    // // now print out the temperature
    float temperatureC = (voltage - 0.33) * 100 ;  //converting from 10 mv per degree wit 500 mV offset
    //                                               //to degrees ((voltage - 500mV) times 100)
    // Serial.print(temperatureC); Serial.println(" degrees C");

    // now convert to Fahrenheit
    float temperatureF = (temperatureC * 9.0 / 5.0) + 32.0;
    //     Serial.print(temperatureF); Serial.println(" degrees F");

    return temperatureF;

}

void parseCommandAndDevice(char c[]) {
    Serial.println(c);
    switch (c[0]) {
        case 'R': //request
            switch (c[1]) {
                case 'A': requested[0] = true;//sonar

                    break;
                case 'B': requested[1] = true;//temp

                    break;
                case 'C': requested[2] = true;//magnet

                    break;
                case 'D': requested[3] = true;//accel

                    break;
            }
            break;
        case 'E'://enable
            switch (c[1]) {
                case 'A': interval[0] = true;//sonar
                    ble.println("EA");
                    break;
                case 'B':
                    interval[1] = true;//temp
                    ble.println("EB");
                    break;
                case 'C':
                    interval[2] = true;//magnet
                    ble.println("EC");
                    break;
                case 'D':
                    interval[3] = true;//accel
                    ble.println("ED");
                    break;
            }
            break;

        case 'D'://disable
            switch (c[1]) {
                case 'A': {
                        interval[0] = false;//sonar
                        ble.println("DA");
                        break;
                    }
                case 'B': {
                        interval[1] = false;//temp
                        ble.println("DB");
                        break;
                    }
                case 'C': {
                        interval[2] = false;//magnet
                        ble.println("DC");
                        break;
                    }
                case 'D': {
                        interval[3] = false;//accel
                        ble.println("DD");
                        break;
                    }
            }
            break;

        case 'V'://update alert threshold
            switch (c[1]) {
                case 'A': thresh[0] = getFloat(c);//sonar

                    break;
                case 'B': thresh[1] = getFloat(c);//temp

                    break;
                case 'C': thresh[2] = getFloat(c);//magnet

                    break;
                case 'D': thresh[3] = getFloat(c);//accel

                    break;
            }
            break;

        case 'U'://update sampling rate
            switch (c[1]) {
                case 'A': {
                        sampleIntervals[0] = (1000 / getFloat(c));
                        Serial.println(sampleIntervals[0]);
                        break;
                    }
                case 'B': {
                        sampleIntervals[1] = (1000 / getFloat(c));
                        Serial.println(sampleIntervals[1]);
                        break;
                    }
                case 'C': {
                        sampleIntervals[2] = (1000 / getFloat(c));
                        Serial.println(sampleIntervals[2]);
                        break;
                    }
                case 'D': {
                        sampleIntervals[3] = (1000 / getFloat(c));
                        Serial.println(sampleIntervals[3]);
                        break;
                    }
            }
            break;
    }
}

void sendAccel(float X, float Y, float Z, char buffer[]) {
    String temp = "";
    temp += 'X';
    temp += String((int)X);
    //Serial.println(temp);
    temp += 'Y';
    temp += String((int)Y);
    temp += 'Z';
    temp += String((int)Z);
    
    temp.toCharArray(buffer, 20);
}

