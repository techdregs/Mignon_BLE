#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <HX711_ADC.h>
#include <EEPROM.h>
#include <ArduinoBLE.h>

// UUIDs for scale service and mass characteristic
BLEService scaleService("1eb9bb89-186e-4d2a-a204-346da73c061c");
BLEFloatCharacteristic massChar("5977b71a-a58c-40d2-85a4-34071043d9ca",
                                BLERead | BLENotify);  //remote clients can get notifications if this characteristic changes
BLEIntCharacteristic buttonChar("a8f2d9f3-c93a-4479-8208-7287262eacf6",
                                BLERead | BLEWrite | BLENotify);
BLEFloatCharacteristic set1Char("3f26c02b-871c-46a1-98c3-75f3fa7f5fd8",
                                BLERead | BLEWrite | BLENotify);
BLEFloatCharacteristic set2Char("e235088d-b515-4a7a-86a3-1b5ed4d886bb",
                                BLERead | BLEWrite | BLENotify);

//Screen resolution
#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 64  // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET -1  // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);


//pins for HX711:
const int HX711_dout = 10;  //mcu > HX711 dout pin
const int HX711_sck = 8;    //mcu > HX711 sck pin

//external control setup
int motorPin = D1;   //motor control pin
float target = 0.0;  //initial target weight for grinder
float fudge = 1;     //fudge factor for hysteresis: increase if too heavy
bool state = 0;      //state variable, 0 = off, 1 = on

//Trickle Variables
const unsigned long bump = 100;     //trickle time
const unsigned long settle = 2000;  //settling time
unsigned long tt = 0;               //timer for settling
bool isSettle = true;

//HX711 constructor:
HX711_ADC LoadCell(HX711_dout, HX711_sck);

// Var for calibration and t for timer
const int calVal_eepromAdress = 0;
unsigned long t = 0;

//Create variable to send to App for mass
float bleMass = 0.00;  // starting BLE mass value


void setup() {
  Serial.begin(57600);
  pinMode(motorPin, OUTPUT);
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ;  // Don't proceed, loop forever
  }
  delay(2000);  // Pause for 2 seconds

  Serial.println();
  Serial.println("Starting...");

  // Clear the buffer
  display.clearDisplay();

  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 10);
  // Display static text
  display.println("Starting Scale");
  display.display();

  LoadCell.begin();
  //LoadCell.setReverseOutput(); //uncomment to turn a negative output value to positive
  float calibrationValue;       // calibration value (see example file "Calibration.ino")
  calibrationValue = -2137.60;  // uncomment this if you want to set the calibration value in the sketch
#if defined(ESP8266) || defined(ESP32)
  //EEPROM.begin(512); // uncomment this if you use ESP8266/ESP32 and want to fetch the calibration value from eeprom
#endif
  //EEPROM.get(calVal_eepromAdress, calibrationValue); // uncomment this if you want to fetch the calibration value from eeprom

  unsigned long stabilizingtime = 2000;  // preciscion right after power-up can be improved by adding a few seconds of stabilizing time
  boolean _tare = true;                  //set this to false if you don't want tare to be performed in the next step
  LoadCell.start(stabilizingtime, _tare);
  if (LoadCell.getTareTimeoutFlag()) {
    Serial.println("Timeout, check MCU>HX711 wiring and pin designations");
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0, 10);
    display.println("Timeout, check MCU>HX711 wiring and pin designations");
    display.display();
    while (1)
      ;
  } else {
    LoadCell.setCalFactor(calibrationValue);  // set calibration value (float)
    Serial.println("Startup is complete");
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0, 10);
    display.println("Load Cell Ok");
    display.display();
  }

  // begin BLE initialization
  if (!BLE.begin()) {
    Serial.println("BLE failed to start!");
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0, 10);
    display.println("BLE failed to start!");
    display.display();
    while (1)
      ;
  }

  // Set BLE device name
  BLE.setLocalName("Xiao_Scale");
  BLE.setAdvertisedService(scaleService);      // add the service UUID
  scaleService.addCharacteristic(massChar);    // add the mass characteristic
  scaleService.addCharacteristic(buttonChar);  // add the buttons characteristic
  scaleService.addCharacteristic(set1Char);    // add the first settings characteristic
  scaleService.addCharacteristic(set2Char);    // add the second settings characteristic
  BLE.addService(scaleService);                // Add the service
  massChar.writeValue(bleMass);                // set initial value for  mass

  set1Char.setEventHandler(BLEWritten, set1CharWritten);
  set2Char.setEventHandler(BLEWritten, set2CharWritten);

  /* Start advertising Bluetooth® Low Energy.  It will start continuously transmitting Bluetooth® Low Energy
     advertising packets and will be visible to remote Bluetooth® Low Energy central devices
     until it receives a new connection */

  // start advertising
  BLE.advertise();

  Serial.println("Bluetooth® device active, waiting for connections...");
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 10);
  display.println("Bluetooth® device active, waiting for connections...");
  display.display();
}

void loop() {
  // wait for a Bluetooth® Low Energy central
  BLEDevice central = BLE.central();

  static boolean newDataReady = 0;
  const int serialPrintInterval = 15;  // increase value to slow down serial print/BLE activity

  // check for new data/start next conversion:
  if (LoadCell.update()) newDataReady = true;

  // get smoothed value from the dataset:
  float newMass = bleMass;
  if (newDataReady) {
    if (millis() > t + serialPrintInterval) {
      float i = LoadCell.getData();
      newMass = i;
      if (state == 1) {                 // if a setting has been called (setting call updates target to desired mass)
        if (newMass >= target + .04) {        // check if the weight is equal to or larger than goal (with some rounding error)
          digitalWrite(motorPin, LOW);  // if we are already at or over target weight, then turn off the motor
          state = 0;
          Serial.print("Motor Off. ");
          Serial.print("Target = ");
          Serial.print(target);
          Serial.print("  Current = ");
          Serial.println(newMass);

        } else {                             // Otherwise
          if (target > (newMass + fudge)) {  // check to see if we are farther away from the target than our fudge factor
            digitalWrite(motorPin, HIGH);    // if we are far away from target, run the grinder
            Serial.print("Motor Running. ");
            Serial.print("Target = ");
            Serial.print(target);
            Serial.print("  Current = ");
            Serial.println(newMass);
          } else {                            // If we are close to our target (less than fudge factor)
            if (isSettle) {                   // Do we need to let the reading settle? (starts as "true")
              digitalWrite(motorPin, LOW);    // if yes, turn the motor off for our settling time
              if (millis() - tt >= settle) {  // if we've waited long enough
                tt = millis();                // reset the time
                isSettle = false;             // we're done settling
              }
            } else {                         // if we are done waiting to settle
              digitalWrite(motorPin, HIGH);  // turn the motor on
              if (millis() - tt >= bump) {   // but just for our bump time
                tt = millis();               // reset the timer
                isSettle = true;             // then go back to settling so we can check the weight
              }
            }
          }
        }
      }
      // Serial.print("Load_cell output val: ");
      //  Serial.println(i);
      newDataReady = 0;
      t = millis();
      display.clearDisplay();
      display.setTextSize(1);
      display.setTextColor(WHITE);
      display.setCursor(0, 10);
      display.println("Mass (g): ");
      display.setTextSize(4);
      display.println(i, 1);

      if (!central.connected()) {
        display.setTextSize(1);
        display.println("No Connection.");
      };

      if (central.connected()) {
        display.setTextSize(1);
        display.println("BLE Connected.");
        if (newMass != bleMass) {
          bleMass = newMass;
          massChar.writeValue(bleMass);
        }
      }
      display.display();
    }
  }

  // receive command from BT app, send '2' to initiate tare operation.
  if (buttonChar.written()) {
    int32_t k = 0;
    buttonChar.readValue(k);
    Serial.print("Received Button val: ");
    Serial.println(k);
    if (k == 1) {
      state = 0;
      digitalWrite(motorPin, LOW);
      Serial.print("Stopped.");
    }
    if (k == 2) {
      LoadCell.tareNoDelay();
      if (LoadCell.getTareStatus() == true) {
        Serial.println("Tare complete");
      }
    }
  }
}

void set1CharWritten(BLEDevice central, BLECharacteristic characteristic) {
  Serial.print("Setting 1 called.");
  state = 1;
  target = set1Char.value();
}

void set2CharWritten(BLEDevice central, BLECharacteristic characteristic) {
  Serial.print("Setting 2 called.");
  state = 1;
  target = set2Char.value();
}