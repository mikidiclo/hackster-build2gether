#include <Adafruit_AHTX0.h>
#include <Notecard.h>
#include <NotecardPseudoSensor.h>
#include <STM32TimerInterrupt.h>

using namespace blues;

Adafruit_AHTX0 aht;

#define PIN_AIN1    PA3  // LOW, HIGH: Direction
#define PIN_AIN2    PA2  // LOW, HIGH: Direction
#define PIN_PWMA    PA1  // 0-255: Duty cycle 
#define PIN_STBY    PB12 // LOW: off, HIGH: on
#define PIN_LED     PC13 /
#define HW_TIMER_INTERVAL_MS 2000

#define usbSerial Serial1
#define productUID "com.gmail.mrthonglion:build2gether"

STM32Timer ITimer(TIM1); 

Notecard notecard;
NotecardPseudoSensor sensor(notecard);

sensors_event_t humidity, temp;
int fanSpeed;

void setup() {
  /* LED */
  pinMode(PC13, OUTPUT);
  /* Serial */
  usbSerial.begin(115200);
  /* Notecard */
  notecard.begin();
  notecard.setDebugOutputStream(usbSerial);
  /* Notecard: Connect to Notehub */
  J *req = notecard.newRequest("hub.set");
  JAddStringToObject(req, "product", productUID);
  JAddStringToObject(req, "mode", "continuous");
  notecard.sendRequest(req);
  /* Motor control */
  analogWriteFrequency(1000);
  pinMode(PIN_PWMA, OUTPUT);
  pinMode(PIN_AIN2, OUTPUT);
  pinMode(PIN_AIN1, OUTPUT);
  pinMode(PIN_STBY, OUTPUT);

  digitalWrite(PIN_AIN2, HIGH);
  digitalWrite(PIN_AIN1, LOW);
  digitalWrite(PIN_STBY, LOW); // Stanby port
  /* Timer */
  ITimer.attachInterruptInterval(HW_TIMER_INTERVAL_MS * 15000, timerInterrupt);
  /* AHT */
  if (! aht.begin()) {
    usbSerial.println("[ERR] ATH10: Error");
  } else {
    usbSerial.println("[INFO] AHT10: Ready");
  }
}

void timerInterrupt() {
  /* Get sensor data */
  aht.getEvent(&humidity, &temp);
  // float temperature = temp.temperature;
  // int humidity = humidity.relative_humidity;
  /* Notecard: Send to Notehub */
  J *req = notecard.newRequest("note.add");
  if (req != NULL)
  {
    JAddStringToObject(req, "file", "sensors.qo");
    JAddBoolToObject(req, "sync", true);
    J *body = JAddObjectToObject(req, "body");
    if (body)
    {
      JAddNumberToObject(body, "temp", temp.temperature);
      JAddNumberToObject(body, "humidity", humidity.relative_humidity);
      JAddNumberToObject(body, "fanSpeed", fanSpeed);      
    }
    notecard.sendRequest(req);
  }
}

int adjustFanSpeed(float temp, float humid) {
  /* Adjust fan speed based on the temp & humid from AHT10 */
  int speed = 0;

  if (temp > 40) {
    speed = 150;
  } else if ((temp <= 40) && (temp >= 30)) {
    speed = 100;
  } else {
    speed = 50;
  }
  if (humid > 80) {
    speed = speed - 10;
  } else if ((humid <= 80) && (humid >= 50)) {
    speed = speed;
  } else {
    speed = speed + 10;
  }

  return speed;
}

void adjustFanModes() {
  /*Adjust fan modes based on the UART receive from nRF52840 */
  while (Serial.available() >= 0) {
    int receivedData = Serial.read();
    if (receivedData == 1) {
      digitalWrite(PIN_STBY, HIGH);
    } else if (receivedData == 0) {
      // Turn off the motor
      digitalWrite(PIN_STBY, LOW);
    }
  }
}

void loop() {
  /* Get sensor data */
  aht.getEvent(&humidity, &temp);
  /* Adjust fans */
  fanSpeed = adjustFanSpeed(temp.temperature, (float) humidity.relative_humidity);
  analogWrite(PIN_PWMA, fanSpeed);
  adjustFanModes();

  delay(1000);

}
