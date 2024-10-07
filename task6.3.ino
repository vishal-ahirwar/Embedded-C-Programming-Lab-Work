#include "Seeed_MCP9808.h"

MCP9808 sensor;

#define LED_0 PB0
#define LED_1 PB7
#define LED_2 PB14
#define BUTTON_PIN PC13
#define TRIG_PIN D12
#define ECHO_PIN D13

#define MCP9808_I2C_ADDRESS 0x18       // Default I2C address for MCP9808
#define AMBIENT_TEMP_REG 0x05          // Ambient temperature register
#define RESOLUTION_REG 0x08            // Resolution register
#define RESOLUTION_0_0625_DEGREE 0x03  // Highest resolution

// Variables will change:
int buttonState;            // the current reading from the input pin
int lastButtonState = LOW;  // the previous reading from the input pin
int counter = 0;

// the following variables are unsigned longs because the time, measured in
// milliseconds, will quickly become a bigger number than can be stored in an int.
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 500;   // the debounce time; increase if the output flickers

volatile bool calibrationComplete = false;
float minDistance = 1000;  // Large initial value for calibration
float maxDistance = 0;     // Small initial value for calibration

void setup6_3() {
  // put your setup code here, to run once:
  pinMode(LED_0, OUTPUT);
  pinMode(LED_1, OUTPUT);
  pinMode(LED_2, OUTPUT);
}

void setup6_4() {
  setup6_3();
  pinMode(BUTTON_PIN, INPUT);
}

void setup6_5() {
  pinMode(LED_0, OUTPUT);
  pinMode(LED_1, OUTPUT);
}
void setup7_1() {
  Serial.begin(9600);         // Initialize serial communication
  pinMode(TRIG_PIN, OUTPUT);  // Set the trigger pin as an output
  pinMode(ECHO_PIN, INPUT);   // Set the echo pin as an input
}
void setup7_2() {
  Serial.begin(9600);         // Initialize serial communication
  pinMode(TRIG_PIN, OUTPUT);  // Set the trigger pin as an output
  pinMode(ECHO_PIN, INPUT);   // Set the echo pin as an input
}
void setup7_3() {
  Serial.begin(9600);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(LED_1, OUTPUT);
  pinMode(BUTTON_PIN, INPUT);

  // Attach interrupt for the button press
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), completeCalibration, FALLING);
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), resetCalibration, RISING);
  // Set up PWM for LED pin
  analogWriteResolution(8);  // 8-bit resolution (0-255)
}
void setup8_1() {
  Serial.begin(9600);
  if (sensor.init()) {
    Serial.println("Sensor init failed.");
    return;
  }
  Serial.println("Sensor init success.");
}
void setup8_2() {
  Wire.begin();
  Serial.begin(9600);

  // Initialize the sensor
  if (initSensor()) {
    Serial.println("Sensor init failed.");
  } else {
    Serial.println("Sensor init success.");
  }
}
void setup() {
  // setup6_3();
  // setup6_4();
  // setup6_5();
  // setup7_1();
  // setup7_2();
  // setup7_3();
  setup8_1();
};


void loop() {
  // task6_3();
  // task6_4();
  // task6_5();
  // task7_1();
  // task7_2();
  // task7_3();
  task8_1();
  // task8_2();
};



void task_6_3() {
  digitalWrite(LED_0, HIGH);
  delay(1000);
  digitalWrite(LED_1, HIGH);
  digitalWrite(LED_0, LOW);
  delay(1000);
  digitalWrite(LED_2, HIGH);
  digitalWrite(LED_0, LOW);
  digitalWrite(LED_1, LOW);
  delay(1000);
  digitalWrite(LED_2, LOW);
}
void task6_4() {
  // read the state of the switch into a local variable:
  int reading = digitalRead(BUTTON_PIN);

  // check to see if you just pressed the button
  // (i.e. the input went from LOW to HIGH), and you've waited long enough
  // since the last press to ignore any noise:

  // If the switch changed, due to noise or pressing:
  if (reading != lastButtonState) {
    // reset the debouncing timer
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    // whatever the reading is at, it's been there for longer than the debounce
    // delay, so take it as the actual current state:

    // if the button state has changed:
    if (reading != buttonState) {
      buttonState = reading;

      // only toggle the LED if the new button state is HIGH
      if (buttonState == HIGH) {
        if (counter == 2) counter = 0;
        else ++counter;
        switch (counter) {
          case 0:
            {
              digitalWrite(LED_0, HIGH);
              digitalWrite(LED_2, LOW);
              break;
            };
          case 1:
            {
              digitalWrite(LED_1, HIGH);
              digitalWrite(LED_0, LOW);
              break;
            };
          case 2:
            {
              digitalWrite(LED_2, HIGH);
              digitalWrite(LED_0, LOW);
              digitalWrite(LED_1, LOW);
              break;
            }
        }
      }
    }
  }

  // save the reading. Next time through the loop, it'll be the lastButtonState:
  lastButtonState = reading;
};

void task6_5() {
  // fade in from min to max in increments of 5 points:
  for (int fV1 = 0; fV1 <= 255; fV1 += 5) {
    // sets the value (range from 0 to 255):
    analogWrite(LED_0, fV1);
    analogWrite(LED_1, 255 - fV1);
    // wait for 30 milliseconds to see the dimming effect
    delay(30);
  }

  // fade out from max to min in increments of 5 points:
  for (int fV1 = 0; fV1 <= 255; fV1 += 5) {
    // sets the value (range from 0 to 255):
    analogWrite(LED_0, 255 - fV1);
    analogWrite(LED_1, fV1);
    // wait for 30 milliseconds to see the dimming effect
    delay(30);
  }
};

void task7_1() {
  float distance = measureDistance();
  // Print the average distance to the Serial Monitor
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  delay(1000);  // Delay before taking another set of measurements
};

void task7_2() {
  float distance = measureDistanceSmooth(10);
  // Print the average distance to the Serial Monitor
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  delay(1000);  // Delay before taking another set of measurements
};

void task7_3() {
  if (!calibrationComplete) {
    float distance = measureDistance();

    // Update min/max distances for calibration
    if (distance < minDistance) minDistance = distance;
    if (distance > maxDistance) maxDistance = distance;

    Serial.print("Calibrating... Min: ");
    Serial.print(minDistance);
    Serial.print(" cm, Max: ");
    Serial.println(maxDistance);

    delay(500);  // Slow down calibration readings
  } else {
    // After calibration, use distance to control LED brightness
    float currentDistance = measureDistanceSmooth(10);
    int brightness = map(currentDistance, minDistance, maxDistance, 0, 255);  // Map distance to brightness
    brightness = constrain(brightness, 0, 255);                               // Ensure the value stays within 0-255 range

    analogWrite(LED_1, brightness);  // Set LED brightness

    Serial.print("Distance(cm):");
    Serial.print(currentDistance);
    Serial.print(",LED Brightness:");
    Serial.println(brightness);

    delay(200);  // Slow down the readings
  }
}

void task8_1() {
  float celsius = 0;
  sensor.get_temp(&celsius);

  float fahrenheit = celsius * 9 / 5 + 32;

  Serial.print("Temperature: ");
  Serial.print(celsius);
  Serial.print("C/");
  Serial.print(fahrenheit);
  Serial.println("F");
  delay(1000);
}
void task8_2() {
  float celsius = readTemperature();
  float fahrenheit = celsius * 9.0 / 5.0 + 32;

  Serial.print("Temperature: ");
  Serial.print(celsius);
  Serial.print(" C / ");
  Serial.print(fahrenheit);
  Serial.println(" F");

  delay(1000);  // Wait 1 second before reading the temperature again
};

bool initSensor() {
  Wire.beginTransmission(MCP9808_I2C_ADDRESS);
  Wire.write(RESOLUTION_REG);            // Set the resolution register
  Wire.write(RESOLUTION_0_0625_DEGREE);  // Set highest resolution (0.0625°C)
  return (Wire.endTransmission() != 0);  // Check for I2C transmission success
}

float readTemperature() {
  Wire.beginTransmission(MCP9808_I2C_ADDRESS);
  Wire.write(AMBIENT_TEMP_REG);  // Set pointer to the temperature register
  Wire.endTransmission();

  Wire.requestFrom(MCP9808_I2C_ADDRESS, 2);  // Request 2 bytes from sensor

  if (Wire.available() < 2) {
    return NAN;  // If data not available, return not-a-number (NAN)
  }

  // Read the 2 bytes of temperature data
  uint8_t msb = Wire.read();
  uint8_t lsb = Wire.read();

  // Combine the two bytes into a 16-bit unsigned integer
  uint16_t rawTemp = ((uint16_t)msb << 8) | lsb;

  // Mask out the sign bit (bit 12) and calculate the temperature
  rawTemp &= 0x0FFF;
  float temp = rawTemp * 0.0625;  // Each increment represents 0.0625°C

  // If the sign bit is set, the temperature is negative
  if (msb & 0x10) {
    temp -= 256;
  }

  return temp;
}
float measureDistance() {
  long duration;
  float distance;
  // Clear the trigger pin
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);

  // Send a 10µs pulse to trigger the sensor
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Measure the time of the echo pulse
  duration = pulseIn(ECHO_PIN, HIGH);

  // Calculate the distance (speed of sound is 34300 cm/s)
  distance = (duration * 0.0343) / 2;
  return distance;
}

float measureDistanceSmooth(int numMeasurements) {
  float totalDistance = 0.0;

  // Take multiple measurements
  for (int i = 0; i < numMeasurements; i++) {
    // Add to the total distance for averaging
    totalDistance += measureDistance();
    delay(5);  // Small delay between measurements
  }

  // return the average distance
  return totalDistance / numMeasurements;
}

// Button interrupt handler to stop calibration
void completeCalibration() {
  calibrationComplete = true;
  Serial.println("Calibration complete.");
}

void resetCalibration() {
  calibrationComplete = false;
  Serial.println("Calibration reset.");
}