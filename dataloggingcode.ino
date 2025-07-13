#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_ADXL375.h>
#include <Adafruit_Sensor.h>

#define SEALEVELPRESSURE_HPA (1013.25)
#define BUTTON_PIN 32
#define SD_CS_PIN 5  // CS pin for microSD on Adalogger Feather RP2040

Adafruit_BMP3XX bmp;
Adafruit_ADXL375 accel = Adafruit_ADXL375(12345);

File dataFile;

bool zeroed = false;
bool launchDetected = false;

float pressureRef = 0;
float altitudeRef = 0;
float accelXRef = 0, accelYRef = 0, accelZRef = 0;
unsigned long startTime = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);
  Wire.begin();

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // Initialize BMP390
  if (!bmp.begin_I2C()) {
    Serial.println("BMP390 not detected!");
    while (1);
  }

  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);

  // Initialize ADXL375
  if (!accel.begin()) {
    Serial.println("ADXL375 not detected!");
    while (1);
  }
  accel.setRange(ADXL375_RANGE_200_G);

  // Initialize SD card
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("Card failed or not present.");
    while (1);
  }

  // Create/open file
  dataFile = SD.open("rocket.csv", FILE_WRITE);
  if (dataFile) {
    dataFile.println("Time(ms),Altitude(m),Pressure(Pa),Temp(C),AccX(G),AccY(G),AccZ(G),AccMag(G),Velocity(m/s),Event");
    dataFile.flush();
  }

  startTime = millis();
  Serial.println("Setup complete.");
}

float prevAltitude = 0;
unsigned long prevTime = 0;

void loop() {
  bmp.performReading();

  float pressure = bmp.pressure;
  float temperature = bmp.temperature;
  float altitude = bmp.readAltitude(SEALEVELPRESSURE_HPA);

  sensors_event_t a;
  accel.getEvent(&a);

  float ax = a.acceleration.x;
  float ay = a.acceleration.y;
  float az = a.acceleration.z;
  float accMag = sqrt(ax * ax + ay * ay + az * az);

  unsigned long currentTime = millis();
  float deltaTime = (currentTime - prevTime) / 1000.0;

  float altitudeZeroed = zeroed ? (altitude - altitudeRef) : 0;
  float velocity = deltaTime > 0 ? (altitude - prevAltitude) / deltaTime : 0;

  // Check for button press (active LOW)
  if (digitalRead(BUTTON_PIN) == LOW) {
    zeroSensors(pressure, altitude, ax, ay, az);
    logEvent("Button Pressed: Zeroing sensors");
    delay(500);  // Debounce
  }

  // Detect launch if altitude exceeds 20m after zeroing
  if (zeroed && !launchDetected && altitudeZeroed > 20.0) {
    launchDetected = true;
    logEvent("Launch Detected: Altitude exceeded 20m");
  }

  // Log data
  if (dataFile) {
    dataFile.print(currentTime - startTime);
    dataFile.print(",");
    dataFile.print(altitudeZeroed, 2);
    dataFile.print(",");
    dataFile.print(pressure, 2);
    dataFile.print(",");
    dataFile.print(temperature, 2);
    dataFile.print(",");
    dataFile.print(ax, 2);
    dataFile.print(",");
    dataFile.print(ay, 2);
    dataFile.print(",");
    dataFile.print(az, 2);
    dataFile.print(",");
    dataFile.print(accMag, 2);
    dataFile.print(",");
    dataFile.print(velocity, 2);
    dataFile.println(",");
    dataFile.flush();
  }

  prevAltitude = altitude;
  prevTime = currentTime;

  delay(100);  // Sample rate: 10 Hz
}

void zeroSensors(float p, float alt, float ax, float ay, float az) {
  pressureRef = p;
  altitudeRef = alt;
  accelXRef = ax;
  accelYRef = ay;
  accelZRef = az;
  zeroed = true;
  launchDetected = false;
}

void logEvent(String message) {
  if (dataFile) {
    unsigned long t = millis() - startTime;
    dataFile.print(t);
    dataFile.print(",,,,,,,,,");
    dataFile.println(message);
    dataFile.flush();
  }
  Serial.println(message);
}
