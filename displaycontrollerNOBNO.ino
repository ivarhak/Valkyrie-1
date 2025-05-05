/*
 * Rocket Flight Computer - ESP32 Display Controller
 * 
 * This code runs on the ESP32 Feather V2 and is responsible for:
 * - Reading data from ADXL375 accelerometer, and BMP390 barometer
 * - Displaying sensor data on a 2.2" TFT display
 * - Implementing sensor zeroing functionality via button press
 * 
 * Hardware:
 * - Adafruit ESP32 Feather V2
 * - Adafruit 2.2" TFT Display (320x240)
 * - ADXL375 High-G Accelerometer
 * - BMP390 Barometric Pressure Sensor
 * - External button for zeroing sensors
 * 
 * Connections:
 * - All sensors connected via I2C (SDA/SCL)
 * - TFT display connected via SPI
 * - Zero button connected to GPIO pin (placeholder)
 */

// Include necessary libraries
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL375.h>
#include <Adafruit_BMP3XX.h>


// Pin definitions for TFT display (updated with user-specified pins)
#define TFT_CS 13      // TCS pin
#define TFT_DC 12      // DC pin
#define TFT_RST 33     // RST pin
#define TFT_MOSI MOSI  // Hardware SPI MOSI pin
#define TFT_CLK SCK    // Hardware SPI SCK pin
#define TFT_MISO MISO  // Hardware SPI MISO pin
#define TFT_LITE 27    // Backlight pin
#define SD_CS 15       // SD card CS pin

// Pin definition for zero button
#define ZERO_BUTTON_PIN 32  // Reset button

// I2C addresses
#define ADXL375_I2C_ADDR 0x53  // Default ADXL375 address
#define BMP3XX_I2C_ADDR 0x77   // Default BMP390 address

// Constants
#define SEALEVELPRESSURE_HPA (1013.25)  // Standard sea level pressure in hPa
#define DISPLAY_UPDATE_RATE 10          // Display update rate in Hz
#define MAX_ALTITUDE_GRAPH 1000         // Maximum altitude for graph in meters
#define MAX_ACCEL_GRAPH 50              // Maximum acceleration for graph in m/s²

// Create TFT display instance - using hardware SPI
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_RST);

// Sensor objects

Adafruit_ADXL375 accel = Adafruit_ADXL375(12345);
Adafruit_BMP3XX bmp;

// Sensor data structures

sensors_event_t accelEvent;
float temperature, pressure, altitude;
float referenceAltitude = 0;  // Reference altitude when zeroed
float relativeAltitude = 0;   // Altitude relative to zeroing point
int maxAltitude = 0;        // Maximum relative altitude
float maxAcceleration = 0;    // Maximum acceleration
float maxBaroVelo = 0;
float baroVelo = 0;
// Reference values for accelerometer zeroing
float accel_ref_x = 0;
float accel_ref_y = 0;
float accel_ref_z = 0;
float adjusted_x = 0;
  float adjusted_y = 0;
  float adjusted_z = 0;
  float prevAlt =0;
  float prevTime = 0;
 float Time = 0;

// Timing variables
unsigned long previousMillis = 0;
const unsigned long displayInterval = 1000 / DISPLAY_UPDATE_RATE;  // in milliseconds

// Button state tracking
bool lastButtonState = HIGH;
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;

// Display layout constants
#define HEADER_HEIGHT 30
#define GRAPH_HEIGHT 40
#define GRAPH_WIDTH 100
#define GRAPH_X 200
#define ALTITUDE_GRAPH_Y 40
#define ACCEL_GRAPH_Y 150
#define DATA_X 10
#define DATA_Y_START 40
#define DATA_Y_SPACING 20

// Altitude history for graph
#define ALTITUDE_HISTORY_SIZE 100
float altitudeHistory[ALTITUDE_HISTORY_SIZE];
int altitudeHistoryIndex = 0;

// Acceleration history for graph
#define ACCEL_HISTORY_SIZE 100
float accelHistory[ACCEL_HISTORY_SIZE];
int accelHistoryIndex = 0;

// Function prototypes
bool initializeSensors();
void initializeDisplay();
void readSensors();
void updateDisplay();
void drawHeader();
void drawData();
void drawAltitudeGraph();
void drawAccelerationGraph();
void checkZeroButton();
void zeroSensors();


void setup() {
  // Initialize serial communication
  Serial.begin(9600);

  // Wait for serial monitor to open (comment out for standalone operation)
  // while (!Serial) delay(10);

  Serial.println("Rocket Flight Computer - ESP32 Display Controller");

  // Set up the zero button pin with internal pull-up resistor
  pinMode(ZERO_BUTTON_PIN, INPUT_PULLUP);

  // Set up backlight pin
  pinMode(TFT_LITE, OUTPUT);
  digitalWrite(TFT_LITE, HIGH);  // Turn on backlight
  delay(100);                    // Add delay after setting backlight

  // Initialize I2C
  Wire.begin();

  // Initialize SPI explicitly
  SPI.begin();

  // Reset the display before initialization
  pinMode(TFT_RST, OUTPUT);
  digitalWrite(TFT_RST, HIGH);
  delay(50);
  digitalWrite(TFT_RST, LOW);
  delay(50);
  digitalWrite(TFT_RST, HIGH);
  delay(50);

  // Initialize display
  initializeDisplay();

  
  // Initialize sensors
  if (!initializeSensors()) {
    Serial.println("Failed to initialize one or more sensors!");
    tft.fillScreen(ILI9341_BLACK);
    tft.setCursor(10, 10);
    tft.setTextColor(ILI9341_RED);
    tft.setTextSize(2);
    tft.println("SENSOR ERROR");
    tft.println("Check connections");
    delay(2000);
    while (1) delay(10);  // Halt if sensors fail to initialize
  }

  // Initialize history arrays
  for (int i = 0; i < ALTITUDE_HISTORY_SIZE; i++) {
    altitudeHistory[i] = 0;
  }

  for (int i = 0; i < ACCEL_HISTORY_SIZE; i++) {
    accelHistory[i] = 0;
  }

  Serial.println("Initialization complete. Display ready.");

  // Draw initial display
  tft.fillScreen(ILI9341_BLACK);
  drawHeader();

  // Load and display bitmap if available

}

void loop() {
  unsigned long currentMillis = millis();

  // Check if it's time to update the display
  if (currentMillis - previousMillis >= displayInterval) {
    previousMillis = currentMillis;

    // Read data from all sensors
    readSensors();

    // Update the display
    updateDisplay();
  }

  // Check if zero button is pressed
  checkZeroButton();

  // Add a small delay to prevent display flickering
  delay(20);
}

bool initializeSensors() {
  bool success = true;

  
  // Initialize ADXL375
  Serial.println("Initializing ADXL375...");
  if (!accel.begin()) {
    Serial.println("Failed to initialize ADXL375!");
    success = false;
  } else {
    Serial.println("ADXL375 initialized successfully");
  }

  // Initialize BMP390
  Serial.println("Initializing BMP390...");
  if (!bmp.begin_I2C(BMP3XX_I2C_ADDR)) {
    Serial.println("Failed to initialize BMP390!");
    success = false;
  } else {
    // Set up oversampling and filter initialization
    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp.setOutputDataRate(BMP3_ODR_50_HZ);
    Serial.println("BMP390 initialized successfully");
    delay(2000);
  }

  return success;
}

void initializeDisplay() {
  Serial.println("Initializing display...");

  // Set SPI clock to a much lower speed to improve stability
  SPI.begin();
  SPI.setFrequency(40000000);  //40MHz
  SPI.setDataMode(SPI_MODE0);

  // More thorough initialization
  tft.begin(4000000);  // Explicitly set 4MHz
  delay(200);

  tft.setRotation(1);  // Landscape mode
  delay(100);

  // Display initialization message
  tft.fillScreen(ILI9341_BLACK);
  delay(50);

  tft.setCursor(10, 10);
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(2);
  tft.println("Rocket Flight Computer");
  tft.println("Initializing...");

  // Add another delay to ensure display is stable
  delay(20);
 
  Serial.println("Display initialized");
}

void readSensors() {
 

  // Read ADXL375 data and apply reference offsets
  accel.getEvent(&accelEvent);

  // Apply reference offsets to get zeroed acceleration values
   adjusted_x = accelEvent.acceleration.x - accel_ref_x;
   adjusted_y = accelEvent.acceleration.y - accel_ref_y;
   adjusted_z = accelEvent.acceleration.z - accel_ref_z;

  // Update the acceleration event with adjusted values
  accelEvent.acceleration.x = adjusted_x;
  accelEvent.acceleration.y = adjusted_y;
  accelEvent.acceleration.z = adjusted_z;

  // Read BMP390 data
  if (bmp.performReading()) {
    temperature = bmp.temperature;
    pressure = bmp.pressure;
    prevAlt = altitude;
    altitude = bmp.readAltitude(SEALEVELPRESSURE_HPA);

    // Calculate relative altitude from reference point
    relativeAltitude = altitude - referenceAltitude;

    // Update max altitude (using relative altitude)
    if (relativeAltitude > maxAltitude) {
      maxAltitude = relativeAltitude;
    }
    if (baroVelo > maxBaroVelo) {
      maxBaroVelo = baroVelo;
    }
        if (sqrt(
    accelEvent.acceleration.x * accelEvent.acceleration.x + accelEvent.acceleration.y * accelEvent.acceleration.y + accelEvent.acceleration.z * accelEvent.acceleration.z)-2.69
> maxAcceleration) {
      maxAcceleration = sqrt(
    accelEvent.acceleration.x * accelEvent.acceleration.x + accelEvent.acceleration.y * accelEvent.acceleration.y + accelEvent.acceleration.z * accelEvent.acceleration.z)- 2.69;

    }

    // Update altitude history with relative altitude
    altitudeHistory[altitudeHistoryIndex] = relativeAltitude;
    altitudeHistoryIndex = (altitudeHistoryIndex + 1) % ALTITUDE_HISTORY_SIZE;
  } else {
    Serial.println("Failed to read BMP390");
  }
}

void updateDisplay() {
  // Update all display elements
  drawHeader();
  drawData();
  drawAltitudeGraph();
  drawAccelerationGraph();

}

void drawHeader() {
  // Draw header with title
  tft.fillRect(0, 0, tft.width(), HEADER_HEIGHT, ILI9341_NAVY);
  tft.setCursor(10, 10);
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(2);
  tft.print("VALKYRIE ONE");
}

void drawData() {
  // Clear data area
  tft.fillRect(0, HEADER_HEIGHT, DATA_X + 150, tft.height() - HEADER_HEIGHT, ILI9341_BLACK);

  // Set text properties
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(1);

  // Display altitude data (now using relative altitude)
  tft.setCursor(DATA_X, DATA_Y_START);
  tft.print("Altitude: ");
  tft.setTextColor(ILI9341_GREEN);
  tft.print(relativeAltitude, 1);
  tft.println("");

  // Display max altitude with highlight
  tft.setCursor(DATA_X, DATA_Y_START + DATA_Y_SPACING);
  tft.setTextColor(ILI9341_WHITE);
  tft.print("Max Alt: ");
  tft.setTextColor(ILI9341_GREEN);
  tft.setTextSize(2);  // Larger text for max values
  tft.print(maxAltitude, 1);
  tft.println("");
  tft.setTextSize(1);  // Reset text size

  // Display acceleration data
  tft.setCursor(DATA_X, DATA_Y_START + 2 * DATA_Y_SPACING + 10);  // Extra spacing for the larger text above
  tft.setTextColor(ILI9341_WHITE);
  tft.print("Accel: ");
  tft.setTextColor(ILI9341_YELLOW);

  // Calculate total acceleration from ADXL375
  float totalAccel = sqrt(
    accelEvent.acceleration.x * accelEvent.acceleration.x + accelEvent.acceleration.y * accelEvent.acceleration.y + accelEvent.acceleration.z * accelEvent.acceleration.z)-2.69;


  tft.print(totalAccel, 1);
  tft.println(" m/s2");

  // Display max acceleration with highlight
  tft.setCursor(DATA_X, DATA_Y_START + 3 * DATA_Y_SPACING + 10);
  tft.setTextColor(ILI9341_WHITE);
  tft.print("Max Accel: ");
  tft.setTextColor(ILI9341_YELLOW);
  tft.setTextSize(2);  // Larger text for max values
  tft.print(maxAcceleration, 1);
  tft.println("");
  tft.setTextSize(1);  // Reset text size
//Display baroVelo and maxBaroVelo
  tft.setCursor(DATA_X, DATA_Y_START + 4 * DATA_Y_SPACING + 10);  // Extra spacing for the larger text above
  tft.setTextColor(ILI9341_WHITE);
  tft.print("Velocity: ");
  tft.setTextColor(ILI9341_CYAN);
// find baroVelo
    prevTime = Time;
  Time = millis() / 1000.000000f;
  baroVelo = (altitude-prevAlt)/(Time-prevTime);


  tft.print(baroVelo, 1);
  tft.println(" m/s");


   tft.setCursor(DATA_X, DATA_Y_START + 5 * DATA_Y_SPACING + 10);
  tft.setTextColor(ILI9341_WHITE);
  tft.print("Max Velo: ");
  tft.setTextColor(ILI9341_CYAN);
  tft.setTextSize(2);  // Larger text for max values
  tft.print(maxBaroVelo, 1);
  tft.println("");
  tft.setTextSize(1);  // Reset text size
tft.setCursor(DATA_X, DATA_Y_START + 6 * DATA_Y_SPACING + 15);
  tft.setTextColor(ILI9341_WHITE);
  tft.print("max Mach: ");
  tft.setTextColor(ILI9341_BLUE);
  tft.setTextSize(2);  // Larger text for max values
  tft.print(maxBaroVelo/343.00, 1);
  tft.println("");
  tft.setTextSize(1);  // Reset text size

  // Display orientation data
 //CUT

  // Display barometric data
  tft.setCursor(DATA_X, DATA_Y_START + 7 * DATA_Y_SPACING + 20);
  tft.setTextColor(ILI9341_WHITE);
  tft.print("Temp: ");
  tft.setTextColor(ILI9341_MAGENTA);
  tft.print(temperature, 1);
  tft.println(" C");

  tft.setCursor(DATA_X, DATA_Y_START + 8 * DATA_Y_SPACING + 20);
  tft.setTextColor(ILI9341_WHITE);
  tft.print("Press: ");
  tft.setTextColor(ILI9341_MAGENTA);
  tft.print(pressure / 100.0, 1);  // Convert Pa to hPa
  tft.println(" hPa");
}

void drawAltitudeGraph() {
  // Draw altitude graph frame
  tft.drawRect(GRAPH_X, ALTITUDE_GRAPH_Y, GRAPH_WIDTH, GRAPH_HEIGHT, ILI9341_WHITE);

  // Draw graph title
  tft.setCursor(GRAPH_X, ALTITUDE_GRAPH_Y - 10);
  tft.setTextColor(ILI9341_GREEN);
  tft.setTextSize(1);
  tft.print("Altitude (m)");

  // Clear graph area
  tft.fillRect(GRAPH_X + 1, ALTITUDE_GRAPH_Y + 1, GRAPH_WIDTH - 2, GRAPH_HEIGHT - 2, ILI9341_BLACK);

  // Draw altitude graph
  for (int i = 0; i < ALTITUDE_HISTORY_SIZE - 1; i++) {
    int x1 = GRAPH_X + i * (GRAPH_WIDTH - 2) / ALTITUDE_HISTORY_SIZE + 1;
    int x2 = GRAPH_X + (i + 1) * (GRAPH_WIDTH - 2) / ALTITUDE_HISTORY_SIZE + 1;

    int idx1 = (altitudeHistoryIndex + i) % ALTITUDE_HISTORY_SIZE;
    int idx2 = (altitudeHistoryIndex + i + 1) % ALTITUDE_HISTORY_SIZE;

    int y1 = ALTITUDE_GRAPH_Y + GRAPH_HEIGHT - 1 - (altitudeHistory[idx1] * (GRAPH_HEIGHT - 2) / MAX_ALTITUDE_GRAPH);
    int y2 = ALTITUDE_GRAPH_Y + GRAPH_HEIGHT - 1 - (altitudeHistory[idx2] * (GRAPH_HEIGHT - 2) / MAX_ALTITUDE_GRAPH);

    // Constrain y values to graph area
    y1 = constrain(y1, ALTITUDE_GRAPH_Y + 1, ALTITUDE_GRAPH_Y + GRAPH_HEIGHT - 1);
    y2 = constrain(y2, ALTITUDE_GRAPH_Y + 1, ALTITUDE_GRAPH_Y + GRAPH_HEIGHT - 1);

    tft.drawLine(x1, y1, x2, y2, ILI9341_GREEN);
  }

  // Redraw bitmap if it was loaded (to ensure it stays visible)
 
}

void drawAccelerationGraph() {
  // Draw acceleration graph frame
  tft.drawRect(GRAPH_X, ACCEL_GRAPH_Y, GRAPH_WIDTH, GRAPH_HEIGHT, ILI9341_WHITE);

  // Draw graph title
  tft.setCursor(GRAPH_X, ACCEL_GRAPH_Y - 10);
  tft.setTextColor(ILI9341_YELLOW);
  tft.setTextSize(1);
  tft.print("Acceleration (m/s2)");

  // Clear graph area
  tft.fillRect(GRAPH_X + 1, ACCEL_GRAPH_Y + 1, GRAPH_WIDTH - 2, GRAPH_HEIGHT - 2, ILI9341_BLACK);

  // Draw acceleration graph
  for (int i = 0; i < ACCEL_HISTORY_SIZE - 1; i++) {
    int x1 = GRAPH_X + i * (GRAPH_WIDTH - 2) / ACCEL_HISTORY_SIZE + 1;
    int x2 = GRAPH_X + (i + 1) * (GRAPH_WIDTH - 2) / ACCEL_HISTORY_SIZE + 1;

    int idx1 = (accelHistoryIndex + i) % ACCEL_HISTORY_SIZE;
    int idx2 = (accelHistoryIndex + i + 1) % ACCEL_HISTORY_SIZE;

    int y1 = ACCEL_GRAPH_Y + GRAPH_HEIGHT - 1 - (accelHistory[idx1] * (GRAPH_HEIGHT - 2) / MAX_ACCEL_GRAPH);
    int y2 = ACCEL_GRAPH_Y + GRAPH_HEIGHT - 1 - (accelHistory[idx2] * (GRAPH_HEIGHT - 2) / MAX_ACCEL_GRAPH);

    // Constrain y values to graph area
    y1 = constrain(y1, ACCEL_GRAPH_Y + 1, ACCEL_GRAPH_Y + GRAPH_HEIGHT - 1);
    y2 = constrain(y2, ACCEL_GRAPH_Y + 1, ACCEL_GRAPH_Y + GRAPH_HEIGHT - 1);

    tft.drawLine(x1, y1, x2, y2, ILI9341_YELLOW);

 
  }
}



void checkZeroButton() {
  // Read the current button state
  int reading = digitalRead(ZERO_BUTTON_PIN);

  // Check if the button state has changed
  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }

  // If the button state has been stable for the debounce delay
  if ((millis() - lastDebounceTime) > debounceDelay) {
    // If the button is pressed (LOW when using INPUT_PULLUP)
    if (reading == LOW) {
      zeroSensors();
    }
  }

  // Save the current button state for next comparison
  lastButtonState = reading;
}

void zeroSensors() {
  Serial.println("Zeroing sensors...");

  // Display zeroing message
  tft.fillRect(20, 100, 80, 20, ILI9341_RED);
  tft.setCursor(50, 110);
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(1.5);
  tft.println("ZEROING...");

  // For BNO085, we can't directly zero it, but we can note the current orientation
  // and use it as a reference in calculations

  // For ADXL375, we can't directly set offsets, but we can store the current values
  // and subtract them in our calculations
  sensors_event_t event;
  accel.getEvent(&event);

  // Store current acceleration values as reference (except keep gravity in Z)
  float accel_ref_x = event.acceleration.x;
  float accel_ref_y = event.acceleration.y - 9.8;
  float accel_ref_z = event.acceleration.z;  // Subtract gravity from Z

  // Log the reference values
  Serial.print("Accel reference - X: ");
  Serial.print(accel_ref_x);
  Serial.print(" Y: ");
  Serial.print(accel_ref_y);
  Serial.print(" Z: ");
  Serial.println(accel_ref_z);

  // Note: Since we can't set hardware offsets in the ADXL375,
  // we'll need to apply these reference values in our calculations
  // This is handled in the readSensors() function

  // For BMP390, store the current altitude as a reference point
  if (bmp.performReading()) {
    referenceAltitude = bmp.readAltitude(SEALEVELPRESSURE_HPA);
    Serial.print("Reference altitude set to: ");
    Serial.println(referenceAltitude);
  }

  // Reset max values
  maxAltitude = 0;
  maxAcceleration = 0;
  maxBaroVelo = 0;
  

  // Reset history arrays
  for (int i = 0; i < ALTITUDE_HISTORY_SIZE; i++) {
    altitudeHistory[i] = 0;
  }

  for (int i = 0; i < ACCEL_HISTORY_SIZE; i++) {
    accelHistory[i] = 0;
  }

  // Clear zeroing message after a delay
  delay(1000);
  updateDisplay();

  Serial.println("Sensors zeroed");

}
