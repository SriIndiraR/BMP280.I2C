#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>

// BMP280 I2C connection
Adafruit_BMP280 bmp;  // I2C interface

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  while (!Serial);  // Wait for serial monitor to open (for native USB)
  delay(1000);
  
  Serial.println("\nBMP280 with Raspberry Pi Pico");

  // Initialize built-in LED
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  // Initialize I2C (default pins: GPIO4=SDA, GPIO5=SCL)
  Wire.begin();
  
  // Try to initialize BMP280 at different addresses
  if (!bmp.begin(0x76)) {  // Common address for BMP280
    Serial.println("Could not find BMP280 at address 0x76");
    if (!bmp.begin(0x77)) {  // Alternative address
      Serial.println("Could not find BMP280 at address 0x77 either");
      Serial.println("Please check wiring and I2C address!");
      while (1) {
        // Blink LED rapidly to indicate error
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
        delay(100);
      }
    }
  }

  // Configure BMP280 settings
  bmp.setSampling(
    Adafruit_BMP280::MODE_NORMAL,     // Operating mode
    Adafruit_BMP280::SAMPLING_X2,     // Temperature oversampling
    Adafruit_BMP280::SAMPLING_X16,    // Pressure oversampling
    Adafruit_BMP280::FILTER_X16,      // Filtering
    Adafruit_BMP280::STANDBY_MS_500   // Standby time
  );

  Serial.println("BMP280 initialized successfully");
}

void loop() {
  // Read sensor data
  float temperature = bmp.readTemperature();
  float pressure = bmp.readPressure() / 100.0F; // Convert Pa to hPa
  float altitude = bmp.readAltitude(1013.25);   // Default sea level pressure

  // Print data to serial
  Serial.print("Temperature = ");
  Serial.print(temperature);
  Serial.print(" °C");
  
  Serial.print("\tPressure = ");
  Serial.print(pressure);
  Serial.print(" hPa");
  
  Serial.print("\tApprox. Altitude = ");
  Serial.print(altitude);
  Serial.println(" m");

  // Blink LED to indicate activity
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);
  
  // Delay between readings (adjust as needed)
  delay(2000);
}