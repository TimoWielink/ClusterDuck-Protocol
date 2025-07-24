/**
 * @file MamaDuck.ino
 * @brief Uses the built in Mama Duck.
 */

 #include <string>
 #include <vector>
 #include <arduino-timer.h>
 #include <CDP.h>
 #include "FastLED.h"
 
 // Setup for W2812 (LED)
 #define LED_TYPE WS2812
 #define DATA_PIN 4
 #define NUM_LEDS 1
 #define COLOR_ORDER GRB
 #define BRIGHTNESS  128
 #include <pixeltypes.h>
 CRGB leds[NUM_LEDS];
 
 //GPS
 #include <TinyGPS++.h>
 TinyGPSPlus tgps;
 HardwareSerial GPS(1);
 
 //Telemetry - XPowersLib for T-Beam Supreme power management
 #include <XPowersLib.h>
 XPowersAXP2101 PMU;
 
 // BME280 sensor
 #include <Wire.h>
 #include <Adafruit_Sensor.h>
 #include <Adafruit_BME280.h>
 Adafruit_BME280 bme;
 
 #ifdef SERIAL_PORT_USBVIRTUAL
 #define Serial SERIAL_PORT_USBVIRTUAL
 #endif
 
 bool sendData(std::vector<byte> message);
 bool sendGPSData(std::vector<byte> message);
bool sendHealthData(std::vector<byte> message);
bool sendBME280Data(std::vector<byte> message);
bool runSensor(void *);
bool runGPS(void *);
bool runHealth(void *);
bool runBME280(void *);
void scanI2C();
 
 // create a built-in mama duck
 MamaDuck duck;
 
 // Create a Display Instance
 DuckDisplay* display = NULL;
 
 // create a timer with default settings
 auto timer = timer_create_default();
 
 // for sending the counter message 
 const int INTERVAL_MS = 1800000;  // ~30 minutes (1,800,000 ms)
 // GPS sends every 15 minutes
 const int GPS_INTERVAL_MS = 900000;  // 15 minutes (900,000 ms)
 // Health sends every 5 minutes
 const int HEALTH_INTERVAL_MS = 150000;  // 1.5 minutes (150,000 ms)
 // BME280 sends every 2 minutes
 const int BME280_INTERVAL_MS = 120000;  // 2 minutes (120,000 ms)
 int counter = 1;
 bool setupOK = false;
 bool pmuAvailable = false;
 
 void setup() {
   Serial.begin(115200);
   delay(1000);
   Serial.println("[MAMA] Starting T-Beam Supreme MamaDuck setup...");
   
   // Initialize LED but don't show yet
   FastLED.addLeds<LED_TYPE, DATA_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection( TypicalSMD5050 );
   FastLED.setBrightness(BRIGHTNESS);
   leds[0] = CRGB::Cyan;
   // Don't call FastLED.show() yet - wait until after radio setup
 
   std::string deviceId("TEST1234"); // Device must be 8 characters
   std::array<byte,8> devId;
   std::copy(deviceId.begin(), deviceId.end(), devId.begin());
   Serial.println(("[MAMA] Setting up MamaDuck with device ID: " + deviceId).c_str());
   if (duck.setupWithDefaults(devId) != DUCK_ERR_NONE) {
     Serial.println("[MAMA] Failed to setup MamaDuck");
     return;
   }
   Serial.println("[MAMA] MamaDuck setup successful");
 
   // Add display setup after setupWithDefaults
   #ifdef ENABLE_DISPLAY
   display = DuckDisplay::getInstance();
   display->setupDisplay(duck.getType(), devId);
 
   display->showDefaultScreen();
   Serial.println("[MAMA] Display setup OK!");
   #endif
 
   //Setup AXP2101 Power Management (T-Beam Supreme)
   // T-Beam Supreme PMU uses I2C pins 42 (SDA) and 41 (SCL) according to hardware specs
   Serial.println("[MAMA] Initializing I2C for AXP2101...");
   Wire.begin(42, 41);  // PMU AXP2101 I2C pins: SDA=42, SCL=41
   Wire.setClock(100000); // Set I2C clock to 100kHz for stability
   
   // Add delay to ensure I2C is ready
   delay(100);
   
   // Test I2C communication
   Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
   byte error = Wire.endTransmission();
   if (error == 0) {
     Serial.println("[MAMA] I2C communication test PASS");
   } else {
     Serial.print("[MAMA] I2C communication test FAIL, error: ");
     Serial.println(error);
   }
   
   // Scan for all I2C devices
   scanI2C();
   
   if (!PMU.begin(Wire, AXP2101_SLAVE_ADDRESS, 42, 41)) {
     Serial.println("[MAMA] AXP2101 Begin FAIL");
     Serial.println("[MAMA] Check I2C connections and power");
     pmuAvailable = false;
   } else {
     Serial.println("[MAMA] AXP2101 Begin PASS");
     pmuAvailable = true;
   }
   
   //Setup GPS
   // T-Beam Supreme GPS pins: RX=9, TX=8 (GPS_RX_PIN=IO09, GPS_TX_PIN=IO08)
   GPS.begin(9600, SERIAL_8N1, 9, 8);
   Serial.println("[MAMA] GPS initialized on pins RX=9, TX=8");
 
   //Setup BME280 sensor (I2C pins 17/18)
   Serial.println("[MAMA] Initializing BME280 sensor...");
   Wire1.begin(17, 18);  // Use Wire1 for BME280 to avoid conflict with PMU
   if (!bme.begin(0x76, &Wire1)) {  // Try address 0x76 first
     if (!bme.begin(0x77, &Wire1)) {  // Try address 0x77 as fallback
       Serial.println("[MAMA] BME280 sensor not found!");
     } else {
       Serial.println("[MAMA] BME280 sensor found at address 0x77");
     }
   } else {
     Serial.println("[MAMA] BME280 sensor found at address 0x76");
   }
 
   // Initializing the timer for runSensor
   timer.every(INTERVAL_MS, runSensor);
   
   // Initialize the timer for GPS (every 15 minutes)
   timer.every(GPS_INTERVAL_MS, runGPS);
   
   // Initialize the timer for Health (every 1.5 minutes)
   timer.every(HEALTH_INTERVAL_MS, runHealth);
   
   // Initialize the timer for BME280 (every 2 minutes)
   timer.every(BME280_INTERVAL_MS, runBME280);
   
   // Test health check immediately after setup
   if (pmuAvailable) {
     Serial.println("[MAMA] Running initial health check...");
     runHealth(nullptr);
   }
   
   // Setup Complete
   Serial.println("[MAMA] Setup OK!");
   setupOK = true;
   
   // Now safe to show LEDs after all radio setup is complete
   delay(100);  // Small delay before LED operations
   leds[0] = CRGB::Gold;
   FastLED.show();
   delay(50);   // Small delay after LED operations
 }
 
 std::vector<byte> stringToByteVector(const std::string& str) {
     std::vector<byte> byteVec;
     byteVec.reserve(str.length());
 
     for (unsigned int i = 0; i < str.length(); ++i) {
         byteVec.push_back(static_cast<byte>(str[i]));
     }
 
     return byteVec;
 }
 
 static void smartDelay(unsigned long ms)
 {
   unsigned long start = millis();
   do
   {
     while (GPS.available())
       tgps.encode(GPS.read());
   } while (millis() - start < ms);
 }
 
 void loop() {
   if (!setupOK) {
     return; 
   }
   timer.tick();
 
   duck.run();
 }
 
 bool runSensor(void *) {
   bool result;
   
   Serial.println("[MAMA] Sensor check triggered");
   
   // Create JSON formatted sensor data
   int mem = freeMemory();
   std::string message = "{\"counter\":" + std::to_string(counter) + ",\"freeMemory\":" + std::to_string(mem) + "}";
   
   Serial.print("[MAMA] sensor data (JSON): ");
   Serial.println(message.c_str());
   Serial.print("[MAMA] Counter: ");
   Serial.println(counter);
   Serial.print("[MAMA] Free memory: ");
   Serial.print(mem);
   Serial.println(" bytes");
   Serial.print("[MAMA] JSON length: ");
   Serial.println(message.length());
 
   result = sendData(stringToByteVector(message));
   if (result) {
      Serial.println("[MAMA] runSensor ok.");
   } else {
      Serial.println("[MAMA] runSensor failed.");
   }
   return result;
 }
 
 bool runGPS(void *) {
   bool result = false;
   
   // Encoding the GPS data
   smartDelay(5000);
   
   // Check if we have valid GPS data
   if (tgps.location.isValid()) {
     // Create JSON formatted GPS data
     std::string gpsMessage = "{\"lat\":" + std::to_string(tgps.location.lat()) + ",\"lng\":" + std::to_string(tgps.location.lng()) + "}";
     
     Serial.print("[MAMA] GPS data: ");
     Serial.println(gpsMessage.c_str());
     Serial.print("[MAMA] Satellites: ");
     Serial.println(tgps.satellites.value());
     
     result = sendGPSData(stringToByteVector(gpsMessage));
     if (result) {
        Serial.println("[MAMA] runGPS ok.");
     } else {
        Serial.println("[MAMA] runGPS failed.");
     }
   } else {
     Serial.println("[MAMA] GPS location not valid yet");
     
     // Check if GPS is receiving data
     if (millis() > 5000 && tgps.charsProcessed() < 10) {
       Serial.println("[MAMA] No GPS data received: check wiring");
     }
   }
   
   return result;
 }
 
 bool runHealth(void *) {
   bool result = false;
   
   Serial.println("[MAMA] Health check triggered");
   
   // Check if PMU is available
   if (!pmuAvailable) {
     Serial.println("[MAMA] PMU not available, skipping health check");
     return false;
   }
   
   // Check if battery is connected - but don't fail if no battery
   bool batteryConnected = PMU.isBatteryConnect();
   Serial.print("[MAMA] Battery connected: ");
   Serial.println(batteryConnected ? "Yes" : "No");
   
   // Continue with health check even if no battery (for USB power scenarios)
   
   // Get battery data from AXP2101 using XPowersLib methods
   float voltage = PMU.getBattVoltage() / 1000.0;  // Convert mV to V
   
   // Calculate battery percentage based on voltage (Li-ion battery curve)
   int percentage = 0;
   if (voltage >= 4.2) {
     percentage = 100;
   } else if (voltage >= 4.0) {
     percentage = 90 + (voltage - 4.0) * 50;  // 4.0V-4.2V: 90-100%
   } else if (voltage >= 3.8) {
     percentage = 70 + (voltage - 3.8) * 100; // 3.8V-4.0V: 70-90%
   } else if (voltage >= 3.6) {
     percentage = 30 + (voltage - 3.6) * 200; // 3.6V-3.8V: 30-70%
   } else if (voltage >= 3.4) {
     percentage = 10 + (voltage - 3.4) * 100; // 3.4V-3.6V: 10-30%
   } else if (voltage >= 3.2) {
     percentage = (voltage - 3.2) * 50;       // 3.2V-3.4V: 0-10%
   }
   
   bool charging = PMU.isVbusIn();                 // Check if charging
   
   // Get temperature from AXP2101 (convert from raw value to Celsius)
   float temperature = PMU.getTemperature() / 10.0;  // Convert from 0.1°C units to °C
   
   // Create JSON formatted health data with cleaner formatting
   char healthBuffer[256];  // Increased buffer size for safety
   memset(healthBuffer, 0, sizeof(healthBuffer));  // Clear buffer
   snprintf(healthBuffer, sizeof(healthBuffer)-1, "{\"voltage\":%.2f,\"percentage\":%d,\"charging\":%s,\"temperature\":%.1f}", 
            voltage, (int)percentage, charging ? "true" : "false", temperature);
   std::string healthMessage(healthBuffer);
   
   Serial.print("[MAMA] Health data: ");
   Serial.println(healthMessage.c_str());
   Serial.print("[MAMA] Battery voltage: ");
   Serial.print(voltage);
   Serial.println("V");
   Serial.print("[MAMA] Battery percentage: ");
   Serial.print(percentage);
   Serial.println("%");
   Serial.print("[MAMA] Charging: ");
   Serial.println(charging ? "Yes" : "No");
   Serial.print("[MAMA] VBUS connected: ");
   Serial.println(charging ? "Yes" : "No");
   Serial.print("[MAMA] Temperature: ");
   Serial.print(temperature);
   Serial.println("°C");
   
   result = sendHealthData(stringToByteVector(healthMessage));
   if (result) {
      Serial.println("[MAMA] runHealth ok.");
   } else {
      Serial.println("[MAMA] runHealth failed.");
   }
   
   return result;
 }
 
 bool sendData(std::vector<byte> message) {
   bool sentOk = false;
   
   int err = duck.sendData(topics::status, message);
   if (err == DUCK_ERR_NONE) {
      counter++;
      sentOk = true;
   }
   if (!sentOk) {
     std::string errMessage = "[MAMA] Failed to send data. error = " + std::to_string(err);
     Serial.println(errMessage.c_str());
   }
   return sentOk;
 }
 
 bool sendGPSData(std::vector<byte> message) {
   bool sentOk = false;
   
   int err = duck.sendData(topics::location, message);
   if (err == DUCK_ERR_NONE) {
      sentOk = true;
   }
   if (!sentOk) {
     std::string errMessage = "[MAMA] Failed to send GPS data. error = " + std::to_string(err);
     Serial.println(errMessage.c_str());
   }
   return sentOk;
 }
 
 bool sendHealthData(std::vector<byte> message) {
   bool sentOk = false;
   
   int err = duck.sendData(topics::health, message);
   if (err == DUCK_ERR_NONE) {
      sentOk = true;
   }
   if (!sentOk) {
     std::string errMessage = "[MAMA] Failed to send health data. error = " + std::to_string(err);
     Serial.println(errMessage.c_str());
   }
   return sentOk;
 }
 
 bool runBME280(void *) {
   bool result = false;
   
   Serial.println("[MAMA] BME280 check triggered");
   
   // Read BME280 sensor data
   float temperature = bme.readTemperature();
   float humidity = bme.readHumidity();
   float pressure = bme.readPressure() / 100.0;  // Convert Pa to hPa
   
   // Check if readings are valid (not NaN)
   if (isnan(temperature) || isnan(humidity) || isnan(pressure)) {
     Serial.println("[MAMA] BME280 sensor readings failed");
     return false;
   }
   
   // Create JSON formatted BME280 data
   char bmeBuffer[256];
   memset(bmeBuffer, 0, sizeof(bmeBuffer));
   snprintf(bmeBuffer, sizeof(bmeBuffer)-1, "{\"temperature\":%.2f,\"humidity\":%.2f,\"pressure\":%.2f}", 
            temperature, humidity, pressure);
   std::string bmeMessage(bmeBuffer);
   
   Serial.print("[MAMA] BME280 data: ");
   Serial.println(bmeMessage.c_str());
   Serial.print("[MAMA] Temperature: ");
   Serial.print(temperature);
   Serial.println("°C");
   Serial.print("[MAMA] Humidity: ");
   Serial.print(humidity);
   Serial.println("%");
   Serial.print("[MAMA] Pressure: ");
   Serial.print(pressure);
   Serial.println("hPa");
   
   result = sendBME280Data(stringToByteVector(bmeMessage));
   if (result) {
      Serial.println("[MAMA] runBME280 ok.");
   } else {
      Serial.println("[MAMA] runBME280 failed.");
   }
   
   return result;
 }
 
 bool sendBME280Data(std::vector<byte> message) {
   bool sentOk = false;
   
   int err = duck.sendData(topics::bmp280, message);
   if (err == DUCK_ERR_NONE) {
      sentOk = true;
   }
   if (!sentOk) {
     std::string errMessage = "[MAMA] Failed to send BME280 data. error = " + std::to_string(err);
     Serial.println(errMessage.c_str());
   }
   return sentOk;
 }
 
 // Function to scan I2C bus for debugging
 void scanI2C() {
   Serial.println("[MAMA] Scanning I2C bus...");
   byte error, address;
   int nDevices = 0;
   
   for(address = 1; address < 127; address++) {
     Wire.beginTransmission(address);
     error = Wire.endTransmission();
     
     if (error == 0) {
       Serial.print("[MAMA] I2C device found at address 0x");
       if (address < 16) {
         Serial.print("0");
       }
       Serial.println(address, HEX);
       nDevices++;
     }
   }
   
   if (nDevices == 0) {
     Serial.println("[MAMA] No I2C devices found");
   } else {
     Serial.print("[MAMA] Found ");
     Serial.print(nDevices);
     Serial.println(" I2C device(s)");
   }
 }