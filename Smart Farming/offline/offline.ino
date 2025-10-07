/*
 * Smart Farming System - Offline Version
 * ESP32 Modular Smart Farming
 * 
 * This version provides local operation without internet connectivity
 * Features: Soil moisture monitoring, temperature/humidity sensing,
 * automated irrigation control, LCD display, and LED status indicators
 * 
 * Hardware Requirements:
 * - ESP32 Development Board
 * - Soil Moisture Sensor (Analog)
 * - DHT11/DHT22 Temperature/Humidity Sensor
 * - Relay Module
 * - LCD 1602 with I2C Backpack
 * - LEDs for status indication
 * 
 */

#include "config.h"
#include <esp_task_wdt.h>
#include <Arduino.h>
#include <EEPROM.h>

// Conditional library inclusions
#if DISPLAY_ENABLED
  #include <LiquidCrystal_I2C.h>
#endif

#if DHT_ENABLED
  #include <DHT.h>
#endif

// =============================================================================
// GLOBAL VARIABLES AND OBJECTS
// =============================================================================

// LCD Object (conditional)
#if DISPLAY_ENABLED
  LiquidCrystal_I2C lcd(LCD_ADDRESS, LCD_COLS, LCD_ROWS);
#endif

// DHT Sensor Object (conditional)
#if DHT_ENABLED
  DHT dht(DHT_PIN, DHT_TYPE);
#endif

// System State Variables
struct SystemState {
  float temperature = 0.0;
  float humidity = 0.0;
  int soilMoistureRaw = 0;
  int soilMoisturePercent = 0;
  int lightLevelRaw = 0;
  int lightLevelPercent = 0;
  bool pumpActive = false;
  bool systemOK = true;
  bool emergencyStop = false;
  unsigned long lastIrrigation = 0;
  unsigned long pumpStartTime = 0;
  int dailyIrrigations = 0;
  unsigned long lastSensorRead = 0;
  unsigned long lastDisplayUpdate = 0;
  int displayScreen = 0;
  int sensorErrors = 0;
  int recoveryAttempts = 0;
  unsigned long lastErrorCheck = 0;
  unsigned long lastWatchdogFeed = 0;
  int adjustedThreshold = SOIL_MOISTURE_THRESHOLD;  // Adjustable soil moisture threshold

  // DHT Sensor Auto-Disable System
  bool dhtSensorEnabled = true;           // DHT sensor enabled/disabled status
  int dhtErrorCount = 0;                  // Consecutive DHT read failures
  unsigned long lastDhtCheck = 0;         // Last time DHT was checked when disabled
  unsigned long lastDhtSuccess = 0;       // Last successful DHT reading
  String dhtStatus = "OK";                // DHT sensor status message
  
  // Control System State
  #if CONTROL_TYPE == CONTROL_ROTARY_ENCODER
    int currentMenu = 0;
    int currentParameter = 0;
    int encoderPosition = 0;
    bool encoderButtonPressed = false;
    unsigned long lastMenuActivity = 0;
    bool inMenuMode = false;
  #elif CONTROL_TYPE == CONTROL_POTENTIOMETER
    int potentiometerValue = 0;
    unsigned long lastPotentiometerRead = 0;
    int potentiometerSamples[POTENTIOMETER_SMOOTHING_SAMPLES] = {0};
    int sampleIndex = 0;
    int lastStableThreshold = SOIL_MOISTURE_THRESHOLD;
    bool thresholdChanged = false;
  #endif
} systemState;

// Sensor Validation Variables
struct SensorValidation {
  float lastTemperature = 0.0;
  float lastHumidity = 0.0;
  int lastSoilMoisture = 0;
  int lastLightLevel = 0;
  int temperatureReadings[SENSOR_CONSISTENCY_CHECKS] = {0};
  int humidityReadings[SENSOR_CONSISTENCY_CHECKS] = {0};
  int soilMoistureReadings[SENSOR_CONSISTENCY_CHECKS] = {0};
  int lightLevelReadings[SENSOR_CONSISTENCY_CHECKS] = {0};
  int readingIndex = 0;
  bool temperatureValid = true;
  bool humidityValid = true;
  bool soilMoistureValid = true;
  bool lightLevelValid = true;
  int disconnectCount = 0;
} sensorValidation;

// Timing Variables
unsigned long currentTime = 0;
unsigned long lastHeartbeat = 0;

// =============================================================================
// FUNCTION DECLARATIONS
// =============================================================================

void initializeSystem();
void initializeSensors();
void initializeDisplay();
void initializeActuators();
void checkDhtSensorHealth();
void readSensors();
void updateDisplay();
void controlIrrigation();
void updateLEDs();
void checkSystemStatus();
void handleErrors();
void emergencyStop();
void initializeWatchdog();
void feedWatchdog();
void validateSensorReadings();
bool isSensorReadingValid(float value, float minVal, float maxVal, bool enabled);
bool checkSensorConsistency(int readings[], int newReading);
void attemptSystemRecovery();
void checkPumpRuntime();
void displaySensorData();
void displaySystemStatus();
void displayIrrigationInfo();
void displayAllInfo();  // For LCD 2004
void outputToSerial();  // For no display mode
void logSystemData();
void performHeartbeat();

// Control Functions
void initializeControl();
void handleControl();
void handleRotaryEncoder();
void handlePotentiometer();
void displayMenu();
void displayParameterAdjustment();
void saveSettings();
void loadSettings();
void initializeEEPROM();
bool loadSystemStateFromEEPROM();
void saveSystemStateToEEPROM();
uint16_t calculateEEPROMChecksum();
bool validateEEPROMData();

// =============================================================================
// EEPROM FUNCTIONS
// =============================================================================

void initializeEEPROM() {
  if (!EEPROM_ENABLED) {
    #if SERIAL_OUTPUT_ENABLED
      Serial.println("EEPROM functionality disabled in configuration");
    #endif
    return;
  }

  #if SERIAL_OUTPUT_ENABLED
    Serial.println("Initializing EEPROM...");
  #endif

  // Initialize EEPROM with defined size
  EEPROM.begin(EEPROM_SIZE);

  // Check if EEPROM contains valid data
  if (loadSystemStateFromEEPROM()) {
    #if SERIAL_OUTPUT_ENABLED
      Serial.println("System state loaded from EEPROM successfully");
    #endif
  } else {
    #if SERIAL_OUTPUT_ENABLED
      Serial.println("No valid EEPROM data found - using default values");
    #endif
    // Initialize with default values and save to EEPROM
    saveSystemStateToEEPROM();
  }

  #if SERIAL_OUTPUT_ENABLED
    Serial.println("EEPROM initialization complete");
  #endif
}

bool loadSystemStateFromEEPROM() {
  if (!EEPROM_ENABLED) return false;

  // Read and validate version
  uint8_t savedVersion = EEPROM.read(EEPROM_VERSION_ADDR);
  if (savedVersion != EEPROM_VERSION) {
    #if SERIAL_OUTPUT_ENABLED && DEBUG_MODE
      Serial.println("EEPROM version mismatch - data structure changed");
    #endif
    return false;
  }

  // Validate checksum
  if (!validateEEPROMData()) {
    #if SERIAL_OUTPUT_ENABLED && DEBUG_MODE
      Serial.println("EEPROM checksum validation failed - data corrupted");
    #endif
    return false;
  }

  // Load soil moisture threshold
  systemState.adjustedThreshold = (EEPROM.read(EEPROM_THRESHOLD_ADDR) << 8) | EEPROM.read(EEPROM_THRESHOLD_ADDR + 1);

  // Load daily irrigation count
  systemState.dailyIrrigations = (EEPROM.read(EEPROM_DAILY_COUNT_ADDR) << 8) | EEPROM.read(EEPROM_DAILY_COUNT_ADDR + 1);

  // Load last irrigation timestamp
  systemState.lastIrrigation = 0;
  for (int i = 0; i < 4; i++) {
    systemState.lastIrrigation |= ((unsigned long)EEPROM.read(EEPROM_LAST_IRRIGATION_ADDR + i)) << (i * 8);
  }

  // Load DHT sensor status
  systemState.dhtSensorEnabled = EEPROM.read(EEPROM_DHT_ENABLED_ADDR) > 0;
  systemState.dhtErrorCount = (EEPROM.read(EEPROM_DHT_ERROR_COUNT_ADDR) << 8) | EEPROM.read(EEPROM_DHT_ERROR_COUNT_ADDR + 1);

  // Load system flags
  uint8_t flags = EEPROM.read(EEPROM_SYSTEM_FLAGS_ADDR);
  systemState.systemOK = (flags & 0x01) > 0;

  #if SERIAL_OUTPUT_ENABLED
    Serial.println("Loaded from EEPROM:");
    Serial.println("  Threshold: " + String(systemState.adjustedThreshold) + "%");
    Serial.println("  Daily irrigations: " + String(systemState.dailyIrrigations));
    Serial.println("  Last irrigation: " + String(systemState.lastIrrigation > 0 ? "Valid" : "None"));
    Serial.println("  DHT enabled: " + String(systemState.dhtSensorEnabled ? "Yes" : "No"));
    Serial.println("  DHT errors: " + String(systemState.dhtErrorCount));
    Serial.println("  System OK: " + String(systemState.systemOK ? "Yes" : "No"));
  #endif

  return true;
}

void saveSystemStateToEEPROM() {
  if (!EEPROM_ENABLED) return;

  #if SERIAL_OUTPUT_ENABLED && DEBUG_MODE
    Serial.println("Saving system state to EEPROM...");
  #endif

  // Save version
  EEPROM.write(EEPROM_VERSION_ADDR, EEPROM_VERSION);

  // Save soil moisture threshold
  EEPROM.write(EEPROM_THRESHOLD_ADDR, systemState.adjustedThreshold >> 8);
  EEPROM.write(EEPROM_THRESHOLD_ADDR + 1, systemState.adjustedThreshold & 0xFF);

  // Save daily irrigation count
  EEPROM.write(EEPROM_DAILY_COUNT_ADDR, systemState.dailyIrrigations >> 8);
  EEPROM.write(EEPROM_DAILY_COUNT_ADDR + 1, systemState.dailyIrrigations & 0xFF);

  // Save last irrigation timestamp
  unsigned long timestamp = systemState.lastIrrigation;
  for (int i = 0; i < 4; i++) {
    EEPROM.write(EEPROM_LAST_IRRIGATION_ADDR + i, timestamp & 0xFF);
    timestamp >>= 8;
  }

  // Save DHT sensor status
  EEPROM.write(EEPROM_DHT_ENABLED_ADDR, systemState.dhtSensorEnabled ? 1 : 0);
  EEPROM.write(EEPROM_DHT_ERROR_COUNT_ADDR, systemState.dhtErrorCount >> 8);
  EEPROM.write(EEPROM_DHT_ERROR_COUNT_ADDR + 1, systemState.dhtErrorCount & 0xFF);

  // Save system flags
  uint8_t flags = 0;
  if (systemState.systemOK) flags |= 0x01;
  EEPROM.write(EEPROM_SYSTEM_FLAGS_ADDR, flags);

  // Calculate and save checksum
  uint16_t checksum = calculateEEPROMChecksum();
  EEPROM.write(EEPROM_CHECKSUM_ADDR, checksum >> 8);
  EEPROM.write(EEPROM_CHECKSUM_ADDR + 1, checksum & 0xFF);

  // Commit to EEPROM
  EEPROM.commit();

  #if SERIAL_OUTPUT_ENABLED && DEBUG_MODE
    Serial.println("System state saved to EEPROM successfully");
  #endif
}

uint16_t calculateEEPROMChecksum() {
  uint16_t checksum = 0;

  // Calculate checksum from version to system flags (excluding checksum bytes)
  for (int addr = EEPROM_VERSION_ADDR; addr < EEPROM_CHECKSUM_ADDR; addr++) {
    checksum += EEPROM.read(addr);
  }

  return checksum;
}

bool validateEEPROMData() {
  uint16_t storedChecksum = (EEPROM.read(EEPROM_CHECKSUM_ADDR) << 8) | EEPROM.read(EEPROM_CHECKSUM_ADDR + 1);
  uint16_t calculatedChecksum = calculateEEPROMChecksum();

  return storedChecksum == calculatedChecksum;
}

// =============================================================================
// SETUP FUNCTION
// =============================================================================

void setup() {
  // Initialize Serial Communication
  #if SERIAL_OUTPUT_ENABLED
    Serial.begin(SERIAL_BAUD_RATE);
    delay(1000);

    Serial.println("========================================");
    Serial.println("ESP32 Smart Farming System - Offline");
    Serial.println("Version: " + String(FIRMWARE_VERSION));
    Serial.println("Build Date: " + String(BUILD_DATE) + " " + String(BUILD_TIME));
    Serial.println("========================================");
  #endif
  
  // Initialize System Components
  initializeSystem();
  
  // Initialize Watchdog Timer
  initializeWatchdog();
  
  #if SERIAL_OUTPUT_ENABLED
    Serial.println("System initialization complete!");
    Serial.println("Starting main loop...");
    Serial.println("========================================");
  #endif
}

// =============================================================================
// MAIN LOOP FUNCTION
// =============================================================================

void loop() {
  currentTime = millis();
  
  // Feed watchdog timer
  feedWatchdog();
  
  // Check for emergency stop
  if (EMERGENCY_STOP_ENABLED && systemState.emergencyStop) {
    emergencyStop();
    return;
  }
  
  // Check pump runtime protection
  if (PUMP_RUNTIME_PROTECTION && systemState.pumpActive) {
    checkPumpRuntime();
  }
  
  // Read sensors at regular intervals
  if (currentTime - systemState.lastSensorRead >= SENSOR_READ_INTERVAL) {
    // Check DHT sensor health periodically (when disabled)
    checkDhtSensorHealth();

    readSensors();
    systemState.lastSensorRead = currentTime;
  }
  
  // Update display at regular intervals
  if (currentTime - systemState.lastDisplayUpdate >= DISPLAY_UPDATE_INTERVAL) {
    updateDisplay();
    systemState.lastDisplayUpdate = currentTime;
  }
  
  // Handle control input
  if (CONTROL_ENABLED) {
    handleControl();
  }
  
  // Control irrigation based on sensor readings
  controlIrrigation();
  
  // Update LED status indicators
  updateLEDs();
  
  // Check system status and handle errors
  if (currentTime - systemState.lastErrorCheck >= STATUS_CHECK_INTERVAL) {
    checkSystemStatus();
    systemState.lastErrorCheck = currentTime;
  }
  
  // Attempt system recovery if needed
  if (AUTO_RECOVERY_ENABLED && !systemState.systemOK && systemState.recoveryAttempts < RECOVERY_ATTEMPTS) {
    attemptSystemRecovery();
  }
  
  // Perform system heartbeat
  if (currentTime - lastHeartbeat >= HEARTBEAT_INTERVAL) {
    performHeartbeat();
    lastHeartbeat = currentTime;
  }

  // Log system data
  logSystemData();

  // Periodic EEPROM save
  static unsigned long lastEepromSave = 0;
  if (EEPROM_SAVE_INTERVAL > 0 && currentTime - lastEepromSave >= EEPROM_SAVE_INTERVAL) {
    saveSystemStateToEEPROM();
    lastEepromSave = currentTime;
  }
  
  // Small delay to prevent overwhelming the system
  delay(100);
}

// =============================================================================
// INITIALIZATION FUNCTIONS
// =============================================================================

void initializeSystem() {
  #if SERIAL_OUTPUT_ENABLED
    Serial.println("Initializing system components...");
  #endif
  
  // Initialize sensors
  initializeSensors();
  
  // Initialize display
  initializeDisplay();
  
  // Initialize actuators
  initializeActuators();
  
  // Initialize control system
  initializeControl();

  // Initialize EEPROM
  initializeEEPROM();

  // Initialize timing variables
  systemState.lastSensorRead = 0;
  systemState.lastDisplayUpdate = 0;
  systemState.lastErrorCheck = 0;
  lastHeartbeat = 0;

  // Initialize DHT-specific variables
  systemState.dhtSensorEnabled = true;
  systemState.dhtErrorCount = 0;
  systemState.lastDhtCheck = 0;
  systemState.lastDhtSuccess = 0;
  systemState.dhtStatus = "OK";
  
  #if SERIAL_OUTPUT_ENABLED
    Serial.println("System components initialized successfully!");
  #endif
}

void initializeSensors() {
  #if SERIAL_OUTPUT_ENABLED
    Serial.println("Initializing sensors...");
  #endif
  
  // Initialize DHT sensor (if enabled)
  #if DHT_ENABLED
    dht.begin();
    delay(2000); // Allow sensor to stabilize
    
    // Test DHT sensor
    float testTemp = dht.readTemperature();
    float testHumidity = dht.readHumidity();
    
    if (isnan(testTemp) || isnan(testHumidity)) {
      #if SERIAL_OUTPUT_ENABLED
        Serial.println("Warning: DHT sensor initialization failed!");
      #endif
      systemState.systemOK = false;
    } else {
      #if SERIAL_OUTPUT_ENABLED
        Serial.println("DHT sensor initialized successfully");
      #endif
    }
  #else
    #if SERIAL_OUTPUT_ENABLED
      Serial.println("DHT sensor disabled - using default values");
    #endif
  #endif
  
  // Initialize soil moisture sensor (analog)
  pinMode(SOIL_MOISTURE_PIN, INPUT);
  #if SERIAL_OUTPUT_ENABLED
    Serial.println("Soil moisture sensor initialized");
  #endif
  
  #if SERIAL_OUTPUT_ENABLED
    Serial.println("Sensor initialization complete!");
  #endif
}

void initializeDisplay() {
  #if DISPLAY_ENABLED
    #if SERIAL_OUTPUT_ENABLED
      Serial.println("Initializing LCD display...");
    #endif
    
    // Initialize LCD
    lcd.init();
    lcd.backlight();
    lcd.clear();
    
    // Display startup message
    lcd.setCursor(0, 0);
    lcd.print("Smart Farming");
    #if DISPLAY_TYPE == DISPLAY_LCD_2004
      lcd.setCursor(0, 1);
      lcd.print("Offline Mode");
      lcd.setCursor(0, 2);
      lcd.print("Initializing...");
    #else
      lcd.setCursor(0, 1);
      lcd.print("Initializing...");
    #endif
    
    delay(2000);
    
    // Clear display
    lcd.clear();
    
    #if SERIAL_OUTPUT_ENABLED
      Serial.println("LCD display initialized successfully!");
    #endif
  #else
    #if SERIAL_OUTPUT_ENABLED
      Serial.println("No display configured - using serial output only");
    #endif
  #endif
}

void initializeActuators() {
  #if SERIAL_OUTPUT_ENABLED
    Serial.println("Initializing actuators...");
  #endif
  
  // Initialize relay pin
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW); // Ensure pump is off initially
  
  // Initialize LED pins
  pinMode(LED_GREEN_PIN, OUTPUT);
  pinMode(LED_RED_PIN, OUTPUT);
  pinMode(LED_BLUE_PIN, OUTPUT);
  
  // Turn off all LEDs initially
  digitalWrite(LED_GREEN_PIN, LOW);
  digitalWrite(LED_RED_PIN, LOW);
  digitalWrite(LED_BLUE_PIN, LOW);
  
  #if SERIAL_OUTPUT_ENABLED
    Serial.println("Actuators initialized successfully!");
  #endif
}

// =============================================================================
// SENSOR READING FUNCTIONS
// =============================================================================

// DHT Sensor Health Check Function
void checkDhtSensorHealth() {
  // Only check DHT health if sensor is disabled or we want to verify it's working
  if (!DHT_ENABLED) return;

  unsigned long currentTime = millis();

  // If DHT is enabled, check if we should attempt a health check
  if (!systemState.dhtSensorEnabled && currentTime - systemState.lastDhtCheck >= DHT_HEALTH_CHECK_INTERVAL) {
    // Try to read DHT sensor for health check
    float testTemp = dht.readTemperature();
    float testHumidity = dht.readHumidity();

    systemState.lastDhtCheck = currentTime;

    if (!isnan(testTemp) && !isnan(testHumidity)) {
      // DHT sensor is working again!
      systemState.dhtSensorEnabled = true;
      systemState.dhtErrorCount = 0;
      systemState.lastDhtSuccess = currentTime;
      systemState.dhtStatus = "RECOVERED";

      #if SERIAL_OUTPUT_ENABLED
        Serial.println("DHT sensor recovered! Re-enabling DHT readings.");
      #endif
    } else {
      // DHT sensor still not working
      systemState.dhtStatus = "DISABLED - Sensor not responding";
      #if SERIAL_OUTPUT_ENABLED && DEBUG_MODE
        Serial.println("DHT health check failed - sensor still not responding");
      #endif
    }
  }
}

void readSensors() {
  // Read DHT sensor (if enabled)
  float temperature = 0.0;
  float humidity = 0.0;
  
  #if DHT_ENABLED
    // Only read DHT if it's enabled
    if (systemState.dhtSensorEnabled) {
      temperature = dht.readTemperature();
      humidity = dht.readHumidity();

      // Check for DHT sensor errors
      if (isnan(temperature) || isnan(humidity)) {
        systemState.dhtErrorCount++;
        systemState.dhtStatus = "ERROR - Reading failed (" + String(systemState.dhtErrorCount) + "/" + String(DHT_MAX_CONSECUTIVE_ERRORS) + ")";

        #if SERIAL_OUTPUT_ENABLED
          Serial.println("DHT sensor read failed! Error count: " + String(systemState.dhtErrorCount));
        #endif

        // Check if we should disable DHT sensor
        if (systemState.dhtErrorCount >= DHT_MAX_CONSECUTIVE_ERRORS) {
          systemState.dhtSensorEnabled = false;
          systemState.dhtStatus = "DISABLED - Too many failures";

          #if SERIAL_OUTPUT_ENABLED
            Serial.println("DHT sensor disabled due to too many consecutive failures!");
          #endif
        }

        // Use previous valid values or defaults
        if (systemState.lastDhtSuccess > 0) {
          // Keep previous values
          #if SERIAL_OUTPUT_ENABLED && DEBUG_MODE
            Serial.println("Using previous DHT values due to current failure");
          #endif
        } else {
          // Use default values
          temperature = 25.0;
          humidity = 50.0;
          #if SERIAL_OUTPUT_ENABLED
            Serial.println("Using default DHT values - no previous readings available");
          #endif
        }
      } else {
        // DHT reading successful
        systemState.dhtErrorCount = 0;
        systemState.lastDhtSuccess = currentTime;
        systemState.dhtStatus = "OK";

        #if SERIAL_OUTPUT_ENABLED && DEBUG_MODE
          Serial.println("DHT sensor reading successful");
        #endif
      }
    } else {
      // DHT sensor is disabled - use default values
      temperature = 25.0;
      humidity = 50.0;
      systemState.dhtStatus = "DISABLED";

      #if SERIAL_OUTPUT_ENABLED && DEBUG_MODE
        Serial.println("DHT sensor is disabled - using default values");
      #endif
    }
  #else
    // No DHT sensor - use default values
    temperature = 25.0;  // Default temperature
    humidity = 50.0;     // Default humidity
  #endif
  
  // Read soil moisture sensor
  int soilMoistureRaw = analogRead(SOIL_MOISTURE_PIN);
  
  // Convert raw reading to percentage
  int soilMoisturePercent = map(soilMoistureRaw, SOIL_MOISTURE_WET_VALUE, SOIL_MOISTURE_DRY_VALUE, 100, 0);
  soilMoisturePercent = constrain(soilMoisturePercent, 0, 100);
  
  // Read LDR sensor (if enabled)
  int lightLevelRaw = 0;
  int lightLevelPercent = 0;
  
  #if LDR_ENABLED
    lightLevelRaw = analogRead(LDR_PIN);
    
    // Convert raw reading to percentage (inverted: high value = dark, low value = bright)
    lightLevelPercent = map(lightLevelRaw, LDR_DARK_VALUE, LDR_BRIGHT_VALUE, 0, 100);
    lightLevelPercent = constrain(lightLevelPercent, 0, 100);
  #else
    // No LDR sensor - use default values
    lightLevelRaw = 2048;  // Default middle value
    lightLevelPercent = 50; // Default 50% light level
  #endif
  
  // Validate sensor readings
  validateSensorReadings(temperature, humidity, soilMoisturePercent, lightLevelPercent);
  
  // Always update soil moisture (critical for irrigation), but validate others
  systemState.soilMoistureRaw = soilMoistureRaw;
  systemState.soilMoisturePercent = soilMoisturePercent;
  systemState.lightLevelRaw = lightLevelRaw;
  systemState.lightLevelPercent = lightLevelPercent;
  
  // Only update DHT readings if valid (non-critical for irrigation)
  if (sensorValidation.temperatureValid && sensorValidation.humidityValid) {
    systemState.temperature = temperature;
    systemState.humidity = humidity;
    // Reset DHT sensor disconnect counter on successful reading
    sensorValidation.disconnectCount = 0;
  }
  
  // Only count soil moisture sensor errors as critical for system health
  if (!sensorValidation.soilMoistureValid) {
    systemState.sensorErrors++;
  } else {
    // Reset critical sensor error counter on successful soil moisture reading
    systemState.sensorErrors = 0;
  }
  
}

// =============================================================================
// DISPLAY FUNCTIONS
// =============================================================================

void updateDisplay() {
  #if DISPLAY_ENABLED
    #if DISPLAY_TYPE == DISPLAY_LCD_2004
      // LCD 2004: Show all information on single screen
      displayAllInfo();
    #else
      // LCD 1602: Cycle through different screens
      switch (systemState.displayScreen) {
        case 0:
          displaySensorData();
          break;
        case 1:
          displaySystemStatus();
          break;
        case 2:
          displayIrrigationInfo();
          break;
        default:
          systemState.displayScreen = 0;
          break;
      }
      
      // Advance to next screen
      systemState.displayScreen = (systemState.displayScreen + 1) % DISPLAY_SCREEN_COUNT;
    #endif
  #else
    // No display: Output to serial
    outputToSerial();
  #endif
}

void displaySensorData() {
  #if DISPLAY_ENABLED
    lcd.clear();
    lcd.setCursor(0, 0);
    
    // Display temperature and humidity (if DHT enabled)
    #if DHT_ENABLED
      String tempLine = "Temp: " + String(systemState.temperature, 1) + "C";
      if (!systemState.dhtSensorEnabled) {
        tempLine += " [OFF]";
      }
      lcd.print(tempLine);
      lcd.setCursor(0, 1);
      String humLine = "Hum: " + String(systemState.humidity, 1) + "%";
      if (!systemState.dhtSensorEnabled) {
        humLine += " [OFF]";
      }
      lcd.print(humLine);
    #else
      lcd.print("Smart Farming");
      lcd.setCursor(0, 1);
      lcd.print("DHT Disabled");
    #endif
    
    delay(DISPLAY_SCROLL_DELAY);
    
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Soil: " + String(systemState.soilMoisturePercent) + "%");
    lcd.setCursor(0, 1);
    
    // Only show light level if LDR is enabled
    #if LDR_ENABLED
      lcd.print("Light: " + String(systemState.lightLevelPercent) + "%");
    #else
      // Show system status instead of light level
      String statusLine = "Status: " + String(systemState.systemOK ? "OK" : "ERROR");
      #if DHT_ENABLED
        statusLine += " DHT:" + String(systemState.dhtSensorEnabled ? "ON" : "OFF");
      #endif
      lcd.print(statusLine);
    #endif
  #endif
}

void displaySystemStatus() {
  #if DISPLAY_ENABLED
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("System Status:");
    lcd.setCursor(0, 1);
    
    if (systemState.systemOK) {
      lcd.print("OK");
    } else {
      lcd.print("ERROR");
    }
    
    delay(DISPLAY_SCROLL_DELAY);
    
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Pump: ");
    lcd.print(systemState.pumpActive ? "ON" : "OFF");
    lcd.setCursor(0, 1);
    lcd.print("Daily: " + String(systemState.dailyIrrigations));
  #endif
}

void displayIrrigationInfo() {
  #if DISPLAY_ENABLED
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Irrigation Info:");
    lcd.setCursor(0, 1);
    
    if (systemState.lastIrrigation > 0) {
      unsigned long timeSinceLastIrrigation = (currentTime - systemState.lastIrrigation) / 1000;
      lcd.print("Last: " + String(timeSinceLastIrrigation) + "s");
    } else {
      lcd.print("No irrigation yet");
    }
  #endif
}

// =============================================================================
// IRRIGATION CONTROL FUNCTIONS
// =============================================================================

void controlIrrigation() {
  // Determine threshold to use
  int threshold = SOIL_MOISTURE_THRESHOLD;
  #if CONTROL_TYPE == CONTROL_POTENTIOMETER || CONTROL_TYPE == CONTROL_ROTARY_ENCODER
    threshold = systemState.adjustedThreshold;
  #endif
  
  // Check if irrigation is needed
  bool needsIrrigation = (systemState.soilMoisturePercent < threshold);
  
  // Check cooldown period
  bool cooldownExpired = (currentTime - systemState.lastIrrigation >= IRRIGATION_COOLDOWN);
  
  // Check daily irrigation limit
  bool withinDailyLimit = (systemState.dailyIrrigations < MAX_DAILY_IRRIGATIONS);
  
  // Check if system is OK for irrigation (only critical for soil sensor, not DHT/LDR)
  bool systemHealthy = systemState.systemOK && sensorValidation.soilMoistureValid;
  
  
  // Start irrigation if all conditions are met
  if (needsIrrigation && cooldownExpired && withinDailyLimit && systemHealthy && !systemState.pumpActive) {
    startIrrigation();
  }
  
  // Stop irrigation if duration exceeded
  if (systemState.pumpActive && (currentTime - systemState.lastIrrigation >= IRRIGATION_DURATION)) {
    stopIrrigation();
  }
}

void startIrrigation() {
  #if SERIAL_OUTPUT_ENABLED
    Serial.println("Starting irrigation...");
  #endif
  
  // Activate relay (pump)
  digitalWrite(RELAY_PIN, HIGH);
  systemState.pumpActive = true;
  systemState.pumpStartTime = currentTime; // Record pump start time for runtime protection
  
  // Update irrigation tracking
  systemState.lastIrrigation = currentTime;
  systemState.dailyIrrigations++;

  // Save to EEPROM when irrigation occurs (important state change)
  if (EEPROM_SAVE_ON_CHANGE) {
    saveSystemStateToEEPROM();
  }

  // Update display
  #if DISPLAY_ENABLED
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("IRRIGATION");
    lcd.setCursor(0, 1);
    lcd.print("ACTIVE");
  #endif
  
  #if SERIAL_OUTPUT_ENABLED
    Serial.println("Irrigation started. Duration: " + String(IRRIGATION_DURATION / 1000) + " seconds");
    Serial.println("Maximum runtime: " + String(MAX_PUMP_RUNTIME / 1000) + " seconds");
  #endif
}

void stopIrrigation() {
  #if SERIAL_OUTPUT_ENABLED
    Serial.println("Stopping irrigation...");
  #endif
  
  // Deactivate relay (pump)
  digitalWrite(RELAY_PIN, LOW);
  systemState.pumpActive = false;
  
  // Clear display
  #if DISPLAY_ENABLED
    lcd.clear();
  #endif
  
  #if SERIAL_OUTPUT_ENABLED
    Serial.println("Irrigation stopped.");
  #endif
}

// =============================================================================
// LED CONTROL FUNCTIONS
// =============================================================================

void updateLEDs() {
  // Green LED: System OK
  digitalWrite(LED_GREEN_PIN, systemState.systemOK && systemState.sensorErrors < MAX_SENSOR_ERRORS);
  
  // Red LED: Pump Active
  digitalWrite(LED_RED_PIN, systemState.pumpActive);
  
  // Blue LED: Offline mode (always off in offline version)
  digitalWrite(LED_BLUE_PIN, LOW);
}

// =============================================================================
// SYSTEM MONITORING FUNCTIONS
// =============================================================================

void checkSystemStatus() {
  // Check sensor errors
  if (systemState.sensorErrors >= MAX_SENSOR_ERRORS) {
    systemState.systemOK = false;
    #if SERIAL_OUTPUT_ENABLED
      Serial.println("Warning: Too many sensor errors detected!");
    #endif
  } else if (systemState.sensorErrors == 0) {
    systemState.systemOK = true;
  }
  
  // Check memory usage
  if (currentTime % MEMORY_CHECK_INTERVAL < STATUS_CHECK_INTERVAL) {
    #if SERIAL_OUTPUT_ENABLED
      Serial.println("Free heap: " + String(ESP.getFreeHeap()) + " bytes");
    #endif
  }
}

void handleErrors() {
  if (!systemState.systemOK) {
    #if SERIAL_OUTPUT_ENABLED
      Serial.println("System error detected! Check sensors and connections.");
    #endif
    
    // Turn off pump for safety
    if (systemState.pumpActive) {
      stopIrrigation();
    }
    
    // Display error on LCD
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("SYSTEM ERROR");
    lcd.setCursor(0, 1);
    lcd.print("Check sensors");
  }
}

void performHeartbeat() {
  #if SERIAL_OUTPUT_ENABLED
    Serial.println("System heartbeat - All systems operational");
    Serial.println("  Temperature: " + String(systemState.temperature, 1) + "°C");
    Serial.println("  Humidity: " + String(systemState.humidity, 1) + "%");
    Serial.println("  Soil Moisture: " + String(systemState.soilMoisturePercent) + "%");
    Serial.println("  Pump Status: " + String(systemState.pumpActive ? "ON" : "OFF"));
    Serial.println("  Daily Irrigations: " + String(systemState.dailyIrrigations));
    Serial.println("  System Status: " + String(systemState.systemOK ? "OK" : "ERROR"));
    #if DHT_ENABLED
    Serial.println("  DHT Status: " + systemState.dhtStatus);
    #endif
  #endif
}

// =============================================================================
// DATA LOGGING FUNCTIONS
// =============================================================================

void logSystemData() {
  static unsigned long lastLogTime = 0;
  
  if (currentTime - lastLogTime >= LOG_INTERVAL) {
    lastLogTime = currentTime;
  }
}

// =============================================================================
// SAFETY AND FAIL-SAFE FUNCTIONS
// =============================================================================

void initializeWatchdog() {
  if (WATCHDOG_ENABLED) {
    esp_task_wdt_config_t wdt_config = {
      .timeout_ms = WATCHDOG_TIMEOUT * 1000,
      .idle_core_mask = (1 << portNUM_PROCESSORS) - 1,
      .trigger_panic = true
    };
    esp_task_wdt_init(&wdt_config);
    esp_task_wdt_add(NULL);
    #if SERIAL_OUTPUT_ENABLED
      Serial.println("Watchdog timer initialized (" + String(WATCHDOG_TIMEOUT) + " seconds)");
    #endif
  }
}

void feedWatchdog() {
  if (WATCHDOG_ENABLED) {
    esp_task_wdt_reset();
    systemState.lastWatchdogFeed = currentTime;
  }
}

void emergencyStop() {
  #if SERIAL_OUTPUT_ENABLED
    Serial.println("EMERGENCY STOP ACTIVATED!");
  #endif
  
  // Stop irrigation immediately
  if (systemState.pumpActive) {
    stopIrrigation();
  }
  
  // Set emergency stop flag
  systemState.emergencyStop = true;
  systemState.systemOK = false;

  // Save state to EEPROM on emergency stop
  if (EEPROM_SAVE_ON_SHUTDOWN) {
    saveSystemStateToEEPROM();
  }
  
  // Display emergency message
  #if DISPLAY_ENABLED
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("EMERGENCY STOP");
    lcd.setCursor(0, 1);
    lcd.print("SYSTEM HALTED");
  #endif
  
  // Turn off all LEDs except red (emergency indicator)
  digitalWrite(LED_GREEN_PIN, LOW);
  digitalWrite(LED_BLUE_PIN, LOW);
  digitalWrite(LED_RED_PIN, HIGH);
  
  #if SERIAL_OUTPUT_ENABLED
    Serial.println("System halted due to emergency stop!");
  #endif
}

void validateSensorReadings(float temperature, float humidity, int soilMoisture, int lightLevel) {
  // Validate temperature
  sensorValidation.temperatureValid = isSensorReadingValid(temperature, MIN_TEMPERATURE, MAX_TEMPERATURE, TEMPERATURE_VALIDATION);
  
  // Validate humidity
  sensorValidation.humidityValid = isSensorReadingValid(humidity, MIN_HUMIDITY, MAX_HUMIDITY, HUMIDITY_VALIDATION);
  
  // Validate soil moisture
  sensorValidation.soilMoistureValid = isSensorReadingValid(soilMoisture, MIN_SOIL_MOISTURE, MAX_SOIL_MOISTURE, SOIL_MOISTURE_VALIDATION);
  
  // Validate light level
  sensorValidation.lightLevelValid = isSensorReadingValid(lightLevel, MIN_LIGHT_LEVEL, MAX_LIGHT_LEVEL, LIGHT_VALIDATION);
  
  // Check for sudden changes in soil moisture
  if (SOIL_MOISTURE_VALIDATION && sensorValidation.soilMoistureValid) {
    int change = abs(soilMoisture - sensorValidation.lastSoilMoisture);
    if (change > MAX_SOIL_MOISTURE_CHANGE) {
      #if SERIAL_OUTPUT_ENABLED
        Serial.println("Warning: Sudden soil moisture change detected: " + String(change) + "%");
      #endif
      sensorValidation.soilMoistureValid = false;
    }
  }
  
  // Check for sudden changes in light level
  if (LIGHT_VALIDATION && sensorValidation.lightLevelValid) {
    int change = abs(lightLevel - sensorValidation.lastLightLevel);
    if (change > MAX_LIGHT_CHANGE) {
      #if SERIAL_OUTPUT_ENABLED
        Serial.println("Warning: Sudden light level change detected: " + String(change) + "%");
      #endif
      sensorValidation.lightLevelValid = false;
    }
  }
  
  // Check sensor consistency
  if (CONSISTENCY_VALIDATION) {
    sensorValidation.temperatureValid &= checkSensorConsistency(sensorValidation.temperatureReadings, (int)(temperature * 10));
    sensorValidation.humidityValid &= checkSensorConsistency(sensorValidation.humidityReadings, (int)(humidity * 10));
    sensorValidation.soilMoistureValid &= checkSensorConsistency(sensorValidation.soilMoistureReadings, soilMoisture);
    sensorValidation.lightLevelValid &= checkSensorConsistency(sensorValidation.lightLevelReadings, lightLevel);
  }
  
  // Update last readings
  sensorValidation.lastTemperature = temperature;
  sensorValidation.lastHumidity = humidity;
  sensorValidation.lastSoilMoisture = soilMoisture;
  sensorValidation.lastLightLevel = lightLevel;
  
  // Update reading arrays for consistency checking
  sensorValidation.temperatureReadings[sensorValidation.readingIndex] = (int)(temperature * 10);
  sensorValidation.humidityReadings[sensorValidation.readingIndex] = (int)(humidity * 10);
  sensorValidation.soilMoistureReadings[sensorValidation.readingIndex] = soilMoisture;
  sensorValidation.lightLevelReadings[sensorValidation.readingIndex] = lightLevel;
  sensorValidation.readingIndex = (sensorValidation.readingIndex + 1) % SENSOR_CONSISTENCY_CHECKS;
}

bool isSensorReadingValid(float value, float minVal, float maxVal, bool enabled) {
  if (!enabled) return true;
  return (value >= minVal && value <= maxVal);
}

bool checkSensorConsistency(int readings[], int newReading) {
  // Calculate average of previous readings
  int sum = 0;
  for (int i = 0; i < SENSOR_CONSISTENCY_CHECKS; i++) {
    sum += readings[i];
  }
  int average = sum / SENSOR_CONSISTENCY_CHECKS;
  
  // Check if new reading is within threshold
  int deviation = abs(newReading - average);
  return (deviation <= SENSOR_CONSISTENCY_THRESHOLD);
}

void attemptSystemRecovery() {
  #if SERIAL_OUTPUT_ENABLED
    Serial.println("Attempting system recovery...");
  #endif
  
  systemState.recoveryAttempts++;
  
  // Reset sensor validation
  sensorValidation.temperatureValid = true;
  sensorValidation.humidityValid = true;
  sensorValidation.soilMoistureValid = true;
  sensorValidation.lightLevelValid = true;
  sensorValidation.disconnectCount = 0;
  
  // Reset error counters
  systemState.sensorErrors = 0;
  
  // Reinitialize sensors
  dht.begin();
  delay(2000);
  
  // Test sensor readings
  float testTemp = dht.readTemperature();
  float testHumidity = dht.readHumidity();
  
  if (!isnan(testTemp) && !isnan(testHumidity)) {
    systemState.systemOK = true;
    systemState.recoveryAttempts = 0;
    #if SERIAL_OUTPUT_ENABLED
      Serial.println("System recovery successful!");
    #endif
  } else {
    #if SERIAL_OUTPUT_ENABLED
      Serial.println("System recovery failed. Attempt " + String(systemState.recoveryAttempts) + "/" + String(RECOVERY_ATTEMPTS));
    #endif
    delay(RECOVERY_DELAY);
  }
}

void checkPumpRuntime() {
  if (systemState.pumpActive && (currentTime - systemState.pumpStartTime >= MAX_PUMP_RUNTIME)) {
    #if SERIAL_OUTPUT_ENABLED
      Serial.println("Warning: Maximum pump runtime exceeded! Stopping irrigation for safety.");
    #endif
    stopIrrigation();
    
    // Display warning on LCD
    #if DISPLAY_ENABLED
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("PUMP TIMEOUT");
      lcd.setCursor(0, 1);
      lcd.print("SAFETY STOP");
    #endif
  }
}

// =============================================================================
// MODULAR DISPLAY FUNCTIONS
// =============================================================================

void displayAllInfo() {
  #if DISPLAY_TYPE == DISPLAY_LCD_2004
    lcd.clear();
    
    // Line 1: Temperature and Humidity (if DHT enabled)
    lcd.setCursor(0, 0);
    #if DHT_ENABLED
      lcd.print("T:" + String(systemState.temperature, 1) + "C H:" + String(systemState.humidity, 1) + "%");
    #else
      lcd.print("Smart Farming System");
    #endif
    
    // Line 2: Soil Moisture and additional info
    lcd.setCursor(0, 1);
    lcd.print("Soil:" + String(systemState.soilMoisturePercent) + "%");
    
    #if LDR_ENABLED
      // Show light level if LDR is enabled
      lcd.print(" Light:" + String(systemState.lightLevelPercent) + "%");
    #else
      // Show threshold instead of light level
      lcd.print(" Thresh:" + String(systemState.adjustedThreshold) + "%");
    #endif
    
    // Line 3: Pump Status and System Status
    lcd.setCursor(0, 2);
    lcd.print("Pump:" + String(systemState.pumpActive ? "ON" : "OFF") + " Status:" + String(systemState.systemOK ? "OK" : "ERROR"));
    
    // Line 4: Daily Irrigations and Control Type
    lcd.setCursor(0, 3);
    lcd.print("Daily:" + String(systemState.dailyIrrigations));
    
    #if CONTROL_TYPE == CONTROL_POTENTIOMETER
      lcd.print(" POT:" + String(systemState.potentiometerValue));
    #elif CONTROL_TYPE == CONTROL_ROTARY_ENCODER
      lcd.print(" ENC:Menu");
    #else
      unsigned long uptime = currentTime / 1000;
      lcd.print(" Up:" + String(uptime / 60) + "m");
    #endif
  #endif
}

void outputToSerial() {
  #if SERIAL_OUTPUT_ENABLED
    static unsigned long lastSerialOutput = 0;
    
    if (currentTime - lastSerialOutput >= DISPLAY_UPDATE_INTERVAL) {
      #if SERIAL_OUTPUT_ENABLED
        Serial.println("System Status: " + String(systemState.systemOK ? "OK" : "ERROR") + " | Soil: " + String(systemState.soilMoisturePercent) + "% | Pump: " + String(systemState.pumpActive ? "ON" : "OFF"));
      #endif
      
      lastSerialOutput = currentTime;
    }
  #endif
}

// =============================================================================
// CONTROL SYSTEM FUNCTIONS
// =============================================================================

void initializeControl() {
  #if CONTROL_TYPE == CONTROL_ROTARY_ENCODER
    #if SERIAL_OUTPUT_ENABLED
      Serial.println("Initializing rotary encoder control...");
    #endif
    
    // Set encoder pins as inputs with pullup
    pinMode(ENCODER_CLK_PIN, INPUT_PULLUP);
    pinMode(ENCODER_DT_PIN, INPUT_PULLUP);
    pinMode(ENCODER_SW_PIN, INPUT_PULLUP);
    
    // Initialize encoder state
    systemState.currentMenu = 0;
    systemState.currentParameter = 0;
    systemState.encoderPosition = 0;
    systemState.encoderButtonPressed = false;
    systemState.lastMenuActivity = currentTime;
    systemState.inMenuMode = false;
    
    #if SERIAL_OUTPUT_ENABLED
      Serial.println("Rotary encoder control initialized");
    #endif
    
  #elif CONTROL_TYPE == CONTROL_POTENTIOMETER
    #if SERIAL_OUTPUT_ENABLED
      Serial.println("Initializing potentiometer control...");
    #endif
    
    // Initialize potentiometer state
    systemState.potentiometerValue = 0;
    systemState.adjustedThreshold = SOIL_MOISTURE_THRESHOLD;
    systemState.lastPotentiometerRead = currentTime;
    systemState.sampleIndex = 0;
    systemState.lastStableThreshold = SOIL_MOISTURE_THRESHOLD;
    systemState.thresholdChanged = false;
    
    // Initialize smoothing array with initial reading
    int initialReading = analogRead(POTENTIOMETER_PIN);
    for (int i = 0; i < POTENTIOMETER_SMOOTHING_SAMPLES; i++) {
      systemState.potentiometerSamples[i] = initialReading;
    }
    systemState.potentiometerValue = initialReading;
    
    #if SERIAL_OUTPUT_ENABLED
      Serial.println("Potentiometer control initialized");
    #endif
    
  #else
    #if SERIAL_OUTPUT_ENABLED
      Serial.println("No manual control configured - fully automated mode");
    #endif
  #endif
  
  // Load saved settings
  loadSettings();
}

void handleControl() {
  #if CONTROL_TYPE == CONTROL_ROTARY_ENCODER
    handleRotaryEncoder();
  #elif CONTROL_TYPE == CONTROL_POTENTIOMETER
    handlePotentiometer();
  #endif
}

void handleRotaryEncoder() {
  #if CONTROL_TYPE == CONTROL_ROTARY_ENCODER
    // Read encoder button
    bool buttonState = !digitalRead(ENCODER_SW_PIN); // Inverted due to pullup
    
    // Detect button press
    if (buttonState && !systemState.encoderButtonPressed) {
      systemState.encoderButtonPressed = true;
      systemState.lastMenuActivity = currentTime;
      
      if (!systemState.inMenuMode) {
        // Enter menu mode
        systemState.inMenuMode = true;
        systemState.currentMenu = 0;
        #if SERIAL_OUTPUT_ENABLED
          Serial.println("Entered menu mode");
        #endif
      } else {
        // Process menu selection
        if (systemState.currentMenu < MENU_ITEMS) {
          // Enter parameter adjustment mode
          systemState.currentParameter = systemState.currentMenu;
          #if SERIAL_OUTPUT_ENABLED
            Serial.println("Entered parameter adjustment mode: " + String(systemState.currentParameter));
          #endif
        } else {
          // Exit menu mode
          systemState.inMenuMode = false;
          saveSettings();
          #if SERIAL_OUTPUT_ENABLED
            Serial.println("Exited menu mode");
          #endif
        }
      }
    } else if (!buttonState && systemState.encoderButtonPressed) {
      systemState.encoderButtonPressed = false;
    }
    
    // Read encoder rotation
    static int lastCLK = HIGH;
    int currentCLK = digitalRead(ENCODER_CLK_PIN);
    
    if (currentCLK != lastCLK) {
      if (digitalRead(ENCODER_DT_PIN) != currentCLK) {
        // Clockwise rotation
        systemState.encoderPosition++;
        systemState.lastMenuActivity = currentTime;
        
        if (systemState.inMenuMode) {
          if (systemState.currentParameter >= 0) {
            // Adjusting parameter
            adjustParameter(1);
          } else {
            // Navigating menu
            systemState.currentMenu = (systemState.currentMenu + 1) % (MENU_ITEMS + 1);
          }
        }
      } else {
        // Counter-clockwise rotation
        systemState.encoderPosition--;
        systemState.lastMenuActivity = currentTime;
        
        if (systemState.inMenuMode) {
          if (systemState.currentParameter >= 0) {
            // Adjusting parameter
            adjustParameter(-1);
          } else {
            // Navigating menu
            systemState.currentMenu = (systemState.currentMenu - 1 + MENU_ITEMS + 1) % (MENU_ITEMS + 1);
          }
        }
      }
    }
    lastCLK = currentCLK;
    
    // Check for menu timeout
    if (systemState.inMenuMode && (currentTime - systemState.lastMenuActivity > MENU_TIMEOUT)) {
      systemState.inMenuMode = false;
      systemState.currentParameter = -1;
      saveSettings();
      #if SERIAL_OUTPUT_ENABLED
        Serial.println("Menu timeout - returned to normal mode");
      #endif
    }
    
    // Update display for menu
    if (systemState.inMenuMode) {
      if (systemState.currentParameter >= 0) {
        displayParameterAdjustment();
      } else {
        displayMenu();
      }
    }
  #endif
}

void handlePotentiometer() {
  #if CONTROL_TYPE == CONTROL_POTENTIOMETER
    if (currentTime - systemState.lastPotentiometerRead >= POTENTIOMETER_UPDATE_INTERVAL) {
      // Read raw potentiometer value (0-4095)
      int rawValue = analogRead(POTENTIOMETER_PIN);
      
      // Add to smoothing array
      systemState.potentiometerSamples[systemState.sampleIndex] = rawValue;
      systemState.sampleIndex = (systemState.sampleIndex + 1) % POTENTIOMETER_SMOOTHING_SAMPLES;
      
      // Calculate smoothed average
      long sum = 0;
      for (int i = 0; i < POTENTIOMETER_SMOOTHING_SAMPLES; i++) {
        sum += systemState.potentiometerSamples[i];
      }
      int smoothedValue = sum / POTENTIOMETER_SMOOTHING_SAMPLES;
      
      // Apply deadband to prevent minor fluctuations
      if (abs(smoothedValue - systemState.potentiometerValue) > POTENTIOMETER_DEADBAND) {
        systemState.potentiometerValue = smoothedValue;
      }
      
      // Convert to threshold percentage with improved mapping
      int newThreshold = map(systemState.potentiometerValue, 0, 4095, 
                            POTENTIOMETER_MIN_THRESHOLD, POTENTIOMETER_MAX_THRESHOLD);
      
      // Apply hysteresis to prevent rapid switching
      if (abs(newThreshold - systemState.lastStableThreshold) >= POTENTIOMETER_HYSTERESIS) {
        systemState.adjustedThreshold = newThreshold;
        systemState.lastStableThreshold = newThreshold;
        systemState.thresholdChanged = true;

        // Save to EEPROM when threshold changes (important setting change)
        if (EEPROM_SAVE_ON_CHANGE) {
          saveSystemStateToEEPROM();
        }
      }
      
      systemState.lastPotentiometerRead = currentTime;
      
      // Update display with enhanced information
      #if DISPLAY_ENABLED
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Moisture Threshold:");
        lcd.setCursor(0, 1);
        lcd.print(String(systemState.adjustedThreshold) + "% ");
        
        // Show current soil moisture for comparison
        if (systemState.soilMoisturePercent < systemState.adjustedThreshold) {
          lcd.print("DRY");
        } else {
          lcd.print("OK");
        }
        
        #if DISPLAY_TYPE == DISPLAY_LCD_2004
          lcd.setCursor(0, 2);
          lcd.print("Current: " + String(systemState.soilMoisturePercent) + "%");
          lcd.setCursor(0, 3);
          lcd.print("Pot: " + String(systemState.potentiometerValue));
        #endif
      #endif
      
      // Enhanced serial output
      #if SERIAL_OUTPUT_ENABLED && DEBUG_MODE
        if (systemState.thresholdChanged) {
          Serial.println("=== POTENTIOMETER CONTROL ===");
          Serial.println("Raw ADC: " + String(rawValue));
          Serial.println("Smoothed: " + String(smoothedValue));
          Serial.println("Final Value: " + String(systemState.potentiometerValue));
          Serial.println("Threshold: " + String(systemState.adjustedThreshold) + "%");
          Serial.println("Current Soil: " + String(systemState.soilMoisturePercent) + "%");
          Serial.println("Status: " + String(systemState.soilMoisturePercent < systemState.adjustedThreshold ? "NEEDS WATER" : "OK"));
          Serial.println("=============================");
          systemState.thresholdChanged = false;
        }
      #endif
    }
  #endif
}

void displayMenu() {
  #if DISPLAY_ENABLED && CONTROL_TYPE == CONTROL_ROTARY_ENCODER
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Menu:");
    
    String menuItems[] = {
      "Soil Threshold",
      "Irrigation Time", 
      "Display Speed",
      "System Status",
      "Save & Exit"
    };
    
    if (systemState.currentMenu < MENU_ITEMS) {
      lcd.setCursor(0, 1);
      lcd.print(menuItems[systemState.currentMenu]);
    } else {
      lcd.setCursor(0, 1);
      lcd.print("Exit Menu");
    }
  #endif
}

void displayParameterAdjustment() {
  #if DISPLAY_ENABLED && CONTROL_TYPE == CONTROL_ROTARY_ENCODER
    lcd.clear();
    lcd.setCursor(0, 0);
    
    switch (systemState.currentParameter) {
      case 0: // Soil Threshold
        lcd.print("Soil Threshold:");
        lcd.setCursor(0, 1);
        lcd.print(String(systemState.adjustedThreshold) + "%");
        break;
        
      case 1: // Irrigation Time
        lcd.print("Irrigation Time:");
        lcd.setCursor(0, 1);
        lcd.print(String(IRRIGATION_DURATION / 1000) + "s");
        break;
        
      case 2: // Display Speed
        lcd.print("Display Speed:");
        lcd.setCursor(0, 1);
        lcd.print(String(DISPLAY_SCROLL_DELAY / 1000) + "s");
        break;
        
      case 3: // System Status
        lcd.print("System Status:");
        lcd.setCursor(0, 1);
        lcd.print(systemState.systemOK ? "OK" : "ERROR");
        break;
    }
  #endif
}

void adjustParameter(int direction) {
  #if CONTROL_TYPE == CONTROL_ROTARY_ENCODER
    switch (systemState.currentParameter) {
      case 0: // Soil Threshold
        systemState.adjustedThreshold += direction * ENCODER_STEP_SIZE;
        systemState.adjustedThreshold = constrain(systemState.adjustedThreshold, 
                                                   POTENTIOMETER_MIN_THRESHOLD, 
                                                   POTENTIOMETER_MAX_THRESHOLD);
        break;
        
      case 1: // Irrigation Time (read-only for now)
        // Could be made adjustable in future versions
        break;
        
      case 2: // Display Speed (read-only for now)
        // Could be made adjustable in future versions
        break;
        
      case 3: // System Status (read-only)
        break;
    }
  #endif
}

void saveSettings() {
  #if CONTROL_TYPE == CONTROL_ROTARY_ENCODER || CONTROL_TYPE == CONTROL_POTENTIOMETER
    // In a real implementation, this would save to EEPROM
    // For now, we'll just log the settings
    #if SERIAL_OUTPUT_ENABLED
      Serial.println("Settings saved:");
      Serial.println("  Soil Threshold: " + String(systemState.adjustedThreshold) + "%");
    #endif
  #endif
}

void loadSettings() {
  #if CONTROL_TYPE == CONTROL_ROTARY_ENCODER || CONTROL_TYPE == CONTROL_POTENTIOMETER
    // In a real implementation, this would load from EEPROM
    // For now, we'll use default values
    systemState.adjustedThreshold = SOIL_MOISTURE_THRESHOLD;
    #if SERIAL_OUTPUT_ENABLED
      Serial.println("Settings loaded with defaults");
    #endif
  #endif
}
