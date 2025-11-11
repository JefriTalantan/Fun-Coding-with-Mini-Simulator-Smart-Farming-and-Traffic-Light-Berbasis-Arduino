/*
 * Kode Smart Farming OTOMATIS Offline
 * - Sensor Tanah + Pompa Otomatis
 * - Sensor DHT (Suhu & Kelembapan)
 */

// 1. LIBRARY
#define BLYNK_PRINT Serial 
#include <DHT.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// 2. DEFINISI PIN HARDWARE
#define DHTPIN 13
#define DHTTYPE DHT22
#define SOIL_PIN 34
#define RELAY_PIN 5

// 3. LOGIKA RELAY (Normal, Active HIGH)
#define RELAY_ON HIGH
#define RELAY_OFF LOW

// 4. LOGIKA AMBANG BATAS (Bisa Anda sesuaikan)
#define AMBANG_KERING_POMPA 50 
#define AMBANG_BASAH_POMPA 85 
#define AMBANG_STATUS_LCD 50 

// 5. INISIALISASI OBJEK
DHT dht(DHTPIN, DHTTYPE);
LiquidCrystal_I2C lcd(0x27, 16, 2);

// 6. VARIABEL GLOBAL
float suhu;
float kelembapan;
int lembapTanah;
int lembapTanahPersen;
bool isPompaOn = false;

// 7. PENGATURAN TIMER
unsigned long previousMillis = 0;
const long interval = 5000; 

const int KERING = 3050; 
const int BASAH = 1350; 


// Fungsi ini akan dipanggil berulang kali oleh loop()
void runSmartFarmingLogic() {
  // 1. Baca Suhu & Kelembapan Udara
  suhu = dht.readTemperature();
  kelembapan = dht.readHumidity();

  if (isnan(suhu) || isnan(kelembapan)) {
    Serial.println("Gagal membaca dari sensor DHT!");
  }

  // 2. Baca Kelembapan Tanah
  lembapTanah = analogRead(SOIL_PIN);
  lembapTanahPersen = map(lembapTanah, KERING, BASAH, 0, 100);
  lembapTanahPersen = constrain(lembapTanahPersen, 0, 100);

  // 3. LOGIKA POMPA OTOMATIS (Histeresis)
  if (lembapTanahPersen < AMBANG_KERING_POMPA) {
    // Tanah terlalu kering, nyalakan pompa
    if (isPompaOn == false) { 
        Serial.println("Tanah kering! Menyalakan pompa...");
        digitalWrite(RELAY_PIN, RELAY_ON);
        isPompaOn = true;
    }
  } 
  else if (lembapTanahPersen > AMBANG_BASAH_POMPA) {
    // Tanah sudah basah, matikan pompa
    if (isPompaOn == true) { 
        Serial.println("Tanah sudah basah! Mematikan pompa...");
        digitalWrite(RELAY_PIN, RELAY_OFF);
        isPompaOn = false;
    }
  }

  // 4. Cetak ke Serial Monitor
  Serial.print("Suhu: "); Serial.print(suhu);
  Serial.print("°C, Kelembapan: "); Serial.print(kelembapan);
  Serial.print("%, Tanah: "); Serial.print(lembapTanahPersen);
  Serial.print("%, Pompa: "); Serial.println(isPompaOn ? "ON" : "OFF");
  
  // 5. Tampilkan ke LCD
  lcd.clear(); 
  
  lcd.setCursor(0, 0);
  lcd.print("S:"); 
  if (!isnan(suhu)) { 
    lcd.print(suhu, 1);
  } else {
    lcd.print("ERR");
  }
  lcd.print("C ");
  
  lcd.print("H:");
  if (!isnan(kelembapan)) { 
    lcd.print(kelembapan, 1);
  } else {
    lcd.print("ERR");
  }
  lcd.print("%");

  lcd.setCursor(0, 1);
  lcd.print("Tanah:");
  lcd.print(lembapTanahPersen);
  lcd.print("% "); 
  if (lembapTanahPersen < AMBANG_STATUS_LCD) {
    lcd.print("Kering");
  } else {
    lcd.print("Basah "); 
  }
}

// ---------------------------------
// SETUP
// ---------------------------------
void setup() {
  Serial.begin(115200);
  delay(100);

  // Inisialisasi I2C dan LCD
  Wire.begin(21, 22);
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Smart Farm OFFLINE");
  lcd.setCursor(0, 1);
  lcd.print("Starting...");

  // Inisialisasi sensor dan pin
  dht.begin(); 
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, RELAY_OFF);
  isPompaOn = false; 

  delay(2000); 
  
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Sistem OTOMATIS");
  lcd.setCursor(0, 1);
  lcd.print("Berjalan...");
}

// ---------------------------------
// LOOP UTAMA
// ---------------------------------
void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    runSmartFarmingLogic();
  }
}