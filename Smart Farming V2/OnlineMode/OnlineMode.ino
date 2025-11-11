/*
 * Kode Smart Farming Online - BLYNK
 */

// Aktifkan baris ini jika Menggunakan DHT22
#define SENSOR_DHT DHT22 

// Aktifkan baris ini jika Menggunakan DHT11
// #define SENSOR_DHT DHT11 

// ----------------------------------------------------

// 1. PENGATURAN BLYNK
#define BLYNK_TEMPLATE_ID "Your Template Id Blynk"
#define BLYNK_TEMPLATE_NAME "Your Template Name Blynk"
#define BLYNK_AUTH_TOKEN "Your Auth Token Device Blynk"

// 2. LIBRARY
#define BLYNK_PRINT Serial
#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <DHT.h> 
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <WiFiUdp.h>
#include <NTPClient.h>

// 3. PENGATURAN WIFI
char ssid[] = "Your SSID Wifi"; 
char pass[] = "Your Password Wifi";

// 4. PENGATURAN JAM (NTP)
// Waktu Indonesia Timur (WIT) adalah UTC+9 (9 * 3600 = 32400 detik)
// GANTI "32400" jika Anda di zona waktu berbeda (WIB=25200, WITA=28800)
const long UTC_OFFSET_IN_SECONDS = 32400; 
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", UTC_OFFSET_IN_SECONDS);
String days[] = {"Minggu", "Senin", "Selasa", "Rabu", "Kamis", "Jumat", "Sabtu"};

// 5. DEFINISI PIN HARDWARE
#define DHTPIN 13 
#define SOIL_PIN 34
#define RELAY_PIN 5

// Logika Relay Normal (Active HIGH)
#define RELAY_ON HIGH
#define RELAY_OFF LOW

// Ambang batas untuk logika
#define AMBANG_KERING_AUTO 50 
#define AMBANG_BASAH_AUTO 85  
#define AMBANG_STATUS_KERING 50 

// 6. INISIALISASI OBJEK
DHT dht(DHTPIN, SENSOR_DHT); 
BlynkTimer timer;
LiquidCrystal_I2C lcd(0x27, 16, 2); 

// 7. VARIABEL GLOBAL
float suhu; 
float kelembapan; 
int lembapTanah;
int lembapTanahPersen;
bool isPompaOn = false;
bool isAutoMode = true;
int lcdSlideState = 0; 

const int KERING = 3050;
const int BASAH = 1350; 


// --- TAMBAHAN: Fungsi BLYNK_CONNECTED ---
// Fungsi ini berjalan SETIAP KALI ESP32 berhasil terhubung ke Blynk
BLYNK_CONNECTED() {
  Serial.println("Blynk Terhubung. Menyinkronkan status...");
  // Kirim status 'isAutoMode' saat ini (yaitu 'true' saat startup) ke saklar V4 di HP
  // Ini akan membuat saklar V4 di HP Anda otomatis 'ON' saat ESP32 menyala
  Blynk.virtualWrite(V4, isAutoMode ? 1 : 0); 
}

// --- Fungsi untuk Saklar Mode Auto (V4) ---
BLYNK_WRITE(V4) {
  isAutoMode = param.asInt() == 1; 
  if (isAutoMode) {
    Serial.println("Mode Otomatis DIAKTIFKAN");
  } else {
    Serial.println("Mode Manual DIAKTIFKAN");
    digitalWrite(RELAY_PIN, RELAY_OFF);
    isPompaOn = false;
    Blynk.virtualWrite(V3, 0); 
  }
}

// --- Fungsi Tombol Pompa Manual (V3) ---
BLYNK_WRITE(V3) {
  if (isAutoMode) {
    Serial.println("Tombol manual nonaktif saat mode auto.");
    Blynk.virtualWrite(V3, isPompaOn ? 1 : 0); 
    return; 
  }

  int statusTombol = param.asInt();
  if (statusTombol == 1) { 
    digitalWrite(RELAY_PIN, RELAY_ON); 
    isPompaOn = true;
    Serial.println("Pompa ON (Manual)");
  } else { 
    digitalWrite(RELAY_PIN, RELAY_OFF); 
    isPompaOn = false;
    Serial.println("Pompa OFF (Manual)");
  }
}

// --- FUNGSI LCD ---
void updateLCD() {
  lcd.clear(); 
  
  switch (lcdSlideState) {
    case 0: { // Slide 1: Hari dan Jam
      timeClient.update(); 
      lcd.setCursor(0, 0);
      lcd.print("Hari: ");
      lcd.print(days[timeClient.getDay()]); 
      lcd.setCursor(0, 1);
      lcd.print("Jam : ");
      lcd.print(timeClient.getFormattedTime()); 
      break;
    }
    case 1: { // Slide 2: Suhu
      lcd.setCursor(0, 0);
      lcd.print("Suhu: ");
      if (isnan(suhu)) { 
        lcd.print("Error");
      } else {
        lcd.print(suhu, 1);
        lcd.print((char)223);
        lcd.print("C");
      }
      lcd.setCursor(0, 1);
      lcd.print("Status: ");
      if (isnan(suhu)) {
        lcd.print("N/A");
      } else if (suhu > 30) {
        lcd.print("Panas");
      } else if (suhu < 20) {
        lcd.print("Dingin");
      } else {
        lcd.print("Hangat");
      }
      break;
    }
    case 2: { // Slide 3: Lembap
      lcd.setCursor(0, 0);
      lcd.print("Lembap: ");
      if (isnan(kelembapan)) { 
        lcd.print("Error");
      } else {
        lcd.print(kelembapan, 1);
        lcd.print("%");
      }
      lcd.setCursor(0, 1);
      lcd.print("Status: ");
      if (isnan(kelembapan)) {
        lcd.print("N/A");
      } else if (kelembapan >= 40 && kelembapan <= 60) {
        lcd.print("Baik");
      } else {
        lcd.print("Tdk Baik");
      }
      break;
    }
    case 3: { // Slide 4: Tanah & Pompa
      lcd.setCursor(0, 0);
      lcd.print("Tanah: ");
      lcd.print(lembapTanahPersen);
      lcd.print("% ");
      if (lembapTanahPersen < AMBANG_STATUS_KERING) {
        lcd.print("Kering");
      } else {
        lcd.print("Basah "); 
      }
      lcd.setCursor(0, 1);
      lcd.print("Pompa: ");
      lcd.print(isPompaOn ? "NYALA" : "MATI ");
      break;
    }
    case 4: { // Slide 5: Status Mode
      lcd.setCursor(0, 0);
      lcd.print("Mode Sistem:");
      lcd.setCursor(0, 1);
      if (isAutoMode) {
        lcd.print("--> OTOMATIS <--");
      } else {
        lcd.print("--> MANUAL   <--");
      }
      break;
    }
  }
  lcdSlideState = (lcdSlideState + 1) % 5; 
}


// --- Fungsi ini hanya baca sensor & kirim data ---
void kirimDataSensor() {
  // 1. Baca Sensor (DHT dan Tanah)
  suhu = dht.readTemperature(); 
  kelembapan = dht.readHumidity(); 
  if (isnan(suhu) || isnan(kelembapan)) { 
    Serial.println("Gagal membaca dari sensor DHT!");
  }

  lembapTanah = analogRead(SOIL_PIN);
  lembapTanahPersen = map(lembapTanah, KERING, BASAH, 0, 100);
  lembapTanahPersen = constrain(lembapTanahPersen, 0, 100);

  // 2. Jalankan Logika Auto
  if (isAutoMode) {
    if (lembapTanahPersen < AMBANG_KERING_AUTO) {
      if (isPompaOn == false) { 
          Serial.println("Tanah kering! Menyalakan pompa (Auto)...");
          digitalWrite(RELAY_PIN, RELAY_ON);
          isPompaOn = true;
          Blynk.virtualWrite(V3, 1); 
      }
    } 
    else if (lembapTanahPersen > AMBANG_BASAH_AUTO) {
      if (isPompaOn == true) { 
          Serial.println("Tanah sudah basah! Mematikan pompa (Auto)...");
          digitalWrite(RELAY_PIN, RELAY_OFF);
          isPompaOn = false;
          Blynk.virtualWrite(V3, 0); 
      }
    }
  }

  // 3. Kirim Data Sensor ke Blynk
  Blynk.virtualWrite(V0, suhu); 
  Blynk.virtualWrite(V1, kelembapan); 
  Blynk.virtualWrite(V2, lembapTanahPersen); 

  // 4. Cetak ke Serial Monitor
  Serial.print("Mode: "); Serial.print(isAutoMode ? "Auto" : "Manual");
  Serial.print(", Suhu: "); Serial.print(suhu);
  Serial.print("°C, Kelembapan: "); Serial.print(kelembapan);
  Serial.print("%, Tanah: "); Serial.print(lembapTanahPersen);
  Serial.print("%, Pompa: "); Serial.println(isPompaOn ? "ON" : "OFF");
}

// ---------------------------------
// SETUP (TIDAK BERUBAH)
// ---------------------------------
void setup() {
  Serial.begin(115200);
  delay(100);

  // Inisialisasi I2C dan LCD
  Wire.begin(21, 22); // SDA, SCL
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Smart Farming");
  lcd.setCursor(0, 1);
  lcd.print("Connecting WiFi...");

  // Inisialisasi sensor dan pin
  dht.begin(); 
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, RELAY_OFF); 

  // Mulai koneksi Blynk
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);

  // --- INISIALISASI JAM (NTP) ---
  timeClient.begin();
  // -----------------------------

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Terkoneksi!");
  lcd.setCursor(0, 1);
  lcd.print("Mulai Sistem...");
  delay(2000); 

  // --- PENGATURAN TIMER BARU ---
  timer.setInterval(5000L, kirimDataSensor);
  timer.setInterval(4000L, updateLCD); 
}

// ---------------------------------
// LOOP UTAMA
// ---------------------------------
void loop() {
  Blynk.run();
  timer.run(); 
}