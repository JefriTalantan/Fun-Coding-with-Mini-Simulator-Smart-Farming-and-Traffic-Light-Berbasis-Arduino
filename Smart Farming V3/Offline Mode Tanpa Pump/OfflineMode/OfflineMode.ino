/*
 * KODE SMART FARMING - MONITORING ONLY (OFFLINE)
 */

#include <DHT.h> 
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// ==========================================
// 1. PENGATURAN SENSOR
// ==========================================
#define SENSOR_DHT DHT22  // Ganti DHT11 jika menggunakan DHT11

// PIN HARDWARE
#define DHTPIN 13      // Pin Data DHT
#define SOIL_PIN 34    // Pin Analog Sensor Tanah

// ==========================================
// 2. VARIABEL GLOBAL
// ==========================================
DHT dht(DHTPIN, SENSOR_DHT);
LiquidCrystal_I2C lcd(0x27, 16, 2); // Ganti 0x3F jika layar gelap

float suhu = 0; 
float kelembapan = 0; 
int lembapTanah = 0;
int lembapTanahPersen = 0;

// Timer
unsigned long prevSensorMillis = 0;  
unsigned long prevLCDMillis = 0;     
int currentSlide = 0;                

// ----------------------------------------------------
// ANGKA KALIBRASI (SESUAIKAN DENGAN HASIL TES ANDA)
// ----------------------------------------------------
const int KERING = 3150; 
const int BASAH = 1200;  


void setup() {
  Serial.begin(115200);
  
  // Init LCD
  Wire.begin(21, 22); 
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Smart Farming");
  lcd.setCursor(0, 1);
  lcd.print("Monitoring Mode");

  // Init DHT
  dht.begin();
  
  delay(2000); 
  lcd.clear();
}

void loop() {
  unsigned long currentMillis = millis();

  // -------------------------------------------------
  // TUGAS 1: BACA SEMUA SENSOR (Setiap 2 Detik)
  // -------------------------------------------------
  if (currentMillis - prevSensorMillis >= 2000) {
    prevSensorMillis = currentMillis;
    bacaSemuaSensor();
  }

  // -------------------------------------------------
  // TUGAS 2: UPDATE LCD (Setiap 3 Detik)
  // -------------------------------------------------
  if (currentMillis - prevLCDMillis >= 3000) {
    prevLCDMillis = currentMillis;
    updateLCDSlides();
  }
}

void bacaSemuaSensor() {
  // 1. BACA TANAH
  lembapTanah = analogRead(SOIL_PIN);
  lembapTanahPersen = map(lembapTanah, KERING, BASAH, 0, 100);
  lembapTanahPersen = constrain(lembapTanahPersen, 0, 100);

  // 2. BACA UDARA (DHT)
  float h = dht.readHumidity();
  float t = dht.readTemperature();

  if (isnan(h) || isnan(t)) {
    Serial.println("⚠️ Gagal baca DHT (Cek Kabel)");
  } else {
    suhu = t;
    kelembapan = h;
  }

  // 3. LAPORAN SERIAL
  Serial.print("Tanah: "); Serial.print(lembapTanahPersen); Serial.print("% | ");
  Serial.print("Suhu: "); Serial.print(suhu); Serial.print(" C | ");
  Serial.print("Lembap: "); Serial.print(kelembapan); Serial.println("%");
}

void updateLCDSlides() {
  // Auto-Repair LCD
  if (currentSlide == 0) { lcd.init(); lcd.backlight(); }
  lcd.clear();

  switch (currentSlide) {
    
    // SLIDE 1: WAKTU NYALA (UPTIME)
    case 0: {
      unsigned long now = millis() / 1000; 
      int jam = now / 3600;
      int menit = (now % 3600) / 60;
      int detik = now % 60;
      
      lcd.setCursor(0, 0); lcd.print("Durasi Nyala:");
      lcd.setCursor(0, 1); 
      if(jam < 10) lcd.print("0"); lcd.print(jam); lcd.print(":");
      if(menit < 10) lcd.print("0"); lcd.print(menit); lcd.print(":");
      if(detik < 10) lcd.print("0"); lcd.print(detik);
      break;
    }

    // SLIDE 2: DATA ANGKA (SUHU & LEMBAP)
    case 1: {
      lcd.setCursor(0, 0); 
      lcd.print("Suhu  : "); lcd.print((int)suhu); lcd.print((char)223); lcd.print("C");
      lcd.setCursor(0, 1); 
      lcd.print("Lembap: "); lcd.print((int)kelembapan); lcd.print("%");
      break;
    }

    // SLIDE 3: STATUS SUHU
    case 2: {
      lcd.setCursor(0, 0); lcd.print("Status Suhu:");
      lcd.setCursor(0, 1); 
      if(suhu == 0) lcd.print("Menunggu..."); 
      else if(suhu > 30) lcd.print(">> PANAS <<");
      else if(suhu < 20) lcd.print(">> DINGIN <<");
      else lcd.print(">> NORMAL <<");
      break;
    }

    // SLIDE 4: STATUS UDARA
    case 3: {
      lcd.setCursor(0, 0); lcd.print("Kualitas Udara:");
      lcd.setCursor(0, 1); 
      if(kelembapan == 0) lcd.print("Menunggu...");
      else if(kelembapan >= 40 && kelembapan <= 70) lcd.print("* SEHAT *");
      else lcd.print("! TIDAK SEHAT !");
      break;
    }

    // SLIDE 5: TANAH
    case 4: {
      lcd.setCursor(0, 0); lcd.print("Tanah: "); lcd.print(lembapTanahPersen); lcd.print("%");
      lcd.setCursor(0, 1);
      if (lembapTanahPersen < 40) lcd.print("[KERING]");
      else if (lembapTanahPersen > 75) lcd.print("[BASAH]");
      else lcd.print("[LEMBAB]"); 
      break;
    }
  }

  currentSlide++;
  if (currentSlide > 4) currentSlide = 0;
}