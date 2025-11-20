/*
 * KODE SMART FARMING - V9
 */

#include <DHT.h> 
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// ==========================================
// 1. PENGATURAN SENSOR & HARDWARE
// ==========================================
#define SENSOR_DHT DHT22 

// PIN HARDWARE
#define DHT_POWER_PIN 27 // PIN Sumber Listrik DHT
#define DHTPIN 13        // Pin Data DHT
#define SOIL_PIN 34      // Pin Analog Sensor Tanah
#define RELAY_PIN 5      // Pin Relay

// Logika Relay
#define RELAY_ON HIGH
#define RELAY_OFF LOW

// Ambang Batas Otomatis
#define BATAS_NYALA 40    
#define BATAS_MATI 75     

// Kalibrasi (GANTI DENGAN ANGKA ANDA)
const int KERING = 3150; 
const int BASAH = 1200;  

// ==========================================
// 2. VARIABEL GLOBAL
// ==========================================
DHT dht(DHTPIN, SENSOR_DHT);
LiquidCrystal_I2C lcd(0x27, 16, 2); 

float suhu = 0; 
float kelembapan = 0; 
int lembapTanah = 0;
int lembapTanahPersen = 0;
bool isPompaOn = false;

// Timer & Pengaman
unsigned long prevTanahMillis = 0;   
unsigned long prevDHTMillis = 0;     
unsigned long prevLCDMillis = 0;
unsigned long prevReportMillis = 0;
unsigned long waktuPompaBerubah = 0; 
int currentSlide = 0; 
int errorCount = 0; // Menghitung berapa kali error berturut-turut

void setup() {
  Serial.begin(115200);
  
  // 1. Init Layar
  Wire.begin(21, 22); 
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Smart Farm V9");
  lcd.setCursor(0, 1);
  lcd.print("Powering DHT...");

  // 2. Init Relay
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, RELAY_OFF); 

  // 3. NYALAKAN SENSOR DHT (HARD RESET DI AWAL)
  pinMode(DHT_POWER_PIN, OUTPUT);
  digitalWrite(DHT_POWER_PIN, LOW); // Matikan dulu
  delay(500);
  digitalWrite(DHT_POWER_PIN, HIGH); // Nyalakan (Memberi daya 3.3V)
  
  // 4. Init DHT
  dht.begin();
  delay(2000); // Tunggu sensor bangun
  
  lcd.clear();
  Serial.println("Sistem Siap. DHT Powered by GPIO 27");
}

void loop() {
  unsigned long currentMillis = millis();

  // TUGAS 1: CEK TANAH & POMPA (0.5 Detik)
  if (currentMillis - prevTanahMillis >= 500) {
    prevTanahMillis = currentMillis;
    cekTanahDanPompa();
  }

  // TUGAS 2: CEK DHT (4 Detik)
  if (currentMillis - prevDHTMillis >= 4000) {
    prevDHTMillis = currentMillis;
    cekUdara();
  }

  // TUGAS 3: UPDATE LCD (3 Detik)
  if (currentMillis - prevLCDMillis >= 3000) {
    prevLCDMillis = currentMillis;
    updateLCDSlides();
  }

  // TUGAS 4: LAPORAN SERIAL (2 Detik)
  if (currentMillis - prevReportMillis >= 2000) {
    prevReportMillis = currentMillis;
    Serial.print("[STATUS] Tanah: "); Serial.print(lembapTanahPersen); Serial.print("%");
    Serial.print(" | Pompa: "); Serial.print(isPompaOn ? "ON" : "OFF");
    Serial.print(" | Suhu: "); Serial.print(suhu); 
    Serial.print(" | Lembap: "); Serial.println(kelembapan);
  }
}

void cekTanahDanPompa() {
  lembapTanah = analogRead(SOIL_PIN);
  lembapTanahPersen = map(lembapTanah, KERING, BASAH, 0, 100);
  lembapTanahPersen = constrain(lembapTanahPersen, 0, 100);

  if (millis() - waktuPompaBerubah < 1000) return; 

  if (lembapTanahPersen < BATAS_NYALA) {
    if (!isPompaOn) {
      Serial.println(">>> TANAH KERING -> POMPA ON <<<");
      digitalWrite(RELAY_PIN, RELAY_ON);
      isPompaOn = true;
      waktuPompaBerubah = millis();
    }
  } 
  else if (lembapTanahPersen > BATAS_MATI) {
    if (isPompaOn) {
      Serial.println(">>> TANAH BASAH -> POMPA OFF <<<");
      digitalWrite(RELAY_PIN, RELAY_OFF);
      isPompaOn = false;
      waktuPompaBerubah = millis();
    }
  }
}

void cekUdara() {
  // Jangan baca saat pompa nyala (Noise tinggi)
  if (isPompaOn == true && suhu != 0) return; 

  float h = dht.readHumidity();
  float t = dht.readTemperature();

  if (isnan(h) || isnan(t)) {
    errorCount++;
    Serial.print("⚠️ Gagal baca DHT. Error ke-"); Serial.println(errorCount);

    // JIKA SUDAH 3 KALI ERROR BERTURUT-TURUT -> LAKUKAN HARD RESET
    if (errorCount >= 3) {
      Serial.println("⚡ MEMATIKAN PAKSA SENSOR DHT (HARD RESET)...");
      
      digitalWrite(DHT_POWER_PIN, LOW); // Cabut nyawa sensor
      delay(500);                       // Tunggu mati total
      digitalWrite(DHT_POWER_PIN, HIGH); // Hidupkan lagi
      delay(1000);                      // Tunggu booting
      dht.begin();                      // Mulai komunikasi ulang
      
      errorCount = 0; // Reset penghitung error
      Serial.println("⚡ SENSOR HIDUP KEMBALI.");
    }
  } else {
    suhu = t;
    kelembapan = h;
    errorCount = 0; // Reset error jika berhasil
  }
}

void updateLCDSlides() {
  // AUTO-REPAIR LCD (Reset layar jika kembali ke awal)
  if (currentSlide == 0) { 
    lcd.init(); lcd.backlight(); 
  }
  lcd.clear();

  switch (currentSlide) {
    
    // --- SLIDE 1: DATA ANGKA (SUHU & LEMBAP) ---
    case 0: {
      lcd.setCursor(0, 0); 
      lcd.print("Suhu  : "); lcd.print((int)suhu); lcd.print((char)223); lcd.print("C");
      lcd.setCursor(0, 1); 
      lcd.print("Lembap: "); lcd.print((int)kelembapan); lcd.print("%");
      break;
    }

    // --- SLIDE 2: STATUS SUHU (TERPISAH) ---
    case 1: {
      lcd.setCursor(0, 0); lcd.print("Status Suhu:");
      lcd.setCursor(0, 1); 
      
      if (suhu == 0) {
        lcd.print("Menunggu...");
      } 
      else if (suhu > 30) {
        lcd.print(">> PANAS <<");
      } 
      else if (suhu < 20) {
        lcd.print(">> DINGIN <<");
      } 
      else {
        lcd.print(">> NORMAL <<");
      }
      break;
    }

    // --- SLIDE 3: STATUS KELEMBAPAN UDARA (TERPISAH) ---
    case 2: {
      lcd.setCursor(0, 0); lcd.print("Kualitas Udara:");
      lcd.setCursor(0, 1); 
      
      if (kelembapan == 0) {
        lcd.print("Menunggu...");
      }
      else if (kelembapan >= 40 && kelembapan <= 70) {
        lcd.print("* SEHAT *");
      } 
      else {
        lcd.print("! TIDAK SEHAT !");
      }
      break;
    }

    // --- SLIDE 4: KONDISI TANAH ---
    case 3: {
      lcd.setCursor(0, 0); lcd.print("Tanah: "); lcd.print(lembapTanahPersen); lcd.print("%");
      lcd.setCursor(0, 1);
      if (lembapTanahPersen < BATAS_NYALA) lcd.print("[KERING - SIRAM]");
      else if (lembapTanahPersen > BATAS_MATI) lcd.print("[BASAH]");
      else lcd.print("[CUKUP]"); 
      break;
    }

    // --- SLIDE 5: STATUS POMPA & WAKTU ---
    case 4: {
      lcd.setCursor(0, 0); lcd.print("Pompa: "); 
      if(isPompaOn) lcd.print("MENYALA"); else lcd.print("MATI");
      
      unsigned long m = millis() / 60000;
      lcd.setCursor(0, 1); lcd.print("Aktif: "); lcd.print(m); lcd.print(" Min");
      break;
    }
  }

  // Pindah ke slide berikutnya (Sekarang 0 sampai 4)
  currentSlide++;
  if (currentSlide > 4) currentSlide = 0;
}