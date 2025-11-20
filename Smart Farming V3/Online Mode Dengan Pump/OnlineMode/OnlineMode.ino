/*
 * KODE SMART FARMING - ONLINE V10
 */

// ----------------------------------------------------
// ## 1. KREDENSIAL BLYNK & WIFI (ISI DISINI!) ##
// ----------------------------------------------------
#define BLYNK_TEMPLATE_ID "TEMPLATE_ID Anda"       // GANTI DENGAN PUNYA ANDA
#define BLYNK_TEMPLATE_NAME "TEMPLATE_NAME Anda" 
#define BLYNK_AUTH_TOKEN "AUTH_TOKEN Anda" // GANTI TOKEN ANDA

char ssid[] = "Nama Wifi Anda";       // Nama WiFi
char pass[] = "Password Wifi Anda";     // Password WiFi

// ----------------------------------------------------
// ## 2. LIBRARY ##
// ----------------------------------------------------
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

// ----------------------------------------------------
// ## 3. PENGATURAN HARDWARE & LOGIKA ##
// ----------------------------------------------------
#define SENSOR_DHT DHT22 

// PIN
#define DHT_POWER_PIN 27 // Sumber listrik DHT (Untuk Hard Reset)
#define DHTPIN 13        // Data DHT
#define SOIL_PIN 34      // Sensor Tanah
#define RELAY_PIN 5      // Relay

// Logika Relay
#define RELAY_ON HIGH
#define RELAY_OFF LOW

// Batas Tanah (Histeresis)
#define BATAS_NYALA 40    // Kering (< 40%)
#define BATAS_MATI 75     // Basah (> 75%)

// Kalibrasi (Pakai angka Anda yang terakhir)
const int KERING = 3150; 
const int BASAH = 1200;  

// Pengaturan Jam (Waktu Indonesia Timur / WIT = UTC+9)
// WIB = 25200, WITA = 28800, WIT = 32400
const long UTC_OFFSET = 32400; 

// ----------------------------------------------------
// ## 4. OBJEK & VARIABEL ##
// ----------------------------------------------------
DHT dht(DHTPIN, SENSOR_DHT);
LiquidCrystal_I2C lcd(0x27, 16, 2); 
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", UTC_OFFSET);

float suhu = 0; 
float kelembapan = 0; 
int lembapTanah = 0;
int lembapTanahPersen = 0;
bool isPompaOn = false;
bool isAutoMode = true; // DEFAULT AUTO

// Timer
unsigned long prevTanahMillis = 0;   
unsigned long prevDHTMillis = 0;     
unsigned long prevLCDMillis = 0;
unsigned long waktuPompaBerubah = 0; 
int currentSlide = 0; 
int errorCount = 0;
String days[] = {"Minggu", "Senin", "Selasa", "Rabu", "Kamis", "Jumat", "Sabtu"};

// ==========================================
// ## 5. FUNGSI BLYNK ##
// ==========================================

// Sinkronisasi saat terkoneksi
BLYNK_CONNECTED() {
  Serial.println(">> Terkoneksi ke Blynk! Sync Status...");
  Blynk.virtualWrite(V4, isAutoMode ? 1 : 0); // Update tombol Mode
  Blynk.virtualWrite(V3, isPompaOn ? 1 : 0);  // Update tombol Pompa
}

// V4: Saklar Mode Auto/Manual
BLYNK_WRITE(V4) {
  int val = param.asInt();
  if (val == 1) {
    isAutoMode = true;
    Serial.println("Mode: AUTO DI-AKTIFKAN");
  } else {
    isAutoMode = false;
    Serial.println("Mode: MANUAL DI-AKTIFKAN");
    // Saat pindah ke manual, matikan pompa demi keamanan
    digitalWrite(RELAY_PIN, RELAY_OFF);
    isPompaOn = false;
    Blynk.virtualWrite(V3, 0);
  }
}

// V3: Tombol Pompa Manual
BLYNK_WRITE(V3) {
  if (isAutoMode) {
    // Jika Mode Auto, tolak perintah manual
    Serial.println("⚠️ Ditolak: Sedang Mode Auto!");
    Blynk.virtualWrite(V3, isPompaOn ? 1 : 0); // Kembalikan posisi tombol
    return;
  }

  int val = param.asInt();
  if (val == 1) {
    digitalWrite(RELAY_PIN, RELAY_ON);
    isPompaOn = true;
    waktuPompaBerubah = millis();
    Serial.println("Pompa ON (Manual)");
  } else {
    digitalWrite(RELAY_PIN, RELAY_OFF);
    isPompaOn = false;
    waktuPompaBerubah = millis();
    Serial.println("Pompa OFF (Manual)");
  }
}

// ==========================================
// ## 6. SETUP & LOOP ##
// ==========================================
void setup() {
  Serial.begin(115200);
  
  // Init LCD
  Wire.begin(21, 22); 
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Smart Farm V10");
  lcd.setCursor(0, 1);
  lcd.print("Connecting...");

  // Init Relay
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, RELAY_OFF); 

  // Init Power DHT (Hard Reset Setup)
  pinMode(DHT_POWER_PIN, OUTPUT);
  digitalWrite(DHT_POWER_PIN, LOW); 
  delay(500);
  digitalWrite(DHT_POWER_PIN, HIGH); 
  
  // Init DHT
  dht.begin();

  // Init Blynk & WiFi
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
  
  // Init Jam
  timeClient.begin();
  
  lcd.clear();
  Serial.println("Sistem Online & Siap!");
}

void loop() {
  Blynk.run();
  timeClient.update(); // Update jam

  unsigned long currentMillis = millis();

  // JALUR 1: CEK TANAH & LOGIKA (0.5 Detik)
  if (currentMillis - prevTanahMillis >= 500) {
    prevTanahMillis = currentMillis;
    cekTanahDanPompa();
  }

  // JALUR 2: CEK DHT (4 Detik)
  if (currentMillis - prevDHTMillis >= 4000) {
    prevDHTMillis = currentMillis;
    cekUdara();
  }

  // JALUR 3: LCD (3 Detik)
  if (currentMillis - prevLCDMillis >= 3000) {
    prevLCDMillis = currentMillis;
    updateLCDSlides();
  }
}

// ==========================================
// ## 7. FUNGSI LOGIKA UTAMA ##
// ==========================================
void cekTanahDanPompa() {
  lembapTanah = analogRead(SOIL_PIN);
  lembapTanahPersen = map(lembapTanah, KERING, BASAH, 0, 100);
  lembapTanahPersen = constrain(lembapTanahPersen, 0, 100);

  // Kirim data ke Blynk (V2)
  Blynk.virtualWrite(V2, lembapTanahPersen);

  // --- LOGIKA OTOMATIS ---
  // Hanya jalan jika isAutoMode = TRUE
  if (isAutoMode) {
    // Pengaman waktu
    if (millis() - waktuPompaBerubah < 1000) return; 

    if (lembapTanahPersen < BATAS_NYALA) {
      if (!isPompaOn) {
        Serial.println(">>> AUTO: TANAH KERING -> POMPA ON <<<");
        digitalWrite(RELAY_PIN, RELAY_ON);
        isPompaOn = true;
        waktuPompaBerubah = millis();
        Blynk.virtualWrite(V3, 1); // Update tombol di HP
      }
    } 
    else if (lembapTanahPersen > BATAS_MATI) {
      if (isPompaOn) {
        Serial.println(">>> AUTO: TANAH BASAH -> POMPA OFF <<<");
        digitalWrite(RELAY_PIN, RELAY_OFF);
        isPompaOn = false;
        waktuPompaBerubah = millis();
        Blynk.virtualWrite(V3, 0); // Update tombol di HP
      }
    }
  }
}

void cekUdara() {
  // JANGAN BACA SAAT POMPA NYALA (Anti-Noise)
  if (isPompaOn == true && suhu != 0) return; 

  float h = dht.readHumidity();
  float t = dht.readTemperature();

  if (isnan(h) || isnan(t)) {
    errorCount++;
    Serial.print("⚠️ Gagal DHT. Retry: "); Serial.println(errorCount);
    
    if (errorCount >= 3) {
      Serial.println("⚡ HARD RESET SENSOR DHT...");
      digitalWrite(DHT_POWER_PIN, LOW);
      delay(500);
      digitalWrite(DHT_POWER_PIN, HIGH);
      delay(1000);
      dht.begin();
      errorCount = 0;
    }
  } else {
    suhu = t;
    kelembapan = h;
    errorCount = 0;
    
    // Kirim ke Blynk
    Blynk.virtualWrite(V0, suhu);
    Blynk.virtualWrite(V1, kelembapan);
  }
}

void updateLCDSlides() {
  // AUTO-REPAIR LCD (Reset layar jika kembali ke awal)
  if (currentSlide == 0) { lcd.init(); lcd.backlight(); }
  lcd.clear();

  switch (currentSlide) {
    
    // SLIDE 1: JAM & HARI (NTP)
    case 0: {
      lcd.setCursor(0, 0); lcd.print(days[timeClient.getDay()]);
      lcd.setCursor(0, 1); lcd.print(timeClient.getFormattedTime());
      break;
    }

    // SLIDE 2: ANGKA SUHU & LEMBAP (INI YANG BARU)
    case 1: {
      lcd.setCursor(0, 0); 
      // Tampilkan Suhu: "Suhu  : 30°C"
      lcd.print("Suhu  : "); 
      if(suhu == 0) lcd.print("--"); 
      else lcd.print((int)suhu); 
      lcd.print((char)223); lcd.print("C");

      lcd.setCursor(0, 1); 
      // Tampilkan Lembap: "Lembap: 80%"
      lcd.print("Lembap: "); 
      if(kelembapan == 0) lcd.print("--"); 
      else lcd.print((int)kelembapan); 
      lcd.print("%");
      break;
    }

    // SLIDE 3: STATUS SUHU (Bergeser jadi Case 2)
    case 2: {
      lcd.setCursor(0, 0); lcd.print("Status Suhu:");
      lcd.setCursor(0, 1); 
      if(suhu == 0) lcd.print("Menunggu..."); 
      else if(suhu > 30) lcd.print(">> PANAS <<");
      else if(suhu < 20) lcd.print(">> DINGIN <<");
      else lcd.print(">> NORMAL <<");
      break;
    }

    // SLIDE 4: STATUS UDARA (Bergeser jadi Case 3)
    case 3: {
      lcd.setCursor(0, 0); lcd.print("Kualitas Udara:");
      lcd.setCursor(0, 1); 
      if(kelembapan == 0) lcd.print("Menunggu...");
      else if(kelembapan >= 40 && kelembapan <= 70) lcd.print("* SEHAT *");
      else lcd.print("! TIDAK SEHAT !");
      break;
    }

    // SLIDE 5: TANAH (Bergeser jadi Case 4)
    case 4: {
      lcd.setCursor(0, 0); lcd.print("Tanah: "); lcd.print(lembapTanahPersen); lcd.print("%");
      lcd.setCursor(0, 1);
      if (lembapTanahPersen < BATAS_NYALA) lcd.print("[KERING]");
      else if (lembapTanahPersen > BATAS_MATI) lcd.print("[BASAH]");
      else lcd.print("[CUKUP]"); 
      break;
    }

    // SLIDE 6: MODE & POMPA (Bergeser jadi Case 5)
    case 5: {
      lcd.setCursor(0, 0); 
      if(isAutoMode) lcd.print("Mode: AUTOMATIS"); else lcd.print("Mode: MANUAL");
      
      lcd.setCursor(0, 1); lcd.print("Pompa: "); 
      if(isPompaOn) lcd.print("ON"); else lcd.print("OFF");
      break;
    }
  }

  // Update Loop Counter (Sekarang sampai 5)
  currentSlide++;
  if (currentSlide > 5) currentSlide = 0;
}