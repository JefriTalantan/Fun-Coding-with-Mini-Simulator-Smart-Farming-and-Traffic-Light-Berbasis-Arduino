/*
 * KODE SMART FARMING - MONITORING ONLINE (BLYNK V12)
 */

// ----------------------------------------------------
// ## 1. KREDENSIAL BLYNK & WIFI (ISI DISINI!) ##
// ----------------------------------------------------
#define BLYNK_TEMPLATE_ID "Template_ID Anda" // GANTI Template_ID ANDA
#define BLYNK_TEMPLATE_NAME "Template_NAME Anda" // GANTI TOKEN ANDA
#define BLYNK_AUTH_TOKEN "Auth_Token Anda" // GANTI TOKEN ANDA

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
// ## 3. PENGATURAN HARDWARE ##
// ----------------------------------------------------
#define SENSOR_DHT DHT22  // Ganti DHT11 jika perlu

// PIN
#define DHTPIN 13      // Pin Data DHT
#define SOIL_PIN 34    // Pin Analog Sensor Tanah

// KALIBRASI (Sesuai hasil tes Anda)
const int KERING = 3150; 
const int BASAH = 1200;  

// JAM (UTC+9 untuk WIT. Ganti 25200 untuk WIB)
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

// Timer
unsigned long prevSensorMillis = 0;  
unsigned long prevLCDMillis = 0;     
int currentSlide = 0; 
String days[] = {"Minggu", "Senin", "Selasa", "Rabu", "Kamis", "Jumat", "Sabtu"};

void setup() {
  Serial.begin(115200);
  
  // Init LCD
  Wire.begin(21, 22); 
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Smart Farm V12");
  lcd.setCursor(0, 1);
  lcd.print("Connecting...");

  // Init Sensor
  dht.begin();

  // Init Blynk
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
  
  // Init Jam
  timeClient.begin();
  
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Online Mode!");
  delay(2000);
}

void loop() {
  Blynk.run();         // Wajib untuk Blynk
  timeClient.update(); // Wajib untuk Jam

  unsigned long currentMillis = millis();

  // -------------------------------------------------
  // TUGAS 1: BACA SENSOR & KIRIM DATA (Setiap 2 Detik)
  // -------------------------------------------------
  if (currentMillis - prevSensorMillis >= 2000) {
    prevSensorMillis = currentMillis;
    bacaDanKirimSensor();
  }

  // -------------------------------------------------
  // TUGAS 2: UPDATE LCD (Setiap 3 Detik)
  // -------------------------------------------------
  if (currentMillis - prevLCDMillis >= 3000) {
    prevLCDMillis = currentMillis;
    updateLCDSlides();
  }
}

void bacaDanKirimSensor() {
  // 1. BACA TANAH
  lembapTanah = analogRead(SOIL_PIN);
  lembapTanahPersen = map(lembapTanah, KERING, BASAH, 0, 100);
  lembapTanahPersen = constrain(lembapTanahPersen, 0, 100);

  // 2. BACA UDARA
  float h = dht.readHumidity();
  float t = dht.readTemperature();

  if (isnan(h) || isnan(t)) {
    Serial.println("⚠️ Gagal baca DHT");
  } else {
    suhu = t;
    kelembapan = h;
  }

  // 3. KIRIM KE BLYNK
  Blynk.virtualWrite(V0, suhu);           // V0: Suhu
  Blynk.virtualWrite(V1, kelembapan);     // V1: Kelembapan
  Blynk.virtualWrite(V2, lembapTanahPersen); // V2: Tanah

  // 4. SERIAL MONITOR
  Serial.print("Tanah: "); Serial.print(lembapTanahPersen); Serial.print("% | ");
  Serial.print("Suhu: "); Serial.print(suhu); Serial.print(" C | ");
  Serial.print("Lembap: "); Serial.print(kelembapan); Serial.println("%");
}

void updateLCDSlides() {
  if (currentSlide == 0) { lcd.init(); lcd.backlight(); }
  lcd.clear();

  switch (currentSlide) {
    
    // SLIDE 1: JAM & HARI (Internet)
    case 0: {
      lcd.setCursor(0, 0); lcd.print(days[timeClient.getDay()]);
      lcd.setCursor(0, 1); lcd.print(timeClient.getFormattedTime());
      break;
    }

    // SLIDE 2: ANGKA SUHU & LEMBAP
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