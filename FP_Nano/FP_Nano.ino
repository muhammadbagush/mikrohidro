#include <Wire.h>                                                         //libraries untuk pengaksesan i2c
#include <Adafruit_BME280.h>                                              //libraries yang baru saja diinstall seperti cara diatas
#include <Adafruit_Sensor.h>                                              //Driver sensor untuk dapat berjalan dikomputer
 
#define SEALEVELPRESSURE_HPA (1013.25)                                    //Nilai awal untuk pressure
#define sensor 3                                                          //Mendeklarasikan pin IR Sensor pada pin 3 arduino
//#define DEBUG                                                             //Mendeklarasikan variabel DEBUG
#define RELEASE                                                         //Mendeklarasikan variabel RELEASE
Adafruit_BME280 bme;                                                      //penggunaan I2C

uint64_t sekarang = 0;                                                    //Variabel untuk waktu akhir
uint64_t waktu = 0;                                                       //Variabel untuk waktu akhir
uint64_t oldtime = 0;                                                     //Variabel untuk waktu akhir
uint64_t timeDelay = 0;
uint64_t periode = 0;                                                     //Variabel selisih waktu
uint64_t waktuBme = 0;
bool pulsa_baru = false;                                                  //Variabel untuk kondisi dengan default false
bool bmeok = true;                                                        //Variabel kondisi sensor bme
float rpm = 0;                                                            //Deklarasi variabel RPM
float rpm_send = 0;
float temp = 0;                                                           //Deklarasi vaiabel suhu 
float hum = 0;                                                            //Deklarasis variabel kelembapan
float bar = 0;                                                            //Deklarasi variabel tekanan
float altd = 0;                                                           //Deklarasi variabel ketinggian
uint64_t rpm2 = 0;                                                        //Variabel untuk menyimpan besar selisih waktu untuk membuat rpm bernilai 0 ketika putaran berhenti
float rpm_kalibrasi = 0;
void setup() 
{
  Serial.begin(9600);                                                     //Syntax untuk serial print monitor
  attachInterrupt(digitalPinToInterrupt(sensor), pulsa, CHANGE);          /*Setting interrupt. "sensor" sebagai input/trigger, "pulsa" sebagai fungsi ISR yang akan dieksekusi, 
                                                                          * "CHANGE" sebagai mode dari interrupt yang artinya 
                                                                          * untuk mentrigger ISR ketika kondisi dari LOW-HIGH dan HIGH-LOW 
                                                                          * 
                                                                          */    
  pinMode(sensor, INPUT_PULLUP);                                          //Setting untuk menjadikan pin 3 sensor sebagai input pullup  
  timeDelay = millis();
  waktu = millis();                                                       //Variabel waktu untuk menampung waktu awal
  waktuBme = millis();                                                    //Variabel waktu untuk menampung waktu awal pada sensor BME
}

void loop() 
{
  rpm_kalibrasi = rpm * 3.9;
  if (!bme.begin(0x76))                                                   //Pengkondisian untuk sensor BME280 jika address 0x76 tidak mendeteksi maka akan menjalankan program didalamnya 
  {                   
      if(millis() - waktuBme>=10000)                                      //Menampilkan program bme bernilai 0 dalam waktu 10s saat sensor bme tidak aktif
      {
      waktuBme = millis();                                                //Menset waktu awal pada sensor BME 
      bmeok = false;
      Serial.print("rpm : ");Serial.print(rpm);                               //Menampilkan nilai rpm
      Serial.print("temp : ");Serial.print(0);                             //Menampilkan nilai suhu
      Serial.print("hum : ");Serial.print(0);                               //Menampilkan nilai kelembapan      
      Serial.print("bar : ");Serial.print(0);                               //Menampilkan nilai tekanan udara
      Serial.print("altd : ");Serial.print(0);                             //Menampilkan nilai ketinggian
      Serial.print("\n");
      }
      //}
  }else{
    bmeok = true;
  }
   
//rpm
  if(pulsa_baru == true )                                                  //Decision untuk kondisi variabel pulsa_baru = TRUE. Jika True maka program yang ada didalam fungsi akan berjalan
  {
    periode = sekarang-waktu;                                              //Variabel untuk menghitung waktu selisih
    rpm = 60000/periode;                                                   //Variabel untuk mengkalkulasi waktu rpm.
    waktu = sekarang;                                                      //Variabel untuk menyimpan waktu terakhir dari variabel sekarang yang terus berjalan
    pulsa_baru = false;                                                    //Variabel untuk mengembalikkan nilai variabel pulsa_baru ke default
  }
  
  rpm2 = millis() - waktu;                                                 //Kalkulasi untuk menghitung selisih waktu tiap pulse atau periodenya
  
  if (rpm2 >=15000)                                                        /*Decision saat nilai selisih waktu atau rpm2 >= 600000, maka akan menjalankan program yaitu 
                                                                            *nilai rpm akan dikembalikan ke 0 rpm.
                                                                           */
  {
    rpm = 0;                                                                //Pengembalian nilai rpm ke nilai default yaitu 0    
  }
  if( millis() - oldtime >=10000 && bmeok == true )                                          //menset waktu untuk setiap 10s menampilkan data
  { 
    oldtime = millis();                                                     //waktu set
    temp = bme.readTemperature();                                           //Variabel untuk membaca nilai Suhu
    hum = bme.readHumidity();                                               //Variabel untuk membaca nilai Kelembaban 
    bar = bme.readPressure() / 100.0F;                                      //Variabel untuk membaca nilai Tekanan
    altd = bme.readAltitude(SEALEVELPRESSURE_HPA);                          //Variabel untuk membaca nilai Ketinggian
  
    #ifdef DEBUG                                                            //Pengkondisian ketika mode DEBUG aktif akan menjalankan program yang ada didalamnya
    Serial.print("rpm : ");Serial.print(rpm_kalibrasi);                               //Menampilkan nilai rpm
    Serial.print("temp : ");Serial.print(temp);                             //Menampilkan nilai suhu
    Serial.print("hum : ");Serial.print(hum);                               //Menampilkan nilai kelembapan      
    Serial.print("bar : ");Serial.print(bar);                               //Menampilkan nilai tekanan udara
    Serial.print("altd : ");Serial.print(altd);                             //Menampilkan nilai ketinggian
    Serial.print("\n");
    #endif                                                                  //Batas program yang dijalankan pada pengkondisian DEBUG
    
    #ifdef RELEASE                                                          //Pengkondisian ketika mode RELEASE aktif akan menjalankan program yang ada didalamnya
    Serial.print("%");                                                      //Identitas untuk parsing data dari arduino nano
    Serial.print(rpm_kalibrasi);Serial.print(";");                                    //Menampilkan data rpm
    Serial.print(temp);Serial.print(";");                                   //Menampilkan data suhu
    Serial.print(hum);Serial.print(";");                                    //Menampilkan data kelembapan
    Serial.print(bar);Serial.print(";");                                    //Menampilkan data tekanan udara
    Serial.print(altd);Serial.print(";");                                   //Menampilkan data ketinggian
    Serial.print("\n");
    #endif                                                                  //Batas program yang dijalankan pada pengkondisian RELEASE  
  } 
  delay(50);                                                                //Memberikan delay pada serial monitor agar data rpm dan bme tidak bertabrakan
}

void pulsa()                                                                //Fungsi ISR, fungsi ini akan berjalan ketika interrupt diaktifkan yang mendapat trigger dari sensor
{
  if(digitalRead(sensor) == HIGH )                                            //Pengkondisian ketika pin sensor HIGH/tidak mendeteksi reflektor maka akan menjalankan program didalam fungsi
    {
    sekarang = millis();                                                    //Variabel untuk menyimpan waktu sekarang
    pulsa_baru = true;                                                      //Variabel untuk status pulsa_baru kondisi true
    }
}
