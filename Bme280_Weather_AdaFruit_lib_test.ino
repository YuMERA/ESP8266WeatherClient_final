/*
|======================================|
|  Weather WiFi Client v1.1 By YuMERA  |
|  31-Dec-2020. 13:39                  | 
|  Sid                                 |
|======================================|
*/ 
  #include <Adafruit_Sensor.h>
  #include <Adafruit_BME280.h>
  #include <Wire.h>
  #include <DallasTemperature.h>        
  #include <ESP8266WiFi.h>
  #include <ESP8266HTTPClient.h>
  #include <WiFiClient.h>
    
  #define SEALEVELPRESSURE_HPA (1013.25) //Vazdusni pritisak na razini mora

  //---------------------
  // The ESP8266 RTC memory is arranged into blocks of 4 bytes. The access methods read and write 4 bytes at a time,
  // so the RTC data structure should be padded to a 4-byte multiple.
  struct {
    uint32_t crc32;   // 4 bytes
    uint8_t channel;  // 1 byte,   5 in total
    uint8_t ap_mac[6];
    uint8_t padding;  // 1 byte,  12 in total
  } rtcData;

  bool rtcValid = false;// variabla za proveru da li postoje snimljeni podaci u rtc memory

    /*
    * Measure Vbat
    * add 100k between Vbat and ADC
    * Voltage divider of 100k+220k over 100k
    * gives 100/420k
    * ergo 4.2V -> 1Volt
    * Max input on A0=1Volt ->1023
    * 4.2*(Raw/1023)=Vbat
    */
    // Voltage divider R1 = 220k+100k+220k =540k and R2=100k => 5.28
    float calibrate = 4.20; // this value for calibrate the battery voltage   
    unsigned int raw=0;
    //--------------------------------------------------------------------------
    //SSID and Password of your WiFi router
    //--------------------------------------------------------------------------
    const char* ssid =     "MeteoStanica-MERA";
    const char* password = "NeveNa91";
    const char* host =     "192.168.4.1";
    const int   httpPort = 3003;
    //const uint8_t bssid[] = {0xBE, 0xDD, 0xC2, 0x9C, 0xB2, 0x79}; //MAC from AP ESP8266
    const uint8_t bssid[] = {0xC4, 0x4F, 0x33, 0x7F, 0xEF, 0x61};   //MAC from AP ESP32
    
    IPAddress mystaticip(192, 168, 4, 2); //Ako bude vise klijenata vezanih na server onda treba menjati IP
    IPAddress mygateway(192, 168, 4, 1);
    IPAddress mysubnet(255, 255, 255, 0);
    
    #define ONE_WIRE_BUS D7               //Data wire prikljucena na digital pin 7 na ESP8266
    OneWire oneWire(ONE_WIRE_BUS);        //Podesavanje oneWire instance za komunikaciju sa nekim OneWire devices 
    DallasTemperature sensors(&oneWire);  //oneWire referenca za Dallas DS18B20 Temp. senzor
    
    volatile unsigned long sTime = 0;
    volatile float pulseTime = 0;         //stores time between one anemomter relay closing and the next
    const unsigned long timeout = 3000;   //timeout u milisekundama 
    float Tem=0.0;                        //Temperatura DS18B20
    float Hum=0.0;                        //Vlaznost vazduha
    float Pre=0.0;                        //Pritisak vazduha
    float Vcc=0.0;                        //Napon napajanja
    int Rsi=0;                            //Jacina WiFi Signala
    float TemBME=0.0;                     //Temperatura izmerena na senzoru pritiska
    float Win = 0.0;                      //Brzina Vetra
    float Slp = 0.0;                      //SeaLevelPreassure
    int sleepTimes ;                      //Time to sleep (in seconds):
    int httpLoop= 0;                      //Broj koji odredjuj broj petlji dok se ne posalju podaci
    #define INTERVAL_NRM  600             //normal wake up interval in seconds
    #define INTERVAL_LOW  900             //wake up interval when low battery
    #define INTERVAL_CRT 1200             //wake up intrrval when crtical battery
    #define LED_BLUE 2                    //plava led dioda na pinu 2 (TX) na nodeMcu
    #define SDA_PIN D2                    //data pin BME280
    #define SCL_PIN D1                    //clock pin BME280
    
    //Setup for battery voltage
    float BATT_NRM = 4.30;                //normal battery voltage    -> use INTERVAL_NRM
    float BATT_LOW = 3.80;                //low battery voltage       -> use INTERVAL_LOW
    float BATT_CRT = 3.50;                //critical battery voltage  -> use INTERVAL_CRT
    
    Adafruit_BME280 bme280;               //I2C
    uint8_t bme_address = 0x76;           //set to 0x76 or 0x77
    
// link sa kojeg su preuzeta resenja      
// weather monitoring https://gitlab.com/diy_bloke/verydeepsleep_bme280_sendtodb/blob/5cfaa397f4a345243831fb27357382abf2de3087/%20VeryDeepSleep_BME280_sendToDB.ino
// https://arduinodiy.wordpress.com/2020/01/21/very-deepsleep-and-energy-saving-on-esp8266-part-2-sending-data-with-http/
void BMEsetup() {
  Serial.println(" BME280 Scenario");
  Serial.println("--------------------------------------------------------------------------------");
  Serial.println("  forced mode, 1x temperature, 1x humidity, 1x pressure oversampling, filter off");
  Serial.println("--------------------------------------------------------------------------------");
  bme280.setSampling(Adafruit_BME280::MODE_FORCED,
                     Adafruit_BME280::SAMPLING_X1, // temperature
                     Adafruit_BME280::SAMPLING_X1, // pressure
                     Adafruit_BME280::SAMPLING_X1, // humidity
                     Adafruit_BME280::FILTER_OFF);

  // suggested rate is 1/60Hz (1m)
  // delayTime = 60000; // in milliseconds
  // t=1+[2xT]+[2xP+0.5]+[2xH+0.5]=1+2+2.5+2.5=13ms
}

uint32_t calculateCRC32( const uint8_t *data, size_t length ) {
  uint32_t crc = 0xffffffff;
  while ( length-- ) {
    uint8_t c = *data++;
    for ( uint32_t i = 0x80; i > 0; i >>= 1 ) {
      bool bit = crc & 0x80000000;
      if ( c & i ) {
        bit = !bit;
      }
      crc <<= 1;
      if ( bit ) {
        crc ^= 0x04c11db7;
      }
    }
  }
  return crc;
}

void setup() {
    Serial.begin(115200);
    pinMode(A0, INPUT);
    pinMode(2, OUTPUT);
    
    Serial.println("");
    Serial.println("\n Start client T.H.P.I.R. sensor for me[R]a Weather station");

/*-------------- Try to read WiFi settings from RTC memory ------------------------*/
    if ( ESP.rtcUserMemoryRead( 0, (uint32_t*)&rtcData, sizeof( rtcData ) ) ) {
      // Calculate the CRC of what we just read from RTC memory, but skip the first 4 bytes as that's the checksum itself.
      uint32_t crc = calculateCRC32( ((uint8_t*)&rtcData) + 4, sizeof( rtcData ) - 4 );
      if (crc == rtcData.crc32) {
        rtcValid = true;
      }
    }  
/*-------------------- Begin BME --------------------------------------------------*/
    Wire.begin(SDA_PIN, SCL_PIN);
    delay(50);
    unsigned long startTime = millis();
    //konekcija na BME cekam max 5s odgovor
    while(!bme280.begin(bme_address,&Wire) && millis() - startTime < 5000UL) {
      Serial.println(" Cannot find sensor.");
      delay(10);
    }
    sensors.begin();
    Serial.println("");
/*-------------------- Config za WiFi ---------------------------------------------*/   
    WiFi.config(mystaticip, mygateway, mysubnet, mygateway);
    WiFi.persistent (false);
    WiFi.mode(WIFI_STA);  
/*---------------------------------------------------------------------------------*/      
}

void loop() {
    Serial.print("\n Start measurement...   \n");
    unsigned long startTime;
    startTime = millis();
    
/*--------------------- Ocitavanje temperature na DS18B20 -------------------------*/
    sensors.requestTemperatures();
    do {
      Tem = sensors.getTempCByIndex(0);
    } while (Tem == 85.0 || Tem == (-127.0) && (millis() - startTime) <= timeout);
    delay(10);
        
/*--------------------- Izmerene vrednosti na BME ---------------------------------*/ 
    BMEsetup();
    bme280.takeForcedMeasurement();
    TemBME=bme280.readTemperature();   
    Hum=bme280.readHumidity();
    Pre=bme280.readPressure()/100.0F;
    Slp=bme280.readAltitude(SEALEVELPRESSURE_HPA);
    // za sad sam ovo iskljucio dok ne ispitam kako da vratim BME u normal mode jer nekad
    // mislim da se zaglupi pa nece da se vrati u normal mod i onda dolazi do wd_reset
    Serial.println(" BME280 to Sleep mode...\n"); //Saljem BME u sleep
    Wire.beginTransmission(bme_address);
    Wire.write((uint8_t)0xF4);                    //Select Control Measurement Register
    Wire.write((uint8_t)0x00);                    //Send '00' for Sleep mode
    Wire.endTransmission(); 
    
/*--------------------- Merenje napona na bateriji --------------------------------*/        
    raw = analogRead(A0);
    delay(10);
    Vcc=raw*calibrate/1023.0;

/*--------------------- Kvalitet wifi signala -------------------------------------*/        
    Rsi=WiFi.RSSI();
    
/*--------------------- Prikaz podataka kroz serial port --------------------------*/
    Serial.println(" Measurement in " + String((millis() - startTime))+ " ms");
    Serial.print (" Battery voltage : ");
    Serial.println (Vcc);
    
    Serial.print("\n - Temperature:     ");
    Serial.print(Tem);
    Serial.println(" [C]");
    Serial.print(" - Temperature BME: ");
    Serial.print(TemBME);
    Serial.println(" [C]");
    Serial.print(" - Humidity:        ");
    Serial.print(Hum);
    Serial.println(" [%]");
    Serial.print(" - Pressure:        ");
    Serial.print(Pre);
    Serial.println(" [mb]");
    Serial.print(" - Approx.Altitude: ");
    Serial.print(Slp);
    Serial.println(" [m]");
    Serial.print(" - Wind speed:       ");
    Serial.print(Win);
    Serial.println(" [m/s]");
    Serial.print(" - Input Voltage:   ");
    Serial.print(Vcc);
    Serial.println(" [V]");
    Serial.print(" - RSSI Value:     ");
    Serial.print(Rsi);
    Serial.println(" [dBm]");
    Serial.println();
    Serial.print(" connecting to SERVER on ip : ");
    Serial.println(host);
    
/*------------------- Kreiram URI za slanje na server -----------------------------*/  
/*------ podatke saljen na server kao argumente u jednoj url adresi ---------------*/
    String url = "http://";
           url += host;
           url += ":";
           url += String(httpPort); 
           url += "/input";
           url += "?Temperature=";
           url += String(Tem,1);
           url += "&Humidity=";
           url += String(Hum,0);
           url += "&Pressure="; 
           url += String(Pre,1);
           url += "&WinSpeed=";
           url += String(Win,2);
           url += "&Voltage=";
           url += String(Vcc,2);
           url += "&RSSI=";
           url += String(Rsi);
           url += "&TempInCase=";
           url += String(TemBME,1);
           url += "&SeaLevelPressure=";
           url += String(Slp,0);
    Serial.print("\n Requesting URL: ");
    Serial.println(url);
    
/*--------------------- Pokrecem wifi konekciju na AP -----------------------------*/
    WiFiBegin(); //connect to wifi router
    Serial.println("\n Measurement and wifi connection in " + String((millis() - startTime))+ " ms\n");
    
/*------------------- Procedura za slanje podataka na server -----------------------*/ 
    WiFiClient client;
    HTTPClient http;
    
    http.begin(client,url);                                      //Posalji podatke na server
    int httpCode = http.GET();                                   //Ocekujem odgovor
    
    Serial.printf (" HTTP Code : %d\n", httpCode);              
    
    if (httpCode > 0) {                                          //Ako ima odgovora od servera 
      if(httpCode == HTTP_CODE_OK) {                             //procitaj koji je
        String response = http.getString();
        Serial.print(" Response code from server: " + response); //Prikaz odgovora na serial portu
      }
    }else {                                                      //ako nema odgovora pokusavam dva puta
      http.end();                                                //slanje podataka a onda zatvaram konekciju
      httpLoop++;
      if (httpLoop > 2){
        Serial.printf(" Error connection. Try again: %s\n", httpLoop);
        return;
      }else {
        httpLoop=0;
        Serial.println(" Client close connection");
      }
    }
    http.end();
    Serial.println("\n Measurement,wifi connection and send data to http in " + String((millis() - startTime))+ " ms\n");
    
/*----------------- Saljem ESP8266 ide u sleep mode -------------------------------*/ 
/*---------- duzina trajanja sleep-a zavisi od nivoa baterije ---------------------*/                    
    Serial.print("\n ESP8266 Going to sleep.\n");
    
      if (Vcc <= BATT_CRT){
        Serial.println(" Battery critical!");
        sleepTimes = INTERVAL_CRT;
      }else if ((Vcc <= BATT_LOW) && (Vcc > BATT_CRT)){
        Serial.println(" Battery low!");
        sleepTimes = INTERVAL_LOW;
      }else if ((Vcc <= BATT_NRM) && (Vcc > BATT_LOW)){
        Serial.println(" Battery normal!");
        sleepTimes = INTERVAL_NRM;
      }  
        Serial.printf(" Sleep Time : %d sec\n", sleepTimes);
        WiFi.disconnect();
        ESP.deepSleep(sleepTimes*1000000); 
        delay(1);
/*---------------------------------------------------------------------------------*/ 
}

void WiFiBegin(){//Konekcija na WiFi ruter i provera uspesnosti konektovanja

    if (WiFi.status() != WL_CONNECTED) {  
        //WiFi.begin(ssid, password);
        if (rtcValid) {
          // The RTC data was good, make a quick connection
          WiFi.begin(ssid, password, rtcData.channel, rtcData.ap_mac, true);
          Serial.println("\n The RTC data was good, make a quick connection\n");
        }else {
          // The RTC data was not valid, so make a regular connection
          WiFi.begin(ssid, password);
          Serial.println("\n The RTC data was not valid, so make a regular connection\n");
        }     
    }
    int count;
    for (count = 0; count < 30 ; count++) {
        int wifi_stat = WiFi.status();  
        Serial.printf("  Attempt: %d >> Wifi Status: %d\n", count, wifi_stat);
        if (wifi_stat == WL_CONNECTED) break;
        delay(500);
    }
    Serial.println("");
    if (count < 30) {
        Serial.println(" WiFi connected");
        //If connection successful show IP address in serial monitor
        Serial.println("");
        Serial.print(" Connected to ");
        Serial.println(ssid);
        Serial.print(" IP address: ");
        Serial.println(WiFi.localIP());  //IP address assigned to your ESP
        Serial.printf(" WiFI chanell : %d\n" , wifi_get_channel());
        Serial.printf(" BSSID: %s\n", WiFi.BSSIDstr().c_str());
        
        // Write current connection info back to RTC
        Serial.println(" Write current connection info back to RTC");
        rtcData.channel = WiFi.channel();
        memcpy(rtcData.ap_mac, WiFi.BSSID(), 6); // Copy 6 bytes of BSSID (AP's MAC address)
        rtcData.crc32 = calculateCRC32(((uint8_t*)&rtcData) + 4, sizeof( rtcData ) - 4);
        ESP.rtcUserMemoryWrite(0, (uint32_t*)&rtcData, sizeof(rtcData));
    }
    else {
        Serial.println(" Client !!!NOT!!! connected");
        ESP.deepSleep(300*1000000); 
        delay(1); 
    }
}
