  /*
  Upgrade with new reprogrammed client v1.1 31-Dec-2020. 13:39
  |======================================|
  |  Weather WiFi Client v2.0 By me[R]a  |
  |  30-Aug-2025. 19:02                  | 
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
      
    #define SEALEVELPRESSURE_HPA (1013.25) //Air pressure at sea level
  
  //RTC
    //The ESP8266 RTC memory is arranged into blocks of 4 bytes. The access methods read and write 4 bytes at a time,
    //so the RTC data structure should be padded to a 4-byte multiple.
    struct {
      uint32_t crc32;   // 4 bytes
      uint8_t channel;  // 1 byte, 5 in total
      uint8_t ap_mac[6];
      uint8_t padding;  // 1 byte, 12 in total
    } rtcData;
  
    bool rtcValid = false; //variable to check if there is any data saved in rtc memory

  //Measure Vbat
    /* add 100k between Vbat and ADC
     * Voltage divider of 100k+220k over 100k
     * gives 100/420k
     * ergo 4.2V -> 1Volt
     * Max input on A0=1Volt ->1023
     * 4.2*(Raw/1023)=Vbat
    */
    //Voltage divider R1 = 220k+100k+220k =540k and R2=100k => 5.28
    float calibrate = 4.20; // this value for calibrate the battery voltage   
    unsigned int raw=0;

  //BME280
    #define SEALEVELPRESSURE_HPA (1013.25)
    #define SDA_PIN D2          //data pin BME280
    #define SCL_PIN D1          //clock pin BME280
    Adafruit_BME280 bme280;     //I2C
    uint8_t bme_address = 0x76; //set to 0x76 or 0x77
    
  //DS18B20
    #define ONE_WIRE_BUS D7              //Data wire connected to digital pin 7 on ESP8266
    OneWire oneWire(ONE_WIRE_BUS);       //Setting up a oneWire instance to communicate with some OneWire devices
    DallasTemperature sensors(&oneWire); //oneWire Reference for Dallas DS18B20 Temp. sensor
  
  //WiFi
    const char* ssid =     "MeteoStanica-MERA";
    const char* password = "NeveNa91";
    const char* host =     "192.168.4.1";
    const int   hostPort = 3003;
    /*const uint8_t bssid[] = {0xBE, 0xDD, 0xC2, 0x9C, 0xB2, 0x79}; //MAC from AP ESP8266 */
    const uint8_t bssid[] = {0xC4, 0x4F, 0x33, 0x7F, 0xEF, 0x61};   //MAC from AP ESP32

  //IP
    IPAddress mystaticip(192, 168, 4, 2); //If there are more clients connected to the server, then the IP needs to be changed.
    IPAddress mygateway(192, 168, 4, 1);
    IPAddress mysubnet(255, 255, 255, 0);
    IPAddress mydns(192, 168, 4, 1);
    
  //Battery setup
    float BATT_NRM = 4.30;                //normal battery voltage    -> use INTERVAL_NRM
    float BATT_LOW = 3.60;                //low battery voltage       -> use INTERVAL_LOW
    float BATT_CRT = 3.30;                //critical battery voltage  -> use INTERVAL_CRT

  //Wake up interval
    #define INTERVAL_NRM  600             //normal wake up interval in seconds
    #define INTERVAL_LOW  900             //wake up interval when low battery
    #define INTERVAL_CRT 1200             //wake up intrrval when crtical battery
    
    unsigned long durationTime = 0;              //Duration time
    const unsigned long timeout = 3000;   //Timeout in millisekonds 
   
    int sleepTimes ;                      //Time to sleep (in seconds):    
    #define LED_BLUE 2                    //Blue led on pin 2 (TX) on nodeMcu
    unsigned long startTime;              //start time
  
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
      
    // Ako zelis debug ispise, odkomentariši #define DEBUG
    //#define DEBUG  

  void setup() {
    startTime = millis();

    #ifdef DEBUG
      Serial.begin(115200);
      Serial.print("\n Begin measurement devices...   \n");
    #endif

    // Gasi ugrađenu LED (štedi struju)
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH); // HIGH = LED off na ESP8266

    pinMode(A0, INPUT);
    pinMode(2, OUTPUT);

    #ifdef DEBUG
      Serial.println("\n Start client T.H.P.I.R. sensor for me[R]a Weather station");
    #endif

    // Pročitaj RTC WiFi podatke
    if ( ESP.rtcUserMemoryRead(0, (uint32_t*)&rtcData, sizeof(rtcData)) ) {
      uint32_t crc = calculateCRC32(((uint8_t*)&rtcData) + 4, sizeof(rtcData) - 4);
      if (crc == rtcData.crc32) {
        rtcValid = true;
      }
    }

    // Inicijalizuj BME280
    Wire.begin(SDA_PIN, SCL_PIN);
    if (!bme280.begin(bme_address, &Wire)) {
      #ifdef DEBUG
        Serial.println(" Could not find a valid BME280 sensor, check wiring!");
      #endif
    }

    // DS18B20 podešavanje
    sensors.begin();
    sensors.setWaitForConversion(false);  // async mod
    sensors.setResolution(10);
    sensors.requestTemperatures(); // startuj konverziju odmah

    // Konfiguriši WiFi
    WiFi.config(mystaticip, mygateway, mysubnet, mydns);
    WiFi.persistent(false);
    WiFi.mode(WIFI_STA);

    if (rtcValid) {
      WiFi.begin(ssid, password, rtcData.channel, rtcData.ap_mac, true);
      #ifdef DEBUG
        Serial.println("\n The RTC data was good, make a quick connection\n");
      #endif
    } else {
      WiFi.begin(ssid, password);
      #ifdef DEBUG
        Serial.println("\n The RTC data was not valid, so make a regular connection\n");
      #endif
    }

    // Čekaj konekciju max ~5 sekundi
    int count = 0;
    while (WiFi.status() != WL_CONNECTED && count < 10) {
      delay(500);
      #ifdef DEBUG
        Serial.println(" Connecting to WiFi...");
      #endif
      count++;
    }

    if (WiFi.status() == WL_CONNECTED) {
      #ifdef DEBUG
        Serial.print(" Connected to ");
        Serial.println(ssid);
        Serial.print(" IP address: ");
        Serial.println(WiFi.localIP());
        Serial.printf(" WiFI channel : %d\n", wifi_get_channel());
        Serial.printf(" BSSID: %s\n", WiFi.BSSIDstr().c_str());
      #endif

      // Snimi aktuelne WiFi parametre u RTC
      rtcData.channel = WiFi.channel();
      memcpy(rtcData.ap_mac, WiFi.BSSID(), 6);
      rtcData.crc32 = calculateCRC32(((uint8_t*)&rtcData) + 4, sizeof(rtcData) - 4);
      ESP.rtcUserMemoryWrite(0, (uint32_t*)&rtcData, sizeof(rtcData));

      sendData();

    } else {
      #ifdef DEBUG
        Serial.println(" Failed to connect to WiFi, going to sleep");
      #endif
      // Ako nema konekcije, obavezno u sleep
      ESP.deepSleep(INTERVAL_LOW * 1000000ULL);
    }
  }

    
  void loop() {
    //Everything is handled in setup, loop is empty
  }
    
  //Sending measured data
  void sendData() {
    // --- BME280 measurement ---
    bme280.setSampling(Adafruit_BME280::MODE_FORCED,
                        Adafruit_BME280::SAMPLING_X1, // temperature
                        Adafruit_BME280::SAMPLING_X1, // pressure
                        Adafruit_BME280::SAMPLING_X1, // humidity
                        Adafruit_BME280::FILTER_OFF);

    float bmeTemp     = bme280.readTemperature();
    float bmeHum      = bme280.readHumidity();
    float bmePressure = bme280.readPressure() / 100.0F;

    // --- DS18B20 temperature ---
    float dsTemp = sensors.getTempCByIndex(0);

    // --- Battery voltage ---
    raw = analogRead(A0);
    float vBattery = raw * calibrate / 1023.0;

    // --- WiFi signal ---
    int Rsi = WiFi.RSSI();

    float Win = 0.0; 
    float Slp = 0.0;

    #ifdef DEBUG
      // Debug printovi samo ako je uključen DEBUG
      Serial.printf("\nTemperature: %.1f C (DS18B20)\n", dsTemp);
      Serial.printf("Temperature BME: %.1f C\n", bmeTemp);
      Serial.printf("Humidity: %.0f %%\n", bmeHum);
      Serial.printf("Pressure: %.1f mb\n", bmePressure);
      Serial.printf("Voltage: %.2f V\n", vBattery);
      Serial.printf("RSSI: %d dBm\n", Rsi);
      Serial.printf("Connecting to SERVER on ip: %s\n", host);
    #endif

    // --- Send data via HTTP ---
    if (WiFi.status() == WL_CONNECTED) {
        String url = "http://" + String(host) + ":" + String(hostPort) + "/input?" +
                      "Temperature="        + String(dsTemp,1) +
                      "&bmeTemp="           + String(bmeTemp) +
                      "&Humidity="          + String(bmeHum,0) +
                      "&Pressure="          + String(bmePressure,1) +
                      "&WinSpeed="          + String(Win,2) +
                      "&Voltage="           + String(vBattery,2) +
                      "&RSSI="              + String(Rsi) +
                      "&TempInCase="        + String(bmeTemp,1) +
                      "&SeaLevelPressure="  + String(Slp,0) +
                      "&DurationTime="       + String(durationTime,0);

        #ifdef DEBUG
                Serial.printf("\nRequesting URL: %s\n", url.c_str());
        #endif

        WiFiClient client;
        HTTPClient http;
        http.begin(client, url);
        
        /*int httpResponseCode = http.GET();  // GET metod
        #ifdef DEBUG
          if (httpResponseCode > 0) {
              Serial.printf("HTTP Response code: %d\n", httpResponseCode);
          } else {
              Serial.printf("Error code: %d\n", httpResponseCode);
          }
        #endif*/

        http.end();

        // --- Disconnect WiFi to save power ---
        WiFi.disconnect(true);
        WiFi.mode(WIFI_OFF);

    }

    // --- Put BME280 to sleep ---
    Wire.beginTransmission(bme_address);
    Wire.write((uint8_t)0xF4); // Control Measurement Register
    Wire.write((uint8_t)0x00); // Sleep mode
    Wire.endTransmission(); 

    // --- Battery check for sleep interval ---
    if (vBattery <= BATT_CRT) {
        sleepTimes = INTERVAL_CRT;
        #ifdef DEBUG
          Serial.println("Battery critical!");
        #endif
    } else if (vBattery <= BATT_LOW) {
        sleepTimes = INTERVAL_LOW;
        #ifdef DEBUG
          Serial.println("Battery low!");
      #endif
    } else if (vBattery <= BATT_NRM) {
        sleepTimes = INTERVAL_NRM;
        #ifdef DEBUG
          Serial.println("Battery normal!");
        #endif
    }
    durationTime = millis() - startTime;
    #ifdef DEBUG
      Serial.printf("Sleep Time: %d sec\n", sleepTimes);
      Serial.println("ESP8266 Going to sleep.\n");
    #endif

    // --- Deep sleep ---
    ESP.deepSleep(sleepTimes * 1000000ULL); 
  }

