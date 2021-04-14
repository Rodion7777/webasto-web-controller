#include "PCF8574.h"
#include <EEPROM.h>
#include <DS18B20.h>
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESPAsyncTCP.h>
#include <ESPDash.h>
#include <ESP8266mDNS.h>
#include <DNSServer.h>
#include <ESPAsyncWebServer.h>     
#include <ESPAsyncWiFiManager.h>
#include <max6675.h>

static const uint8_t D0 = 16, D1 = 5, D2 = 4,D3 = 0, D4 = 2, D5 = 14, D6 = 12, D7 = 13, D8 = 15, D9 = 3, D10 = 1;

static const uint8_t  PCF_P0_WATER_PUMP_CTL = 0, //output
                      PCF_P1_SYSTEM_ENABLED_CTL = 1, //output
                      PCF_P2_BURNER_CTL = 2, //output
                      PCF_P5_SYS_BLOCKED_SIG = 5, //input
                      PCF_P6_LOW_PRESSURE_SIG = 6, //input
                      PCF_P7_SYS_WORKING_SIG = 7; //input

int const MIN_TEMP_LOWER_THRESHHOLD = 30;
int const MAX_TEMP_LOWER_THRESHHOLD = 50;
int const MIN_TEMP_UPPER_THRESHHOLD = 55;
int const MAX_TEMP_UPPER_THRESHHOLD = 88;

const char *ssid = "Alpha Centauri";
const char *password = "dai_inet";
const char *dname = "esp8266";

bool wifiDisconnectedFlag;
bool lowPressureSignal;
bool systemWorkingSignal;
bool systemBlockedSignal;
bool lowPressureSignalOld  = true;
bool systemWorkingSignalOld = true;
bool systemBlockedSignalOld  = true;
bool systemEnabled = false;
bool waterPumpEnabled;

DS18B20 ds(D4);


uint8_t ts1Addr[] = {40, 41, 10, 131, 34, 32, 1, 150};
uint8_t ts2Addr[] = {40, 140, 23, 116, 34, 32, 1, 70};
MAX6675 ts3(PIN_SPI_SCK, PIN_SPI_MOSI, PIN_SPI_MISO);

int statusLabelId;
int graphId;
int millisLabelId;
int testSwitchId;
long oldTime;
float t1 = 0;
float t2 = 0;
float t3 = 0;

AsyncWebServer server(80);
ESPDash dashboard(&server); 
DNSServer dns;

Card thermostatStatusCard(&dashboard, STATUS_CARD, "THERMOSTAT");
Card systemWorkingSignalStatusCard(&dashboard, STATUS_CARD, "BOILER STATUS");
Card lowPressureSignalStatusCard(&dashboard, STATUS_CARD, "PRESSURE STATUS");
Card systemBlockedSignalStatusCard(&dashboard, STATUS_CARD, "SYSTEM LOCK STATE");


Card temperatureIntakeCard(&dashboard, TEMPERATURE_CARD, "RETURN t1", "°C");
Card temperatureOtletCard(&dashboard, TEMPERATURE_CARD, "SUPPLY t2", "°C");
Card temperatureExhaustCard(&dashboard, TEMPERATURE_CARD, "EXHAUST t3", "°C");

Card minTempSliderCard(&dashboard, SLIDER_CARD, "COOLANT Min t°C", "", MIN_TEMP_LOWER_THRESHHOLD, MAX_TEMP_LOWER_THRESHHOLD);
Card maxTempSliderCard(&dashboard, SLIDER_CARD, "COOLANT Max t°C", "", MIN_TEMP_UPPER_THRESHHOLD, MAX_TEMP_UPPER_THRESHHOLD);

Card mainSwitchButtonCard(&dashboard, BUTTON_CARD, "LAUNCHER SWITCH");
Card forceWatertPumpButtonCard(&dashboard, BUTTON_CARD, "FORCE PUMP");

uint settingsAddr = 0;
struct { 
    int minTemp;
    int maxTemp;
} settings;


// adjust addresses if needed
PCF8574 PCF(0x27);

int pin = 0;


void loadDefaults(){
  settings.minTemp = MIN_TEMP_LOWER_THRESHHOLD;
  settings.maxTemp = MIN_TEMP_UPPER_THRESHHOLD;
}

void loadSettingsFromEEPROM(){
  EEPROM.begin(512);
  EEPROM.get(settingsAddr, settings);
}

void updateEEPROM(){
  EEPROM.put(settingsAddr,settings);
  EEPROM.commit();
}


float getT1(){
    ds.select(ts1Addr);
    return (float)ds.getTempC();
}

float getT2(){
    ds.select(ts2Addr);
    return (float)ds.getTempC();
}

float getT3(){
    return ts3.readCelsius();
}

void forceEnableWaterPump(){
  PCF.write(PCF_P0_WATER_PUMP_CTL, LOW);
}

void disableWaterPumpForcing(){
  PCF.write(PCF_P0_WATER_PUMP_CTL, HIGH);
}

void enableBurner(){
  if(systemEnabled && !systemBlockedSignal && !lowPressureSignal){
      PCF.write(PCF_P2_BURNER_CTL, LOW);
  }
}

void disableBurner(){
  PCF.write(PCF_P2_BURNER_CTL, HIGH);
}

void launchSystem(){
  if(!systemBlockedSignal && !systemWorkingSignal && !lowPressureSignal){
      PCF.write(PCF_P1_SYSTEM_ENABLED_CTL, LOW);
      systemEnabled = true;
      mainSwitchButtonCard.update(String("STOP"), "danger");
  }
  
}

void haltSystem(){
  systemEnabled = false;
  disableBurner();
  PCF.write(PCF_P1_SYSTEM_ENABLED_CTL, HIGH);
}

bool getBurnerState(){
  return !PCF.read(PCF_P2_BURNER_CTL);
}

void updateSystemStatus(){
  t1 = getT1();
  t2 = getT2();
  t3 = getT3();
  temperatureIntakeCard.update(t1);
  temperatureOtletCard.update(t2);
  temperatureExhaustCard.update(t3);
  
  systemBlockedSignal = !bool(PCF.read(PCF_P5_SYS_BLOCKED_SIG));
  lowPressureSignal = !bool(PCF.read(PCF_P6_LOW_PRESSURE_SIG));
  systemWorkingSignal = !bool(PCF.read(PCF_P7_SYS_WORKING_SIG));

    if(systemWorkingSignal){
        systemWorkingSignalStatusCard.update(String("ENABLED"), "success");
    } else {
        systemWorkingSignalStatusCard.update(String("DISABLED"), "warning");
    }
    
    if(lowPressureSignal){
      lowPressureSignalStatusCard.update(String("LOW PRESSURE"), "danger");
      mainSwitchButtonCard.update(String("CRITICAL ERROR"), "idle");
      haltSystem();
    } else {
      lowPressureSignalStatusCard.update(String("OK"), "success");
    }
    
    if(systemBlockedSignal){
      systemBlockedSignalStatusCard.update(String("BLOCKED"), "danger");
      mainSwitchButtonCard.update(String("DISABLED"), "idle");
      haltSystem();
    } else {
      systemBlockedSignalStatusCard.update(String("READY"), "success");
      if(!systemEnabled)
          mainSwitchButtonCard.update(String("START"), "success");
    }
    
  if(systemWorkingSignal != systemWorkingSignalOld){
    systemWorkingSignalOld = systemWorkingSignal;
    Serial.println("new systemWorkingSignal: " + String(systemWorkingSignal));
  } 
    
  if(lowPressureSignal != lowPressureSignalOld){
    lowPressureSignalOld = lowPressureSignal;
    Serial.println("new lowPressureSignal: " + String(lowPressureSignal));
  }

  if(systemBlockedSignal != systemBlockedSignalOld){
    systemBlockedSignalOld = systemBlockedSignal;
    Serial.println("new systemBlockedSignal: " + String(systemBlockedSignal));
  }
}

void updateBurnerState(){
  bool burning = getBurnerState();
  updateSystemStatus();

  if(burning){
    thermostatStatusCard.update(String("[Δ°C ") + String(t2-t1) + String("]"), "danger");
  } else {
    thermostatStatusCard.update(String("[Δ°C ") + String(t2-t1) + String("]"), "success");
  }
  
  
  if(t1 <= 0 || t2 <= 0){
    disableBurner();
    return;
  }

 
  if(systemEnabled){
    if(!burning && t1 < settings.minTemp && t2 < settings.minTemp){
       enableBurner();
    }
  } else {
    disableBurner();
  }

  if(burning && (t1 > settings.maxTemp || t2 > settings.maxTemp)){
     disableBurner();
  }

}


void setup()
{
  Serial.begin(115200);
  Serial.println("\nInitializing....");
  loadSettingsFromEEPROM();

  if(settings.maxTemp < 0 || settings.maxTemp > MAX_TEMP_UPPER_THRESHHOLD){
      loadDefaults();
  }


  if (!PCF.begin())
  {
    Serial.println("could not initialize...");
  }
  
  if (!PCF.isConnected())
  {
    Serial.println("=> not connected");
    while(1);
  }

  updateSystemStatus();

  Serial.print("DS 18b20 Devices: ");
  Serial.println(ds.getNumberOfDevices());
  Serial.println();


  /* Connect WiFi */
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
      Serial.printf("WiFi Failed!\n");
      return;
  }
  
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  minTempSliderCard.update(settings.minTemp);
  maxTempSliderCard.update(settings.maxTemp);
  thermostatStatusCard.update(String("N/A"), "idle");
  
  minTempSliderCard.attachCallback([&](int value){
    minTempSliderCard.update(value);
    settings.minTemp=value;
    updateEEPROM();
    dashboard.sendUpdates();
  });
  
  maxTempSliderCard.attachCallback([&](int value){
    maxTempSliderCard.update(value);
    settings.maxTemp=value;
    updateEEPROM();
    dashboard.sendUpdates();
  });

  mainSwitchButtonCard.attachCallback([&](bool value){
    if(!systemEnabled){
       launchSystem();
    } else {
       haltSystem();
       mainSwitchButtonCard.update(String("START"), "success");
    }

    dashboard.sendUpdates();
  });

  forceWatertPumpButtonCard.update(String("OFF"), "danger");
  forceWatertPumpButtonCard.attachCallback([&](bool value){
    waterPumpEnabled = waterPumpEnabled?false:true;
    if(waterPumpEnabled){
      forceWatertPumpButtonCard.update(String("ON"), "success");
      disableWaterPumpForcing();
    } else {
      forceWatertPumpButtonCard.update(String("OFF"), "danger");
      forceEnableWaterPump();
    }
    dashboard.sendUpdates();
  });

  server.begin();
  
}


void loop()
{

  if (millis() - oldTime > 700) {
      updateBurnerState();
      dashboard.sendUpdates();
      oldTime = millis();
  }

  if ( digitalRead(D3) == LOW ) {
    WiFi.disconnect(true);
    AsyncWiFiManager wifiManager(&server,&dns);
    wifiManager.startConfigPortal("Disel heater");
    Serial.println("connected...yeey :)");
  }

}


// -- END OF FILE --
