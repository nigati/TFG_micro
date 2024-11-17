#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <DHT.h>
#include <XSpaceBioV10.h>
#include <MAX30100_PulseOximeter.h>


Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

#define DEBUG_SERIAL Serial
#define ACCELERATION_SERVICE_UUID "0000180A-0000-1000-8000-00805F9B34FB"
#define ACCELERATION_CHARACTERISTIC_UUID "00002A58-0000-1000-8000-00805F9B34FB"

BLEServer *pServer = NULL;
BLECharacteristic *pCharacteristic = NULL;
bool deviceConnected = false;

// Pin definitions for the first AD8232 sensor setup
#define ECG_OUT_A 34  // ECG output pin for the first AD8232 sensor
#define SDN_A 33      // Shutdown pin for the first AD8232 sensor
#define LOH_A 25      // Lead-off detection high pin for the first sensor
#define LOL_A 26      // Lead-off detection low pin for the first sensor
#define AD8232_A 0    // Identifier for the first AD8232 sensor
XSpaceBioV10Board heartMonitor;
//Pin definitions for DHT22
#define DHTPIN 23  // GPIO for DHT22 Data pin
#define DHTTYPE DHT22  // DHT 22 (AM2302)
DHT dht(DHTPIN, DHTTYPE);

void displaySensorDetails(void) {
  sensor_t sensor;
  accel.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print("Sensor:       "); Serial.println(sensor.name);
  Serial.print("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" m/s^2");
  Serial.print("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" m/s^2");
  Serial.print("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" m/s^2"); 
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}


class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
  };

  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
  }
};

void setup(void) {
  Serial.begin(115200);
  Serial.println("Accelerometer, ECG, DHT22 and MAX30100 test"); 
  if(!accel.begin()) {
    Serial.println("Ooops, no ADXL345 detected ... Check your wiring!");
    while (1) {
      delay(10);
    }
  }
  heartMonitor.AD8232_init(AD8232_A, SDN_A, LOH_A, LOL_A, ECG_OUT_A);
  heartMonitor.AD8232_Wake(AD8232_A);

  dht.begin();

  BLEDevice::init("Smart Helmet");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(ACCELERATION_SERVICE_UUID);

  pCharacteristic = pService->createCharacteristic(
                      ACCELERATION_CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ |
                      BLECharacteristic::PROPERTY_NOTIFY
                    );
  pCharacteristic->addDescriptor(new BLE2902());

  pService->start();
  pServer->getAdvertising()->start();
  Serial.println("Waiting for a client connection to notify...");

  accel.setRange(ADXL345_RANGE_16_G);
  displaySensorDetails();
}

void loop(void) {
  float temperature = dht.readTemperature();
  float humidity = dht.readHumidity();
  float ecgValue= heartMonitor.AD8232_GetVoltage(AD8232_A);
  
  
  if (deviceConnected) {
    sensors_event_t event;
    accel.getEvent(&event);
    String data = String(event.acceleration.x) + "," + 
                  String(event.acceleration.y) + "," + 
                  String(event.acceleration.z) + "," +
                  String(temperature) + "," +
                  String(humidity) + "," +
                  String(ecgValue);
    Serial.println("Sending data to client: " + data);
    pCharacteristic->setValue(data.c_str());
    pCharacteristic->notify();
    delay(1000);
  } else {
    sensors_event_t event;
    accel.getEvent(&event);
    Serial.print("X: "); Serial.print(event.acceleration.x); Serial.print("  ");
    Serial.print("Y: "); Serial.print(event.acceleration.y); Serial.print("  ");
    Serial.print("Z: "); Serial.print(event.acceleration.z); Serial.print("  ");
    Serial.println("m/s^2 ");
    Serial.print("Temperature: "); Serial.print(temperature); Serial.print("  ");
    Serial.print("Humidity: "); Serial.print(humidity); Serial.print("  ");
    Serial.print("ECG voltage: "); Serial.print(ecgValue); Serial.print("  ");


    if (abs(event.acceleration.z) <= 5) {
      Serial.println("Not worn");
    } else {
      Serial.println("Worn");
    }
    delay(10);
  }
 } 
