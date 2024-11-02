#include <Arduino.h>
#include <Wire.h>
#include <vl53lx_class.h>

#include <BTstackLib.h>
#include <SPI.h>
#include <ble/att_server.h>

#include "DistanceService.h"

static char characteristic_data = 'H';
const long interval = 1000; 
uint16_t char_val;

#define BLEDEVICENAME "DstSns"
#define ADD_KRILON_DIST_SRV_ADV 1
#define DEBUG_SETUP 0
#define PRINT_DIST_REP 0
#define DEV_I2C Wire

VL53LX sensor_vl53lx_sat(&DEV_I2C, -1);

// the setup function runs once when you press reset or power the board
void setup() {

  VL53LX_Error status;
  Serial.begin(115200);
#if DEBUG_SETUP  
  while (!Serial) yield();
#endif
  
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  // Initialize I2C bus.
  DEV_I2C.setSDA(20);
  DEV_I2C.setSCL(21);
  DEV_I2C.begin();

  // Configure VL53LX satellite component.
  sensor_vl53lx_sat.begin();

  // Switch off VL53LX satellite component.
  sensor_vl53lx_sat.VL53LX_Off();

   //Initialize VL53LX satellite component.
  status = sensor_vl53lx_sat.InitSensor(VL53LX_DEFAULT_DEVICE_ADDRESS);
  Serial.printf("init distance: %u\n", status);

  VL53LX_DeviceInfo_t devInfo;
  status = sensor_vl53lx_sat.VL53LX_GetDeviceInfo(&devInfo);
  Serial.printf("Distance device: %u [%u.%u] (%u)\n", devInfo.ProductType, devInfo.ProductRevisionMajor, devInfo.ProductRevisionMinor, status);

  VL53LX_DistanceModes mode;
  status = sensor_vl53lx_sat.VL53LX_GetDistanceMode(&mode);
  Serial.printf("Distance mode: %u (%u)\n", mode, status);

  status = sensor_vl53lx_sat.VL53LX_StopMeasurement();
  Serial.printf("stop distance: %u\n", status);

  // Start Measurements
  status = sensor_vl53lx_sat.VL53LX_StartMeasurement();
  Serial.printf("start distance: %u\n", status);

  //status = sensor_vl53lx_sat.VL53LX_ClearInterruptAndStartMeasurement();
  //Serial.printf("Clear and start: %u\n", status);

  // set callbacks
  BTstack.setBLEDeviceConnectedCallback(deviceConnectedCallback);
  BTstack.setBLEDeviceDisconnectedCallback(deviceDisconnectedCallback);
  BTstack.setGATTCharacteristicRead(gattReadCallback);
  BTstack.setGATTCharacteristicWrite(gattWriteCallback);
  BTstack.setGATTCharacteristicSubscribedCallback(gattSubscribedCallback);
  BTstack.setGATTCharacteristicUnsubscribedCallback(gattUnsubscribedCallback);
  
  char_val = DistanceService::SetupBtStack();
  
  // startup Bluetooth and activate advertisements
  BTstack.setup(BLEDEVICENAME);

  distanceService.Init(char_val);

  Serial.print("Add gatt char: ");
  Serial.println(char_val, HEX);
  Serial.flush();

  SetupAdvData(BLEDEVICENAME);
  BTstack.startAdvertising();

  digitalWrite(LED_BUILTIN, LOW);
}


uint8_t adv_data[37];

void SetupAdvData(const char *name)
{
      // setup advertisements data
    uint8_t adv_data_len = 0;
    int pos = 0;
    const uint8_t flags[] = { 0x02, 0x01, 0x02 };
    memcpy(&adv_data[pos], flags, sizeof(flags));
    pos += sizeof(flags);
    adv_data[pos++] = strlen(name) + 1;
    adv_data[pos++] = 0x09;
    memcpy(&adv_data[pos], name, strlen(name));
    pos += strlen(name);

#if ADD_KRILON_DIST_SRV_ADV
    UUID distServUuid(DIST_GATT_SERVICE);
    const uint8_t *uuidPtr = distServUuid.getUuid();
    adv_data[pos++] = 16 + 1;
    adv_data[pos++] = 7;
    for(int n = 15; n >=0; n--)
    {
      adv_data[pos++] = uuidPtr[n];
    }
#endif    

    adv_data_len = pos;
    Serial.printf("Adding ADV data %d\n", adv_data_len);
    gap_advertisements_set_data(adv_data_len, adv_data);
}

int ledState = LOW;  
unsigned long previousMillis = 0;

// the loop function runs over and over again forever
void loop()
{
#if LOOP_EXCEPTIONS
  try
  {
#endif
    BTstack.loop();
    unsigned long currentMillis = millis();
#if 0
    if (currentMillis - previousMillis >= interval) {
    // save the last time you blinked the LED
    previousMillis = currentMillis;
    // if the LED is off turn it on and vice-versa:
    if (ledState == LOW) {
      ledState = HIGH;
    } else {
      ledState = LOW;
    }

    // set the LED with the ledState of the variable:
    digitalWrite(LED_BUILTIN, ledState);
  }
#endif

    VL53LX_MultiRangingData_t MultiRangingData;
    VL53LX_MultiRangingData_t *pMultiRangingData = &MultiRangingData;
    uint8_t NewDataReady = 0;
    int no_of_object_found = 0, j;
    char report[64];
    int status;

    status = sensor_vl53lx_sat.VL53LX_GetMeasurementDataReady(&NewDataReady);
    if ((!status) && (NewDataReady != 0))
    {
      status = sensor_vl53lx_sat.VL53LX_GetMultiRangingData(pMultiRangingData);
      no_of_object_found = pMultiRangingData->NumberOfObjectsFound;
      if (no_of_object_found > 0)
      {
        int16_t distance = -1;
#if PRINT_DIST_REP
        snprintf(report, sizeof(report), "VL53LX Satellite: Count=%d, #Objs=%1d ", pMultiRangingData->StreamCount, no_of_object_found);
        Serial.print(report);
#endif
        for (j = 0; j < no_of_object_found; j++)
        {
          int16_t targetDistance = pMultiRangingData->RangeData[j].RangeMilliMeter;
          //Serial.printf("Measure[%d]: %d vs %d\n", j, targetDistance, distance);

          if (distance < 0 || targetDistance < distance)
          {
            distance = targetDistance;
          }
#if PRINT_DIST_REP
          if (j != 0)
            Serial.print("\r\n                               ");

          Serial.print("status=");
          Serial.print(pMultiRangingData->RangeData[j].RangeStatus);
          Serial.print(", D=");

          Serial.print(pMultiRangingData->RangeData[j].RangeMilliMeter);
          Serial.print("mm");
          Serial.print(", Signal=");
          Serial.print((float)pMultiRangingData->RangeData[j].SignalRateRtnMegaCps / 65536.0);
          Serial.print(" Mcps, Ambient=");
          Serial.print((float)pMultiRangingData->RangeData[j].AmbientRateRtnMegaCps / 65536.0);
          Serial.print(" Mcps");
#endif
        }

#if PRINT_DIST_REP
        Serial.println("");
#endif
        if (distance >= 0)
        {
          //Serial.printf("Distance: %d\n", distance);
          distanceService.DistanceServiceUpdateValue(static_cast<uint16_t>(distance));
        }
      }

      if (status == 0)
      {
        status = sensor_vl53lx_sat.VL53LX_ClearInterruptAndStartMeasurement();
      }
    }

    if (status != VL53LX_ERROR_NONE)
    {
      Serial.printf("Poll distance: %u\n", status);
    }

    distanceService.DistanceServiceNotify(currentMillis);
#if LOOP_EXCEPTIONS
  }
  catch (const std::exception &e)
  {
    Serial.printf("Loop failed: %s\n", e.what());
  }
#endif
}

BLEDevice *connectedDevice;
/*
   @section Device Connected Callback

   @text When a remove device connects, device connected callback is callec.
*/
/* LISTING_START(LEPeripheralDeviceConnectedCallback): Device Connected Callback */
void deviceConnectedCallback(BLEStatus status, BLEDevice *device) {
  connectedDevice = device;
  switch (status) {
    case BLE_STATUS_OK:
      Serial.println("Device connected!");
      break;
    default:
      break;
  }
}

bool isNotificationsEnabled = false;
void deviceDisconnectedCallback(BLEDevice * device) {
  if (connectedDevice == device)
  {
    connectedDevice = 0;
    isNotificationsEnabled = false;
  }

  distanceService.Disconnect();
  Serial.println("Disconnected.");
}

void gattSubscribedCallback(BLEStatus status, BLEDevice *device) {
  
  //connectedDevice = device;
  switch (status) {
    case BLE_STATUS_OK:
      Serial.println("Device subscribed !");
      break;
    default:
      break;
  }
}

void gattUnsubscribedCallback(BLEStatus status, BLEDevice *device) {
  switch (status) {
    case BLE_STATUS_OK:
      Serial.println("Device unsubscribed !");
      break;
    default:
      break;
  }
}


/*
   @section Read Callback

   @text In BTstack, the Read Callback is first called to query the size of the
   Characteristic Value, before it is called to provide the data.
   Both times, the size has to be returned. The data is only stored in the provided
   buffer, if the buffer argeument is not NULL.
   If more than one dynamic Characteristics is used, the value handle is used
   to distinguish them.
*/
uint16_t gattReadCallback(uint16_t value_handle, uint8_t * buffer, uint16_t buffer_size) {
  (void) buffer_size;
  if (buffer) {
    Serial.print("gattReadCallback (");
    Serial.print(value_handle, HEX);
    Serial.print(") value: ");
    Serial.println(characteristic_data, HEX);
    buffer[0] = characteristic_data;
  }
  return 1;
}

uint16_t distanceMeasureNotifyConfig = 0;
uint16_t distanceMeasureNotifyConHandle = 0;


/*
   @section Write Callback

   @text When the remote device writes a Characteristic Value, the Write callback
   is called. The buffer arguments points to the data of size size/
   If more than one dynamic Characteristics is used, the value handle is used
   to distinguish them.
*/
int gattWriteCallback(uint16_t con_handle, uint16_t att_handle, uint8_t *buffer, uint16_t size) {
  (void) size;
  Serial.print("gattWriteCallback (");
  Serial.print(att_handle, HEX);
  Serial.print(", ");
  Serial.print(char_val, HEX);
  if (att_handle == char_val)
  {
    characteristic_data = buffer[0];
    Serial.print(") value: ");
    Serial.println(characteristic_data, HEX);
  }
  else if (att_handle != char_val + 1)
  {
    uint16_t notifyConfig = little_endian_read_16(buffer, 0);
    distanceMeasureNotifyConfig = notifyConfig;
    distanceMeasureNotifyConHandle = con_handle;
    Serial.println(") Notify: ");
    Serial.println(distanceMeasureNotifyConfig, HEX);
  }
  else
  {
    Serial.println(")");
  }
  
  return 0;
}


