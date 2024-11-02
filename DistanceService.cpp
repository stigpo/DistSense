#include <Arduino.h>

#include "DistanceService.h"
#include <BTstackLib.h>
#include <btstack.h>

#define PRINT_DIST_NOTIFY_DISCONECTED 1

static att_service_handler_t gattDistanceService;
DistanceService distanceService;

DistanceService::DistanceService()
{
    reportWithCmResolution = true;
    con_handle = 0;
    prevNotifyMillis = 0;
}

uint16_t DistanceService::SetupBtStack()
{
      // setup GATT Database
  BTstack.addGATTService(new UUID(DIST_GATT_SERVICE));
  return BTstack.addGATTCharacteristicDynamic(new UUID(DIST_GATT_CHAR), ATT_PROPERTY_READ | ATT_PROPERTY_NOTIFY, 0);
}

void DistanceService::Init(uint16_t serviceHandle)
{
  // Should these be extracted from DB using the utility functions?

  uint16_t start_handle = 0;
  uint16_t end_handle   = 0xffff;
  bool service_found = gatt_server_get_handle_range_for_service_with_uuid128((new UUID(DIST_GATT_SERVICE))->getUuid(), &start_handle, &end_handle);

  Serial.printf("Lookup service: %d [%u,%u]\n", service_found, start_handle, end_handle);
  if (service_found)
  {
    UUID distCharUuid(DIST_GATT_CHAR);

    uint16_t value_handle = gatt_server_get_value_handle_for_characteristic_with_uuid128(start_handle, end_handle, distCharUuid.getUuid());
    uint16_t client_handle = gatt_server_get_client_configuration_handle_for_characteristic_with_uuid128(start_handle, end_handle, distCharUuid.getUuid());
    Serial.printf("Char handles [%d,%d]\n", value_handle, client_handle);
  }

  gattDistanceService.start_handle = serviceHandle;
  gattDistanceService.end_handle = serviceHandle + 1;
  gattDistanceService.read_callback = &GattDistanceServiceReadCallback;
  gattDistanceService.write_callback = &GattDistanceServiceWriteCallback;

  measurement_value_handle = serviceHandle;
  measurement_client_configuration_descriptor_handle = serviceHandle + 1;

  Serial.printf("Register service [%u,%u] with char [%u, %u]\n", gattDistanceService.start_handle, gattDistanceService.end_handle,  measurement_value_handle, measurement_client_configuration_descriptor_handle);
  att_server_register_service_handler(&gattDistanceService);
}

void DistanceService::Disconnect()
{
    // We got a device disconnect. Reset all callbacks.
    measurement_client_configuration_descriptor_notify = 0;
}

void DistanceService::DistanceServiceUpdateValue(uint16_t distanceInMm)
{
    if (reportWithCmResolution)
    {
        distanceInMm = ((distanceInMm + 5)/10)*10;
    }

    uint16_t oldValue = distanceValue;
    distanceValue = distanceInMm;
    valueUpdated = oldValue != distanceInMm;
}
  
void DistanceService::DistanceServiceNotify(unsigned long currentMillis)
{
    if (currentMillis < 0)
    {
        currentMillis = millis();
    }

    bool notify = false;
    if (valueUpdated)
    {
        // 4 Hz if updated
        if (currentMillis - prevNotifyMillis >= 200)
        {
            notify = true;
        }
    }
    else
    {
        // 1Hz if not updated
        if (currentMillis - prevNotifyMillis >= 1000)
        {
            notify = true;
        }
    }

    if (notify)
    {
        prevNotifyMillis = currentMillis;
        if (measurement_client_configuration_descriptor_notify)
        {
            digitalWrite(LED_BUILTIN, HIGH);

            measurement_callback.callback = &DistanceServiceCanSendNow;
            measurement_callback.context = (void *)this;
            att_server_register_can_send_now_callback(&measurement_callback, con_handle);
        }
        else
        {
#if PRINT_DIST_NOTIFY_DISCONECTED
            Serial.printf("Nothing connected to BLE, distance: %u\n", distanceValue);
            valueUpdated = false;
            digitalWrite(LED_BUILTIN, LOW);
#endif
        }
    }
}

void DistanceService::DistanceServiceCanSendNow(void * context){
    DistanceService *instance = (DistanceService *) context;
    
    uint8_t value[10];
    int pos = 0;

    little_endian_store_16(value, pos, instance->distanceValue);
    pos += 2;

    uint8_t status = att_server_notify(instance->con_handle, instance->measurement_value_handle, &value[0], pos);
    if (status != ERROR_CODE_SUCCESS)
    {
      Serial.printf("Distance notify failed with: %u\n", status);
    }
    else
    {
      digitalWrite(LED_BUILTIN, LOW);
      instance->valueUpdated = false;
      //Serial.printf("Distance notify: %u\n", instance->distanceValue);
    }
}

uint16_t DistanceService::GattDistanceServiceReadCallback(hci_con_handle_t con_handle, uint16_t attribute_handle, uint16_t offset, uint8_t * buffer, uint16_t buffer_size)
{
    UNUSED(con_handle);
    DistanceService *instance = &distanceService;
    
    if (attribute_handle == distanceService.measurement_client_configuration_descriptor_handle){
        if (buffer && (buffer_size >= offset + 2u)){
            little_endian_store_16(buffer, offset, instance->measurement_client_configuration_descriptor_notify);
        } 
        return 2;
    }
    
    if (attribute_handle == instance->measurement_value_handle){
        if (buffer && (buffer_size >= 2u)){
          uint16_t pos = offset; 
          little_endian_store_16(buffer, pos, instance->distanceValue);
          pos += 2;
        }

        return 2;
    }

    return 0;
}

int DistanceService::GattDistanceServiceWriteCallback(hci_con_handle_t con_handle, uint16_t attribute_handle, uint16_t transaction_mode, uint16_t offset, uint8_t *buffer, uint16_t buffer_size)
{
    UNUSED(offset);
    UNUSED(buffer_size);

    DistanceService *instance = &distanceService;

    if (transaction_mode != ATT_TRANSACTION_MODE_NONE){
        return ATT_ERROR_SUCCESS;
    }

    if (attribute_handle == instance->measurement_client_configuration_descriptor_handle){
        if (buffer_size < 2u){
            return ATT_ERROR_INVALID_OFFSET;
        }

        instance->measurement_client_configuration_descriptor_notify = little_endian_read_16(buffer, 0);
        instance->con_handle = con_handle;
        return ATT_ERROR_SUCCESS;
    }

    return ATT_ERROR_SUCCESS;
}
