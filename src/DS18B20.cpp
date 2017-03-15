
#include "DS18B20.h"

// #define DEBUG_DS18B20

#define ASCII_0 48  // char(48)

/*
 * Digital Temperature Sensor DS18B20 commands (see sensor data sheet)
 */
#define CMD_CONVERT_TEMP     0x44  // write(0x44, 1)  // 1 = keep line high during conversion
#define CMD_READ_SCRATCHPAD  0xBE  // write(0xBE)


/*
 * Configuration-byte position in data vector returned by the DS18B20
 */
#define DS18B20_DATA_CONFIG_BYTE  4

uint8_t DS18B20_Controller::setupSensors() {
  for(uint8_t i=0; i<numSensors; i++) {
    if(sensors[i]->idUndefined()) {
      sensors[i]->sensorStatus = DS18B20_SENSOR_ID_UNDEFINED;
      #ifdef DEBUG_DS18B20
        Serial.print(F("DEBUG_DS18B20: Sensor "));
        Serial.print(sensors[i]->label);
        Serial.println(F(": setting to status ID undefined"));
      #endif
    }
  }
  
  oneWire->reset();
  uint8_t addr[DS18B20_SENSOR_ID_BYTES];
  uint8_t matchedAddressCount = 0;
  
  while(oneWire->search(addr)) { 
    if (OneWire::crc8(addr, DS18B20_SENSOR_ID_BYTES-1) != addr[DS18B20_SENSOR_ID_BYTES-1]) {
      continue;
    }
    
    DS18B20_Sensor *sensor = getSensor(addr);
    if (sensor != NULL) {
        sensor->sensorStatus = DS18B20_SENSOR_OK;
        matchedAddressCount++;
    
    } else {
      sensor = getFirstUndefinedSensor();
      if (sensor != NULL) {
        memcpy(sensor->id, addr, DS18B20_SENSOR_ID_BYTES);
        sensor->sensorStatus = DS18B20_SENSOR_ID_AUTO_ASSIGNED;
        matchedAddressCount++;
      }
    }
  }
  oneWire->reset_search();

  //
  // CONSIDER: set sensor resolution
  //
  return matchedAddressCount;
}

void DS18B20_Controller::initSensorReadout() {
  oneWire->reset();
  // Talk to all slaves on bus:
  oneWire->skip();
  // Start temp readout and conversion to scratchpad, with parasite power on at the end
  oneWire->write(CMD_CONVERT_TEMP, 1);
  //
  // Important: do not oneWire->reset() now for at least 750 ms (for 12-bit temperature resolution)
  //
}


void DS18B20_Controller::completeSensorReadout() {
  // initialise OK sensors to state NOK and leave other states untouched:
  for(uint8_t i=0; i<numSensors; i++) {
    if(sensors[i]->sensorStatus == DS18B20_SENSOR_OK) {
      sensors[i]->sensorStatus = DS18B20_SENSOR_NOK;
    }
    sensors[i]->currentTemp  = AACF_UNDEFINED_TEMPERATURE;
  }
  
  uint8_t addr[DS18B20_SENSOR_ID_BYTES];
  
  while(oneWire->search(addr)) {  
    if (OneWire::crc8(addr, DS18B20_SENSOR_ID_BYTES-1) != addr[DS18B20_SENSOR_ID_BYTES-1]) {
      continue;
    }
    
    uint8_t data[DS18B20_SENSOR_READOUT_BYTES];
    if (! readSensorScratchpad(addr, data)) {
      continue;
    }
    
    DS18B20_Sensor *sensor = getSensor(addr);
    if (sensor != NULL) {
      DS18B20_Readout readout = getCelcius(data);
      #ifdef DEBUG_DS18B20
        Serial.print(F("DEBUG_DS18B20: Sensor "));
        Serial.print(sensor->label);
      #endif
      
      // only set temperature and state for NOK sensors and AUTO_ASSIGNED sensors (leave others untouched):
      if (sensor->sensorStatus == DS18B20_SENSOR_ID_AUTO_ASSIGNED) {
        sensor->currentTemp  = readout.celcius;
        #ifdef DEBUG_DS18B20
          char buf[MAX_DS18B20_SENSOR_ID_STR_LEN];
          Serial.print(F(": ID AUTO ASSIGNED, "));
          Serial.println(formatTemperature(readout.celcius, buf));
        #endif
        
      } else if (sensor->sensorStatus == DS18B20_SENSOR_NOK) {
        // Ensure temperature is plausible:
        if((sensor->rangeMin == ACF_UNDEFINED_TEMPERATURE || readout.celcius >= sensor->rangeMin) 
            && (sensor->rangeMax == ACF_UNDEFINED_TEMPERATURE || readout.celcius <= sensor->rangeMax)) {
          sensor->sensorStatus = DS18B20_SENSOR_OK;
          sensor->currentTemp  = readout.celcius;
          #ifdef DEBUG_DS18B20
            char buf[MAX_DS18B20_SENSOR_ID_STR_LEN];
            Serial.print(F(": OK, "));
            Serial.println(formatTemperature(readout.celcius, buf));
          #endif
        } else {
          #ifdef DEBUG_DS18B20
            Serial.println(F(": NOK"));
          #endif
        }
      }
    } 
  }
  oneWire->reset_search();
}


DS18B20_Sensor *DS18B20_Controller::getSensor(uint8_t addr[]) {
  for(uint8_t i=0; i<numSensors; i++) {
    if (! memcmp(addr, sensors[i]->id, DS18B20_SENSOR_ID_BYTES)) {
      return sensors[i];
    }
  }
  return NULL;
}

DS18B20_Sensor *DS18B20_Controller::getFirstUndefinedSensor() {
  for(uint8_t i=0; i<numSensors; i++) {
    if(sensors[i]->idUndefined()) {
      return sensors[i];
    }
  }
  return NULL;
}

/*
 * @param addr  4-byte ID of DS18B20 temperature sensor
 * @param data  data vector returned by the DS18B20: 8 byte data + 1 byte CRC
 */
boolean DS18B20_Controller::readSensorScratchpad(uint8_t addr[], uint8_t *data) {
  oneWire->reset();
  // Talk only to sensor with 'addr':
  oneWire->select(addr);
  oneWire->write(CMD_READ_SCRATCHPAD);
  
  // Read 8 byte of data + 1 byte of CRC
  for (uint8_t i = 0; i < DS18B20_SENSOR_READOUT_BYTES; i++) {           
    data[i] = oneWire->read();
  }
  oneWire->reset();
  return OneWire::crc8(data, DS18B20_SENSOR_READOUT_BYTES-1) == data[DS18B20_SENSOR_READOUT_BYTES-1];
}

/*
 * Parse temperature [°C] from raw DS18B20 data
 * @param data  data vector returned by the DS18B20: 8 byte data + 1 byte CRC
 */
DS18B20_Readout DS18B20_Controller::getCelcius(uint8_t data[]) {
  int16_t raw = (data[1] << 8) | data[0];
  #ifdef DEBUG_DS18B20
    Serial.print(F("DEBUG_DS18B20: read raw temp = "));
    Serial.println(raw);
  #endif
  uint8_t configByte = (data[DS18B20_DATA_CONFIG_BYTE] & 0x60);
  
  DS18B20_Readout temp;
  // At lower resolution, the low bits are undefined, so let's zero them
  if (configByte == 0x00) {
    raw = raw & ~7;  // 9 bit resolution, 93.75 ms
    temp.resolution = 9;
  } else if (configByte == 0x20) {
    raw = raw & ~3; // 10 bit res, 187.5 ms
    temp.resolution = 10;
  } else if (configByte == 0x40) {
    raw = raw & ~1; // 11 bit res, 375 ms
    temp.resolution = 11;
  } else /* configByte ==  0x60 */ {
    temp.resolution = 12;
    // default is 12 bit resolution, 750 ms conversion time
  }
  temp.celcius = (int16_t)(raw * 100.0 / 16.0);  // [°C * 100]
  return temp;
}


char *formatTemperature(ACF_Temperature t, char s[MAX_TEMPERATURE_STR_LEN]) {
  uint8_t deg = t / 100;
  uint8_t frac = t % 100;
  s[8] = '\0';
  s[7] = 'C';
  s[6] = '\'';
  s[5] = ASCII_0 + frac % 10;
  s[4] = ASCII_0 + frac / 10;
  s[3] = '.';
  s[2] = ASCII_0 + deg % 10;
  s[1] = deg >= 10 ? ASCII_0 + deg / 10 : ' ';
  s[0] = t > 0 ? ' ' : '-';
  return s;
}

char *formatDS18B20_SensorID(DS18B20_SensorID id, char s[MAX_DS18B20_SENSOR_ID_STR_LEN]) {
  uint8_t pos = 0;
  for (uint8_t i=0; i<DS18B20_SENSOR_ID_BYTES; i++) {
    sprintf(&s[pos], "%02X", id[i]);
    pos += 2;
    if (i < DS18B20_SENSOR_ID_BYTES - 1) {
      s[pos++] = '-';
    }
  }
  s[pos++] = '\0';
  return s;
}


