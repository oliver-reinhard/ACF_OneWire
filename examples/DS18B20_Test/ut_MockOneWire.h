#ifndef UT_MOCKONEWIRE_H_INCLUDED
  #define UT_MOCKONEWIRE_H_INCLUDED
  
  #include <OneWire.h>

 // #define DEBUG_MOCK_ONE_WIRE
  
  #define DS18B20_SENSOR_ID_BYTES 8
  #define DS18B20_SENSOR_READOUT_BYTES 9
  #define MAX_PRECONFIGURED_TEMPERATURES 10

  class MockOneWire : public OneWire{
    public:
      MockOneWire(uint8_t pin) : OneWire(pin) { }
 
      void _setSearchResults(uint8_t *addr, uint8_t len) {
        searchResult = addr;
        searchResultLen = len;
        searchResultIndex = 0;
        #ifdef DEBUG_MOCK_ONE_WIRE
          Serial.print(F("DEBUG_MOCK_ONE_WIRE: _setSearchResults: "));
          for(uint8_t i=0; i<len; i++) {
              Serial.print(formatDS18B20_SensorID(&searchResult[i * DS18B20_SENSOR_ID_BYTES]));
              Serial.print(", ");
          }
          Serial.println();
        #endif
      }
      

      /* 
       *  Override.
       *  NOTE: Ensure that the declaration in OneWire.h has "virtual" inserted.
       */
      uint8_t search(uint8_t *newAddr, bool) {
        if (searchResultIndex >= searchResultLen) {
          return 0;
        }
        uint8_t *sensorAddr = &searchResult[searchResultIndex * DS18B20_SENSOR_ID_BYTES];
        searchResultIndex++;
        memcpy(newAddr, sensorAddr, DS18B20_SENSOR_ID_BYTES);
        #ifdef DEBUG_MOCK_ONE_WIRE
          Serial.print(F("DEBUG_MOCK_ONE_WIRE: search returns "));
          Serial.println(formatDS18B20_SensorID(newAddr));
        #endif
        return 1;
      }
      
      
      /* 
       *  Override.
       *  NOTE: Ensure that the declaration in OneWire.h has "virtual" inserted.
       */
      void reset_search() {
        searchResultIndex = 0;
      }

      void _setReadResults(const int16_t *temperature100, uint8_t len) {
        readResultLen = min(len, MAX_PRECONFIGURED_TEMPERATURES);
        for(uint8_t i=0; i<readResultLen; i++) {
          putTemperature(&readResult[i * DS18B20_SENSOR_READOUT_BYTES], temperature100[i]);
        }
        readIndex = 0;
      }
      
      
      /* 
       *  Override.
       *  NOTE: Ensure that the declaration in OneWire.h has "virtual" inserted.
       */
      uint8_t read(void) { 
        if (readIndex >= readResultLen * DS18B20_SENSOR_READOUT_BYTES) {
          return 0;
        }
        return readResult[readIndex++];
      }
      
    private:
      uint8_t *searchResult;
      uint8_t searchResultLen = 0;
      uint8_t searchResultIndex = 0;
      uint8_t readResult[MAX_PRECONFIGURED_TEMPERATURES * DS18B20_SENSOR_READOUT_BYTES];
      uint8_t readResultLen = 0;
      uint8_t readIndex = 0;

      void putTemperature(uint8_t *buf, int16_t temperature100) {
        int16_t raw = (int16_t) (temperature100 * 16.0 / 100.0);
        #ifdef DEBUG_MOCK_ONE_WIRE
          Serial.print(F("DEBUG_MOCK_ONE_WIRE: _setReadResults: raw temp = "));
          Serial.println(raw);
        #endif
        uint8_t *p = buf;
        *p++ = (uint8_t) (raw & 0xFF);
        *p++ = (uint8_t) (raw >> 8);
        *p++ = 0x00;
        *p++ = 0x00;
        *p++ = 0x60; // 4 = config byte => 12-bit resolution
        *p++ = 0x00;
        *p++ = 0x00;
        *p++ = 0x00;
        *p++ = OneWire::crc8(buf, DS18B20_SENSOR_READOUT_BYTES-1);
      }
      
      String formatDS18B20_SensorID(uint8_t *id) {
        char s[3*DS18B20_SENSOR_ID_BYTES];
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
  };
  
#endif
