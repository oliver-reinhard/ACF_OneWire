#ifndef CF_DS18B20_H_INCLUDED
  #define CF_DS18B20_H_INCLUDED
  
  #include <Arduino.h>
  #include <OneWire.h>
  
  typedef int16_t CF_Temperature;  // [°C * 100]
  const CF_Temperature CF_UNDEFINED_TEMPERATURE = INT16_MIN; // [°C * 100];

  const uint8_t DS18B20_SENSOR_ID_BYTES = 8;
  typedef uint8_t DS18B20_SensorID[DS18B20_SENSOR_ID_BYTES];
  const DS18B20_SensorID  DS18B20_UNDEFINED_SENSOR_ID = {0,0,0,0,0,0,0,0};
  
  /*
   * Number of bytes of data vector returned by the DS18B20: 8 byte data + 1 byte CRC
   */
  const uint8_t DS18B20_SENSOR_READOUT_BYTES = 9;

  /*
   * States of a DS18B20 Temperature Sensor.
   */
  typedef enum {
    DS18B20_SENSOR_INITIALISING = 0x1,
    DS18B20_SENSOR_ID_AUTO_ASSIGNED = 0x2, // a physical sensor was assigned to the logical sensor but it may not correspond to the intended sensor
    DS18B20_SENSOR_ID_UNDEFINED = 0x4,     // no physical sensor could be found for the logical sensor
    DS18B20_SENSOR_OK = 0x8,               // temperature value is defined and within the given range
    DS18B20_SENSOR_NOK = 0x10              // temperature value could not be obtained or is outside the given range
  } DS18B20_StatusEnum;

  typedef uint8_t DS18B20_StatusID;

  /*
   * DS18B20 Temperature Sensor.
   */
  class DS18B20_Sensor {
    public:
    
      DS18B20_Sensor(const char label[10]) {
        strncpy(this->label, label, sizeof(this->label)-1);
        setId(DS18B20_UNDEFINED_SENSOR_ID);
      }

      /* Human-readable label. */
      char label[10];
      /* The ROM sensor address of the corresponding physical temperature sensor. */
      DS18B20_SensorID id;
      /* If defined then a temperature below this lower temperature limit will lead to a sensor status of DS18B20_SENSOR_NOK. */
      CF_Temperature rangeMin = CF_UNDEFINED_TEMPERATURE;
      /* If defined then a temperature above this lower temperature limit will lead to a sensor status of DS18B20_SENSOR_NOK. */
      CF_Temperature rangeMax = CF_UNDEFINED_TEMPERATURE;
      /* The current state / quality of the temperature information of the assigned physical sensor. */
      DS18B20_StatusEnum sensorStatus = DS18B20_SENSOR_INITIALISING;
      /* Only valid if status is DS18B20_SENSOR_OK or DS18B20_SENSOR_ID_AUTO_ASSIGNED. */
      CF_Temperature currentTemp = CF_UNDEFINED_TEMPERATURE;
      /* Temperature value that was last recorded by the logger. */
      CF_Temperature lastLoggedTemp = CF_UNDEFINED_TEMPERATURE;
      /* Timestamp captured when the temperature value was last recorded by the logger. */
      uint32_t lastLoggedTime = 0L;

      /*
       * Set the address of the physical sensor this object corresponds with.
       */
      void setId(const DS18B20_SensorID id) {
        memcpy(this->id, id, DS18B20_SENSOR_ID_BYTES);
      }

      /*
       * Returns true if the ID of this sensor is DS18B20_UNDEFINED_SENSOR_ID, i.e. this object does represent a physical sensor.
       */
      boolean idUndefined() {
        return idUndefined(this->id);
      }

      /*
       * Returns true if the given is DS18B20_UNDEFINED_SENSOR_ID.
       */
      static boolean idUndefined(const DS18B20_SensorID id) {
        return ! memcmp(id, DS18B20_UNDEFINED_SENSOR_ID, DS18B20_SENSOR_ID_BYTES);
      }

      /*
       * Sets the sensor status to DS18B20_SENSOR_OK iff it is DS18B20_SENSOR_ID_AUTO_ASSIGNED.
       * 
       * @return true if status was changed.
       */
      boolean confirmId() {
        if (sensorStatus == DS18B20_SENSOR_ID_AUTO_ASSIGNED) {
          sensorStatus = DS18B20_SENSOR_OK;
          return true;
        }
        return false;
      }
  };
  
  struct DS18B20_Readout {
    uint8_t resolution;     // number of bits (= 9..12)
    CF_Temperature celcius; // [°C * 100]
  };
  

  class DS18B20_Controller {
    public:

      /*
       * @param oneWire cannot be NULL
       * @param sensors an array of sensors managed by this controller
       * @param numSensors the number of elements of the sensors array
       */
      DS18B20_Controller(OneWire *oneWire, DS18B20_Sensor **sensors, uint8_t numSensors) {
        this->oneWire = oneWire;
        this->sensors = sensors;
        this->numSensors = numSensors;
      }

      /*
       * Searches for temperature sensors on OneWire and matches the found addresses to the configured sensor IDs  
       * of this controller. Unmatched addresses will be assigned to sensors without an ID if there are any. 
       * 
       * The found but unmatched addresses are assigned to sensors without ID in sequential order: first address 
       * to first sensor, second address to second sensor, etc. If there are fewer unmatched addresses than configured 
       * sensors without ID then the sensors at the end are left without an ID. If there are more unmatched addresses  
       * than configured sensors without ID then the sensors with these addresses will be ignored until the next 
       * time setupSensors() is invoked.
       * 
       * Configured sensors with newly assigned addresses are set to DS18B20_SENSOR_ID_AUTO_ASSIGNED state. They remain in
       * this state until they are (manually or programatically) set to DS18B20_SENSOR_OK. Temperature is read from 
       * DS18B20_SENSOR_ID_AUTO_ASSIGNED sensors but temperature ranges are not checked.
       * 
       * @return the number of matched or assigned addresses.
       */
      uint8_t setupSensors();

      /*
       * Sends a command over the wire to all configured sensors to start converting the temperature.
       */
      void initSensorReadout();

      /*
       * Reads the temperaturs of those physical sensors that have a configured logical sensor and 
       * assignes status and temperature to the logical configured sensors.
       */
      void completeSensorReadout();
      
    protected:
      /* The wire (pin) this controller is connected to. */
      OneWire *oneWire;
      /* Configured sensors. */
      DS18B20_Sensor **sensors;
      /* Number of configured sensors. */
      uint8_t numSensors;

      /*
       * Returns the configured sensor whose ID matches the given address, or NULL if none mathces.
       * 
       * @param addr must be of length DS18B20_SENSOR_ID_BYTES
       */
      DS18B20_Sensor *getSensor(uint8_t addr[]);

      /*
       * Returns the first configured sensor whose ID is undefined so far (DS18B20_UNDEFINED_SENSOR_ID).
       */
      DS18B20_Sensor *getFirstUndefinedSensor();

      /*
       * Reads the data vector of the sensor with the given address.
       * 
       * @param addr must be of length DS18B20_SENSOR_ID_BYTES
       * @param data receives the result and must be of length DS18B20_SENSOR_READOUT_BYTES
       */
      boolean readSensorScratchpad(uint8_t addr[], uint8_t *data);

      /*
       * Converts sensor vector data to degrees celcius in the corresonding resolution.
       * 
       * @param data receives the result and must be of length DS18B20_SENSOR_READOUT_BYTES
       */
      DS18B20_Readout getCelcius(uint8_t data[]);
  };

  #define MAX_TEMPERATURE_STR_LEN 9
  
  /**
   * Returns the temperature in a dotted notation, 8 chars + terminal '\0': ["-"]dd.ff"°C" (optional "-", d = degrees Celsius (2 digits), f = fraction (1 digit))
   */
  char *formatTemperature(CF_Temperature t, char buf[MAX_TEMPERATURE_STR_LEN]);


  #define MAX_DS18B20_SENSOR_ID_STR_LEN (3*DS18B20_SENSOR_ID_BYTES)
  
  /*
   * Returns the ID in a dashed hex notation, e.g."28-8C-8C-79-6-00-00-89"
   */
  char *formatDS18B20_SensorID(DS18B20_SensorID id, char buf[MAX_DS18B20_SENSOR_ID_STR_LEN]);

#endif
