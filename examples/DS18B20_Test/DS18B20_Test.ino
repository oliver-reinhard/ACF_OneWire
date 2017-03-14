#include <ArduinoUnitX.h>
#include <DS18B20.h>
#include "ut_MockOneWire.h"

/* 
 *  The tests in this file can be ran against two physical DS18B20 temperature sensors on the same OneWire
 *  or the can be used with a mocked OneWire object and simulated DS18B20 temperature sensors.
 *  
 *  ******* Note: for the mocking to work, the original library file OneWire.h needs to be modified to enable overriding
 *  *******       and dynamic dispatching by introducing the "virtual" keyword:
 *  *******
 *  *******       class OneWire:
 *  *******
 *  *******       - virtual uint8_t read(void);
 *  *******       - virtual void reset_search();
 *  *******       - virtual uint8_t search(uint8_t *newAddr, bool search_mode = true);
 */
// #define MOCK_ONE_WIRE

//#define DEBUG_DS18B20_MAIN

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect.
  }
  //Test::min_verbosity = TEST_VERBOSITY_ALL;
  
  #if defined MOCK_ONE_WIRE
    Serial.println(F("Unit testing using MOCKING: MockOneWire\n"));
  #else
    Serial.println(F("Unit testing USING REAL SENSORS -- make sure they are wired properly.\n"));
  #endif
}

void loop() {
  Test::run();
}

// ------ Unit Tests --------

#define ONE_WIRE_PIN 10

#if defined MOCK_ONE_WIRE
  MockOneWire oneWire = MockOneWire(ONE_WIRE_PIN);
  /* 
   *  The IDs do not have to match phyisical sensor ROM addresses, however, the last byte has to be a crc8
   *  of the previous 7 bytes.
   */
  const uint8_t MOCK_SENSOR_IDS[][DS18B20_SENSOR_ID_BYTES] = { {0x28, 0x8C, 0x8C, 0x79, 0x06, 0x00, 0x00, 0x89}, 
                                                     {0x28, 0x7C, 0x28, 0x79, 0x06, 0x00, 0x00, 0xD7} };


#else
  OneWire oneWire = OneWire(ONE_WIRE_PIN);  // on pin 10 (a 4.7K pull-up resistor to +5V is necessary)
#endif

test(a_sensor_setup) {
  DS18B20_Sensor s1 = DS18B20_Sensor("S1");
  assertEqual(s1.sensorStatus, DS18B20_SENSOR_INITIALISING);
  
  DS18B20_Sensor s2 = DS18B20_Sensor("S2");
  DS18B20_Sensor s3 = DS18B20_Sensor("S3");
  
  DS18B20_Sensor *sensors[] = {&s1, &s2, &s3};
  DS18B20_Controller controller = DS18B20_Controller(&oneWire, sensors, 3);

  #ifdef MOCK_ONE_WIRE
    oneWire._setSearchResults((uint8_t *) MOCK_SENSOR_IDS, 2);
  #endif
  uint8_t matched = controller.setupSensors();
  #ifndef MOCK_ONE_WIRE
    if (matched < 2) {
      Serial.println(F("Cannot access one or more physical sensors -> check wiring."));
    }
  #endif
  assertEqual(matched, 2);
  assertEqual(s1.sensorStatus, DS18B20_SENSOR_ID_AUTO_ASSIGNED);
  assertEqual(s2.sensorStatus, DS18B20_SENSOR_ID_AUTO_ASSIGNED);
  assertEqual(s3.sensorStatus, DS18B20_SENSOR_ID_UNDEFINED);
  #ifdef MOCK_ONE_WIRE
    // check that the sensor IDs were "recognized" (mocking) correctly:
    assertTrue(! memcmp(s1.id, MOCK_SENSOR_IDS[0], DS18B20_SENSOR_ID_BYTES));
    assertTrue(! memcmp(s2.id, MOCK_SENSOR_IDS[1], DS18B20_SENSOR_ID_BYTES));
  #else
    assertTrue(! s1.idUndefined());
    assertTrue(! s2.idUndefined());
  #endif
  assertTrue(s3.idUndefined());
}


test(b_single_sensor_readout) {
  DS18B20_Sensor s1 = DS18B20_Sensor("S1");
  s1.rangeMin = -200; // -2°C
  s1.rangeMax = 6000; // 60°C
  
  DS18B20_Sensor *sensors[] = {&s1};
  DS18B20_Controller controller = DS18B20_Controller(&oneWire, sensors, 1);
  
  #ifdef MOCK_ONE_WIRE
    oneWire._setSearchResults((uint8_t *) MOCK_SENSOR_IDS, 1);
  #endif
  uint8_t matched = controller.setupSensors();
  assertEqual(matched, 1);
  assertEqual(s1.sensorStatus, DS18B20_SENSOR_ID_AUTO_ASSIGNED);
  // manually confirm auto-assigned ID:
  s1.confirmId();
  
  #ifdef MOCK_ONE_WIRE
    const int16_t temperatures100_1[] = {2300};
    oneWire._setReadResults(temperatures100_1, 1);
  #endif
  
  controller.initSensorReadout();
  #ifndef MOCK_ONE_WIRE
    delay(800); // 12-bit sensor readout takes 750 ms
  #endif
  controller.completeSensorReadout();
  assertEqual(s1.sensorStatus, DS18B20_SENSOR_OK);
  
  #ifdef MOCK_ONE_WIRE
    assertEqual(s1.currentTemp, 2300);
  #else
    char buf[MAX_DS18B20_SENSOR_ID_STR_LEN];
    Serial.print(F("Temperature readout S1: "));
    Serial.println(formatTemperature(s1.currentTemp, buf));
    // assumed room temperature between 0 and 40°C:
    assertTrue(s1.currentTemp > 0);
    assertTrue(s1.currentTemp < 4000);
  #endif
  
  #ifdef MOCK_ONE_WIRE
    const int16_t temperatures100_2[] = {-1200, 2400, 6500};
    oneWire._setReadResults(temperatures100_2, 3);
    
    controller.initSensorReadout();
    controller.completeSensorReadout();
    assertEqual(s1.sensorStatus, DS18B20_SENSOR_NOK);
    
    controller.initSensorReadout();
    controller.completeSensorReadout();
    assertEqual(s1.currentTemp, 2400);
    assertEqual(s1.sensorStatus, DS18B20_SENSOR_OK);
    
    controller.initSensorReadout();
    controller.completeSensorReadout();
    assertEqual(s1.sensorStatus, DS18B20_SENSOR_NOK);
  #endif
}


test(c_twin_sensor_readout) {
  DS18B20_Sensor s1 = DS18B20_Sensor("S1");
  DS18B20_Sensor s2 = DS18B20_Sensor("S2");
  s2.rangeMin = -200; // -2°C
  s2.rangeMax = 6000; // 60°C
  
  DS18B20_Sensor *sensors[] = {&s1, &s2};
  DS18B20_Controller controller = DS18B20_Controller(&oneWire, sensors, 2);
  
  #ifdef MOCK_ONE_WIRE
    oneWire._setSearchResults((uint8_t *) MOCK_SENSOR_IDS, 2);
  #endif
  uint8_t matched = controller.setupSensors();
  assertEqual(matched, 2);
  assertEqual(s1.sensorStatus, DS18B20_SENSOR_ID_AUTO_ASSIGNED);
  // manually confirm auto-assigned ID for s1 but not for s2:
  s1.confirmId();
  
  #ifdef MOCK_ONE_WIRE
    const int16_t temperatures100_1[] = {2300, 2400};
    oneWire._setReadResults(temperatures100_1, 2);
  #endif
  
  controller.initSensorReadout();
  #ifndef MOCK_ONE_WIRE
    delay(800); // 12-bit sensor readout takes 750 ms
  #endif
  controller.completeSensorReadout();
  assertEqual(s1.sensorStatus, DS18B20_SENSOR_OK);
  assertEqual(s2.sensorStatus, DS18B20_SENSOR_ID_AUTO_ASSIGNED);
  
  #ifdef MOCK_ONE_WIRE
    assertEqual(s1.currentTemp, 2300);
    assertEqual(s2.currentTemp, 2400);
  #else
    char buf[MAX_DS18B20_SENSOR_ID_STR_LEN];
    Serial.print(F("Temperature readout S1: "));
    Serial.println(formatTemperature(s1.currentTemp, buf));
    // assumed room temperature between 0 and 40°C:
    assertTrue(s1.currentTemp > 0);
    assertTrue(s1.currentTemp < 4000);
    Serial.print(F("Temperature readout S2: "));
    Serial.println(formatTemperature(s2.currentTemp, buf));
    // assumed room temperature between 0 and 40°C:
    assertTrue(s2.currentTemp > 0);
    assertTrue(s2.currentTemp < 4000);
  #endif
  
  #ifdef MOCK_ONE_WIRE
    const int16_t temperatures100_2[] = {2500, 10000};  // S2: 100°C but range not checked for DS18B20_SENSOR_ID_AUTO_ASSIGNED
    oneWire._setReadResults(temperatures100_2, 2);
    
    controller.initSensorReadout();
    controller.completeSensorReadout();
    assertEqual(s1.sensorStatus, DS18B20_SENSOR_OK);
    assertEqual(s2.sensorStatus, DS18B20_SENSOR_ID_AUTO_ASSIGNED);
    assertEqual(s1.currentTemp, 2500);
    assertEqual(s2.currentTemp, 10000);
  #endif
}

