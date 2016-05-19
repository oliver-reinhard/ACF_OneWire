#include <OneWire.h>

// comment this line to suppress printing raw value
//#define PRINT_RAW_SENSOR_VALUES

#define ONE_WIRE_PIN 10

/*
 * Temperature sensor resolution in bits. Range: 9..12
 */
const byte NEW_SENSOR_RESOLUTION = 11;

/*
 * Digital Temperature Sensor DS18B20 commands (see sensor data sheet)
 */
const byte CMD_CONVERT_TEMP     = 0x44;  // write(0x44, 1)  // 1 = keep line high during conversion
const byte CMD_RECALL_E2        = 0xB8;  // write(0xB8)
const byte CMD_READ_SCRATCHPAD  = 0xBE;  // write(0xBE)
const byte CMD_WRITE_SCRATCHPAD = 0x4E;  // must always write 3 Byte data: byte data[4]; data[0] = 0x4E; ...
                                         // write_bytes(data, 4)
const byte CMD_COPY_SCRATCHPAD  = 0x48;  // write(0x48)

/*
 * The first byte (0X28) is the chip type:
 * - 0x10  DS18S20 (or old DS1820)
 * - 0x28  new DS18B20
 * - 0x22  new DS122
 */
const int ID_LENGTH = 8;
const byte waterTempSensorId[ID_LENGTH]   = {0x28, 0x8C, 0x8C, 0x79, 0x06, 0x00, 0x00, 0x89};
const byte ambientTempSensorId[ID_LENGTH] = {0x28, 0x7C, 0x28, 0x79, 0x06, 0x00, 0x00, 0xD7};


/*
 * Number of bytes of the data vector (= temperature)
 */
const byte DATA_LENGTH = 9;
const byte DATA_CONFIG_BYTE = 4;

const byte WRITE_BUF_LENGTH = 4; 

struct Temperature {
  byte resolution; // bits (= 9..12)
  float celcius;
};


OneWire  ds(ONE_WIRE_PIN);  // on pin 10 (a 4.7K pull-up resistor to 5V is necessary)
boolean firstRun = true;

void setup() {
  Serial.begin(9600);
  
  byte addr[ID_LENGTH];
  while(ds.search(addr)) {
    Serial.print("Detected Sensor ");
    printAddr(addr);
  
    if (OneWire::crc8(addr, ID_LENGTH-1) != addr[ID_LENGTH-1]) {
        Serial.println(" --> CRC is not valid!");
        continue;
    }

    if (isWaterSensor(addr)) {
      Serial.print(" --> Water Sensor");
    } else if (isAmbientSensor(addr)) {
      Serial.print(" --> Ambient Sensor");
    } else {
      Serial.print(" --> UNKNOWN Sensor");
    }

    //
    // Read CONFIG => resolution
    //
    byte resolution;
    if (! readResolution(addr, &resolution)) {
      continue;
    }
    Serial.print(", resolution: ");
    Serial.print(resolution);
    
    Serial.println();

    //
    // Update resolution (if needed)
    //
    if (resolution != NEW_SENSOR_RESOLUTION) {
      Serial.print("Changing resolution to ");
      Serial.println(NEW_SENSOR_RESOLUTION);
      if (! writeResolution(addr, NEW_SENSOR_RESOLUTION)) {
        Serial.println("Writing new resolution failed.");
        continue;
      }
      byte resolution2;
      if (! readResolution(addr, &resolution2)) {
        continue;
      }
      Serial.print("New resolution: ");
      Serial.println(resolution2);
    }
  }
  
  Serial.println("No more addresses.");
  Serial.println();
  ds.reset_search();
  delay(1000);
}

void loop() {
  byte addr[ID_LENGTH];
  
  while(ds.search(addr)) {  
    if (OneWire::crc8(addr, ID_LENGTH-1) != addr[ID_LENGTH-1]) {
      Serial.print("Sensor ID ");
      printAddr(addr);
      Serial.println(" --> CRC of ID is not valid!");
      continue;
    }
    
    ds.reset();
    // Talk to sensor with 'addr' only:
    ds.select(addr);
    // Start temp readout and conversion to scratchpad, with parasite power on at the end
    ds.write(CMD_CONVERT_TEMP, 1);        
    delay(800);     // 12 bit resolution reauires 750ms  
    
    byte data[DATA_LENGTH];
    if (! readScratchpad(addr, &data[0])) {
      continue;
    }
    
    Temperature temp = getCelcius(data);

    if (isWaterSensor(addr)) {
      Serial.print("Water:   ");
    } else if (isAmbientSensor(addr)) {
      Serial.print("Ambient: ");
    } else {
      Serial.print("UNKNOWN Sensor ");
      printAddr(addr);
      Serial.print(": ");
    }
    Serial.print(temp.celcius);
    Serial.print("°C");

    if (firstRun) {
      Serial.print(", resolution: ");
      Serial.print(temp.resolution);
      Serial.print(" bits");
    }
    Serial.println();
    
    #ifdef PRINT_RAW_SENSOR_VALUES
      Serial.print("Raw data: ");
      printRawData(data);
      Serial.println();
    #endif      
  }
  
  ds.reset_search();
  
  if (firstRun) {
    Serial.println();
    Serial.println("(wait 5 sec)");
  }
  delay(5000);
  Serial.println();
  firstRun = false;
}

boolean readScratchpad(byte addr[], byte *data) {
  // byte present = ds.reset();  // returns number of slaves
  ds.reset();
  // Talk to sensor with 'addr' only:
  ds.select(addr);
  ds.write(CMD_READ_SCRATCHPAD);
  
  // Read 8 byte of data + 1 byte of CRC
  for (byte i = 0; i < DATA_LENGTH; i++) {           
    data[i] = ds.read();
  }
  ds.reset();
  
  if (OneWire::crc8(data, DATA_LENGTH-1) != data[DATA_LENGTH-1]) {
    Serial.print("Sensor ID ");
    printAddr(addr);
    Serial.println(" --> CRC of data is not valid!");
    Serial.print("Raw data: ");
    printRawData(data);
    Serial.println();
    return false;
  }
  return true;
}

Temperature getCelcius(byte data[]) {
  int16_t raw = (data[1] << 8) | data[0];
  byte cfg = (data[4] & 0x60);
  Temperature temp;
  
  // At lower resolution, the low bits are undefined, so let's zero them
  if (cfg == 0x00) {
    raw = raw & ~7;  // 9 bit resolution, 93.75 ms
    temp.resolution = 9;
  } else if (cfg == 0x20) {
    raw = raw & ~3; // 10 bit res, 187.5 ms
    temp.resolution = 10;
  } else if (cfg == 0x40) {
    raw = raw & ~1; // 11 bit res, 375 ms
    temp.resolution = 11;
  } else {
    temp.resolution = 12;
    // default is 12 bit resolution, 750 ms conversion time
  }
  temp.celcius = (float)raw / 16.0;
  return temp;
}

boolean readResolution(byte addr[], byte *resolution) {
  byte data[DATA_LENGTH];
  if (! readScratchpad(addr, &data[0])) {
    return false;
  }
  // see specification for config byte: resolution is in bits 5, 6
  *resolution = byte(9 + (data[DATA_CONFIG_BYTE] >> 5));
  return true;
}

boolean writeResolution(byte addr[], byte resolution) {
  if (resolution < 9 || resolution > 12) {
    Serial.print("Invalid resolution: ");
    Serial.println(resolution);
    return false;
  }
  ds.reset();
  // Talk to sensor with 'addr' only:
  ds.select(addr);
  // see specification for config byte: resolution is in bits 5+6, with bits 0..4 = don't care
  byte config = (resolution - 9) << 5;
  byte buf[WRITE_BUF_LENGTH] = { CMD_WRITE_SCRATCHPAD, 0, 0, config};
  ds.write_bytes(buf, WRITE_BUF_LENGTH);
  ds.reset();
  return true;
}

void printAddr(byte addr[]) {
  for(byte i = 0; i < ID_LENGTH; i++) {
    if (addr[i] < 16) {
       Serial.print('0');
    }
    Serial.print(addr[i], HEX);
    if (i < ID_LENGTH - 1) {
      Serial.write('-');
    }
  }
}

void printRawData(byte data[]) {
  for(byte i = 0; i < DATA_LENGTH-1; i++) {
    Serial.write(' ');
    Serial.print(data[i], HEX);
  }
  Serial.print(" CRC=");
  Serial.print(OneWire::crc8(data, DATA_LENGTH-1), HEX);
}

boolean isWaterSensor(byte addr[]) {
  return ! memcmp(addr, &waterTempSensorId, ID_LENGTH);
}

boolean isAmbientSensor(byte addr[]) {
  return ! memcmp(addr, &ambientTempSensorId, ID_LENGTH);
}

