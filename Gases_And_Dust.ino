#include <LoRaWan.h>
#include <Multichannel_Gas_GMXXX.h>
#include "Seeed_HM330X.h"
#include "TinyGPS++.h"
#include <Wire.h>

TinyGPSPlus GPS;

GAS_GMXXX<TwoWire> GasSensor;

HM330X DustSensor;

char buffer[256];

struct sensors{
  uint8_t Sat;
  uint8_t HDOP;
  uint32_t Lat;
  uint32_t Lng;
  uint16_t NO2;
  uint16_t C2H5OH;
  uint16_t VOC;
  uint16_t CO;
  uint16_t PM01;
  uint16_t PM02;
  uint16_t PM10;
} sensor_readings;       // Structure variable

void setup(void){
  SerialUSB.begin(115200);
  //while(!SerialUSB);

  
  //Setup steps regarding the GPS module (if present)
  Serial2.begin(9600);
  while(!GPS.location.isValid()){
    while(Serial2.available() > 0){
      if(GPS.encode(Serial2.read())){
        break;
      }
    }
 
    if(millis() > 15000 && GPS.charsProcessed() < 10){
      SerialUSB.println(F("No GPS detected: check wiring."));
      SerialUSB.println(GPS.charsProcessed());
      while(true);
    }else if(millis() > 60000){
      SerialUSB.println(F("Not able to get a fix in alloted time."));     
      break;
    }else{
      SerialUSB.print('.');
    }
  }
  
  //Setup steps regarding the gas sensor
  GasSensor.begin(Wire, 0x08);


  //Setup steps regarding the dust sensor
  if(DustSensor.init()){
    SerialUSB.println("HM330X init failed!!");
  }

  //Setup steps regarding the LoRaWAN module
  lora.init();
  
  memset(buffer, 0, 256);
  lora.getVersion(buffer, 256, 1);
  SerialUSB.print(buffer); 
  
  memset(buffer, 0, 256);
  lora.getId(buffer, 256, 1);
  SerialUSB.print(buffer);
  
  lora.setKey("2B7E151628AED2A6ABF7158809CF4F3C", "2B7E151628AED2A6ABF7158809CF4F3C", "2B7E151628AED2A6ABF7158809CF4F3C");
  
  lora.setDeciveMode(LWOTAA);
  lora.setDataRate(DR0, EU868);
  
  lora.setChannel(0, 868.1);
  lora.setChannel(1, 868.3);
  lora.setChannel(2, 868.5);
  
  lora.setReceiceWindowFirst(0, 868.1);
  lora.setReceiceWindowSecond(869.5, DR3);
  
  lora.setDutyCycle(false);
  lora.setJoinDutyCycle(false);
  
  lora.setPower(14);
  lora.setPort(2);
  
  while(!lora.setOTAAJoin(JOIN, 10));
}

void loop(void){
  //Reading GPS data
  unsigned long start = millis();
  do{
    while(Serial2.available() > 0){
      GPS.encode(Serial2.read());
    }
  }while(millis() - start < 1000);

  if(GPS.satellites.isValid()){
    sensor_readings.Sat = GPS.satellites.value();
  }
  if(GPS.hdop.isValid()){
    sensor_readings.HDOP = GPS.hdop.value();
  }
  if(GPS.location.isValid()){
    sensor_readings.Lat = GPS.location.lat()*1000000;
    sensor_readings.Lng = GPS.location.lng()*1000000;
  }
  if(GPS.altitude.isValid()){
    SerialUSB.printf("Altitude: %.1f ", GPS.altitude.meters());
  }else{
    SerialUSB.print("Altitude: INVALID ");
  }
  if(GPS.speed.isValid()){
    SerialUSB.printf("Speed: %.2f\n", GPS.speed.kmph());
  }else{
    SerialUSB.print("Speed: INVALID\n");
  }
  
  //Reading Multichannel gas sensor
  sensor_readings.NO2     = GasSensor.measure_NO2();
  sensor_readings.C2H5OH  = GasSensor.measure_C2H5OH();
  sensor_readings.VOC     = GasSensor.measure_VOC();
  sensor_readings.CO      = GasSensor.measure_CO();

  //Reading Dust Sensor
  uint8_t raw_data[30];
  if(DustSensor.read_sensor_value(raw_data, 29)){
    SerialUSB.println("HM330X read result failed!!");
  }else{
    sensor_readings.PM01 = (uint16_t) raw_data[10] << 8 | raw_data[11];
    sensor_readings.PM02 = (uint16_t) raw_data[12] << 8 | raw_data[13];
    sensor_readings.PM10 = (uint16_t) raw_data[14] << 8 | raw_data[15];
  }


  /*sensor_readings.Sat     = 0x01;
  sensor_readings.HDOP    = 0x02;
  sensor_readings.Lat     = 0x03040506;
  sensor_readings.Lng     = 0x0708090A;
  sensor_readings.NO2     = 0x0B0C;
  sensor_readings.C2H5OH  = 0x0D0E;
  sensor_readings.VOC     = 0x0F10;
  sensor_readings.CO      = 0x1112;
  sensor_readings.PM01    = 0x1314;
  sensor_readings.PM02    = 0x1516;
  sensor_readings.PM10    = 0x1718;*/

  
  SerialUSB.printf("Sat:%u ",     sensor_readings.Sat   );
  SerialUSB.printf("HDOP:%u ",    sensor_readings.HDOP  );
  SerialUSB.printf("Lat:%d ",     sensor_readings.Lat   );
  SerialUSB.printf("Lng:%d ",     sensor_readings.Lng   );
  SerialUSB.printf("NO2:%u ",     sensor_readings.NO2   );
  SerialUSB.printf("C2H5OH:%u ",  sensor_readings.C2H5OH);
  SerialUSB.printf("VOC:%u ",     sensor_readings.VOC   );
  SerialUSB.printf("CO:%u ",      sensor_readings.CO    );
  SerialUSB.printf("PM1.0:%u ",   sensor_readings.PM01  );
  SerialUSB.printf("PM2.5:%u ",   sensor_readings.PM02  );
  SerialUSB.printf("PM10:%u\n",   sensor_readings.PM10  );
  SerialUSB.printf("Payload size:%u\n",  sizeof(sensors));

  
  if(lora.transferPacket((unsigned char *)&sensor_readings, sizeof(sensor_readings), 10)){
    short length;
    short rssi;
    
    memset(buffer, 0, 256);
    length = lora.receivePacket(buffer, 256, &rssi);
    
    if(length){
      SerialUSB.printf("Length is: %d, RSSI is: %d, Data is: ", length, rssi);
      for(unsigned char i = 0; i < length; i ++){
        SerialUSB.print("0x");
        SerialUSB.print(buffer[i], HEX);
        SerialUSB.print(" ");
      }
      SerialUSB.println();
    }
  }
}
