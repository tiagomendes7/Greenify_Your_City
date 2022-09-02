
#include <LoRaWan.h>
#include <AHT20.h>

char buffer[256];
AHT20 TempHumSensors;

struct weatherSensors{
  uint16_t temp;
  uint16_t hum;
} sensor_readings; // Structure Variable


void setup(void)
{
    SerialUSB.begin(115200);
    //while(!SerialUSB);

    // Setup steps regarding the weather sensors 
    TempHumSensors.begin();

    // Setup steps regarding the LoRaWAN module
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
    
    while(!lora.setOTAAJoin(JOIN,20));
}



void loop(void)
{   
    // Reading weather sensors
    float t, h;
    int ret = TempHumSensors.getSensor(&h, &t);

    sensor_readings.temp = t * 100;
    sensor_readings.hum =  h * 10000;
   
    if(ret){     // GET DATA OK
        Serial.print("Humidity: ");
        Serial.print(h*100);
        Serial.print("%\t Temperature: ");
        Serial.println(t);
    }else{  // GET DATA FAIL
        Serial.println("GET DATA FROM AHT20 FAIL");
    }

    // Enviar dados por Lora 
    if( lora.transferPacket( (unsigned char*)&sensor_readings, sizeof(sensor_readings), 600) ){
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
