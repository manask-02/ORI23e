#include<Nextion.h>
#include <mcp_can.h>
#include <SPI.h>
#define INT8U unsigned char
#define INT32U unsigned char
#define MARK_BUTTON 5
#define SENS_POS_BUTTON 3
#define SENS_NEG_BUTTON 4
INT32U canId = 0x000;
MCP_CAN CAN(9);
int32_t SPEEDACT = 0;
int32_t RPMMAX = 0 ;
int32_t rpmr = 0;
int lv = 0;
int lvval = 0;
int Mark = 0;
unsigned long time_now = 0;
int p1 = 700;
int mark = 0;
int current_page = 0;
int current_page1 = 1;
INT8U len = 8 ;

void setup() {
  Serial.begin(9600);
  pinMode(MARK_BUTTON, INPUT);
  pinMode(SENS_POS_BUTTON, INPUT);
  pinMode(SENS_NEG_BUTTON, INPUT);
if(CAN.begin(CAN_500KBPS) ==CAN_OK) 
{
//  Serial.print("can init ok!!\r\n");
}
else {
//  Serial.print("Can init fail!!\r\n");
}
}

void loop() {
   int speeddecoded;
  // put your main code here, to run repeatedly:
 
canId = CAN.getCanId();
uint8_t buf[8] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
CAN.readMsgBuf(&len, buf);
//-------------------------------------RPM-----------------------------------------

      if(buf[0]==0x30)
      {
        uint16_t SPEEDACTRAW = ((unsigned int)buf[2]<<8) + buf[1];

        SPEEDACT = (int16_t)SPEEDACTRAW;

      }
      if(buf[0]==0xCE)
      {
        uint16_t RPMMAXRAW = ((unsigned int)buf[2]<<8) + buf[1];

        RPMMAX= (int16_t)RPMMAXRAW;
      
      
      }
 rpmr=RPMMAX*(SPEEDACT)*(-0.0000305);

      Serial.print("rpm.val=");
      Serial.print(69);
      Serial.write(0xff);
      Serial.write(0xff);
      Serial.write(0xff);

//---------------------------------LV BATT---------------------------------
      lv=analogRead(A0);
      lvval = map(lv, 0, 1023, 0, 139);
      Serial.print("lvv.val=");
      Serial.print(lvval);
      Serial.write(0xff);
      Serial.write(0xff);
      Serial.write(0xff);


//-----------------------------PAGE CHANGE------------------------

 if (digitalRead(MARK_BUTTON) != current_page)
    {
      current_page = digitalRead(MARK_BUTTON);
    }
    if (current_page == HIGH)
    { 
    time_now = millis();
    while (millis() < time_now + p1)
    {}
    (current_page1++);

    if (current_page1 >= 3)
    {current_page1 = 1;}

     if(current_page1 == 1)
    {
      Serial.print("page 0");
      Serial.write(0xff);
      Serial.write(0xff);
      Serial.write(0xff);  
    }
    else if(current_page1 == 2)
    {
      Serial.print("page 1");
      Serial.write(0xff);
      Serial.write(0xff);
      Serial.write(0xff);  
    }
}
      Serial.print("rpm.val=");
      Serial.print(69);
      Serial.write(0xff);
      Serial.write(0xff);
      Serial.write(0xff);      Serial.print("rpm.val=");
      Serial.print(69);
      Serial.write(0xff);
      Serial.write(0xff);
      Serial.write(0xff);
}
      
