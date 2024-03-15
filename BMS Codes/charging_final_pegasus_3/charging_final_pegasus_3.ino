#include <Arduino.h>
#include <stdint.h>
#include "Linduino.h"
#include "LT_SPI.h"
#include "UserInterface.h"
#include "LTC681x.h"
#include "LTC6813.h"
#include <SPI.h>
#include <Wire.h>
#include "mcp_can.h"
#define ENABLED 1
#define DISABLED 0
#define DATALOG_ENABLED 1
#define DATALOG_DISABLED 0
#define AMS_FAULT_PIN 9
#define INT8U unsigned char
#define INT32U unsigned char
float power=0;
float x_cur=0.00;
int gapV, count=100;
const int TOTAL_IC = 8;
const int CELLS = 14;
const int TEMP = 6;
uint16_t UV = 32500, OV = 41500;
float avgtemp_1;
float avgtemp_2;
float avgtemp,avg;
float CS=0, cur=0;
float cellMaxV;
float cellMinV;
//float cellMin;
float sum=0;
float aux1;
int aux_state;
float lowestcelltemp,highestcelltemp;
float lowestcelltemp_1,lowestcelltemp_2;
float highestcelltemp_1,highestcelltemp_2;
int i,j,k,l;
int c=0;
int csread;

float csv;
INT32U canId2 = 0x00000000;
INT32U canId3 = 0x00000000;
byte abcd[32];
INT32U len2 ;
INT32U buf2[8];

uint16_t currSend;
float readcellV[TOTAL_IC][CELLS]={0};
float sensorValue[TOTAL_IC][TEMP]={0};
const double R_inf = 10000 * exp( -3988 / 298.15 );
double gpio_supply , R , temp;
uint32_t conv_time;
int8_t error = 0;
double totalvoltage;

double R_DC;
int8_t stop_can = 0;
char input = 0, get_char();
const int SPI_CS_PIN = 10;       // CAN CS FOR L MASTER
MCP_CAN CAN(SPI_CS_PIN);

void serial_print_hex(uint8_t data);
const uint8_t ADC_OPT = ADC_OPT_DISABLED;
const uint8_t ADC_CONVERSION_MODE = MD_7KHZ_3KHZ;
const uint8_t ADC_DCP = DCP_DISABLED;
const uint16_t MEASUREMENT_LOOP_TIME = 20;//milliseconds(mS)
const uint8_t WRITE_CONFIG = ENABLED; // This is ENABLED or DISABLED
const uint8_t READ_CONFIG = ENABLED; // This is ENABLED or DISABLED  
const uint8_t MEASURE_CELL = ENABLED; // This is ENABLED or DISABLED
const uint8_t MEASURE_AUX = ENABLED; // This is ENABLED or DISABLED
const uint8_t PRINT_PEC = ENABLED; //This is ENABLED or DISABLED

int fic,fc,ftic,ftc;
  
cell_asic bms_ic[TOTAL_IC];
bool REFON = true, ADCOPT = false, FDRF = false, DTMEN = false;
bool gpioBits_a[5] = {true, true, true, true, true}; // Gpio 1,2,3,4,5
bool dccBits_a[12] = {false, false, false, false, false, false, false, false, false, false, false, false}; //Dcc 1,2,3,4,5,6,7,8,9,10,11,12
bool dctoBits[4] = {false, false, false, false}; //Dcto 0,1,2,3
bool psBits[2] = {false, false}; //ps-0,1
bool gpioBits_b[4] = {true, true, true, true}; // Gpio 6,7,8,9
bool dccBits_b[7] = {false, false, false, false, false, false, false}; //Dcc 0,13,14,15,16

unsigned char canMsg7[8] = {0x12, 0x28 , 0x00, 0x64, 0x00, 0x00, 0xFF, 0xFF}; //--FOR 464.8V 

void setup()
{
  pinMode(53,OUTPUT);
  pinMode(10,OUTPUT);
  pinMode(AMS_FAULT_PIN,OUTPUT);
  pinMode(29,OUTPUT);
  digitalWrite(29,LOW);
  digitalWrite(53,LOW);
  pinMode(28,INPUT);
  pinMode(A15,INPUT);

  digitalWrite(AMS_FAULT_PIN,HIGH);
  Serial.begin(115200);
  
  if(CAN.begin(CAN_1000KBPS) == CAN_OK)
  {
  Serial.println("CAN init ok\r\n");
  }
  else 
  {
    Serial.print("CAN init failed\r\n");
  }
  
  quikeval_SPI_connect();
  spi_enable(SPI_CLOCK_DIV32); // This will set the Linduino to have a 1MHz Clock
  LTC6813_init_cfg(TOTAL_IC, bms_ic);   
  LTC6813_init_cfgb(TOTAL_IC,bms_ic);
  for (uint8_t current_ic = 0; current_ic<TOTAL_IC;current_ic++) 
  {
    LTC6813_set_cfgr(current_ic,bms_ic,REFON,ADCOPT,gpioBits_a,dccBits_a, dctoBits, UV, OV);
    LTC6813_set_cfgrb(current_ic,bms_ic,FDRF,DTMEN,psBits,gpioBits_b,dccBits_b);   
  }   
  LTC6813_reset_crc_count(TOTAL_IC,bms_ic);
  LTC6813_init_reg_limits(TOTAL_IC,bms_ic);
  LTC6813_wrcfg(TOTAL_IC,bms_ic);
  LTC6813_wrcfgb(TOTAL_IC,bms_ic);
}

void loop()
{  
  double start = 0;
  double stop1 = 0;
  start = millis();
  wakeup_sleep(TOTAL_IC);
  wakeup_idle(TOTAL_IC);
  int8_t error = 0;
  uint32_t conv_time = 0;
  uint32_t user_command;
  int8_t readIC=0;
  char input = 0;
  uint32_t adcstate =0;
  wakeup_sleep(TOTAL_IC);
  wakeup_idle(TOTAL_IC);
  LTC6813_wrcfg(TOTAL_IC,bms_ic);
  LTC6813_wrcfg(TOTAL_IC,bms_ic);
  measurement_loop(DATALOG_DISABLED);
  amsfault();
  CANSEND();
  CurrentSensor();
  aux1=digitalRead(28);
  Serial.println(); 
  Serial.print("The aux state is :");
  Serial.print(aux1);
  Serial.println();
  Serial.print("Current:");
  Serial.print(x_cur);
  Serial.print("A");
  Serial.println();

  if(aux1>0)
  {
   charging();
   aux_state=aux1;
  }
   else
   {
    x_cur=0.00;
   }
 

  quikeval_SPI_connect();
  spi_enable(SPI_CLOCK_DIV32);
  delay(MEASUREMENT_LOOP_TIME); 
     if(x_cur<10.00 && (bms_ic[fic].cells.c_codes[fc]>UV && bms_ic[fic].cells.c_codes[fc]<OV )&&( sensorValue[ftic][ftc] < 50 && sensorValue[ftic][ftc] > 10) && CS==0.00)
       {
        digitalWrite(AMS_FAULT_PIN,HIGH);
        Serial.println("****************************************************NO AMS FAULT**************************************************************************");      
        Serial.println();  
       }       
  //delay(678);
  stop1 = millis();
  Serial.println("Loop Timing: ");
  Serial.print(stop1-start);
  quikeval_SPI_connect();
  spi_enable(SPI_CLOCK_DIV32);

}

void measurement_loop(uint8_t datalog_en)
{
  int8_t error = 0;
  if (WRITE_CONFIG == ENABLED)
  {
    wakeup_idle(TOTAL_IC);
    LTC6813_wrcfg(TOTAL_IC, bms_ic);
    LTC6813_wrcfgb(TOTAL_IC, bms_ic);
  }
  if (READ_CONFIG == ENABLED) 
  {
    wakeup_idle(TOTAL_IC);
    error = LTC6813_rdcfg(TOTAL_IC, bms_ic);
    error = LTC6813_rdcfgb(TOTAL_IC, bms_ic);
  }
  if (MEASURE_CELL == ENABLED)
  {
    wakeup_sleep(TOTAL_IC);
    wakeup_idle(TOTAL_IC);
    LTC6813_adcv(ADC_CONVERSION_MODE, ADC_DCP, CELL_CH_ALL);
    conv_time = LTC6813_pollAdc();
    wakeup_sleep(TOTAL_IC);
    wakeup_idle(TOTAL_IC);
    LTC6813_adcvsc(ADC_CONVERSION_MODE, ADC_DCP);
    conv_time = LTC6813_pollAdc();
    wakeup_idle(TOTAL_IC);
    error = LTC6813_rdcv(REG_ALL, TOTAL_IC, bms_ic);
    print_cells(datalog_en);
    print_totalvoltage();
    wakeup_idle(TOTAL_IC);
    error = LTC6813_rdstat(REG_ALL, TOTAL_IC, bms_ic); // Set to read back all aux registers
  }
  if (MEASURE_AUX == ENABLED)
  {
    wakeup_sleep(TOTAL_IC);
    wakeup_idle(TOTAL_IC);
    LTC6813_adax(ADC_CONVERSION_MODE , AUX_CH_ALL);
    wakeup_idle(TOTAL_IC);
    error = LTC6813_rdaux(0, TOTAL_IC, bms_ic); // Set to read back all aux registers
    print_aux(datalog_en);
    print_avgtemp();
  }
 quikeval_SPI_connect();
 spi_enable(SPI_CLOCK_DIV32);
}

void charging()
{
  Serial.println("Sending CAN*****************************");
  CAN.sendMsgBuf(MCP_TX0IF,0x1806E5F4, 1,0 , 8, canMsg7);  // REPLACE THE ACTUAL REQUIRED CANMSG HERE ACCORDING TO SETUP VARIABLES ABOVE
 //   CAN.sendMsgBuf(0x1806E5F4, 1, 8, canMsg7);  // REPLACE THE ACTUAL REQUIRED CANMSG HERE ACCORDING TO SETUP VARIABLES ABOVE

  canId2 = CAN.getCanId();
  //Serial.println(canId2,HEX);
  CAN.readMsgBuf(&len2, buf2);
  //Serial.println(len2);
  //Serial.print(buf2[0]);
  if(canId2==0x18FF50E5)
  {
  for(int i = 0; i<len2; i++) 
  {
    Serial.print("recieved msg");
    abcd[i] = buf2[i];  
    }
  }
  
  //Serial.print("CAN ID: ");
  //Serial.println(canId2,HEX);
  //Serial.print("data len = ");
  //Serial.println(len2);
  //Serial.print(buf2[0]);
  //for(int i = 0; i<len2; i++){
  //Serial.print(buf2[i]);
  //Serial.print("\t");
  //}
  //Serial.println();
  //for(int i=0;i<16;i++){Serial.print(abcd[i]); Serial.print(" ");
  //}
  //Serial.println("****************************************"); 
  //digitalWrite(28,HIGH);
  //Serial.print("-");
  //delay(1000);
  if(count>10)
  {
  //Serial.println(buf2[3]+ 0.2);
  count++;
  }
  else{
  //Serial.println(buf2[3]+ 0.5);
}
buf2[2]<<8;
x_cur = ((buf2[2] + (buf2[3]+0.2))/10)-0.46;
}



void amsfault()
{
  for (int current_ic = 0 ; current_ic < TOTAL_IC; current_ic++)
  {
    for (int i=0; i<CELLS; i++)
      { 
      if (bms_ic[current_ic].cells.c_codes[i] < UV || bms_ic[current_ic].cells.c_codes[i] > OV )
     {  
       fic=current_ic;
       fc=i;
       digitalWrite(AMS_FAULT_PIN,LOW);
       Serial.println();
       Serial.print("Fault in Stack ");
       Serial.print(fic);
       Serial.print(" Cell ");
       Serial.print(fc);
       Serial.println();     
      }
      }
     for (int i=0; i <5 ; i++)
    {      
       if (sensorValue[current_ic][i] > 50 || sensorValue[current_ic][i] < 10)
       {
          ftic=current_ic;
          ftc=i;
          digitalWrite(AMS_FAULT_PIN,LOW);
          Serial.println();
          Serial.print("Fault in Stack ");
          Serial.print(ftic);
          Serial.print(" Cell ");
          Serial.print(ftc);
          Serial.println();
        }
    }
     for (int i=6; i < 7; i++)
    {      
       if (sensorValue[current_ic][i-2] > 50 || sensorValue[current_ic][i-2] < 10)
     {
          ftic=current_ic;
          ftc=i;
          digitalWrite(AMS_FAULT_PIN,LOW);
          Serial.println();
          Serial.print("Fault in Stack ");
          Serial.print(ftic);
          Serial.print(" Cell ");
          Serial.print(ftc);
          Serial.println(); 
        }
    } 
    }
    if (x_cur>10.00)
    { digitalWrite(AMS_FAULT_PIN,LOW);
      Serial.println("Over current");
    }
    if (CS!=0.00)
    { digitalWrite(AMS_FAULT_PIN,LOW);
      Serial.println("Current sensor fault");
    }
}

void print_cells(uint8_t datalog_en)
{
  readcellV[TOTAL_IC][16]={0};
  for (int current_ic = 0 ; current_ic < TOTAL_IC; current_ic++)
  {
   Serial.print(" IC ");
   Serial.print(current_ic + 1, DEC);
   Serial.print(" , ");
    for (int i = 0; i < CELLS; i++)
    {
      wakeup_sleep(TOTAL_IC);
      wakeup_idle(TOTAL_IC);
     
      Serial.print(" C");
      Serial.print(i+1,DEC);
      Serial.print(":");
      Serial.print(bms_ic[current_ic].cells.c_codes[i] * 0.0001,4);
      Serial.print(",");  
        
      }
     Serial.println();
    }
     Serial.println();
    
  }

  void print_totalvoltage()
  {
  float sum[TOTAL_IC] = {0};
  totalvoltage = 0;
  cellMaxV=bms_ic[0].cells.c_codes[0]*0.0001;
  cellMinV=bms_ic[0].cells.c_codes[0]*0.0001;
     for (int current_ic = 0 ; current_ic < TOTAL_IC; current_ic++)
      {
        for(i=0;i<CELLS;i++)
        {
          sum[current_ic] = sum[current_ic] +  bms_ic[current_ic].cells.c_codes[i]*0.0001;
          if(bms_ic[current_ic].cells.c_codes[i]*0.0001<=cellMinV)
         {  
          cellMinV=bms_ic[current_ic].cells.c_codes[i]*0.0001;
         }
          if(bms_ic[current_ic].cells.c_codes[i]*0.0001>=cellMaxV)
         {  
          cellMaxV=bms_ic[current_ic].cells.c_codes[i]*0.0001;
         }
        }
//          Serial.println("Stack Voltage ");
//          Serial.print(current_ic);
//          Serial.print(sum[current_ic]);
            totalvoltage=totalvoltage+sum[current_ic];    
      }
           //power = totalvoltage*cur;
           Serial.println("Power: ");
           //Serial.print(power);
           Serial.println("Wh");
           Serial.println();
          Serial.println("Minimum Cell Voltage: ");
          Serial.print(cellMinV,4);
          Serial.println();
          Serial.println("Maximum Cell Voltage: ");
          Serial.print(cellMaxV,4);
          Serial.println();
          Serial.println("Total Accumulator Voltage ");
          Serial.print(totalvoltage+1);      
          Serial.print("V");
          Serial.println();
          Serial.println();
  }
   

void print_aux(uint8_t datalog_en)
{ 
  int i=0;
  avgtemp = 0;
  avgtemp_1 = 0;
  avgtemp_2 = 0;
  for (int current_ic = 0 ; current_ic < TOTAL_IC; current_ic++)
  {
    gpio_supply = bms_ic[current_ic].aux.a_codes[5] * 0.0001;
    Serial.print("IC ");
    Serial.print(current_ic + 1, DEC);
      for (int i = 0; i <5 ; i++)
      {      
        R = bms_ic[current_ic].aux.a_codes[i] / ( gpio_supply - (bms_ic[current_ic].aux.a_codes[i] * 0.0001)) ;
        sensorValue[current_ic][i]  = (3988 / log( R / R_inf)) - 273.15;
        Serial.print(" Temp-");
        Serial.print(i+1, DEC);
        Serial.print(":");
        Serial.print(sensorValue[current_ic][i]);
        Serial.print(",");  
        avgtemp_1 = avgtemp_1 + sensorValue[current_ic][i];     
       }
      for (int i = 6; i < 7; i++)
      {
        R = bms_ic[current_ic].aux.a_codes[i] / ( gpio_supply - (bms_ic[current_ic].aux.a_codes[i] * 0.0001)) ;
        sensorValue[current_ic][i-2] = (3988 / log( R / R_inf)) - 273.15;
        Serial.print(" Temp-");
        Serial.print(i, DEC);
        Serial.print(":");
        Serial.print(sensorValue[current_ic][i-2]);
        Serial.print(",");
        avgtemp_2 = avgtemp_2 + (sensorValue[current_ic][i-2]);
   
      }      
 Serial.println();
}

        Serial.println();
}

void print_avgtemp()
{
  lowestcelltemp = sensorValue[0][0];
  highestcelltemp = sensorValue[0][0];
  for (int current_ic = 0 ; current_ic < TOTAL_IC; current_ic++)
  {
    for (int i = 0; i <5 ; i++)
      {
          if(sensorValue[current_ic][i]>=highestcelltemp)
           {
              highestcelltemp_1=sensorValue[current_ic][i];
           }
      
        if(sensorValue[current_ic][i]<=lowestcelltemp)
        {
        lowestcelltemp_1=sensorValue[current_ic][i];
         }
      }
    for (int i = 6; i < 7; i++)
      {
        if(sensorValue[current_ic][i-2]>=highestcelltemp)
           {
             highestcelltemp_2=sensorValue[current_ic][i-2];
           }
      
      
        if(sensorValue[current_ic][i-2]<=lowestcelltemp_1)
        {
        lowestcelltemp_2=sensorValue[current_ic][i-2];
         }
      } 
      if(highestcelltemp_1>=highestcelltemp_2){
        highestcelltemp = highestcelltemp_1;
       }
       else{
        highestcelltemp = highestcelltemp_2;
       }
      
       if(lowestcelltemp_1<=lowestcelltemp_2){
        lowestcelltemp = lowestcelltemp_1;
       }
       else{
        lowestcelltemp = lowestcelltemp_2;
       } 
  }
       Serial.println("Minimum Temperature ");
       Serial.print(lowestcelltemp);
       Serial.println();
       Serial.println("Maximum Temperature ");
       Serial.print(highestcelltemp);
       Serial.println();

       avgtemp = (avgtemp_1+avgtemp_2)/(TOTAL_IC*8);
       Serial.println();
       Serial.println("Average Temperature ");
       Serial.print(avgtemp);
       Serial.println();
}

void CurrentSensor()
{
CS=digitalRead(A15);
cur=CS*(5/1023);
//cur=map(CS,102.3,920.7,-500,500);
//Serial.println("Current value is: ");
//Serial.print(CS);
//Serial.print("A");
//Serial.println();
}

void print_config()
{
 int cfg_pec;
  for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++)
  {
    for (int i = 0; i < 6; i++)
    {
      serial_print_hex(bms_ic[current_ic].config.tx_data[i]);
    }
    cfg_pec = pec15_calc(6, &bms_ic[current_ic].config.tx_data[0]);
    serial_print_hex((uint8_t)(cfg_pec >> 8));
    serial_print_hex((uint8_t)(cfg_pec));
    for (int i = 0; i < 6; i++)
    {
      serial_print_hex(bms_ic[current_ic].configb.tx_data[i]);
    }
    cfg_pec = pec15_calc(6, &bms_ic[current_ic].configb.tx_data[0]);
    serial_print_hex((uint8_t)(cfg_pec >> 8));
    serial_print_hex((uint8_t)(cfg_pec));
  }
}

void CANSEND(){
  
  Serial.print("CANSEND func called");
  uint8_t Voltage1[8] = {lowByte(bms_ic[0].cells.c_codes[0]),highByte(bms_ic[0].cells.c_codes[0]),lowByte(bms_ic[0].cells.c_codes[1]),highByte(bms_ic[0].cells.c_codes[1]),lowByte(bms_ic[0].cells.c_codes[2]),highByte(bms_ic[0].cells.c_codes[2]),lowByte(bms_ic[0].cells.c_codes[3]),highByte(bms_ic[0].cells.c_codes[3])};
  uint8_t Voltage2[8] = {lowByte(bms_ic[0].cells.c_codes[4]),highByte(bms_ic[0].cells.c_codes[4]),lowByte(bms_ic[0].cells.c_codes[5]),highByte(bms_ic[0].cells.c_codes[5]),lowByte(bms_ic[0].cells.c_codes[6]),highByte(bms_ic[0].cells.c_codes[6]),lowByte(bms_ic[0].cells.c_codes[7]),highByte(bms_ic[0].cells.c_codes[7])};
  uint8_t Voltage3[8] = {lowByte(bms_ic[0].cells.c_codes[8]),highByte(bms_ic[0].cells.c_codes[8]),lowByte(bms_ic[0].cells.c_codes[9]),highByte(bms_ic[0].cells.c_codes[9]),lowByte(bms_ic[0].cells.c_codes[10]),highByte(bms_ic[0].cells.c_codes[10]),lowByte(bms_ic[0].cells.c_codes[11]),highByte(bms_ic[0].cells.c_codes[11])};
  uint8_t Voltage4[8] = {lowByte(bms_ic[0].cells.c_codes[12]),highByte(bms_ic[0].cells.c_codes[12]),lowByte(bms_ic[0].cells.c_codes[13]),highByte(bms_ic[0].cells.c_codes[13]),lowByte(bms_ic[1].cells.c_codes[0]),highByte(bms_ic[1].cells.c_codes[0]),lowByte(bms_ic[1].cells.c_codes[1]),highByte(bms_ic[1].cells.c_codes[1])};
   
  uint8_t Voltage5[8] = {lowByte(bms_ic[1].cells.c_codes[2]),highByte(bms_ic[1].cells.c_codes[2]),lowByte(bms_ic[1].cells.c_codes[3]),highByte(bms_ic[1].cells.c_codes[3]),lowByte(bms_ic[1].cells.c_codes[4]),highByte(bms_ic[1].cells.c_codes[4]),lowByte(bms_ic[1].cells.c_codes[5]),highByte(bms_ic[1].cells.c_codes[5])};
  uint8_t Voltage6[8] = {lowByte(bms_ic[1].cells.c_codes[6]),highByte(bms_ic[1].cells.c_codes[6]),lowByte(bms_ic[1].cells.c_codes[7]),highByte(bms_ic[1].cells.c_codes[7]),lowByte(bms_ic[1].cells.c_codes[8]),highByte(bms_ic[1].cells.c_codes[8]),lowByte(bms_ic[1].cells.c_codes[9]),highByte(bms_ic[1].cells.c_codes[9])};
  uint8_t Voltage7[8] = {lowByte(bms_ic[1].cells.c_codes[10]),highByte(bms_ic[1].cells.c_codes[10]),lowByte(bms_ic[1].cells.c_codes[11]),highByte(bms_ic[1].cells.c_codes[11]),lowByte(bms_ic[1].cells.c_codes[12]),highByte(bms_ic[1].cells.c_codes[12]),lowByte(bms_ic[1].cells.c_codes[13]),highByte(bms_ic[1].cells.c_codes[13])};
  uint8_t Voltage8[8] = {lowByte(bms_ic[2].cells.c_codes[0]),highByte(bms_ic[2].cells.c_codes[0]),lowByte(bms_ic[2].cells.c_codes[1]),highByte(bms_ic[2].cells.c_codes[1]),lowByte(bms_ic[2].cells.c_codes[2]),highByte(bms_ic[2].cells.c_codes[2]),lowByte(bms_ic[2].cells.c_codes[3]),highByte(bms_ic[2].cells.c_codes[3])};
 
  uint8_t Voltage9[8] = {lowByte(bms_ic[2].cells.c_codes[4]),highByte(bms_ic[2].cells.c_codes[4]),lowByte(bms_ic[2].cells.c_codes[5]),highByte(bms_ic[2].cells.c_codes[5]),lowByte(bms_ic[2].cells.c_codes[6]),highByte(bms_ic[2].cells.c_codes[6]),lowByte(bms_ic[2].cells.c_codes[7]),highByte(bms_ic[2].cells.c_codes[7])};
  uint8_t Voltage10[8] = {lowByte(bms_ic[2].cells.c_codes[8]),highByte(bms_ic[2].cells.c_codes[8]),lowByte(bms_ic[2].cells.c_codes[9]),highByte(bms_ic[2].cells.c_codes[9]),lowByte(bms_ic[2].cells.c_codes[10]),highByte(bms_ic[2].cells.c_codes[10]),lowByte(bms_ic[2].cells.c_codes[11]),highByte(bms_ic[2].cells.c_codes[11])};
  uint8_t Voltage11[8] = {lowByte(bms_ic[2].cells.c_codes[12]),highByte(bms_ic[2].cells.c_codes[12]),lowByte(bms_ic[2].cells.c_codes[13]),highByte(bms_ic[2].cells.c_codes[13]),lowByte(bms_ic[3].cells.c_codes[0]),highByte(bms_ic[3].cells.c_codes[0]),lowByte(bms_ic[3].cells.c_codes[1]),highByte(bms_ic[3].cells.c_codes[1])};
  uint8_t Voltage12[8] = {lowByte(bms_ic[3].cells.c_codes[2]),highByte(bms_ic[3].cells.c_codes[2]),lowByte(bms_ic[3].cells.c_codes[3]),highByte(bms_ic[3].cells.c_codes[3]),lowByte(bms_ic[3].cells.c_codes[4]),highByte(bms_ic[3].cells.c_codes[4]),lowByte(bms_ic[3].cells.c_codes[5]),highByte(bms_ic[3].cells.c_codes[5])};
 
  uint8_t Voltage13[8] = {lowByte(bms_ic[3].cells.c_codes[6]),highByte(bms_ic[3].cells.c_codes[6]),lowByte(bms_ic[3].cells.c_codes[7]),highByte(bms_ic[3].cells.c_codes[7]),lowByte(bms_ic[3].cells.c_codes[8]),highByte(bms_ic[3].cells.c_codes[8]),lowByte(bms_ic[3].cells.c_codes[9]),highByte(bms_ic[3].cells.c_codes[9])};
  uint8_t Voltage14[8] = {lowByte(bms_ic[3].cells.c_codes[10]),highByte(bms_ic[3].cells.c_codes[10]),lowByte(bms_ic[3].cells.c_codes[11]),highByte(bms_ic[3].cells.c_codes[11]),lowByte(bms_ic[3].cells.c_codes[12]),highByte(bms_ic[3].cells.c_codes[12]),lowByte(bms_ic[3].cells.c_codes[13]),highByte(bms_ic[3].cells.c_codes[13])};
  uint8_t Voltage15[8] = {lowByte(bms_ic[4].cells.c_codes[0]),highByte(bms_ic[4].cells.c_codes[0]),lowByte(bms_ic[4].cells.c_codes[1]),highByte(bms_ic[4].cells.c_codes[1]),lowByte(bms_ic[4].cells.c_codes[2]),highByte(bms_ic[4].cells.c_codes[2]),lowByte(bms_ic[4].cells.c_codes[3]),highByte(bms_ic[4].cells.c_codes[3])};
  uint8_t Voltage16[8] = {lowByte(bms_ic[4].cells.c_codes[4]),highByte(bms_ic[4].cells.c_codes[4]),lowByte(bms_ic[4].cells.c_codes[5]),highByte(bms_ic[4].cells.c_codes[5]),lowByte(bms_ic[4].cells.c_codes[6]),highByte(bms_ic[4].cells.c_codes[6]),lowByte(bms_ic[4].cells.c_codes[7]),highByte(bms_ic[4].cells.c_codes[7])};
 
  uint8_t Voltage17[8] = {lowByte(bms_ic[4].cells.c_codes[8]),highByte(bms_ic[4].cells.c_codes[8]),lowByte(bms_ic[4].cells.c_codes[9]),highByte(bms_ic[4].cells.c_codes[9]),lowByte(bms_ic[4].cells.c_codes[10]),highByte(bms_ic[4].cells.c_codes[10]),lowByte(bms_ic[4].cells.c_codes[11]),highByte(bms_ic[4].cells.c_codes[11])};
  uint8_t Voltage18[8] = {lowByte(bms_ic[4].cells.c_codes[12]),highByte(bms_ic[4].cells.c_codes[12]),lowByte(bms_ic[4].cells.c_codes[13]),highByte(bms_ic[4].cells.c_codes[13]),lowByte(bms_ic[5].cells.c_codes[0]),highByte(bms_ic[5].cells.c_codes[0]),lowByte(bms_ic[5].cells.c_codes[1]),highByte(bms_ic[5].cells.c_codes[1])};
  uint8_t Voltage19[8] = {lowByte(bms_ic[5].cells.c_codes[2]),highByte(bms_ic[5].cells.c_codes[2]),lowByte(bms_ic[5].cells.c_codes[3]),highByte(bms_ic[5].cells.c_codes[3]),lowByte(bms_ic[5].cells.c_codes[4]),highByte(bms_ic[5].cells.c_codes[4]),lowByte(bms_ic[5].cells.c_codes[5]),highByte(bms_ic[5].cells.c_codes[5])};
  uint8_t Voltage20[8] = {lowByte(bms_ic[5].cells.c_codes[6]),highByte(bms_ic[5].cells.c_codes[6]),lowByte(bms_ic[5].cells.c_codes[7]),highByte(bms_ic[5].cells.c_codes[7]),lowByte(bms_ic[5].cells.c_codes[8]),highByte(bms_ic[5].cells.c_codes[8]),lowByte(bms_ic[5].cells.c_codes[9]),highByte(bms_ic[5].cells.c_codes[9])};
 
  uint8_t Voltage21[8] = {lowByte(bms_ic[5].cells.c_codes[10]),highByte(bms_ic[5].cells.c_codes[10]),lowByte(bms_ic[5].cells.c_codes[11]),highByte(bms_ic[5].cells.c_codes[11]),lowByte(bms_ic[5].cells.c_codes[12]),highByte(bms_ic[5].cells.c_codes[12]),lowByte(bms_ic[5].cells.c_codes[13]),highByte(bms_ic[5].cells.c_codes[13])};
  uint8_t Voltage22[8] = {lowByte(bms_ic[6].cells.c_codes[0]),highByte(bms_ic[6].cells.c_codes[0]),lowByte(bms_ic[6].cells.c_codes[1]),highByte(bms_ic[6].cells.c_codes[1]),lowByte(bms_ic[6].cells.c_codes[2]),highByte(bms_ic[6].cells.c_codes[2]),lowByte(bms_ic[6].cells.c_codes[3]),highByte(bms_ic[6].cells.c_codes[3])};
  uint8_t Voltage23[8] = {lowByte(bms_ic[6].cells.c_codes[4]),highByte(bms_ic[6].cells.c_codes[4]),lowByte(bms_ic[6].cells.c_codes[5]),highByte(bms_ic[6].cells.c_codes[5]),lowByte(bms_ic[6].cells.c_codes[6]),highByte(bms_ic[6].cells.c_codes[6]),lowByte(bms_ic[6].cells.c_codes[7]),highByte(bms_ic[6].cells.c_codes[7])};
  uint8_t Voltage24[8] = {lowByte(bms_ic[6].cells.c_codes[8]),highByte(bms_ic[6].cells.c_codes[8]),lowByte(bms_ic[6].cells.c_codes[9]),highByte(bms_ic[6].cells.c_codes[9]),lowByte(bms_ic[6].cells.c_codes[10]),highByte(bms_ic[6].cells.c_codes[10]),lowByte(bms_ic[6].cells.c_codes[11]),highByte(bms_ic[6].cells.c_codes[11])};

  uint8_t Voltage25[8] = {lowByte(bms_ic[6].cells.c_codes[12]),highByte(bms_ic[6].cells.c_codes[12]),lowByte(bms_ic[6].cells.c_codes[13]),highByte(bms_ic[6].cells.c_codes[13]),lowByte(bms_ic[7].cells.c_codes[0]),highByte(bms_ic[7].cells.c_codes[0]),lowByte(bms_ic[7].cells.c_codes[1]),highByte(bms_ic[7].cells.c_codes[1])};
  uint8_t Voltage26[8] = {lowByte(bms_ic[7].cells.c_codes[2]),highByte(bms_ic[7].cells.c_codes[2]),lowByte(bms_ic[7].cells.c_codes[3]),highByte(bms_ic[7].cells.c_codes[3]),lowByte(bms_ic[7].cells.c_codes[4]),highByte(bms_ic[7].cells.c_codes[4]),lowByte(bms_ic[7].cells.c_codes[5]),highByte(bms_ic[7].cells.c_codes[5])};
  uint8_t Voltage27[8] = {lowByte(bms_ic[7].cells.c_codes[6]),highByte(bms_ic[7].cells.c_codes[6]),lowByte(bms_ic[7].cells.c_codes[7]),highByte(bms_ic[7].cells.c_codes[7]),lowByte(bms_ic[7].cells.c_codes[8]),highByte(bms_ic[7].cells.c_codes[8]),lowByte(bms_ic[7].cells.c_codes[9]),highByte(bms_ic[7].cells.c_codes[9])};
  uint8_t Voltage28[8] = {lowByte(bms_ic[7].cells.c_codes[10]),highByte(bms_ic[7].cells.c_codes[10]),lowByte(bms_ic[7].cells.c_codes[11]),highByte(bms_ic[7].cells.c_codes[11]),lowByte(bms_ic[7].cells.c_codes[12]),highByte(bms_ic[7].cells.c_codes[12]),lowByte(bms_ic[7].cells.c_codes[13]),highByte(bms_ic[7].cells.c_codes[13])};

  uint8_t Temp1_1[8]    = {lowByte((uint16_t)sensorValue[0][0]),highByte((uint16_t)sensorValue[0][0]),lowByte((uint16_t)sensorValue[0][1]),highByte((uint16_t)sensorValue[0][1]),lowByte((uint16_t)sensorValue[0][2]),highByte((uint16_t)sensorValue[0][2]),lowByte((uint16_t)sensorValue[0][3]),highByte((uint16_t)sensorValue[0][3])};
  uint8_t Temp1_2[8]    = {lowByte((uint16_t)sensorValue[0][4]),highByte((uint16_t)sensorValue[0][4]),lowByte((uint16_t)sensorValue[0][5]),highByte((uint16_t)sensorValue[0][5]),lowByte((uint16_t)sensorValue[1][0]),highByte((uint16_t)sensorValue[1][0]),lowByte((uint16_t)sensorValue[1][1]),highByte((uint16_t)sensorValue[1][1])};
  uint8_t Temp2_1[8]    = {lowByte((uint16_t)sensorValue[1][2]),highByte((uint16_t)sensorValue[1][2]),lowByte((uint16_t)sensorValue[1][3]),highByte((uint16_t)sensorValue[1][3]),lowByte((uint16_t)sensorValue[1][4]),highByte((uint16_t)sensorValue[1][4]),lowByte((uint16_t)sensorValue[1][5]),highByte((uint16_t)sensorValue[1][5])};
  uint8_t Temp2_2[8]    = {lowByte((uint16_t)sensorValue[2][0]),highByte((uint16_t)sensorValue[2][0]),lowByte((uint16_t)sensorValue[2][1]),highByte((uint16_t)sensorValue[2][1]),lowByte((uint16_t)sensorValue[2][2]),highByte((uint16_t)sensorValue[2][2]),lowByte((uint16_t)sensorValue[2][3]),highByte((uint16_t)sensorValue[2][3])};
  uint8_t Temp3_1[8]    = {lowByte((uint16_t)sensorValue[2][4]),highByte((uint16_t)sensorValue[2][4]),lowByte((uint16_t)sensorValue[2][5]),highByte((uint16_t)sensorValue[2][5]),lowByte((uint16_t)sensorValue[3][0]),highByte((uint16_t)sensorValue[3][0]),lowByte((uint16_t)sensorValue[3][1]),highByte((uint16_t)sensorValue[3][1])};
  uint8_t Temp3_2[8]    = {lowByte((uint16_t)sensorValue[3][2]),highByte((uint16_t)sensorValue[3][2]),lowByte((uint16_t)sensorValue[3][3]),highByte((uint16_t)sensorValue[3][3]),lowByte((uint16_t)sensorValue[3][4]),highByte((uint16_t)sensorValue[3][4]),lowByte((uint16_t)sensorValue[3][5]),highByte((uint16_t)sensorValue[3][5])};
  uint8_t Temp4_1[8]    = {lowByte((uint16_t)sensorValue[4][0]),highByte((uint16_t)sensorValue[4][0]),lowByte((uint16_t)sensorValue[4][1]),highByte((uint16_t)sensorValue[4][1]),lowByte((uint16_t)sensorValue[4][2]),highByte((uint16_t)sensorValue[4][2]),lowByte((uint16_t)sensorValue[4][3]),highByte((uint16_t)sensorValue[4][3])};
  uint8_t Temp4_2[8]    = {lowByte((uint16_t)sensorValue[4][4]),highByte((uint16_t)sensorValue[4][4]),lowByte((uint16_t)sensorValue[4][5]),highByte((uint16_t)sensorValue[4][5]),lowByte((uint16_t)sensorValue[5][0]),highByte((uint16_t)sensorValue[5][0]),lowByte((uint16_t)sensorValue[5][1]),highByte((uint16_t)sensorValue[5][1])};
  uint8_t Temp5_1[8]    = {lowByte((uint16_t)sensorValue[5][2]),highByte((uint16_t)sensorValue[5][2]),lowByte((uint16_t)sensorValue[5][3]),highByte((uint16_t)sensorValue[5][3]),lowByte((uint16_t)sensorValue[5][4]),highByte((uint16_t)sensorValue[5][4]),lowByte((uint16_t)sensorValue[5][5]),highByte((uint16_t)sensorValue[5][5])};
  uint8_t Temp5_2[8]    = {lowByte((uint16_t)sensorValue[6][0]),highByte((uint16_t)sensorValue[6][0]),lowByte((uint16_t)sensorValue[6][1]),highByte((uint16_t)sensorValue[6][1]),lowByte((uint16_t)sensorValue[6][2]),highByte((uint16_t)sensorValue[6][2]),lowByte((uint16_t)sensorValue[6][3]),highByte((uint16_t)sensorValue[6][3])};
  uint8_t Temp6_1[8]    = {lowByte((uint16_t)sensorValue[6][4]),highByte((uint16_t)sensorValue[6][4]),lowByte((uint16_t)sensorValue[6][5]),highByte((uint16_t)sensorValue[6][5]),lowByte((uint16_t)sensorValue[7][0]),highByte((uint16_t)sensorValue[7][0]),lowByte((uint16_t)sensorValue[7][1]),highByte((uint16_t)sensorValue[7][1])};
  uint8_t Temp6_2[8]    = {lowByte((uint16_t)sensorValue[7][2]),highByte((uint16_t)sensorValue[7][2]),lowByte((uint16_t)sensorValue[7][3]),highByte((uint16_t)sensorValue[7][3]),lowByte((uint16_t)sensorValue[7][4]),highByte((uint16_t)sensorValue[7][4]),lowByte((uint16_t)sensorValue[7][5]),highByte((uint16_t)sensorValue[7][5])};
  uint8_t Fault[8] = {lowByte((uint16_t)fic),highByte((uint16_t)fic),lowByte((uint16_t)fc),highByte((uint16_t)fc),lowByte((uint16_t)ftic),highByte((uint16_t)ftic),lowByte((uint16_t)ftc),highByte((uint16_t)ftc)};
  uint8_t Cell_info[8] = {lowByte((uint16_t)lowestcelltemp),highByte((uint16_t)lowestcelltemp),lowByte((uint16_t)highestcelltemp),highByte((uint16_t)highestcelltemp),lowByte((uint16_t)cellMinV),highByte((uint16_t)cellMinV),lowByte((uint16_t)cellMaxV),highByte((uint16_t)cellMaxV)};
  uint8_t avgtemp1[8] = {lowByte((uint16_t)avgtemp),highByte((uint16_t)avgtemp)};
 
  uint8_t Voltage29[8]    = {lowByte((uint16_t)totalvoltage),highByte((uint16_t)totalvoltage),lowByte((uint16_t)cur),highByte((uint16_t)cur),lowByte((uint16_t)power),highByte((uint16_t)power)};
  CAN.sendMsgBuf(0x069,0,8,Voltage1);
  CAN.sendMsgBuf(0x06A,0,8,Voltage2);
  CAN.sendMsgBuf(0x06B,0,8,Voltage3);
  CAN.sendMsgBuf(0x06C,0,8,Voltage4);
  CAN.sendMsgBuf(0x06D,0,8,Voltage5);
  CAN.sendMsgBuf(0x06E,0,8,Voltage6);
  CAN.sendMsgBuf(0x06F,0,8,Voltage7);
  CAN.sendMsgBuf(0x070,0,8,Voltage8);
  CAN.sendMsgBuf(0x071,0,8,Voltage9);
  CAN.sendMsgBuf(0x072,0,8,Voltage10);
  CAN.sendMsgBuf(0x073,0,8,Voltage11);
  CAN.sendMsgBuf(0x078,0,8,Voltage16);
  CAN.sendMsgBuf(0x079,0,8,Voltage17);
  CAN.sendMsgBuf(0x07A,0,8,Voltage18);
  CAN.sendMsgBuf(0x07B,0,8,Voltage19);
  CAN.sendMsgBuf(0x07C,0,8,Voltage20);
  CAN.sendMsgBuf(0x07D,0,8,Voltage21);
  CAN.sendMsgBuf(0x07E,0,8,Voltage22);
  CAN.sendMsgBuf(0x07F,0,8,Voltage23);
  CAN.sendMsgBuf(0x080,0,8,Voltage24);
  CAN.sendMsgBuf(0x081,0,8,Voltage25);
  CAN.sendMsgBuf(0x082,0,8,Voltage26);
  CAN.sendMsgBuf(0x083,0,8,Voltage27);
  CAN.sendMsgBuf(0x084,0,8,Voltage28);
  CAN.sendMsgBuf(0x085,0,8,Temp1_1);
  CAN.sendMsgBuf(0x086,0,8,Temp1_2);
  CAN.sendMsgBuf(0x087,0,8,Temp2_1);
  CAN.sendMsgBuf(0x088,0,8,Temp2_2);
  CAN.sendMsgBuf(0x089,0,8,Temp3_1);
  CAN.sendMsgBuf(0x08A,0,8,Temp3_2);
  CAN.sendMsgBuf(0x08B,0,8,Temp4_1);
  CAN.sendMsgBuf(0x08C,0,8,Temp4_2);
  CAN.sendMsgBuf(0x08D,0,8,Temp5_1);
  CAN.sendMsgBuf(0x08E,0,8,Temp5_2);
  CAN.sendMsgBuf(0x08F,0,8,Temp6_1);
  CAN.sendMsgBuf(0x09A,0,8,Temp6_2);
  CAN.sendMsgBuf(0x09B,0,8,avgtemp1);
  CAN.sendMsgBuf(0x09C,0,8,Cell_info);
  CAN.sendMsgBuf(0x09D,0,8,Fault);
  CAN.sendMsgBuf(0x09E,0,8,Voltage29);
  //CAN.sendMsgBuf(0x06A,0,8,Voltage27);
 
  Serial.print("CANSEND func ended");
  Serial.println();
}       
