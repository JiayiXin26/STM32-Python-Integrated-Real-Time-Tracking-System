#include "global.h"

 int G_T3_CNT;// T3 timer entry count
 int   GetCSCH_Num=0; // Indicator for measured ultrasonic channel. CH1~CH4.
 uint16_t  CSCH_EndFlag=0;// CH1 measurement end flag £º0xAA measurement finished
 uint16_t  SendDataFlag;// Data send flag

 uint16_t  CH1_dt=0; // Measured length
 uint16_t  CH2_dt=0;
 uint16_t  CH3_dt=0;
 uint16_t  CH4_dt=0;
 char  RecHeadFlag=0x90;
 //int  DJ_PWM_FY=0; // Motor pitch value
 //int  DJ_PWM_FW=0;// Motor azimuth value
 float  DJ_Angel_FY=0; // Motor pitch value
 float  DJ_Angel_FW=0;// Motor azimuth value
 uint32_t  DJ_PWMCNT_FY; // Motor pitch PWM count
 uint32_t  DJ_PWMCNT_FW;// Motor azimuth  PWM count
