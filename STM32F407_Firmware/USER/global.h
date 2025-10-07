#ifndef __GLOBAL_H
#define __GLOBAL_H

#include "stm32f4xx.h"



extern  int G_T3_CNT;// T3 timer entry count
extern  int   GetCSCH_Num; // Indicator for measured ultrasonic channel. CH1~CH4.
extern  uint16_t  CSCH_EndFlag;// CH measurement end flag £º0xAA measurement finished

extern uint16_t  CH1_dt; // Measured length
extern uint16_t  CH2_dt;
extern uint16_t  CH3_dt;
extern uint16_t  CH4_dt;
extern char  RecHeadFlag;
//extern int  DJ_PWM_FY; // Motor pitch value
//extern int  DJ_PWM_FW;// Motor azimuth value
extern float  DJ_Angel_FY; // Motor pitch angle value
extern float  DJ_Angel_FW;// Motor azimuth angle value

extern uint32_t  DJ_PWMCNT_FY; // Motor pitch PWM count
extern uint32_t  DJ_PWMCNT_FW;// Motor azimuth PWM count

extern  uint16_t  SendDataFlag;// Data send flag

void RECPCData(void); // Function for receiving and processing data from upper computer
void GetCSCHData(void);// Get ultrasonic probe data
void SendDatatoPC(void);// Send data to the upper computer, send high byte first, then low byte
void SendU16(uint16_t data);
void SendU32(uint32_t data);

#endif //__GLOBAL_H
