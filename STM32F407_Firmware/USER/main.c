#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "timer.h"
#include "global.h"
#include "PWM.h"

extern u8  TIM5CH1_CAPTURE_STA;		// Input capture status		    				
extern u32	TIM5CH1_CAPTURE_VAL;	// Input capture value  

extern u8  TIM5CH2_CAPTURE_STA;	// Input capture status		    				
extern u32	TIM5CH2_CAPTURE_VAL;	// Input capture value (TIM2/TIM5 are 32-bit)

extern u8  TIM5CH3_CAPTURE_STA;	// Input capture status		    				
extern u32	TIM5CH3_CAPTURE_VAL;	// Input capture value (TIM2/TIM5 are 32-bit)

extern u8  TIM5CH4_CAPTURE_STA;	// Input capture status		    				
extern u32	TIM5CH4_CAPTURE_VAL;	// Input capture value (TIM2/TIM5 are 32-bit)



#define DJLime 667

int main(void)
{ 
	long long DIS_L=0; 
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);// Set interrupt priority grouping to Group 2
	delay_init(168);  // Initialize delay function
	LED_Init();				// Initialize LED port
	CSBStartIO_Init(); // Initialize ultrasonic sensor probe


	RecHeadFlag=0;// Data receive header state
  uart_init(460800);	//921600  8bit ,1 stop bit, no parity // UART initialization

	
	// Motor initialization
	TIM13_PWM_Init(20000-1,84-1);	////84M/84=1Mhz counting frequency,20000 1M/20000=50hz. 
	TIM14_PWM_Init(20000-1,84-1);	////84M/84=1Mhz counting frequency,20000 1M/20000=50hz. 
	delay_init(168);  // Initialize delay function again
		
	//TIM_SetCompare1(TIM13,1500);	// Pitch: modify compare value, initial motor at middle position 1500 range 500~2500, increase = CW rotation
	//TIM_SetCompare1(TIM14,1500);	// Yaw: modify compare value, initial motor at middle position, increase = CW rotation 
  DJ_Angel_FW=0.0;DJ_Angel_FY=0.0;
	SetFWAngle(0.0); // CW is positive (left), CCW is negative (right)
	SetFYAngle(0.0);// CW is positive, CCW is negative
	
	// Ultrasonic module distance capture
	TIM5_CH1_Cap_Init(0XFFFFFFFF,84-1); // Counting frequency 1Mhz // Initialize 4 ultrasonic channels
	
	TIM2_Int_Init(0XFFFFFFFF,84-1);// Free run, unit in microseconds
	// Timer every 51ms to get ultrasonic sensor data
 	TIM3_Int_Init(510-1,8400-1);	// Timer clock 84M, prescaler 8400, so 84M/8400=10Khz counting frequency, counting 500 times = 50ms 
  G_T3_CNT=0;// Count number of times TIME3 enters	
	GetCSCH_Num=0;// Measurement channel indicator
	
	while(1)
	{
		//LED1_Toggle;//DS0 toggle
		
		GetCSCHData();
		if(SendDataFlag==0xAA)
		{
			LED1_Toggle;//DS1 toggle
			SendDatatoPC();// Send communication data
			SendDataFlag=0;
		}
		
				
		 RECPCData();// Receive data from upper computer
		 // delay_ms(500);//Delay 200ms
		
	};
}

//------------------------------------------------------

void GetCSCHData(void)
{
	 long long temp1,temp2,temp3,temp4; 
		if(TIM5CH1_CAPTURE_STA&0X80)        // Successfully captured one high level
		{
			u32 D_LL;
			double f_L;
			temp1=TIM5CH1_CAPTURE_STA&0X3F; 
			temp1*=0XFFFFFFFF;		 		         // Total overflow time
			temp1+=TIM5CH1_CAPTURE_VAL;		   // Get total high-level time
			CH1_dt=temp1;
			//printf("CH 1 :%lld us\r\n",temp1); // Print total high-level time
			//TIM5CH1_CAPTURE_STA=0;			     // Enable next capture		
		}
		if(TIM5CH2_CAPTURE_STA&0X80)        // Successfully captured one high level
		{
			u32 D_LL;
			double f_L;
			temp2=TIM5CH2_CAPTURE_STA&0X3F; 
			temp2*=0XFFFFFFFF;		 		         // Total overflow time
			temp2+=TIM5CH2_CAPTURE_VAL;		   // Get total high-level time
			CH2_dt=temp2;
			//printf("CH 2 :%lld us\r\n",temp2); // Print total high-level time
			//TIM5CH1_CAPTURE_STA=0;			     // Enable next capture		
		}		
		if(TIM5CH3_CAPTURE_STA&0X80)        // Successfully captured one high level
		{
			u32 D_LL;
			double f_L;
			temp3=TIM5CH3_CAPTURE_STA&0X3F; 
			temp3*=0XFFFFFFFF;		 		         // Total overflow time
			temp3+=TIM5CH3_CAPTURE_VAL;		   // Get total high-level time
			CH3_dt=temp3;			
			//printf("CH 3 :%lld us\r\n",temp3); // Print total high-level time
			
			//TIM5CH1_CAPTURE_STA=0;			     // Enable next capture		
		}		
		if(TIM5CH4_CAPTURE_STA&0X80)        // Successfully captured one high level
		{
			u32 D_LL;
			double f_L;
			temp4=TIM5CH4_CAPTURE_STA&0X3F; 
			temp4*=0XFFFFFFFF;		 		         // Total overflow time
			temp4+=TIM5CH4_CAPTURE_VAL;		   // Get total high-level time
			CH4_dt=temp4;
			//printf("CH 4 :%lld us\r\n",temp4); // Print total high-level time
			//TIM5CH1_CAPTURE_STA=0;			     // Enable next capture		
		}	
}



void RECPCData(void)// Receive data from PC end
{
	int i;
	uint16_t RCPWMFW,RCPWMFY;
	char CMD;
		if(USART_RX_STA&0x8000) // Data received
		{	
		  int len;
			len=USART_RX_STA&0x3fff;// Get the length of received data
			if(len>3)
			{
				if((USART_RX_BUF[0]==0xEC)&&(USART_RX_BUF[1]==0x91))// Data header 0xEC 0x91, data end 0x0d 0x0a
				{
					CMD=USART_RX_BUF[2];
					
					// When sending from upper computer, low byte first then high byte
					RCPWMFW=0;
					RCPWMFW=USART_RX_BUF[4];
					RCPWMFW=RCPWMFW<<8;
					RCPWMFW=RCPWMFW+USART_RX_BUF[3];
					
					RCPWMFY=0;
					RCPWMFY=USART_RX_BUF[6];
					RCPWMFY=RCPWMFY<<8;
					RCPWMFY=RCPWMFY+USART_RX_BUF[5];
					
					if(CMD==0x55)// Set motor angle
					{
						int16_t jiao_FW,jiao_FY;
						float f_FW,f_FY;
						if(RCPWMFW<500) RCPWMFW=500;
						if(RCPWMFW>2500) RCPWMFW=2500;
						jiao_FW=RCPWMFW;
						jiao_FW=jiao_FW-1500;
						f_FW=(float)jiao_FW*(270.0/2000.0);// Convert to degrees
						SetFWAngle(f_FW); // CW positive (left), CCW negative (right)

						if(RCPWMFY<500) RCPWMFY=500;
						if(RCPWMFY>2500) RCPWMFY=2500;
						jiao_FY=RCPWMFY;
						jiao_FY=jiao_FY-1500;
						f_FY=(float)jiao_FY*(270.0/2000.0);// Convert to degrees
						SetFYAngle(f_FY); // CW positive (left), CCW negative (right)										
					}
					
					if(CMD==0xAA)// Reset
					{
						SetFWAngle(0.0); // CW positive (left), CCW negative (right)
						SetFYAngle(0.0); // CW positive (left), CCW negative (right)						
					}
					
		/*
					USART_SendData(USART1, len);         // Send data to USART1
					while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);// Wait for send completion					
					USART_SendData(USART1, USART_RX_BUF[0]);         // Send data to USART1
					while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);// Wait for send completion
					USART_SendData(USART1, USART_RX_BUF[1]);         // Send data to USART1
					while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);// Wait for send completion
					USART_SendData(USART1, USART_RX_BUF[2]);         // Send data to USART1
					while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);// Wait for send completion
			*/	
					for(i=0;i<len;i++)
					{
				
						USART_RX_BUF[i]=0; // Clear receive buffer
					}
					
				}
			}
				USART_RX_STA=0;
		}
}


void SendU16(uint16_t data)
{
	uint16_t tempH,tempL;
	tempH=(data >> 8) & 0xFF;
	tempL=data & 0xFF;
	while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);// Wait for send complete					
	USART_SendData(USART1, tempH);         // Send high byte
	while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);// Wait for send complete					
	USART_SendData(USART1, tempL);         // Send low byte
}

void SendU32(uint32_t data)
{
	uint16_t temp[4],i;
	temp[0]=(data >> 24) & 0xFF;
	temp[1]=(data >> 16) & 0xFF;	
	temp[2]=(data >> 8) & 0xFF;
	temp[3]=data & 0xFF;
	for(i=0;i<4;i++)
	{
		while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);// Wait for send complete					
		USART_SendData(USART1, temp[i]);        
	}		
}


void SendDatatoPC(void)// Send data to PC, high byte first
{// Send data to PC, high byte first;
	uint16_t  U16_DJPWM_FW,U16_DJPWM_FY;
	U16_DJPWM_FW=DJ_PWMCNT_FW;
	U16_DJPWM_FY=DJ_PWMCNT_FY;
	
	while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);// Wait for send complete					
	USART_SendData(USART1, 0xEA);         // Send header
	
	SendU32(TIM2->CNT);// Send system time
	
	SendU16(U16_DJPWM_FW); // Motor position
	SendU16(U16_DJPWM_FY);	
	
	SendU16(CH1_dt); // Ultrasonic channel
	SendU16(CH2_dt);
	SendU16(CH3_dt);
	SendU16(CH4_dt);
	SendU16(GetCSCH_Num);// Current tested channel
	SendU16(CSCH_EndFlag);
	
	SendU16(0xAA);
	SendU16(0xAA);
	SendU16(0xAA);
	SendU16(0xAA);	
}

//==============================================================// Interrupt Service Routine
// Timer 3 interrupt service function
void TIM3_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM3,TIM_IT_Update)==SET) // Overflow interrupt
	{
		SendDataFlag=0xAA;// Communication send valid flag	
		// LED flashing status indication
		G_T3_CNT++;
		if(G_T3_CNT>=20)
		{
			//LED1_Toggle;//DS1 toggle
			G_T3_CNT=0;
		}
		
	
		
		GetCSCH_Num++;
		if(GetCSCH_Num>=5)
		{GetCSCH_Num=1;}
		
		RestCap5();
	  delay_us(10);	
		
		switch(GetCSCH_Num){
			case 1:
							CSB0_H;
							delay_us(15);// Delay 20us start
							CSB0_L;
			        CSCH_EndFlag=0;
							break;
			case 2:
							CSB1_H;
							delay_us(15);// Delay 20us start
							CSB1_L;
			        CSCH_EndFlag=0;
							break;
			case 3:
							CSB2_H;
							delay_us(15);// Delay 20us start
							CSB2_L;
							CSCH_EndFlag=0;
							break;
			case 4:
							CSB3_H;
							delay_us(15);// Delay 20us start
							CSB3_L;
			        CSCH_EndFlag=0;
							break;			
			default:
					    CSB0_H;
							delay_us(15);// Delay 20us start
							CSB0_L;
			        CSCH_EndFlag=0;
							break;	
			
			
		}

		//CSB0_Toggle;// Test use
	}
	TIM_ClearITPendingBit(TIM3,TIM_IT_Update);  // Clear interrupt flag
}
