#include<iom16v.h>
#include<macros.h>
#include"I2C.h"

#define uchar  unsigned char
#define uint   unsigned int

//����            // PB0 PB1 PB2 PB3 PB4 PB5        0x3f
#define WHITE     0x3E        //�׵ư�����                    
#define BLUE      0x3D        //���ư�����         
#define RED       0x3B        //��ư�����
#define YGD1      0x37         //Ԥ���1������
#define YGD2     0x2f           //Ԥ���2������
#define YGD3    0x1f          //Ԥ���3������
//����
uchar CODE_W=0x51;  //�׵���ֵ
uchar CODE_B=0x52;  //������ֵ 
uchar CODE_R=0x53;  //�����ֵ
uchar CODE_Y=0x54; //Ԥ���

//uchar distance;   //����
#define DCI_ON    PORTD|=BIT(7)
#define DCI_OFF   PORTD&=~BIT(7)

uchar dat[]={0,0,0,0};
uchar keyvalue;
uchar  buf_writedata[10];          //д�����ݻ���
uint count_WD;


void port_init(void)
{
       DDRB=0xc0;PORTB=0x3f;          // ����
	   DDRA=0x0f;PORTA=0x0f;        //   ��
       DDRD=0xff; 
}

void WDT_init()
{
	  WDR();
	  WDTCR = ( (1<< WDTOE)|(1<< WDE) | (1 << WDP2) | (1 <<WDP1))|(1 << WDP0);   //�趨����Ϊ2S
}

//***********************************************************************************
//��ʱ��0��ʼ������
//***********************************************************************************
void timer0_init(void)
{
	TCCR0 = 0x00; //stop
 	TCNT0 = 0x06; //set count     timer0 ÿ2ms�ж�һ��256- (FOCS/PRE)* T
 	OCR0  = 0xFA;  //set compare
 	TCCR0 = 0x03; //start timer   64��Ƶ
}
#pragma interrupt_handler deal_Timer0:iv_TIMER0_OVF
void deal_Timer0()
{
    TCNT0 = 0x06; //�ж�һ�ζ�TMR0���¸���ֵ
    count_WD++;           //ι������
	
	if(count_WD>= 500)  
	 {
	     WDR();          //1���Ӽ��ι��   //��ι����־     
		 count_WD=0;
     }    
}

void keyScan(void)
{

    if((PINB&0x3f)!=0x3f)
	{
	     delay_ms(10);
		 if((PINB&0x3f)!=0x3f)
		 {
		     keyvalue = PINB;
			 PORTB=0x3f;
			 keyHandle();
		 }
	}          
}

void keyHandle()
{
    switch(keyvalue)
	{
	    case WHITE:          //�׵ư�������
		    {
			    
				PORTA&=~BIT(3);           //�����׵�				
				deal_fixation_point(buf_writedata, CODE_W, 0xf1);
				//delay_ms(1);
				PORTA|=BIT(3);	
				keyvalue=0x00;
                break;                    
			}
			
		case BLUE:          //���ư�������
		    {	
				PORTA&=~BIT(2);	
				deal_fixation_point(buf_writedata, CODE_B,0xf2);
				//delay_ms(1);
				PORTA|=BIT(2);
				keyvalue=0x00;
                break;                    
			}
			
		case RED:          //��ư�������
		    {
			    PORTA&=~BIT(1);	
				deal_fixation_point(buf_writedata, CODE_R,0xf3);
				//delay_ms(1);
				PORTA|=BIT(1);
				keyvalue=0x00;
                break;                    
			}

		case YGD1:          //Ԥ���1��������
		    {
			    PORTA&=~BIT(0);	
				deal_fixation_point(buf_writedata,CODE_Y,0xf1);
				//delay_ms(1);
				PORTA|=BIT(0);	
				keyvalue=0x00;
                break;                    
			}
			
		case YGD2:          //Ԥ���2��������
		    {
			    PORTA&=~BIT(0);	
				deal_fixation_point(buf_writedata,CODE_Y,0xf2);
				//delay_ms(1);
				PORTA|=BIT(0);	
				keyvalue=0x00;
                break;                    
			}
			
		case YGD3:          //Ԥ���3��������
		    {
			    PORTA&=~BIT(0);	
				deal_fixation_point(buf_writedata,CODE_Y,0xf3);
				//delay_ms(1);
				PORTA|=BIT(0);	
				keyvalue=0x00;
                break;                    
			}
			default: break;
	}
}

void deal_fixation_point(uchar buf_w[], uchar led,uchar distance)
{
	uchar temp1= 0;
	uchar temp2 =0;
	
	buf_w[0]=0x02;
	buf_w[1]= led;
	buf_w[2]= distance;            //����
	buf_w[3]= 0x44;           //44
	
	if(led == CODE_Y)
	{ 
	     buf_w[0]=0x01;
		 buf_w[1]=CODE_R;
	}
    
	//temp1 = Write_2Byte1(0xdc,0x1e, buf_writedata);
	 temp1 = Write_2Byte1(0xa0,0x1e, buf_writedata);
	delay_ms(200);
	//temp2 = Write_2Byte2(0xdc,0x20, buf_writedata);
	temp2 = Write_2Byte2(0xa0,0x20, buf_writedata);
	delay_ms(200);
   WDR();
   
	if(temp1&&temp2)			
	{
	    //PORTA=0x00;       //
	    DCI_OFF;
	    delay_ms(8000);
		   //PORTA=0x0f;
	}
	else
	{
		  PORTA=0x00;
		  delay_ms(500000);
	}
	
}


void main(void)
{
    
	//Write_4Byte(0xa0,0x00, dat); 
	//delay_ms(100);
	unsigned char buff[] = {0x01, 0x02};
	WDR();
	port_init();
	WDT_init();
	I2C_init();
	while(1)
	{	     
		 WDR();
		 DCI_ON;
		 keyScan(); 
	}
}






