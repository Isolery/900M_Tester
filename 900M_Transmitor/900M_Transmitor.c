#include <avr/io.h>
#include <util/twi.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "predefine.h"

/* TWI����START�źţ�����TWI���� */
#define TWISTART()    (TWCR = (1<<TWSTA)|(1<<TWINT)|(1<<TWEN)|(1<<TWIE))    

/* ����  */  // PB0 PB1 PB2 PB3 PB4 PB5 
#define WHITE    0x3E        //�׵ư�����                    
#define BLUE       0x3D      //���ư�����         
#define RED        0x3B       //��ư�����
#define YGD1      0x37      //Ԥ���1������
#define YGD2      0x2F      //Ԥ���2������
#define YGD3      0x1F       //Ԥ���3������

/* ���� */
#define CODE_W    0x51	//�׵���ֵ
#define CODE_B     0x52	//������ֵ 
#define CODE_R     0x53	//�����ֵ
#define CODE_Y     0x54	//Ԥ���

#define DCI_ON    (PORTD |= (1<<7))
#define DCI_OFF   (PORTD &=~ (1<<7))

#define TRUE    1
#define FALSE   0

#define WDR()	  asm("wdr")

/* �豸��ַ */
#define DEVICEADDR    0xA0

uint8_t WordAddr;    //�ڴ��ַ
uint8_t keyValue;
uint8_t TransOk;
uint16_t tWDCount;

uint8_t writedata[2];          //д�����ݻ���

/*����������*/
void TWI_Init();
void Port_Init(void);
void KeyScan(void);
void INT0_Init(void);
void WDT_Init(void);
void SystemInit(void);
void KeyHandle(void);

/**********************************************************************************************
* �� �� ��  ��main
* ����˵����c�������
* ���������void
* �� �� ֵ ��void
*********************************************************************************************/
int main(void)
{
	SystemInit(); 

	while(1)
	{
		WDR();
		DCI_ON;
		_delay_ms(1000);
	}
	return 0;
}

/**********************************************************************************************
* �� �� ��  ��Port_Init
* ����˵�����˿ڳ�ʼ��
* ���������void
* �� �� ֵ ��void
*********************************************************************************************/
void Port_Init(void)
{      
	   DDRA = 0x0f;PORTA = 0x0f;         //��
	   DDRB = 0xc0;PORTB = 0x3f;        //����
       DDRD = 0xff; 
	   DDRD &=~ (1<<2); PORTD |= (1<<2);   //INT0
}

/**********************************************************************************************
* �� �� ��  ��WDT_Init
* ����˵�������Ź���ʼ����2s��ûι����λ
* ���������void
* �� �� ֵ ��void
*********************************************************************************************/
void WDT_Init(void)
{
	  WDR();
	  WDTCR = ( (1<< WDTOE)|(1<< WDE) | (1 << WDP2) | (1 <<WDP1))|(1 << WDP0);   //�趨����Ϊ2S
}

/**********************************************************************************************
* �� �� ��  ��INT0_Init
* ����˵�����ⲿ�ж�0��ʼ���������͵�ƽ����
* ���������void
* �� �� ֵ ��void
*********************************************************************************************/
void INT0_Init(void)
{
	GICR &= ~(1<<INT0);                     //�ر�INT0�ж�ʹ��
	MCUCR = 0x00;    //�͵�ƽ����
	GICR |= (1<<INT0);                        //����INT0�ж�ʹ��
}

/**********************************************************************************************
* ����˵�����ⲿ�ж�0�жϺ���
* ���������void
* �� �� ֵ ��void
*********************************************************************************************/
ISR(INT0_vect)
{
	GICR &= ~(1<<INT0);	
	TransOk = FALSE;
	sei();    //��λȫ���жϱ�־λ����TWI�жϿ��Է���		
	KeyScan();
	if(TransOk == FALSE)
	{
		PORTA = 0x00;
		_delay_ms(500);
	}
	PORTA = 0xFF;
	GICR |= (1<<INT0);    
}

/**********************************************************************************************
* �� �� ��  ��KeyScan
* ����˵��������ɨ�躯��
* ���������void
* �� �� ֵ ��void
*********************************************************************************************/
void KeyScan(void)
{
    if((PINB&0x3f) != 0x3f)
	{
	     _delay_ms(10);
		 if((PINB&0x3f) != 0x3f)
		 {
		     keyValue = PINB;
			 PORTB = 0x3f;
			 KeyHandle();
		 }
	}          
}

/**********************************************************************************************
* �� �� ��  ��KeyHandle
* ����˵��������������
* ���������void
* �� �� ֵ ��void
*********************************************************************************************/
void KeyHandle(void)
{
    switch(keyValue)
	{
	    case WHITE:          //�׵ư�������
		{
			PORTA &= ~(1<<3);           //�����׵�		
			
			/* ��Ҫ���͵�����д�뻺������ */
			writedata[0] = 0x02;
			writedata[1] = CODE_W;
			WordAddr = 0x1E;
			TWISTART();    //����TWI����
			_delay_ms(50);
			/* ��Ҫ���͵�����д�뻺������ */
			writedata[0] = 0xF1;
			writedata[1] = 0x44;
			WordAddr = 0x20;
			TWISTART();//����TWI����

			PORTA |= (1<<3);	
			keyValue = 0x00;
			break;                    
		}
			
		case BLUE:          //���ư�������
		{	
			PORTA &= ~(1<<2);    //��������
				
			/* ��Ҫ���͵�����д�뻺������ */
			writedata[0] = 0x02;
			writedata[1] = CODE_B;
			WordAddr = 0x1E;
			TWISTART();    //����TWI����
			_delay_ms(50);
			/* ��Ҫ���͵�����д�뻺������ */
			writedata[0] = 0xF2;
			writedata[1] = 0x44;
			WordAddr = 0x20;
			TWISTART();//����TWI����
			
			PORTA |= (1<<2);
			keyValue = 0x00;
			break;                    
		}
			
		case RED:          //��ư�������
		{
			PORTA &=~ (1<<1);    //�������
				
			/* ��Ҫ���͵�����д�뻺������ */
			writedata[0] = 0x02;
			writedata[1] = CODE_R;
			WordAddr = 0x1E;
			TWISTART();    //����TWI����
			_delay_ms(50);
			/* ��Ҫ���͵�����д�뻺������ */
			writedata[0] = 0xF3;
			writedata[1] = 0x44;
			WordAddr = 0x20;
			TWISTART();//����TWI����
						
			PORTA |= (1<<1);
			keyValue = 0x00;
            break;                    
		}

		case YGD1:          //Ԥ���1��������
		{
			PORTA &= ~(1<<0);    //�����Ƶ�
				
			/* ��Ҫ���͵�����д�뻺������ */
			writedata[0] = 0x01;
			writedata[1] = CODE_Y;
			WordAddr = 0x1E;
			TWISTART();    //����TWI����
			_delay_ms(50);
			/* ��Ҫ���͵�����д�뻺������ */
			writedata[0] = 0xF1;
			writedata[1] = 0x44;
			WordAddr = 0x20;
			TWISTART();//����TWI����
			
			PORTA |= (1<<0);	
			keyValue = 0x00;
            break;                    
		}
			
		case YGD2:          //Ԥ���2��������
		{
			PORTA &= ~(1<<0);	

			/* ��Ҫ���͵�����д�뻺������ */
			writedata[0] = 0x01;
			writedata[1] = CODE_Y;
			WordAddr = 0x1E;
			TWISTART();    //����TWI����
			_delay_ms(50);
			/* ��Ҫ���͵�����д�뻺������ */
			writedata[0] = 0xF2;
			writedata[1] = 0x44;
			WordAddr = 0x20;
			TWISTART();//����TWI����
			
			PORTA |= (1<<0);	
			keyValue=0x00;
			break;                    
		}
			
		case YGD3:          //Ԥ���3��������
		{
			PORTA &= ~(1<<0);	
				
			/* ��Ҫ���͵�����д�뻺������ */
			writedata[0] = 0x01;
			writedata[1] = CODE_Y;
			WordAddr = 0x1E;
			TWISTART();    //����TWI����
			_delay_ms(50);
			/* ��Ҫ���͵�����д�뻺������ */
			writedata[0] = 0xF3;
			writedata[1] = 0x44;
			WordAddr = 0x20;
			TWISTART();//����TWI����
			
			PORTA |= (1<<0);	
			keyValue = 0x00;
            break;                    
		}
		default: break;
	}
}

/******************************************************************************************
* �� �� ��  ��TWI_Init
* ����˵����TWI��ʼ�� --> �����ʡ�TWCR
* ���������void
* �� �� ֵ ��void
*******************************************************************************************/
void TWI_Init()
{
	TWSR = 0;    //Ԥ��Ƶ����Ϊ0
	TWBR = 15;    //Fscl = FOSC/(16+2��TWBR��4^TWPS)
	TWCR = (1<<TWEN)|(1<<TWIE);    //ʹ��TWI�ӿڡ�TWI�ж�; ����λ����
}

/******************************************************************************************
* ����˵����TWI�жϺ�����MTģʽ
* ���������void
* �� �� ֵ ��void
******************************************************************************************/
ISR(TWI_vect)
{
	uint8_t i;

	switch(TW_STATUS)
	{
		case TW_START:    //START�ѷ���
		{
			/* ����SLA+W */
			TWDR = DEVICEADDR;
			TWCR = ((1<<TWINT)|(1<<TWEN)|(1<<TWIE));     //д1���TWINT��־λ������TWI����
			break;
		}
		case TW_REP_START:    //RESTART�ѷ���
		{
			/* ����SLA+W */
			TWDR = DEVICEADDR;
			TWCR = ((1<<TWINT)|(1<<TWEN)|(1<<TWIE));     //д1���TWINT��־λ������TWI����
			break;
		}
		case TW_MT_SLA_ACK:    //SLA+W�ѷ����ҽ��յ�ACK
		{
			/* �������� */
			TWDR = WordAddr;    //ADDR
			TWCR = ((1<<TWINT)|(1<<TWEN)|(1<<TWIE));    //д1���TWINT��־λ������TWI����
			break;
		}
		case TW_MT_SLA_NACK:    //SLA+W�ѷ����ҽ��յ�NACK
		{
			/* RESTART */
			TWCR = ((1<<TWSTA)|(1<<TWEN)|(1<<TWIE)|(1<<TWINT));
			break;
		}
		case TW_MT_DATA_ACK:    //�����ѷ����ҽ��յ�ACK
		{
			/* ������������ */
			for(i=0; i<2; i++)
			{
				TWDR = writedata[i];
				TWCR = ((1<<TWINT)|(1<<TWEN)|(1<<TWIE));     //д1���TWINT��־λ������TWI����
				while (!(TWCR & (1<<TWINT)));    //TWINT��λ��ʾ���յ��ӻ���ACK����˿��Բ��������жϼ���ִ��forѭ��
			}
			TransOk = TRUE;
			TWCR = ((1<<TWSTO)|(1<<TWEN)|(1<<TWIE)|(1<<TWINT));    //����STOP
			break;
		}
		case TW_MT_DATA_NACK:    //�����ѷ����ҽ��յ�NACK
		{
			/* RESTART */
			TWCR = ((1<<TWSTA)|(1<<TWEN)|(1<<TWIE)|(1<<TWINT));
			break;
		}
		case TW_MT_ARB_LOST:    //SLA+W�����ݵ��ٲ�ʧ��
		{
			/* �Զ�����ӻ�ģʽ */
			//TWCR = ((1<<TWSTA)|(1<<TWEN)|(1<<TWIE)|(1<<TWINT));
			break;
		}
		
		default: break;
	}
}

/************************************************************************************************
* �� �� �� ��SystemInit
* ����˵����ϵͳ��ʼ������ \ �˿ڳ�ʼ�������Ź���ʼ�����ⲿ�жϳ�ʼ����TWI��ʼ��
* ���������void
* �� �� ֵ ��void
************************************************************************************************/
void SystemInit(void)
{
	WDR();
	Port_Init();
	WDT_Init();
	INT0_Init();
	TWI_Init();
	sei();
}