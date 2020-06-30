#ifndef __I2C_H__
#define __I2C_H__

#define uchar  unsigned char
#define uint   unsigned int
//#define BIT(bit) (1<<(bit))

#define SDA_H()         PORTC|=BIT(1)       
#define SDA_L()          PORTC&=~BIT(1)   
#define SCL_H()          PORTC|=BIT(0)
#define SCL_L()           PORTC&=~BIT(0)
#define SDA_OUT()    DDRC|=BIT(1)
#define SDA_IN()       DDRC&=~BIT(1)
#define SCL_OUT()    DDRC|=BIT(0)

extern void delay_ms(unsigned int x);
extern void I2C_init(void);
extern void I2C_Start(void);
extern void I2C_Stop(void);
extern void I2C_Ack(uchar ack);
extern uchar I2C_Tack(void);
extern void I2C_Writebyte(uchar dat);
extern uchar I2C_Readbyte(void);
extern uchar Write_4Byte(uchar dadd,uchar wadd,uchar data[]) ;
extern uchar  Write_2Byte1(uchar dadd,uchar wadd,uchar data[]);
extern uchar  Write_2Byte2(uchar dadd,uchar wadd,uchar data[]); 

#endif