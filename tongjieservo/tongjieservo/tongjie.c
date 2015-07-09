
#include "my_avr_gcc.h"
/*******************************************************************************/
/**
fork_jiance()函数更改，加入判断前后限位

如：if(((PINK&0x04)==0&&Motor3Status==MOTOR_FORWARD)||((PINK&0x08)==0&&Motor3Status==MOTOR_BACKWARD))
    若有问题可将Motor3Status==MOTOR_FORWARD与Motor3Status==MOTOR_BACKWARD对调


*/

/**************************引脚变更说明2015.2.26**************************************************************************************************
motor1:
	  oc1A->pulse（差动信号）
	  PORTL:2->enable
	  PortL:3->sign（差动信号）
	
motor2:
	  oc1B->pulse（差动信号）
	  PORTL:4->enable
	  PortL:5->sign（差动信号）
	
/**************************输入方式说明2015.2.26**************************************************************************************************
参考文档6.2.1
PT模式位置命令：
脉波型式：2 脉波列 + 符号
滤波宽度：1
逻辑型式：0 正逻辑
外部脉波输入来源：1 低速差动信号 500KPPS

接线方式参考3.3

/**************************函数变更说明2015.2.26**************************************************************************************************
添加中断函数
ISR(TIMER1_OVF_vect) 定时器1溢出中断，用来生成pwm波,定时周期2um，即500KHZ。（待测试）

更改
motor1_start
motor1_stop


motor2_start
motor2_stop

timer1_init 定时生成差动信号

motor1_slow 无效
motor2_slow 无效


由于驱动器可手动设定为位置模式和速度模式，现采用位置模式，手柄控制协议对1,2号电机速度控制无效

另外在handleAction函数中motor_stop()之后是否需要将对应的motor_step清零？

/**************************变量变更说明2015.2.26**************************************************************************************************

/**************************引脚说明2014**************************************************************************************************
motor1:
	  oc1A/PB5->pwm
	  PORTL:2->enable
	  PortL:3->方向
	  ICP1->霍尔元件脉冲检测
motor2:
	  oc1B/PB6->pwm
	  PORTL:4->enable
	  PortL:5->方向
	  ICP3->霍尔元件脉冲检测
motor3:
	  oc3A->pwm
	  PORTL:6->enable
	  PortL:7->方向
	  ICP4->霍尔元件脉冲检测
motor4:
	  oc3B->pwm
	  PORTD:0->enable
	  PortD:1->方向
	  ICP5->霍尔元件脉冲检测
motor5:
	  oc4A->pwm
	  PORTD:6->enable
	  PortD:7->方向
	  外部芯片->霍尔元件脉冲检测
motor6:
	  oc4B->pwm
	  PORTB:3->enable
	  PortB:4->方向
	  外部芯片->霍尔元件脉冲检测
	  
	  
	  
	  
LED:
	PB:0,1,2
	PH:5,6
	PE:6
	
	
	
限位检测：

	PK:0,1
	   2,3
	   4,5
	   6,7
	
通信协议：UART0：	波特率9600,8,1，无校验。
					数据格式：所有数据以ASCII表示
							  motor ID：一个字节，0x31表示1号，依次类推
							  开始/停止:一个字节，0x31表示开始，0x30停止
							  方向：    一个字节，0x30后退，0x31 前进
							  步数：     九个字节，如000001000步，为0x303030303031303030
							  结束符：   一个字节，0X0D

								示例：字符串形式“1111000\r” 表示1号前进1000步
									  16进制0x31 31 31 31 30 30 30 0D
									  
									  
					增加设置速度命令：
					           五个字节，0x0d结尾，前四位为速度，0-1023.如0256\r.



***************************************************************************************************************************************/

#define PULSE_WIDTH 0x1ff

#define MOTOR_FORWARD 0X31                                 //方向向前
#define MOTOR_BACKWARD 0X30								//方向后退
#define MOTOR_STOP 0X32								//停止


#define MOTOR1  0X31
#define MOTOR2  0X32
#define MOTOR3  0X33
#define MOTOR4  0X34
#define MOTOR5  0X35
#define MOTOR6  0X36

#define START   0X31
#define STOP    0x30

#define SLOW_STEP 10                            //距终点30减速

#define TRUE 1
#define FALSE 0


#define FOSC 11059200// Clock Speed
#define BAUD 9600
#define MYUBRR (FOSC/16/BAUD-1)


#define UART0_RECV_BUFFER_LENGTH 13
#define UART1_RECV_BUFFER_LENGTH 4



typedef struct UART0_COMMAND
{
	unsigned char MotorID;
	unsigned char MotorStart;
	unsigned char MotorDirection;
	unsigned char MotorStep[9];
	unsigned char End;


}MOTOR_COMMAND;

MOTOR_COMMAND motor_command;



/************************************************中断函数声明***********************************************************************/
/*
#pragma interrupt_handler uart0_isr:26                         //uart0接收中断26
#pragma interrupt_handler uart1_isr:37                         //uart0接收中断37

#pragma interrupt_handler timer1CAP_isr:17                     //定时器1脉冲检测中断
#pragma interrupt_handler timer3CAP_isr:32                     //定时器3脉冲检测中断
#pragma interrupt_handler timer4CAP_isr:42                     //定时器4脉冲检测中断
#pragma interrupt_handler timer5CAP_isr:47                     //定时器5脉冲检测中断*/

/***********************************************************************************************************************************/
unsigned char UART0_RECV_FLAG;                                //uart0 接收成功标志位，指令接收成功置1
unsigned char UART1_RECV_FLAG; 									//uart1 接收成功标志位，指令接收成功置1
unsigned char UART0_RECV_BUFFER[UART0_RECV_BUFFER_LENGTH];									//数据包存放位置
unsigned char UART1_RECV_BUFFER[UART0_RECV_BUFFER_LENGTH];
unsigned char UART0_RECV_INDEX=0;                             //uart0,接收数据索引，用于串行接收
unsigned char UART1_RECV_INDEX=0;								//uart1,接收数据索引，用于串行接收


unsigned char Motor1Status;
unsigned char Motor2Status;
unsigned char Motor3Status;
unsigned char Motor4Status;
unsigned char Motor5Status;
unsigned char Motor6Status;

unsigned char Motor1Direct;
unsigned char Motor2Direct;

long Position1;
int Direct1;

long Position2;
int Direct2;

long Position3;
int Direct3;

long Position4;
int Direct4;

long Position5;
int Direct5;

long motor1Step;
long motor2Step;
long motor3Step;
long motor4Step;
long motor5Step;
long motor6Step;

long motor1CommandStep;
long motor2CommandStep;
long motor3CommandStep;
long motor4CommandStep;
long motor5CommandStep;
long motor6CommandStep;

char Motor1CommandStep[10];
char Motor2CommandStep[10];
char Motor3CommandStep[10];
char Motor4CommandStep[10];
char Motor5CommandStep[10];
char Motor6CommandStep[10];

unsigned char global_yon_motor1_running;
unsigned char global_yon_motor2_running;
unsigned char global_yon_motor3_running;
unsigned char global_yon_motor4_running;
unsigned char global_yon_motor5_running;
unsigned char global_yon_motor6_running;

int global_pwm;
long global_pwm_c=255;               //电脑上位机控制



/////////////////////////////////////////////////
//函数声明
/////////////////////////////////////////////////

void USART0_Init( unsigned int ubrr);
void USART0_Transmit( unsigned char data );
void USART1_Init( unsigned int ubrr);
void USART1_Transmit( unsigned char data );
void motor1_start(char direct);
void motor1_stop();
void motor1_slow();
void motor1_speed(int pwm);

void motor2_start(char direct);
void motor2_stop();
void motor2_slow();
void motor2_speed(int pwm);

void motor3_start(char direct);
void motor3_stop();
void motor3_slow();
void motor3_speed(int pwm);

void motor4_start(char direct);
void motor4_stop();
void motor4_slow();
void motor4_speed(int pwm);

void motor5_start(char direct);
void motor5_stop();
void motor5_slow();
void motor5_speed(int pwm);

void motor6_start(char direct);
void motor6_stop();
void motor6_slow();
void motor6_speed(int pwm);

void timer1_init();
void timer3_init();
void timer4_init();
void timer5_init();

void init_device(void);

void  delayms(volatile unsigned int n);

void fork_uart0_command();
void fork_uart1_command();
void handleAction(char buffer);
void fork_step_count();

void fork_jiance();
void USART0_Transmit_Str(char *data);


/*****************************************中断函数**********************************************************************************/



ISR(TIMER1_CAPT_vect)
{
	motor1Step++;
	
	if(Motor1Status==MOTOR_FORWARD)
	{
		Position1++;
	}
	else if(Motor1Status==MOTOR_BACKWARD)
	{
		Position1--;
	}

}

ISR(TIMER3_CAPT_vect)
{
	motor2Step++;
		if(Motor2Status==MOTOR_FORWARD)
	{
		Position2++;
	}
	else if(Motor2Status==MOTOR_BACKWARD)
	{
		Position2--;
	}

}

ISR(TIMER4_CAPT_vect)
{
	motor3Step++;
	if(Motor3Status==MOTOR_FORWARD)
	{
		Position3++;
	}
	else if(Motor3Status==MOTOR_BACKWARD)
	{
		Position3--;
	}

}
ISR(TIMER5_CAPT_vect)
{
	motor4Step++;
	if(Motor4Status==MOTOR_FORWARD)
	{
		Position4++;
	}
	else if(Motor4Status==MOTOR_BACKWARD)
	{
		Position4--;
	}

}


ISR(USART1_RX_vect)
   {
    unsigned char temp;
	
	temp=UDR1;
	UART1_RECV_BUFFER[UART1_RECV_INDEX]=temp;
	
	if(UART1_RECV_INDEX==3)
	   {
	     UART1_RECV_FLAG=1;
		 UART1_RECV_INDEX=0;
	   
	   }
	   
	 else
	 {
		 UART1_RECV_INDEX++;
		 UART1_RECV_FLAG=0;
	 
	 }
//	PORTB |=(1<<0);
	//uart has received a character in UDR
  /*  if (global_yon_process_message == TRUE)
       {
        temp=UDR1;
        process_message(temp);
	   }
	else
	   {	 
	   global_232_receive_buffer[0]=UDR1;
       global_yon_232_receive=TRUE;     
	   }*/
   }


ISR(USART0_RX_vect)
    {
    unsigned char temp;
	unsigned char pwm[5];

	temp=UDR0;
	UART0_RECV_BUFFER[UART0_RECV_INDEX]=temp;
	if(temp==0x0D)
	{
		if(UART0_RECV_INDEX>=10)
		{
			UART0_RECV_FLAG=1;
		
		}
		else
		{
			memcpy(pwm,UART0_RECV_BUFFER,4);
			global_pwm_c=atol(pwm);
			
			if(global_pwm_c>1023)
				global_pwm_c=1023;
		//global_pwm_c=1023;
		
		}
		UART0_RECV_INDEX=0;
	}
	else
	{
		
		UART0_RECV_INDEX++;
		UART0_RECV_FLAG=0;
	}


   }
/************************************************************************************************************************************/
void USART0_Init( unsigned int ubrr){
	/* Set baud rate */
	UBRR0H = (unsigned char)(ubrr>>8);
	UBRR0L = (unsigned char)ubrr;
	
	/* Enable receiver and transmitter */
	UCSR0B = (1<<RXEN0)|(1<<TXEN0)|(1<<7);  //允许中断
	/* Set frame format: 8data, 1stop bit */
	UCSR0C = 0x06;
} // USART_Init




void USART0_Transmit( unsigned char data )
{
	/* Wait for empty transmit buffer */
	while ( !( UCSR0A & (1<<UDRE0)) )
	;
	/* Put data into buffer, sends the data */
	UDR0 = data;
}

void USART1_Init( unsigned int ubrr){
	/* Set baud rate */
	UBRR1H = (unsigned char)(ubrr>>8);
	UBRR1L = (unsigned char)ubrr;

	/* Enable receiver and transmitter */
	UCSR1B = (1<<RXEN1)|(1<<TXEN1)|(1<<7);  //允许中断
	/* Set frame format: 8data, 1stop bit */
	UCSR1C = 0x06;
} // USART_Init

void USART1_Transmit( unsigned char data )
{
	/* Wait for empty transmit buffer */
	while ( !( UCSR1A & (1<<UDRE1)) )
	;
	/* Put data into buffer, sends the data */
	UDR1 = data;
}


/***************************************motor1********************************************************************/
void motor1_start(char direct)
{

	motor1Step=0;
	
	OCR1AH=global_pwm_c>>8;
	OCR1AL=global_pwm_c&0xff;
	//OCR1A = PULSE_WIDTH;
	DDRL |=(1<<2)|(1<<3) ;
	
	//设置方向
	if(direct=='0')
		
		PORTL &=~(1<<3);
	else if(direct=='1')
		PORTL |=(1<<3);

	PORTL |=(1<<2);     //enable
		
	TCCR1A|=1<<7;                 //启动PWM
	PORTH &=0xFE<<5;            //LED
	
}

void motor1_stop()
{    

	PORTL &=~(1<<2);    //disable
	TCCR1A &=~(1<<7);                 //停止PWM
	PORTH |=0x01<<5;            //LED

}

void motor1_slow()
{
	OCR1BH= PULSE_WIDTH/2>>8;
	OCR1BL = PULSE_WIDTH/2;	//Load Pulse width
	
	
	PORTL |=(1<<2);     //enable
	TCCR1A|=1<<7;                 //启动PWM

}

void motor1_speed(int pwm)
{
	OCR1AH= pwm>>8;
	OCR1AL = pwm;	//Load Pulse width
	//OCR1AH= 3;
	//OCR1AL = 255;	//Load Pulse width
}

/***************************************motor2********************************************************************/




void motor2_start(char direct){


	motor2Step=0;
	OCR1BH=global_pwm_c>>8;
	OCR1BL=global_pwm_c&0xff;
	//OCR1B = PULSE_WIDTH;
	DDRL |=(1<<4)|(1<<5) ;
	//设置方向
	if(direct=='0')
		PORTL &=~(1<<5);
	else if(direct=='1')
		PORTL |=(1<<5);	
	
	PORTL |=(1<<4);     //enable
	

	TCCR1A|=1<<5;                //启动PWM
	PORTH &=0xFE<<6;
}

void motor2_stop()
{
	PORTL &=~(1<<4);
	TCCR1A &=~(1<<5);                 //停止PWM
	 
	PORTH |=0x01<<6;   

}

void motor2_slow()
{
	OCR1AH = PULSE_WIDTH/2>>8;	//Load Pulse width
	OCR1AL = PULSE_WIDTH/2;	//Load Pulse width
	PORTL |=(1<<4);     //enable
	TCCR1A|=1<<5;
	

}

void motor2_speed(int pwm)
{
	
	OCR1BH= pwm>>8;
	OCR1BL = pwm;	//Load Pulse width

}
/***************************************motor3********************************************************************/




void motor3_start(char direct){


	motor3Step=0;
	
	//OCR3A = PULSE_WIDTH;
	DDRL |=(1<<6)|(1<<7) ;

		OCR3AH=global_pwm_c>>8;
	OCR3AL=global_pwm_c&0xff;

		PORTL |=(1<<6);     //enable

	
		//设置方向
	if(direct=='0')
		PORTL &=~(1<<7);
	else if(direct=='1')
	{
			PORTL &=~(1<<7);
			_delay_ms(1000);
		PORTL |=(1<<7);
	}


		
		
	TCCR3A|=1<<7;                 //启动PWM

	PORTB &=0xFE<<0;

	
}

void motor3_stop()
{
	PORTL&=~(1<<6);     //disble
	TCCR3A &=~(1<<7);
	
	PORTB |=0x01<<0;

}

void motor3_slow()
{
	
	OCR3AH = PULSE_WIDTH/2>>8;
	OCR3AL = PULSE_WIDTH/2;	//Load Pulse width
	PORTL |=(1<<6);     //enable
	TCCR3A|=1<<7;
}

void motor3_speed(int pwm)
{
	
	OCR3AH= pwm>>8;
		
	OCR3AL = pwm;	//Load Pulse width

}
/***************************************motor4********************************************************************/




void motor4_start(char direct){

	motor4Step=0;
	OCR3BH=global_pwm_c>>8;
	OCR3BL=global_pwm_c&0xff;
	//OCR3B = PULSE_WIDTH;
	DDRD |=(1<<0)|(1<<1) ;
	//设置方向
	if(direct=='0')
		PORTD &=~(1<<1);
	else if(direct=='1')
		PORTD |=(1<<1);
	
	PORTD |=(1<<0);     //enable
		
		
	TCCR3A|=1<<5;

	PORTB &=~(0x01<<1);
	

	
}

void motor4_stop()
{
	PORTD &=~(1<<0);     //disble
	TCCR3A &=~(1<<5);
	
	PORTB |=0x01<<1;

}

void motor4_slow()
{
	
	OCR3B = PULSE_WIDTH/2;	//Load Pulse width
	PORTD |=(1<<0);    //enable
	TCCR3A|=1<<5;
}

void motor4_speed(int pwm)
{
	
	OCR3BH= pwm>>8;
		
	OCR3BL = pwm;	//Load Pulse width

}
/***************************************motor5********************************************************************/




void motor5_start(char direct){

	motor5Step=0;
	
	OCR4AH=global_pwm_c>>8;
	OCR4AL=global_pwm_c&0xff;
	//OCR4A = PULSE_WIDTH;
	DDRD |=(1<<6)|(1<<7) ;
	//设置方向
	if(direct=='0')
		PORTD &=~(1<<7);
	else if(direct=='1')
		PORTD |=(1<<7);
	
	PORTD |=(1<<6);     //enable
	
	
	TCCR4A|=1<<7;
	PORTB &=0xFE<<2;
}

void motor5_stop()
{
	
	PORTD &=~(1<<6);
	TCCR4A &=~(1<<7);
	PORTB |=0x01<<2;

}

void motor5_slow()
{

	
	OCR4AH=PULSE_WIDTH/2>>8;
	OCR4AL = PULSE_WIDTH/2;	//Load Pulse width
	

	PORTD |=(1<<6);     //enable
	TCCR4A|=1<<7;

}

void motor5_speed(int pwm)
{
	
	OCR4AH= pwm>>8;
		
	OCR4AL = pwm;	//Load Pulse width

}
/***************************************motor6********************************************************************/




void motor6_start(char direct){

	motor6Step=0;
	
	OCR4BH=global_pwm_c>>8;
	OCR4BL=0xff;
	//OCR4B = PULSE_WIDTH;
	DDRB |=(1<<3)|(1<<4) ;
	//设置方向
	if(direct=='0')
		PORTB &=~(1<<4);
	else if(direct=='1')
		PORTB |=(1<<4);
	
	
	PORTB |=(1<<3);
	
	
	TCCR4A|=1<<5;
	PORTE &=0xFE<<6;
	
}

void motor6_stop()
{
	
	PORTB &=~(1<<3);  
	TCCR4A &=~(1<<5);
	PORTE |=0x01<<6;  

}

void motor6_slow()
{
	
	OCR4BH = PULSE_WIDTH/2>>8;
	OCR4BL = PULSE_WIDTH/2;	//Load Pulse width
	
	PORTB |=(1<<3);
	TCCR4A|=1<<5;

}
void motor6_speed(int pwm)
{
	
	OCR4BH= pwm>>8;
		
	OCR4BL = pwm;	//Load Pulse width

}
/****************************************timer1 init**************************************************************/
void timer1_init()
{
	DDRB |= (1<<5)|(1<<6);		//PortD.5 as o/p

	//OCR1AH= PULSE_WIDTH>>8;
		
	//OCR1AL = PULSE_WIDTH;	//Load Pulse width
	OCR1AH=0;
	OCR1AL=255;

	//OCR1BH = PULSE_WIDTH>>8;	//Load Pulse width
	//OCR1BL = PULSE_WIDTH;	//Load Pulse width
	OCR1BH=0;
	OCR1BL=255;
	
	
	//DDRD& =(0xff<<4);                   //PD4输入
	TCCR1A = 0x03;
	TCCR1B = 0x09;                 //start Timer
	TIMSK1	|=(1<<ICIE1);            //允许脉冲捕获中断

}

/*void timer1_init()
{
	DDRB |= (1<<5)|(1<<6);		//PortD.5 as o/p

	//OCR1AH= PULSE_WIDTH>>8;
		

	
	TCNT1H=0xff;
	TCNT1L=0xe7;
	//DDRD& =(0xff<<4);                   //PD4输入
	TCCR1A = 0x00;
	TCCR1B = 0x01;                 //start Timer
	TIMSK1	|=(1<<TOIE1);            //允许脉冲捕获中断

}*/

void timer3_init()
{
	DDRE |=(1<<3)|(1<<4);

	
	//OCR3AH = PULSE_WIDTH>>8;	//Load Pulse width
	//OCR3AL = PULSE_WIDTH;	//Load Pulse width

	//OCR3BH = PULSE_WIDTH>>8;	//Load Pulse width
	//OCR3BL = PULSE_WIDTH;	//Load Pulse width

	OCR3AH=0;
	OCR3AL=64;
	OCR3BH=0;
	OCR3BL=64;


	DDRE&=~(1<<7);                   //PD4输入
	TCCR3A = 0x03;
	TCCR3B = 0x09; //start Timer
	TIMSK3	|=(1<<ICIE3);            //允许脉冲捕获中断

}

void timer4_init()
{
	DDRH |=(1<<3)|(1<<4);

	//OCR4AH = PULSE_WIDTH>>8;	//Load Pulse width
	//OCR4AL = PULSE_WIDTH;	//Load Pulse width

	//OCR4BH = PULSE_WIDTH>>8;	//Load Pulse width
	//OCR4BL = PULSE_WIDTH;	//Load Pulse width
	
	OCR4AH=1;
	OCR4AL=255;
	OCR4BH=1;
	OCR4BL=255 ;

	DDRL&=~(1<<0);                   //PD4输入
	TCCR4A = 0x03;
	TCCR4B = 0x09; //start Timer
	TIMSK4	|=(1<<ICIE4);            //允许脉冲捕获中断

}


void timer5_init()
{
	
	//DDRL & =(0xff<<1);                   //PD4输入
	TCCR5A = 0x00;
	TCCR5B = 0x09; //start Timer
	TIMSK5	|=(1<<ICIE5) ;           //允许脉冲捕获中断

}

void init_device(void)
{
 //stop errant interrupts until set up
 cli(); //disable all interrupts
 //XDIV  = 0x00; //xtal divider
 XMCRA = 0x00; //external memory
 
 //INIT PROT
 DDRB=0XFF;
 PORTB=0X07;
 
 DDRE=0XFF;
 PORTE=0X40;
 
 DDRH=0XFF;
 PORTH=0X60;

 //watchdog_init();
 PORTL=0X00;
 DDRL=0XFC;     //PL0,1输入，其他输出
 
 PORTK=0Xff;
 DDRK=0X00;
 
 //timer1_init();  //停止状态机
 
 USART0_Init(MYUBRR);

 USART1_Init(MYUBRR);

 /*uart0_init();
 uart1_init();*/
 //MCUCR = 0x00;
 timer1_init();
 timer3_init();
 timer4_init();
 timer5_init();
 
 UART0_RECV_FLAG=0;
 UART1_RECV_FLAG=0;

  Position1=0;


 Position2=0;


 Position3=0;
 
 Position4=0;


 Position5=0;


 motor1Step=0;
 motor2Step=0;
 motor3Step=0;
 motor4Step=0;
 motor5Step=0;
sei(); //re-enable interrupts
 //all peripherals are now initialized


}

void  delayms(volatile unsigned int n)       
{ long i;
  long j;
  for(i=0;i<100;i++)
  {
  	  for(j=n;j>0;j--)
	  {}
  	} 
 
} 

void delayus(volatile unsigned int n)
{
   long i;
   long j;
   for(i=0;i<2;i++)
   {
      for(j=n;j>0;j--)
	  {}
   }


}

void fork_uart0_command()
{
    int i;
		if(UART0_RECV_FLAG==1)
	{
		memcpy(&motor_command,UART0_RECV_BUFFER,sizeof(UART0_RECV_BUFFER));
		switch(motor_command.MotorStart)
		{
			case START:
			{
				switch(motor_command.MotorID)
				{
					case MOTOR1:
					global_yon_motor1_running=TRUE;
					motor1_start(motor_command.MotorDirection);
					Motor1Status=motor_command.MotorDirection;
					memcpy(Motor1CommandStep,&motor_command.MotorStep,sizeof(motor_command.MotorStep));
					motor1CommandStep=atol(Motor1CommandStep);
					for(i=0;Motor1CommandStep[i]!='\0';i++)
					{
					USART0_Transmit((Motor1CommandStep[i]));
					}
					break;
					
					case MOTOR2:
					global_yon_motor2_running=TRUE;
					motor2_start(motor_command.MotorDirection);
					Motor2Status=motor_command.MotorDirection;
					memcpy(Motor2CommandStep,&motor_command.MotorStep,sizeof(motor_command.MotorStep));
					motor2CommandStep=atol(Motor2CommandStep);
					break;

					case MOTOR3:
					global_yon_motor3_running=TRUE;
					motor3_start(motor_command.MotorDirection);
					Motor3Status=motor_command.MotorDirection;
					memcpy(Motor3CommandStep,&motor_command.MotorStep,sizeof(motor_command.MotorStep));
					motor3CommandStep=atol(Motor3CommandStep);
					break;

					case MOTOR4:
					global_yon_motor4_running=TRUE;
					motor4_start(motor_command.MotorDirection);
					Motor4Status=motor_command.MotorDirection;
					memcpy(Motor4CommandStep,&motor_command.MotorStep,sizeof(motor_command.MotorStep));
					motor4CommandStep=atol(Motor4CommandStep);
					break;

					case MOTOR5:
					global_yon_motor5_running=TRUE;
					motor5_start(motor_command.MotorDirection);
					Motor5Status=motor_command.MotorDirection;
					memcpy(Motor5CommandStep,&motor_command.MotorStep,sizeof(motor_command.MotorStep));
					motor5CommandStep=atol(Motor5CommandStep);
					break;

					case MOTOR6:
					global_yon_motor6_running=TRUE;
					motor6_start(motor_command.MotorDirection);
					Motor6Status=motor_command.MotorDirection;
					memcpy(Motor6CommandStep,&motor_command.MotorStep,sizeof(motor_command.MotorStep));
					motor6CommandStep=atol(Motor6CommandStep);
					break;

				
				}

				
				
			
				break;
			}

			case STOP:
			{
				switch(motor_command.MotorID)
				{
					case MOTOR1:
					motor1_stop();
					global_yon_motor1_running=FALSE;
					Motor1Status=MOTOR_STOP;
					motor1Step=0;
					
					break;
					
					case MOTOR2:
					motor2_stop();
					global_yon_motor2_running=FALSE;
					Motor2Status=MOTOR_STOP;
					motor2Step=0;
					break;

					case MOTOR3:
					motor3_stop();
					global_yon_motor3_running=FALSE;
					Motor3Status=MOTOR_STOP;
					motor3Step=0;
					break;

					case MOTOR4:
					motor4_stop();
					global_yon_motor4_running=FALSE;
					Motor4Status=MOTOR_STOP;
					motor4Step=0;
					break;

					case MOTOR5:
					motor5_stop();
					global_yon_motor5_running=FALSE;
					Motor5Status=MOTOR_STOP;
					motor5Step=0;
					break;

					case MOTOR6:
					motor6_stop();
					global_yon_motor6_running=FALSE;
					Motor6Status=MOTOR_STOP;
					motor6Step=0;
					break;
			
				}
			break;
			}
		}
		UART0_RECV_FLAG=0;
	}
}

void fork_uart1_command()
{
	if(UART1_RECV_FLAG==1)
	{
	   handleAction(UART1_RECV_BUFFER[0]);
	   handleAction(UART1_RECV_BUFFER[1]);
	   handleAction(UART1_RECV_BUFFER[2]);
	   handleAction(UART1_RECV_BUFFER[3]);
		UART1_RECV_FLAG=0;
	
	}

}

void handleAction(char buffer)
{
	unsigned char temp;
    char tempc;

	if((buffer&0b11000000)==0b00000000)
	{
	   global_pwm=0x3FF/32*(buffer&0b00011111);
	   motor1_speed(0x3FF/32*(buffer&0b00011111));
	  // motor2_speed(0x3FF/32*(buffer&0b00011111));
	   if((buffer&0b00011111)==0)
	   {
	    motor1_stop();
		//motor2_stop();
	   }
	   else
	   {
		temp=buffer&0b00100000;
		if (temp==0) 
		   {
		    tempc='0';
		   }
        else 
		   {
		    tempc='1';
		   } 
		motor1_start(tempc);
		//motor2_start(tempc);
	   
	   }
	
	}
	
	else if((buffer&0b11000000)==0b01000000)
	{
		motor2_speed(0x3FF/32*(buffer&0b00011111));
	   
	   if((buffer&0b00011111)==0)
	   {
	    motor2_stop();
		
	   }
	   else
	   {
		temp=buffer&0b00100000;
		if (temp==0) 
		   {
		    tempc='0';
		   }
        else 
		   {
		    tempc='1';
		   } 
		motor2_start(tempc);
	   
	   }
	
	}
	
	else if((buffer&0b11000000)==0b10000000)
	{
		motor3_speed(0x3FF/32*(buffer&0b00011111));
	 
	   if((buffer&0b00011111)==0)
	   {
	    motor3_stop();
		
	   }
	   else
	   {
		temp=buffer&0b00100000;
		if (temp==0) 
		   {
		    tempc='0';
		   }
        else 
		   {
		    tempc='1';
		   } 
		motor3_start(tempc);
	   
	   }
	}
	else if((buffer&0b11000000)==0b11000000)
	{
	
		motor4_speed(0x3FF/32*(buffer&0b00011111));
	  
	   if((buffer&0b00011111)==0)
	   {
	    motor4_stop();
	
	   }
	   else
	   {
    	temp=buffer&0b00100000;
		if (temp==0) 
		   {
		    tempc='0';
		   }
        else 
		   {
		    tempc='1';
		   } 

		motor4_start(tempc);
	   
	   }
	
	}
	


}
void fork_step_count()
{

    if (global_yon_motor1_running==TRUE) {
		if(motor1Step>=motor1CommandStep-fmin(motor1CommandStep*0.05,SLOW_STEP)&&motor1Step<motor1CommandStep)
	{
		motor1_slow();
	 }
	 }

     if (global_yon_motor2_running==TRUE) {
     	if(motor2Step>=motor2CommandStep-fmin(motor2CommandStep*0.05,SLOW_STEP)&&motor2Step<motor2CommandStep)
	{
		motor2_slow();
	 }
	 }

     if (global_yon_motor3_running==TRUE) {
	 	if(motor3Step>=motor3CommandStep-fmin(motor3CommandStep*0.05,SLOW_STEP)&&motor3Step<motor3CommandStep)
	{
		motor3_slow();
	 }
	 }

     if (global_yon_motor4_running==TRUE) {
	 	if(motor4Step>=motor4CommandStep-fmin(motor4CommandStep*0.05,SLOW_STEP)&&motor4Step<motor4CommandStep)
	{
		motor4_slow();
	 }
     }

     if (global_yon_motor5_running==TRUE) {
	 	if(motor5Step>=motor5CommandStep-fmin(motor5CommandStep*0.05,SLOW_STEP)&&motor5Step<motor5CommandStep)
	{
		motor5_slow();
	 }
	 }

     if (global_yon_motor6_running==TRUE) {
	 	if(motor6Step>=motor6CommandStep-fmin(motor6CommandStep*0.05,SLOW_STEP)&&motor6Step<motor1CommandStep)
	{
		motor6_slow();
	 }
	 }

     if (global_yon_motor1_running==TRUE) {
		if(motor1CommandStep<=motor1Step)
	{
		motor1_stop();
        global_yon_motor1_running=FALSE;
		motor1Step=0;
	 }
	 }

     if (global_yon_motor2_running==TRUE) {
	 	if(motor2CommandStep<=motor2Step)  
	{
		motor2_stop();
        global_yon_motor2_running=FALSE;
		motor2Step=0;
	 }
	 }

     if (global_yon_motor3_running==TRUE) {
		if(motor3CommandStep<=motor3Step)
	{
		motor3_stop();
        global_yon_motor3_running=FALSE;
		motor3Step=0;
	 }
	 }

     if (global_yon_motor4_running==TRUE) {
	 	if(motor4CommandStep<=motor4Step)
	{
		motor4_stop();
        global_yon_motor4_running=FALSE;
		motor4Step=0;
	 }
	 }

     if (global_yon_motor5_running==TRUE) {
	 	if(motor5CommandStep<=motor5Step)
	{
		motor5_stop();
        global_yon_motor5_running=FALSE;
		motor5Step=0;
	 }
	 }

     if (global_yon_motor6_running==TRUE) {
	    if(motor6CommandStep<=motor6Step)
	{
		motor6_stop();
        global_yon_motor6_running=FALSE;
		motor6Step=0;
	 }
	 }
}

void fork_jiance()
{
	
	
	if(((PINK&0x01)==0&&Motor1Status==MOTOR_FORWARD)||((PINK&0x02)==0&&Motor1Status==MOTOR_BACKWARD))
	{
		motor1_stop();
		
        global_yon_motor1_running=FALSE;
      
		motor1Step=0;
		
	}
	if(((PINK&0x04)==0&&Motor2Status==MOTOR_FORWARD)||((PINK&0x08)==0&&Motor2Status==MOTOR_BACKWARD))
	{
		motor2_stop();
        global_yon_motor2_running=FALSE;
		motor2Step=0;
	
	}

	if(((PINK&0x10)==0&&Motor4Status==MOTOR_FORWARD)||((PINK&0x20)==0&&Motor4Status==MOTOR_BACKWARD))
	{
		motor3_stop();
        global_yon_motor3_running=FALSE;
		motor3Step=0;
	
	}

/*	if(((PINK&0x40)==0&&Motor5Status==MOTOR_FORWARD)||((PINK&0x80)==0&&Motor5Status==MOTOR_BACKWARD))
	{
		motor4_stop();
        global_yon_motor4_running=FALSE;
		motor4Step=0;
	}*/
	

}

void USART0_Transmit_Str(char *data)

{
  while(*data!='\0')
     USART0_Transmit(*data++);
}
int main()
{
 
	init_device();

    

//	motor1_start(MOTOR_FORWARD);
//	motor3_start(MOTOR_BACKWARD);
//	motor4_start(MOTOR_BACKWARD);
//	motor5_start(MOTOR_FORWARD);
//	motor1_start('1');
//	motor3_start(motor_command.MotorDirection);
	char  des[10];
	while(1)
	{
//	
	    fork_uart0_command();               //处理命令
		fork_uart1_command();
		fork_step_count();					//检测步数，减速，停止
		fork_jiance();                      //限位开关检测

		//sprintf(des,"step:%f\r\n",motor1Step);

		//ltoa(motor1Step,des,10);
		//USART0_Transmit_Str(des);
		//USART0_Transmit_Str("\r\n");
	    //motor1_speed(10);

	}
}


void test(void)
{

	global_yon_motor5_running=TRUE;
	motor5_start('1');
	Motor5Status='1';
	motor5CommandStep=200000;
  
    while (global_yon_motor5_running==TRUE) 
	{
		fork_step_count();					//检测步数，减速，停止
	
	} 
    
	_delay_ms(100);    
  
	global_yon_motor5_running=TRUE;
	motor5_start('0');
	Motor5Status='1';
	motor5CommandStep=200000;
    while (global_yon_motor5_running==TRUE) 
	{
		fork_step_count();					//检测步数，减速，停止
	
	} 
    _delay_ms(100);


}
