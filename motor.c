# include <iom640v.h>
# include <AVRdef.h>

/**************************����˵��**************************************************************************************************
motor1:
	  oc1A->pwm
	  PORTL:2->enable
	  PortL:3->����
	  ICP1->����Ԫ��������
motor2:
	  oc1B->pwm
	  PORTL:4->enable
	  PortL:5->����
	  ICP3->����Ԫ��������
motor3:
	  oc3A->pwm
	  PORTL:6->enable
	  PortL:7->����
	  ICP4->����Ԫ��������
motor4:
	  oc3B->pwm
	  PORTD:0->enable
	  PortD:1->����
	  ICP5->����Ԫ��������
motor5:
	  oc4A->pwm
	  PORTD:6->enable
	  PortD:7->����
	  �ⲿоƬ->����Ԫ��������
motor6:
	  oc4B->pwm
	  PORTB:3->enable
	  PortB:4->����
	  �ⲿоƬ->����Ԫ��������
	  
	  
	  
	  
LED:
	PB:0,1,2
	PH:5,6
	PE:6
	
	
	
��λ��⣺

	PK:0,1
	   2,3
	   4,5
	   6,7
	



***************************************************************************************************************************************/

#define PULSE_WIDTH 0xff

#define MOTOR_FORWARD 0X01                                 //������ǰ
#define MOTOR_BACKWARD 0X00								//�������



#define FOSC 11059200// Clock Speed
#define BAUD 9600
#define MYUBRR (FOSC/16/BAUD-1)




/************************************************�жϺ�������***********************************************************************/

#pragma interrupt_handler uart0_isr:26                         //uart0�����ж�26
#pragma interrupt_handler uart1_isr:37                         //uart0�����ж�37

#pragma interrupt_handler timer1CAP_isr:17                     //��ʱ��1�������ж�
#pragma interrupt_handler timer3CAP_isr:32                     //��ʱ��3�������ж�
#pragma interrupt_handler timer4CAP_isr:42                     //��ʱ��4�������ж�
#pragma interrupt_handler timer5CAP_isr:47                     //��ʱ��5�������ж�

/***********************************************************************************************************************************/
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







/*****************************************�жϺ���**********************************************************************************/
void timer1CAP_isr()
{
	motor1Step++;
	motor2Step++;

}

void timer3CAP_isr()
{
	motor3Step++;

}

void timer4CAP_isr()
{
	motor4Step++;

}
void timer5CAP_isr()
{
	motor5Step++;

}

uart0_isr()
{

}

uart1_isr()
{


}
/************************************************************************************************************************************/
void USART0_Init( unsigned int ubrr){
	/* Set baud rate */
	UBRR0H = (unsigned char)(ubrr>>8);
	UBRR0L = (unsigned char)ubrr;
	/* Enable receiver and transmitter */
	UCSR0B = (1<<RXEN0)|(1<<TXEN0)|(1<<7);  //�����ж�
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
	UCSR1B = (1<<RXEN1)|(1<<TXEN1)|(1<<7);  //�����ж�
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
void motor1_start(int direct){
	
	OCR1A = PULSE_WIDTH;
	DDRL |=(1<<4)|(1<<5) ;
	
	
	PORTL |=(1<<4);     //enable
	//���÷���
	if(direct==0)
		PORTL &=(0xff<<5);
	else if(direct==1)
		PORTL |=(1<<5);

	
}

void motor1_stop()
{
	PORTL &=(0xff<<4);     //enable

}

void motor1_slow()
{
	OCR1B = PULSE_WIDTH/2;	//Load Pulse width
	PORTL |=(1<<4);     //enable

}


/***************************************motor2********************************************************************/




void motor2_start(int direct){
	
	OCR1B = PULSE_WIDTH;
	DDRL |=(1<<2)|(1<<3) ;
	
	
	PORTL |=(1<<2);     //enable
	//���÷���
	if(direct==0)
		PORTL &=(0xff<<3);
	else if(direct==1)
		PORTL |=(1<<3);

	
}

void motor2_stop()
{
	PORTL &=(0xff<<2);     //enable

}

void motor2_slow()
{
	OCR1A = PULSE_WIDTH/2;	//Load Pulse width
	PORTL |=(1<<2);     //enable

}

/***************************************motor3********************************************************************/




void motor3_start(int direct){
	
	OCR3A = PULSE_WIDTH;
	DDRL |=(1<<6)|(1<<7) ;
	
	
	PORTL |=(1<<6);     //enable
	//���÷���
	if(direct==0)
		PORTL &=(0xff<<7);
	else if(direct==1)
		PORTL |=(1<<7);

	
}

void motor3_stop()
{
	PORTL &=(0xff<<6);     //enable

}

void motor3_slow()
{
	OCR3A = PULSE_WIDTH/2;	//Load Pulse width
	PORTL |=(1<<6);     //enable

}


/***************************************motor4********************************************************************/




void motor4_start(int direct){
	
	OCR3B = PULSE_WIDTH;
	DDRD |=(1<<0)|(1<<1) ;
	
	
	PORTD |=(1<<0);     //enable
	//���÷���
	if(direct==0)
		PORTD &=(0xff<<1);
	else if(direct==1)
		PORTD |=(1<<1);

	
}

void motor4_stop()
{
	PORTD &=~(1<<2);     //enable

}

void motor4_slow()
{
	OCR3B = PULSE_WIDTH/2;	//Load Pulse width
	PORTD |=(1<<0);     //enable

}

/***************************************motor5********************************************************************/




void motor5_start(int direct){
	
	OCR4A = PULSE_WIDTH;
	DDRD |=(1<<6)|(1<<7) ;
	
	
	PORTD |=(1<<6);     //enable
	//���÷���
	if(direct==0)
		PORTD &=(0xff<<7);
	else if(direct==1)
		PORTD |=(1<<7);

	
}

void motor5_stop()
{
	PORTD &=~(1<<6);     //enable

}

void motor5_slow()
{
	OCR4A = PULSE_WIDTH/2;	//Load Pulse width
	PORTD |=(1<<6);     //enable

}


/***************************************motor6********************************************************************/




void motor6_start(int direct){
	
	OCR4B = PULSE_WIDTH;
	DDRB |=(1<<3)|(1<<4) ;
	
	
	PORTB |=(1<<3);     //enable
	//���÷���
	if(direct==0)
		PORTB &=~(1<<4);
	else if(direct==1)
		PORTB |=(1<<4);

	
}

void motor6_stop()
{
	PORTB &=~(1<<3);     //enable

}

void motor6_slow()
{
	OCR4B = PULSE_WIDTH/2;	//Load Pulse width
	PORTB|=(1<<3);     //enable

}
/****************************************timer1 init**************************************************************/
void timer1_init()
{
	DDRB |= (1<<5)|(1<<6);		//PortD.5 as o/p	
	OCR1A = PULSE_WIDTH;	//Load Pulse width
	OCR1B = PULSE_WIDTH;	//Load Pulse width
	//DDRD& =(0xff<<4);                   //PD4����
	TCCR1A = 0x83;
	TCCR1B = 0x09; //start Timer
	TIMSK1	|=(1<<ICIE1);            //�������岶���ж�

}

void timer3_init()
{
	DDRE |=(1<<3)|(1<<4);
	OCR3A = PULSE_WIDTH;	//Load Pulse width
	OCR3B = PULSE_WIDTH;	//Load Pulse width
//	DDRE & =(0xff<<7);                   //PD4����
	TCCR3A = 0x83;
	TCCR3B = 0x09; //start Timer
	TIMSK3	|=(1<<ICIE3);            //�������岶���ж�

}

void timer4_init()
{
	DDRH |=(1<<3)|(1<<4);
	OCR4A = PULSE_WIDTH;	//Load Pulse width
	OCR4B = PULSE_WIDTH;	//Load Pulse width
//	DDRL & =(0xff<<0);                   //PD4����
	TCCR4A = 0x83;
	TCCR4B = 0x09; //start Timer
	TIMSK4	|=(1<<ICIE4);            //�������岶���ж�

}


void timer5_init()
{
	
	//DDRL & =(0xff<<1);                   //PD4����
	TCCR5A = 0x00;
	TCCR5B = 0x09; //start Timer
	TIMSK5	|=(1<<ICIE5) ;           //�������岶���ж�

}

void init_device(void)
{
 //stop errant interrupts until set up
 CLI(); //disable all interrupts
 //XDIV  = 0x00; //xtal divider
 XMCRA = 0x00; //external memory
 
 //INIT PROT
 DDRB=0XFF;
 PORTB=0X00;
 
 DDRE=0XFF;
 PORTE=0X00;
 
 DDRH=0XFF;
 PORTH=0X00;

 //watchdog_init();
 PORTL=0X00;
 DDRL=0X3F;     //PL0,1���룬�������
 
 PORTK=0X00;
 DDRK=0X00;
 
 //timer1_init();  //ֹͣ״̬��
 
 USART0_Init(MYUBRR);
 USART1_Init(MYUBRR);
 //MCUCR = 0x00;
 timer1_init();
 timer3_init();
 timer4_init();
 timer5_init();
 
 
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
SEI(); //re-enable interrupts
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
void main()
{
 
	init_device();
	
	
	
	while(1)
	{motor1_start(MOTOR_FORWARD);
	motor3_start(MOTOR_FORWARD);
	motor4_start(MOTOR_FORWARD);
	motor5_start(MOTOR_FORWARD);
	PORTB=0b00000111;
	delayms(1000);
    USART0_Transmit( 'A' );
	 USART1_Transmit( 'A' );
	 motor1_stop();
	 motor3_stop();
	 motor4_stop();
	 motor5_stop();
	}
}