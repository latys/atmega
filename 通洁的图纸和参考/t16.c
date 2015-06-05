// 旋转滤光片
// 硬件变更

// 主板 pin1－7
// 1－ PA7
// 2-  PC6
// 3-  PC7
// 4-  PG2
// 5-  PA6
// 6-  +5V VCC
// 7-  GND
//
// 主板上 D2 POWER LED
//        D3 PG0 LED
//        D8 PG1 LED
//
// LED PCB  PIN1-PIN8
//     PIN1 共阳极LED
//     pin2	led 负极
//     。。。
//     pin8 led 负极
// 


// for single motor
//硬件说明

//PE0 -> LED D7
//PE1 -> LED D6
//PE2 -> NC
//PE3 -> NC
//PE4 -> NC
//PE5 -> NC
//PE6       -> RETB -> BLUE TOOTH RESET
//PE7 -> NC

//PD0 -> NC
//PD1 -> NC
//PD2(RXD1) -> RXD3 -> BLUE TOOTH TXD
//PD3(TXD1) -> TXD3 -> BLUE TOOTH RXD
//PE6       -> RETB -> BLUE TOOTH RESET

//PB0 -> FORWARD_LIMIT1	->J17 PIN1
//PB1 -> REVERSE_LIMIT1 ->J17 PIN3

//PB2 -> FORWARD_LIMIT2	->J18 PIN1
//PB3 -> REVERSE_LIMIT2	->J18 PIN3

//PB4 -> FORWARD_LIMIT3	->J19 PIN1
//PB5 -> REVERSE_LIMIT3	->J19 PIN3

//PB6 -> FORWARD_LIMIT4	->J20 PIN1
//PB7 -> REVERSE_LIMIT4	->J20 PIN3

//PD4 -> REF1  ->J17 PIN5
//PD5 -> REF2  ->J18 PIN5
//PD6 -> REF3  ->J19 PIN5
//PD7 -> REF4  ->J20 PIN5

//PA0 -> P311 -> P31 -> J24 PIN2   motor driver -> cp
//PA1 -> P321 -> P32 -> J24 PIN3   motor driver -> dir
//PA2 -> P331 -> P33 -> J24 PIN4   motor driver -> sleep (NC)
//PA3 -> P411 -> P41 -> J26 PIN2   motor driver -> cp
//PA4 -> P421 -> P42 -> J26 PIN3   motor driver -> dir
//PA5 -> P431 -> P43 -> J26 PIN4   motor driver -> sleep (NC)
//PA6 -> OELIN1L  -> SN74LVCC3245 LOW->EFFECT HIGH->ISOLATION  LIMIT+REF into cpu
//PA7 -> OELIN2L  -> SN74LVCC3245 LOW->EFFECT HIGH->ISOLATION  LIMIT+REF into cpu

//PC0 -> P111 -> P11 -> J22 PIN2   motor driver -> cp        
//PC1 -> P121 -> P12 -> J22 PIN3   motor driver -> dir       
//PC2 -> P131 -> P13 -> J22 PIN4   motor driver -> sleep (NC)
//PC3 -> P211 -> P21 -> J23 PIN2   motor driver -> cp        
//PC4 -> P221 -> P22 -> J23 PIN3   motor driver -> dir       
//PC5 -> P231 -> P23 -> J23 PIN4   motor driver -> sleep (NC)
//PC6 -> NC
//PC7 -> NC

//AREF -> VCC

//VCC                 J16  PIN1
//GND                 J16  PIN2
//PF0    AD0   //NC   J16  PIN3
//PF1    AD1   //NC   J16  PIN4
//PF2    AD2   //NC   J16  PIN5
//PF3    AD3   //NC   J16  PIN6


//MOTOR CONTROL J17-J20: PIN1 FORWARD LIMIT
//                       PIN2 GIND
//                       PIN3 REVERSE LIMIT
//                       PIN4 GIND
//                       PIN5 REF
//                       PIN6 GND
//                       PIN7 +5V
//                       PIN8 GND

//motor driver J22,J23,J24,J26: PIN1 +5V
//                              PIN2 CP
//                              PIN3 DIR
//                              PIN4 SLEEP(NC)
//                              PIN5 GND



// new board 说明
// PE0 RXD0	 TO PC	 
// PE1 TXD0	 TO PC
// PE6 RETB  OUTPUT
//
// PB0 FL3	 INPUT   LOW EFFECT
// PB1 RL3
// PB2 FL2
// PB3 RL2
// PB4 FL1
// PB5 RL1
// PB6 FL0
// PB7 RL0
// 
// PD2 RXD1
// PD3 TXD1
// PD4 RF0   INPUT  LOW EFFECT
// PD5 RF1
// PD6 RF2
// PD7 RF3
//
// PG0 LED D3
// PG1 LED D8
//
// PC0 STEP0
// PC1 DIR0
// PC2 SLEEP0
// PC3 STEP1
// PC4 DIR1
// PC5 SLEEP1
// PC6
// PC7
//
// PA0 SLEEP3	high effect
// PA1 DIR3
// PA2 STEP3
// PA3 SLEEP2
// PA4 DIR2
// PA5 STEP2
// PA6
// PA7
// 
// PF0 AD0
// PF1 AD1
// PF2 AD2
// PF3 AD3


/////////////////////////////////////////////////
// resource alloc
// timer1 for main time slit
// TIMER1 
// 64 DIV  4us/step resolution
//         max 262ms interval step
// change compare would change the motor speed
//
/////////////////////////////////////////////////
/////////////////////////////////////////////////
// GCC application builder : 2011-7-17 by xxx
// Target : at90usb1287
// board name:motor 1106
// Crystal: 16.000Mhz
/////////////////////////////////////////////////

#include "my_avr_gcc.h"  

// follow is the reference software design 
// create this file 2009 04 09
// hardware board for motor board and renishaw encoder
// by xxx
// 时隙设计 50us
// 电机最快速度 20000步每秒，每步间隔1 时隙   每秒电机转动20000/16/200 =6.25 cycle 
// 电机最慢速度 100步  每秒，每步间隔200时隙  每秒电机转动100/16/200 =0.03125 CYCLE ，每圈需要32秒 
// 加减速度设计是每步之间时隙改变量，初步设计约为1 counter

//时隙设计修改为 75us 改慢为了提高最快速度扭力  
// BY XXX  2010 04 17

///////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//global define
//////////////////////////////////////////////////////////////////////

#define MOTOR_NUMBER 4   //TOTAL MOTOR NUMBER

//中断时隙驱动电机，实现下面为标志的状态机
#define MOTOR_STATUS_STOP 0                     //停止状态
#define MOTOR_STATUS_NORMAL_RUN 1               //正常运行
#define MOTOR_STATUS_FORWARD_LIMIT 2            //停止在前限位点
#define MOTOR_STATUS_REVERSE_LIMIT 3            //停止在后限位点
#define MOTOR_STATUS_REFERENCE 4                //停止在参考点
#define MOTOR_STATUS_SLOW_GOTO_FORWARD_LIMIT  5 //慢速运行 走向前限位点
#define MOTOR_STATUS_SLOW_GOTO_REVERSE_LIMIT  6 //慢速运行 走向后限位点
#define MOTOR_STATUS_SLOW_GOTO_REFERENCE      7 //慢速运行 走向参考点
#define MOTOR_STATUS_OTHER    255               //其它状态


#define MOTOR_SPEED_STATUS_RUN 15
#define MOTOR_SPEED_STATUS_STOP 16
#define MOTOR_SPEED_STATUS_ACCEL 17
#define MOTOR_SPEED_STATUS_DECEL 18
#define MOTOR_SPEED_STATUS_SLOW_RUN 19

#define MESSAGE_LENGTH 8

#define PROTOCAL_RECEIVE_BOARD_ID     0
#define PROTOCAL_RECEIVE_MOTOR_NUMBER 1
#define PROTOCAL_RECEIVE_COMMAND      2
#define PROTOCAL_RECEIVE_DIRECTION    3
#define PROTOCAL_RECEIVE_PAR1         4
#define PROTOCAL_RECEIVE_PAR2         5
#define PROTOCAL_RECEIVE_PAR3         6
#define PROTOCAL_RECEIVE_PAR4         7

#define PROTOCAL_SEND_BOARD_ID        0
#define PROTOCAL_SEND_MOTOR_NUMBER    1
#define PROTOCAL_SEND_COMMAND         2
#define PROTOCAL_SEND_STATUS_ID       3
#define PROTOCAL_SEND_PAR1            4
#define PROTOCAL_SEND_PAR2            5
#define PROTOCAL_SEND_PAR3            6
#define PROTOCAL_SEND_PAR4            7

#define COMMAND_RUN                      1
#define COMMAND_READ_STEP_POSITION       2
#define COMMAND_READ_ENCODER_POSITION    3
#define COMMAND_CLEAR_STEP_POSITION      4
#define COMMAND_CLEAR_ENCODER_POSITION   5
#define COMMAND_GOTO_REFERENCE           6
#define COMMAND_GOTO_FORWARD_LIMIT       7
#define COMMAND_GOTO_REVERSE_LIMIT       8
#define COMMAND_STOP                     9
#define COMMAND_SET_SPEED                10
#define COMMAND_SET_ACCEL                11
#define COMMAND_SET_DECEL                12
#define COMMAND_READ_SPEED               13
#define COMMAND_READ_ACCEL               14
#define COMMAND_READ_DECEL               15
#define COMMAND_READ_STATUS              33
#define COMMAND_READ_BOARD_ID           255

#define TRUE 1
#define FALSE 0

#define DIRECTION_FORWARD 1  
#define DIRECTION_REVERSE  0

#define MOTOR_DEFAULT_SPEED 100
#define MOTOR_DEFAULT_ACCEL 100
#define MOTOR_DEFAULT_DECEL 100
#define MOTOR_DEFAULT_SLOW_SPEED 1
#define MOTOR_DEFAULT_SLOW_ACCEL 1
#define MOTOR_DEFAULT_SLOW_DECEL 1

#define TIME_SLOT 29 //50us FOR 19 //75us FOR 29 //100us for 39
//////////////////////////////////////////////////////////////////////

//#define DISABLE_INTERRUPT   (*AT91C_AIC_DCR)|=AT91C_AIC_DCR_GMSK 
//#define ENABLE_INTERRUPT    (*AT91C_AIC_DCR)&=(~(AT91C_AIC_DCR_GMSK))
#define DISABLE_INTERRUPT   cli() 
#define ENABLE_INTERRUPT    sei()

#define RS232_RECEIVE_BUFFER_LENGTH 1
#define RS232_RECEIVE_BUFFER_LENGTH_PROTOCOL 8
#define RS232_SEND_BUFFER_LENGTH_PROTOCOL 20

#define USB_MESSAGE_TYPE 0
#define RS232_MESSAGE_TYPE 1

#define RUN 1
#define STOP 0

//////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////
//global var
////////////////////////////////////////////////////////////

unsigned char global_board_id;             //board id get form switch array
                                           //板子ID号码

unsigned long global_run_step_number[MOTOR_NUMBER];//电机还即将运行的步数
unsigned char global_motor_number;//当前操作电机号码

int global_counter_speed[MOTOR_NUMBER];//电机运行时需要等待的时隙个数
int global_counter_k_speed[MOTOR_NUMBER];//电机运行时速度改变时的计数器K
int global_counter_setting_speed[MOTOR_NUMBER];
int global_counter_low_speed[MOTOR_NUMBER];
int global_counter_low_setting_speed[MOTOR_NUMBER];

long global_motor_position_step[MOTOR_NUMBER];//电机的步数位置
long global_motor_position_encoder[MOTOR_NUMBER];//电机encoder的数据

unsigned char global_motor_status[MOTOR_NUMBER];//电机运行状态
unsigned char global_motor_speed_status[MOTOR_NUMBER];//电机速度状态
unsigned char global_motor_goal_status[MOTOR_NUMBER];//电机目标状态

unsigned char global_motor_direction[MOTOR_NUMBER]; //电机运行方向

unsigned long global_motor_current_speed[MOTOR_NUMBER];//电机当前速度
unsigned long global_motor_current_accel[MOTOR_NUMBER];//电机当前加速度
unsigned long global_motor_current_decel[MOTOR_NUMBER];//电机当前减速度

unsigned long global_motor_setting_speed[MOTOR_NUMBER];//电机设定速度
unsigned long global_motor_setting_accel[MOTOR_NUMBER];//电机设定加速度
unsigned long global_motor_setting_decel[MOTOR_NUMBER];//电机设定减速度
unsigned long global_motor_setting_slow_speed[MOTOR_NUMBER];//电机设定慢速速度
unsigned long global_motor_setting_slow_accel[MOTOR_NUMBER];//电机设定慢速加速度
unsigned long global_motor_setting_slow_decel[MOTOR_NUMBER];//电机设定慢速减速度

unsigned char global_yon_motor_speed_control[MOTOR_NUMBER];//电机是否进行加减速度控制

unsigned char global_receive_buffer[MESSAGE_LENGTH];//接收命令BUFFER
unsigned char global_send_buffer[MESSAGE_LENGTH];//发送命令BUFFER

unsigned char global_usb_receive_buffer[MESSAGE_LENGTH];//USB接口接收BUFFER
//unsigned char global_232_receive_buffer[MESSAGE_LENGTH];//232接口接收BUFFER
unsigned char global_232_receive_buffer[RS232_RECEIVE_BUFFER_LENGTH];//232接口接收buffer
unsigned char global_232_receive_buffer_protocol[RS232_RECEIVE_BUFFER_LENGTH_PROTOCOL];//232接口接收buffer
unsigned char global_232_send_buffer[RS232_SEND_BUFFER_LENGTH_PROTOCOL];//232接口接收buffer
unsigned int global_232_send_buffer_length; //232接口发送缓冲BUFFER的有效内容字节数
unsigned char global_yon_232_receive;//232接口全局标志
unsigned char global_232_receive_byte_counter;

unsigned char global_usb_send_buffer[MESSAGE_LENGTH];//USB接口发送BUFFER
//unsigned char global_232_send_buffer[MESSAGE_LENGTH];//232接口发送BUFFER

unsigned char global_yon_receive;//全局变量接收命令标志
unsigned char global_yon_send;//全局变量发送命令标志
unsigned char global_message_type; //全局变量表示通讯端口方式是 232 还是 USB
unsigned char global_yon_return_message; //是否返回状态message 标志

//unsigned char global_yon_usb_receive;//USB接口是否接收到命令标志
//unsigned char global_yon_232_receive;//232接口是否接收到命令标志
//unsigned char global_yon_usb_send;//USB接口是否发送标志
//unsigned char global_yon_232_send;//232接口是否发送标志

unsigned char global_time_slot_high;
unsigned char global_time_slot_low;

unsigned char global_yon_232_receive_timeout;//232接收超时状态

unsigned char global_yon_already_send_s;
unsigned char global_yon_already_send_t;

unsigned char global_yon_stop_key;

unsigned char global_yon_process_message;
unsigned char global_message;
// system var

//struct _AT91S_CDC 	pCDC;
//extern unsigned char DF_buffer[528];  //memory.c

///////////////////////////////////////////////////////////
//global function
///////////////////////////////////////////////////////////
//hardware function

void one_step(unsigned char motor_number);
void set_direction(unsigned char motor_number,unsigned char direction);
void get_motor_switch_status(void);
void get_motor_position_encoder(void);
int get_motor_position_step(unsigned char motor_number);
void change_speed(unsigned char motor_number,unsigned char motor_speed_status);
void get_board_id(void);

void isr_time_segment(void);
void isr_limit_switch(void);

void init_board(void);   //include board id, usb,device ,I/O,UART,etc
void init_par(void);     //include global par
void init_isr(void);     //include isr interrupt

void watchdog_init(void);
void timer1_init(void);
void uart1_init(void);
void uart0_init(void);

void timer3_init(void);
void timer3_close(void);

/////////////////////////////////////////////////////////////////////
//software function
void fork_main(void);         //check comunication protocal

void send_data(void);
void send_data_usb(void);
void send_data_232_uart0(void);
void send_data_232_uart1(void);
//void send_repeat_232(void); //为了多板级连使用

//void check_usb_receive(void);  //check board id
void check_fork_232_receive(void);  //check board id
void check_fork_232_receive_protocol(void);

void function_run(void);
void function_read_step_position(void);
void function_read_encoder_position(void);
void function_clear_step_position(void);
void function_clear_encoder_position(void);
void function_goto_reference(void);
void function_goto_forward_limit(void);
void function_goto_reverse_limit(void);
void function_stop(void);
void function_set_speed(void);
void function_set_accel(void);
void function_set_decel(void);
void function_read_speed(void);
void function_read_accel(void);
void function_read_decel(void);
void function_read_status(void);
void function_read_board_id(void);

void function_return_message(void);

void status_machine(void);

void send_status(void);
void send_status_old(void);

//system function
void init_var(void);
void init_device(void);
void Wait(unsigned int i);

unsigned char goto_reference(unsigned char number);
void process_message_init(void);
void process_message(unsigned char message);
void process_message_close(void);

void fork_main2(void);

void motor_sleep(unsigned char number);
void motor_unsleep(unsigned char number);

void open_led(unsigned char k);
void close_led(unsigned char k);
void open_power_led(void);
void close_power_led(void);

////////////////////////////////////////////////////////
//Watchdog initialize
// prescale: 512K
void watchdog_init(void)
{
 //wdr (); //this prevents a timeout on enabling
// WDTCR |= (1<<WDCE) | (1<<WDE);/* 30-Oct-2006 Umesh*/  
// WDTCR = 0x0D; //WATCHDOG ENABLED - dont forget to issue WDRs
}

//TIMER3 initialize - prescale:1024
// WGM: 0) Normal, TOP=0xFFFF
// desired value: 500mSec
// actual value: 499.968mSec (0.0%)
void timer3_init(void)
{
 TCCR3B = 0x00; //stop
 TCNT3H = 0xEA; //setup
 TCNT3L = 0xE9;
 OCR3AH = 0x15;
 OCR3AL = 0x17;
 OCR3BH = 0x15;
 OCR3BL = 0x17;
 OCR3CH = 0x15;
 OCR3CL = 0x17;
 ICR3H  = 0x15;
 ICR3L  = 0x17;
 TCCR3A = 0x00;
 TCCR3B = 0x05; //start Timer
}
void timer3_close(void)
{
  TCCR3A =0;
  TCCR3B =0;
}

//TIMER1 initialize - prescale:64
// WGM: 0) Normal, TOP=0xFFFF
// desired value: 100uSec
// actual value: 100.000uSec (0.0%)
void timer1_init(void)
{
 TCCR1B = 0x00; //stop
 TCNT1H = 0xfe; //setup
 TCNT1L = 0xE7;
 OCR1AH = 0x00;
 OCR1AL = 0x19;
 OCR1BH = 0x00;
 OCR1BL = 0x19;
 OCR1CH = 0x00;
 OCR1CL = 0x19;
 ICR1H  = 0x00;
 ICR1L  = 0x19;
 TCCR1A = 0x00;
 TCCR1B = 0x03; //start Timer
}

////////////////////////////////////////////////////////
ISR(TIMER1_OVF_vect)
    {
		 //reload
	     TCCR1A=0;     //stop
		 TCCR1B=0;

         //TIMER1 has overflowed
         TCNT1H = 0xfe; //reload counter high value
         TCNT1L = 0xE7; //reload counter low value
         
		 TCCR1B=0x03;  //start
         
		 // run status machine
         status_machine();
         

}

////////////////////////////////////////////////////////
ISR(TIMER3_OVF_vect)
    {
		 //reload
	     TCCR3A=0;     //stop
		 TCCR3B=0;

         //TIMER1 has overflowed
         //TCNT1H = 0xFF; //reload counter high value
         //TCNT1L = 0xE7; //reload counter low value
         
         //232 timeout
         global_yon_232_receive_timeout=TRUE;
         global_232_receive_byte_counter = 0;

}
////////////////////////////////////////////////////////////

//UART1 initialize
// desired baud rate:115200
// change 9600
// actual baud rate:111111 (3.7%)
// char size: 8 bit
// parity: Disabled
void uart1_init(void)
{
 //for 115200bps
 UCSR1B = 0x00; //disable while setting baud rate
 UCSR1A = 0x00;
 UCSR1C = 0x06;
 UBRR1L = 0x05; //set baud rate lo
 UBRR1H = 0x00; //set baud rate hi
 UCSR1B = 0x98;
 
 // for 9600bps
 //UCSR1B = 0x00; //disable while setting baud rate
 //UCSR1A = 0x00;
 //UCSR1C = 0x06;
 //UBRR1L = 0x67; //set baud rate lo
 //UBRR1H = 0x00; //set baud rate hi
 //UCSR1B = 0x98;

}

ISR(USART1_RX_vect)
   {
    unsigned char temp;
	//uart has received a character in UDR
    if (global_yon_process_message == TRUE)
       {
        temp=UDR1;
        process_message(temp);
	   }
	else
	   {	 
	   global_232_receive_buffer[0]=UDR1;
       global_yon_232_receive=TRUE;     
	   }
   }

void uart0_init(void)
{
 //for 115200bps
 UCSR0B = 0x00; //disable while setting baud rate
 UCSR0A = 0x00;
 UCSR0C = 0x06;
 UBRR0L = 0x05; //set baud rate lo
 UBRR0H = 0x00; //set baud rate hi
 UCSR0B = 0x98;
 
 // for 9600bps
 //UCSR1B = 0x00; //disable while setting baud rate
 //UCSR1A = 0x00;
 //UCSR1C = 0x06;
 //UBRR1L = 0x67; //set baud rate lo
 //UBRR1H = 0x00; //set baud rate hi
 //UCSR1B = 0x98;

}

ISR(USART0_RX_vect)
   {
    unsigned char temp;
	//uart has received a character in UDR
    if (global_yon_process_message == TRUE)
       {
        temp=UDR0;
        process_message(temp);
	   }
	else
	   {	 
	   global_232_receive_buffer[0]=UDR0;
       global_yon_232_receive=TRUE;     
	   }
   }

///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////
void init_var(void)
{
    int i;
    
    get_board_id();    //global_board_id    board ID号码

    for (i=0;i<MOTOR_NUMBER;++i)
        {
         //电机控制全局变量
         global_run_step_number[i]=0;   //电机还即将运行的步数
         global_counter_speed[i]=0;     //电机运行时需要等待的时隙个数
         global_counter_k_speed[i]=0;//电机运行时速度改变时的计数器K
         global_counter_low_speed[i]=0; 

         //位置全局变量
         global_motor_position_step[i]=0;//电机的步数位置
         global_motor_position_encoder[i]=0;//电机encoder的数据

         //电机状态全局变量
         global_motor_status[i]=MOTOR_STATUS_STOP;//电机运行状态
         global_motor_speed_status[i]=MOTOR_SPEED_STATUS_STOP;//电机速度状态
         global_motor_goal_status[i]=MOTOR_STATUS_STOP;//电机目标状态

         //方向全局变量
         global_motor_direction[i]=DIRECTION_FORWARD; //电机运行方向

         //速度全局变量
         global_motor_current_speed[i]=0;//电机当前速度 2
         global_motor_current_accel[i]=0;//电机当前加速度
         global_motor_current_decel[i]=0;//电机当前减速度

         //加减速度控制全局标志
         global_yon_motor_speed_control[i]=TRUE;//电机是否进行加减速度控制
         
         //需要从flash 中获取的设定参量
         //????????????????????????????????????????????????
         global_motor_setting_speed[i]=0;//电机设定速度 2
         global_motor_setting_accel[i]=0;//电机设定加速度
         global_motor_setting_decel[i]=0;//电机设定减速度
         global_motor_setting_slow_speed[i]=2;//电机设定慢速速度
         global_motor_setting_slow_accel[i]=0;//电机设定慢速加速度
         global_motor_setting_slow_decel[i]=0;//电机设定慢速减速度
         
         //加减速度设置变量
         global_motor_current_accel[i]=global_motor_setting_accel[i];
         global_motor_current_decel[i]=global_motor_setting_decel[i];

	 //速度控制变量,需要换算成为counter,时间隙个数,需要考虑算法
         //????????????????????????????????????????????????
         global_counter_low_setting_speed[i]=global_motor_setting_slow_speed[i];
         global_counter_setting_speed[i]=global_motor_setting_speed[i];
         //global_counter_k_speed[i]= 1；
		 

        }

    //全局标志变量
    global_yon_receive=FALSE;//全局变量接收命令标志
    global_yon_send=FALSE;//全局变量发送命令标志
    //global_yon_usb_receive=FALSE;//USB接口是否接收到命令标志
    //global_yon_232_receive=FALSE;//232接口是否接收到命令标志
    //global_yon_usb_send=FALSE;//USB接口是否发送标志
    //global_yon_232_send=FALSE;//232接口是否发送标志
    global_yon_232_receive=FALSE;
    global_232_receive_byte_counter=0;
    
    global_yon_return_message = FALSE;
	
	global_yon_already_send_s = FALSE;
    global_yon_already_send_t = FALSE;
    
    process_message_close();
}

///////////////////////////////////////////////////////////
void init_device(void)
{
 //stop errant interrupts until set up
 cli(); //disable all interrupts
 //XDIV  = 0x00; //xtal divider
 XMCRA = 0x00; //external memory
 
 //INIT PROT
 PORTA = 0b11000000;
 DDRA  = 0xff; //output 
 
 PORTB = 0xff;
 DDRB  = 0x00; //input and pull up
 
 PORTC = 0b11000000; 
 DDRC  = 0xff; //output
 
 PORTD = 0b11110111;
 DDRD  = 0b00001000;
 
 PORTE = 0b11111100;
 DDRE  = 0xff; //OUTPUT
 
 PORTF = 0xff;
 DDRF  = 0x00; //input

 PORTG = 0xff;
 DDRG  = 0xff; //input

 //watchdog_init();
 
 //timer1_init();  //停止状态机
 
 uart0_init();

 MCUCR = 0x00;
 EICRA = 0x00; //extended ext ints
 EICRB = 0x00; //extended ext ints
 EIMSK = 0x00;
 TIMSK = 0x01; //timer interrupt sources timer1 //status machine
 //TIMSK3 = 0x01; //timer interrupt sources timer3 //uart over time
 ETIMSK = 0x04; //extended timer interrupt sources
 sei(); //re-enable interrupts
 //all peripherals are now initialized


}

void Wait(unsigned int i)
{
  unsigned int j,k;
  for (j=0;j<i ;++j )
    {
     for (k=0;k<10 ;++k )
      {
       ;
	  }
	}

}

//*--------------------------------------------------------------------------------------
//* Function Name       : main
//* Object              :
//*--------------------------------------------------------------------------------------
int main ( void )
{
     // Init global var
     init_var();
     // Init per device
     init_device();
     // Init board
     
     Wait(0xff);     
     // fork
	 //for (; ; )
	 //   {
     //    global_232_send_buffer[0] = 'O';
     //    global_232_send_buffer[1] = 'K';
     //    global_232_send_buffer_length = 2;
	 //	 global_yon_send = TRUE;
	 //	 Wait(10000);
	 //	 send_data_232();
	 //	}
     
	 //sleep motor
	 motor_sleep(0);  
	 motor_sleep(1);  
	 motor_sleep(2);  
	 motor_sleep(3);  

     open_power_led();
	  
	 for (;;)
       {

        //check_usb_receive();  //check board id ,motor number,save to global_receive_buffer
		                      //set global_yon_receive flag
							  //if board id == 0 for read board id command
        check_fork_232_receive_protocol();  //for joystick change the motor speed
							  //if board id == 0 for read board id command
        if (global_yon_receive == TRUE )
          {
            fork_main2();
		  }
        //send_status();

       }

}


//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//function
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
void fork_main(void)
{
	 global_yon_receive = FALSE;
         //set global_motor_number var , function procedure use this var
	 global_motor_number=global_receive_buffer[PROTOCAL_RECEIVE_MOTOR_NUMBER];

	 switch (global_receive_buffer[PROTOCAL_RECEIVE_COMMAND])   //check command
         {
     case   COMMAND_RUN:
		    function_run();
            break;
     case   COMMAND_READ_STEP_POSITION:
		    function_read_step_position();
            break;
     case   COMMAND_READ_ENCODER_POSITION:
		    function_read_encoder_position();
            break;
     case   COMMAND_CLEAR_STEP_POSITION:
		    function_clear_step_position();
            break;
     case   COMMAND_CLEAR_ENCODER_POSITION:
		    function_clear_encoder_position();
            break;
     case   COMMAND_GOTO_REFERENCE:
		    function_goto_reference();
            break;
     case   COMMAND_GOTO_FORWARD_LIMIT:
		    function_goto_forward_limit();
            break;
     case   COMMAND_GOTO_REVERSE_LIMIT:
		    function_goto_reverse_limit();
            break;
     case   COMMAND_STOP:
		    function_stop();
            break;
     case   COMMAND_SET_SPEED:
		    function_set_speed();
            break;
     case   COMMAND_SET_ACCEL:
		    function_set_accel();
            break;
     case   COMMAND_SET_DECEL:
		    function_set_decel();
            break;
     case   COMMAND_READ_SPEED:
		    function_read_speed();
            break;
     case   COMMAND_READ_ACCEL:
		    function_read_accel();
            break;
     case   COMMAND_READ_DECEL:
		    function_read_decel();
            break;
     case   COMMAND_READ_STATUS:
		    function_read_status();
		    break;
     case   COMMAND_READ_BOARD_ID:
            function_read_board_id();
            break;
         }
}

void fork_main2(void)
{
   unsigned long temp1,temp2,temp3,temp4;
   unsigned long step;
   unsigned long i;
   unsigned char direct;
     
	 global_yon_receive = FALSE;
         //set global_motor_number var , function procedure use this var
	 global_motor_number=global_receive_buffer[PROTOCAL_RECEIVE_MOTOR_NUMBER];

	 switch (global_receive_buffer[PROTOCAL_RECEIVE_COMMAND])   //check command
         {
     case   COMMAND_RUN:

            //open led
			open_led(global_motor_number);
			//set unsleep
            motor_unsleep(global_motor_number);
			Wait(0xff);
			//
	        temp1=global_receive_buffer[PROTOCAL_RECEIVE_PAR1];
	        temp2=global_receive_buffer[PROTOCAL_RECEIVE_PAR2];
	        temp3=global_receive_buffer[PROTOCAL_RECEIVE_PAR3];
	        temp4=global_receive_buffer[PROTOCAL_RECEIVE_PAR4];
	        //高位在前，低位在后
	        step=temp1<<24;
            step+=temp2<<16;
            step+=temp3<<8;
            step+=temp4;
			//set direction
            direct=global_receive_buffer[PROTOCAL_RECEIVE_DIRECTION];   
            global_motor_direction[global_motor_number]=direct;
            set_direction(global_motor_number,global_motor_direction[global_motor_number]);
	        
     	    //global_motor_number already set at fork
        	global_run_step_number[global_motor_number]=step;
	 		global_motor_status[global_motor_number]=MOTOR_STATUS_NORMAL_RUN;
         	
			process_message_init();
			for (i=0;i<global_run_step_number[global_motor_number] ;++i )
            	{
             	 one_step(global_motor_number);
				 if (global_message == STOP)
				   {
				   break;
				   }
				} 
			process_message_close();
			//run end
         	global_motor_status[global_motor_number]=MOTOR_STATUS_STOP;
 
            Wait(0x1fff);
            //set sleep
            motor_sleep(global_motor_number);
			Wait(0xf);
			//
			
			//return message for 232
            if (global_message == STOP)
              {
               global_232_send_buffer[0] = 'E';
			   }
			else
			 {	 
			  global_232_send_buffer[0] = 'O';
			 }
			global_232_send_buffer_length = 1;
            //return message begin
        	global_yon_return_message=TRUE;
            function_return_message();
           	//return message end
	        //close led
			close_led(global_motor_number);
			
			break;
     case   COMMAND_READ_STEP_POSITION:
            step=global_motor_position_step[global_motor_number];  //global_motor_number already set at fork()
            //高位在前，低位在后
            temp1=step >> 24;
            temp2=(step & 0x00ffffff) >> 16;
            temp3=(step & 0x0000ffff) >> 8;
            temp4=(step & 0x000000ff);
            //return message for 232
            global_232_send_buffer[0]=temp1;
            global_232_send_buffer[1]=temp2;
            global_232_send_buffer[2]=temp3;
            global_232_send_buffer[3]=temp4;
            global_232_send_buffer_length = 4 ;
            //return message end
            function_return_message();
            break;
     case   COMMAND_CLEAR_STEP_POSITION:
            global_motor_position_step[global_motor_number] = 0;
            //return message for 232
            global_232_send_buffer[0]='O';
            global_232_send_buffer_length=1;
            //return message end
            function_return_message();
            break;
     case   COMMAND_GOTO_REFERENCE:
		    //open led
			open_led(global_motor_number);
			if(goto_reference(global_motor_number)==TRUE)
			 {
	          //send O
             global_232_send_buffer[0]='O';
             global_232_send_buffer_length=1;
             //return message end
             function_return_message();

			  }
			else
			 {
			  //send E
              global_232_send_buffer[0]='E';
              global_232_send_buffer_length=1;
              //return message end
              function_return_message();
		      }	 
            //close led
			close_led(global_motor_number);
			break;
     case   COMMAND_GOTO_FORWARD_LIMIT:
		    //function_goto_forward_limit();
            break;
     case   COMMAND_GOTO_REVERSE_LIMIT:
		    //function_goto_reverse_limit();
            break;
     case   COMMAND_STOP:
		    //function_stop();
            break;
     case   COMMAND_SET_SPEED:
		    //function_set_speed();
            break;
     case   COMMAND_READ_SPEED:
		    //function_read_speed();
            break;
     case   COMMAND_READ_STATUS:
		    //function_read_status();
		    break;
     case   COMMAND_READ_BOARD_ID:
            //function_read_board_id();
            break;
         }
}
/////////////////////////////////////////////////////////////////////
void function_return_message(void)
{
    if  (global_message_type == USB_MESSAGE_TYPE)
      {
      //prepare send data for usb
	 global_send_buffer[PROTOCAL_SEND_BOARD_ID] = global_board_id;
	 global_send_buffer[PROTOCAL_SEND_MOTOR_NUMBER] = global_motor_number;
	 global_send_buffer[PROTOCAL_SEND_COMMAND] = global_receive_buffer[PROTOCAL_RECEIVE_COMMAND];
	 global_send_buffer[PROTOCAL_SEND_STATUS_ID] = global_motor_status[global_motor_number];
     //send data already set
	 //send
	 global_yon_send = TRUE ;
	 send_data();
       }
     else if (global_message_type == RS232_MESSAGE_TYPE )
     { 
     //prepare send data for 232
     //send data already set
	 //send
	 global_yon_send = TRUE ;
	 send_data_232_uart0();
     }   
}
/////////////////////////////////////////////////////////////////////
void send_data(void)
{ 
//     void *p;
//     char *s;
//     // send data to usb
//     if (global_yon_send == TRUE)
//        {
//         p=global_send_buffer;
//         s=p;
//         pCDC.Write(&pCDC,s,MESSAGE_LENGTH);
//        }
//     global_yon_send=FALSE;
//         
}
/////////////////////////////////////////////////////////////////////
void send_data_232_uart1(void)
{ 
	 // send data to 232
     unsigned char i;
	 if (global_yon_send == TRUE)
        {
         for (i=0;i<global_232_send_buffer_length ;++i )
            {
  	         while ( !(UCSR1A & (1<<UDRE1)) )
		     ; /* wait for empty transmit buffer */
	         UDR1 = global_232_send_buffer[i]; /* start transmittion */
			}
		 //AT91F_US_Put(global_232_send_buffer,global_232_send_buffer_length);   
	    }
     global_yon_send=FALSE;
}
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
void send_data_232_uart0(void)
{ 
	 // send data to 232
     unsigned char i;
	 if (global_yon_send == TRUE)
        {
         for (i=0;i<global_232_send_buffer_length ;++i )
            {
  	         while ( !(UCSR0A & (1<<UDRE0)) )
		     ; /* wait for empty transmit buffer */
	         UDR0 = global_232_send_buffer[i]; /* start transmittion */
			}
		 //AT91F_US_Put(global_232_send_buffer,global_232_send_buffer_length);   
	    }
     global_yon_send=FALSE;
}
/////////////////////////////////////////////////////////////////////

void function_run(void)
{
     unsigned long temp1,temp2,temp3,temp4;
	 unsigned long step;
     unsigned char direct;
	 temp1=global_receive_buffer[PROTOCAL_RECEIVE_PAR1];
	 temp2=global_receive_buffer[PROTOCAL_RECEIVE_PAR2];
	 temp3=global_receive_buffer[PROTOCAL_RECEIVE_PAR3];
	 temp4=global_receive_buffer[PROTOCAL_RECEIVE_PAR4];
	 //高位在前，低位在后
	 step=temp1<<24;
     step+=temp2<<16;
     step+=temp3<<8;
     step+=temp4;

     //set direction
     direct=global_receive_buffer[PROTOCAL_RECEIVE_DIRECTION];   
     global_motor_direction[global_motor_number]=direct;
     set_direction(global_motor_number,global_motor_direction[global_motor_number]);
	 //close global interrupt
     DISABLE_INTERRUPT;
	 
	 //global_motor_number already set at fork
	 global_run_step_number[global_motor_number]=step;
	 
	 //set begin and end status
     global_motor_status[global_motor_number]=MOTOR_STATUS_NORMAL_RUN;
	 global_motor_goal_status[global_motor_number]=MOTOR_STATUS_STOP;
	 global_motor_speed_status[global_motor_number]=MOTOR_SPEED_STATUS_RUN;
	 //open global interrupt
     ENABLE_INTERRUPT;
	 //set begin and end status end
         
     //return message for usb
     global_send_buffer[PROTOCAL_SEND_PAR1] = 0;
	 global_send_buffer[PROTOCAL_SEND_PAR2] = 0;
	 global_send_buffer[PROTOCAL_SEND_PAR3] = 0;
	 global_send_buffer[PROTOCAL_SEND_PAR4] = 0;
     //return message for 232
     global_232_send_buffer[0] = 'O';
     global_232_send_buffer[1] = 'K';
     global_232_send_buffer_length = 2;
     //return message end
	 global_yon_return_message=TRUE;
     function_return_message();
}

/////////////////////////////////////////////////////////////////////
void function_read_step_position(void)
{
     unsigned long temp1,temp2,temp3,temp4;
     unsigned long step; 
     step=global_motor_position_step[global_motor_number];  //global_motor_number already set at fork()
     //高位在前，低位在后
     temp1=step >> 24;
     temp2=(step & 0x00ffffff) >> 16;
     temp3=(step & 0x0000ffff) >> 8;
     temp4=(step & 0x000000ff);
     //retrun message for usb
     global_send_buffer[PROTOCAL_SEND_PAR1] = temp1;
     global_send_buffer[PROTOCAL_SEND_PAR2] = temp2;
     global_send_buffer[PROTOCAL_SEND_PAR3] = temp3;
     global_send_buffer[PROTOCAL_SEND_PAR4] = temp4;
     //return message for 232
     global_232_send_buffer[0]=temp1;
     global_232_send_buffer[1]=temp2;
     global_232_send_buffer[2]=temp3;
     global_232_send_buffer[3]=temp4;
     global_232_send_buffer_length = 4 ;
     //return message end
     function_return_message();

}

/////////////////////////////////////////////////////////////////////
void function_read_encoder_position(void)
{
     unsigned long temp1,temp2,temp3,temp4;
     unsigned long step;
     get_motor_position_encoder();
     step=global_motor_position_encoder[global_motor_number];  //global_motor_number already set at fork()
     //高位在前，低位在后
     temp1=step >> 24;
     temp2=(step & 0x00ffffff) >> 16;
     temp3=(step & 0x0000ffff) >> 8;
     temp4=(step & 0x000000ff);
     //retrun message
     global_send_buffer[PROTOCAL_SEND_PAR1] = temp1;
     global_send_buffer[PROTOCAL_SEND_PAR2] = temp2;
     global_send_buffer[PROTOCAL_SEND_PAR3] = temp3;
     global_send_buffer[PROTOCAL_SEND_PAR4] = temp4;
     function_return_message();

}

/////////////////////////////////////////////////////////////////////
void function_clear_step_position(void)
{
     global_motor_position_step[global_motor_number] = 0;
     //return message usb
     global_send_buffer[PROTOCAL_SEND_PAR1] = 0;
     global_send_buffer[PROTOCAL_SEND_PAR2] = 0;
     global_send_buffer[PROTOCAL_SEND_PAR3] = 0;
     global_send_buffer[PROTOCAL_SEND_PAR4] = 0;
     //return message for 232
     global_232_send_buffer[0]='O';
     global_232_send_buffer[1]='K';
     global_232_send_buffer_length=2;
     //return message end
     function_return_message();

}
/////////////////////////////////////////////////////////////////////
void function_clear_encoder_position(void)
{
     /*
	 global_motor_position_encoder[global_motor_number] = 0;
     //hardware reset encoder 
     if ( global_motor_number ==0 )
       {
        //reset x axis
        AT91F_PIO_ClearOutput( AT91C_BASE_PIOA, H_ENCODER_RSTX ) ;
        Wait(1000);
        AT91F_PIO_SetOutput( AT91C_BASE_PIOA, H_ENCODER_RSTX ) ;
       }
     if (global_motor_number ==1)
      {
       //reset y axis
        AT91F_PIO_ClearOutput( AT91C_BASE_PIOA, H_ENCODER_RSTY ) ;
        Wait(1000);
        AT91F_PIO_SetOutput( AT91C_BASE_PIOA, H_ENCODER_RSTY ) ;
       } 
     //return message
     global_send_buffer[PROTOCAL_SEND_PAR1] = 0;
     global_send_buffer[PROTOCAL_SEND_PAR2] = 0;
     global_send_buffer[PROTOCAL_SEND_PAR3] = 0;
     global_send_buffer[PROTOCAL_SEND_PAR4] = 0;
     function_return_message();
     */
}
/////////////////////////////////////////////////////////////////////
void function_goto_reference(void)
{
     //set direction   
     global_motor_direction[global_motor_number]=DIRECTION_FORWARD;
     set_direction(global_motor_number,DIRECTION_FORWARD); 
     //set begin and end status
     DISABLE_INTERRUPT;
     global_motor_status[global_motor_number]=MOTOR_STATUS_SLOW_GOTO_REFERENCE;
     global_motor_goal_status[global_motor_number]=MOTOR_STATUS_REFERENCE;
     global_motor_speed_status[global_motor_number]=MOTOR_SPEED_STATUS_SLOW_RUN;
     ENABLE_INTERRUPT;
     //return message
     global_send_buffer[PROTOCAL_SEND_PAR1] = 0;
     global_send_buffer[PROTOCAL_SEND_PAR2] = 0;
     global_send_buffer[PROTOCAL_SEND_PAR3] = 0;
     global_send_buffer[PROTOCAL_SEND_PAR4] = 0;
     //return message for 232
     global_232_send_buffer[0]='R';
     global_232_send_buffer[1]='E';
     global_232_send_buffer[2]='F';
     global_232_send_buffer[3]='E';
     global_232_send_buffer[4]='R';
     global_232_send_buffer[5]='E';
     global_232_send_buffer[6]='N';
     global_232_send_buffer[7]='C';
     global_232_send_buffer[8]='E';
     global_232_send_buffer[9]='_';
     global_232_send_buffer[10]='P';
     global_232_send_buffer[11]='O';
     global_232_send_buffer[12]='S';
     
     global_232_send_buffer_length=13;
     //return message end

     global_yon_return_message=TRUE;
     function_return_message();
     


}
/////////////////////////////////////////////////////////////////////
void function_goto_forward_limit(void)
{
     //set direciton
     global_motor_direction[global_motor_number]=DIRECTION_FORWARD;
     set_direction(global_motor_number,DIRECTION_FORWARD); 
     //set begin and end status
     DISABLE_INTERRUPT;
     global_motor_status[global_motor_number]=MOTOR_STATUS_SLOW_GOTO_FORWARD_LIMIT;
     global_motor_goal_status[global_motor_number]=MOTOR_STATUS_FORWARD_LIMIT;
     global_motor_speed_status[global_motor_number]=MOTOR_SPEED_STATUS_SLOW_RUN;
     ENABLE_INTERRUPT;
     //return message
     global_send_buffer[PROTOCAL_SEND_PAR1] = 0;
     global_send_buffer[PROTOCAL_SEND_PAR2] = 0;
     global_send_buffer[PROTOCAL_SEND_PAR3] = 0;
     global_send_buffer[PROTOCAL_SEND_PAR4] = 0;
     //return message for 232
     global_232_send_buffer[0]='F';
     global_232_send_buffer[1]='O';
     global_232_send_buffer[2]='R';
     global_232_send_buffer[3]='W';
     global_232_send_buffer[4]='A';
     global_232_send_buffer[5]='R';
     global_232_send_buffer[6]='D';
     global_232_send_buffer[7]='_';
     global_232_send_buffer[8]='L';
     global_232_send_buffer[9]='I';
     global_232_send_buffer[10]='M';
     global_232_send_buffer[11]='I';
     global_232_send_buffer[12]='T';
     
     global_232_send_buffer_length=13;
     //return message end

     global_yon_return_message=TRUE;
     function_return_message();


}
/////////////////////////////////////////////////////////////////////
void function_goto_reverse_limit(void)
{
     //set direciton
     global_motor_direction[global_motor_number]=DIRECTION_REVERSE;
     set_direction(global_motor_number,DIRECTION_REVERSE); 
     //set begin and end status
     DISABLE_INTERRUPT;
     global_motor_status[global_motor_number]=MOTOR_STATUS_SLOW_GOTO_REVERSE_LIMIT;
     global_motor_goal_status[global_motor_number]=MOTOR_STATUS_REVERSE_LIMIT;
     global_motor_speed_status[global_motor_number]=MOTOR_SPEED_STATUS_SLOW_RUN;
     ENABLE_INTERRUPT;
     //return message
     global_send_buffer[PROTOCAL_SEND_PAR1] = 0;
     global_send_buffer[PROTOCAL_SEND_PAR2] = 0;
     global_send_buffer[PROTOCAL_SEND_PAR3] = 0;
     global_send_buffer[PROTOCAL_SEND_PAR4] = 0;
     //return message for 232
     global_232_send_buffer[0]='R';
     global_232_send_buffer[1]='E';
     global_232_send_buffer[2]='V';
     global_232_send_buffer[3]='E';
     global_232_send_buffer[4]='R';
     global_232_send_buffer[5]='S';
     global_232_send_buffer[6]='E';
     global_232_send_buffer[7]='_';
     global_232_send_buffer[8]='L';
     global_232_send_buffer[9]='I';
     global_232_send_buffer[10]='M';
     global_232_send_buffer[11]='I';
     global_232_send_buffer[12]='T';
     
     global_232_send_buffer_length=13;
     //return message end

     global_yon_return_message=TRUE;
     function_return_message();

}
/////////////////////////////////////////////////////////////////////
void function_stop(void)
{
     //DISABLE_INTERRUPT;
     cli();
	 //set global motor number to 0
     global_run_step_number[global_motor_number]=0;
     //set begin and end status
     global_motor_status[global_motor_number]=MOTOR_STATUS_STOP;
     global_motor_goal_status[global_motor_number]=MOTOR_STATUS_STOP;
     global_motor_speed_status[global_motor_number]=MOTOR_SPEED_STATUS_RUN;
     //ENABLE_INTERRUPT;
     sei();
	 //return message
     global_send_buffer[PROTOCAL_SEND_PAR1] = 0;
     global_send_buffer[PROTOCAL_SEND_PAR2] = 0;
     global_send_buffer[PROTOCAL_SEND_PAR3] = 0;
     global_send_buffer[PROTOCAL_SEND_PAR4] = 0;
     function_return_message();

}
/////////////////////////////////////////////////////////////////////
void function_set_speed(void)
{
     unsigned long temp1,temp2,temp3,temp4;
     unsigned long speed;
     temp1=global_receive_buffer[PROTOCAL_RECEIVE_PAR1];
     temp2=global_receive_buffer[PROTOCAL_RECEIVE_PAR2];
     temp3=global_receive_buffer[PROTOCAL_RECEIVE_PAR3];
     temp4=global_receive_buffer[PROTOCAL_RECEIVE_PAR4];
     //高位在前，低位在后
     speed=temp1<<24;
     speed+=temp2<<16;
     speed+=temp3<<8;
     speed+=temp4;
     //
     global_motor_setting_speed[global_motor_number]=speed;
     global_counter_setting_speed[global_motor_number]=global_motor_setting_speed[global_motor_number];
     //return message
     global_send_buffer[PROTOCAL_SEND_PAR1] = 0;
     global_send_buffer[PROTOCAL_SEND_PAR2] = 0;
     global_send_buffer[PROTOCAL_SEND_PAR3] = 0;
     global_send_buffer[PROTOCAL_SEND_PAR4] = 0;
     //return message for 232
     global_232_send_buffer[0] = 'O';
     global_232_send_buffer[1] = 'K';
     global_232_send_buffer_length = 2;
     //return message end

     function_return_message();
}
/////////////////////////////////////////////////////////////////////
void function_set_accel(void)
{
     unsigned long temp1,temp2,temp3,temp4;
	 unsigned long accel;
	 temp1=global_receive_buffer[PROTOCAL_RECEIVE_PAR1];
	 temp2=global_receive_buffer[PROTOCAL_RECEIVE_PAR2];
	 temp3=global_receive_buffer[PROTOCAL_RECEIVE_PAR3];
	 temp4=global_receive_buffer[PROTOCAL_RECEIVE_PAR4];
	 //高位在前，低位在后
	 accel=temp1<<24;
         accel+=temp2<<16;
         accel+=temp3<<8;
         accel+=temp4;
     //
     global_motor_setting_accel[global_motor_number]=accel;
	 //return message
     global_send_buffer[PROTOCAL_SEND_PAR1] = 0;
     global_send_buffer[PROTOCAL_SEND_PAR2] = 0;
     global_send_buffer[PROTOCAL_SEND_PAR3] = 0;
     global_send_buffer[PROTOCAL_SEND_PAR4] = 0;
	 function_return_message();
}
/////////////////////////////////////////////////////////////////////
void function_set_decel(void)
{
     unsigned long temp1,temp2,temp3,temp4;
	 unsigned long decel;
	 temp1=global_receive_buffer[PROTOCAL_RECEIVE_PAR1];
	 temp2=global_receive_buffer[PROTOCAL_RECEIVE_PAR2];
	 temp3=global_receive_buffer[PROTOCAL_RECEIVE_PAR3];
	 temp4=global_receive_buffer[PROTOCAL_RECEIVE_PAR4];
	 //高位在前，低位在后
	 decel=temp1<<24;
         decel+=temp2<<16;
         decel+=temp3<<8;
         decel+=temp4;
     //
     global_motor_setting_speed[global_motor_number]=decel;
	 //return message
     global_send_buffer[PROTOCAL_SEND_PAR1] = 0;
     global_send_buffer[PROTOCAL_SEND_PAR2] = 0;
     global_send_buffer[PROTOCAL_SEND_PAR3] = 0;
     global_send_buffer[PROTOCAL_SEND_PAR4] = 0;
	 function_return_message();
}
/////////////////////////////////////////////////////////////////////
void function_read_speed(void)
{
     unsigned long temp1,temp2,temp3,temp4;
	 unsigned long speed;
	 speed=global_motor_setting_speed[global_motor_number];  //global_motor_number already set at fork()
	 //高位在前，低位在后
	 temp1=speed >> 24;
	 temp2=(speed & 0x00ffffff) >> 16;
	 temp3=(speed & 0x0000ffff) >> 8;
	 temp4=(speed & 0x000000ff);
     //retrun message
     global_send_buffer[PROTOCAL_SEND_PAR1] = temp1;
     global_send_buffer[PROTOCAL_SEND_PAR2] = temp2;
     global_send_buffer[PROTOCAL_SEND_PAR3] = temp3;
     global_send_buffer[PROTOCAL_SEND_PAR4] = temp4;
     function_return_message();
}
/////////////////////////////////////////////////////////////////////
void function_read_accel(void)
{
     unsigned long temp1,temp2,temp3,temp4;
	 unsigned long accel;
	 accel=global_motor_setting_accel[global_motor_number];  //global_motor_number already set at fork()
	 //高位在前，低位在后
	 temp1=accel >> 24;
	 temp2=(accel & 0x00ffffff) >> 16;
	 temp3=(accel & 0x0000ffff) >> 8;
	 temp4=(accel & 0x000000ff);
     //retrun message
     global_send_buffer[PROTOCAL_SEND_PAR1] = temp1;
     global_send_buffer[PROTOCAL_SEND_PAR2] = temp2;
     global_send_buffer[PROTOCAL_SEND_PAR3] = temp3;
     global_send_buffer[PROTOCAL_SEND_PAR4] = temp4;
	 function_return_message();
}
/////////////////////////////////////////////////////////////////////
void function_read_decel(void)
{
     unsigned long temp1,temp2,temp3,temp4;
	 unsigned long decel;
	 decel=global_motor_setting_decel[global_motor_number];  //global_motor_number already set at fork()
	 //高位在前，低位在后
	 temp1=decel >> 24;
	 temp2=(decel & 0x00ffffff) >> 16;
	 temp3=(decel & 0x0000ffff) >> 8;
	 temp4=(decel & 0x000000ff);
     //retrun message
     global_send_buffer[PROTOCAL_SEND_PAR1] = temp1;
     global_send_buffer[PROTOCAL_SEND_PAR2] = temp2;
     global_send_buffer[PROTOCAL_SEND_PAR3] = temp3;
     global_send_buffer[PROTOCAL_SEND_PAR4] = temp4;
	 function_return_message();
}
/////////////////////////////////////////////////////////////////////
void function_read_status(void)
{
     //status parpare  in the function_return_message();
	 //retrun message
     global_send_buffer[PROTOCAL_SEND_PAR1] = 0;
     global_send_buffer[PROTOCAL_SEND_PAR2] = 0;
     global_send_buffer[PROTOCAL_SEND_PAR3] = 0;
     global_send_buffer[PROTOCAL_SEND_PAR4] = 0;
	 function_return_message();
}
/////////////////////////////////////////////////////////////////////
void function_read_board_id(void)
{
     //status parpare  in the function_return_message();
	 //retrun message
     global_send_buffer[PROTOCAL_SEND_PAR1] = 0;
     global_send_buffer[PROTOCAL_SEND_PAR2] = 0;
     global_send_buffer[PROTOCAL_SEND_PAR3] = 0;
     global_send_buffer[PROTOCAL_SEND_PAR4] = 0;
	 function_return_message();
}
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
void get_motor_switch_status(void)
{
	 unsigned char data;
	 unsigned char temp;
	 unsigned char motor_number;

	 //PINB LIMIT SWITCH
	 //PIND REF SWITCH PD4-PD7

     data = PIND;  //REF
     data = ~data;
     temp = data&0b00010000;  //REF0 
	 if (temp != 0)
       {
        // motor 1 reference switch
        motor_number =0;
	    global_motor_status[motor_number] = MOTOR_STATUS_REFERENCE;
	   }
     else 
	   {
        motor_number =0;
	    global_motor_status[motor_number] = MOTOR_STATUS_OTHER;
		}
	 
     temp = data & 0b00100000; //REF1  
     if (temp != 0 )
       {
        // motor 2  reference switch
	    motor_number =1 ;
        global_motor_status[motor_number] = MOTOR_STATUS_REFERENCE;
	   }
     else 
	   {
        motor_number =1;
	    global_motor_status[motor_number] = MOTOR_STATUS_OTHER;
		}

     temp = data&0b01000000;  //REF2
	 if (temp != 0)
       {
        // motor 3 reference switch
        motor_number =2;
	    global_motor_status[motor_number] = MOTOR_STATUS_REFERENCE;
	   }
     else 
	   {
        motor_number =2;
	    global_motor_status[motor_number] = MOTOR_STATUS_OTHER;
		}

     temp = data & 0b10000000; //REF3  
     if (temp != 0 )
       {
        // motor 4  reference switch
	    motor_number =3 ;
        global_motor_status[motor_number] = MOTOR_STATUS_REFERENCE;
	   }
     else 
	   {
        motor_number =3;
	    global_motor_status[motor_number] = MOTOR_STATUS_OTHER;
		}
	 

	 /*
	 data = PINB;   //LIMIT

     //need not by xxx 20111109
	 data=(~data);
     //end by xxx 20111109
     temp = data & 0b00000001;   //p1   
     if (temp == 0)            
       {
	   // motor 1 p1 limit switch
 	   motor_number =0;
	   global_motor_status[motor_number] = MOTOR_STATUS_FORWARD_LIMIT;
	   }
     temp = data & 0b00000010;   //q1
     if (temp == 0)
       {
	   // motor 1 q1 limit switch
	   motor_number =0;
	   global_motor_status[motor_number] = MOTOR_STATUS_REVERSE_LIMIT;
   	   }
	 */
	 /*
     temp = data & 0b00000100;   //p2   
     if (temp == 0)            
       {
	   // motor 2 p2 limit switch
 	   motor_number =1;
	   global_motor_status[motor_number] = MOTOR_STATUS_FORWARD_LIMIT;
	   }
     temp = data & 0b00001000;   //q2
     if (temp == 0)
       {
	   // motor 2 q2 limit switch
	   motor_number =1;
	   global_motor_status[motor_number] = MOTOR_STATUS_REVERSE_LIMIT;
   	   }
	   
     temp = data & 0b00010000;   //p3   
     if (temp == 0)            
       {
	   // motor 3 p3 limit switch
 	   motor_number =2;
	   global_motor_status[motor_number] = MOTOR_STATUS_FORWARD_LIMIT;
	   }
     temp = data & 0b00100000;   //q3
     if (temp == 0)
       {
	   // motor 3 q3 limit switch
	   motor_number =2;
	   global_motor_status[motor_number] = MOTOR_STATUS_REVERSE_LIMIT;
   	   }

     temp = data & 0b01000000;   //p4   
     if (temp == 0)            
       {
	   // motor 4 p4 limit switch
 	   motor_number =3;
	   global_motor_status[motor_number] = MOTOR_STATUS_FORWARD_LIMIT;
	   }
     temp = data & 0b10000000;   //q4
     if (temp == 0)
       {
	   // motor 4 q4 limit switch
	   motor_number =3;
	   global_motor_status[motor_number] = MOTOR_STATUS_REVERSE_LIMIT;
   	   }
	 */
}

/////////////////////////////////////////////////////////////////////
void isr_limit_switch(void)
{
	 //如果进入此子程序，表明进入中断IRQ0，
	 //low effect
	 //此子程序需要判断中断源，具体是那个motor，那个方向的限位开关
     //SET READCS1  = 0
	 //MPU_RCS0=1 MPU_RCS1=0 
     get_motor_switch_status();     
}

/////////////////////////////////////////////////////////////////////
void get_motor_position_encoder(void)
{
//	//
//	//ENCODER  HCTL2032
//    //PA10    MPU_XY    OUTPUT    CHOICE X OR Y , LOW FIRST COUNTER, HIGH SECOND COUNTER
//    //PA29    MPU_RSTX  OUTPUT    CLEAR COUNTER LOW EFFECT
//    //PA30    MPU_RSTY  OUTPUT    CLEAR COUNTER LOW EFFECT
//    //PA17    MPU_SEL1  OUTPUT    READ DATA  
//    //PA18    MPU_SEL2  OUTPUT    READ DATA
//    //                      MPU_SEL1   MPUSEL2   DATR
//    //                        0          1         D4 
//    //                        1          1         D3
//    //                        0          0         D2
//    //                        1          0         D1
//
//    //PA28    MPU_D7    INPUT
//    //PA27    MPU_D6    INPUT
//    //PA26    MPU_D5    INPUT
//    //PA25    MPU_D4    INPUT
//    //PA24    MPU_D3    INPUT
//    //PA23    MPU_D2    INPUT
//    //PA22    MPU_D1    INPUT
//    //PA21    MPU_D0    INPUT
//
//	unsigned int temp_data;
//	unsigned char temp_byte;
//	int data;
//        
//       DISABLE_INTERRUPT;
//        //set READCS0=0     MPU_RCS0=0 MPU_RCS1=0
//        AT91F_PIO_ClearOutput( AT91C_BASE_PIOA, H_RCS0);
//        AT91F_PIO_ClearOutput( AT91C_BASE_PIOA, H_RCS1);
//
//        data=0;
//        Wait(1);
//	//OPEN OE
//        AT91F_PIO_ClearOutput( AT91C_BASE_PIOA, H_ENCODER_OE ) ;         //OPEN OE
//        //read X axis
//        AT91F_PIO_ClearOutput( AT91C_BASE_PIOA, H_ENCODER_XY ) ;         //SET MPU_XY LOW,FIRST COUNTER
//     
//       AT91F_PIO_ClearOutput( AT91C_BASE_PIOA, H_ENCODER_SEL1 ) ;         
//        AT91F_PIO_SetOutput( AT91C_BASE_PIOA, H_ENCODER_SEL2 ) ;         //MSB
//	Wait(1); 
//	temp_data = AT91F_PIO_GetInput(AT91C_BASE_PIOA);
//	temp_data &= 0x1fe00000;
//        temp_byte = temp_data >> 21;                                   
//        temp_data = temp_byte <<24;
//        data+=temp_data ;
//     
//        AT91F_PIO_SetOutput( AT91C_BASE_PIOA, H_ENCODER_SEL1 ) ;         
//        AT91F_PIO_SetOutput( AT91C_BASE_PIOA, H_ENCODER_SEL2 ) ;         //D3
//	Wait(1);
//        temp_data = AT91F_PIO_GetInput(AT91C_BASE_PIOA);
//	temp_data &= 0x1fe00000;
//        temp_byte = temp_data >> 21;                                   
//	temp_data = temp_byte << 16;
//        data+=temp_data;
//     
//        AT91F_PIO_ClearOutput( AT91C_BASE_PIOA, H_ENCODER_SEL1 ) ;         
//        AT91F_PIO_ClearOutput( AT91C_BASE_PIOA, H_ENCODER_SEL2 ) ;         //D2
//	Wait(1); 
//	temp_data = AT91F_PIO_GetInput(AT91C_BASE_PIOA);
//	temp_data &= 0x1fe00000;
//       temp_byte = temp_data >> 21;                                   
//	temp_data = temp_byte << 8;
//        data+=temp_data;
//	  
//        AT91F_PIO_SetOutput( AT91C_BASE_PIOA, H_ENCODER_SEL1 ) ;         
//        AT91F_PIO_ClearOutput( AT91C_BASE_PIOA, H_ENCODER_SEL2 ) ;         //D1 LSB
//	Wait(1); 
//	temp_data = AT91F_PIO_GetInput(AT91C_BASE_PIOA);
//	temp_data &= 0x1fe00000;
//        temp_byte = temp_data >> 21;                                   
//        data+=temp_byte;
//
//        //CLOSE OE
//        AT91F_PIO_SetOutput( AT91C_BASE_PIOA, H_ENCODER_OE ) ;         //close OE
//
//	global_motor_position_encoder[0] = data;
//        //read X axis end
//        
//	data=0;
//        Wait(1);
//	//OPEN OE
//        AT91F_PIO_ClearOutput( AT91C_BASE_PIOA, H_ENCODER_OE ) ;         //OPEN OE
//	//read Y axis
//	AT91F_PIO_SetOutput( AT91C_BASE_PIOA, H_ENCODER_XY ) ;           //SET MPU_XY HIGH,SECOND COUNTER
//        AT91F_PIO_ClearOutput( AT91C_BASE_PIOA, H_ENCODER_SEL1 ) ;         
//        AT91F_PIO_SetOutput( AT91C_BASE_PIOA, H_ENCODER_SEL2 ) ;         //MSB
//	Wait(1); 
//	temp_data = AT91F_PIO_GetInput(AT91C_BASE_PIOA);
//	temp_data &= 0x1fe00000;
//        temp_byte = temp_data >> 21;                                   
//       temp_data = temp_byte <<24;
//       data+=temp_data ;
//     
//        AT91F_PIO_SetOutput( AT91C_BASE_PIOA, H_ENCODER_SEL1 ) ;         
//        AT91F_PIO_SetOutput( AT91C_BASE_PIOA, H_ENCODER_SEL2 ) ;         //D3
//	Wait(1); 
//	temp_data = AT91F_PIO_GetInput(AT91C_BASE_PIOA);
//	temp_data &= 0x1fe00000;
//        temp_byte = temp_data >> 21;                                   
//	temp_data = temp_byte << 16;
//        data+=temp_data;
//     
//        AT91F_PIO_ClearOutput( AT91C_BASE_PIOA, H_ENCODER_SEL1 ) ;         
//        AT91F_PIO_ClearOutput( AT91C_BASE_PIOA, H_ENCODER_SEL2 ) ;         //D2
//	Wait(1); 
//	temp_data = AT91F_PIO_GetInput(AT91C_BASE_PIOA);
//	temp_data &= 0x1fe00000;
//        temp_byte = temp_data >> 21;                                   
//	temp_data = temp_byte << 8;
//        data+=temp_data;
//	  
//        AT91F_PIO_SetOutput( AT91C_BASE_PIOA, H_ENCODER_SEL1 ) ;         
//        AT91F_PIO_ClearOutput( AT91C_BASE_PIOA, H_ENCODER_SEL2 ) ;         //D1 LSB
//	Wait(1); 
//	temp_data = AT91F_PIO_GetInput(AT91C_BASE_PIOA);
//	temp_data &= 0x1fe00000;
//        temp_byte = temp_data >> 21;                                   
//        data+=temp_byte;
//
//        //CLOSE OE
//        AT91F_PIO_SetOutput( AT91C_BASE_PIOA, H_ENCODER_OE ) ;         //close OE
//       
//        global_motor_position_encoder[1] = data;
//        
//        ENABLE_INTERRUPT;
}
//
/////////////////////////////////////////////////////////////////////
int get_motor_position_step(unsigned char motor_number)
{
    //此函数为空，global_motor_position_step[]  被正常维护
	return TRUE;
}
/////////////////////////////////////////////////////////////////////
void one_step(unsigned char motor_number)
{
     if (motor_number == 0 )
        {
          //step pluse
		  PORTC |=0b00000001;
		  //wait 1us
          Wait(2);
		  if (global_motor_direction[0] == DIRECTION_FORWARD)
		    {
		     global_motor_position_step[0]++;
			}
		  else
			{
		     global_motor_position_step[0]--;
		    }
		  PORTC &=0b11111110;
		}
	 
	 if (motor_number == 1)
	   {
	      //step pluse
          PORTC|=0b00001000;
		  //wait 1us
		  Wait(0x2);
		  if (global_motor_direction[1] == DIRECTION_FORWARD)
		    {
		     global_motor_position_step[1]++;
			}
		  else
			{
		     global_motor_position_step[1]--;
		    }
		  PORTC&=0b11110111;
	   }

	 if (motor_number == 2)
	   {
	      //step pluse
          PORTA|=0b00100000;
		  //wait 1us
		  Wait(0x2);
		  if (global_motor_direction[1] == DIRECTION_FORWARD)
		    {
		     global_motor_position_step[1]++;
			}
		  else
			{
		     global_motor_position_step[1]--;
		    }
		  PORTA&=0b11011111;
	   }

	 if (motor_number == 3)
	   {
	      //step pluse
          PORTA|=0b00000100;
		  //wait 1us
		  Wait(0x2);
		  if (global_motor_direction[1] == DIRECTION_FORWARD)
		    {
		     global_motor_position_step[1]++;
			}
		  else
			{
		     global_motor_position_step[1]--;
		    }
		  PORTA&=0b11111011;
	   }
	 Wait(0x6);
}

/////////////////////////////////////////////////////////////////////
void set_direction(unsigned char motor_number,unsigned char direction)
{
     if (motor_number == 0)
       {
        if (direction == DIRECTION_FORWARD)
          {
		   PORTC &=0b11111101;
		   global_motor_direction[0] = DIRECTION_FORWARD;
		  }
		else
		   {
		   PORTC |=0b00000010;
		   global_motor_direction[0] = DIRECTION_REVERSE;
		   }
	   }
	 
     if (motor_number == 1)
       {
        if (direction == DIRECTION_FORWARD)
          {
		   PORTC &=0b11101111;
		   global_motor_direction[1] = DIRECTION_FORWARD;
		  }
		else
		   {
		   PORTC |=0b00010000;
		   global_motor_direction[1] = DIRECTION_REVERSE;
		   }
	   }

	 if (motor_number == 2)
       {
        if (direction == DIRECTION_FORWARD)
          {
		   PORTA &=0b11101111;
		   global_motor_direction[2] = DIRECTION_FORWARD;
		  }
		else
		   {
		   PORTA |=0b00010000;
		   global_motor_direction[2] = DIRECTION_REVERSE;
		   }
	   }

     if (motor_number == 3)
       {
        if (direction == DIRECTION_FORWARD)
          {
		   PORTA &=0b11111101;
		   global_motor_direction[3] = DIRECTION_FORWARD;
		  }
		else
		   {
		   PORTA |=0b00000010;
		   global_motor_direction[3] = DIRECTION_REVERSE;
		   }
	   }
	 
}

/////////////////////////////////////////////////////////////////////
void status_machine(void)
{
   unsigned char  motor_number;

   //读取当前状态
   get_motor_switch_status();
   //读取完成

   for ( motor_number=0;motor_number<1 ;motor_number++ )
    {
	 if (global_motor_status[motor_number]!=global_motor_goal_status[motor_number])   //是否目标状态
	    {
	     if (global_motor_goal_status[motor_number] == MOTOR_STATUS_STOP)   //是否正常运行状态
	       {
	        //正常运行状态
			if (global_run_step_number[motor_number]!=0)  //是否有未运行步数
			   {
			    if (global_counter_speed[motor_number] == 0) //是否到运行时隙
			       {
			        if ( ((global_motor_direction[motor_number] == DIRECTION_FORWARD) &&
						 (global_motor_status[motor_number]==MOTOR_STATUS_FORWARD_LIMIT))  
						|| 
						  ((global_motor_direction[motor_number] == DIRECTION_REVERSE) &&
						 (global_motor_status[motor_number]==MOTOR_STATUS_REVERSE_LIMIT)) ) //是否方向到达限位点
			           {
			            //到达限位点，且同方向运动
						//不执行任何运动，
						;
					   }
					else
					   {
					    //可以运行
						one_step(motor_number);
						//时隙处理
						global_counter_speed[motor_number] = global_counter_setting_speed[motor_number];
						//步数处理 
						global_run_step_number[motor_number]-- ;
						//全局位置处理已经在one_step()中处理
					    }
				   }
				 else
                   {
				    //没有到达运行时隙
					global_counter_speed[motor_number]--;
				    }
			   }
			 else
			   {
			    //已经运行完成所有步数
				//改变电机状态
				global_motor_status[motor_number] = MOTOR_STATUS_STOP; 
			    }
			
			}
		  else 
			{
		     //限位运行状态
             if (global_counter_low_speed[motor_number] == 0)
               {
                //速度计数器设初值
				global_counter_low_speed[motor_number] = global_counter_low_setting_speed[motor_number];
				//运行一步
		        if ( ((global_motor_direction[motor_number] == DIRECTION_FORWARD) &&
					 (global_motor_status[motor_number]==MOTOR_STATUS_FORWARD_LIMIT))  
					|| 
    				 ((global_motor_direction[motor_number] == DIRECTION_REVERSE) &&
					 (global_motor_status[motor_number]==MOTOR_STATUS_REVERSE_LIMIT)) ) //是否方向到达限位点
			           {
			            //到达限位点，且同方向运动
						//不执行任何运动，
						;
					   }
				else
					   {
					    //可以运行
						one_step(motor_number);
					    }

			   }
			 else
               {
			    global_counter_low_speed[motor_number]--;
			   }

		     }
	
         }

    }
    
   
  /*
  //  for 232b cancle by xxx 2011 02 11
    if ((global_yon_return_message==TRUE) && (global_motor_status[1]==MOTOR_STATUS_STOP)) 
            {
              global_yon_return_message = FALSE;
              function_return_message();
              return;
             
            }
    if ((global_yon_return_message==TRUE) && ((global_motor_status[1]==MOTOR_STATUS_REVERSE_LIMIT)&&(global_motor_goal_status[1]==MOTOR_STATUS_REVERSE_LIMIT)))
            {
              global_yon_return_message = FALSE;
              function_return_message();
             
            }
    if ((global_yon_return_message==TRUE) && ((global_motor_status[1]==MOTOR_STATUS_FORWARD_LIMIT)&&(global_motor_goal_status[1]==MOTOR_STATUS_FORWARD_LIMIT)))
            {
              global_yon_return_message = FALSE;
              function_return_message();
             
            }
   
   
    if ((global_yon_return_message==TRUE) && ((global_motor_status[1]==MOTOR_STATUS_STOP) || 
              (global_motor_status[1]==MOTOR_STATUS_REVERSE_LIMIT) ||(global_motor_status[1]==MOTOR_STATUS_FORWARD_LIMIT)
               ))
            {
              global_yon_return_message = FALSE;
              function_return_message();
             
            }
    */
   	 
}
/////////////////////////////////////////////////////////////////////
void get_board_id(void)
{
     //unsigned int data_int;
     //unsigned char data;
     //unsigned char motor_number;
     //SET READCS2  = 0
     //MPU_RCS0=0 MPU_RCS1=1 
     //AT91F_PIO_SetOutput( AT91C_BASE_PIOA, H_RCS1 ) ;         //RCS0=1
     //AT91F_PIO_ClearOutput( AT91C_BASE_PIOA, H_RCS0 ) ;       //RCS1=0
	 //READ DATA
     //Wait(0xfff);
     //data_int = AT91F_PIO_GetInput(AT91C_BASE_PIOA);
     //data = data_int >> 21;                                   //H_D0
     //global_board_id = data;     
  	 global_board_id = 0;
}

//////////////////////////////////////////////////////////////////////
//check the rs232 command from joystick
//and perform the command
/////////////////////////////////////////////////////////////////////
void check_fork_232_receive(void)
{
     unsigned char temp;
     unsigned char temp2;
     unsigned char temp3;
     if (global_yon_232_receive==TRUE)
       {
         global_yon_232_receive=FALSE;
         temp=global_232_receive_buffer[0]&0x3f; //0b00111111
         temp2=global_232_receive_buffer[0]&0x80;
         temp3=global_232_receive_buffer[0]&0x40;
         if (temp2)  //0b10000000
           {
            // motor number is 1
            if (temp==0)
               {
                //stop
                global_run_step_number[1]=0; 
               }
            else
               {
                 //run
                 //setting direction
                 if (temp3) //0b01000000
                    {
                     //direction forward
                     global_motor_direction[1]=DIRECTION_FORWARD;
                    }
                 else
                    {
                     //direction reverse
                     global_motor_direction[1]=DIRECTION_REVERSE; 
                    }
                 set_direction(1,global_motor_direction[1]);
                 //set direction end
                 //change speed
                 if (temp>32) 
                    {global_counter_setting_speed[1]=(63-temp)*2;}
                 else
                    {global_counter_setting_speed[1]=(63-temp)*64;}
                 //change speed end
                 //set step number and goal status
                 global_run_step_number[1]=100000;
                 global_motor_status[1]=MOTOR_STATUS_NORMAL_RUN;
                 global_motor_goal_status[1]=MOTOR_STATUS_STOP;  
                 //set step number and goal status end
               }
           }
         else
           {
            // motor number is 0
            if (temp==0)
               {
                //stop
                global_run_step_number[0]=0; 
               }
            else
               {
                 //run
                 //setting direction
                 if (temp3) //0b01000000
                    {
                     //direction forward
                     global_motor_direction[0]=DIRECTION_FORWARD;
                    }
                 else
                    {
                     //direction reverse
                     global_motor_direction[0]=DIRECTION_REVERSE; 
                    }
                 set_direction(0,global_motor_direction[0]);
                 //set direction end
                 //change speed
                 if (temp>32) 
                    {global_counter_setting_speed[0]=(63-temp)*2;}
                 else
                    {global_counter_setting_speed[0]=(63-temp)*64;}
                 //change speed end
                 //set step number and goal status
                 global_run_step_number[0]=100000;
                 global_motor_status[0]=MOTOR_STATUS_NORMAL_RUN;
                 global_motor_goal_status[0]=MOTOR_STATUS_STOP;  
                 //set step number and goal status end
               }
           } 
       }

}


//////////////////////////////////////////////////////////////////////
//check the rs232 command from PC protocol
//and perform the command
/////////////////////////////////////////////////////////////////////
void check_fork_232_receive_protocol(void)
{
     unsigned int i;
     unsigned char temp_receive_buffer[8];
     
	 //if (global_yon_232_receive_timeout=TRUE)
     //   {
     //    global_232_receive_byte_counter = 0;
	 //   }
     
	 if (global_yon_232_receive!=TRUE)
        { return; }
     // receive byte from 232
     global_yon_232_receive=FALSE;
     global_232_receive_buffer_protocol[global_232_receive_byte_counter]= global_232_receive_buffer[0];
     global_232_receive_byte_counter++;
  	 // set 232 receive timeout
	 global_yon_232_receive_timeout=FALSE;
	 timer3_init();
	 if (global_232_receive_byte_counter >= RS232_RECEIVE_BUFFER_LENGTH_PROTOCOL ) 
        {
        global_232_receive_byte_counter = 0;
		//close timeout 
		timer3_close();
        }
     else
        {
         return;
         }
      // receive protocol from 232
      // begin 232 fork
     temp_receive_buffer[0]=global_232_receive_buffer_protocol[0];//global_board_id;
     
	 //temp_receive_buffer[1]=global_232_receive_buffer_protocol[1];//motor number , 232 control motor number 0
     //change motor number
	 if (global_232_receive_buffer_protocol[1]==0) 
	    {
		 temp_receive_buffer[1]=3;
		 }      
     else if (global_232_receive_buffer_protocol[1]==1)
	    {
		 temp_receive_buffer[1]=0;
		 }      
     else if (global_232_receive_buffer_protocol[1]==2)
	    {
		 temp_receive_buffer[1]=2;
		 }      
     else if (global_232_receive_buffer_protocol[1]==3)
	    {
		 temp_receive_buffer[1]=1;
		 }      
     else 
	     {
	     temp_receive_buffer[1]=global_232_receive_buffer_protocol[1];
		 }
     //change end

     switch (global_232_receive_buffer_protocol[2])   //check command
         {
     case   1:      //run
		            temp_receive_buffer[2]=1;   //command
                    /*
					if (global_232_receive_buffer_protocol[1]==0 )
                    {
                    //temp_receive_buffer[3]=1;
                    temp_receive_buffer[3]=global_232_receive_buffer_protocol[1]; //direction
                    }
                    if (global_232_receive_buffer_protocol[1]==1 )
                    {
                    //temp_receive_buffer[3]=0;
                    temp_receive_buffer[3]=global_232_receive_buffer_protocol[1]; //direction
                    }
                    */
                    temp_receive_buffer[3]=global_232_receive_buffer_protocol[3]; //par1
                    
                    temp_receive_buffer[4]=global_232_receive_buffer_protocol[4]; //par1
                    temp_receive_buffer[5]=global_232_receive_buffer_protocol[5]; //par2
                    temp_receive_buffer[6]=global_232_receive_buffer_protocol[6]; //par3
                    temp_receive_buffer[7]=global_232_receive_buffer_protocol[7]; //par4
            break;
     case   2:      //query
		            temp_receive_buffer[2]=2;
                    temp_receive_buffer[3]=0; //direction
                    temp_receive_buffer[4]=0; //par1
                    temp_receive_buffer[5]=0; //par2
                    temp_receive_buffer[6]=0; //par3
                    temp_receive_buffer[7]=0; //par4
            break;
     case   4:      //clear current position
		            temp_receive_buffer[2]=4;
                    temp_receive_buffer[3]=0; //direction
                    temp_receive_buffer[4]=0; //par1
                    temp_receive_buffer[5]=0; //par2
                    temp_receive_buffer[6]=0; //par3
                    temp_receive_buffer[7]=0; //par4
            break;
	 case    6:
		            //goto reference
		            temp_receive_buffer[2]=6;
                    temp_receive_buffer[3]=0; //direction
                    temp_receive_buffer[4]=0; //par1
                    temp_receive_buffer[5]=0; //par2
                    temp_receive_buffer[6]=0; //par3
                    temp_receive_buffer[7]=0; //par4
	        break;
	 case    7:
		            //goto forward limit
		            temp_receive_buffer[2]=7;
                    temp_receive_buffer[3]=0; //direction
                    temp_receive_buffer[4]=0; //par1
                    temp_receive_buffer[5]=0; //par2
                    temp_receive_buffer[6]=0; //par3
                    temp_receive_buffer[7]=0; //par4
	        break;
	 case    8:
		            //goto reverse limit
		            temp_receive_buffer[2]=8;
                    temp_receive_buffer[3]=0; //direction
                    temp_receive_buffer[4]=0; //par1
                    temp_receive_buffer[5]=0; //par2
                    temp_receive_buffer[6]=0; //par3
                    temp_receive_buffer[7]=0; //par4
	        break;
	 case    9:
		            //stop
		            temp_receive_buffer[2]=9;
                    temp_receive_buffer[3]=0; //direction
                    temp_receive_buffer[4]=0; //par1
                    temp_receive_buffer[5]=0; //par2
                    temp_receive_buffer[6]=0; //par3
                    temp_receive_buffer[7]=0; //par4
	        break;
	 case   10:     //test ?
		    ;
            break;
     case   21:     //goto forward limit
                    temp_receive_buffer[2]=7;
                    temp_receive_buffer[3]=0; //direction
                    temp_receive_buffer[4]=0; //par1
                    temp_receive_buffer[5]=0; //par2
                    temp_receive_buffer[6]=0; //par3
                    temp_receive_buffer[7]=0; //par4
            break;
     case   22:     //goto reverse limit 
                    temp_receive_buffer[2]=8;
                    temp_receive_buffer[3]=0; //direction
                    temp_receive_buffer[4]=0; //par1
                    temp_receive_buffer[5]=0; //par2
                    temp_receive_buffer[6]=0; //par3
                    temp_receive_buffer[7]=0; //par4
            break;
     case   25:     //set delay time
        		    temp_receive_buffer[2]=10;
                    temp_receive_buffer[3]=0; //direction
                    temp_receive_buffer[4]=global_232_receive_buffer_protocol[4]; //par1
                    temp_receive_buffer[5]=global_232_receive_buffer_protocol[5]; //par2
                    temp_receive_buffer[6]=global_232_receive_buffer_protocol[6]; //par3
                    temp_receive_buffer[7]=global_232_receive_buffer_protocol[7]; //par4                    
            break;
     case   250:    //reset mpu
            break;
     case   255:    //stop motor
                    temp_receive_buffer[2]=9;
                    temp_receive_buffer[3]=0; //direction
                    temp_receive_buffer[4]=0; //par1
                    temp_receive_buffer[5]=0; //par2
                    temp_receive_buffer[6]=0; //par3
                    temp_receive_buffer[7]=0; //par4
            break;
     default :
            return; //none command
         }      
     
     for (i=0;i<8;++i)
          {
           global_receive_buffer[i]=temp_receive_buffer[i];
          }
     //set global_yon_receive flag
     global_yon_receive = TRUE;
     global_message_type = RS232_MESSAGE_TYPE;
     // goto main fork loop      
}

////////////////////////////////////////////////////////////////////////
void send_status_old(void)
{
     unsigned char temp_motor_status;
     cli();
	 get_motor_switch_status();
	 temp_motor_status = global_motor_status[0];
     sei();
     
	 if (temp_motor_status==MOTOR_STATUS_FORWARD_LIMIT) 
	    {
         if (global_yon_already_send_s==TRUE)
		    {
			 exit;
			}
         else
		    {
		    global_message_type = RS232_MESSAGE_TYPE;
            //return message for 232
            global_232_send_buffer[0] = 'S';
            global_232_send_buffer_length = 1;
            //return message end
            function_return_message();
    	    //Wait(0xff);
		    global_yon_already_send_s = TRUE;
     	    global_yon_already_send_t = FALSE;
     	    }
		}
     else
	    {
         if (global_yon_already_send_t==TRUE)
		    {
			 exit;
			}
		 else
		    {
		    global_message_type = RS232_MESSAGE_TYPE;
            //return message for 232
            global_232_send_buffer[0] = 'T';
            global_232_send_buffer_length = 1;
            //return message end
            function_return_message();
		    //Wait(0xff);
     	    global_yon_already_send_s = FALSE;
     	    global_yon_already_send_t = TRUE;
     	    }
		}
 
     if (temp_motor_status==MOTOR_STATUS_REVERSE_LIMIT) 
	    {
		  //send 5v single
		  //pc3
  		  PORTC|=0b00001000;
		  //wait 1us
		  //Wait(0xf);
		}
      else
	    {
     	  PORTC&=0b11110111;		
		  //Wait(0xf);
         }
	  
    
}

////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
void send_status(void)
{
     unsigned char temp_motor_status;
     unsigned char temp;
	 cli();
	 get_motor_switch_status();
	 temp_motor_status = global_motor_status[0];
     sei();
     
	 if (temp_motor_status==MOTOR_STATUS_FORWARD_LIMIT) 
	    {
         if (global_yon_already_send_s==TRUE)
		    {
			 exit;
			}
         else
		    {
		    global_message_type = RS232_MESSAGE_TYPE;
            //return message for 232
            global_232_send_buffer[0] = 'S';
            global_232_send_buffer_length = 1;
            //return message end
            function_return_message();
    	    //Wait(0xff);
		    global_yon_already_send_s = TRUE;
     	    global_yon_already_send_t = FALSE;
     	    }
         return;
		}
 
     //check pd6 interval switch
     temp=PIND;
	 //need not  by xxx 20111109
	 temp=(~temp);
	 //end by xxx 20111109
	 temp&=0b01000000;  
	 if (temp!=0)
	    {
		 ;//NO MAG 
		}
     else 
	    {
		;// NEED START
	    if (global_yon_already_send_t==TRUE)
		   {
		    exit;
		   }
        else
		   {
		   global_message_type = RS232_MESSAGE_TYPE;
           //return message for 232
           global_232_send_buffer[0] = 'T';
           global_232_send_buffer_length = 1;
            //return message end
           function_return_message();
	       //Wait(0xff);
  	       global_yon_already_send_s = FALSE;
   	       global_yon_already_send_t = TRUE;
		   }

		}
     
	 if (temp_motor_status==MOTOR_STATUS_REVERSE_LIMIT) 
	    {
		  //send 5v single
  		  PORTC|=0b00001000;
		  //wait 1us
		  //Wait(0xf);
         if (global_yon_already_send_t==TRUE)
		    {
			 exit;
			}
		 else
		    {
		    global_message_type = RS232_MESSAGE_TYPE;
            //return message for 232
            global_232_send_buffer[0] = 'T';
            global_232_send_buffer_length = 1;
            //return message end
            function_return_message();
		    //Wait(0xff);
     	    global_yon_already_send_s = FALSE;
     	    global_yon_already_send_t = TRUE;
     	    }
		  return;
		}
      else
	    {
     	  PORTC&=0b11110111;		
		  //Wait(0xf);
         }

      
	  //check key
	  //check pb2
	  temp=PINB;
	  temp&=0b00000100;
	  if (temp==0) 
		 {
	      //pb2 low ;stop ;key press
		    if (global_yon_stop_key ==FALSE)
			{ 
       	    Wait(0x3ff);
    	    global_message_type = RS232_MESSAGE_TYPE;
            //return message for 232
            global_232_send_buffer[0] = 'S';
            global_232_send_buffer_length = 1;
            //return message end
            function_return_message();
    	    global_yon_stop_key=TRUE;
			}
		  }
	  else
	     {
	      //pb2 high ; start
          if (global_yon_stop_key == TRUE )
 		    {
	        Wait(0x3ff);
			global_message_type = RS232_MESSAGE_TYPE;
            //return message for 232
            global_232_send_buffer[0] = 'T';
            global_232_send_buffer_length = 1;
            //return message end
            function_return_message();
     	    global_yon_stop_key = FALSE;
		    }

		 }
}

////////////////////////////////////////////////////////////////////////////////////////
unsigned char goto_reference(unsigned char number)
{
	 unsigned char temp_status1,temp_status2,temp_status3;
     int i;
	 long k;

	 if (number > 3)
	    {
	     return FALSE;
		}

     k=0;
     if ((number==0)||(number==2)||(number==3))
        {
         //set unsleep
         motor_unsleep(number);
	     Wait(0xff);
	     //

         //set direction
	     set_direction(number,DIRECTION_FORWARD);
	     //
		
         process_message_init(); 
         
		 //do
         //   {
  	     //    get_motor_switch_status();
	     //    one_step(number);k++;
		 //	 } while ((global_motor_status[number] == MOTOR_STATUS_REFERENCE)&&(global_message == RUN)&&(k<13000) );

          get_motor_switch_status(); 	       
          if (global_motor_status[number]==MOTOR_STATUS_REFERENCE) {
		        for (i=0;i<2000;++i)
		        {
	             one_step(number);
			    }
				}

	      process_message_close();
	      if ((global_message == STOP)||(k>13000))
	        {
	         return FALSE;
	        }

          k=0;	 
	      process_message_init(); 
		  do
            {
  	         get_motor_switch_status();
	         one_step(number);k++;
             } while ((global_motor_status[number] != MOTOR_STATUS_REFERENCE)&&(global_message == RUN)&&(k<13000) );
	      process_message_close();
          
		  for (i=0;i<1000;++i)
	        {
             one_step(number);
		    }


          //set sleep
	      Wait(0xfff);
          motor_sleep(number);
	      Wait(0xf);
	      //

	      if ((global_message == RUN)&&(k<13000))
	         {
	          return TRUE;
	          }
	      else 
		      {
		       return FALSE;
		       }
          }        
   
     // motor 1 reference

     //set unsleep
     motor_unsleep(number);
	 Wait(0xff);
	 //

     //set direction
	 set_direction(number,DIRECTION_FORWARD);
	 //
          
     k=0;     
     temp_status1=1;
     temp_status2=2;
     temp_status3=3;

     do {
     //FIRST GET MOTOR STATUS
     get_motor_switch_status();
	 temp_status1=global_motor_status[number];
     
	 for (i=1;i<100;++i){one_step(number);k++;} 

     //second GET MOTOR STATUS
     get_motor_switch_status();
	 temp_status2=global_motor_status[number];

	 for (i=1;i<100;++i){one_step(number);k++;} 
     
     //third GET MOTOR STATUS
     get_motor_switch_status();
	 temp_status3=global_motor_status[number];
     }
     while ((temp_status1!=temp_status2)||((temp_status1!=temp_status3))||((temp_status2!=temp_status3)));	  

     if (temp_status3==MOTOR_STATUS_REFERENCE)
	     {
          process_message_init(); 
	      do
            {
  	         get_motor_switch_status();
	         one_step(number);k++;
             } while ((global_motor_status[number] == MOTOR_STATUS_REFERENCE)&&(global_message == RUN) );
	      process_message_close();
	      if (global_message == STOP)
	        {
	         return FALSE;
	        }
	 
    	  for (i=1;i<1000;++i){one_step(number);k++;} 

	      process_message_init(); 
	      do
           {
  	        get_motor_switch_status();
	        one_step(number);k++;
           } while ((global_motor_status[number] != MOTOR_STATUS_REFERENCE)&&(global_message == RUN) );
	      process_message_close();
         }
     else
	     {
	      process_message_init(); 
	      do
           {
  	        get_motor_switch_status();
	        one_step(number);k++;
           } while ((global_motor_status[number] != MOTOR_STATUS_REFERENCE)&&(global_message == RUN) );
	      process_message_close();
		 } 

     //REFERENCE  TWICE
	 k=0; 
     temp_status1=1;
     temp_status2=2;
     temp_status3=3;

     do {
     //FIRST GET MOTOR STATUS
     get_motor_switch_status();
	 temp_status1=global_motor_status[number];
     
	 for (i=1;i<100;++i){one_step(number);k++;} 

     //second GET MOTOR STATUS
     get_motor_switch_status();
	 temp_status2=global_motor_status[number];

	 for (i=1;i<100;++i){one_step(number);k++;} 
     
     //third GET MOTOR STATUS
     get_motor_switch_status();
	 temp_status3=global_motor_status[number];
     }
     while ((temp_status1!=temp_status2)||((temp_status1!=temp_status3))||((temp_status2!=temp_status3)));	  

     if (temp_status3==MOTOR_STATUS_REFERENCE)
	     {
          process_message_init(); 
	      do
            {
  	         get_motor_switch_status();
	         one_step(number);k++;
             } while ((global_motor_status[number] == MOTOR_STATUS_REFERENCE)&&(global_message == RUN) );
	      process_message_close();
	      if (global_message == STOP)
	        {
	         return FALSE;
	        }
	 
    	  for (i=1;i<1000;++i){one_step(number);k++;} 

	      process_message_init(); 
	      do
           {
  	        get_motor_switch_status();
	        one_step(number);k++;
           } while ((global_motor_status[number] != MOTOR_STATUS_REFERENCE)&&(global_message == RUN) );
	      process_message_close();
         }
     else
	     {
	      process_message_init(); 
	      do
           {
  	        get_motor_switch_status();
	        one_step(number);k++;
           } while ((global_motor_status[number] != MOTOR_STATUS_REFERENCE)&&(global_message == RUN) );
	      process_message_close();
		 } 

     //
	
	 if (k<100000) 
	    {
		
	 k=0; 
     temp_status1=1;
     temp_status2=2;
     temp_status3=3;

     do {
     //FIRST GET MOTOR STATUS
     get_motor_switch_status();
	 temp_status1=global_motor_status[number];
     
	 for (i=1;i<100;++i){one_step(number);k++;} 

     //second GET MOTOR STATUS
     get_motor_switch_status();
	 temp_status2=global_motor_status[number];

	 for (i=1;i<100;++i){one_step(number);k++;} 
     
     //third GET MOTOR STATUS
     get_motor_switch_status();
	 temp_status3=global_motor_status[number];
     }
     while ((temp_status1!=temp_status2)||((temp_status1!=temp_status3))||((temp_status2!=temp_status3)));	  

     if (temp_status3==MOTOR_STATUS_REFERENCE)
	     {
          process_message_init(); 
	      do
            {
  	         get_motor_switch_status();
	         one_step(number);k++;
             } while ((global_motor_status[number] == MOTOR_STATUS_REFERENCE)&&(global_message == RUN) );
	      process_message_close();
	      if (global_message == STOP)
	        {
	         return FALSE;
	        }
	 
    	  for (i=1;i<1000;++i){one_step(number);k++;} 

	      process_message_init(); 
	      do
           {
  	        get_motor_switch_status();
	        one_step(number);k++;
           } while ((global_motor_status[number] != MOTOR_STATUS_REFERENCE)&&(global_message == RUN) );
	      process_message_close();
         }
     else
	     {
	      process_message_init(); 
	      do
           {
  	        get_motor_switch_status();
	        one_step(number);k++;
           } while ((global_motor_status[number] != MOTOR_STATUS_REFERENCE)&&(global_message == RUN) );
	      process_message_close();
		
		}

     }
     //set sleep
     Wait(0xfff);
	 motor_sleep(number);
	 Wait(0xff);
	 //

	 if (global_message == RUN)
	   {
	    return TRUE;
	   }
	 else 
		{
		 return FALSE;
		}
}
////////////////////////////////////////////////////////////////////////////////////////
void process_message_init(void)
{
     global_yon_process_message = TRUE;
	 global_message = RUN;
}
////////////////////////////////////////////////////////////////////////////////////////
void process_message(unsigned char message)
{
     if (message == 'S')
        {
       	 global_message = STOP;
		}
}
////////////////////////////////////////////////////////////////////////////////////////
void process_message_close(void)
{
     global_yon_process_message = FALSE;
}
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
void motor_sleep(unsigned char number)
{
 //sleep all
 // OLD DRIVER SLEEP HIGH EFFECT
 // NEW DRIVER SLEEP LOW EFFECT
	PORTC &=0b11111011;
	PORTC &=0b11011111;
	PORTA &=0b11110111;
	PORTA &=0b11111110;
  return ;
//END
 
 if (number==0)
   {
   	//PC2 SLEEP0
	PORTC |=0b00000100;
   }
 else if (number==1)
   {
   	//pc5 sleep1
	PORTC |=0b00100000;
   }
 else if (number==2)
   {
   	//PA3 SLEEP2
	PORTA |=0b00001000;
   }
 else if (number==3)
   {
   	//PA0 SLEEP3
	PORTA |=0b00000001;
   }
}

void motor_unsleep(unsigned char number)
{
 //unsleep all
 // OLD DRIVER SLEEP HIGH EFFECT
 // NEW DRIVER SLEEP LOW EFFECT
 	PORTC |=0b00000100;
 	PORTC |=0b00100000;
 	PORTA |=0b00001000;
 	PORTA |=0b00000001;
  return ;
//END
 
 if (number==0)
   {
   	//PC2 SLEEP0
	PORTC &=0b11111011;
   }
 else if (number==1)
   {
   	//pc5 sleep1
	PORTC &=0b11011111;
   }
 else if (number==2)
   {
   	//PA3 SLEEP2
	PORTA &=0b11110111;
   }
 else if (number==3)
   {
   	//PA0 SLEEP3
	PORTA &=0b11111110;
   }
}


////////////////////////////////////////////////////////////////////////////////////////
// 主板 pin1－7
// 1－ PA7
// 2-  PC6
// 3-  PC7
// 4-  PG2
// 5-  PA6
// 6-  +5V VCC
// 7-  GND
// pcb 电机排列    1     2   0   3
// 对应 外部排列   3     2   1   0
// 对应 LED        PA6  PG2 PC7 PC6
// 主供电指示LED PA7     
// 函数参数对应于内部排列号码，点亮 对应LED 号码
void open_led(unsigned char k)
{
     if (k==0) 
	 {
	     PORTC&=(~(1<<7));
	 }
	 else if (k==1)
	 {
	     PORTA&=(~(1<<6));
	 }
	 else if (k==2)
	 {
	     PORTG&=(~(1<<2));
	 }
     else if (k==3)
	 {
	     PORTC&=(~(1<<6));
	 }
}
void close_led(unsigned char k)
{
     if (k==0) 
	 {
	     PORTC|=(1<<7);
	 }
	 else if (k==1)
	 {
	     PORTA|=(1<<6);
	 }
	 else if (k==2)
	 {
	     PORTG|=(1<<2);
	 }
     else if (k==3)
	 {
	     PORTC|=(1<<6);
	 }

}

void open_power_led(void)
{
//power led pa7
   PORTA&=(~(1<<7));
}
void close_power_led(void)
{
   PORTA|=(1<<7);
}

////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////
//通讯协议
//USB AND 232
//
//所有参数均是高位在前，低位在后。
//
//send  
// 8 BYTE 
//
// 0 byte   board id   
// 1 byte   motor number
// 2 byte   command
// 3 byte   direction
// 4 byte   step number 1
// 5 byte   step number 2
// 6 byte   step number 3
// 7 byte   step number 4
//
// board id 1-254          reserved 0 for read board id command
//
// motor number 0-1
// command 
//            1 run                        (5 byte par)             return message command id
//            2 read step position         (no par)                 return message step position
//            3 read encoder position      (no par)                 return message encoder position
//            4 clear step position        (no par)                 return message command id
//            5 clear encoder position     (no par)                 return message command id
//            6 goto reference             (no par)                 return message command id
//            7 goto forward limit         (no par)                 return message command id
//            8 goto reverse limit         (no par)                 return message command id
//            9 stop                       (no par)                 return message command id
//            10 set speed                 (5-8 4 byte par)         return message command id
//            11 set accel                 (5-8 4 byte par)         return message command id 
//            12 set decel                 (5-8 4 byte par)         return message command id
//            13 read speed                (no par)                 return message speed
//            14 read accel                (no par)                 return message accel
//            15 read decel                (no par)                 return message decel
//            33 read status               (no par)                 return message status
//            56 read board id             (no par)                 return message command id inculd board id
//
// reserve 0 and 255
//
// return message
// 8 byte
// 
// 1 byte borad id
// 2 byte motor number
// 3 byte command id
// 4 byte status id
// 5 byte par1
// 6 byte par2
// 7 byte par3
// 8 byte par4
// 
// board id 0-255
// motor id 0-1
// command id
// status id
//            MOTOR_STATUS_STOP 0
//            MOTOR_STATUS_RUN 1
//            MOTOR_STATUS_FORWARD_LIMIT 2
//            MOTOR_STATUS_REVERSE_LIMIT 3
//            MOTOR_STATUS_REFERENCE 4
// par1
// par2
// par3
// par4


// for 10us time slit
// input speed ---------------- step per second
// input accel ---------------- step per second
// input decel ---------------- step per second
// counter_speed -------------- 100000/step per second
// counter_speed at accel------ 100000/(input accel *k) till <= counter_speed
// counter_speed at decel------ counter_speed+100000/(input decel * k) till >= 0







/*
// hardware 描述 
// motor 步进电机板载程序

用于MOTOR 步进电机驱动板
包含步进电机驱动芯片A3979
RENISAHW 的ENCODE 
2008 10 18


////////////////////////////////////////////////
硬件说明
////////////////////////////////////////////////

MOTOR A3979
PA31    RESET    OUTPUT    LOW EFFECT
PA0     STEP1    OUTPUT    LOW TO HIGH TRANSITION
PA15    STEP2
PA1     DIR1     OUTPUT    HIGH INCREASE  LOW MINUS
PA16    DIR2
PA2     SLEEP1
PA3     SLEEP2

ENCODER  HCTL2032
PA10    MPU_XY    OUTPUT    CHOICE X OR Y , LOW FIRST COUNTER, HIGH SECOND COUNTER
PA29    MPU_RSTX  OUTPUT    CLEAR COUNTER LOW EFFECT
PA30    MPU_RSTY  OUTPUT    CLEAR COUNTER LOW EFFECT
PA17    MPU_SEL1  OUTPUT    READ DATA  
PA18    MPU_SEL2  OUTPUT    READ DATA
                            MPU_SEL1   MPUSEL2   DATR
                            0          1         D4     MSB
                            1          1         D3 
                            0          0         D2
                            1          0         D1     LSB

PA28    MPU_D7    INPUT
PA27    MPU_D6    INPUT
PA26    MPU_D5    INPUT
PA25    MPU_D4    INPUT
PA24    MPU_D3    INPUT
PA23    MPU_D2    INPUT
PA22    MPU_D1    INPUT
PA21    MPU_D0    INPUT

PA20    INTERRUPT    IRQ0   RECEIVE MOTOR LIMIT POSITION , LOW EFFECT ?

PA7     MPU_RCS0  OUTPUT CONTROL READCS
PA8     MPU_RCS1  OUTPUT CONTROL READCS
                  READCS0  MPU_RCS0=0 MPURCS1=0    
                  READCS1  MPU_RCS0=1 MPURCS1=0
                  READCS2  MPU_RCS0=0 MPURCS1=1                          
                  
                  READCS0  READ HCTL2032 DATA
                  READCS1  READ MOTOR LIMIT STATUS
                  READCS2  READ BOARD ADDRESS
                  
                  READCS1  MOTOR LIMIT STATUS
                           MPU_D0 -> Z1
                           MPU_D1 -> Z2
                           MPU_D2 -> P1
                           MPU_D3 -> Q1
                           MPU_D4 -> P2
                           MPU_D5 -> Q2

SPI
PA12    MISO         SPI
PA13    MOSI         SPI
PA14    SPCK         SPI
PA11    NPCS         SPI     45DB161 FLASH MEMORY

RS232 TO MAX3221
PA4     EN232        OUTPUT  LOW FORCE OFF TXD, HIGH OPEN TXD, ALLOW RECEIVE ANYTIME 
                             CONTROL LED
                             LOW ,LIGHT LED
                             HIGH,OFF LED
PA5     RXD          RXD
PA6     TXD          TXD

USB
PA9     USB_EN       OUTPUT  LOW ENABLE USB,HIGH OFF USB

OTHER
PA19    EX1          FIQ    INPUT OR OUTPUT

//////////////////////////////////////////////////////////////////////
*/


//////////////////////////////////////////////////////////////////////
//
//旧电机串行接口协议
//
//6 byte 接口协议
// 第一字节  命令字
//                  1   run 
//                  2   query
//                  25  set delay time (run speed)
//                  21  goto forward limit
//                  22  goto reverse limit
//                  4   clear current position
//                  250 reset mpu
//                  255 stop run
//                  10  test ?
// 第二字节  方向
//                   0:forward 
//                   1:reverse
// 4字节
//                   if set motor step:            
//                                4byte : step number (high bit at first)
//                   if set time:
//                               1byte: delay ms
//                               1byte: delay us
//                               1byte: 0
//                               1byte: 0
//
//
//下位机应答
//
//    tempstring='OK'			电机到指定位置或者电机参数执行完毕
//    tempstring='FORWARD_LIMIT'		电机到达正方向限位器
//    tempstring='REVERSE_LIMIT'		电机到达反方向限位器
//    tempstring='TEMP_STOP'		已取消此功能
//    tempstring='REFERENCE_POS'		已取消此功能
//    在上位机查询电机位置指令时，应答为4字节长整形
//
////////////////////////////////////////////////////////////////////////
