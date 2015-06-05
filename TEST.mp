NOTE: AVR Code addresses are word addresses
All other addresses (including ones in FLASH) are byte addresses

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                            text   01C8   06DE =   1758. bytes (rel,con,rom)

       Addr  Global Symbol
      -----  --------------------------------
       00E4  __start
       00E4  __text_start
       0106  _exit
       0107  _timer1CAP_isr
       0158  _timer3CAP_isr
       0185  _timer4CAP_isr
       01BA  _timer5CAP_isr
       01E7  _uart0_isr
       01E8  _uart1_isr
       01E9  _USART0_Init
       01F7  _USART0_Transmit
       01FE  _USART1_Init
       020C  _USART1_Transmit
       0213  _motor1_start
       0236  _motor1_stop
       023C  _motor1_slow
       0248  _motor2_start
       026B  _motor2_stop
       0271  _motor2_slow
       027D  _motor3_start
       02A0  _motor3_stop
       02A6  _motor3_slow
       02B2  _motor4_start
       02C9  _motor4_stop
       02CB  _motor4_slow
       02D3  _motor5_start
       02EA  _motor5_stop
       02EC  _motor5_slow
       02F4  _motor6_start
       0309  _motor6_stop
       030B  _motor6_slow
       0313  _timer1_init
       032C  _timer3_init
       0345  _timer4_init
       0360  _timer5_init
       036C  _init_device
       03E8  _delayms
       0436  _main
       0453  __text_end

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                             bss   0200   0032 =     50. bytes (rel,con,ram)

       Addr  Global Symbol
      -----  --------------------------------
       0200  _motor5Step
       0200  __bss_start
       0204  _motor4Step
       0208  _motor3Step
       020C  _motor2Step
       0210  _motor1Step
       0214  _Direct5
       0216  _Position5
       021A  _Direct4
       021C  _Position4
       0220  _Direct3
       0222  _Position3
       0226  _Direct2
       0228  _Position2
       022C  _Direct1
       022E  _Position1
       0232  __bss_end

Area                               Start  End    Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                          vector   0000   00BC =    188. bytes (abs,ovr,rom)

Files Linked      [ module(s) ]

D:\Èí¼þ2\iccv7avr\lib\crtatmega.o	[ crtatmega.s ]
motor.o	[ motor.c ]
<library>	

User Global Definitions

ram_end = 0x21ff
hwstk_size = 0x1e

User Base Address Definitions

func_lit = 0x1c8
data = 0x200
eeprom:0.4096

