/////////////////////////////////////////////
// Type 선언
/////////////////////////////////////////////
typedef unsigned char	INT8U;			//Unsigned  8 bit quantity
typedef signed	 char	INT8S;			//Signed    8 bit quantity
typedef unsigned int	INT16U; 		//Unsigned 16 bit quantity
typedef signed	 int	INT16S; 		//Signed   16 bit quantity
typedef unsigned long	INT32U; 		//Unsigned 32 bit quantity
typedef signed	 long	INT32S; 		//Signed   32 bit quantity
typedef float			FP32;			//Single precision floating point
typedef double			FP64;			//Double precision floating point
#define _NULL    '\0'     // NULL은 Null Pointer 지칭에 사용(PreDefine되어 있음)
                          // _NULL은 String의 끝에 사용 (waring이 나오는것을 지우기 위함)
#define BYTE			INT8U
#define WORD			INT16U
#define LONG			INT32S
#define DWORD			INT32U
#define UINT			INT16U

typedef UINT		    WPARAM;
typedef LONG		    LPARAM;
typedef LONG		    LRESULT;
typedef UINT		    ATOM;

#define PASCAL		    _pascal
#define WINAPI		    _far _pascal
#define CALLBACK	    _far _pascal
#define FALSE		    0
#define TRUE		    1

//******************************************************/
// Kinhead.h
//******************************************************/
// 변환식...
#define 	DEGTORAD 	0.017453293
#define 	RADTODEG 	57.29577951
#define 	EPSI 		0.000001
// #define 	PI 			3.141592654 // Arduino에서 더 정확한 값을 사용한다.
#define 	PI_DEG 		180.0
#define 	LARGE 		100000.0

// Degree와 Position의 변환식.
#define CONV_D2P	3.41		// 1023/300 = 3.41
#define CONV_P2D	0.293255	// 300/1023 = 0.29325513196480938416422287390029

// DH Parameter
#define		DH_d2 	0
#define		DH_a3 	80.5
#define		DH_a4 	80.5
#define		DH_a6 	32.2

#define 	B_DH_a1	 73.4
#define 	B_DH_a2	-14.9
#define 	B_DH_a3	 68.0
#define 	B_DH_a4	 94.0

// [0] == 우, [1] == 좌
#define RIGHT_LEG	0
#define LEFT_LEG	1
#define RIGHT		0
#define LEFT		1

// for debugging
#define check 	_printF(F("check\r\n")); getSerialChar();
// 역기구학의 최대 해의 개수
#define SOL_1	0
#define SOL_2	2
#define MAX_SOL	4

// AX-12의 속도 변환
#define NORMAL			 1.00	// 주기적인 제어를 하지 않는 경우
#define PERIOD_20MSEC	21.89	// 20msec마다 제어하는 경우
#define PERIOD_10MSEC	43.86	// 10msec마다 제어하는 경우.
// 중간 중간의 delay 요인(printf)를 없앴더니 속도변환 30으로는 제대로 처리되지 않는다???
#define EXPERIENCE		50.00	// 경험에 의한 값...

#define SPEED_MIN	10
#define SPEED_MAX	1023

// 각 관절의 무게, 단위 : g
#define G_C_MC	226	
#define G_C_M3	172
#define G_C_M4	86
#define G_C_M6	176

#define G_C_MA1	586
#define G_C_MA3	74
#define G_C_MA4	74

// 위치
#define P_BC_0	0
#define P_BC_4	1
#define P_BC_6	2
#define P_BC_F	3

#define PX	0
#define PY	1
#define PZ	2

#define X	0
#define Y	1
#define Z	2

#define POS	3

// 초기 X 위치. ( {BC}를 기준 좌표계로 함. )
#define X_BASE_POS      -220

// 각 관절의 위치 Index
#define R6 0
#define R4 1
#define R3 2
#define BC 3
#define L3 4
#define L4 5
#define L6 6
#define LEG_TORQUE 7

#define AR1	7
#define AR3	8
#define AR4	9
#define AL3	10
#define AL4	11
#define ALL_TORQUE 12

#define BR1	0
#define BR3	1
#define BR4	2
#define BL3	3
#define BL4	4
#define BODY_TORQUE 5

#define G_Acc 9.8

//******************************************************/
// Timer.h
//******************************************************/
// 8254 제어 레지스터
// DOS -> Arduino로 변경
//#define  TICK_T0_8254_CWR             0x43       /* 8254 PIT Control Word Register address.            */
//#define  TICK_T0_8254_CTR0            0x40       /* 8254 PIT Timer 0 Register address.                 */
//#define  TICK_T0_8254_CTR1            0x41       /* 8254 PIT Timer 1 Register address.                 */
//#define  TICK_T0_8254_CTR2            0x42       /* 8254 PIT Timer 2 Register address.                 */

//#define  TICK_T0_8254_CTR0_MODE3      0x36       /* 8254 PIT Binary Mode 3 for Counter 0 control word. */
//#define  TICK_T0_8254_CTR2_MODE0      0xB0       /* 8254 PIT Binary Mode 0 for Counter 2 control word. */
//#define  TICK_T0_8254_CTR2_LATCH      0x80       /* 8254 PIT Latch command control word                */

//#define  VECT_TICK                    0x08       /* Vector number for 82C54 timer tick                 */

//******************************************************/
// Serial.h
//******************************************************/
// 화면 출력 없이 RX Data를 없앰.
#define DUMP_RX_DATA	{while (IsReadyToRX) g_Qfirst++;}

// 포트 번호 상수
// DOS -> Arduino로 변경
//#define COM1	0
//#define	COM2	1
//#define	COM3	2
//#define	COM4	3

// 포트의 기본 주소
#define COM1BASE		0x3F8
#define COM2BASE		0x2F8
#define COM3BASE		0x3E8
#define COM4BASE		0x2E8

// 포트의 인터럽트 벡터 번호
#define COM13_INTERRUPT		0x0C
#define COM24_INTERRUPT		0x0B

// 인터럽트 마스크
#define MASKON13	0xEF
#define MASKON24	0xF7
#define MASKOFF13	0x10
#define MASKOFF24	0x08

// 데이터 비트 상수
#define DATABIT7		0x02
#define DATABIT8		0x03

// 정지 비트 상수
#define STOPBIT1		0x00
#define STOPBIT2		0x04

// 패리티 비트 상수
#define PARITYNONE	0x00
#define PARITYODD	0x08
#define PARITYEVEN	0x18

// 전송속도
#define BAUD1200		0x80
#define BAUD2400		0xA0
#define BAUD4800		0xC0
#define BAUD9600		0xE0

#define BPS1200		1200L
#define BPS2400		2400L
#define BPS4800		4800L
#define BPS9600		9600L
#define BPS19200	19200L
#define BPS38400	38400L
#define BPS57600	57600L
#define BPS115200	115200L

// UART 레지스터를 가리키기 위한 오프셋
#define IER_OFFSET	1
#define LCR_OFFSET	3
#define MCR_OFFSET	4
#define LSR_OFFSET	5
#define MSR_OFFSET	6

// PIC 칩에 있는 레지스터의 포트 번호
#define PICMASK	0x21
#define PICEIO	0x20

// 데이터 저장을 위한 버퍼와 버퍼 정보 변수
// Queue Index를 증감할때 Rount되도록 하기 위함.
// #define BUFFER_LENGTH 	256
#define BUFFER_LENGTH 	128
// Arduino의 메모리 부족으로 128으로 줄임, Sync All에서 102를 사용한다.
// 이상 여부는 확인 필요(TBD)
#define MAX_CMD_BUFFER  40  // Command Line Buffer

// #define TX_QUEUE_LENGTH	16
#define TX_QUEUE_LENGTH	8
// Arduino의 메모리 부족으로 8로 줄임
// 이상 여부는 확인 필요(TBD)

#ifndef FALSE
	#define FALSE 0
#endif
#ifndef TRUE
	#define TRUE 1
#endif

/*
// MCR의 초기값이 0x08이고 DTR을 사용하지 않으므로 아래와 같다.
// RTS 신호를 SET -> 송신모드
#define SET_RTS		outportb(g_base_addr+MCR_OFFSET,0x0A);
// RTS 신호를 RESET -> 수신모드
#define RESET_RTS	while(!(inportb(g_base_addr+LSR_OFFSET)&0x40)); outportb(g_base_addr+MCR_OFFSET,0x08);
*/
// Arduino에서는 AX-12의 통신 특성상 Data 송신 시 SET을 해야 한다.
#define RTS_PIN 13  // (TBD)
#define SET_RTS digitalWrite(RTS_PIN, HIGH);  // 데이터를 보내기 전 RST를 HIGH로 한다.
// #define RESET_RTS while(Serial2.availableForWrite()); digitalWrite(RTS_PIN, LOW); // 데이터를 다 보낸 후 RST를 Low로 한다.
#define RESET_RTS Serial2.flush(); digitalWrite(RTS_PIN, LOW); // (TBC) 데이터를 다 보낸 후 RST를 Low로 한다.

//*****************************************************************
//    매크로 선언
//*****************************************************************

// ----------------------
// 수신 Queue
// ----------------------
// 읽을 데이터가 있는지 확인, First와 last가 같지 않으면 읽을 데이터가 있다.
// g_Qfirst : 지금 데이터를 읽어야 할 지점을 가리킴. 
// g_Qlast : 다음 입력할 버퍼의위치를 가리킴. 
// Queue가 가득차서 overflow되는것은 Buffer에 쓸때(인터럽트에서) 방지해야 한다.
#define IsReadyToRX	(g_Qfirst!=g_Qlast)

// 다음 읽을 위치로 이동.
// #define NEXT_FIRST  {g_Qfirst++; if (g_Qfirst==BUFFER_LENGTH)	g_Qfirst = 0;}
// Queue의 크기가 256개 이므로 그냥 하나 증가시키면 알아서 처리된다. 
#define NEXT_FIRST  g_Qfirst++; 

// 다음 쓸 위치로 이동.
// #define NEXT_LAST   {g_Qlast++;  if (g_Qlast ==BUFFER_LENGTH)	g_Qlast  = 0;}
// Queue의 크기가 256개 이므로 그냥 하나 증가시키면 알아서 처리된다. 
#define NEXT_LAST   g_Qlast++;


// ----------------------
// 송신 Packet Queue
// ----------------------
// 읽을 데이터가 있는지 확인, First와 last가 같지 않으면 읽을 데이터가 있다.
// g_Tx_Qfirst : 지금 데이터를 읽어야 할 지점을 가리킴. 
// g_Tx_Qlast : 다음 입력할 버퍼의위치를 가리킴. 
// Queue가 가득차서 overflow되는것은 Buffer에 쓸때(인터럽트에서) 방지해야 한다.
#define IsReadyToTX		(g_Tx_Qfirst!=g_Tx_Qlast)
// Queue가 가득 찻는지 확인.
#define IsTxQueueFull 	(((g_Tx_Qlast+1)&0x0F) == (g_Tx_Qfirst))
// Queue가 비어있는지 확인.
#define IsTxQueueEmpty 	(g_Tx_Qfirst == g_Tx_Qlast)

// 다음 읽을 위치로 이동.
// Queue의 크기가 16개 이므로 그냥 하나 증가시키고 하위 Nibble을 취하면 알아서 처리된다. 
#define NEXT_TX_FIRST  {g_Tx_Qfirst = (g_Tx_Qfirst+1)&0x0F;}

// 다음 쓸 위치로 이동.
// Queue의 크기가 16개 이므로 그냥 하나 증가시키고 하위 Nibble을 취하면 알아서 처리된다. 
#define NEXT_TX_LAST   {g_Tx_Qlast = (g_Tx_Qlast+1)&0x0F;}


//******************************************************/
//******************************************************/
//******************************************************/
// Main.h
//******************************************************/
//******************************************************/
//******************************************************/
///////////////////////////////////////////////
//--- AX-12 ---
//--- Control Table Address ---
///////////////////////////////////////////////
//EEPROM AREA
#define P_MODEL_NUMBER_L 			0
#define P_MODOEL_NUMBER_H 			1
#define P_VERSION 					2
#define P_ID 						3
#define P_BAUD_RATE 				4
#define P_RETURN_DELAY_TIME 		5
#define P_CW_ANGLE_LIMIT_L 			6
#define P_CW_ANGLE_LIMIT_H 			7
#define P_CCW_ANGLE_LIMIT_L 		8
#define P_CCW_ANGLE_LIMIT_H 		9
#define P_SYSTEM_DATA2 				10
#define P_LIMIT_TEMPERATURE 		11
#define P_DOWN_LIMIT_VOLTAGE 		12
#define P_UP_LIMIT_VOLTAGE 			13
#define P_MAX_TORQUE_L 				14
#define P_MAX_TORQUE_H 				15
#define P_RETURN_LEVEL 				16
#define P_ALARM_LED 				17
#define P_ALARM_SHUTDOWN 			18
#define P_OPERATING_MODE 			19
#define P_DOWN_CALIBRATION_L 		20
#define P_DOWN_CALIBRATION_H 		21
#define P_UP_CALIBRATION_L 			22
#define P_UP_CALIBRATION_H 			23

#define P_TORQUE_ENABLE 			(24)
#define P_LED 						(25)
#define P_CW_COMPLIANCE_MARGIN 		(26)
#define P_CCW_COMPLIANCE_MARGIN		(27)
#define P_CW_COMPLIANCE_SLOPE 		(28)
#define P_CCW_COMPLIANCE_SLOPE 		(29)
#define P_GOAL_POSITION_L 			(30)
#define P_GOAL_POSITION_H 			(31)
#define P_GOAL_SPEED_L 				(32)
#define P_GOAL_SPEED_H 				(33)
#define P_TORQUE_LIMIT_L 			(34)
#define P_TORQUE_LIMIT_H 			(35)
#define P_PRESENT_POSITION_L 		(36)
#define P_PRESENT_POSITION_H 		(37)
#define P_PRESENT_SPEED_L 			(38)
#define P_PRESENT_SPEED_H 			(39)
#define P_PRESENT_LOAD_L 			(40)
#define P_PRESENT_LOAD_H 			(41)
#define P_PRESENT_VOLTAGE 			(42)
#define P_PRESENT_TEMPERATURE 		(43)
#define P_REGISTERED_INSTRUCTION	(44)
#define P_PAUSE_TIME 				(45)
#define P_MOVING 					(46)
#define P_LOCK 						(47)
#define P_PUNCH_L 					(48)
#define P_PUNCH_H 					(49)

// AX-12 Instruction
#define INST_PING					0x01
#define INST_READ 					0x02
#define INST_WRITE 					0x03
#define INST_REG_WRITE 				0x04
#define INST_ACTION 				0x05
#define INST_RESET 					0x06
#define INST_DIGITAL_RESET 			0x07
#define INST_SYSTEM_READ 			0x0C
#define INST_SYSTEM_WRITE 			0x0D
#define INST_SYNC_WRITE 			0x83
#define INST_SYNC_REG_WRITE 		0x84

//////////////////////////////////////////
// 통신 Port 선언 
//////////////////////////////////////////
#define PORT	COM1

#define MAX_ID 	19
#define BROADCASTING_ID	0xFE


