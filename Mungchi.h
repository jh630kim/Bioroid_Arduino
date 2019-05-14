////////////////////////////////////////////////////////////////////
// �ۼ��� : 051231
// ��  �� : Ÿ�̸� ���ͷ�Ʈ
// ��  �� : A_MOVE_1.cpp�� ������.
////////////////////////////////////////////////////////////////////

// Arduino H/W for Arduino Mega 
// �������� : 5V
// �������� : 7 ~ 12V
// ������� : 6 ~ 20V
// Ŭ���ӵ� : 16MHz
// Serial  : 0(RX),1(TX)    - for PC
// Serial1 : 19(RX),18(TX)  - for BT
// Serial2 : 17(RX),16(TX)  - for AX-12
// Serial3 : 15(RX),14(TX)
// SPI��� : 50(MISO), 51(MOSI), 52(SCK), 53(SS)
// I2C��� : 20(SDA), 21(SCL)
// ����LED : 13

// HC-06 ������� ��� ����
// RXD <-> 18(TX, Serial 1)
// TXD <-> 19(RX, Serial 1)
// GNC <-> GND
// VCC <-> 5V (�Ǵ� 3.3V, ��� ���� Ȯ�� �ʿ�)
// ��� �̸� : HC-06
// PIN : 1234

// GY-61 �Ƴ��α� ���ӵ� ����
// ���� : �����̰� �ִ� ������ ����� ���ӵ��� ���е��� �ʴ´�?!
//        ���̷� ���� Ȱ�� �ʿ�!!!
// VCC - 5V
// X-OUT - A0
// Y-OUT - A1
// Z-OUT - A2
// GND - GND

// ǥ�� �Լ� ���̺귯�� (DOS���� �ִ���)
// �� ������ �������� �ǳ�???
#include <stdio.h>
#include <stdarg.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
// #include <conio.h>   // arduino error
// #include <dos.h>     // arduino error

// Arduino�� Library
// �޸� �뷮������ Arduino Uno�� ������� ���Ѵ�.
// Arduino Mega�� Serial Port�� 4����.
// Serial -> USB, PC Monitor��
// Serial1 -> Bluetooth
// Serial2 -> AX-12 ���Ϳ� �������.

// #include <AltSoftSerial.h>  // Arduino�� Software Serial
// #include <SoftwareSerial.h> // Arduino�� Software Serial
/* 3�� Serial ����� �Բ� ���
https://www.pjrc.com/teensy/td_libs_AltSoftSerial.html
Hardware Serial : Softserial���ٴ� ������ �Ѵ�.          (9600)
  -> Serial Monitor for PC
AltSoftSerial : SoftSerial���� 10��� ������ �Ѵ�.       (9600)
  -> BT for Smart Phone
SoftSerial : �ٸ� �ͺ��� ������ ���� Baud Rate���� �Ѵ�. (115200)
  -> AX-12 ����
*/
#include <MsTimer2.h> // Timer Interrrupt
#include <SPI.h>      // for SD Card Interface
#include <SD.h>       // for SD Card

// ���� ����
#include "define.h"
// ��� �Լ� ����
#include "main.h"
#include "kinhead.h"
#include "serial.h"
#include "timer.h"
#include "utility.h"
#include "debug.h"

//////////////////////////////////////////////////
// 1_Main.ino
//////////////////////////////////////////////////
int g_t1x_diff, g_t2x_diff;
int g_t1y_diff, g_t2y_diff;
int g_t1z_diff, g_t2z_diff;
int g_TE_diff;

int	g_Emer_Stop;

unsigned char dbg_jhk1[] = {255,255,254,64,131,30, 4, 8,255, 1,10, 0,10,42, 2,50, 0,12,169, 2,50, 0,
                  14,195, 1,50, 0,16,178, 1,10, 0,18,42, 2,50, 0, 9,255, 1,10, 0,11,62, 2,
                  50, 0,13,154, 1,50, 0,15,63, 2,100, 0,17,141, 2,50, 0,19,62, 2,50, 0, 3};

unsigned char dbg_jhk2[] = {255,255,254,64,131,30, 4, 8,255, 1,10, 0,10,209, 1,50, 0,12,92, 2,10, 0,
					14,203, 1,50, 0,16,93, 1,50, 0,18,209, 1,50, 0, 9,255, 1,10, 0,11,229, 1,50, 0,
					13,73, 1,50, 0,15,62, 2,10, 0,17,60, 2,50, 0,19,229, 1,50, 0, 2};

double g_z_RIGHT_BASE, g_z_LEFT_BASE;
char MonCmd[MAX_CMD_BUFFER]; // = NULL;	// ��� ���� ����

//////////////////////////////////////////////////
// 2_Kin_M.ino
//////////////////////////////////////////////////
///////////// ���� ���� //////////////////////////
// �κ��� ���� ��ġ�� ���� 
int g_LEG_Position[2][6];	// AX-12�� ��ġ�� ����.
int g_ARM_Position[2][4];	// AX-12�� ��ġ�� ����.

// �� ������ �����߽��� ��ġ
const double P_C_MC[3] = { -5,-37,  0};
const double P_3_M3[3] = { 16, 17,  0};
const double P_4_M4[3] = { 14,-14,  0};
const double P_6_M6[3] = {-16,  0,-17};

const double P_AR1_MAR1[3] = { 60,-27, 76.2};
// const double P_AL1_MAL1[3] = { 60,-27,-76.2};
const double P_A3_MA3[3]   = { 24,  0,  0.0};
// 1mm���̴�. �׳�... 0���� ����. 
// const double P_AR4_MAR4[3]   = { 26, -1,  0.0};
// const double P_AL4_MAL4[3]   = { 26,  1,  0.0};
const double P_A4_MA4[3]   = { 26,  0,  0.0};

// �ٸ��� ���� -> AX-12��ġ ��ȯ�� -> ��� �̴�!!!
const double LEG_BASE[][6]   = {{   0,   0, 360,   0,   0, 360}, {   0,   0,   0, 360, 360, 360}};
const double LEG_SIGN[][6]   = {{   1,   1,  -1,   1,   1,  -1}, {   1,   1,   1,  -1,  -1,  -1}};
const double LEG_DtoP[][6]   = {{ 180, 180, 169.2417, 111.5165, 169.2417, 180}, 
                                { 180, 180, 169.2417, 111.5165, 169.2417, 180}};

const double ARM_BASE[][4]   = {{   0, 360, 360, 360}, {   0,   0, 360, 360}};
const double ARM_SIGN[][4]   = {{   1,  -1,  -1,  -1}, {   1,   1,  -1,  -1}};
const double ARM_DtoP[][4]   = {{  96.254, 173.746, 270, 180}, 
                                {  96.254, 173.746,  90, 180}};

/*
//const double LEG_MIN_DEG[][6]    = {{ -30, -40,-110, -20, -80, -40}, { -30, -40,-110, -20, -80, -40}};
//const double LEG_MAX_DEG[][6]    = {{  30,  40,  30, 120,  30,  40}, {  30,  40,  30, 120,  30,  40}};
const double LEG_MIN_RAD[][6] = {{-0.5236,-0.6981,-1.9199,-0.3491,-1.3963,-0.6981},
								 {-0.5236,-0.6981,-1.9199,-0.3491,-1.3963,-0.6981}};
const double LEG_MAX_RAD[][6] = {{ 0.5236, 0.6981, 0.5236, 2.0944, 0.5236, 0.6981},
								 { 0.5236, 0.6981, 0.5236, 2.0944, 0.5236, 0.6981}};
*/
// ���� ���� ����. 
const double LEG_MIN_RAD[][6] = {{-100,-100,-100,-100,-100,-100},
								 {-100,-100,-100,-100,-100,-100}};
const double LEG_MAX_RAD[][6] = {{ 100, 100, 100, 100, 100, 100},
								 { 100, 100, 100, 100, 100, 100}};
								 
const double LEG_WEIGHT[]  = {G_C_M6, G_C_M4, G_C_M3, G_C_MC, G_C_M3, G_C_M4, G_C_M6};
const double BODY_WEIGHT[] = {G_C_MA1,G_C_MA3,G_C_MA4,G_C_MA3,G_C_MA4};
const double ALL_WEIGHT[]  = {G_C_M6, G_C_M4, G_C_M3, G_C_MC, G_C_M3, G_C_M4, G_C_M6,
							  G_C_MA1,G_C_MA3,G_C_MA4,G_C_MA3,G_C_MA4};

//////////// ���� ��ȯ ��Ʈ���� ///////////////////
// �ٸ�
// T_C_R0 = T_C_0[RIGHT_LEG]
// T_C_L0 = T_C_0[LEFT_LEG]
const double T_C_0[2][3][4]  = {{{ 0, 0,-1,-50.0},
				 				 { 0, 1, 0,-14.5},
						     	 { 1, 0, 0,-32.7}},
							 	{{ 0, 0,-1,-50.0},
						     	 { 0, 1, 0,-14.5},
						     	 { 1, 0, 0, 32.7}}};

const double T_R0_C[3][4]  = {{ 0, 0, 1, 32.7},
				 		      { 0, 1, 0, 14.5},
						      {-1, 0, 0,-50.0}};
const double T_L0_C[3][4]  = {{ 0, 0, 1,-32.7},
						      { 0, 1, 0, 14.5},
						      {-1, 0, 0,-50.0}};

// T_R6_F = T_L6_F
const double T_R6_F[3][4] = {{ 1, 0, 0, 32.2},
						     { 0, 1, 0,  0.0},
						     { 0, 0, 1,  0.0}};

// T_F_R6 = T_F_L6
const double T_F_R6[3][4] = {{ 1, 0, 0,-32.2},	
						     { 0, 1, 0,  0.0},
						     { 0, 0, 1,  0.0}};

// �߽ɿ��� �߱����� ȸ�� ��Ʈ����
const double R_C_F[][3]  = {{-1, 0, 0},
						    { 0, 0, 1},
						    { 0, 1, 0}};
// ��ü
// T_R6_F = T_L6_F
const double T_BC_R0[3][4] = {{ 1, 0, 0,  0.0},
						      { 0, 1, 0,  0.0},
						      { 0, 0, 1,-76.2}};
const double T_BC_L0[3][4] = {{ 1, 0, 0,  0.0},
						      { 0, 1, 0,  0.0},
						      { 0, 0, 1, 76.2}};
						      
//////////////////////////////////////////////////
// 3_SERIAL.ino
//////////////////////////////////////////////////
// Com Port�� base address����. 
int g_base_addr;
int g_port_int_no;
int g_imr_on;
int g_imr_off;

// ������ ������ ���� ���ۿ� ���� ���� ����, 
// ���� ����, 
// main������ g_Qfirst != g_Qlast�̸� Read_From_Port�� ȣ���Ѵ�.  
unsigned char buffer_queue[BUFFER_LENGTH];
unsigned char g_Qfirst = 0;		// g_Qfirst : ���� �����͸� �о�� �� ������ ����Ŵ. 
unsigned char g_Qlast = 0;		// g_Qlast : ���� �Է��� ��������ġ�� ����Ŵ. 

// 10msec���� �����͸� �����ϱ� ���� ť
// �۽� Packet ����, 
// Interrupt���� g_Tx_Qfirst != g_Tx_Qlast�̸� Read_From_Port�� ȣ���Ѵ�.  
// Timer ���ͷ�Ʈ���� ������. 
unsigned char g_Tx_Packet[TX_QUEUE_LENGTH][BUFFER_LENGTH];
volatile unsigned char g_Tx_Qfirst = 0;		// g_Qfirst : ���� �����͸� �о�� �� ������ ����Ŵ. 
volatile unsigned char g_Tx_Qlast = 0;		// g_Qlast : ���� �Է��� ��������ġ�� ����Ŵ. 

// ��/���� �ٸ��� AX-12 ���� ID
const int    LEG_ID[][6]     = {{   8,  10,  12,  14,  16,  18}, {   9,  11,  13,  15,  17,  19}};
const int 	 WAIST_ID		 = 1;
const int    ARM_ID[][4]     = {{   1,   2,   4,   6}, {   1,  3,  5,  7}};


#define CNT_RX  2
#define CNT_TX  3
// AltSoftSerial   BTSerial;                 // RX-8, TX-9, Unusable-10pin
#define BTSerial  Serial1                 // RX-8, TX-9, Unusable-10pin
// SoftwareSerial  AXSerial(CNT_RX,CNT_TX);  // RX-2, TX-3
#define AXSerial  Serial2


//////////////////////////////////////////////////
// 4_TIMER.ino
//////////////////////////////////////////////////
unsigned long g_time_period;
unsigned long g_prev_time;
unsigned long g_curr_time;

unsigned int	g_Send_Flag;
// static INT16U    PC_ElapsedOverhead;


//////////////////////////////////////////////////
// 5_Utility.ino
//////////////////////////////////////////////////
// Set up variables using th SD utility library funcions:
Sd2Card card;
SdVolume volume;
SDFile root;

// change this to match your SD shield or module:
// Arduino Ethernet shield : pin 4
#define SD_CS 4


