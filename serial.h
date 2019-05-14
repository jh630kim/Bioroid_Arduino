//*****************************************************************
//    함수선언(com_test하구 6_serial에서 같이 쓴다, 형을 함부로 바꾸지 말자)
//*****************************************************************
// 새로 정의되는 interrrupt handdler
// 통신 포트에 데이터가 들어오면 인터럽트가 걸린다. 
// void interrupt port_interrupt(...); // for DOS
// for Arduino, Interrupt 대신 사용, Arduino Mega를 사용함으로 serialEvent2()으로 대치함.
// void port_interrupt(void);              
// 기존의 인터럽트 벡터를 저장하기 위한 함수
// void interrupt (*old_intr)(...);

// 직렬 포트 초기화 및 인터럽트 설정. 
void Serial_Port_Init(void);  // Arduino용

// 포트를 닫지는 못한다.
// 데이터를 수신하는데 사용한 인터럽트 벡터를 원래로 돌리고,
// 데이터 수신 인터럽트가 걸리지 않도록 한다.  
void Close_Com(void);

// 수신 버퍼로부터 1byte의 데이터를 읽음. 
unsigned char ReadChar(void);

// Hex 1byte 데이터를 화면으로 출력. 
void PrintHex(unsigned char bHex);

// Hex String을 화면으로 출력. 
void PrintHexStr(unsigned char* bpBuffer, unsigned char bCount);

// Hex String을 Serial Port로 전송
void SendHexStr(unsigned char* bpBuffer, int bCount);

// 양쪽 LEG의 Serial Data 전송
void Sync_Write_LEG(int position_ax[2][6],int speed_ax[2][6]);
void Sync_Write_BODY(int position_ax[2][4],int speed_ax[2][4]);
void Sync_Write_ALL(int position_ax_LEG[2][6],int speed_ax_LEG[2][6],
					int position_ax_ARM[2][4],int speed_ax_ARM[2][4] );
// Serial Data의 checksum 계산.
unsigned char CalCheckSum(unsigned char* TxData, int Count);

// Packet을 TX Queue로 전송.
// TX Queue는 Timer Interrupt에서 보낸다.
void SendHexStr_to_TxQueue(unsigned char* bpBuffer);

int RX_data_check(unsigned char* pRxData);

void Dump_RX_data(void);

void WriteHex(int ID, unsigned char Addr, unsigned char * pTable, int Length);

char getSerialChar(void);

