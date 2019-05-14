//*****************************************************************
//    �Լ�����(com_test�ϱ� 6_serial���� ���� ����, ���� �Ժη� �ٲ��� ����)
//*****************************************************************
// ���� ���ǵǴ� interrrupt handdler
// ��� ��Ʈ�� �����Ͱ� ������ ���ͷ�Ʈ�� �ɸ���. 
// void interrupt port_interrupt(...); // for DOS
// for Arduino, Interrupt ��� ���, Arduino Mega�� ��������� serialEvent2()���� ��ġ��.
// void port_interrupt(void);              
// ������ ���ͷ�Ʈ ���͸� �����ϱ� ���� �Լ�
// void interrupt (*old_intr)(...);

// ���� ��Ʈ �ʱ�ȭ �� ���ͷ�Ʈ ����. 
void Serial_Port_Init(void);  // Arduino��

// ��Ʈ�� ������ ���Ѵ�.
// �����͸� �����ϴµ� ����� ���ͷ�Ʈ ���͸� ������ ������,
// ������ ���� ���ͷ�Ʈ�� �ɸ��� �ʵ��� �Ѵ�.  
void Close_Com(void);

// ���� ���۷κ��� 1byte�� �����͸� ����. 
unsigned char ReadChar(void);

// Hex 1byte �����͸� ȭ������ ���. 
void PrintHex(unsigned char bHex);

// Hex String�� ȭ������ ���. 
void PrintHexStr(unsigned char* bpBuffer, unsigned char bCount);

// Hex String�� Serial Port�� ����
void SendHexStr(unsigned char* bpBuffer, int bCount);

// ���� LEG�� Serial Data ����
void Sync_Write_LEG(int position_ax[2][6],int speed_ax[2][6]);
void Sync_Write_BODY(int position_ax[2][4],int speed_ax[2][4]);
void Sync_Write_ALL(int position_ax_LEG[2][6],int speed_ax_LEG[2][6],
					int position_ax_ARM[2][4],int speed_ax_ARM[2][4] );
// Serial Data�� checksum ���.
unsigned char CalCheckSum(unsigned char* TxData, int Count);

// Packet�� TX Queue�� ����.
// TX Queue�� Timer Interrupt���� ������.
void SendHexStr_to_TxQueue(unsigned char* bpBuffer);

int RX_data_check(unsigned char* pRxData);

void Dump_RX_data(void);

void WriteHex(int ID, unsigned char Addr, unsigned char * pTable, int Length);

char getSerialChar(void);

