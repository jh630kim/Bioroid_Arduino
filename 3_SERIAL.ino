////////////////////////////////////////////////////////////////////
// �ۼ��� : 051126
// ��  �� : Robot�� AVR�� ����� �ϴ� ���α׷�.
// ��  �� : COM2, ?????BPS, 8 data bit, 1 stop bit, no parity�� ������. 
//          �� PC�� COM1�� ���峵��. 
////////////////////////////////////////////////////////////////////

//*****************************************************************
//   �Լ�. 
//*****************************************************************
// ���� ���ǵǴ� interrrupt handdler
// ��� ��Ʈ�� �����Ͱ� ������ ���ͷ�Ʈ�� �ɸ���. (for DOS)
/*
void interrupt port_interrupt(...)
{
	// Queue�� ���� ������ ���...
	// �ʿ��ұ�?
	// 1000�� ���۰� �� ������ ó���� ���� �ʴ´ٴ� �� ���� ������ �ִ�.
	// �׳� �Ѿ��.

	// ��Ʈ���� �����͸� �о�� ����
	buffer_queue[g_Qlast] = inportb(g_base_addr);
	
	// g_Qlast : ���� �Է��� ��������ġ�� ����Ŵ. 
	NEXT_LAST;
	
	// PIC�� EIO ���ͷ�Ʈ�� ���ͷ�Ʈ�� �����ٴ� ��ȣ ������
	// outportb(PICEIO,0x20);
}
*/

// Arduino�� Software Serial�� Interrupt�� ����.
// (Hardware Serial�� serialEvent()��� Interrupt�� �ִ�)
// �׷���, ���� Data�� RX_data_check()�� ReadChar()������ ����Ѵ�.
// �� �Լ� �ȿ��� ���ͷ�Ʈ ��� �������
// �� ����Ʈ�� �����͸� �о buffer_queue�� �ִ´�.
// AX-12�� �����͸� ��û�� �� feedback�� �ϴµ�,
// ������ ��û �� 10ms delay�� �ش�.
/*
void port_interrupt(void) // // AXSerial(Serial2)�� ���� Interrupt ��� ���
{
	if (AXSerial.available() > 0)
	{
	  // ��Ʈ���� �����͸� �о�� ����
	  buffer_queue[g_Qlast] = AXSerial.read();
	  // g_Qlast : ���� �Է��� ��������ġ�� ����Ŵ. 
	  NEXT_LAST;
	}
}
*/
void serialEvent2(void)  // AXSerial(Serial2)�� ���� Interrupt
{
	while (AXSerial.available() > 0)
	{
	  // ��Ʈ���� �����͸� �о�� ����
	  buffer_queue[g_Qlast] = AXSerial.read();
	  // g_Qlast : ���� �Է��� ��������ġ�� ����Ŵ. 
	  NEXT_LAST;
	}
}

// ���� ��Ʈ �ʱ�ȭ �� ���ͷ�Ʈ ����. 
// ���ͷ�Ʈ ������ outport�� ���� �����ؾ� �Ѵ�.
void Serial_Port_Init(void)
{
  // Serial SetUp
  // SoftwareSerial�� Port
  // pinMode(CNT_RX, INPUT);
  // pinMode(CNT_TX, OUTPUT);
  pinMode(RTS_PIN, OUTPUT);  // RST PIN�� Output���� ����
  Serial.begin(9600);       // PC�� USB�� Serial Port
  BTSerial.begin(9600);     // BT��
  AXSerial.begin(115200);   // ���������
}
	 
void Close_Com(void)
{
  Serial.end();       // PC�� USB�� Serial Port
  BTSerial.end();     // BT��
  AXSerial.end();   // ���������
}

// ���� ���۷κ��� 1byte�� �����͸� ����. 
unsigned char ReadChar(void)
{
	unsigned char result;
	 
  // port_interrupt(); // �� ����Ʈ�� �����͸� �о buffer_queue�� �ִ´�.

	// ������ �б�
	result = buffer_queue[g_Qfirst];
	
	// ���� �����͸� ���� ��ġ�� �̵�
	NEXT_FIRST;

	return result;
}

// Hex 1byte �����͸� ȭ������ ���. 
void PrintHex(unsigned char bHex)
{
	unsigned char bpStr[2];
	
	// ù��° ����Ʈ 
	bpStr[0] =((unsigned char)(bHex>>4)&0x0f) + (unsigned char)'0';
	if(bpStr[0] > '9') bpStr[0] += 7;
	
	// �ι�° ����Ʈ
	bpStr[1] =(unsigned char)(bHex & 0x0f) + (unsigned char)'0';
	if(bpStr[1] > '9') bpStr[1] += 7;
	
	// ȭ�� ���
	_printF(F("%c%c "),bpStr[0], bpStr[1]);
	// Serial.print(bHex,HEX);
}

// Hex String�� ȭ������ ���. 
void PrintHexStr(unsigned char* bpBuffer, unsigned char bCount)
{
	int i;
	for (i=0;i<bCount;i++)
	{
		PrintHex(bpBuffer[i]);
	}
}

// Hex String�� Serial Port�� ����
void SendHexStr(unsigned char* bpBuffer, int bCount)
{
	int i;
	SET_RTS;  // (TBC) ù��° �����Ͱ� ������ �� ������ SendHexStr�� ȣ���ϴ� ������ �ű���.
	for(i=0;i<=bCount;i++)
	{
		AXSerial.write(bpBuffer[i]);  // Arduino (TBC, Ȯ�� �ʿ�) Write ���� �ٸ� ����?
	}
	RESET_RTS;
}

// Hex String�� TxQueue�� ����
// Tx Queue������ Timer Interrupt���� Packet�� �����Ѵ�. 
void SendHexStr_to_TxQueue(unsigned char* bpBuffer)
{
	int total_length;
	char ch;

	// Queue�� ���� �� �ִµ��� ���
	// Timer Interrupt�� ������ �ʴ� �� ���ѷ����� ���� ������ ����. 
	do
	{	
		// if (kbhit())
		if (Serial1.available()>0)		
		{
			ch = (char)Serial.read();  			// ch = getch();
			switch(ch)
			{
				case 's' : g_Send_Flag ^= 1; 	break;
				case 'c' : g_Emer_Stop = 1;		break;
			}			
		}
		delay(50);
	}while(IsTxQueueFull);
	
	total_length = min(bpBuffer[3]+4,BUFFER_LENGTH);
	for (int i=0;i<total_length;i++)
	{
		g_Tx_Packet[g_Tx_Qlast][i] = bpBuffer[i];
	}
	
	NEXT_TX_LAST;
}

void Sync_Write_ALL(int position_ax_LEG[2][6],int speed_ax_LEG[2][6],
					int position_ax_ARM[2][4],int speed_ax_ARM[2][4] )
{	
	int i;
	unsigned char TxData[BUFFER_LENGTH];
		
	TxData[0] = TxData[1] = 0xFF;
	TxData[2] = BROADCASTING_ID;
	TxData[3] = 99;			// Length
	TxData[4] = INST_SYNC_WRITE;
	TxData[5] = P_GOAL_POSITION_L;
	TxData[6] = 4;
	for (i=0;i<6;i++)
	{
		TxData[7+i*5] = LEG_ID[RIGHT_LEG][i];
		TxData[8+i*5] = position_ax_LEG[RIGHT_LEG][i] & 0xFF;
		TxData[9+i*5] = (position_ax_LEG[RIGHT_LEG][i]>>8) & 0xFF;
		TxData[10+i*5] = speed_ax_LEG[RIGHT_LEG][i] & 0xFF;
		TxData[11+i*5] = (speed_ax_LEG[RIGHT_LEG][i]>>8) & 0xFF;
	}
	for (i=0;i<6;i++)
	{
		TxData[37+i*5] = LEG_ID[LEFT_LEG][i];
		TxData[38+i*5] = position_ax_LEG[LEFT_LEG][i] & 0xFF;
		TxData[39+i*5] = (position_ax_LEG[LEFT_LEG][i]>>8) & 0xFF;
		TxData[40+i*5] = speed_ax_LEG[LEFT_LEG][i] & 0xFF;
		TxData[41+i*5] = (speed_ax_LEG[LEFT_LEG][i]>>8) & 0xFF;
	}
	
	// �㸮
	TxData[67] = WAIST_ID;
	TxData[68] = position_ax_ARM[RIGHT][0] & 0xFF;
	TxData[69] = (position_ax_ARM[RIGHT][0]>>8) & 0xFF;
	TxData[70] = speed_ax_ARM[RIGHT][0] & 0xFF;
	TxData[71] = (speed_ax_ARM[RIGHT][0]>>8) & 0xFF;
	
	// ������
	for (i=1;i<4;i++)
	{
		TxData[67+i*5] = ARM_ID[RIGHT][i];
		TxData[68+i*5] = position_ax_ARM[RIGHT][i] & 0xFF;
		TxData[69+i*5] = (position_ax_ARM[RIGHT][i]>>8) & 0xFF;
		TxData[70+i*5] = speed_ax_ARM[RIGHT][i] & 0xFF;
		TxData[71+i*5] = (speed_ax_ARM[RIGHT][i]>>8) & 0xFF;
	}
	// ����
	for (i=1;i<4;i++)
	{
		TxData[82+i*5] = ARM_ID[LEFT][i];
		TxData[83+i*5] = position_ax_ARM[LEFT][i] & 0xFF;
		TxData[84+i*5] = (position_ax_ARM[LEFT][i]>>8) & 0xFF;
		TxData[85+i*5] = speed_ax_ARM[LEFT][i] & 0xFF;
		TxData[86+i*5] = (speed_ax_ARM[LEFT][i]>>8) & 0xFF;
	}	
	
	TxData[102] = CalCheckSum(TxData,102);
	
	// Data �۽�.
	// SendHexStr(TxData, 67);		// 0 ~ i���� ������ �۽�
	SendHexStr_to_TxQueue(TxData);		// 0 ~ i���� ������ �۽�
}

void Sync_Write_LEG(int position_ax[2][6],int speed_ax[2][6])
{	
	int i;
	unsigned char TxData[BUFFER_LENGTH];
		
	TxData[0] = TxData[1] = 0xFF;
	TxData[2] = BROADCASTING_ID;
	TxData[3] = 64;			// Length
	TxData[4] = INST_SYNC_WRITE;
	TxData[5] = P_GOAL_POSITION_L;
	TxData[6] = 4;
	for (i=0;i<6;i++)
	{
		TxData[7+i*5] = LEG_ID[RIGHT_LEG][i];
		TxData[8+i*5] = position_ax[RIGHT_LEG][i] & 0xFF;
		TxData[9+i*5] = (position_ax[RIGHT_LEG][i]>>8) & 0xFF;
		TxData[10+i*5] = speed_ax[RIGHT_LEG][i] & 0xFF;
		TxData[11+i*5] = (speed_ax[RIGHT_LEG][i]>>8) & 0xFF;
	}
	for (i=0;i<6;i++)
	{
		TxData[37+i*5] = LEG_ID[LEFT_LEG][i];
		TxData[38+i*5] = position_ax[LEFT_LEG][i] & 0xFF;
		TxData[39+i*5] = (position_ax[LEFT_LEG][i]>>8) & 0xFF;
		TxData[40+i*5] = speed_ax[LEFT_LEG][i] & 0xFF;
		TxData[41+i*5] = (speed_ax[LEFT_LEG][i]>>8) & 0xFF;
	}
	TxData[67] = CalCheckSum(TxData,67);
	// TxData[67] = 100;
	
	// Data �۽�.
	// SendHexStr(TxData, 67);		// 0 ~ i���� ������ �۽�
	SendHexStr_to_TxQueue(TxData);		// 0 ~ i���� ������ �۽�
}

void Sync_Write_BODY(int position_ax[2][4],int speed_ax[2][4])
{
	int i;
	unsigned char TxData[BUFFER_LENGTH];
	
	TxData[0] = TxData[1] = 0xFF;
	TxData[2] = BROADCASTING_ID;
	TxData[3] = 39;			// Length
	TxData[4] = INST_SYNC_WRITE;
	TxData[5] = P_GOAL_POSITION_L;
	TxData[6] = 4;
	
	// �㸮
	TxData[ 7] = WAIST_ID;
	TxData[ 8] = position_ax[RIGHT][0] & 0xFF;
	TxData[ 9] = (position_ax[RIGHT][0]>>8) & 0xFF;
	TxData[10] = speed_ax[RIGHT][0] & 0xFF;
	TxData[11] = (speed_ax[RIGHT][0]>>8) & 0xFF;
	
	// ������
	for (i=1;i<4;i++)
	{
		TxData[ 7+i*5] = ARM_ID[RIGHT][i];
		TxData[ 8+i*5] = position_ax[RIGHT][i] & 0xFF;
		TxData[ 9+i*5] = (position_ax[RIGHT][i]>>8) & 0xFF;
		TxData[10+i*5] = speed_ax[RIGHT][i] & 0xFF;
		TxData[11+i*5] = (speed_ax[RIGHT][i]>>8) & 0xFF;
	}
	// ����
	for (i=1;i<4;i++)
	{
		TxData[22+i*5] = ARM_ID[LEFT][i];
		TxData[23+i*5] = position_ax[LEFT][i] & 0xFF;
		TxData[24+i*5] = (position_ax[LEFT][i]>>8) & 0xFF;
		TxData[25+i*5] = speed_ax[LEFT][i] & 0xFF;
		TxData[26+i*5] = (speed_ax[LEFT][i]>>8) & 0xFF;
	}	
	TxData[42] = CalCheckSum(TxData,42);
	
	// Data �۽�.
	// SendHexStr(TxData, 67);		// 0 ~ i���� ������ �۽�
	SendHexStr_to_TxQueue(TxData);		// 0 ~ i���� ������ �۽�
}

// TxData�� Packet�� Checksum�� ���.
// Count�� Checksum�� ��ġ��.
unsigned char CalCheckSum(unsigned char* TxData, int Count)
{
	unsigned char CheckSum;
	int j;
		
	// Check Sum�� ���
	CheckSum = 0;
	for(j=2;j<Count;j++)
	{
		CheckSum = CheckSum + TxData[j];
	}
	return ~CheckSum;
}

// return  : ���� ������ ���� (Error�� ��� 0�� Return��)
// pRxData : ���� ������
int RX_data_check(unsigned char* pRxData)
{
	int i, Rx_Len, Packet_Length, Result;
	unsigned char CheckSum;
	
	Result = 0;			// Data ����.
	pRxData[2] = 0;		// ID�� 0���� ����.(������� �ʴ� ID)
	/*
	do  	// �����Ͱ� �ִ� ���� ��� �о buffer_queue�� �ִ´�
	      // -> serialevent()�Լ��� ��ġ��.
	{
	  port_interrupt();
	} while (AXSerial.available() > 0);
	*/
	if (!IsReadyToRX)	// g_Qlast == g_Qfirst
	{
		_printF(F("No data in RX buffer \r\n"));
		// Serial.println("No data in RX buffer");
	}
	else
	{
		do
		{
			// [FF] [FF] [ID] [LEN] [ERR] [PARM...] [CHECKSUM]
			if (g_Qlast < g_Qfirst) Rx_Len = g_Qlast+BUFFER_LENGTH - g_Qfirst;
			else 					Rx_Len = g_Qlast - g_Qfirst;
			
			#define MIN_PACKET  6
			//---------------------------------
			// Packet �ּұ��� ���.
			//---------------------------------
			if (Rx_Len >= MIN_PACKET)	
			// [����] ���� �����Ͱ� Packet�� �ּұ��̺��� �� ���
			{
				//---------------------------------
				// Header Ȯ��.
				//--------------------------------- 
				if ((buffer_queue[g_Qfirst] == 0xFF) && (buffer_queue[g_Qfirst+1] == 0xFF))
				// [����] Header�� 0xFF, 0xFF�� ���
				{
					// Packet Length ���.
					Packet_Length = buffer_queue[g_Qfirst + 3];
					
					//---------------------------------
					// Packet Length Ȯ��
					//---------------------------------
					if (Rx_Len >= (Packet_Length + 4))
					// [����] ���� �����Ͱ� Packet�� �ѱ��̺��� ����
					{
						// CheckSum�� ���.
						CheckSum = 0;
						for(i=2;i<(Packet_Length + 3);i++)
						{
							CheckSum = CheckSum + buffer_queue[g_Qfirst + i];
						}
						CheckSum = ~CheckSum;
						
						//---------------------------------
						// Check Sum Ȯ��. 
						//---------------------------------
						if (buffer_queue[g_Qfirst + Packet_Length + 3] == CheckSum)
						// [����]
						{
							// Packet ���.
							for (i=0;i<(Packet_Length + 4);i++)
							{
								pRxData[i] = buffer_queue[g_Qfirst+i];	// ������ ���� ��ȯ.
							}
							Result = i;
							// g_Qfirst�� ���� Packet�� ���� ������ �̵�.
							g_Qfirst = g_Qfirst + Packet_Length + 4;
							break;
						}
						else
						// [�̻�] 
						{
							// printF(F("CheckSum Error");
							// g_Qfirst�� �ϳ� �̵����Ѽ� �ٽ� �˻�.
							NEXT_FIRST;
						}
					}
					else 
					// [�̻�] Length�� ����Ҷ� Length�� ª�� ���.
					{
						// printF(F("RX is not complete(2) \r\n"));
						break;
					}
				}
				// [�̻�] ���� ����� �ƴ� ��� 
				else
				{
					// g_Qfirst�� �ϳ� �̵����Ѽ� �ٽ� �˻�.
					// printF(F("Header Error \r\n"));
					NEXT_FIRST;
				}
			}
			// [�̻�] ���� �����Ͱ� Packet�� �ּұ��̺��� �� ���
			else
			{
				// printF(F("RX is not complete(1), first = %d, last = %d \r\n",g_Qfirst,g_Qlast);
				break;
			}
		}while(TRUE);
		/*
		while(IsReadyToRX)
		{
			putch(ReadChar());
		}
		*/
	}
	return Result;
}

// ������ RX Data�� ��� ȭ�鿡 ���
void Dump_RX_data()
{
	while (IsReadyToRX)	// g_Qlast == g_Qfirst
	{
		_printF(F("["));
		PrintHex(buffer_queue[g_Qfirst++]);
		_printF(F("]"));
	}
	_printF(F("\r\n"));
	return;
}

// AX-12�� Register�� pTable�� Data�� Length��ŭ ����.
// Length�� pTable�� ���� ������.
void WriteHex(int ID, unsigned char Addr, unsigned char * pTable, int Length)
{
	int i;
	unsigned char pTxData[BUFFER_LENGTH];
	
	pTxData[0] = pTxData[1] = 0xFF;
	pTxData[2] = ID;
	pTxData[3] = Length + 3;
	pTxData[4] = INST_WRITE;
	pTxData[5] = Addr;
	// Parameter
	for (i=0;i<Length;i++)
	{
		pTxData[i+6] = pTable[i];
	}
	// Check Sum ���.
	pTxData[Length + 6] = CalCheckSum(pTxData,Length + 6);
	
	// Data �۽�.
	SendHexStr(pTxData, Length + 6);	
	_printF(F("\r\n"));
}

// Ű���� �Ǵ� BT���� �� ����Ʈ�� ���������� ���
char getSerialChar(void)
{
  char temp;

 	while(Serial.available()>0) Serial.read();  // �����Ⱚ ������
  while(BTSerial.available()>0) BTSerial.read();  // �����Ⱚ ������

 	while((Serial.available()==0) && (BTSerial.available()==0));  // Ű�� ������ ���� ���
  if (Serial.available() > 0) temp = (char)Serial.read();
 	else if (BTSerial.available() > 0) temp = (char)BTSerial.read();
 	else temp = _NULL;
 	
 	return temp;
}

