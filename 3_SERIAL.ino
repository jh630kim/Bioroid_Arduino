////////////////////////////////////////////////////////////////////
// 작성일 : 051126
// 제  목 : Robot의 AVR과 통신을 하는 프로그램.
// 내  용 : COM2, ?????BPS, 8 data bit, 1 stop bit, no parity로 고정됨. 
//          내 PC의 COM1이 고장났다. 
////////////////////////////////////////////////////////////////////

//*****************************************************************
//   함수. 
//*****************************************************************
// 새로 정의되는 interrrupt handdler
// 통신 포트에 데이터가 들어오면 인터럽트가 걸린다. (for DOS)
/*
void interrupt port_interrupt(...)
{
	// Queue가 가득 찻으면 경고...
	// 필요할까?
	// 1000개 버퍼가 다 차도록 처리를 하지 않는다는 건 뭔가 문제가 있다.
	// 그냥 넘어가자.

	// 포트에서 데이터를 읽어와 저장
	buffer_queue[g_Qlast] = inportb(g_base_addr);
	
	// g_Qlast : 다음 입력할 버퍼의위치를 가리킴. 
	NEXT_LAST;
	
	// PIC의 EIO 인터럽트에 인터럽트가 끝났다는 신호 보내기
	// outportb(PICEIO,0x20);
}
*/

// Arduino의 Software Serial은 Interrupt가 없다.
// (Hardware Serial은 serialEvent()라는 Interrupt가 있다)
// 그런데, 수신 Data는 RX_data_check()와 ReadChar()에서만 사용한다.
// 그 함수 안에서 인터럽트 대신 사용하자
// 한 바이트의 데이터를 읽어서 buffer_queue에 넣는다.
// AX-12는 데이터를 요청할 때 feedback을 하는데,
// 데이터 요청 후 10ms delay를 준다.
/*
void port_interrupt(void) // // AXSerial(Serial2)에 대한 Interrupt 대신 사용
{
	if (AXSerial.available() > 0)
	{
	  // 포트에서 데이터를 읽어와 저장
	  buffer_queue[g_Qlast] = AXSerial.read();
	  // g_Qlast : 다음 입력할 버퍼의위치를 가리킴. 
	  NEXT_LAST;
	}
}
*/
void serialEvent2(void)  // AXSerial(Serial2)에 대한 Interrupt
{
	while (AXSerial.available() > 0)
	{
	  // 포트에서 데이터를 읽어와 저장
	  buffer_queue[g_Qlast] = AXSerial.read();
	  // g_Qlast : 다음 입력할 버퍼의위치를 가리킴. 
	  NEXT_LAST;
	}
}

// 직렬 포트 초기화 및 인터럽트 설정. 
// 인터럽트 설정은 outport로 직접 설정해야 한다.
void Serial_Port_Init(void)
{
  // Serial SetUp
  // SoftwareSerial용 Port
  // pinMode(CNT_RX, INPUT);
  // pinMode(CNT_TX, OUTPUT);
  pinMode(RTS_PIN, OUTPUT);  // RST PIN을 Output으로 설정
  Serial.begin(9600);       // PC의 USB용 Serial Port
  BTSerial.begin(9600);     // BT용
  AXSerial.begin(115200);   // 모터제어용
}
	 
void Close_Com(void)
{
  Serial.end();       // PC의 USB용 Serial Port
  BTSerial.end();     // BT용
  AXSerial.end();   // 모터제어용
}

// 수신 버퍼로부터 1byte의 데이터를 읽음. 
unsigned char ReadChar(void)
{
	unsigned char result;
	 
  // port_interrupt(); // 한 바이트의 데이터를 읽어서 buffer_queue에 넣는다.

	// 데이터 읽기
	result = buffer_queue[g_Qfirst];
	
	// 다음 데이터를 읽을 위치로 이동
	NEXT_FIRST;

	return result;
}

// Hex 1byte 데이터를 화면으로 출력. 
void PrintHex(unsigned char bHex)
{
	unsigned char bpStr[2];
	
	// 첫번째 바이트 
	bpStr[0] =((unsigned char)(bHex>>4)&0x0f) + (unsigned char)'0';
	if(bpStr[0] > '9') bpStr[0] += 7;
	
	// 두번째 바이트
	bpStr[1] =(unsigned char)(bHex & 0x0f) + (unsigned char)'0';
	if(bpStr[1] > '9') bpStr[1] += 7;
	
	// 화면 출력
	_printF(F("%c%c "),bpStr[0], bpStr[1]);
	// Serial.print(bHex,HEX);
}

// Hex String을 화면으로 출력. 
void PrintHexStr(unsigned char* bpBuffer, unsigned char bCount)
{
	int i;
	for (i=0;i<bCount;i++)
	{
		PrintHex(bpBuffer[i]);
	}
}

// Hex String을 Serial Port로 전송
void SendHexStr(unsigned char* bpBuffer, int bCount)
{
	int i;
	SET_RTS;  // (TBC) 첫번째 데이터가 깨지는 것 같으면 SendHexStr을 호출하는 쪽으로 옮기자.
	for(i=0;i<=bCount;i++)
	{
		AXSerial.write(bpBuffer[i]);  // Arduino (TBC, 확인 필요) Write 말고 다른 것은?
	}
	RESET_RTS;
}

// Hex String을 TxQueue로 전송
// Tx Queue에서는 Timer Interrupt에서 Packet을 전송한다. 
void SendHexStr_to_TxQueue(unsigned char* bpBuffer)
{
	int total_length;
	char ch;

	// Queue가 가득 차 있는동안 대기
	// Timer Interrupt가 멈추지 않는 한 무한루프에 빠질 염려는 없다. 
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
	
	// 허리
	TxData[67] = WAIST_ID;
	TxData[68] = position_ax_ARM[RIGHT][0] & 0xFF;
	TxData[69] = (position_ax_ARM[RIGHT][0]>>8) & 0xFF;
	TxData[70] = speed_ax_ARM[RIGHT][0] & 0xFF;
	TxData[71] = (speed_ax_ARM[RIGHT][0]>>8) & 0xFF;
	
	// 오른팔
	for (i=1;i<4;i++)
	{
		TxData[67+i*5] = ARM_ID[RIGHT][i];
		TxData[68+i*5] = position_ax_ARM[RIGHT][i] & 0xFF;
		TxData[69+i*5] = (position_ax_ARM[RIGHT][i]>>8) & 0xFF;
		TxData[70+i*5] = speed_ax_ARM[RIGHT][i] & 0xFF;
		TxData[71+i*5] = (speed_ax_ARM[RIGHT][i]>>8) & 0xFF;
	}
	// 왼팔
	for (i=1;i<4;i++)
	{
		TxData[82+i*5] = ARM_ID[LEFT][i];
		TxData[83+i*5] = position_ax_ARM[LEFT][i] & 0xFF;
		TxData[84+i*5] = (position_ax_ARM[LEFT][i]>>8) & 0xFF;
		TxData[85+i*5] = speed_ax_ARM[LEFT][i] & 0xFF;
		TxData[86+i*5] = (speed_ax_ARM[LEFT][i]>>8) & 0xFF;
	}	
	
	TxData[102] = CalCheckSum(TxData,102);
	
	// Data 송신.
	// SendHexStr(TxData, 67);		// 0 ~ i까지 데이터 송신
	SendHexStr_to_TxQueue(TxData);		// 0 ~ i까지 데이터 송신
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
	
	// Data 송신.
	// SendHexStr(TxData, 67);		// 0 ~ i까지 데이터 송신
	SendHexStr_to_TxQueue(TxData);		// 0 ~ i까지 데이터 송신
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
	
	// 허리
	TxData[ 7] = WAIST_ID;
	TxData[ 8] = position_ax[RIGHT][0] & 0xFF;
	TxData[ 9] = (position_ax[RIGHT][0]>>8) & 0xFF;
	TxData[10] = speed_ax[RIGHT][0] & 0xFF;
	TxData[11] = (speed_ax[RIGHT][0]>>8) & 0xFF;
	
	// 오른팔
	for (i=1;i<4;i++)
	{
		TxData[ 7+i*5] = ARM_ID[RIGHT][i];
		TxData[ 8+i*5] = position_ax[RIGHT][i] & 0xFF;
		TxData[ 9+i*5] = (position_ax[RIGHT][i]>>8) & 0xFF;
		TxData[10+i*5] = speed_ax[RIGHT][i] & 0xFF;
		TxData[11+i*5] = (speed_ax[RIGHT][i]>>8) & 0xFF;
	}
	// 왼팔
	for (i=1;i<4;i++)
	{
		TxData[22+i*5] = ARM_ID[LEFT][i];
		TxData[23+i*5] = position_ax[LEFT][i] & 0xFF;
		TxData[24+i*5] = (position_ax[LEFT][i]>>8) & 0xFF;
		TxData[25+i*5] = speed_ax[LEFT][i] & 0xFF;
		TxData[26+i*5] = (speed_ax[LEFT][i]>>8) & 0xFF;
	}	
	TxData[42] = CalCheckSum(TxData,42);
	
	// Data 송신.
	// SendHexStr(TxData, 67);		// 0 ~ i까지 데이터 송신
	SendHexStr_to_TxQueue(TxData);		// 0 ~ i까지 데이터 송신
}

// TxData의 Packet의 Checksum을 계산.
// Count는 Checksum의 위치임.
unsigned char CalCheckSum(unsigned char* TxData, int Count)
{
	unsigned char CheckSum;
	int j;
		
	// Check Sum의 계산
	CheckSum = 0;
	for(j=2;j<Count;j++)
	{
		CheckSum = CheckSum + TxData[j];
	}
	return ~CheckSum;
}

// return  : 받은 데이터 개수 (Error의 경우 0을 Return함)
// pRxData : 받은 데이터
int RX_data_check(unsigned char* pRxData)
{
	int i, Rx_Len, Packet_Length, Result;
	unsigned char CheckSum;
	
	Result = 0;			// Data 없음.
	pRxData[2] = 0;		// ID를 0으로 변경.(사용하지 않는 ID)
	/*
	do  	// 데이터가 있는 동안 계속 읽어서 buffer_queue에 넣는다
	      // -> serialevent()함수로 대치함.
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
			// Packet 최소길이 계산.
			//---------------------------------
			if (Rx_Len >= MIN_PACKET)	
			// [정상] 수신 데이터가 Packet의 최소길이보다 긴 경우
			{
				//---------------------------------
				// Header 확인.
				//--------------------------------- 
				if ((buffer_queue[g_Qfirst] == 0xFF) && (buffer_queue[g_Qfirst+1] == 0xFF))
				// [정상] Header가 0xFF, 0xFF인 경우
				{
					// Packet Length 계산.
					Packet_Length = buffer_queue[g_Qfirst + 3];
					
					//---------------------------------
					// Packet Length 확인
					//---------------------------------
					if (Rx_Len >= (Packet_Length + 4))
					// [정상] 수신 데이터가 Packet의 총길이보다 긴경우
					{
						// CheckSum의 계산.
						CheckSum = 0;
						for(i=2;i<(Packet_Length + 3);i++)
						{
							CheckSum = CheckSum + buffer_queue[g_Qfirst + i];
						}
						CheckSum = ~CheckSum;
						
						//---------------------------------
						// Check Sum 확인. 
						//---------------------------------
						if (buffer_queue[g_Qfirst + Packet_Length + 3] == CheckSum)
						// [정상]
						{
							// Packet 출력.
							for (i=0;i<(Packet_Length + 4);i++)
							{
								pRxData[i] = buffer_queue[g_Qfirst+i];	// 버퍼의 값을 반환.
							}
							Result = i;
							// g_Qfirst를 다음 Packet을 읽을 곳으로 이동.
							g_Qfirst = g_Qfirst + Packet_Length + 4;
							break;
						}
						else
						// [이상] 
						{
							// printF(F("CheckSum Error");
							// g_Qfirst를 하나 이동시켜서 다시 검사.
							NEXT_FIRST;
						}
					}
					else 
					// [이상] Length를 고려할때 Length가 짧은 경우.
					{
						// printF(F("RX is not complete(2) \r\n"));
						break;
					}
				}
				// [이상] 정상 헤더가 아닌 경우 
				else
				{
					// g_Qfirst를 하나 이동시켜서 다시 검사.
					// printF(F("Header Error \r\n"));
					NEXT_FIRST;
				}
			}
			// [이상] 수신 데이터가 Packet의 최소길이보다 긴 경우
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

// 수신한 RX Data를 모두 화면에 출력
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

// AX-12의 Register에 pTable의 Data를 Length만큼 넣음.
// Length는 pTable의 인자 개수임.
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
	// Check Sum 계산.
	pTxData[Length + 6] = CalCheckSum(pTxData,Length + 6);
	
	// Data 송신.
	SendHexStr(pTxData, Length + 6);	
	_printF(F("\r\n"));
}

// 키보드 또는 BT에서 한 바이트를 읽을때까지 대기
char getSerialChar(void)
{
  char temp;

 	while(Serial.available()>0) Serial.read();  // 쓰레기값 버리기
  while(BTSerial.available()>0) BTSerial.read();  // 쓰레기값 버리기

 	while((Serial.available()==0) && (BTSerial.available()==0));  // 키가 눌릴때 까지 대기
  if (Serial.available() > 0) temp = (char)Serial.read();
 	else if (BTSerial.available() > 0) temp = (char)BTSerial.read();
 	else temp = _NULL;
 	
 	return temp;
}

