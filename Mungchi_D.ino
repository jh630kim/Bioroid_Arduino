////////////////////////////////////////////////////////////////////
// 작성일 : 051231
// 수정일 : 180521
// 제  목 : DOS용으로 개발한 프로그램을 Arduino로 수정
// 내  용 : 
////////////////////////////////////////////////////////////////////

#include "Mungchi.h"

///////////////////////////////////////////////
// put your setup code here, to run once:
///////////////////////////////////////////////
void setup() 
{
  // Serial Port(for PC, for BlueTooth, for AX-12)
  Serial_Port_Init();
  
  // Timer SetUp
	// Timer Interrupt 변경
	Timer_Interrupt_Init();   // 내용을 Arduino의 MsTimer2 초기화로 변경함.

	// 변수 초기화
	g_t1x_diff = g_t2x_diff = 0;
	g_t1y_diff = g_t2y_diff = 0;
	g_t1z_diff = g_t2z_diff = 0;
	g_TE_diff = 0;

	g_z_RIGHT_BASE = -40;
	g_z_LEFT_BASE = 40;

	g_Send_Flag = 1;    // Timer Interrupt에서 Data를 보낼 것인지 결정.
	g_Emer_Stop = 0;
  g_curr_time = micros();

 	for (int i=0;i<MAX_CMD_BUFFER;i++) MonCmd[i] = _NULL;

	// SD Card 확인
  _printF(F("Initializing SD card...\r\n"));
  if (!card.init(SPI_HALF_SPEED, SD_CS))
    _printF(F("Init SD Card failed.\r\n"));
  else if (!volume.init(card))
    _printF(F("Card format is not FAT16 or FAT32.\r\n"));
  else
    _printF(F("SD Card is correctly inserted.\r\n"));
  
  // SD Card 초기화
  if (!SD.begin(SD_CS))   // 50,51,52 for SPI, 53 for Chip Select (Mega)
    _printF(F("Begin SD Card failed.\r\n"));
  else 
    _printF(F("Begin SD Card.\r\n"));
    
  // 초기 프롬프트
	_printF(F(">")); // BT쪽 추가 (TBD)
}

///////////////////////////////////////////////
// put your main code here, to run repeatedly:
///////////////////////////////////////////////
void loop() 
{
	// char szTmp[2];
	// 
	// 명령 라인 버퍼 초기화 (TBD)
	// for (int i=0;i<80;i++) OneLineBuff[80] = NULL;
	// 여기서 초기화를 시키면 안된다. 계속 loop를 돌게 되니까.
	// 그런데... 생성될 때 null이 되나?
	// 위처럼 초기화를 시켜봤자... 안되잖아?

  // The first byte of incomming data available
 	while((Serial.available()==0));// && (BTSerial.available()==0));  // 키가 눌릴때 까지 대기
  if (Serial.available() > 0) // PC 입력의 경우
  {
    int i = 0;
    MonCmd[i++] = (char)Serial.read();
    _printf("%s\r\n",MonCmd);
    MonitorRoutine();
  }
 	else if (BTSerial.available() > 0)  // BT 입력의 경우
 	{
    int i = 0;
    MonCmd[i++] = (char)BTSerial.read();
    _printf("%s\r\n",MonCmd);
    MonitorRoutine();
  }
 	for (int i=0;i<MAX_CMD_BUFFER;i++) MonCmd[i] = _NULL;
/* Arduino의 PC 입력 특성상 필요 없다. BT의 경우 확인 필요(TBD)
	switch(szTmp[0])
	{
		// CR : 명령 해석 시작
	   	case 13:
			MonitorRoutine(); //MonCmd
			OneLineBuff[0]=_NULL;
			break;
		// BS : Back Space
	   	case 8:
			if (OneLineBuff[0]!=_NULL)
	    	{
	    		OneLineBuff[strlen(OneLineBuff)-1]=0;
	    		_printF(F("\b ")); _printF(F("\b"));	//Echo, 필요 없을 듯(TBD)
	    	}
			break;
		// 명령 입력
		default:
			if (strlen(OneLineBuff)<sizeof(OneLineBuff)-1)
	    	{
	    		strcat(OneLineBuff, szTmp);
	    		_printF(F("echo=%s"), szTmp);	//Echo
	    	}
	    	break;
	}
*/
}

