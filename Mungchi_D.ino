////////////////////////////////////////////////////////////////////
// �ۼ��� : 051231
// ������ : 180521
// ��  �� : DOS������ ������ ���α׷��� Arduino�� ����
// ��  �� : 
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
	// Timer Interrupt ����
	Timer_Interrupt_Init();   // ������ Arduino�� MsTimer2 �ʱ�ȭ�� ������.

	// ���� �ʱ�ȭ
	g_t1x_diff = g_t2x_diff = 0;
	g_t1y_diff = g_t2y_diff = 0;
	g_t1z_diff = g_t2z_diff = 0;
	g_TE_diff = 0;

	g_z_RIGHT_BASE = -40;
	g_z_LEFT_BASE = 40;

	g_Send_Flag = 1;    // Timer Interrupt���� Data�� ���� ������ ����.
	g_Emer_Stop = 0;
  g_curr_time = micros();

 	for (int i=0;i<MAX_CMD_BUFFER;i++) MonCmd[i] = _NULL;

	// SD Card Ȯ��
  _printF(F("Initializing SD card...\r\n"));
  if (!card.init(SPI_HALF_SPEED, SD_CS))
    _printF(F("Init SD Card failed.\r\n"));
  else if (!volume.init(card))
    _printF(F("Card format is not FAT16 or FAT32.\r\n"));
  else
    _printF(F("SD Card is correctly inserted.\r\n"));
  
  // SD Card �ʱ�ȭ
  if (!SD.begin(SD_CS))   // 50,51,52 for SPI, 53 for Chip Select (Mega)
    _printF(F("Begin SD Card failed.\r\n"));
  else 
    _printF(F("Begin SD Card.\r\n"));
    
  // �ʱ� ������Ʈ
	_printF(F(">")); // BT�� �߰� (TBD)
}

///////////////////////////////////////////////
// put your main code here, to run repeatedly:
///////////////////////////////////////////////
void loop() 
{
	// char szTmp[2];
	// 
	// ��� ���� ���� �ʱ�ȭ (TBD)
	// for (int i=0;i<80;i++) OneLineBuff[80] = NULL;
	// ���⼭ �ʱ�ȭ�� ��Ű�� �ȵȴ�. ��� loop�� ���� �Ǵϱ�.
	// �׷���... ������ �� null�� �ǳ�?
	// ��ó�� �ʱ�ȭ�� ���Ѻ���... �ȵ��ݾ�?

  // The first byte of incomming data available
 	while((Serial.available()==0));// && (BTSerial.available()==0));  // Ű�� ������ ���� ���
  if (Serial.available() > 0) // PC �Է��� ���
  {
    int i = 0;
    MonCmd[i++] = (char)Serial.read();
    _printf("%s\r\n",MonCmd);
    MonitorRoutine();
  }
 	else if (BTSerial.available() > 0)  // BT �Է��� ���
 	{
    int i = 0;
    MonCmd[i++] = (char)BTSerial.read();
    _printf("%s\r\n",MonCmd);
    MonitorRoutine();
  }
 	for (int i=0;i<MAX_CMD_BUFFER;i++) MonCmd[i] = _NULL;
/* Arduino�� PC �Է� Ư���� �ʿ� ����. BT�� ��� Ȯ�� �ʿ�(TBD)
	switch(szTmp[0])
	{
		// CR : ��� �ؼ� ����
	   	case 13:
			MonitorRoutine(); //MonCmd
			OneLineBuff[0]=_NULL;
			break;
		// BS : Back Space
	   	case 8:
			if (OneLineBuff[0]!=_NULL)
	    	{
	    		OneLineBuff[strlen(OneLineBuff)-1]=0;
	    		_printF(F("\b ")); _printF(F("\b"));	//Echo, �ʿ� ���� ��(TBD)
	    	}
			break;
		// ��� �Է�
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

