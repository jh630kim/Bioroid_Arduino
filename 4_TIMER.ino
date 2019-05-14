////////////////////////////////////////////////////////////////////
// �ۼ��� : 051231
// ��  �� : Timer Interrutp
// ��  �� : ���� �ð��������� ���ͷ�Ʈ�� �߻��ϴ� ���α׷�
////////////////////////////////////////////////////////////////////

//*****************************************************************
//   �Լ�. 
//*****************************************************************
// ���� ���ǵǴ� interrrupt handdler ����.. DOS�� ���
// �� 17msec(1/60)���� ���ͷ�Ʈ�� �ɸ���. 
// void interrupt timer_interrupt(...)  // for DOS
void timer_interrupt(void)  // for Arduino
{
	// JHK_debug
	// Timer Interrupt�� �ֱ� ����.
	// g_time_period = PC_ElapsedStop();  // PC�� Timer#2�� �̿��� ���ͷ�Ʈ ������ üũ��
	// PC_ElapsedStart();                 // Arduino������ �ٸ� ����� ã�ƾ� �Ѵ�. (TBD)
	g_prev_time = g_curr_time;
	g_curr_time = micros();
  g_time_period = g_curr_time - g_prev_time;

	// TX Queue�� �����Ͱ� ������, �׸��� �۽����(g_Send_Flag)�̸� �����͸� �۽�
	if (IsReadyToTX && g_Send_Flag)
	{
		SET_RTS;			// �۽� ���� ����
		for (int i=0;i<(g_Tx_Packet[g_Tx_Qfirst][3]+4);i++)
		{
		  // Serial Data �۽�
			AXSerial.flush(); // ������ �� ������ ������ �Ϸ�� �� ���� ���
      AXSerial.write(g_Tx_Packet[g_Tx_Qfirst][i]);  // Arduino (TBC, Ȯ�� �ʿ�) Write ���� �ٸ� ����?
		}
		RESET_RTS;			// ���� ���� ����.

		// Queue First�� �ϳ� ����.
		NEXT_TX_FIRST;
	}

	// ���� Timer Interrupt�� ISR ����. // for DOS
	// (*old_intr_08)();
}

void Timer_Interrupt_Init(void)
{
  // 10ms���� timer_interrupt ȣ��
	MsTimer2::set(10,timer_interrupt);
	MsTimer2::start();  // timer_interrupt ����
}

void Timer_Interrupt_Close(void)
{
	MsTimer2::stop();   // timer_interrupt ����
}


///////////////////////////////////////////////////
// PC Timer Interrupt��, Arduino������ �ʿ� ����
///////////////////////////////////////////////////
/*
*********************************************************************************************************
*                                      SET THE PC'S TICK FREQUENCY
*
* Description: This function is called to change the tick rate of a PC.
*
* Arguments  : freq      is the desired frequency of the ticker (in Hz)
*
* Returns    : none
*
* Notes      : 1) The magic number 2386360 is actually twice the input frequency of the 8254 chip which
*                 is always 1.193180 MHz.
*              2) The equation computes the counts needed to load into the 8254.  The strange equation
*                 is actually used to round the number using integer arithmetic.  This is equivalent to
*                 the floating point equation:
*
*                             1193180.0 Hz
*                     count = ------------ + 0.5
*                                 freq
*********************************************************************************************************
*/
/*
void PC_SetTickRate (unsigned int freq)
{
    unsigned int     count;


    if (freq == 18) 
    {                            // See if we need to restore the DOS frequency        
        count = 0;
    } 
    else if (freq > 0) 
    {                        
                                                 // Compute 8254 counts for desired frequency and ...  
                                                 // ... round to nearest count                         
        count = (INT16U)(((INT32U)2386360L / freq + 1) >> 1); 
    } 
    else 
    {
        count = 0;
    }

    outp(TICK_T0_8254_CWR,  TICK_T0_8254_CTR0_MODE3); // Load the 8254 with desired frequency           
    outp(TICK_T0_8254_CTR0, count & 0xFF);            // Low  byte                                     
    outp(TICK_T0_8254_CTR0, (count >> 8) & 0xFF);     // High byte                                     
}
*/
/*
*********************************************************************************************************
*                                       ELAPSED TIME INITIALIZATION
*
* Description : This function initialize the elapsed time module by determining how long the START and
*               STOP functions take to execute.  In other words, this function calibrates this module
*               to account for the processing time of the START and STOP functions.
*
* Arguments   : None.
*
* Returns     : None.
*********************************************************************************************************
*/
/*
void PC_ElapsedInit(void)
{
    PC_ElapsedOverhead = 0;
    PC_ElapsedStart();
    PC_ElapsedOverhead = PC_ElapsedStop();
    printf("%d \r\n",PC_ElapsedOverhead);
}
*/
/*
*********************************************************************************************************
*                                         INITIALIZE PC'S TIMER #2
*
* Description : This function initialize the PC's Timer #2 to be used to measure the time between events.
*               Timer #2 will be running when the function returns.
*
* Arguments   : None.
*
* Returns     : None.
*********************************************************************************************************
*/
/*
void PC_ElapsedStart(void)
{
    INT8U      data;

    data  = (INT8U)inp(0x61);                              // Disable timer #2
    data &= 0xFE;
    outp(0x61, data);
    
    outp(TICK_T0_8254_CWR,  TICK_T0_8254_CTR2_MODE0);      // Program timer #2 for Mode 0
    outp(TICK_T0_8254_CTR2, 0xFF);
    outp(TICK_T0_8254_CTR2, 0xFF);
    
    data |= 0x01;                                          // Start the timer
    outp(0x61, data);
}
*/

/*
*********************************************************************************************************
*                                 STOP THE PC'S TIMER #2 AND GET ELAPSED TIME
*
* Description : This function stops the PC's Timer #2, obtains the elapsed counts from when it was
*               started and converts the elapsed counts to micro-seconds.
*
* Arguments   : None.
*
* Returns     : The number of micro-seconds since the timer was last started.
*
* Notes       : - The returned time accounts for the processing time of the START and STOP functions.
*               - 54926 represents 54926S-16 or, 0.838097 which is used to convert timer counts to
*                 micro-seconds.  The clock source for the PC's timer #2 is 1.19318 MHz (or 0.838097 uS)
*********************************************************************************************************
*/
/*
INT16U PC_ElapsedStop(void)
{
    INT8U      data;
    INT8U      low;
    INT8U      high;
    INT16U     cnts;

    data  = (INT8U)inp(0x61);                                    // Disable the timer
    data &= 0xFE;
    outp(0x61, data);
    
    outp(TICK_T0_8254_CWR, TICK_T0_8254_CTR2_LATCH);             // Latch the timer value
    low  = inp(TICK_T0_8254_CTR2);
    high = inp(TICK_T0_8254_CTR2);
    cnts = (INT16U)0xFFFF - (((INT16U)high << 8) + (INT16U)low); // Compute time it took for operation

    return ((INT16U)((INT32U)cnts * 54926L >> 16) - PC_ElapsedOverhead);
}*/
