
//*****************************************************************
//    �Լ�����(com_test�ϱ� 6_serial���� ���� ����, ���� �Ժη� �ٲ��� ����)
//*****************************************************************
// void interrupt timer_interrupt(...); // for DOS

// ������ ���ͷ�Ʈ ���͸� �����ϱ� ���� �Լ� ??
// ���� Timer Interrupt�� ISR ���� �뵵��, �Լ��ΰ�? �����ΰ�? ,
// void interrupt (*old_intr_08)(...);

void timer_interrupt(void);   // for Arduino
void Timer_Interrupt_Init(void);
void Timer_Interrupt_Close(void);
// void PC_ElapsedInit(void);
// void PC_ElapsedStart(void);
// INT16U PC_ElapsedStop(void);
// void PC_SetTickRate (unsigned int freq);

//*****************************************************************/
//    ��ũ�� ����
//*****************************************************************