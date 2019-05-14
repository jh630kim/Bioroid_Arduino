
//*****************************************************************
//    함수선언(com_test하구 6_serial에서 같이 쓴다, 형을 함부로 바꾸지 말자)
//*****************************************************************
// void interrupt timer_interrupt(...); // for DOS

// 기존의 인터럽트 벡터를 저장하기 위한 함수 ??
// 이전 Timer Interrupt의 ISR 수행 용도임, 함수인가? 선언인가? ,
// void interrupt (*old_intr_08)(...);

void timer_interrupt(void);   // for Arduino
void Timer_Interrupt_Init(void);
void Timer_Interrupt_Close(void);
// void PC_ElapsedInit(void);
// void PC_ElapsedStart(void);
// INT16U PC_ElapsedStop(void);
// void PC_SetTickRate (unsigned int freq);

//*****************************************************************/
//    매크로 선언
//*****************************************************************
