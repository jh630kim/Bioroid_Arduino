/////////////////////////////////////////
// ÇÔ¼ö Header
/////////////////////////////////////////
// void MonitorRoutine(char* MonCmd);
void MonitorRoutine(void);
void Get_Position(char* pArgument, int ArgLen);
void Move_Position(char* pArgument,int ArgLen);
void Move_For_Kin(char* pArgument, int ArgLen);
void Move_For_Kin_BODY(char* pArgument,int ArgLen);
void Move_Inv_Kin(char* pArgument,int ArgLen);
void Move_Both_LEG(char* pArgument,int ArgLen);
void Check_Kin(char* pArgument,int ArgLen);
void Check_Kin_Body(char* pArgument,int ArgLen);
void Move_Zero(void);
void Move_Zero_BODY(void);
void Get_Hex_Data(char* pArgument, int ArgLen);
void Move_COG_Walking(char* pArgument,int ArgLen);
void Change_Comp_Slope(char* pArgument,int ArgLen);
void Get_COG_Data(void);

void Check_Mtx(char* pArgument,int ArgLen);
void Check_All_Torque(char* pArgument,int ArgLen);
void Check_BODY_Torque(char* pArgument,int ArgLen);


