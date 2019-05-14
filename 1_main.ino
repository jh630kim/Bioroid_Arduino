//-----------------------------------------------------------------------------
//	Monitor Routinue
//-----------------------------------------------------------------------------
// void MonitorRoutine(char* MonCmd)
void MonitorRoutine(void)
{
	char* pData;
	int ID;
	int ArgLen;
	unsigned char pRxData[BUFFER_LENGTH], pTable[BUFFER_LENGTH];

	// Serial.println("");
	_printF(F("\r\n"));
  _printf("M1");
	ArgLen = strlen(MonCmd);		// ��ü ����� ����.

	pData = strtok(MonCmd," ");						// ��� ����
	pData = strtok(pData + strlen(pData) + 1," ");	// ù��° ���� ����
	ArgLen = (int)(ArgLen - (pData - MonCmd));				// ��ü ����� Argument�� ����.

	// ��� Ȯ��, ù��° ���
	switch (MonCmd[0])
	{
		case 'd':
			if (MonCmd[1] == '1')	{SendHexStr_to_TxQueue(dbg_jhk1);	break;}
			if (MonCmd[1] == '2')	{SendHexStr_to_TxQueue(dbg_jhk2);	break;}
			break;
		case 'c':
			if (MonCmd[1] == 'p')
			{
				int i, result;
				result = RX_data_check(pRxData);
				for (i=0;i<result;i++)
				{
					 _printF(F("[%02X]"),pRxData[i]);
					// Serial.print("[");
					// Serial.print(pRxData[i],HEX);
					// Serial.print("]");
				}
				// Serial.println(" ");
				_printF(F("\r\n"));
				break;
			}
			if (MonCmd[1] == 'd')	{ Dump_RX_data();					break; }
			if (MonCmd[1] == 'f')	{ _printF(F("Control Period = %u usec\r\n"), g_time_period); break; }
                      		//{ Serial.print( "Control Period ="); Serial.println(g_time_period); break; }
			if (MonCmd[1] == 'c') 	{ Check_Mtx(pData,ArgLen);			break;   }
			// if (MonCmd[1] == 't') 	{ Check_BODY_Torque(pData,ArgLen);	break;   }
			if (MonCmd[1] == 't') 	{ Check_All_Torque(pData,ArgLen);	break;   }
			goto DispUsage;
		case 'w':
			if (MonCmd[1] == 'c')	{ Change_Comp_Slope(pData,ArgLen);	break; }
			if (MonCmd[1] == 't')
			{
				if (*pData == 'a')
				{
					pData = strtok(pData + strlen(pData) + 1," ");	// �ι�° ���� ����
					pTable[0] = atoi(pData);
					WriteHex(BROADCASTING_ID, P_TORQUE_ENABLE, pTable, 1);
				}
				else
				{
					ID = atoi(pData);

					pData = strtok(pData + strlen(pData) + 1," ");	// �ι�° ���� ����
					pTable[0] = atoi(pData);

					WriteHex(ID, P_TORQUE_ENABLE, pTable, 1);
				}
				break;
			}
		 	else if (MonCmd[1] == 'l')
			{
				if (*pData == 'a')
				{
					pData = strtok(pData + strlen(pData) + 1," ");	// �ι�° ���� ����
					pTable[0] = atoi(pData);
					WriteHex(BROADCASTING_ID, P_LED, pTable, 1);
				}
				else
				{
					ID = atoi(pData);

					pData = strtok(pData + strlen(pData) + 1," ");	// �ι�° ���� ����
					pTable[0] = atoi(pData);

					WriteHex(ID, P_LED, pTable, 1);
				}
				break;
			}
			goto DispUsage;
		case 'm':
			if (MonCmd[1] == 'v') {	Move_Position(pData,ArgLen);	break;   }
		  if (MonCmd[1] == 'f') {	Move_For_Kin(pData, ArgLen);   	break;   }
		   // if (MonCmd[1] == 'f') {	Move_For_Kin_BODY(pData, ArgLen);  	break;   }
		  if (MonCmd[1] == 'i') {	Move_Inv_Kin(pData, ArgLen);   	break;   }
			if (MonCmd[1] == 'c') {	Check_Kin(pData,ArgLen);    	break;   }
			// if (MonCmd[1] == 'c') {	Check_Kin_Body(pData,ArgLen);    	break;   }
			if (MonCmd[1] == 'z') {	Move_Zero();			    	break;   }
			if (MonCmd[1] == 'b') {	Move_Zero_BODY();		    	break;   }
		  if (MonCmd[1] == 'l') {	Move_Both_LEG(pData,ArgLen);	break;   }
		  if (MonCmd[1] == 'w') {	Move_COG_Walking(pData,ArgLen);	break;   }
			goto DispUsage;
		case 'r':
			if (MonCmd[1] == 'p') {	Get_Position(pData, ArgLen);	break;   }
			if (MonCmd[1] == 'h') {	Get_Hex_Data(pData, ArgLen);	break;   }
			if (MonCmd[1] == 'c') {	Get_COG_Data();	break;   }
			goto DispUsage;
		case 'h':
			_printF(F("[Monitor Command List]\r\n\r\n"
					"cc  -> Check COG (cc [X pos][Y pos][Z pos])\r\n"
					"cp  -> Check receive Packet \r\n"
					"cd  -> Check receive data Dump \r\n"
					"cf  -> Check timer interrupt Frequency \r\n"
					"wt  -> Write Torque Enable to AX-12 register (wt [ID][0|1]) \r\n"
					"wl  -> Write Led status to AX-12 register (wl [ID][0|1]) \r\n"
					"wc  -> Write Compliance slope to AX-12 register (wc [ID][SlopeValue]) \r\n"
					"rp  -> Read current Position (rp [ID])	\r\n"
					"rh  -> Read AX-12 Hex_parameter (rh [ID])	\r\n"
					"rc  -> Read COG (rc)	\r\n"
					"mv  -> MoVe AX-12 to POSITION (mv [ID][POSITION][SPEED])  \r\n"
					"mf  -> Move Forward kinematics (mf [J1][J2][J3][J4][J5][J6]) \r\n"
					"mi  -> Move Inverse kinematics (mf [X pos][Y pos][Z pos]) \r\n"
					"mc  -> Moving Check (Forward->Inverse) (mc [J2][J3][J4]) \r\n"
					"ml  -> Moving both LEG (ml [P_Rx][P_Ry][P_Rz][P_Lx][P_Ly][P_Lz]) \r\n"
					"mw  -> Moving by COG control (mw [t1][t2][xh][yf][yr][zp(zn)])\r\n"
					"ESC -> Exit Program \r\n"
					"\r\n"));
			break;
		default:
			DispUsage:
			break;
	}
	_printF(F(">"));
}

void Get_Position(char* pArgument, int ArgLen)
{
	char* pData;
	int Len_Sum,j;
	int ID, Result;
	unsigned char TxData[BUFFER_LENGTH];
	unsigned char pRxData[BUFFER_LENGTH];

	Len_Sum = 0;

	// ID
	if (Len_Sum > ArgLen)
	{
		_printF(F("Not enough parameter. \r\n"));
		goto Exit;
	}

	pData = strtok(pArgument," ");
	ID = atoi(pData);

	if ((ID < 0x00) || (ID > 0xFF))
	{
		_printF(F("Out of range : ID = 0 ~ 255 \r\n"));
		goto Exit;
	}

	// Make Packet
	// [FF][FF][ID][LEN][CMD][30][P_L][P_H][S_L][S_H][CHECKSUM]
	// Header
	TxData[0] = TxData[1] = 0xFF;
	TxData[2] = ID;
	TxData[3] = 4;
	TxData[4] = INST_READ;
	TxData[5] = P_PRESENT_POSITION_L;
	TxData[6] = 2;
	TxData[7] = CalCheckSum(TxData,7);

	// Data �۽�.
	SendHexStr(TxData, 7);		// 0 ~ i���� ������ �۽�

	delay(10);					// 10ms delay
	// ������ ����
	Result = RX_data_check(pRxData);

	for (j=0;j<Result;j++)
	{
		_printF(F("[%02X]"),pRxData[j]);
	}
	_printF(F("\r\n"));
	_printF(F("Current Position = %d \r\n"),pRxData[5]+(pRxData[6]<<8));

Exit:
	return;
}

void Get_Hex_Data(char* pArgument, int ArgLen)
{
	char* pData;
	int Len_Sum,j;
	int ID, Addr, Result;
	unsigned char TxData[BUFFER_LENGTH];
	unsigned char pRxData[BUFFER_LENGTH];

	Len_Sum = 0;

	// ID
	if (Len_Sum > ArgLen)
	{
		_printF(F("Not enough parameter. \r\n"));
		goto Exit;
	}

	pData = strtok(pArgument," ");
	ID = atoi(pData);

	if ((ID < 0x00) || (ID > 0xFF))
	{
		_printF(F("Out of range : ID = 0 ~ 255 \r\n"));
		goto Exit;
	}

	Len_Sum = (int)(Len_Sum + strlen(pData) + (pData-pArgument) + 1);
	pArgument = pData + strlen(pData) + 1;

	// Addr
	if (Len_Sum > ArgLen)
	{
		_printF(F("Not enough parameter. \r\n"));
		goto Exit;
	}

	pData = strtok(pArgument," ");
	Addr = atoi(pData);

	if ((Addr < 0) || (Addr > 49))
	{
		_printF(F("Out of range : Addr = 0 ~ 49 \r\n"));
		goto Exit;
	}

	// Make Packet
	// Header
	TxData[0] = TxData[1] = 0xFF;
	TxData[2] = ID;
	TxData[3] = 4;
	TxData[4] = INST_READ;
	TxData[5] = Addr;
	TxData[6] = 2;
	TxData[7] = CalCheckSum(TxData,7);

	// Data �۽�.
	SendHexStr(TxData, 7);		// 0 ~ i���� ������ �۽�

	delay(10);					// 10ms delay
	// ������ ����
	Result = RX_data_check(pRxData);

	for (j=0;j<Result;j++)
	{
		_printF(F("[%02X]"),pRxData[j]);
	}
	_printF(F("\r\n"));
	_printF(F("Value = %d \r\n"),pRxData[5]+(pRxData[6]<<8));

Exit:
	return;
}

// �ٸ�
// ���ⱸ�п� ���� ��ġ�� �̵�
void Move_For_Kin(char* pArgument,int ArgLen)
{
	char* pData;
	int Len_Sum,i;
	int speed_ax[2][6], position_ax[2][6];
	double theta[6];		// degree
	double mtx[3][4];
	double abgxyz[6];

	Len_Sum = 0;

	////////////////////////////////////
	// �� ������ ��ǥ ����(deg) ����.
	// ���ⱸ��(?)
	////////////////////////////////////
	// theta 0 ... 6
	for (i=0;i<6;i++)	// 0 ... 5
	{
		if (Len_Sum > ArgLen)
		{
			_printF(F("Not enough parameter. \r\n"));
			goto Exit;
		}

		pData = strtok(pArgument," ");
		theta[i] = atof(pData);

		Len_Sum = (int)(Len_Sum + strlen(pData) + (pData-pArgument) + 1);
		pArgument = pData + strlen(pData) + 1;
	}

	// ȸ����.
	_printF(F("Angle = "));
	for (i=0;i<6;i++)	_printF(F("[%9.4f]"),theta[i]);
	_printF(F("\r\n"));

	// ���ⱸ��.
	T06_for_M2(theta, mtx);

	// ItoU
	ItouXyz(mtx, abgxyz);
	_printF(F("U xyz = "));
	for (i=0;i<3;i++)	_printF(F("[%9.4f]"),abgxyz[i]*RADTODEG);
	for (i=3;i<6;i++)	_printF(F("[%9.4f]"),abgxyz[i]);
	_printF(F("\r\n"));

	// ������ �ٸ�
	// ��ġ�� ��ȯ.
	LEG_D2P(theta, position_ax[RIGHT_LEG], RIGHT_LEG);

	// AX-12�� ��ġ
	_printF(F("R_LEG = "));
	for (i=0;i<6;i++)	_printF(F("[%9d]"),position_ax[RIGHT_LEG][i]);
	_printF(F("\r\n"));

	// ���� �ٸ�
	// ��ġ�� ��ȯ.
	LEG_D2P(theta, position_ax[LEFT_LEG], LEFT_LEG);

	// AX-12�� ��ġ
	_printF(F("L_LEG = "));
	for (i=0;i<6;i++)	_printF(F("[%9d]"),position_ax[LEFT_LEG][i]);
	_printF(F("\r\n"));

	// Ȯ��.
	_printF(F("Run AX-12, press 'g' \r\n"));
	// while(kbhit());
  if (getSerialChar() != 'g') goto Exit;    //	if (getch() != 'g') goto Exit;

	// Make & Send Packet
	Cal_LEG_Speed(position_ax, speed_ax, NORMAL);
	Sync_Write_LEG(position_ax, speed_ax);

Exit:
	_printF(F("\r\n"));
    return;
}

// ��ü
// ���ⱸ�п� ���� ��ġ�� �̵�
void Move_For_Kin_BODY(char* pArgument,int ArgLen)
{
	char* pData;
	int Len_Sum,i;
	int speed_ax[2][4], position_ax[2][4];
	double theta[4];		// degree
	double mtx[3][4];
	double abgxyz[6];

	Len_Sum = 0;

	////////////////////////////////////
	// �� ������ ��ǥ ����(deg) ����.
	// (�㸮, ��������)
	// ���ⱸ��(?)
	////////////////////////////////////
	// theta 0 ... 3
	for (i=0;i<4;i++)	// 0 ... 3
	{
		if (Len_Sum > ArgLen)
		{
			_printF(F("Not enough parameter. \r\n"));
			goto Exit;
		}

		pData = strtok(pArgument," ");
		theta[i] = atof(pData);

		Len_Sum = (int)(Len_Sum + strlen(pData) + (pData-pArgument) + 1);
		pArgument = pData + strlen(pData) + 1;
	}

	// ȸ����.
	_printF(F("Angle = "));
	for (i=0;i<4;i++)	_printF(F("[%9.4f]"),theta[i]);
	_printF(F("\r\n"));

	// ���ⱸ��.
	T05_for_M2_BODY(theta, mtx);

	// ItoU
	ItouXyz(mtx, abgxyz);
	_printF(F("U xyz = "));
	for (i=0;i<3;i++)	_printF(F("[%9.4f]"),abgxyz[i]*RADTODEG);
	for (i=3;i<6;i++)	_printF(F("[%9.4f]"),abgxyz[i]);
	_printF(F("\r\n"));

	// �㸮�� ������ ��
	// ��ġ�� ��ȯ.
	ARM_D2P(theta, position_ax[RIGHT], RIGHT);

	// AX-12�� ��ġ
	_printF(F("R_ARM = "));
	for (i=0;i<4;i++)	_printF(F("[%9d]"),position_ax[RIGHT][i]);
	_printF(F("\r\n"));

	////////////////////////////////////
	// �� ������ ��ǥ ����(deg) ����.
	// (������), -> �㸮(theta[0])�� �������� ���� ������.
	// ���ⱸ��(?)
	////////////////////////////////////
	// theta 1 ... 3
	for (i=1;i<4;i++)	// 1 ... 3
	{
		if (Len_Sum > ArgLen)
		{
			_printF(F("Not enough parameter. \r\n"));
			goto Exit;
		}

		pData = strtok(pArgument," ");
		theta[i] = atof(pData);

		Len_Sum = (int)(Len_Sum + strlen(pData) + (pData-pArgument) + 1);
		pArgument = pData + strlen(pData) + 1;
	}

	// ȸ����.
	_printF(F("Angle = "));
	for (i=0;i<4;i++)	_printF(F("[%9.4f]"),theta[i]);
	_printF(F("\r\n"));

	// ���ⱸ��.
	T05_for_M2_BODY(theta, mtx);

	// ItoU
	ItouXyz(mtx, abgxyz);
	_printF(F("U xyz = "));
	for (i=0;i<3;i++)	_printF(F("[%9.4f]"),abgxyz[i]*RADTODEG);
	for (i=3;i<6;i++)	_printF(F("[%9.4f]"),abgxyz[i]);
	_printF(F("\r\n"));

	// ���� ��
	// ��ġ�� ��ȯ.
	ARM_D2P(theta, position_ax[LEFT], LEFT);

	// AX-12�� ��ġ
	_printF(F("L_ARM = "));
	for (i=0;i<4;i++)	_printF(F("[%9d]"),position_ax[LEFT][i]);
	_printF(F("\r\n"));

	// Ȯ��.
	_printF(F("Run AX-12, press 'g' \r\n"));
  //	while(kbhit());
  if (getSerialChar() != 'g') goto Exit;    //	if (getch() != 'g') goto Exit;

	// Make & Send Packet

	// JHK ����!!!!!!!!!!
	Cal_ARM_Speed(position_ax, speed_ax, NORMAL);
	Sync_Write_BODY(position_ax, speed_ax);

Exit:
	_printF(F("\r\n"));
    return;
}

// ���ⱸ�п� ���� ��ġ�� �̵�
void Move_Inv_Kin(char* pArgument,int ArgLen)
{
	char* pData;
	char str[2];
	int Len_Sum,i;
	unsigned int state_to_run;
	double theta[6];		// degree
	double mtx[3][4] = {{0,0,0,0},{0,0,0,0},{0,0,0,0}};

	int speed_ax[2][6], position_ax[2][6];
	int sol_cnt, state[MAX_SOL];
	double jt_deg[MAX_SOL][6];

	Len_Sum = 0;

	////////////////////////////////////
	// X,Y,Z ���� ��ǥ ��ġ�� ����.
	// ���ⱸ��
	////////////////////////////////////
	for (i=0;i<3;i++)	// 0 ... 2
	{
		if (Len_Sum > ArgLen)
		{
			_printF(F("Not enough parameter. \r\n"));
			goto Exit;
		}

		pData = strtok(pArgument," ");

		// ����) mtx�� 	mtx[0][3](x_pos), mtx[1][3](y_pos), mtx[2][3](z_pos)�� �����.
		mtx[i][3] = atof(pData);

		Len_Sum = (int)(Len_Sum + strlen(pData) + (pData-pArgument) + 1);
		pArgument = pData + strlen(pData) + 1;
	}

	// ���ⱸ���� ���.
	sol_cnt = Inverse_Kin_M2(mtx, jt_deg, state);

	// ���� ���ⱸ���� Ȯ��.
	if (sol_cnt != 0)
	{
		_printF(F("INV_T06 \r\n"));
		for (i=0;i<MAX_SOL;i++)
		{
			if (state[i] == 1)
			{
				_printF(F("%d : %9.4lf,%9.4lf,%9.4lf,%9.4lf,%9.4lf,%9.4lf\r\n"),
			           i,jt_deg[i][0],jt_deg[i][1],jt_deg[i][2],jt_deg[i][3],jt_deg[i][4],jt_deg[i][5]);
			}
		}
		// Ȯ��.
		_printF(F("Run AX-12, press '0 ~ 3' \r\n"));
	  //	while(kbhit());
  	str[0] = getSerialChar();  		// str[0] = getch();
	  str[1] = _NULL;
		state_to_run = atoi(str);

		if ((state_to_run < MAX_SOL) && (state[state_to_run] == 1))
		{
			for (i=0;i<6;i++)	theta[i] = jt_deg[state_to_run][i];

			// ������ �ٸ�
			// ��ġ�� ��ȯ.
			LEG_D2P(theta, position_ax[RIGHT_LEG], RIGHT_LEG);

			// AX-12�� ��ġ
			_printF(F("R_LEG = "));
			for (i=0;i<6;i++)	_printF(F("[%9d]"),position_ax[RIGHT_LEG][i]);
			_printF(F("\r\n"));

			// ���� �ٸ�
			// ��ġ�� ��ȯ.
			LEG_D2P(theta, position_ax[LEFT_LEG], LEFT_LEG);

			// AX-12�� ��ġ
			_printF(F("L_LEG = "));
			for (i=0;i<6;i++)	_printF(F("[%9d]"),position_ax[LEFT_LEG][i]);
			_printF(F("\r\n"));

			// Ȯ��.
			_printF(F("Run AX-12, press 'g' \r\n"));
			// while(kbhit());
			if (getSerialChar() != 'g') goto Exit;  			// if (getch() != 'g') goto Exit;

			// Make & Send Packet
			Cal_LEG_Speed(position_ax, speed_ax, NORMAL);
			Sync_Write_LEG(position_ax, speed_ax);
		}
	}
Exit:
	return;
}

// ��ü�� �����߽��� Ȯ��
void Check_All_Torque(char* pArgument,int ArgLen)
{
	char* pData;
	// char str[2];
	int Len_Sum,i;
	unsigned int state_to_run;
	double theta[2][6], theta_waist, arm_theta[2][4];		// degree
	double mtx[3][4] = {{0,0,0,0},{0,0,0,0},{0,0,0,0}};
	double T06[3][4];
	// double position[2][4][3];

	int position_ax[2][6];
	// int speed_ax[2][6];
	int sol_cnt, state[MAX_SOL];
	double jt_deg[MAX_SOL][6];

	double Pcog[3], Torque[3];
	Len_Sum = 0;

	////////////////////////////////////
	// X,Y,Z ���� ��ǥ ��ġ�� ����.
	// ���ⱸ��
	////////////////////////////////////
	/*****************************************/
	// ������
	/*****************************************/
	// ����) mtx�� 	mtx[0][3](x_pos), mtx[1][3](y_pos), mtx[2][3](z_pos)�� �����.
	//       �׷���... T06�� ����ϱ� ���� RBF�� �ʿ�.
	//       RBF�� �׻� �����Ǿ� ����.
	// ���� : �߽ɿ��� �߳��� ȸ�� Matrix
	Get_R_C_F(mtx);

	// ���� ��ġ(����(BC)�� �������� �� �߳�(F)�� ��ġ)
	for (i=0;i<3;i++)	// 0 ... 2
	{
		if (Len_Sum > ArgLen)
		{
			_printF(F("Not enough parameter. \r\n"));
			goto Exit;
		}

		pData = strtok(pArgument," ");

		// ����) mtx�� 	mtx[0][3](x_pos), mtx[1][3](y_pos), mtx[2][3](z_pos)�� �����.
		mtx[i][3] = atof(pData);

		Len_Sum = (int)(Len_Sum + strlen(pData) + (pData-pArgument) + 1);
		pArgument = pData + strlen(pData) + 1;
	}

	// {BC}{F}T -> {0}{6}T�� ����.
	Cal_LEG_BASE(mtx, T06, RIGHT_LEG);

	// ���ⱸ���� ���.
	sol_cnt = Inverse_Kin_M2(T06, jt_deg, state);

	// ���� ���ⱸ���� Ȯ��.
	if (sol_cnt != 0)
	{
		/*
		_printF(F("INV_T06_RIGHT \r\n"));
		for (i=0;i<MAX_SOL;i++)
		{
			if (state[i] == 1)
			{
				_printF(F("%d : %9.4lf,%9.4lf,%9.4lf,%9.4lf,%9.4lf,%9.4lf \r\n"),
			           i,jt_deg[i][0],jt_deg[i][1],jt_deg[i][2],jt_deg[i][3],jt_deg[i][4],jt_deg[i][5]);
			}
		}
		*/
		// Ȯ��.
		// _printF(F("Run AX-12 for RIGHT_LEG, press '0 ~ 3' \r\n"));
		// while(kbhit());
		// str[0] = getch();
		// str[1] = NULL;
		// state_to_run = atoi(str);
		state_to_run = 0;
		if ((state_to_run < MAX_SOL) && (state[state_to_run] == 1))
		{
			for (i=0;i<6;i++)	theta[RIGHT_LEG][i] = jt_deg[state_to_run][i];

			// ������ �ٸ�		// ��ġ�� ��ȯ.
			LEG_D2P(theta[RIGHT_LEG], position_ax[RIGHT_LEG], RIGHT_LEG);
		}
	}
	else
	{
		_printF(F("Right LEG does not have solution \r\n"));
		goto Exit;
	}

	/*****************************************/
	// ������
	/*****************************************/
	// ����) mtx�� 	mtx[0][3](x_pos), mtx[1][3](y_pos), mtx[2][3](z_pos)�� �����.
	//       �׷���... T06�� ����ϱ� ���� RBF�� �ʿ�.
	//       RBF�� �׻� �����Ǿ� ����.

	// ���� : �߽ɿ��� �߳��� ȸ�� Matrix
	Get_R_C_F(mtx);

	// ��ġ
	for (i=0;i<3;i++)	// 0 ... 2
	{
		if (Len_Sum > ArgLen)
		{
			_printF(F("Not enough parameter. \r\n"));
			goto Exit;
		}
		pData = strtok(pArgument," ");

		mtx[i][3] = atof(pData);

		Len_Sum = (int)(Len_Sum + strlen(pData) + (pData-pArgument) + 1);
		pArgument = pData + strlen(pData) + 1;
	}

	// {BC}{F}T -> {0}{6}T�� ����.
	Cal_LEG_BASE(mtx, T06, LEFT_LEG);

	// ���ⱸ���� ���.
	sol_cnt = Inverse_Kin_M2(T06, jt_deg, state);

	// ���� ���ⱸ���� Ȯ��.
	if (sol_cnt != 0)
	{
		/*
		_printF(F("INV_T06_LEFT \r\n"));
		for (i=0;i<MAX_SOL;i++)
		{
			if (state[i] == 1)
			{
				_printF(F("%d : %9.4lf,%9.4lf,%9.4lf,%9.4lf,%9.4lf,%9.4lf \r\n"),
			           i,jt_deg[i][0],jt_deg[i][1],jt_deg[i][2],jt_deg[i][3],jt_deg[i][4],jt_deg[i][5]);
			}
		}
		*/
		// Ȯ��.
		// _printF(F("Run AX-12 for LEFT LEG, press '0 ~ 3' \r\n"));
		// while(kbhit());
		// str[0] = getch();
		// str[1] = NULL;
		// state_to_run = atoi(str);
		state_to_run = 0;
		if ((state_to_run < MAX_SOL) && (state[state_to_run] == 1))
		{
			for (i=0;i<6;i++)	theta[LEFT_LEG][i] = jt_deg[state_to_run][i];

			// ���� �ٸ�	// ��ġ�� ��ȯ.
			LEG_D2P(theta[LEFT_LEG], position_ax[LEFT_LEG], LEFT_LEG);
		}
	}
	else
	{
		_printF(F("Left LEG does not have solution \r\n"));
		goto Exit;
	}

	/*****************************************/
	// ����� ����
	/*****************************************/

	// �㸮�� ����
	if (Len_Sum > ArgLen)
	{
		_printF(F("Not enough parameter. \r\n"));
		goto Exit;
	}

	pData = strtok(pArgument," ");

	theta_waist = atof(pData);

	Len_Sum = (int)(Len_Sum + strlen(pData) + (pData-pArgument) + 1);
	pArgument = pData + strlen(pData) + 1;

	// ������ ���� ����
	for (i=1;i<4;i++)	// 0 ... 2
	{
		if (Len_Sum > ArgLen)
		{
			_printF(F("Not enough parameter. \r\n"));
			goto Exit;
		}

		pData = strtok(pArgument," ");

		// ����) mtx�� 	mtx[0][3](x_pos), mtx[1][3](y_pos), mtx[2][3](z_pos)�� �����.
		arm_theta[RIGHT][i] = atof(pData);

		Len_Sum = (int)(Len_Sum + strlen(pData) + (pData-pArgument) + 1);
		pArgument = pData + strlen(pData) + 1;
	}

	// ���� ���� ����
	for (i=1;i<4;i++)	// 0 ... 2
	{
		if (Len_Sum > ArgLen)
		{
			_printF(F("Not enough parameter. \r\n"));
			goto Exit;
		}

		pData = strtok(pArgument," ");

		// ����) mtx�� 	mtx[0][3](x_pos), mtx[1][3](y_pos), mtx[2][3](z_pos)�� �����.
		arm_theta[LEFT][i] = atof(pData);

		Len_Sum = (int)(Len_Sum + strlen(pData) + (pData-pArgument) + 1);
		pArgument = pData + strlen(pData) + 1;
	}

	arm_theta[RIGHT][0] = arm_theta[LEFT][0] = theta_waist;

	// �����߽� Ȯ��.

	// ���⼭���� ���� ���. JHK
	Cal_Torque_ALL(arm_theta, theta, Torque, Pcog, RIGHT_LEG);
	_printF(F("COG : %6.1f, %6.1f, %6.1f \r\n"),Pcog[0], Pcog[1], Pcog[2]);
Exit:
	return;
}

// ��ü�� �����߽��� Ȯ��
void Check_BODY_Torque(char* pArgument,int ArgLen)
{
	char* pData;
	// char str[2];
	int Len_Sum,i;
	double theta_waist, arm_theta[2][4];		// degree

	double Pcog[3], Torque[3];
	Len_Sum = 0;

	////////////////////////////////////
	// X,Y,Z ���� ��ǥ ��ġ�� ����.
	////////////////////////////////////

	/*****************************************/
	// ����� ����
	/*****************************************/

	// �㸮�� ����
	if (Len_Sum > ArgLen)
	{
		_printF(F("Not enough parameter. \r\n"));
		goto Exit;
	}

	pData = strtok(pArgument," ");

	theta_waist = atof(pData);

	Len_Sum = (int)(Len_Sum + strlen(pData) + (pData-pArgument) + 1);
	pArgument = pData + strlen(pData) + 1;

	// ������ ���� ����
	for (i=1;i<4;i++)	// 0 ... 2
	{
		if (Len_Sum > ArgLen)
		{
			_printF(F("Not enough parameter. \r\n"));
			goto Exit;
		}

		pData = strtok(pArgument," ");

		// ����) mtx�� 	mtx[0][3](x_pos), mtx[1][3](y_pos), mtx[2][3](z_pos)�� �����.
		arm_theta[RIGHT][i] = atof(pData);

		Len_Sum = (int)(Len_Sum + strlen(pData) + (pData-pArgument) + 1);
		pArgument = pData + strlen(pData) + 1;
	}

	// ���� ���� ����
	for (i=1;i<4;i++)	// 0 ... 2
	{
		if (Len_Sum > ArgLen)
		{
			_printF(F("Not enough parameter. \r\n"));
			goto Exit;
		}

		pData = strtok(pArgument," ");

		// ����) mtx�� 	mtx[0][3](x_pos), mtx[1][3](y_pos), mtx[2][3](z_pos)�� �����.
		arm_theta[LEFT][i] = atof(pData);

		Len_Sum = (int)(Len_Sum + strlen(pData) + (pData-pArgument) + 1);
		pArgument = pData + strlen(pData) + 1;
	}

	arm_theta[RIGHT][0] = arm_theta[LEFT][0] = theta_waist;

	// �����߽� Ȯ��.
	Cal_Torque_BODY(arm_theta, Torque, Pcog);
	_printF(F("COG : %6.1f, %6.1f, %6.1f \r\n"),Pcog[0], Pcog[1], Pcog[2]);

Exit:
	return;
}

// ���� ���� �̵�
void Move_Both_LEG(char* pArgument,int ArgLen)
{
	char* pData;
	// char str[2];
	int Len_Sum,i;
	unsigned int state_to_run;
	double theta[2][6];		// degree
	double mtx[3][4] = {{0,0,0,0},{0,0,0,0},{0,0,0,0}};
	double T06[3][4];
	double position[2][4][3];

	int speed_ax[2][6], position_ax[2][6];
	int sol_cnt, state[MAX_SOL];
	double jt_deg[MAX_SOL][6];

	double Pcog[3], Torque[3];
	Len_Sum = 0;

	////////////////////////////////////
	// X,Y,Z ���� ��ǥ ��ġ�� ����.
	// ���ⱸ��
	////////////////////////////////////
	/*****************************************/
	// ������
	/*****************************************/
	// ����) mtx�� 	mtx[0][3](x_pos), mtx[1][3](y_pos), mtx[2][3](z_pos)�� �����.
	//       �׷���... T06�� ����ϱ� ���� RBF�� �ʿ�.
	//       RBF�� �׻� �����Ǿ� ����.
	// ���� : �߽ɿ��� �߳��� ȸ�� Matrix
	Get_R_C_F(mtx);

	// ��ġ
	for (i=0;i<3;i++)	// 0 ... 2
	{
		if (Len_Sum > ArgLen)
		{
			_printF(F("Not enough parameter. \r\n"));
			goto Exit;
		}

		pData = strtok(pArgument," ");

		// ����) mtx�� 	mtx[0][3](x_pos), mtx[1][3](y_pos), mtx[2][3](z_pos)�� �����.
		mtx[i][3] = atof(pData);

		Len_Sum = (int)(Len_Sum + strlen(pData) + (pData-pArgument) + 1);
		pArgument = pData + strlen(pData) + 1;
	}

	// {BC}{F}T -> {0}{6}T�� ����.
	Cal_LEG_BASE(mtx, T06, RIGHT_LEG);

	// ���ⱸ���� ���.
	sol_cnt = Inverse_Kin_M2(T06, jt_deg, state);

	// ���� ���ⱸ���� Ȯ��.
	if (sol_cnt != 0)
	{
		_printF(F("INV_T06_RIGHT \r\n"));
		for (i=0;i<MAX_SOL;i++)
		{
			if (state[i] == 1)
			{
				_printF(F("%d : %9.4lf,%9.4lf,%9.4lf,%9.4lf,%9.4lf,%9.4lf \r\n"),
			           i,jt_deg[i][0],jt_deg[i][1],jt_deg[i][2],jt_deg[i][3],jt_deg[i][4],jt_deg[i][5]);
			}
		}
		// Ȯ��.
		// _printF(F("Run AX-12 for RIGHT_LEG, press '0 ~ 3' \r\n"));
		// while(kbhit());
		// str[0] = getch();
		// str[1] = NULL;
		// state_to_run = atoi(str);
		state_to_run = 0;
		if ((state_to_run < MAX_SOL) && (state[state_to_run] == 1))
		{
			for (i=0;i<6;i++)	theta[RIGHT_LEG][i] = jt_deg[state_to_run][i];

			// ������ �ٸ�		// ��ġ�� ��ȯ.
			LEG_D2P(theta[RIGHT_LEG], position_ax[RIGHT_LEG], RIGHT_LEG);
		}
	}
	else
	{
		_printF(F("Right LEG does not have solution \r\n"));
		goto Exit;
	}

	/*****************************************/
	// ������
	/*****************************************/
	// ����) mtx�� 	mtx[0][3](x_pos), mtx[1][3](y_pos), mtx[2][3](z_pos)�� �����.
	//       �׷���... T06�� ����ϱ� ���� RBF�� �ʿ�.
	//       RBF�� �׻� �����Ǿ� ����.

	// ���� : �߽ɿ��� �߳��� ȸ�� Matrix
	Get_R_C_F(mtx);

	// ��ġ
	for (i=0;i<3;i++)	// 0 ... 2
	{
		if (Len_Sum > ArgLen)
		{
			_printF(F("Not enough parameter. \r\n"));
			goto Exit;
		}
		pData = strtok(pArgument," ");

		mtx[i][3] = atof(pData);

		Len_Sum = (int)(Len_Sum + strlen(pData) + (pData-pArgument) + 1);
		pArgument = pData + strlen(pData) + 1;
	}

	// {BC}{F}T -> {0}{6}T�� ����.
	Cal_LEG_BASE(mtx, T06, LEFT_LEG);

	// ���ⱸ���� ���.
	sol_cnt = Inverse_Kin_M2(T06, jt_deg, state);

	// ���� ���ⱸ���� Ȯ��.
	if (sol_cnt != 0)
	{
		_printF(F("INV_T06_LEFT \r\n"));
		for (i=0;i<MAX_SOL;i++)
		{
			if (state[i] == 1)
			{
				_printF(F("%d : %9.4lf,%9.4lf,%9.4lf,%9.4lf,%9.4lf,%9.4lf \r\n"),
			           i,jt_deg[i][0],jt_deg[i][1],jt_deg[i][2],jt_deg[i][3],jt_deg[i][4],jt_deg[i][5]);
			}
		}
		// Ȯ��.
		// _printF(F("Run AX-12 for LEFT LEG, press '0 ~ 3' \r\n"));
		// while(kbhit());
		// str[0] = getch();
		// str[1] = NULL;a
		// state_to_run = atoi(str);
		state_to_run = 0;
		if ((state_to_run < MAX_SOL) && (state[state_to_run] == 1))
		{
			for (i=0;i<6;i++)	theta[LEFT_LEG][i] = jt_deg[state_to_run][i];

			// ���� �ٸ�	// ��ġ�� ��ȯ.
			LEG_D2P(theta[LEFT_LEG], position_ax[LEFT_LEG], LEFT_LEG);
		}
	}
	else
	{
		_printF(F("Left LEG does not have solution \r\n"));
		goto Exit;
	}
	// �����߽� Ȯ��.
	Cal_Torque(theta, Torque, Pcog, LEFT_LEG);
	_printF(F("COG : %6.1f, %6.1f, %6.1f \r\n"),Pcog[0], Pcog[1], Pcog[2]);

	// �ٸ��� �� ��ǥ���� ���� ��ġ Ȯ��
	Cal_LEG_Position(theta, position);

	_printF(F("R0 : %6.1f, %6.1f, %6.1f\r\n"),
		position[RIGHT_LEG][P_BC_0][PX],position[RIGHT_LEG][P_BC_0][PY],position[RIGHT_LEG][P_BC_0][PZ]);
	_printF(F("L0 : %6.1f, %6.1f, %6.1f\r\n"),
		position[LEFT_LEG][P_BC_0][PX] ,position[LEFT_LEG][P_BC_0][PY], position[LEFT_LEG][P_BC_0][PZ]);

	_printF(F("R4 : %6.1f, %6.1f, %6.1f\r\n"),
		position[RIGHT_LEG][P_BC_4][PX],position[RIGHT_LEG][P_BC_4][PY],position[RIGHT_LEG][P_BC_4][PZ]);
	_printF(F("L4 : %6.1f, %6.1f, %6.1f\r\n"),
		position[LEFT_LEG][P_BC_4][PX] ,position[LEFT_LEG][P_BC_4][PY], position[LEFT_LEG][P_BC_4][PZ]);

	_printF(F("R6 : %6.1f, %6.1f, %6.1f\r\n"),
		position[RIGHT_LEG][P_BC_6][PX],position[RIGHT_LEG][P_BC_6][PY],position[RIGHT_LEG][P_BC_6][PZ]);
	_printF(F("L6 : %6.1f, %6.1f, %6.1f\r\n"),
		position[LEFT_LEG][P_BC_6][PX] ,position[LEFT_LEG][P_BC_6][PY], position[LEFT_LEG][P_BC_6][PZ]);

	_printF(F("RF : %6.1f, %6.1f, %6.1f\r\n"),
		position[RIGHT_LEG][P_BC_F][PX],position[RIGHT_LEG][P_BC_F][PY],position[RIGHT_LEG][P_BC_F][PZ]);
	_printF(F("LF : %6.1f, %6.1f, %6.1f\r\n"),
		position[LEFT_LEG][P_BC_F][PX] ,position[LEFT_LEG][P_BC_F][PY], position[LEFT_LEG][P_BC_F][PZ]);

	// Ȯ��.
	_printF(F("Run AX-12, press 'g' \r\n"));
	// while(kbhit());
	if (getSerialChar() != 'g') goto Exit;  // if (getch() != 'g') goto Exit;
	// Make & Send Packet
	Cal_LEG_Speed(position_ax, speed_ax, NORMAL);
	Sync_Write_LEG(position_ax, speed_ax);

	_printF(F("%4d, %4d, %4d, %4d, %4d, %4d\r\n"),
				position_ax[RIGHT_LEG][0], position_ax[RIGHT_LEG][1], position_ax[RIGHT_LEG][2],
				position_ax[RIGHT_LEG][3], position_ax[RIGHT_LEG][4], position_ax[RIGHT_LEG][5]);

	_printF(F("%4d, %4d, %4d, %4d, %4d, %4d\r\n"),
				speed_ax[RIGHT_LEG][0], speed_ax[RIGHT_LEG][1], speed_ax[RIGHT_LEG][2],
				speed_ax[RIGHT_LEG][3], speed_ax[RIGHT_LEG][4], speed_ax[RIGHT_LEG][5]);
Exit:
	return;
}


#define REPETITION_COUNT	4
#define Z_COMP_VALUE 		0

// COG ��� ���� ����.
void Move_COG_Walking(char* pArgument,int ArgLen)
{
	char* pData;
	int Len_Sum,i, j, k;
	unsigned int state_to_run;
	double theta[2][6];		// degree
	double theta_ARM[2][4];		// degree
	double mtx[3][4] = {{0,0,0,0},{0,0,0,0},{0,0,0,0}};
	double T06[3][4];

	int t, t1, t2, t1x, t2x, t1xm, t1y, t2y, t1z, TE;
	double xh, yf, yr, zp, zn;
	double x_Up_Speed, x_Dn_Speed, y_Speed, z_Speed;
	double x[2], y[2], z[2];
	double x_comp[2], y_comp[2], z_comp[2];

	int Arg[6];
	int step;

	int speed_ax[2][6], position_ax[2][6];
	int speed_ax_ARM[2][4], position_ax_ARM[2][4];
	int sol_cnt, state[MAX_SOL];
	double jt_deg[MAX_SOL][6];
	double best_sol[6];

	double Pcog[3], Torque[3];
	
	File  fp_out;
	int   FileNumber = 0;
	char  FileName[] = "LOG_00.dat";
	
	Len_Sum = 0;
	x_comp[RIGHT_LEG] = x_comp[LEFT_LEG] = 0;
	y_comp[RIGHT_LEG] = y_comp[LEFT_LEG] = 0;
	z_comp[RIGHT_LEG] = z_comp[LEFT_LEG] = 0;
  // (TBC)
  // SD Card�� File�� �����
	// FILE *fp_out;
  
  do
  {
    FileName[4] = FileNumber/10+'0';
    FileName[5] = FileNumber%10+'0';
	  FileNumber++;
	} while(SD.exists(FileName) && FileNumber < 100);
  
	// ���� ����
	// if((fp_out = fopen("COG_.dat","wt")) == NULL)
	// 	_printF(F("Can not open file COG_.dat\n"));
	fp_out = SD.open(FileName,FILE_WRITE);
	if (!fp_out)  _printF(F("File Open Error.\r\n")); // Serial.println("File Open Error.");
	
	//////////////////////////////
	// ���� ����
	/////////////////////////////
	for (i=0;i<6;i++)	// t1, t2, xh, yf, yr, zp(zn), 0 ... 5
	{
		if (Len_Sum > ArgLen)
		{
			_printF(F("Not enough parameter. \r\n"));
			goto Exit;
		}

		pData = strtok(pArgument," ");

		Arg[i] = atof(pData);

		Len_Sum = (int)(Len_Sum + strlen(pData) + (pData-pArgument) + 1);
		pArgument = pData + strlen(pData) + 1;
	}
	t1 = (int)Arg[0];
	t2 = (int)Arg[1];

	t1x = t1 + g_t1x_diff;
	t2x = t2 + g_t2x_diff;
	t1xm = (t1x+t2x)/2;

	t1y = t1 + g_t1y_diff;
	t2y = t2 + g_t2y_diff;

	t1z = t1 + g_t1z_diff;

	TE = t2 + g_TE_diff;

	xh = Arg[2];
	x_Up_Speed = xh / (t1xm - t1x);
	x_Dn_Speed = xh / (t2x - t1xm);

	yf = Arg[3];
	yr = Arg[4];
	y_Speed = (yf - yr) / (t2y - t1y);

	zp = Arg[5];
	zn = -Arg[5];
	z_Speed = (zp - zn) / (t1z);

	/////////////////////
	// ���� ���
	/////////////////////
	_fprintf(fp_out,"------------------------------------------------------------------------------------------------------------\r\n");
	_fprintf(fp_out,"--- Controlled by Center of Gravity ------------------------------------------------------------------------\r\n");
	_fprintf(fp_out,"------------------------------------------------------------------------------------------------------------\r\n");
	_fprintf(fp_out,"t1  = %4d, t2   = %4d\r\n",t1,t2);
	_fprintf(fp_out,"t1x = %4d, t1xm = %4d, t2x = %4d\r\n",t1x,t1xm,t2x);
	_fprintf(fp_out,"t1y = %4d, t2y  = %4d\r\n",t1y,t2y);
	_fprintf(fp_out,"t1z = %4d\r\n",t1z);
	_fprintf(fp_out,"------------------------------------------------------------------------------------------------------------\r\n");
	_fprintf(fp_out,"xh  = %5.1f\r\n",xh);
	_fprintf(fp_out,"yf  = %5.1f, yr = %5.1f\r\n",yf,yr);
	_fprintf(fp_out,"zp  = %5.1f, zn = %5.1f\r\n",zp,zn);
	_fprintf(fp_out,"------------------------------------------------------------------------------------------------------------\r\n");
	_fprintf(fp_out,"x_Up_Speed = %5.1f, x_Dn_Speed = %5.1f\r\n",x_Up_Speed, x_Dn_Speed);
	_fprintf(fp_out,"y_Speed    = %5.1f\r\n",y_Speed);
	_fprintf(fp_out,"z_Speed    = %5.1f\r\n",z_Speed);
	_fprintf(fp_out,"\r\n");
	_fprintf(fp_out,"------------------------------------------------------------------------------------------------------------\r\n");
  _fprintf(fp_out,"  tn,R_X_Pos,R_Y_Pos,R_Z_Pos,L_X_Pos,L_Y_Pos,L_Z_Pos,  COG_X,  COG_Y,  COG_Z, Torque_X, Torque_Y, Torque_Z \r\n");
	_fprintf(fp_out,"------------------------------------------------------------------------------------------------------------\r\n");

	//////////////////////////////
	// �ʱ� ��ġ�� �̵�.
	//////////////////////////////
	// t=0������ �ʱⰪ(���� ������ �����Ҷ�)
	step = LEFT_LEG;		// �����ϴ� ��.
	x[RIGHT_LEG] = X_BASE_POS;			x[LEFT_LEG] = X_BASE_POS;
	y[RIGHT_LEG] = yr;					y[LEFT_LEG] = yf;
	z[RIGHT_LEG] = zp + g_z_RIGHT_BASE;	z[LEFT_LEG] = zp + g_z_LEFT_BASE;

	// k = RIGHT_LEG(0), LEFT_LEG(1)
	for (k=0;k<2;k++)
	{
		Get_R_C_F(mtx);
		mtx[X][3] = x[k];
		mtx[Y][3] = y[k];
		mtx[Z][3] = z[k];

		// {BC}{F}T -> {0}{6}T�� ����.
		Cal_LEG_BASE(mtx, T06, k);

		// ���ⱸ���� ���.
		sol_cnt = Inverse_Kin_M2(T06, jt_deg, state);

		// state_to_run = ������(�����... �� ������ ������...)
		state_to_run = 0;
		if ((state_to_run < MAX_SOL) && (state[state_to_run] == 1))
		{
			for (i=0;i<6;i++)	theta[k][i] = jt_deg[state_to_run][i];
			LEG_D2P(theta[k], position_ax[k], k);
		}
		else
		{
			_printF(F("No Initial Solution for %s\r\n"),((k==RIGHT_LEG)?"RIGHT_LEG":"LEFT_LEG"));
			goto Exit;
		}
	}

	// Make & Send Packet
	Cal_LEG_Speed(position_ax, speed_ax, NORMAL);
	Sync_Write_LEG(position_ax, speed_ax);


	// �㸮�� ������ ��
	theta_ARM[RIGHT][0] = 0;
	theta_ARM[RIGHT][1] = 0;
	theta_ARM[RIGHT][2] = -20;
	theta_ARM[RIGHT][3] = 20;

	// ��ġ�� ��ȯ.
	ARM_D2P(theta_ARM[RIGHT], position_ax_ARM[RIGHT], RIGHT);


	// ���� ��
	theta_ARM[LEFT][0] = 0;
	theta_ARM[LEFT][1] = 0;
	theta_ARM[LEFT][2] = 20;
	theta_ARM[LEFT][3] = -20;

	// ��ġ�� ��ȯ.
	ARM_D2P(theta_ARM[LEFT], position_ax_ARM[LEFT], LEFT);

	// Make & Send Packet
	Cal_ARM_Speed(position_ax_ARM, speed_ax_ARM, NORMAL);
	Sync_Write_BODY(position_ax_ARM, speed_ax_ARM);

	///////////////////////////
	// �̵� ���� Ȯ��.
	///////////////////////////
	_printF(F("Run AX-12, press any key \r\n"));
  getSerialChar();    // getch();
	//////////////////////////////
	// t�� ���� ����(??msec���� ����)
	//////////////////////////////
	for (j = 0;j<REPETITION_COUNT;j++)
	{
		for (t=0;t<TE;t++)
		{
			if (g_Emer_Stop)	goto Exit;
		//////////////////////////////
		// t�� ���� �̵� ��ġ ���
		//////////////////////////////
			// X���� �̵�
			if (step == LEFT_LEG)
			{
				if (t<t1x)			{ x[RIGHT_LEG] = X_BASE_POS;	x[LEFT_LEG] = X_BASE_POS;}
				else if (t<t1xm)	{ x[RIGHT_LEG] += x_Up_Speed; 	x[LEFT_LEG] = X_BASE_POS;}
				else if (t<t2x)		{ x[RIGHT_LEG] -= x_Dn_Speed; 	x[LEFT_LEG] = X_BASE_POS;}
				else 				{ x[RIGHT_LEG] = X_BASE_POS;	x[LEFT_LEG] = X_BASE_POS;}
			}
			else
			{
				if (t<t1x)			{ x[RIGHT_LEG] = X_BASE_POS;	x[LEFT_LEG] = X_BASE_POS;}
				else if (t<t1xm)	{ x[RIGHT_LEG] = X_BASE_POS;	x[LEFT_LEG] += x_Up_Speed;}
				else if (t<t2x)		{ x[RIGHT_LEG] = X_BASE_POS;	x[LEFT_LEG] -= x_Dn_Speed;}
				else 				{ x[RIGHT_LEG] = X_BASE_POS;	x[LEFT_LEG] = X_BASE_POS;}
			}

			// Y�� ������ �̵�
			if (step == LEFT_LEG)
			{
				if (t<t1y)			{ y[RIGHT_LEG] = yr;			y[LEFT_LEG] = yf; }
				else if (t<t2y)		{ y[RIGHT_LEG] += y_Speed;		y[LEFT_LEG] -= y_Speed; }
				else				{ y[RIGHT_LEG] = yf;			y[LEFT_LEG] = yr; }
			}
			else
			{
				if (t<t1y)			{ y[RIGHT_LEG] = yf; 			y[LEFT_LEG] = yr; }
				else if (t<t2y)		{ y[RIGHT_LEG] -= y_Speed; 		y[LEFT_LEG] += y_Speed; }
				else				{ y[RIGHT_LEG] = yr; 			y[LEFT_LEG] = yf; }
			}

			// Z�� ������ �̵�
			if (step == LEFT_LEG)
			{
				// z0 = zp + g_z_RIGHT/LEFT_BASE;
				if (t<t1z)	{ z[RIGHT_LEG] -= z_Speed;				z[LEFT_LEG] -= z_Speed; }
				else 		{ z[RIGHT_LEG] = zn + g_z_RIGHT_BASE;	z[LEFT_LEG] = zn + g_z_LEFT_BASE; }
			}
			else
			{
				// z0 = zn + g_z_RIGHT/LEFT_BASE;
				if (t<t1z)	{ z[RIGHT_LEG] += z_Speed;				z[LEFT_LEG] += z_Speed; }
				else 		{ z[RIGHT_LEG] = zp + g_z_RIGHT_BASE;	z[LEFT_LEG] = zp + g_z_LEFT_BASE; }
			}

		//////////////////////////////
		// ���� ��ġ�� ���� theta ���.
		//////////////////////////////
			// k = RIGHT_LEG(0), LEFT_LEG(1)
			for (k=0;k<2;k++)
			{
				Get_R_C_F(mtx);
				mtx[X][3] = x[k] + x_comp[k];
				mtx[Y][3] = y[k] + y_comp[k];
				mtx[Z][3] = z[k] + z_comp[k];

				// {BC}{F}T -> {0}{6}T�� ����.
				Cal_LEG_BASE(mtx, T06, k);

				// ���ⱸ���� ���.
				sol_cnt = Inverse_Kin_M2(T06, jt_deg, state);

				// ������ ���
				if (!sol_cnt)
				{
					_printF(F("No Solution for %s\r\n"),((k==RIGHT_LEG)?"RIGHT_LEG":"LEFT_LEG"));
					goto Exit;
				}
				BestSol_M(state, jt_deg, theta[k], best_sol);

				// ���� ��ġ ��� (�������� best solution�� ���ؼ�)
				for (i=0;i<6;i++)	theta[k][i] = best_sol[i];

				// best solution�� �̿��Ͽ� �̵� ��ġ ���.
				LEG_D2P(best_sol, position_ax[k], k);
			}

		//////////////////////////////////////////////////////////////////
		// �����ϴ� ���� ������ �ϴ� ��ũ ���� �׿����� ����.
		//////////////////////////////////////////////////////////////////
			// ���� ��ǥ�踦 �������� �����.
			// Cal_Torque(theta, Torque, Pcog, step);
			Cal_Torque_ALL(theta_ARM, theta, Torque, Pcog, step);

			// ��ü{BC}�� Z���� Torque�� 0�� �ǵ��� ������.
			if (Torque[Y] < 0)
			{
				z_comp[RIGHT_LEG] -= Z_COMP_VALUE;
				z_comp[ LEFT_LEG] -= Z_COMP_VALUE;
			}
			else if (Torque[Y] > 0)
			{
				z_comp[RIGHT_LEG] += Z_COMP_VALUE;
				z_comp[ LEFT_LEG] += Z_COMP_VALUE;
			}

		///////////////////////////
		// Make & Send Packet
		///////////////////////////
			Cal_LEG_Speed(position_ax, speed_ax, EXPERIENCE);
			// Sync_Write_LEG(position_ax, speed_ax);
			Sync_Write_ALL(position_ax, speed_ax, position_ax_ARM, speed_ax_ARM);

		///////////////////////////
		// ���.
		///////////////////////////
			if (j<2)
			{
				_fprintf(fp_out,"%4d, %6.1f, %6.1f, %6.1f, %6.1f, %6.1f, %6.1f, %6.1f, %6.1f, %6.1f, %8.5f, %8.5f, %8.5f \r\n",
						t,
						x[RIGHT_LEG]+x_comp[RIGHT_LEG], y[RIGHT_LEG]+y_comp[RIGHT_LEG], z[RIGHT_LEG]+z_comp[RIGHT_LEG],
						x[LEFT_LEG]+x_comp[RIGHT_LEG], y[LEFT_LEG]+y_comp[RIGHT_LEG], z[LEFT_LEG]+z_comp[LEFT_LEG],
						Pcog[X], Pcog[Y], Pcog[Z], Torque[X], Torque[Y], Torque[Z]);
			}
		}
		///////////////////////////////
		// �� �ٲٱ�.
		///////////////////////////////
		if (step == RIGHT_LEG) 	step = LEFT_LEG;
		else					step = RIGHT_LEG;
	}
Exit:
	// fclose(fp_out);
	fp_out.close(); // �ݵ�� �ݾƾ� �����!!!!

	g_Send_Flag = 1;
	return;
}

// �ٸ��� �ⱸ�� ����.
void Check_Kin(char* pArgument,int ArgLen)
{
	char* pData;
	int Len_Sum,i;
	double theta[6];		// degree
	double mtx[3][4];

	int sol_cnt, state[MAX_SOL];
	double jt_deg[MAX_SOL][6];

	Len_Sum = 0;

	////////////////////////////////////
	// �� ������ ��ǥ ����(deg) ����.
	// ���ⱸ��(?)
	////////////////////////////////////
	// theta 0 ... 6
	theta[0] = 0;
	theta[4] = 0;
        theta[5] = 0;
	for (i=1;i<4;i++)	// 1 ... 3
	{
		if (Len_Sum > ArgLen)
		{
			_printF(F("Not enough parameter. \r\n"));
			goto Exit;
		}

		pData = strtok(pArgument," ");
		theta[i] = atof(pData);

		Len_Sum = (int)(Len_Sum + strlen(pData) + (pData-pArgument) + 1);
		pArgument = pData + strlen(pData) + 1;
	}

	// ȸ����.
	_printF(F("Angle = "));
	for (i=0;i<6;i++)	_printF(F("[%9.4f]"),theta[i]);
	_printF(F("\r\n"));

	// ���ⱸ��.
	T06_for_M2(theta, mtx);

	// ���ܺ� ��ġ
	_printF(F("X pos=[%9.4f],y pos=[%9.4f],z pos=[%9.4f] \r\n"),mtx[0][3],mtx[1][3],mtx[2][3]);

	// ���� ������ ��� 0���� �ʱ�ȭ ��.
	// (���ⱸ�н� ��ġ���� �̿��Ͽ� �����.)
	for (i=0;i<2;i++)
	{
		mtx[i][0] = 0;
		mtx[i][1] = 0;
		mtx[i][2] = 0;
	}

	// ���ⱸ���� ���.
	sol_cnt = Inverse_Kin_M2(mtx, jt_deg, state);
	sol_cnt = sol_cnt;

	// ���� ���ⱸ���� Ȯ��.
	_printF(F("INV_T06 \r\n"));
	for (i=0;i<MAX_SOL;i++)
	{
		if (state[i] == 1)
		{
			_printF(F("%d : %9.4lf,%9.4lf,%9.4lf,%9.4lf,%9.4lf,%9.4lf\r\n"),
		           i,jt_deg[i][0],jt_deg[i][1],jt_deg[i][2],jt_deg[i][3],jt_deg[i][4],jt_deg[i][5]);
		}
	}
Exit:
	return;
}

// ��ü�� �ⱸ�� ����.
void Check_Kin_Body(char* pArgument,int ArgLen)
{
	char* pData;
	int Len_Sum,i;
	double theta[4];		// degree
	double mtx[3][4];

	// int sol_cnt, state[MAX_SOL];
	// double jt_deg[MAX_SOL][6];

	Len_Sum = 0;

	////////////////////////////////////
	// �� ������ ��ǥ ����(deg) ����.
	// ���ⱸ��(?)
	////////////////////////////////////
	// theta 0 ... 3
	for (i=0;i<4;i++)	// 0 ... 3
	{
		if (Len_Sum > ArgLen)
		{
			_printF(F("Not enough parameter. \r\n"));
			goto Exit;
		}

		pData = strtok(pArgument," ");
		theta[i] = atof(pData);

		Len_Sum = (int)(Len_Sum + strlen(pData) + (pData-pArgument) + 1);
		pArgument = pData + strlen(pData) + 1;
	}

	// ȸ����.
	_printF(F("Angle = "));
	for (i=0;i<4;i++)	_printF(F("[%9.4f]"),theta[i]);
	_printF(F("\r\n"));

	// ���ⱸ��.
	T02_for_M2_BODY(theta, mtx);
	_printF(F("X pos=[%9.4f],y pos=[%9.4f],z pos=[%9.4f] \r\n"),mtx[0][3],mtx[1][3],mtx[2][3]);

	T03_for_M2_BODY(theta, mtx);
	_printF(F("X pos=[%9.4f],y pos=[%9.4f],z pos=[%9.4f] \r\n"),mtx[0][3],mtx[1][3],mtx[2][3]);

	T04_for_M2_BODY(theta, mtx);
	_printF(F("X pos=[%9.4f],y pos=[%9.4f],z pos=[%9.4f] \r\n"),mtx[0][3],mtx[1][3],mtx[2][3]);

	T05_for_M2_BODY(theta, mtx);
	_printF(F("X pos=[%9.4f],y pos=[%9.4f],z pos=[%9.4f] \r\n"),mtx[0][3],mtx[1][3],mtx[2][3]);

	/*
	// ���� ������ ��� 0���� �ʱ�ȭ ��.
	// (���ⱸ�н� ��ġ���� �̿��Ͽ� �����.)
	for (i=0;i<2;i++)
	{
		mtx[i][0] = 0;
		mtx[i][1] = 0;
		mtx[i][2] = 0;
	}

	// ���ⱸ���� ���.
	sol_cnt = Inverse_Kin_M2(mtx, jt_deg, state);
	sol_cnt = sol_cnt;

	// ���� ���ⱸ���� Ȯ��.
	_printF(F("INV_T06 \r\n"));
	for (i=0;i<MAX_SOL;i++)
	{
		if (state[i] == 1)
		{
			_printF(F("%d : %9.4lf,%9.4lf,%9.4lf,%9.4lf,%9.4lf,%9.4lf\r\n"),
		           i,jt_deg[i][0],jt_deg[i][1],jt_deg[i][2],jt_deg[i][3],jt_deg[i][4],jt_deg[i][5]);
		}
	}
	*/
Exit:
	return;
}

// Zero ��ġ�� �̵�.
void Move_Zero(void)
{
	int i;
	double theta[6];		// degree
	int speed_ax[2][6], position_ax[2][6];
	double mtx[3][4];
	double abgxyz[6];

	////////////////////////////////////
	// �� ������ ��ǥ ����(deg) ����.
	// ���ⱸ��(?)
	////////////////////////////////////
	// theta 0 ... 6
	for (i=0;i<6;i++)	// 0 ... 5
	{
		theta[i] = 0;
	}

	// ���ⱸ��.
	T06_for_M2(theta, mtx);

	// ItoU
	ItouXyz(mtx, abgxyz);
	_printF(F("U xyz = "));
	for (i=0;i<3;i++)	_printF(F("[%9.4f]"),abgxyz[i]*RADTODEG);
	for (i=3;i<6;i++)	_printF(F("[%9.4f]"),abgxyz[i]);
	_printF(F("\r\n"));

	// ������ �ٸ� ��ġ�� ��ȯ.
	LEG_D2P(theta, position_ax[RIGHT_LEG], RIGHT_LEG);

	// AX-12�� ��ġ Ȯ��
	_printF(F("R_LEG = "));
	for (i=0;i<6;i++)	_printF(F("[%9d]"),position_ax[RIGHT_LEG][i]);
	_printF(F("\r\n"));

	// ���� �ٸ� ��ġ�� ��ȯ.
	LEG_D2P(theta, position_ax[LEFT_LEG], LEFT_LEG);

	// AX-12�� ��ġ Ȯ��.
	_printF(F("L_LEG = "));
	for (i=0;i<6;i++)	_printF(F("[%9d]"),position_ax[LEFT_LEG][i]);
	_printF(F("\r\n"));

	// Make & Send Packet
	for (i=0;i<6;i++)
	{
		speed_ax[RIGHT_LEG][i] = 100;
		speed_ax[LEFT_LEG][i] = 100;
	}
	for (i=0;i<6;i++)
	{
		// ���� ��ġ ����.
		g_LEG_Position[RIGHT_LEG][i] = position_ax[RIGHT_LEG][i];
		g_LEG_Position[ LEFT_LEG][i] = position_ax[ LEFT_LEG][i];
	}

	Sync_Write_LEG(position_ax, speed_ax);
}


// Zero ��ġ�� �̵�.
void Move_Zero_BODY(void)
{
	int i;
	double theta[4];		// degree
	int speed_ax[2][4], position_ax[2][4];
	double mtx[3][4];
	double abgxyz[6];

	////////////////////////////////////
	// �� ������ ��ǥ ����(deg) ����.
	// ���ⱸ��(?)
	////////////////////////////////////
	// theta 0 ... 6
	for (i=0;i<4;i++)	// 0 ... 5
	{
		theta[i] = 0;
	}

	// ���ⱸ��.
	T05_for_M2_BODY(theta, mtx);

	// ItoU
	ItouXyz(mtx, abgxyz);
	_printF(F("U xyz = "));
	for (i=0;i<3;i++)	_printF(F("[%9.4f]"),abgxyz[i]*RADTODEG);
	for (i=3;i<6;i++)	_printF(F("[%9.4f]"),abgxyz[i]);
	_printF(F("\r\n"));

	// ������ �ٸ� ��ġ�� ��ȯ.
	ARM_D2P(theta, position_ax[RIGHT], RIGHT);

	// AX-12�� ��ġ Ȯ��
	_printF(F("R_ARM = "));
	for (i=0;i<4;i++)	_printF(F("[%9d]"),position_ax[RIGHT_LEG][i]);
	_printF(F("\r\n"));

	// ���� �ٸ� ��ġ�� ��ȯ.
	ARM_D2P(theta, position_ax[LEFT], LEFT);

	// AX-12�� ��ġ Ȯ��.
	_printF(F("L_ARM = "));
	for (i=0;i<4;i++)	_printF(F("[%9d]"),position_ax[LEFT_LEG][i]);
	_printF(F("\r\n"));

	// Make & Send Packet
	for (i=0;i<4;i++)
	{
		speed_ax[RIGHT][i] = 100;
		speed_ax[LEFT][i] = 100;
	}
	for (i=0;i<4;i++)
	{
		// ���� ��ġ ����.
		g_ARM_Position[RIGHT][i] = position_ax[RIGHT][i];
		g_ARM_Position[ LEFT][i] = position_ax[ LEFT][i];
	}

	Sync_Write_BODY(position_ax, speed_ax);
}

void Get_COG_Data(void)
{
	int i, k;
	double theta[2][6];		// degree
	int position_ax[2][6];
	double Pcog[3], Torque[3];;
    double mtx[3][4];
	double FootPosition[2][3];

	int Result;
	unsigned char TxData[BUFFER_LENGTH];
	unsigned char pRxData[BUFFER_LENGTH];

	// ����  Rx ������ ���� ����
	DUMP_RX_DATA;

	///////////////////////////
	// �ٸ��� ���� ��ġ �б�

	///////////////////////////
	// Make Packet
	// [FF][FF][ID][LEN][CMD][30][P_L][P_H][S_L][S_H][CHECKSUM]
	// �����
	TxData[0] = TxData[1] = 0xFF;
	// TxData[2] = ID;
	TxData[3] = 4;
	TxData[4] = INST_READ;
	TxData[5] = P_PRESENT_POSITION_L;
	TxData[6] = 2;
	// TxData[7] = CalCheckSum(TxData,7);
	for (k=0;k<2;k++)
	{
		for (i=0;i<6;i++)
		{
			TxData[2] = LEG_ID[k][i];
			TxData[7] = CalCheckSum(TxData,7);

			// Data �۽�.
			SendHexStr(TxData, 7);		// 0 ~ i���� ������ �۽�

			// ������ ����
      delay(10);                  // 10ms delay
			Result = RX_data_check(pRxData);

            // 060807 ����, ID Ȯ��
            if ((Result == 8) && (pRxData[2] == TxData[2]))
			{
				position_ax[k][i] = pRxData[5]+(pRxData[6]<<8);
			}
			else
			{
                i--;			// ��û ����� �ٽ� ������.

                // 060807 �߰�.
				delay(10);      // 10ms delay
				DUMP_RX_DATA;	// ����  Rx ������ ���� ����
			}

            // delay(100);
		}
		// ���� ��ġ�� ������ ��ȯ.
		LEG_P2D(position_ax[k], theta[k], k);
	}

	//////////////////////////////////////////
	// ���� �ٸ���ġ�� �̿��Ͽ� COG ���.
	//////////////////////////////////////////
        Cal_Torque(theta, Torque, Pcog, LEFT_LEG);

	//////////////////////////////////////////
	// ���� �ٸ���ġ�� �̿��Ͽ� ���� ��ġ ���.
	//////////////////////////////////////////
	for (k=0;k<2;k++)
	{
        TCF_for_M2(theta[k], mtx, k);

		FootPosition[k][X] = mtx[X][3];
		FootPosition[k][Y] = mtx[Y][3];
		FootPosition[k][Z] = mtx[Z][3];
	}

	//////////////////////////////////////////
	// ��� ���
	//////////////////////////////////////////
	_printF(F("RIGHT_LEG_Theta \r\n"));
	_printF(F("   %6.1f, %6.1f, %6.1f, %6.1f, %6.1f, %6.1f\r\n"),
			theta[RIGHT_LEG][0],theta[RIGHT_LEG][1],theta[RIGHT_LEG][2],
			theta[RIGHT_LEG][3],theta[RIGHT_LEG][4],theta[RIGHT_LEG][5]);
	_printF(F("LEFT_LEG_Theta \r\n"));
	_printF(F("   %6.1f, %6.1f, %6.1f, %6.1f, %6.1f, %6.1f\r\n"),
			theta[LEFT_LEG][0],theta[LEFT_LEG][1],theta[LEFT_LEG][2],
			theta[LEFT_LEG][3],theta[LEFT_LEG][4],theta[LEFT_LEG][5]);

	_printF(F("RIGHT_Foot_Positon \r\n"));
	_printF(F("   %6.1f, %6.1f, %6.1f\r\n"),
			FootPosition[RIGHT_LEG][X],FootPosition[RIGHT_LEG][Y],FootPosition[RIGHT_LEG][Z]);

	_printF(F("LEFT_Foot_Positon \r\n"));
	_printF(F("   %6.1f, %6.1f, %6.1f\r\n"),
			FootPosition[LEFT_LEG][X],FootPosition[LEFT_LEG][Y],FootPosition[LEFT_LEG][Z]);

	_printF(F("COG \r\n"));
	_printF(F("   %6.1f, %6.1f, %6.1f\r\n"),Pcog[X], Pcog[Y], Pcog[Z]);
}

void Change_Comp_Slope(char* pArgument,int ArgLen)
{
	char* pData;
	int Len_Sum, i;
	int Arg[2];

	int ID;
	unsigned char pTable[2];

	//////////////////////////////
	// ���� ����
	/////////////////////////////
	Len_Sum = 0;
	for (i=0;i<2;i++)
	{
		if (Len_Sum > ArgLen)
		{
			_printF(F("Not enough parameter. \r\n"));
			goto Exit;
		}

		pData = strtok(pArgument," ");

		Arg[i] = atof(pData);

		Len_Sum = (int)(Len_Sum + strlen(pData) + (pData-pArgument) + 1);
		pArgument = pData + strlen(pData) + 1;
	}
	ID = (((int)Arg[0] == 0)? BROADCASTING_ID : (int)Arg[0]);
	pTable[0] = pTable[1] = (int)Arg[1];

	// AX-12�� �����͸� �� ����.
	WriteHex(ID, P_CW_COMPLIANCE_SLOPE, pTable, 2);
Exit:
	return;
}

// Position���� �̵�.
// ID POSITION SPEED
// Speed�� ���� ��쿡�� 100���� ��.
void Move_Position(char* pArgument,int ArgLen)
{
	char* pData;
	int Len_Sum;
	int ID, Position, Speed;
	unsigned char TxData[BUFFER_LENGTH];

	Len_Sum = 0;

	// ID
	if (Len_Sum > ArgLen)
	{
		_printF(F("Not enough parameter. \r\n"));
		goto Exit;
	}

	pData = strtok(pArgument," ");
	Len_Sum = (int)(Len_Sum + strlen(pData) + (pData-pArgument) + 1);

	pArgument = pData + strlen(pData) + 1;
	ID = atoi(pData);

	if ((ID < 0x00) || (ID > 0xFF))
	{
		_printF(F("Out of range : ID = 0 ~ 255 \r\n"));
		goto Exit;
	}

	// Position
	if (Len_Sum > ArgLen)
	{
		_printF(F("Not enough parameter. \r\n"));
		goto Exit;
	}

	pData = strtok(pArgument," ");
	Len_Sum = (int)(Len_Sum + strlen(pData) + (pData-pArgument) + 1);

	pArgument = pData + strlen(pData) + 1;
	Position = atoi(pData);

	if ((Position < 0x00) || (Position > 0x3FF))
	{
		_printF(F("Out of range : Position = 0 ~ 1023 \r\n"));
		goto Exit;
	}

	// Speed
	if (Len_Sum > ArgLen)
	{
		Speed = 100;	// Default
	}
	else
	{
		pData = strtok(pArgument," ");
		Speed = atoi(pData);
	}

	if ((Speed < 0x00) || (Speed > 0x3FF))
	{
		_printF(F("Out of range : Speed = 0 ~ 1023 \r\n"));
		goto Exit;
	}

	// Make Packet
	// [FF][FF][ID][LEN][CMD][P_L][P_H][S_L][S_H][CHECKSUM]
	// Header
	TxData[0] = TxData[1] = 0xFF;
	TxData[2] = ID;
	TxData[3] = 7;
	TxData[4] = INST_WRITE;
	TxData[5] = P_GOAL_POSITION_L;
	TxData[6] = Position & 0xFF;
	TxData[7] = (Position >> 8) & 0xFF;
	TxData[8] = Speed & 0xFF;
	TxData[9] = (Speed >> 8) & 0xFF;
	TxData[10] = CalCheckSum(TxData,10);

	// Data �۽�.
	SendHexStr(TxData, 10);		// 0 ~ i���� ������ �۽�
	_printF(F("\r\n"));

Exit:
    return;
}

// �����߽��� Ȯ���� ���� �Լ�
// R0, L0�� �������� �� ��ǥ���� �Է����� Theta ���
// Theata�� ���� LF�� �������� COG ���.
// #define 	LIFT	0	// �� �� ��ŭ ������ ���� ��� �ø���.

void Check_Mtx(char* pArgument,int ArgLen)
{
	char* pData;
	int Len_Sum,i;
	double theta[2][6];		// degree
	double Pcog[3], Torque[3];

	double mtx[3][4] = {{0,0,0,0},{0,0,0,0},{0,0,0,0}};

	int speed_ax[2][6], position_ax[2][6];
	int state[MAX_SOL];
	double jt_deg[MAX_SOL][6];
	int LIFT;

	Len_Sum = 0;

	////////////////////////////////////
	// X,Y,Z ���� ��ǥ ��ġ�� ����.
	// ���ⱸ��
	////////////////////////////////////
	for (i=0;i<3;i++)	// 0 ... 2
	{
		if (Len_Sum > ArgLen)
		{
			// Serial.println("Not enough parameter.");
			_printF(F("Not enough parameter.\r\n"));
			goto Exit;
		}

		pData = strtok(pArgument," ");

		// ����) mtx�� 	mtx[0][3](x_pos), mtx[1][3](y_pos), mtx[2][3](z_pos)�� �����.
		mtx[i][3] = atof(pData);

		Len_Sum = (int)(Len_Sum + strlen(pData) + (pData-pArgument) + 1);
		pArgument = pData + strlen(pData) + 1;
	}
	// Lift��(������ ���� ���ø� ����)�� ����.
	// �����ϸ� 0��.
	if (Len_Sum > ArgLen)
	{
		LIFT = 0;
	}
	else
	{
		pData = strtok(pArgument," ");
		LIFT = atof(pData);
	}

	// ���ⱸ���� ���.(left)
	(void)Inverse_Kin_M2(mtx, jt_deg, state);

	if (state[0] == 0)
	{
		// Serial.println("No Solution"));
		_printF(F("No Solution\r\n"));
		goto Exit;
	}
	for (i=0;i<6;i++)       theta[LEFT_LEG][i] = jt_deg[0][i];

	// ���ⱸ���� ���.(right)
	// mtx[Z][POS] = mtx[Z][POS] - LIFT;
	mtx[Z][POS] = mtx[Z][POS] + LIFT;	// 060808 ����. ��ǥ�迡 ������ ����.
	(void)Inverse_Kin_M2(mtx, jt_deg, state);

	if (state[0] == 0)
	{
	        // Serial.println("No Solution"));
		      _printF(F("No Solution\r\n"));
	        goto Exit;
	}
	for (i=0;i<6;i++)       theta[RIGHT_LEG][i] = jt_deg[0][i];

	// ȸ����.
	_printF(F("Angle = "));
	for (i=0;i<6;i++)	_printF(F("[%9.4f]"),theta[0][i]);
	_printF(F("\r\n"));

	// �����߽ɰ��
	if (LIFT <= 0)
	{
		Cal_Torque(theta, Torque, Pcog, LEFT_LEG);
		DBG_Print_P("LF based COG ", Pcog);
	}
	else
	{
		Cal_Torque(theta, Torque, Pcog, RIGHT_LEG);
		DBG_Print_P("RF based COG ", Pcog);
	}

	// �̵� Ȯ��.
	// Serial.println("Run AX-12, press 'g'");
	_printF(F("Run AX-12, press 'g'\r\n"));
	// while(kbhit());
  if (getSerialChar() != 'g') goto Exit;		// if (getch() != 'g') goto Exit;

	// ������ �ٸ� ��ġ�� ��ȯ.
	LEG_D2P(theta[RIGHT_LEG], position_ax[RIGHT_LEG], RIGHT_LEG);

	// ���� �ٸ� ��ġ�� ��ȯ.
	LEG_D2P(theta[LEFT_LEG], position_ax[LEFT_LEG], LEFT_LEG);

	// Make & Send Packet
	Cal_LEG_Speed(position_ax, speed_ax, NORMAL);
	Sync_Write_LEG(position_ax, speed_ax);

Exit:
	_printF(F("\r\n"));
    return;
}

