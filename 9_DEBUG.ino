/* Not Used (18.5.10, by JHK)
void DBG_Print_T(const char* Title, const double mtx[3][4])
{
	int i;
	
	if (Title != NULL)	_printF(F("Title = %s \r\n",Title);
	
	for (i=0;i<3;i++)
		_printF(F("   %9.4f, %9.4f, %9.4f, %9.4f\r\n",mtx[i][0], mtx[i][1], mtx[i][2],mtx[i][3]);
	getSerialChar();    // getch();
}

void DBG_Print_theta(const char* Title, const double theta[6])
{
	if (Title != NULL)	_printF(F("Title = %s \r\n",Title);
	
	_printF(F("   %9.4f, %9.4f, %9.4f, %9.4f, %9.4f, %9.4f\r\n",theta[0],theta[1],theta[2],theta[3],theta[4],theta[5]);
	getSerialChar();  // getch();
}
*/
void DBG_Print_P(const char* Title, const double P[3])
{
	if (Title != NULL)	_printF(F("%s : "),Title);
	
	_printF(F("Px = %6.1f, Py = %6.1f, Pz = %6.1f\r\n"),P[X],P[Y],P[Z]);
}

/* Not Used (18.5.10, by JHK)
void DBG_LF(void)
{
	_printF(F("\r\n"));
	getSerialChar();  // getch();
}

void DBG_R_default(double mtx[3][4])
{
	mtx[0][3] = -200;
	mtx[1][3] = 0;
	mtx[2][3] = -32.7;
}

void DBG_L_default(double mtx[3][4])
{
	mtx[0][3] = -200;
	mtx[1][3] = 0;
	mtx[2][3] = 32.7;
}

void DBG_Print_Packet(const unsigned char TxData[BUFFER_LENGTH])
{
	int total_length;
	
	total_length = min(TxData[3]+4,BUFFER_LENGTH);
	
	for (int i=0;i<total_length;i++)
	{
		_printF(F("[%02X]",TxData[i]);
	}
	_printF(F("\r\n"));	
}
*/
		
	
	
