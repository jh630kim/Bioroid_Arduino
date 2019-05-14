/* _printf, _fprintf�� ����ϱ� ���ؼ��� 
   #C:\Program Files (x86)\Arduino\hardware\arduino\avr�� �Ʒ��� �ִ� 
   platform.txt�� �Ʒ��� ���� �ٲ�� �Ѵ�.
       compiler.c.elf.extra_flags=
       -> compiler.c.elf.extra_flags=-Wl,-u,vfprintf -lprintf_flt -lm
*/

void _printf(const char *s,...)
{
  va_list args;
  va_start(args,s);
  int n=vsnprintf(NULL,0,s,args);
  char *str=new char[n+1];
  vsprintf(str,s,args);
  va_end(args);
  Serial.print(str);
  BTSerial.print(str);
  
  delete [] str;
  // ��� ��
  // _printf("%d+%d=%d\n",1,2,3);
  // _printf("float var %.4f\n",1.234);
  // _printf("hello %s\n", "world"); // %s�� �ҹ���!!!
}

void _printF(const __FlashStringHelper *s,...)
{
  va_list args;
  va_start(args,s);
  int n=vsnprintf_P(NULL,0,(const char*)s,args);
  char *str=new char[n+1];
  vsprintf_P(str,(const char*)s,args);
  va_end(args);
  Serial.print(str);
  BTSerial.print(str);
  delete [] str;
  // ��� ��
  // _printF(F("%d+%d=%d\n"),1,2,3);
  // _printF(F("float var %.4f\n"),1.234);
  // _printF(F("hello %S\n"), F("world")); // %s�� �빮��!!!
}

void _fprintf(File myFile, const char *s,...)
{
  va_list args;
  va_start(args,s);
  int n=vsnprintf(NULL,0,s,args);
  char *str=new char[n+1];
  vsprintf(str,s,args);
  va_end(args);
  myFile.print(str);
  delete [] str;
  // ��� ��
  // _printf("%d+%d=%d\n",1,2,3);
  // _printf("float var %.4f\n",1.234);
  // _printf("hello %s\n", "world"); // %s�� �ҹ���!!!
}

