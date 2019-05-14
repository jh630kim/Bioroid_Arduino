/* _printf, _fprintf를 사용하기 위해서는 
   #C:\Program Files (x86)\Arduino\hardware\arduino\avr의 아래에 있는 
   platform.txt를 아래와 같이 바꿔야 한다.
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
  // 사용 예
  // _printf("%d+%d=%d\n",1,2,3);
  // _printf("float var %.4f\n",1.234);
  // _printf("hello %s\n", "world"); // %s는 소문자!!!
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
  // 사용 예
  // _printF(F("%d+%d=%d\n"),1,2,3);
  // _printF(F("float var %.4f\n"),1.234);
  // _printF(F("hello %S\n"), F("world")); // %s는 대문자!!!
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
  // 사용 예
  // _printf("%d+%d=%d\n",1,2,3);
  // _printf("float var %.4f\n",1.234);
  // _printf("hello %s\n", "world"); // %s는 소문자!!!
}

