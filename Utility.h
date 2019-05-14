void _printf(const char *s,...);                // 문자열 상수를 메모리에 저장(전역변수 공간)
void _printF(const __FlashStringHelper *s,...); // 문자열 상수를 플래시에 저장(프로그램 메모리 공간)
void _fprintf(File myFile, const char *s,...);

