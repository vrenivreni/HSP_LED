/*
 * wait.c
 *
 * Created: 12.08.2018 17:11:24
 *  Author: mea39511
 */ 

void waitForUs(unsigned int us)
{
	__asm__ volatile (
	"1: sbiw %0,1" "\n"
	"nop" "\n"
	"nop" "\n"
	"nop" "\n"
	"nop" "\n"
	"nop" "\n"
	"nop" "\n"
	"nop" "\n"
	"nop" "\n"
	"nop" "\n"
	"nop" "\n"
	"nop" "\n"
	"nop" "\n"
	"brne 1b"
	: "+w" (us)   );
}



void waitForMs(unsigned int ms)
{
	unsigned int f;
	for(f=0;f<ms;f++)
	waitForUs(1000);
}


