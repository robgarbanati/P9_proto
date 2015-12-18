#!armcc -E --device=DARMCM1
#define PROG_START			0x00000200
#define BOOT_START			0x00010000
#define ROM_SIZE			0x12000
#define SRAM_START			0x20000000
#define SRAM_END			0x20002000
#define SRAM_SIZE			0x2000

LOAD_ROM		0x00000 ROM_SIZE
{
	_RESET		 +0
	{
		startup_N572_ISP.o(RESET, +FIRST)
		startup_N572_ISP.o(EXCEPTION)
	}
	_JMPTABLE	PROG_START FIXED
	{
		startup_N572_ISP.o(JMPTABLE)
	}
	_CODE		+0
	{
		startup_N572_ISP.o(STARTUP)	
		* (+RO)
	}
	_IRQTABLE	SRAM_START ALIGN 4 
	{
		startup_N572_ISP.o(IRQTABLE)
	}
	_STACK		+0 UNINIT ALIGN 4
	{
		startup_N572_ISP.o(STACK)
	}
	_SRAM		+0 ALIGN 4
	{
		* (+RW, +ZI)
	}
}
