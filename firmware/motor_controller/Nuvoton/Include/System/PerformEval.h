// user timer2, clock sorce internal 24MHz ROSC
// prescale = 0
// compare value = 65535 (max value, compare value cannot be 0 or 1.)
// ==> 1 clock count value == 1 / 24,000,000 sec. 
// Notice:
// 1. do not execute _COUNT too frequent to reduce redudant cycles
// 2. must execute _COUNT in 65535/24000000 ec.

//UINT32 u32TimerCnt = 0;
#if (defined(__N572F064__)||defined(__N572F065__))
#define PERFORMEVAL_TIMER_SOURCE	eDRVCLK_TIMERSRC_24M
#else
#define PERFORMEVAL_TIMER_SOURCE	eDRVCLK_TIMERSRC_48M
#endif	// (defined(__N572F064__)||defined(__N572F065__))

#define _PERFORMEVAL_INIT_COUNT()\
	DrvCLK_SetClkSrcTmr2(PERFORMEVAL_TIMER_SOURCE); \
	DrvTimer_OpenTmr2(DRVTIMER_START_COUNTING|DRVTIMER_PERIODIC_MODE|TMR_TCSR_CRST|0,65535); \
	DrvTimer_EnableIntTmr2();

#define _PERFORMEVAL_RESET_COUNT(u32TimerCnt)  \
	u32TimerCnt = 0; \
	DrvTimer_SetCompareValueTmr2(65535); \
	DrvTimer_EnableIntTmr2();

#define _PERFORMEVAL_COUNT(u32TimerCnt) \
	DrvTimer_DisableIntTmr2(); \
	u32TimerCnt += DrvTimer_GetClkCountTmr2();\
	DrvTimer_SetCompareValueTmr2(65535); 

#define _PERFORMEVAL_COUNT_IRQ(u32TimerCnt) \
	u32TimerCnt += 65535;
