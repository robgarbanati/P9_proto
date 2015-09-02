/*------------------------------------------------------------------------*/
/* Universal string handler for user console interface  (C)ChaN, 2011     */
/*------------------------------------------------------------------------*/

#ifndef _XPRINTF_H_
#define _XPRINTF_H_

#define _USE_XFUNC_OUT	1	/* 1: Use output functions */
#define	_CR_CRLF				1	/* 1: Convert \n ==> \r\n in the output char */

#define _USE_XFUNC_IN	1	/* 1: Use input function */
#define	_LINE_ECHO		1	/* 1: Echo back input chars in xgets function */

#define _USE_XFLOAT_OUT	0   /* 1: Handle floating point output format */
#define _USE_XDUMP_OUT	0   /* 1: Include dump function */

// EOL character to use.
#define	_XEOL						'\r'
// #define	_XEOL						'\n'
#define	_XBKSP					0x7F

#if _USE_XFUNC_OUT
#define xdev_out(func) xfunc_out = (void(*)(unsigned char))(func)
extern void (*xfunc_out)(unsigned char);
void xputc (char c);
void xputs (const char* str);
void xprintf (const char* fmt, ...);
void xsprintf (char* buff, const char* fmt, ...);
void xfprintf (void (*func)(unsigned char), const char*	fmt, ...);
#if _USE_XDUMP_OUT
void xdump(const void* buff, unsigned long addr, int len, int width);
#define DW_CHAR		sizeof(char)
#define DW_SHORT	sizeof(short)
#define DW_LONG		sizeof(long)
#endif
#endif

#if _USE_XFUNC_IN
#define xdev_in(func) xfunc_in = (unsigned char(*)(void))(func)
extern unsigned char (*xfunc_in)(void);
int xgets (char * buff, int len, int tocase);
int xparse(char **str, char **word);
int xatoi (char** str, long* res);
#endif

#endif // _XPRINTF_H_
