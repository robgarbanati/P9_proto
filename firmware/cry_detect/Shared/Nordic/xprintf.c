/*------------------------------------------------------------------------/
/  Universal string handler for user console interface
/-------------------------------------------------------------------------/
/
/  Copyright (c) 2013, ChaN, all right reserved.
/
/ * This software is a free software and there is NO WARRANTY.
/ * No restriction on use. You can use, modify and redistribute it for
/   personal, non-profit or commercial products UNDER YOUR RESPONSIBILITY.
/ * Redistributions of source code must retain the above copyright notice.
/
/-------------------------------------------------------------------------*/

#include <ctype.h>
#include <string.h>
#include <stdarg.h>
#include "xprintf.h"

#define DO_FLOAT

#if _USE_XFUNC_OUT
#include <stdarg.h>

// Pointer to the output stream.
void (*xfunc_out)(unsigned char) = NULL;

// Put a character.
void xputc(char c)
{
	// CR -> CRLF
	if (_CR_CRLF && c == '\n') xputc('\r');

	if (xfunc_out) xfunc_out((unsigned char) c);
}


// Put a string to the default device.
void xputs(const char* str)
{
	while (*str) xputc(*str++);
}


//
// Formatted string output
//
//  xprintf("%d", 1234);			"1234"
//  xprintf("%6d,%3d%%", -200, 5);	"  -200,  5%"
//  xprintf("%-6u", 100);			"100   "
//  xprintf("%ld", 12345678L);		"12345678"
//  xprintf("%04x", 0xA3);			"00a3"
//  xprintf("%08LX", 0x123ABC);		"00123ABC"
//  xprintf("%016b", 0x550F);		"0101010100001111"
//  xprintf("%s", "String");		"String"
//  xprintf("%-4s", "abc");			"abc "
//  xprintf("%4s", "abc");			" abc"
//  xprintf("%c", 'a');				"a"
//  xprintf("%f", 10.0);            "10.0000"

#if _USE_XFLOAT_OUT
// This routine correctly formats a floating point number into it's 
// ASCII representation.  The routine will insert leading spaces as 
// necessary to satisfy the requirements of the prescribed format. 
// w is the field width and pw is the precision width. The buffer 
// passed in must be at least 16 characters.
//
// eg:
// xftoa(12.3456789,buf,10,5);
// After the routine, buf will contain: '  12.34567'.
//
static void xftoa(double fval, char *buf, unsigned int fw, unsigned int pw)
{
	long lval;
	unsigned int i;
	unsigned int j;
	unsigned int k;

	// Sanity check the field and places values.
	if (pw > 5) pw = 5;
	if (fw < 1) fw = 1;
	if (fw > 15) fw = 15;

	// Pull the integer off.
	lval = (long) fval;

	// Index the start and end of the buffer.
	i = 0;
	j = 15;
	k = 15;

	// Handle a negative integer.
	if (lval < 0) { buf[j--] = '-'; lval = -lval; }

	// Handle the case of a zero integer.
	if (!lval) { buf[j--] = '0'; }

	// Put the rest of the integer digits into the buffer in reverse order.
	while (lval) { buf[i++] = '0' + (char) (lval % 10); lval /= 10; }

	// Reorder the digits at the end of the buffer.
	while (i) { buf[j--] = buf[--i]; }

	// Do we need the decimal part?
	if (pw)
	{
		// Place '.' into the buffer.
		buf[j--] = '.';

		// Prevent a negative fraction.
		if (fval < 0.0f) fval = -fval;

		// Gather the fraction digits.
		while (pw--)
		{
			// Remove integer part of the float value...
			fval = fval - (long) fval;

			// Multiply to get the next digit.
			fval *= 10.0f;

			// Pull the next digit off.
			lval = (long) fval;

			// Write the digit.
			buf[j--] = '0' + (char) lval;
		}
	}

	// Put any needed white space.
	if (fw > (k - j)) { while (i < (fw - (k - j))) buf[i++] = ' '; }

	// Pull the digits off the buffer to the correct location.
	while (k != j) { buf[i++] = buf[k--]; }

	// Null terminate.
	buf[i] = 0;
}
#endif // _USE_XFLOAT_OUT

// Put a character to a string or function.
static void xoutc(char **outs, void(*outf)(unsigned char), char c)
{
	// CR -> CRLF
	if (_CR_CRLF && c == '\n') xoutc(outs, outf, '\r');

	// Putting into a string?
	if (*outs)
	{
		*(*outs)++ = c;
	}
	// Putting to a function?
	else if (outf)
	{
		outf((unsigned char) c);
	}	
}

// Get the next char from the point reading from data or program memory.
static char xnextch(const char* s)
{
	return *s;
}

// Output the formatted string.
static void xvprintf(char *outs, void(*outf)(unsigned char), const char* fmt, va_list arp)
{
	char c;
	char d;
	char *p;
	char s[16];
	unsigned int f;
	unsigned int r;
	unsigned int i;
	unsigned int j;
	unsigned int w;
	unsigned long v;
#if _USE_XFLOAT_OUT
	unsigned int pw = 0;
#endif

	for (;;)
	{
		// Get a character.
		c = xnextch(fmt++);
		// End of format?
		if (!c) break;
		// Pass through if it not a % sequence.
		if (c != '%') {
			xoutc(&outs, outf, c);
			continue;
		}
		f = 0;
		// Get first character of % sequence.
		c = xnextch(fmt++);
		// Flag: '0' padded.
		if (c == '0') {
			f = 1;
			c = xnextch(fmt++);
		} else {
			// Flag: left justified.
			if (c == '-') {
				f = 2;
				c = xnextch(fmt++);
			}
		}
		for (w = 0; c >= '0' && c <= '9'; c = xnextch(fmt++))	/* Minimum width */
			w = w * 10 + c - '0';
#if _USE_XFLOAT_OUT
		// Float precision width.
		if (c == '.') {
			c = xnextch(fmt++);
			for (pw = 0; c >= '0' && c <= '9'; c = xnextch(fmt++))	/* Precision width */
				pw = pw * 10 + c - '0';
		}
#endif
		// Prefix: Size is long int.
		if (c == 'l' || c == 'L') {
			f |= 4;
			c = xnextch(fmt++);
		}
		// End of format.
		if (!c) break;
		d = c;
		if (d >= 'a') d -= 0x20;
		// Type is...
		switch (d) {
		// String.
		case 'S' :
			p = va_arg(arp, char*);
			for (j = 0; p[j]; j++);
			while (!(f & 2) && j++ < w) xoutc(&outs, outf, ' ');
			while (*p) xoutc(&outs, outf, *p++);
			while (j++ < w) xoutc(&outs, outf, ' ');
			continue;
		// Character.
		case 'C' :
			xoutc(&outs, outf, (char) va_arg(arp, int));
			continue;
#if _USE_XFLOAT_OUT
		// Float.
		case 'F' :
			p = s;
			xftoa((double) va_arg(arp, double), p, w, pw);
			while (*p) xoutc(&outs, outf, *p++);
			continue;
#endif
		// Binary.
		case 'B' :
			r = 2;
			break;
		// Octal.
		case 'O' :
			r = 8;
			break;
		// Signed decimal.
		case 'D' :
		// Unsigned decimal.
		case 'U' :
			r = 10;
			break;
		// Hexadecimal.
		case 'X' :
			r = 16;
			break;
		// Unknown type (passthrough).
		default:
			xoutc(&outs, outf, c);
			continue;
		}

		// Get an argument and put it in numeral.
		v = (f & 4) ? va_arg(arp, long) : ((d == 'D') ? (long) va_arg(arp, int) : (long) va_arg(arp, unsigned int));
		if (d == 'D' && (v & 0x80000000)) {
			v = 0 - v;
			f |= 8;
		}
		i = 0;
		do {
			d = (char)(v % r); v /= r;
			if (d > 9) d += (c == 'x') ? 0x27 : 0x07;
			s[i++] = d + '0';
		} while (v && i < sizeof(s));
		if (f & 8) s[i++] = '-';
		j = i; d = (f & 1) ? '0' : ' ';
		while (!(f & 2) && j++ < w) xoutc(&outs, outf, d);
		do xoutc(&outs, outf, s[--i]); while(i);
		while (j++ < w) xoutc(&outs, outf, ' ');
	}
	// Null terminate if output string.
	if (outs) *outs = 0;
}


// Put a formatted string to the default device.
void xprintf(const char* fmt, ...)
{
	va_list arp;

	va_start(arp, fmt);
	xvprintf(NULL, xfunc_out, fmt, arp);
	va_end(arp);
}


// Put a formatted string to the memory.
void xsprintf(char* buff, const char *fmt, ...)
{
	va_list arp;

	va_start(arp, fmt);
	xvprintf(buff, NULL, fmt, arp);
	va_end(arp);
}


// Put a formatted string to the specified device.
void xfprintf(void(*outf)(unsigned char), const char* fmt, ...)
{
	va_list arp;

	va_start(arp, fmt);
	xvprintf(NULL, outf, fmt, arp);
	va_end(arp);
}


#if _USE_XDUMP_OUT
// Dump a line of binary data.
// buff		Pointer to the array to be dumped
// addr		Heading address value
// len		Number of items to be dumped
// width	Size of the items (DW_CHAR, DW_SHORT, DW_LONG)
void xdump(const void* buff, unsigned long addr, int len, int width)
{
	int i;
	const unsigned char *bp;
	const unsigned short *sp;
	const unsigned long *lp;

	// Output the heading address.
	xprintf("%08lX ", addr);

	switch (width) {
	case DW_CHAR:
		bp = buff;
		// Hexadecimal dump.
		for (i = 0; i < len; i++)
			xprintf(" %02X", bp[i]);
		xputc(' ');
		// ASCII dump.
		for (i = 0; i < len; i++)
			xputc((bp[i] >= ' ' && bp[i] <= '~') ? bp[i] : '.');
		break;
	case DW_SHORT:
		sp = buff;
		// Hexadecimal dump.
		do
			xprintf(" %04X", *sp++);
		while (--len);
		break;
	case DW_LONG:
		lp = buff;
		// Hexadecimal dump.
		do
			xprintf(" %08LX", *lp++);
		while (--len);
		break;
	}

	xputc('\n');
}
#endif // _USE_XDUMP_OUT
#endif // _USE_XFUNC_OUT


#if _USE_XFUNC_IN

// Pointer to the input stream.
unsigned char (*xfunc_in)(void) = NULL;

// Get a line from the input.
// buff		Pointer to the buffer
// len		Buffer length
// tocase <0: To lower case, >0: To upper case
// Returns 0:End of stream, 1:A line arrived
int xgets(char * buff, int len, int tocase)
{
	int c, i;

	// Sanity check for input function.
	if (!xfunc_in) return 0;

	i = 0;
	for (;;)
	{
		// Get a char from the incoming stream.
		c = xfunc_in();
		// Convert to upper/lower case?
		if (tocase > 0) c = toupper(c);
		if (tocase < 0) c = tolower(c);
		// End of stream?
		if (!c) return 0;
		// End of line?
		if (c == _XEOL) break;
		// Back space?
		if (c == '\b' && i)
		{
			i--;
			if (_LINE_ECHO) { xputc(c); xputc(' '); xputc(c); }
			continue;
		}
		// Visible chars.
		if (c >= ' ' && i < len - 1)
		{
			buff[i++] = c;
			if (_LINE_ECHO) xputc(c);
		}
	}
	// Null terminate.
	buff[i] = 0;
	if (_LINE_ECHO) xputc('\n');
	return 1;
}


// Parsing out the next non-space word from a string.
// str		Pointer to pointer to the string
// cmd		Pointer to pointer of command
// Returns 0:Failed, 1:Successful
int xparse(char **str, char **word)
{
  // Skip leading spaces.
	while (**str && isspace(**str)) (*str)++;

  // Set the word.
  *word = *str;

  // Skip non-space characters.
	while (**str && !isspace(**str)) (*str)++;

  // Null terminate the word.
  if (**str) *(*str)++ = 0;

  return (*str != *word) ? 1 : 0;
}


// Get a value of the string.
//  "123 -5   0x3ff 0b1111 0377  w
//  ^                               1st call returns 123 and next pointer
//       ^                          2nd call returns -5 and next pointer
//            ^                     3rd call returns 1023 and next pointer
//                  ^               4th call returns 15 and next pointer
//                         ^        5th call returns 255 and next pointer
//                               ^  6th call fails and returns 0
int xatoi(char **str, long *res)
{
	unsigned long val;
	unsigned char c;
	unsigned char r;
	unsigned char s = 0;

	// Default is failure.
	*res = 0;

	// Skip leading spaces.
	while ((c = **str) == ' ') (*str)++;

	// Negative?
	if (c == '-')
	{
		s = 1;
		c = *(++(*str));
	}

	if (c == '0') {
		c = *(++(*str));
		switch (c) {
		// Hexadecimal
		case 'x':
		case 'X':
			r = 16; c = *(++(*str));
			break;
		// Binary
		case 'b':
		case 'B':
			r = 2; c = *(++(*str));
			break;
		default:
			// Single zero?
			if (c <= ' ') return 1;
			// Invalid character?
			if (c < '0' || c > '9') return 0;
			// Octal
			r = 8;
		}
	}
	else
	{
		// EOL or invalid char?
		if (c < '0' || c > '9') return 0;
		// Decimal
		r = 10;
	}

	val = 0;
	while (c > ' ')
	{
		if (c >= 'a') c -= 0x20;
		c -= '0';
		if (c >= 17) {
			c -= 7;
			// Invalid character?
			if (c <= 9) return 0;
		}
		// Invalid character for radix?
		if (c >= r) return 0;
		val = val * r + c;
		c = *(++(*str));
	}

	// Apply sign if needed.
	if (s) val = 0 - val;

	// Set the result value.
	*res = val;

	return 1;
}

#endif // _USE_XFUNC_IN
