/*! \file xmodem.c \brief XModem Transmit/Receive Implementation with CRC and 1K support. */
//*****************************************************************************
//
// File Name  : 'xmodem.c'
// Title    : XModem Transmit/Receive Implementation with CRC and 1K support
// Author     : Pascal Stang - Copyright (C) 2006
// Created    : 4/22/2006
// Revised    : 7/22/2006
// Version    : 0.1
// Target MCU   : AVR processors
// Editor Tabs  : 4
//
// This code is distributed under the GNU Public License
//    which can be found at http://www.gnu.org/licenses/gpl.txt
//

#include <string.h>
#include <stdlib.h>
#include "Global.h"
#include "SpiFS.h"
#include "Shell.h"
#include "XModem.h"

static p_xm_send_func xmodem_out_func;
static p_xm_recv_func xmodem_in_func;

static unsigned char xmbuf[128 + 6];

static uint16_t crc_xmodem_update(uint16_t crc, uint8_t data)
{
  int i;

  crc = crc ^ ((uint16_t) data << 8);
  for (i = 0; i < 8; i++)
  {
    if (crc & 0x8000)
      crc = (crc << 1) ^ 0x1021;
    else
      crc <<= 1;
  }

  return crc;
}

static int xmodem_crc_check( int crcflag, const unsigned char *buffer, int size )
{
  // crcflag=0 - do regular checksum
  // crcflag=1 - do CRC checksum

  if (crcflag)
  {
    unsigned short crc = 0;
    unsigned short pktcrc = (buffer[size] << 8) + buffer[size + 1];

    // Do CRC checksum.
    while (size--) crc = crc_xmodem_update(crc, *buffer++);

    // Check checksum against packet.
    if (crc == pktcrc) return 1;
  }
  else
  {
    int i;
    unsigned char cksum = 0;

    // Do regular checksum.
    for (i=0; i < size; ++i) cksum += buffer[i];

    // Check checksum against packet.
    if (cksum == buffer[size]) return 1;
  }

  return 0;
}

static void xmodem_flush(void)
{
  while( xmodem_in_func(XMODEM_FLUSH_DELAY) >= 0 );
}

void xmodem_init( p_xm_send_func send_func, p_xm_recv_func recv_func )
{
  xmodem_out_func = send_func;
  xmodem_in_func = recv_func;
}

long xmodem_receive(SPIFS_FILE *recvFile)
{
	int c;
  unsigned short i;
  unsigned char seqnum = 1;     // xmodem sequence number starts at 1
  unsigned char response = 'C';   // solicit a connection with CRC
  unsigned short pktsize = 128;   // default packet size is 128 bytes
  char retry = XMODEM_RETRY_LIMIT;
  unsigned char crcflag = 0;
  unsigned long totalbytes = 0;

	// Run xmodem processing state machine.
  while (retry > 0)
  {
    // Solicit a connection/packet.
    xmodem_out_func(response);

    // Wait for start of packet.
    if ((c = xmodem_in_func(XMODEM_TIMEOUT_DELAY)) >= 0)
    {
      switch(c)
      {
				case SOH:
					pktsize = 128;
					break;
				case EOT:
					// Completed transmission normally.
					xmodem_flush();
					xmodem_out_func(ACK);
					return totalbytes;
				case CAN:
					if ((c = xmodem_in_func(XMODEM_TIMEOUT_DELAY)) == CAN)
					{
						// Transaction cancelled by remote node.
						xmodem_flush();
						xmodem_out_func(ACK);
						return XMODEM_ERROR_REMOTECANCEL;
					}
				default:
					break;
      }
    }
    else
    {
      // Timed out so try again.  No need to flush because 
			// receive buffer is already empty.
      retry--;
      //response = NAK;
      continue;
    }

    // Check if CRC mode was accepted.
    if (response == 'C') crcflag = 1;

    // Got SOH/STX, add it to processing buffer.
    xmbuf[0] = c;

    // Try to get rest of packet.
    for (i  =0; i < (pktsize + crcflag + 4 - 1); i++)
    {
      if ((c = xmodem_in_func(XMODEM_TIMEOUT_DELAY)) >= 0)
      {
        xmbuf[i + 1] = c;
      }
      else
      {
        // timed out, try again
        retry--;
        xmodem_flush();
        response = NAK;
        break;
      }
    }
    // packet was too small, retry
    if (i < (pktsize+crcflag+4-1))
      continue;

    // Got whole packet. Check validity of packet.
    // Sequence number was transmitted w/o error and packet is not corrupt.
    if ((xmbuf[1] == (unsigned char)(~xmbuf[2])) && xmodem_crc_check(crcflag, &xmbuf[3], pktsize))
    {
      // is this the packet we were waiting for?
      if (xmbuf[1] == seqnum)
      {
        // Write to the file.
				if (!spiFSWrite(recvFile, (UINT8 *) &xmbuf[3], (INT16 *) &pktsize))
				{
          // Cancel transmission
          xmodem_flush();
          xmodem_out_func(CAN);
          xmodem_out_func(CAN);
          xmodem_out_func(CAN);
          return XMODEM_ERROR_FILEIO;   
				}

        // total bytes written
        totalbytes += pktsize;
        // next sequence number
        seqnum++;
        // reset retries
        retry = XMODEM_RETRY_LIMIT;
        // reply with ACK
        response = ACK;
        continue;
      }
      else if (xmbuf[1] == (unsigned char)(seqnum - 1))
      {
        // This is a retransmission of the last packet ACK and move on.
        response = ACK;
        continue;
      }
      else
      {
        // We are completely out of sync so cancel transmission.
        xmodem_flush();
        xmodem_out_func(CAN);
        xmodem_out_func(CAN);
        xmodem_out_func(CAN);
        return XMODEM_ERROR_OUTOFSYNC;
      }
    }
    else
    {
      // Packet was corrupt so NAK it and try again.
      retry--;
      xmodem_flush();
      response = NAK;
      continue;
    }
  }

  // Exceeded retry count.
  xmodem_flush();
  xmodem_out_func(CAN);
  xmodem_out_func(CAN);
  xmodem_out_func(CAN);
  return XMODEM_ERROR_RETRYEXCEED;
}

