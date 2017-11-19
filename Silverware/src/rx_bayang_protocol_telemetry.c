/*
The MIT License (MIT)

Copyright (c) 2016 silverx

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/


#include "binary.h"
#include "drv_spi.h"
#include "project.h"
#include "xn297.h"
#include "drv_time.h"
#include <stdio.h>
#include "config.h"
#include "defines.h"

#include "rx_bayang.h"

#include "util.h"


#define RX_MODE_NORMAL RXMODE_NORMAL
#define RX_MODE_BIND RXMODE_BIND
// radio settings

// packet period in uS
#define PACKET_PERIOD 3000
#define PACKET_PERIOD_TELEMETRY 5000

// was 250 ( uS )
#define PACKET_OFFSET 0

#ifdef USE_STOCK_TX
#undef PACKET_PERIOD
#define PACKET_PERIOD 2000
#undef PACKET_OFFSET
#define PACKET_OFFSET 0
#endif

// how many times to hop ahead if no reception
#define HOPPING_NUMBER 4


#if defined(RX_BAYANG_PROTOCOL_TELEMETRY) || defined(RX_BAYANG_PROTOCOL_TELEMETRY_PID) //extra condition for PID displaying (added by silverAG)

extern float rx[4];
extern char aux[AUXNUMBER];
extern char lastaux[AUXNUMBER];
extern char auxchange[AUXNUMBER];


char lasttrim[4];
char rfchannel[4];
//int rxaddress[5]; // not used (!)
//int rxmode = 0; //disabled for enabling BLE and Devo TLM
extern int rxmode; //added for enabling BLE and Devo TLM
int rf_chan = 0;



void writeregs(uint8_t data[], uint8_t size)
{
    spi_cson();
    for (uint8_t i = 0; i < size; i++)
      {
          spi_sendbyte(data[i]);
      }
    spi_csoff();
}



void rx_init_tlm()
{


// always on (CH_ON) channel set 1
    aux[AUXNUMBER - 2] = 1;
// always off (CH_OFF) channel set 0
    aux[AUXNUMBER - 1] = 0;
#ifdef AUX1_START_ON
    aux[CH_AUX1] = 1;
#endif


#ifdef RADIO_XN297L

#ifndef TX_POWER
#define TX_POWER 7
#endif
	
// Gauss filter amplitude - lowest
static uint8_t demodcal[2] = { 0x39 , B00000001 };
writeregs( demodcal , sizeof(demodcal) );

// powerup defaults
//static uint8_t rfcal2[7] = { 0x3a , 0x45 , 0x21 , 0xef , 0xac , 0x3a , 0x50};
//writeregs( rfcal2 , sizeof(rfcal2) );
	
#define XN_TO_RX B10001111
#define XN_TO_TX B10000010
#define XN_POWER B00000001|((TX_POWER&7)<<3)

#endif



#ifdef RADIO_XN297
    static uint8_t bbcal[6] = { 0x3f, 0x4c, 0x84, 0x6F, 0x9c, 0x20 };
    writeregs(bbcal, sizeof(bbcal));
// new values
    static uint8_t rfcal[8] =
        { 0x3e, 0xc9, 0x9a, 0xA0, 0x61, 0xbb, 0xab, 0x9c };
    writeregs(rfcal, sizeof(rfcal));

// 0xa7 0x03
    static uint8_t demodcal[6] =
        { 0x39, 0x0b, 0xdf, 0xc4, B00100111, B00000000 };
    writeregs(demodcal, sizeof(demodcal));


#ifndef TX_POWER
#define TX_POWER 3
#endif


#define XN_TO_RX B00001111
#define XN_TO_TX B00000010
#define XN_POWER (B00000001|((TX_POWER&3)<<1))
#endif

    delay(100);

// write rx address " 0 0 0 0 0 "
        
   static uint8_t rxaddr[6] = { 0x2a , 0 , 0 , 0 , 0 , 0  };
   writeregs( rxaddr , sizeof(rxaddr) );
    
    xn_writereg(EN_AA, 0);      // aa disabled
    xn_writereg(EN_RXADDR, 1);  // pipe 0 only
    xn_writereg(RF_SETUP, XN_POWER);    // power / data rate / lna
    xn_writereg(RX_PW_P0, 15);  // payload size
    xn_writereg(SETUP_RETR, 0); // no retransmissions ( redundant?)
    xn_writereg(SETUP_AW, 3);   // address size (5 bytes)
    xn_command(FLUSH_RX);
    xn_writereg(RF_CH, 0);      // bind on channel 0


#ifdef RADIO_XN297L
    xn_writereg(0x1d, B00111000);   // 64 bit payload , software ce
    spi_cson();
    spi_sendbyte(0xFD);         // internal CE high command
    spi_sendbyte(0);            // required for above
    spi_csoff();
#endif

#ifdef RADIO_XN297
    xn_writereg(0x1d, B00011000);   // 64 bit payload , software ce
#endif

    xn_writereg(0, XN_TO_RX);   // power up, crc enabled, rx mode

#ifdef RADIO_CHECK
    int rxcheck = xn_readreg(0x0f); // rx address pipe 5   
    // should be 0xc6
    extern void failloop(int);
    if (rxcheck != 0xc6)
        failloop(3);
#endif
}




// re-initializing TX address for switching back to Devo telemetry - added by silverAG
#if ((defined(RX_BAYANG_PROTOCOL_TELEMETRY) || defined(RX_BAYANG_PROTOCOL_TELEMETRY_PID)) && defined(RX_BAYANG_BLE_APP))
void rx_init_tlm2()
{
	delay(100);
	extern uint8_t rxaddr_global[6];  // original TX address - filled up at bind code where rxaddr array is filled also
                      
  // write original rx address
  writeregs( rxaddr_global , sizeof(rxaddr_global) );
  rxaddr_global[0] = 0x30; // tx register ( write ) number
                      
  // write original tx address
  writeregs( rxaddr_global , sizeof(rxaddr_global) );
	
	xn_writereg(0x25, rfchannel[rf_chan]);    // Set channel frequency 
	delay(100);
}
#endif














//#define RXDEBUG

#ifdef RXDEBUG
unsigned long packettime;
int channelcount[4];
int failcount;
int skipstats[12];
int afterskip[12];

#warning "RX debug enabled"
#endif

int packetrx;
int packetpersecond;


void send_telemetry(void);
void nextchannel(void);

int loopcounter = 0;
unsigned int send_time;
int telemetry_send = 0;
int oldchan = 0;

#define TELEMETRY_TIMEOUT 10000

void beacon_sequence()
{
    static int beacon_seq_state = 0;

    switch (beacon_seq_state)
      {
          case 0:
              // send data
              telemetry_send = 1;
              send_telemetry();
              beacon_seq_state++;
              break;

          case 1:
              // wait for data to finish transmitting
              if ((xn_readreg(0x17) & B00010000))
                {
                    xn_writereg(0, XN_TO_RX);
                    beacon_seq_state = 0;
                    telemetry_send = 0;
                    nextchannel();
                }
              else
                {   // if it takes too long we get rid of it
                    if (gettime() - send_time > TELEMETRY_TIMEOUT)
                      {
                          xn_command(FLUSH_TX);
                          xn_writereg(0, XN_TO_RX);
                          beacon_seq_state = 0;
                          telemetry_send = 0;
                      }
                }
              break;

          default:
              beacon_seq_state = 0;
              break;



      }

}



extern int lowbatt;
extern float vbattfilt;
extern float vbatt_comp;

void send_telemetry()
{

    int txdata[15];
    for (int i = 0; i < 15; i++)
        txdata[i] = i;
    txdata[0] = 133;
    txdata[1] = lowbatt;

// PID code start (added by silverAG)
// ----------------------------------
#ifdef RX_BAYANG_PROTOCOL_TELEMETRY_PID
	extern int tlm_or_pid;
	if (tlm_or_pid)
	{
	extern float pidkp[]; // current_PID_for_display = 0, 1, 2
	extern float pidki[]; // current_PID_for_display = 3, 4, 5
	extern float pidkd[]; // current_PID_for_display = 6, 7, 8
	extern float apidkp[]; // current_PID_for_display = 9, 10
	extern float apidki[]; //  current_PID_for_display = 11, 12
	extern float apidkd[]; //  current_PID_for_display = 13, 14

	extern int current_pid_term; //0 = pidkp, 1 = pidki, 2 = pidkd
	extern int current_pid_axis; //0 = roll, 1 = pitch, 2 = yaw
  int selectedPID = ((current_pid_term)*3)+(current_pid_axis);
	unsigned long pid_for_display = 0;

#ifdef COMBINE_PITCH_ROLL_PID_TUNING
 if (selectedPID == 1)	selectedPID =2;
 if (selectedPID == 4)	selectedPID =5;
 if (selectedPID == 7)	selectedPID =8;
#endif

		int PIDposition = 0;
  switch ( selectedPID )
 {
	 case 0:pid_for_display =(uint16_t)(pidkp[0]*100.0f);PIDposition=11;break;
	 case 1:pid_for_display =(uint16_t)(pidkp[1]*100.0f);PIDposition=12;break;
	 case 2:pid_for_display =(uint16_t)(pidkp[2]*100.0f);PIDposition=13;break;
	 case 3:pid_for_display =(uint16_t)(pidki[0]*100.0f);PIDposition=21;break;
	 case 4:pid_for_display =(uint16_t)(pidki[1]*100.0f);PIDposition=22;break;
	 case 5:pid_for_display =(uint16_t)(pidki[2]*100.0f);PIDposition=23;break;
	 case 6:pid_for_display =(uint16_t)(pidkd[0]*100.0f);PIDposition=31;break;
	 case 7:pid_for_display =(uint16_t)(pidkd[1]*100.0f);PIDposition=32;break;
	 case 8:pid_for_display =(uint16_t)(pidkd[2]*100.0f);PIDposition=33;break;	
 }	


		int vbattx = pid_for_display;
		txdata[3] = (vbattx >> 8) & 0xff;
    txdata[4] = vbattx & 0xff;

		vbattx = 900+(PIDposition);
    txdata[5] = (vbattx >> 8) & 0xff;
    txdata[6] = vbattx & 0xff;
	}	
else
// ------------
// PID code end	
#endif
{	
    int vbatt = vbattfilt * 100;
// battery volt filtered    
    txdata[3] = (vbatt >> 8) & 0xff;
    txdata[4] = vbatt & 0xff;

    vbatt = vbatt_comp * 100;
// battery volt compensated 
    txdata[5] = (vbatt >> 8) & 0xff;
    txdata[6] = vbatt & 0xff;
}


    int temp = packetpersecond / 2;
    if (temp > 255)
        temp = 255;

    txdata[7] = temp;           // rx strenght

    if (lowbatt)
        txdata[3] |= (1 << 3);

    int sum = 0;
    for (int i = 0; i < 14; i++)
      {
          sum += txdata[i];
      }

    txdata[14] = sum;

    xn_command(FLUSH_TX);

    xn_writereg(0, XN_TO_TX);

    xn_writepayload(txdata, 15);

    send_time = gettime();

    return;
}



static char checkpacket()
{
    int status = xn_readreg(7);

    if (status & (1 << MASK_RX_DR))
      {                         // rx clear bit
          // this is not working well
          // xn_writereg( STATUS , (1<<MASK_RX_DR) );
          //RX packet received
          //return 1;
      }
    if ((status & B00001110) != B00001110)
      {
          // rx fifo not empty        
          return 2;
      }

    return 0;
}


int rxdata[15];


float packettodata(int *data)
{
    return (((data[0] & 0x0003) * 256 + data[1]) - 512) * 0.001953125;
}


static int decodepacket(void)
{
    if (rxdata[0] == 165)
      {
          int sum = 0;
          for (int i = 0; i < 14; i++)
            {
                sum += rxdata[i];
            }
          if ((sum & 0xFF) == rxdata[14])
            {
                rx[0] = packettodata(&rxdata[4]);
                rx[1] = packettodata(&rxdata[6]);
                rx[2] = packettodata(&rxdata[10]);
                // throttle     
                rx[3] =
                    ((rxdata[8] & 0x0003) * 256 +
                     rxdata[9]) * 0.000976562f;

#ifndef DISABLE_EXPO
                rx[0] = rcexpo(rx[0], EXPO_XY);
                rx[1] = rcexpo(rx[1], EXPO_XY);
                rx[2] = rcexpo(rx[2], EXPO_YAW);
#endif



#ifdef USE_STOCK_TX
                char trims[4];
                trims[0] = rxdata[6] >> 2;
                trims[1] = rxdata[4] >> 2;

                for (int i = 0; i < 2; i++)
                    if (trims[i] != lasttrim[i])
                      {
                          aux[CH_PIT_TRIM + i] = trims[i] > lasttrim[i];
                          lasttrim[i] = trims[i];
                      }
#else
                aux[CH_INV] = (rxdata[3] & 0x80) ? 1 : 0;   // inverted flag

                aux[CH_VID] = (rxdata[2] & 0x10) ? 1 : 0;

                aux[CH_PIC] = (rxdata[2] & 0x20) ? 1 : 0;
#endif

                aux[CH_FLIP] = (rxdata[2] & 0x08) ? 1 : 0;

                aux[CH_EXPERT] = (rxdata[1] == 0xfa) ? 1 : 0;

                aux[CH_HEADFREE] = (rxdata[2] & 0x02) ? 1 : 0;

                aux[CH_RTH] = (rxdata[2] & 0x01) ? 1 : 0;   // rth channel



                for (int i = 0; i < AUXNUMBER - 2; i++)
                  {
                      auxchange[i] = 0;
                      if (lastaux[i] != aux[i])
                          auxchange[i] = 1;
                      lastaux[i] = aux[i];
                  }

                return 1;       // valid packet 
            }
          return 0;             // sum fail
      }
    return 0;                   // first byte different
}



void nextchannel()
{
    rf_chan++;
    rf_chan &= 3; // same as %4
    xn_writereg(0x25, rfchannel[rf_chan]);
}


unsigned long lastrxtime;
unsigned long failsafetime;
unsigned long secondtimer;

//int failsafe = 0; //disabled for enabling BLE and Devo TLM
extern int failsafe; //added for enabling BLE and Devo TLM


unsigned int skipchannel = 0;
int lastrxchan;
int timingfail = 0;
int telemetry_enabled = 0;
int packet_period = PACKET_PERIOD;

void checkrx_tlm(void)
{
    int packetreceived = checkpacket();
    int pass = 0;
    if (packetreceived)
      {
          if (rxmode == RX_MODE_BIND)
            {                   // rx startup , bind mode
                xn_readpayload(rxdata, 15);

                if (rxdata[0] == 0xa4 || rxdata[0] == 0xa3)
                  {             // bind packet
                      if (rxdata[0] == 0xa3)
                        {
                            telemetry_enabled = 1;
                            packet_period = PACKET_PERIOD_TELEMETRY;
                        }
                        
                      rfchannel[0] = rxdata[6];
                      rfchannel[1] = rxdata[7];
                      rfchannel[2] = rxdata[8];
                      rfchannel[3] = rxdata[9];
                        

                      uint8_t rxaddr[6] = { 0x2a ,  };
											extern uint8_t rxaddr_global[6];
											
                      for ( int i = 1 ; i < 6; i++)
                      {
                        rxaddr[i] = rxdata[i];
												rxaddr_global[i] = rxdata[i];
                      }
                      // write new rx address
                      writeregs( rxaddr , sizeof(rxaddr) );
                      rxaddr[0] = 0x30; // tx register ( write ) number
                      
                      // write new tx address
                      writeregs( rxaddr , sizeof(rxaddr) );

                      xn_writereg(0x25, rfchannel[rf_chan]);    // Set channel frequency 
                      rxmode = RX_MODE_NORMAL;

#ifdef SERIAL
                      printf(" BIND \n");
#endif
                  }
            }
          else
            {                   // normal mode  
#ifdef RXDEBUG
                channelcount[rf_chan]++;
                packettime = gettime() - lastrxtime;

                if (skipchannel && !timingfail)
                    afterskip[skipchannel]++;
                if (timingfail)
                    afterskip[0]++;

#endif

                unsigned long temptime = gettime();

                xn_readpayload(rxdata, 15);
                pass = decodepacket();

                if (pass)
                  {
                      packetrx++;
                      if (telemetry_enabled)
                          beacon_sequence();
                      skipchannel = 0;
                      timingfail = 0;
                      lastrxchan = rf_chan;
                      lastrxtime = temptime;
                      failsafetime = temptime;
                      failsafe = 0;
                      if (!telemetry_send)
                          nextchannel();
                  }
                else
                  {
#ifdef RXDEBUG
                      failcount++;
#endif
                  }

            }                   // end normal rx mode

      }                         // end packet received

// finish sending if already started
    if (telemetry_send)
        beacon_sequence();

    unsigned long time = gettime();


    if (time - lastrxtime > (HOPPING_NUMBER * packet_period + 1000)
        && rxmode != RX_MODE_BIND)
      {
          //  channel with no reception   
          lastrxtime = time;
          // set channel to last with reception
          if (!timingfail)
              rf_chan = lastrxchan;
          // advance to next channel
          nextchannel();
          // set flag to discard packet timing
          timingfail = 1;
      }

    if (!timingfail && !telemetry_send && skipchannel < HOPPING_NUMBER + 1
        && rxmode != RX_MODE_BIND)
      {
          unsigned int temp = time - lastrxtime;

          if (temp > 1000
              && (temp - (PACKET_OFFSET)) / ((int) packet_period) >=
              (skipchannel + 1))
            {
                nextchannel();
#ifdef RXDEBUG
                skipstats[skipchannel]++;
#endif
                skipchannel++;
            }
      }

    if (time - failsafetime > FAILSAFETIME)
      {                         //  failsafe
          failsafe = 1;
          rx[0] = 0;
          rx[1] = 0;
          rx[2] = 0;
          rx[3] = 0;
      }

    if (gettime() - secondtimer > 1000000)
      {
          packetpersecond = packetrx;
          packetrx = 0;
          secondtimer = gettime();
      }


}


#endif
