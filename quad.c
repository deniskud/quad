/*22.03.19*/
#include <time.h>
#include <stdio.h>
//#include <conio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <lime/LimeSuite.h>
#include "wiringPi.h"
#include "wiringPiI2C.h"
///////////////////////////////////
int main(int argc, char** argv)
{
///////////////debug
clock_t start, end;
clock_t sstart, send;
double cpu_time_used;

//////////////
  unsigned int buffer_size = 5*1000*10;   //1000x50=10milisec@SR=5MHz 1/100 sec
  struct s16iq_sample_s {
    short i;
    short q;
  } __attribute__((packed));
  struct s16iq_sample_s *buffxa = (struct s16iq_sample_s*)malloc(sizeof(struct s16iq_sample_s) * buffer_size);
  struct s16iq_sample_s *buffxb = (struct s16iq_sample_s*)malloc(sizeof(struct s16iq_sample_s) * buffer_size);
  struct s16iq_sample_s *buffya = (struct s16iq_sample_s*)malloc(sizeof(struct s16iq_sample_s) * buffer_size);
  struct s16iq_sample_s *buffyb = (struct s16iq_sample_s*)malloc(sizeof(struct s16iq_sample_s) * buffer_size);

  struct s16iq_sample_s *bufftmp1 = (struct s16iq_sample_s*)malloc(sizeof(struct s16iq_sample_s) * buffer_size);
  struct s16iq_sample_s *bufftmp2 = (struct s16iq_sample_s*)malloc(sizeof(struct s16iq_sample_s) * buffer_size);
  if ( buffxa == NULL ||buffxb == NULL ||buffya == NULL ||buffyb == NULL ) {
    perror("malloc()");
    return 1;
  }
  int nb_samples=0;
  double gain = 24;    // 0..73
  unsigned int freq = 433000000; // ? 100fm
  double bandwidth_calibrating = 5000000;
  double sample_rate = 5000000;
  double host_sample_rate;         //
  int maxnumber=300;               // 1 sec

////////////////////////////////        INIT I2C         ////////////////
  int devlna[5]; 
  int extlnaon(int ch){if (ch>0) wiringPiI2CWriteReg8 (devlna[ch], 9, 0x7f);}  // set @channal lna=ON  att=0 filters=bypass 
  int extlnaoff(int ch){if (ch>0) wiringPiI2CWriteReg8 (devlna[ch], 9, 0xff);} // set  @channal lna=OFF att=0 filters=bypass
  int extlnaallon() {for (int i=1;i<5;i++) wiringPiI2CWriteReg8 (devlna[i], 9, 0x7f);}// set all lna=ON  att=0 filters=bypass 
  int extlnaalloff(){for (int i=1;i<5;i++) wiringPiI2CWriteReg8 (devlna[i], 9, 0xff);}// set all lna=OFF  att=0 filters=bypass     
  int setant1(){wiringPiI2CWriteReg8 (devlna[0], 9, 0x18);} // Set antenna pair 1-2 1-4 (xS  yS)
  int setant2(){wiringPiI2CWriteReg8 (devlna[0], 9, 0x0c);} // Set antenna pair 1-3 1-5 (xL  yL)
  int setant3(){wiringPiI2CWriteReg8 (devlna[0], 9, 0x71);} // Set antenna pair 8-7 8-6 (x2S y2S)
  int setant4(){wiringPiI2CWriteReg8 (devlna[0], 9, 0x53);} // Set antenna pair 8-5 8-3 (x2L e2L)
//////////////////// init
  wiringPiSetup();
  for (int i=0;i<5;i++) {
    devlna[i] = wiringPiI2CSetup(0x20+i);
    wiringPiI2CWriteReg8 (devlna[i], 0, 0);
  }
////////////////////////////////        INIT LIME       /////////////////
  int errcnt=0;
  lms_device_t* devicex = NULL;
  lms_device_t* devicey = NULL;
  int device_count = LMS_GetDeviceList(NULL);
  lms_info_str_t device_list[ device_count ];
  LMS_GetDeviceList(device_list);
  errcnt+=  LMS_Open(&devicex, device_list[ 0 ], NULL); //try open X device
  errcnt+=  LMS_Open(&devicey, device_list[ 1 ], NULL); //try open Y device
  errcnt+=  LMS_Reset(devicex); //   reset X-device
  errcnt+=  LMS_Reset(devicey); //   reset Y-device
  errcnt+=  LMS_Init(devicex);  //  default X-device
  errcnt+=  LMS_Init(devicey);  //  default Y-device
  errcnt+=  LMS_EnableChannel(devicex, LMS_CH_RX, 0, true); //enable RX-0 channel
  errcnt+=  LMS_EnableChannel(devicex, LMS_CH_RX, 1, true); //enable RX-1 channel
  errcnt+=  LMS_EnableChannel(devicey, LMS_CH_RX, 0, true); //enable RX-0 channel
  errcnt+=  LMS_EnableChannel(devicey, LMS_CH_RX, 1, true); //enable RX-1 channel
//  errcnt+=  LMS_EnableChannel(devicex, LMS_CH_TX, 0, false); //enable RX-0 channel
//  errcnt+=  LMS_EnableChannel(devicex, LMS_CH_TX, 1, false); //enable RX-1 channel
//  errcnt+=  LMS_EnableChannel(devicey, LMS_CH_TX, 0, false); //enable RX-0 channel
//  errcnt+=  LMS_EnableChannel(devicey, LMS_CH_TX, 1, false); //enable RX-1 channel
  errcnt+=  LMS_SetSampleRate(devicex, sample_rate, 0); //set sample rate for oall channals fo X-device
  errcnt+=  LMS_SetSampleRate(devicey, sample_rate, 0); //set sample rate for oall channals fo Y-device
  errcnt+=  LMS_SetAntenna( devicex, LMS_CH_RX, 0, 2 );//1=RX LNA_H 2=RX LNA_L 3=RX LNA_W 
  errcnt+=  LMS_SetAntenna( devicex, LMS_CH_RX, 1, 2 );//1=RX LNA_H 2=RX LNA_L 3=RX LNA_W 
  errcnt+=  LMS_SetAntenna( devicey, LMS_CH_RX, 0, 2 );//1=RX LNA_H 2=RX LNA_L 3=RX LNA_W 
  errcnt+=  LMS_SetAntenna( devicey, LMS_CH_RX, 1, 2 );//1=RX LNA_H 2=RX LNA_L 3=RX LNA_W 
  errcnt+=  LMS_Calibrate( devicex, LMS_CH_RX, 0, bandwidth_calibrating, 0 );
  errcnt+=  LMS_Calibrate( devicex, LMS_CH_RX, 1, bandwidth_calibrating, 0 );
  errcnt+=  LMS_Calibrate( devicey, LMS_CH_RX, 0, bandwidth_calibrating, 0 );
  errcnt+=  LMS_Calibrate( devicey, LMS_CH_RX, 1, bandwidth_calibrating, 0 );
  errcnt+=  LMS_SetLOFrequency( devicex, LMS_CH_RX, 0, freq);
  errcnt+=  LMS_SetLOFrequency( devicex, LMS_CH_RX, 1, freq);
  errcnt+=  LMS_SetLOFrequency( devicey, LMS_CH_RX, 0, freq);
  errcnt+=  LMS_SetLOFrequency( devicey, LMS_CH_RX, 1, freq);
  errcnt+=  LMS_SetGaindB( devicex, LMS_CH_RX, 0, gain );
  errcnt+=  LMS_SetGaindB( devicex, LMS_CH_RX, 1, gain );
  errcnt+=  LMS_SetGaindB( devicey, LMS_CH_RX, 0, gain );
  errcnt+=  LMS_SetGaindB( devicey, LMS_CH_RX, 1, gain );
  if (  errcnt!=0 ){printf("errors%d\n",errcnt);return -1;}
  else printf("Init ok both devices all channals\n");    
///////////// init & start stream
  lms_stream_t rx_streamxa = {
    .channel = 0,    
    .fifoSize = buffer_size * sizeof(*buffxa),
    .throughputVsLatency = 0.5,
    .isTx = LMS_CH_RX,
    .dataFmt = LMS_FMT_I16
  };
  lms_stream_t rx_streamxb = {
    .channel = 1,
    .fifoSize = buffer_size * sizeof(*buffxb),
    .throughputVsLatency = 0.5,
    .isTx = LMS_CH_RX,
    .dataFmt = LMS_FMT_I16
  };
  lms_stream_t rx_streamya = {
    .channel = 0,
    .fifoSize = buffer_size * sizeof(*buffya),
    .throughputVsLatency = 0.5,
    .isTx = LMS_CH_RX,
    .dataFmt = LMS_FMT_I16
  };
  lms_stream_t rx_streamyb = {
    .channel = 1,
    .fifoSize = buffer_size * sizeof(*buffyb),
    .throughputVsLatency = 0.5,
    .isTx = LMS_CH_RX,
    .dataFmt = LMS_FMT_I16
  };  
  if (  LMS_SetupStream(devicex, &rx_streamxa)!=0) printf("setup XA ERROR\n");
  else printf("setup XA Ok.\n");
  if (  LMS_SetupStream(devicex, &rx_streamxb)!=0) printf("setup XB ERROR\n");
  else printf("setup XB Ok.\n");
  if (  LMS_SetupStream(devicey, &rx_streamya)!=0) printf("setup YA ERROR\n");
  else printf("setup YA Ok.\n");
  if (  LMS_SetupStream(devicey, &rx_streamyb)!=0) printf("setup YB ERROR\n");
  else printf("setup XB Ok.\n");
  if (  LMS_StartStream(&rx_streamxa)!=0) printf("start stream XA ERROR\n");
  else printf("Start stream XA Ok.\n");
  if (  LMS_StartStream(&rx_streamxb)!=0) printf("start stream XB ERROR\n");
  else printf("Start stream XB Ok.\n");
  if (  LMS_StartStream(&rx_streamya)!=0) printf("start stream YA ERROR\n");
  else printf("Start stream YA Ok.\n");
  if (  LMS_StartStream(&rx_streamyb)!=0) printf("start stream YB ERROR\n");
  else printf("Start stream YB Ok.\n");

  double tmpa=0,tmpb=0,tmpc=0,tmpd=0;
  double maxinbuf1=0,midinbuf1=0;
  double maxinbuf2=0, midinbuf2=0;
  double maxinbuf3=0,midinbuf3=0;
  double maxinbuf4=0, midinbuf4=0;
  double amp1=0, amp2=0, amp3=0, amp4=0;
  lms_stream_meta_t metadata1, metadata2, metadata3, metadata4;
  long int prevtsx=0;
  long int prevtsy=0;
  printf("maxnumber=%d\n",maxnumber);
  sstart = clock(); //debug time
  int mn = maxnumber;
  char chin;
  extlnaalloff();// all LNA=OFF
  while( maxnumber!=0 ) {   ///////////////////    start main loop
    maxnumber--;
    start = clock(); //debug time
    nb_samples = LMS_RecvStream( &rx_streamxa, buffxa, buffer_size, &metadata1, 1000 );
    nb_samples = LMS_RecvStream( &rx_streamxb, buffxb, buffer_size, &metadata2, 1000 );
    nb_samples = LMS_RecvStream( &rx_streamya, buffya, buffer_size, &metadata3, 1000 );
    nb_samples = LMS_RecvStream( &rx_streamyb, buffyb, buffer_size, &metadata4, 1000 );
    maxinbuf1=maxinbuf2=maxinbuf3=maxinbuf4=0;
    midinbuf1=midinbuf2=midinbuf3=midinbuf4=0;
    for (int pntr=0; pntr<buffer_size; pntr++) {
      tmpa=buffxa[pntr].i*buffxa[pntr].i+buffxa[pntr].q*buffxa[pntr].q;
      tmpb=buffxb[pntr].i*buffxb[pntr].i+buffxb[pntr].q*buffxb[pntr].q;
      tmpc=buffya[pntr].i*buffya[pntr].i+buffya[pntr].q*buffya[pntr].q;
      tmpd=buffyb[pntr].i*buffyb[pntr].i+buffyb[pntr].q*buffyb[pntr].q;
      midinbuf1+=tmpa/(buffer_size);
      midinbuf2+=tmpb/(buffer_size);
      midinbuf3+=tmpc/(buffer_size);
      midinbuf4+=tmpd/(buffer_size);
      if (tmpa>maxinbuf1) maxinbuf1=tmpa;
      if (tmpb>maxinbuf2) maxinbuf2=tmpb;
      if (tmpc>maxinbuf3) maxinbuf3=tmpc;
      if (tmpd>maxinbuf4) maxinbuf4=tmpd;
    }
 
   printf(" %lld %lld %lld %lld ",metadata1.timestamp, metadata3.timestamp, (metadata1.timestamp-prevtsx),  (metadata3.timestamp-prevtsy));
    printf("%5.0f %3.0f   %5.0f %3.0f   %5.0f %3.0f   %5.0f %3.0f  --",sqrt(maxinbuf1),sqrt(midinbuf1),sqrt(maxinbuf2),sqrt(midinbuf2),sqrt(maxinbuf3),sqrt(midinbuf3),sqrt(maxinbuf4),sqrt(midinbuf4));
    prevtsx=metadata1.timestamp;
    prevtsy=metadata3.timestamp;
    end = clock(); //debug time
    cpu_time_used = 1000*((double) (end - start))/ CLOCKS_PER_SEC;;
    printf("iteration time=%.3fmilisec  speed=%.1fMbit/sec\n",cpu_time_used,64*buffer_size/1000/cpu_time_used);
  } //////////////////////////////////////////////// end mainloop	
  send = clock(); //debug time
  cpu_time_used = 1000*((double)(send - sstart)) / CLOCKS_PER_SEC;
  printf("all iteration time = %.3fmilisec middle=%.1fMbps\n",cpu_time_used,mn*64*buffer_size/1000/cpu_time_used);
  LMS_StopStream(&rx_streamxa);
  LMS_StopStream(&rx_streamxb);
  LMS_StopStream(&rx_streamya);
  LMS_StopStream(&rx_streamyb);
  LMS_DestroyStream(devicex, &rx_streamxa);
  LMS_DestroyStream(devicex, &rx_streamxb);
  LMS_DestroyStream(devicey, &rx_streamya);
  LMS_DestroyStream(devicey, &rx_streamyb);
  free( buffxa );
  free( buffxb );
  free( buffya );
  free( buffyb );
  LMS_Close(devicex);
  LMS_Close(devicey);
  return 0;
}
