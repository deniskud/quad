/*16.05.17*/
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <lime/LimeSuite.h>
#include "wiringPi.h"
#include "wiringPiI2C.h"

#define XA 0
#define XB 1
#define YA 2
#define YB 3

///////////////////////////////////
int main(int argc, char** argv)
{
///////////////debug
clock_t start, end;
clock_t sstart, send;
double cpu_time_used;

//////////////
  unsigned int buffer_size = 5*1000*10;   //1000x50=10milisec@SR=5MHz 1/100 sec
  int maxnumber=20;               // 1/10 sec
  typedef struct {
    short lna;  // 1-on;0-off; +18dB
    short flt;  // 1-8
    short att;  // att 0-15
    short bnd; // band 
    short reg; // register i2c
  }extlna_t;
  extlna_t curext[4];
  struct s16iq_sample_s {
    short i;
    short q;
  } __attribute__((packed));
//  struct s16iq_sample_s *buffxa = (struct s16iq_sample_s*)malloc(sizeof(struct s16iq_sample_s) * buffer_size);
//  struct s16iq_sample_s *buffxb = (struct s16iq_sample_s*)malloc(sizeof(struct s16iq_sample_s) * buffer_size);
//  struct s16iq_sample_s *buffya = (struct s16iq_sample_s*)malloc(sizeof(struct s16iq_sample_s) * buffer_size);
//  struct s16iq_sample_s *buffyb = (struct s16iq_sample_s*)malloc(sizeof(struct s16iq_sample_s) * buffer_size);
//  struct s16iq_sample_s *bufftmp2 = (struct s16iq_sample_s*)malloc(sizeof(struct s16iq_sample_s) * buffer_size);

  struct s16iq_sample_s *buff[4];
  for (int i=0;i<4;i++) buff[i] = (struct s16iq_sample_s*)malloc(sizeof(struct s16iq_sample_s) * buffer_size);
  if ( buff[XA] == NULL ||buff[XB] == NULL ||buff[YA] == NULL ||buff[YB] == NULL ) {
    perror("malloc()");
    return 1;
  }
  
  int nb_samplesxa,nb_samplesxb,nb_samplesya,nb_samplesyb;
  int nb_samples[4];
  double gain = 0;    // 0..73
  unsigned int freq = 2457000000; // ? 100fm
  double bandwidth_calibrating = 5000000;
  double sample_rate = 5000000;
  double host_sample_rate;         //

//////////////////////////////// I2C  ////////////////
  union i2c_smbus_data{
    uint8_t  byte ;
    uint16_t word ;
    uint8_t  block [34] ;// block [0] is used for length + one more for PEC
  };
  int devlna[5];
///// LNA on/off 
  int extlnaon(int ch){
    if (ch>0 & ch<5) {
      int reg=wiringPiI2CReadReg8(devlna[ch],9);
      if (reg>127) wiringPiI2CWriteReg8 (devlna[ch], 9, (reg-128));
    }
  }  // set @channal lna=ON
  int extlnaoff(int ch){
    if (ch>0 & ch<5) {
      int reg=wiringPiI2CReadReg8(devlna[ch],9);
      if (reg<=127) wiringPiI2CWriteReg8 (devlna[ch], 9, (reg+128));
    }
  } // set  @channal lna=OFF
  int extlnaallon() {
    for (int i=1;i<5;i++) wiringPiI2CWriteReg8 (devlna[i], 9, 0x7f);
  }// set all lna=ON  att=0 filters=bypass 
  int extlnaalloff(){
    for (int i=1;i<5;i++) wiringPiI2CWriteReg8 (devlna[i], 9, 0xff);
  }// set all lna=OFF  att=0 filters=bypass     
//////   ATT on/off
  int extatton(int ch){
    if (ch>0 & ch<5) {
      int reg=wiringPiI2CReadReg8(devlna[ch],9);
      wiringPiI2CWriteReg8 (devlna[ch], 9, (reg&135));
    }
  } // set  @channal att -16dB
  int extattoff(int ch){
    if (ch>0 & ch<5) {
      int reg=wiringPiI2CReadReg8(devlna[ch],9);
      wiringPiI2CWriteReg8 (devlna[ch], 9, (reg&135+120));
    }
  } // set  @channal att 0dB
////////  BPF/BAND
  int setfilter(int ch, int n){
    if (n<1 || n>8) n=8;
    int reg=wiringPiI2CReadReg8(devlna[ch],9);
    int bpf[9]={0,3,1,2,0,4,6,5,7};  //array of bytes for i2c reg
    wiringPiI2CWriteReg8 (devlna[ch], 9, (reg&248 + bpf[n] ) );
    return (reg&248 + bpf[n] ); // 
  }
  int setband(int ch,int band){
    int reg=wiringPiI2CReadReg8(devlna[ch],9);
    int bands[7]={3,5,20,28,8,7,1}; // array of availeble bands
    int bpf[8] = {3,1,2,0,4,6,5,7}; // array of bytes for i2c reg
    int i=0;
    while ( i<7 && bands[i]!=band) i++;
    wiringPiI2CWriteReg8 (devlna[ch], 9, (reg&248 + bpf[i] ) );
    return i+1; //number of filter BPFn 1-8
  }
///////// Antenns
  int setant(int n){ // 0-3
    if (n<0 || n>3) return -1;
    short reg[4]={0x18,0x1c,0x71,0x53};
    wiringPiI2CWriteReg8 (devlna[0], 9, reg[n]);
  }
  int setshort1(){wiringPiI2CWriteReg8 (devlna[0], 9, 0x18);} // Set antenna pair 1-2 1-4 (xS  yS)
  int setlong1(){wiringPiI2CWriteReg8 (devlna[0], 9, 0x0c);} // Set antenna pair 1-3 1-5 (xL  yL)
  int setshort2(){wiringPiI2CWriteReg8 (devlna[0], 9, 0x71);} // Set antenna pair 8-7 8-6 (x2S y2S)
  int setlong2(){wiringPiI2CWriteReg8 (devlna[0], 9, 0x53);} // Set antenna pair 8-5 8-3 (x2L y2L)
//////// setgain
  int closetarget(){     //LNA=Off ATT=On
    for(int i=1;i<5;i++){
      extatton(i);
      extlnaoff(i);
    };
  };
  int normaltarget(){    //LNA=Off ATT=Off
    for(int i=1;i<5;i++){
      extattoff(i);
      extlnaoff(i);
    };
  };
  int fartarget(){    //LNA=On ATT=Off
    for(int i=1;i<5;i++){
      extattoff(i);
      extlnaon(i);
    };
  };

//////////////////// init comutator & 4xLNA-ATT-BPF
  printf("I2C init:");
  wiringPiSetup();
  printf("Ok\n"); 
  printf("comutator init:");
  devlna[0] = wiringPiI2CSetup(0x24);    //return devcie comutator
  wiringPiI2CWriteReg8 (devlna[0], 0, 0); // init comutator
  wiringPiI2CWriteReg8 (devlna[0], 8, 18); // init comutator
  printf("Ok\n"); 
  for (int i=1;i<5;i++) {
  printf("LNA-ATT-BPF %d init: ",i);
    devlna[i] = wiringPiI2CSetup(0x1f+i); //Init LNA1-4
    wiringPiI2CWriteReg8 (devlna[i], 0, 0);
    wiringPiI2CWriteReg8 (devlna[i], 9, 255);
    printf("Ok\n"); 
  }
//
//  extlnaon(3);
//  extlnaon(4);
//  extatton(3);
//  extattoff(4);
//  setfilter(1,0);
//  setfilter(3,5);
//  printf("%d\n",setband(2,20));
//
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
  lms_stream_t rx_stream[4];
///////////
  rx_stream[XA].channel = 0;
  rx_stream[XA].fifoSize = buffer_size * sizeof(*buff[XA]);
  rx_stream[XA].throughputVsLatency = 0.5;
  rx_stream[XA].isTx = LMS_CH_RX;
  rx_stream[XA].dataFmt = LMS_FMT_I16;
///////////
  rx_stream[XB].channel = 1;
  rx_stream[XB].fifoSize = buffer_size * sizeof(*buff[XB]);
  rx_stream[XB].throughputVsLatency = 0.5;
  rx_stream[XB].isTx = LMS_CH_RX;
  rx_stream[XB].dataFmt = LMS_FMT_I16;
///////////
  rx_stream[YA].channel = 0;
  rx_stream[YA].fifoSize = buffer_size * sizeof(*buff[YA]);
  rx_stream[YA].throughputVsLatency = 0.5;
  rx_stream[YA].isTx = LMS_CH_RX;
  rx_stream[YA].dataFmt = LMS_FMT_I16;
///////////
  rx_stream[YB].channel = 1;
  rx_stream[YB].fifoSize = buffer_size * sizeof(*buff[YB]);
  rx_stream[YB].throughputVsLatency = 0.5;
  rx_stream[YB].isTx = LMS_CH_RX;
  rx_stream[YB].dataFmt = LMS_FMT_I16;
///////////
  if (  LMS_SetupStream(devicex, &rx_stream[XA])!=0) printf("setup XA ERROR\n");
  else printf("stream setup XA Ok.\n");
  if (  LMS_SetupStream(devicex, &rx_stream[XB])!=0) printf("setup XB ERROR\n");
  else printf("stream setup XB Ok.\n");
  if (  LMS_SetupStream(devicey, &rx_stream[YA])!=0) printf("setup YA ERROR\n");
  else printf("stream setup YA Ok.\n");
  if (  LMS_SetupStream(devicey, &rx_stream[YB])!=0) printf("setup YB ERROR\n");
  else printf("stream setup XB Ok.\n");
////////////// start stream
  if (  LMS_StartStream(&rx_stream[XA])!=0) printf("start stream XA ERROR\n");
  else printf("Start stream XA Ok.\n");
  if (  LMS_StartStream(&rx_stream[XB])!=0) printf("start stream XB ERROR\n");
  else printf("Start stream XB Ok.\n");
  if (  LMS_StartStream(&rx_stream[YA])!=0) printf("start stream YA ERROR\n");
  else printf("Start stream YA Ok.\n");
  if (  LMS_StartStream(&rx_stream[YB])!=0) printf("start stream YB ERROR\n");
  else printf("Start stream YB Ok.\n");
////////////////////////////////////////////////////////////////////////////////
  lms_stream_meta_t metadata[4];//, metadata2, metadata3, metadata4;
  for (int i=0;i<4;i++) metadata[i].timestamp=0;
  long int prevtsx=0;
  long int prevtsy=0;
//////////////////
  int getbuf(short n){  // 0,1,2,3
    nb_samples[n]=LMS_RecvStream( &rx_stream[n], buff[n], buffer_size, &metadata[n], 1000 );
    return nb_samples[n];
  };
  int getbuf2(short flag){ // flag 0/1 0-first device 1-second
    int ch1=0,ch2=1;
    if (flag!=0) {ch1=2,ch2=3;}
    if (getbuf(ch1)==getbuf(ch2)) return nb_samples[ch1];
    else printf("returned different buffers size: Xa=%d Xb=%d\n",nb_samples[ch1],nb_samples[ch2]);
    return -1; // 
  }
  int getbuf4(){
    for (short i=0;i<4;i++) getbuf(i);
//      nb_samples[i] = LMS_RecvStream( &rx_stream[i], buff[i], buffer_size, &metadata[i], 1000 );
    if (nb_samples[0]!=nb_samples[1] || nb_samples[2]!=nb_samples[3] || nb_samples[0]!=nb_samples[2])
      printf("returned different buffers size: Xa=%d Xb=%d Ya=%d Yb=%d\n",nb_samples[0],nb_samples[1],nb_samples[2],nb_samples[3]);
    else return nb_samples[0];
    return -1;
  };
  int maxinbuf(short n){
    unsigned long long tmp,max=0;
    for (int pntr=0; pntr<buffer_size; pntr++) {
      tmp=buff[n][pntr].i*buff[n][pntr].i+buff[n][pntr].q*buff[n][pntr].q;//+buffxa[pntr].q*buffxa[pntr].q;
      if (max<tmp) max=tmp;
    };
    return (int)round(sqrt(max));
  };
  int midinbuf(short n){
    double tmp,mid=0;
    for (int pntr=0; pntr<buffer_size; pntr++) {
      tmp=buff[n][pntr].i*buff[n][pntr].i+buff[n][pntr].q*buff[n][pntr].q;//+buffxa[pntr].q*buffxa[pntr].q;
      mid+=tmp/buffer_size;
    };
    return (int)round(sqrt(mid));
  };
  double snrinbuf(short n, int window){
    unsigned long long tmp=0,max=0,min=0;
    int pntr=0;
    while (pntr<buffer_size-window) {
      for (int cnt=1; cnt<=window; cnt++) {
        tmp+=buff[n][pntr].i*buff[n][pntr].i+buff[n][pntr].q*buff[n][pntr].q;
        pntr++;
      }
      tmp=round(tmp/window);
      if (max<tmp) max=tmp;
      if (min>tmp) min=tmp;
      tmp=0;
    };
    int sbm=round(100*log10(max/min));
    return round(sbm)/10;
  };
  
  int mine_maxskr(short n, int window){ //n=0..3 window 1-1000 maximum <= buffer_size 
    unsigned long long tmp=0,max=0;
    int pntr=0;
    while (pntr<buffer_size-window)  {
      for (int cnt=1; cnt<=window; cnt++) {
        tmp+=buff[n][pntr].i*buff[n][pntr].i+buff[n][pntr].q*buff[n][pntr].q;
        pntr++;
      }
      tmp=(int)round(tmp/window);
      if (max<tmp) max=tmp;
      tmp=0;
    };
    return (int)round(sqrt(max));
  };
////////
  int outlevel(){
    printf("Xa:%5d %5d %.1fdB  Xb:%5d %5d %.1fdB  Ya:%5d %5d %.1fdB  Yb:%5d %5d %.1fdB\n",
      maxinbuf1,midinbuf1,snrinbuf(XA,16), 
      maxinbuf2,midinbuf2,snrinbuf(XB,16),
      maxinbuf3,midinbuf3,snrinbuf(YA,16),
      maxinbuf4,midinbuf4,snrinbuf(YB,16)
    );  
  }
  int outmetadata(){
    printf("X1:%8lld X2:%8lld Y1:%8lld Y2:%8lld dX1:%7lld dY1:%6lld dX1Y1:%d ",metadata[0].timestamp,metadata[1].timestamp, metadata[2].timestamp, metadata[3].timestamp, (metadata[0].timestamp-prevtsx),  (metadata[2].timestamp-prevtsy),(metadata[0].timestamp-metadata[2].timestamp));
    prevtsx=metadata[0].timestamp;
    prevtsy=metadata[2].timestamp;
  };
///////// gains
  int setgain(int gain){
    int error_count;
    error_count+=LMS_SetGaindB( devicex, LMS_CH_RX, 0, gain );
    error_count+=LMS_SetGaindB( devicex, LMS_CH_RX, 1, gain );
    error_count+=LMS_SetGaindB( devicey, LMS_CH_RX, 0, gain );
    error_count+=LMS_SetGaindB( devicey, LMS_CH_RX, 1, gain );
    if (error_count!=0) return -1;
    return 0;
  }
  short curzone=4;
  short zone[6]={0,0,24,48,72,72};
  int setzone(short z){ // 0-5 
    if (z>5 || z<0) z=5;
    curzone=z;
    setgain(zone[z]);
    if (z==0) closetarget();
    else{ 
      if (z==5) fartarget();
      else normaltarget();
    }
    return z;
  }
  int zoneup(){
    if (curzone==5) return -1;
    curzone++;
    setzone(curzone);
    return curzone;
  };
  int zonedown(){
    if (curzone==0) return -1;
    curzone--;
    setzone(curzone);
    return curzone;
  };

  int setautolevel(int ch,int window){
    int uplimit=24000;
    int minlimit=300;
    int tmp=0;
    while(tmp>uplimit || tmp<minlimit) {    
      getbuf(ch);
      tmp=mine_maxskr(ch,window);
      printf("tmp=%d\n",tmp);
      if (tmp>uplimit)  {if (curzone>0) zonedown(); else return -1;}
      if (tmp<minlimit) {if (curzone<5) zoneup(); else return 1;}
    }
    return 0;
  };
//  int read
/////////////////////////
  printf("maxnumber=%d\nbufersize=%d\n",maxnumber,buffer_size);
  sstart = clock(); //debug time
  int mn = maxnumber;  
  while( maxnumber--!=0 ) {   ///////////////////    start main loop
    start = clock(); //debug time
    getbuf4();
//   printf("errorlevel=%d curzone=%d\n",setautolevel(0,25),curzone);
//  outlevel();
    outmetadata();

    end = clock(); //debug time
    cpu_time_used = 1000*((double) (end - start))/ CLOCKS_PER_SEC;
    printf("  -- iteration time=%.3fmilisec  speed=%.1fMbit/sec\n",cpu_time_used,64*buffer_size/1000/cpu_time_used);
  } //////////////////////////////////////////////// end mainloop	
  send = clock(); //debug time
  cpu_time_used = 1000*((double)(send - sstart)) / CLOCKS_PER_SEC;
  printf("  --  All bufers time=%.3fmilisecsec; all iteration time=%.3fmilisec; middle speed=%.1fMbps.\n", 1000*buffer_size*mn/sample_rate,cpu_time_used,mn*64*buffer_size/1000/cpu_time_used);
  for (int i=0; i<4;i++)LMS_StopStream(&rx_stream[i]);
//  LMS_StopStream(&rx_stream[1]);
//  LMS_StopStream(&rx_stream[2]);
//  LMS_StopStream(&rx_stream[3]);
  LMS_DestroyStream(devicex, &rx_stream[XA]);
  LMS_DestroyStream(devicex, &rx_stream[XB]);
  LMS_DestroyStream(devicey, &rx_stream[YA]);
  LMS_DestroyStream(devicey, &rx_stream[YB]);
  for (int i=0; i<4;i++) free( buff[i] );
  LMS_Close(devicex);
  LMS_Close(devicey);
  return 0;
}

