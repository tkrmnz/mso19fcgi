/* -*- Mode: C; indent-tabs-mode: t; c-basic-offset: 4; tab-width: 4 -*-  */
/*
 * main.c
 * Copyright (C) 2015 John Yeh <jyeh@linkinstruments.com>
 * 
 * mso19_tst4 is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * mso19_tst4 is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/* -*- Mode: C; indent-tabs-mode: t; c-basic-offset: 4; tab-width: 4 -*-  */
/*
 * main.c
 * Copyright (C) 2015 hwyeh <tnkrmnz@gmail.com>
 * 
 * mso19_tst1 is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * mso19_tst1 is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdio.h>
#include <stdlib.h>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <aio.h>
#include <errno.h>
#include <dirent.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include "fcgi_config.h"
#include "fcgi_stdio.h"
//#include "fcgiapp.h"
#include <time.h>

#define BAUDRATE B460800
//#define BAUDRATE B921600
#define TRUE 1
#define FALSE 0

#define FIELD_LEN 250 /* how long each name or value can be */
#define NV_PAIRS 200  /* how many name=value pairs we can process */

typedef struct name_value_st {
    char name[FIELD_LEN + 1];
    char value[FIELD_LEN + 1];
} name_value;

name_value name_val_pairs[NV_PAIRS];

static void GetSerPort19();
static void GetUSBVid();
static void Parse19Sn(unsigned char *Buf);
static void portconfig(int *device, int len);
static unsigned char BufTransLower(unsigned char Addr, unsigned char Val);
static unsigned char BufTransUpper(unsigned char Addr, unsigned char Val);
static unsigned int SerBufInit(unsigned int StartVal);
static void ResetADC();
static void ResetFSM();
static unsigned char CheckTriggerStatus();
static long RotateSid(long sidT);
static void ArmMSO();
static void ReadMso19Param();
static void InitSettings();
static void WriteMsoSettings();
static int ReadMsoSettings19();
static char CalcRateMSBLSB(int nsRate);
static void ClkRate_Out(unsigned char MSB, unsigned char LSB);
static double GetOffsetVBit(unsigned char Chan);
static double GetOffsetCenterVal(unsigned char Chan);
static double GetVbit(unsigned char Chan);
static unsigned short CalcOffsetRawValueFromVoltage(double volts,unsigned char Chan);
static unsigned short CalcRawValueFromVoltage(double val, unsigned char Chan);
static double CalcVoltageFromRawValue(unsigned short pt, double vbit, int ProbeAttn);
static void VoltageConvert();
static void ConfigureHardwareMSO19();
static void send_error(char *error_text);
static char x2c(char *what);
static void unescape_url(char *url);
static void load_nv_pair(char *tmp_buffer, int nv_entry);
static int Get_input(void);
static void ReadQueryString();
static void DAC_Out(unsigned short DacVal);
static void ConfigureTriggerHardware19();
static void ConfigureTriggerPosition();
static void ReadBuffer19();

char * line=NULL;
unsigned char HexBuf[4096];
unsigned char DSOCtlBase;
unsigned int BufTransCnt;
int fd_w, code;
struct termios oldtio;
struct termios newtio;
unsigned int serialNumber, modelNumber, msoType;
double vbit200[2],OffsetVBit200[2],vbit[2],OffsetVBit[2];
unsigned short OffsetCenterVal200[2],OffsetCenterVal[2];
unsigned char MBuf[128],FBuf[128];
unsigned char ACDCM;
unsigned char ClkRateMSB, ClkRateLSB;
int sampleInterval;
unsigned char SlowMode;
unsigned char LogFam;
int ProbeAttn[2];
float VGND = 511.0;
float ADCMAX = 1023.0;
unsigned short OffsetDacVal[2];
int vDiv[2];
char TrigLAWord[9];
//double OffsetDbl[2];
int OffsetDbl[2];
unsigned char ACDCMode[2];
int TrigLevel[3];
unsigned char TrigSlope[3];
const unsigned char SLOPE_RISING = 0;
const unsigned char SLOPE_FALLING = 1;
int TrigPosition;
char TrigWdtmp[37];
char TrigSPIWd[37];
char TrigI2CWd[37];
unsigned char TrigSPIMode = 0;
unsigned int TrigWidth[3];
unsigned char TrigModeDSO[3];

unsigned short AnalogDataA[1024];
unsigned short AnalogDataB[1024];
unsigned char LogicData[1024];
float AnalogVoltageDataA[1024];
float AnalogVoltageDataB[1024];
unsigned char SetChanged=0;

unsigned char SerTrigWdX[8];
//unsigned char TrigChSelX;
unsigned char TrigOutSrc;

long sid=0,CurrSid=0,PrevSid=0;
unsigned char MsoBusy=0;
long sidA=0,sidB=0,sidC=0,sidD=0,sidE=0;

typedef enum {
    TRIG_CHAN_DSO,
	TRIG_CHAN_DSO1,
	TRIG_CHAN_LA,
	TRIG_CHAN_SER_I2C,
	TRIG_CHAN_SER_SPI,
	nMaxTrigChanDefines
} TrigChanDefines ;

TrigChanDefines TrigChan;

typedef enum  {
	TRIG_DSO_EDGE,
	TRIG_DSO_GE,
	TRIG_DSO_LT,
	nMaxTrigModedefines
} TrigModeDefines;

typedef enum {
	TrigOut, 
	FuncGenOut, 
	OutPGBit7, 
	WhiteNoise, 
	nMaxTrigOutModes 
} TrigOutModes;


//#define _POSIX_SOURCE 1 // POSIX compliant source 
#define FALSE 0
#define TRUE 1

volatile int STOP=FALSE; 


//-----------------------------------------------------
void GetSerPort19()
{
	DIR *dp;
	char CurDir[FILENAME_MAX];
	struct dirent *entry;
	char *dir = "/dev";
	char *sdir = NULL;
	char * sdirnm = NULL;
	int newlen;

	getcwd(CurDir, sizeof(CurDir));//get current working directory

	if((dp = opendir(dir)))
	{
		chdir(dir);//cd to dir
		while((entry = readdir(dp)))
		{//search for MSO-19
			if((strcmp(".",entry->d_name) == 0)||
				(strcmp("..",entry->d_name) == 0))
				continue;
			sdir = entry->d_name;
			if(strlen(sdir)>6)
			{
				if((sdir[0] == 'M') && (sdir[1] == 'S') && (sdir[2] == 'O')
				   && (sdir[3] == '-') && (sdir[4] == '1') && (sdir[5] == '9'))
				{
				newlen = 5 + strlen(sdir) + 2;
				sdirnm = (char *) calloc(newlen,sizeof(char));
				strcat(sdirnm,"/dev/");
				strcat(sdirnm,sdir);
				strcpy(line,sdirnm);
				free(sdirnm);
			   }//if MSO-19
			}//if Len>6
		}//readdir

	chdir("..");
	closedir(dp);
	chdir(CurDir);//Restore working directory

	}//opendir ok
	else
	{
		fprintf(stderr,"cannot open directory: %s\n", dir);
//		printf("cannot open directory: %s\n", dir);
	}//opendir ng

}
//-------------------------------------------------
void GetUSBVid()
{
	FILE *pp;
	FILE *fp;
	char sBuf[512];
	char UsbBus[4];
	char UsbDevice[4];
	char Mso19Sn[43];

	pp = popen("lsusb -d 3195:f190","r");
	fp = fopen("./mso19sn.txt","wb");
	if (pp != NULL) {
	while (1) {
	  char *line;
	  char buf[1000];

	  line = fgets(buf, sizeof buf, pp);
	  if (line == NULL) break;
	  else  {
//		 printf("%s", line);
	//	fprintf(fp,"%s", &line[0]);
		strncpy(UsbBus,&line[4],3);
		UsbBus[3]='\0';
		strncpy(UsbDevice,&line[15],3);
		UsbDevice[3]='\0';
//		printf("%s\n",UsbBus);
//		printf("%s\n",UsbDevice);
	  }
	}
	//	fprintf(fp,"%c", 0x00);
//	fclose(fp);
	pclose(pp);
	}//Get USB Bus/Device number
	
	sprintf(sBuf,"lsusb -D /dev/bus/usb/%s/%s\n\n",UsbBus,UsbDevice);
	pp = popen(sBuf,"r");
	if (pp != NULL) {
	while (1) {
	  char *line;
	  char buf[1000];

	  line = fgets(buf, sizeof buf, pp);
	  if (line == NULL) break;
	  else  if((line[0] == ' ')&&(line[1] == ' ')&&(line[2] == 'i')
			&&(line[3] == 'S')&&(line[4] == 'e')&&(line[5] == 'r')
	        &&(line[6] == 'i')&&(line[7] == 'a')&&(line[8] == 'l'))   
	           {
  //  			printf("%s, %d\n", line, strlen(line));
				if(strlen(line) == 48)	strncpy(Mso19Sn,&line[28],30);
				else 	strncpy(Mso19Sn,&line[28],41);
//				printf("%s\n", Mso19Sn);
				fprintf(fp,"%s", Mso19Sn);
				//	fprintf(fp,"%c", 0x00);
				fclose(fp);
				   
			}
		}
	}
	//system(sBuf); 

//	return 0;
}
//-----------------------------------------------------
//Parse SPI EEProm data into MSO calibration parameters
void Parse19Sn(unsigned char *Buf)
{
//    int BufPos;
//	unsigned short tmp;
	int BufVal;


	BufVal = ((Buf[0]&0x0f)*10000)+((Buf[1]&0x0f)*1000)
			+((Buf[2]&0x0f)*100)+((Buf[3]&0x0f)*10)
			+(Buf[4]&0x0f);

	vbit[0] = (double)BufVal/10000.0;
	vbit[1] = vbit[0];
	
	BufVal = ((Buf[5]&0x0f)*100)+((Buf[6]&0x0f)*10)
			+(Buf[7]&0x0f);

	OffsetVBit[0] = (double)3000.0/BufVal;
	OffsetVBit[1] = OffsetVBit[0];

	BufVal = ((Buf[8]&0x0f)*100)+((Buf[9]&0x0f)*10)
			+(Buf[10]&0x0f);
	OffsetCenterVal[0] = BufVal;
	OffsetCenterVal[1] =OffsetCenterVal[0];

	BufVal = ((Buf[13]&0x0f)*100000)+((Buf[14]&0x0f)*10000)
			+((Buf[15]&0x0f)*1000)+((Buf[16]&0x0f)*100)
			+((Buf[17]&0x0f)*10)+(Buf[18]&0x0f);

	serialNumber= BufVal;

	BufVal = ((Buf[19]&0x0f)*10000)+((Buf[20]&0x0f)*1000)
			+((Buf[21]&0x0f)*100)+((Buf[22]&0x0f)*10)
			+(Buf[23]&0x0f);

	if(BufVal == 0) vbit200[0] = vbit[0];
	else vbit200[0] = (double)BufVal/10000.0;
	vbit200[1] =vbit200[0];

	
	BufVal = ((Buf[24]&0x0f)*100)+((Buf[25]&0x0f)*10)
			+(Buf[26]&0x0f);

	if(BufVal == 0) OffsetVBit200[0] = OffsetVBit[0];
	else OffsetVBit200[0] = (double)3000.0/BufVal;
	OffsetVBit200[1] =OffsetVBit200[0];
	
	BufVal = ((Buf[27]&0x0f)*100)+((Buf[28]&0x0f)*10)
			+(Buf[29]&0x0f);

	if(BufVal == 0) OffsetCenterVal200[0] = OffsetCenterVal[0];
	else OffsetCenterVal200[0] = BufVal;
	OffsetCenterVal200[1] =OffsetCenterVal200[0];
	
//	printf("%f %f %d %d %f %f %d ",
//	       vbit[0],OffsetVBit[0],OffsetCenterVal[0],serialNumber,
//	       vbit200[0],OffsetVBit200[0],OffsetCenterVal200[0]);
	
	msoType = Buf[11];
	modelNumber = Buf[12];

}
//-----------------------------------------------------


//Configure serial port
void portconfig(int *device, int len)
{
	struct termios newtio;
	int cod = 0;

	newtio.c_cflag = 0;
	newtio.c_iflag = 0;
	newtio.c_oflag = 0;
	newtio.c_lflag = 0;

	cod=tcsetattr(*device,TCSANOW,&newtio);
	if(cod<0)
	{
		 perror("Setting attributes failed!");
		 exit(-1);
	}
	cfsetispeed(&newtio, BAUDRATE);
	cfsetospeed(&newtio, BAUDRATE);

	newtio.c_cflag |= (CS8 | CLOCAL | CREAD );//exp
	newtio.c_lflag = 0;
	newtio.c_iflag |= IGNPAR;
	newtio.c_oflag = 0;
	newtio.c_cc[VTIME]    = 0;   // inter-character timer unused 
	newtio.c_cc[VMIN]     = len;   // blocking read until 5 chars received 


	tcflush(*device,TCIOFLUSH);//flushes untransmitted output
	cod=tcsetattr(*device,TCSANOW,&newtio);	//changes occurs immediately.
	
	if(cod<0)
	{
		 perror("Setting attributes failed!");
		 exit(-1);
	}

}
//-----------------------------------------------------

unsigned char BufTransLower(unsigned char Addr, unsigned char Val)
{
	unsigned char i;
	i = Val & 0x3f;
	if ((i & 0x20) != 0x20)
	    i |= 0x40;
	return i;
}

//-----------------------------------------------------

unsigned char BufTransUpper(unsigned char Addr, unsigned char Val)
{
	unsigned char i, j, k;
	i = Addr & 0x0f;
	j = Val & 0xc0;
	j = j >> 2;
	k = i | j;
	if ((k & 0x20) != 0x20)
	    k |= 0x40;
	return k;
}
//-----------------------------------------------------
unsigned int SerBufInit(unsigned int StartVal)
{
	//Preload @LDS~
	HexBuf[StartVal] = 0x40;
    HexBuf[StartVal+1] = 0x4C;
    HexBuf[StartVal+2] = 0x44;
    HexBuf[StartVal+3] = 0x53;
    HexBuf[StartVal+4] = 0x7E;

	return StartVal + 5;
}
//------------------------------------------------
//Issues Reset to ADC and puts the MSO in ready to be arm mode
void ResetADC()
{
	fd_w = open(line,O_RDWR | O_NOCTTY |  O_NDELAY);	//opening serial port device for write action
	if(fd_w < 0){
         printf("Serial error Rst ADC\n");
	}
	else
	{
		portconfig(&fd_w,1);
		DSOCtlBase = 0x10;
		BufTransCnt = 0;
		BufTransCnt = SerBufInit(BufTransCnt);
		HexBuf[BufTransCnt++] = BufTransUpper(0x0e, (DSOCtlBase | 0x40));
		HexBuf[BufTransCnt++] = BufTransLower(0x0e, (DSOCtlBase | 0x40));
		HexBuf[BufTransCnt++] = BufTransUpper(0x0e, DSOCtlBase);
		HexBuf[BufTransCnt++] = BufTransLower(0x0e, DSOCtlBase);
		HexBuf[BufTransCnt++] = 0x7E;
		write(fd_w,HexBuf,BufTransCnt);  
		close(fd_w);
	}
}
//-----------------------------------------------------
void ResetFSM()
{
	fd_w = open(line,O_RDWR | O_NOCTTY |  O_NDELAY);	//opening serial port device for write action
	if(fd_w < 0)
	{
         printf("Serial error RstFSM\n");
	}
	else
	{
		portconfig(&fd_w,1);
		DSOCtlBase = 0x10;
		BufTransCnt = 0;
		BufTransCnt = SerBufInit(BufTransCnt);
		HexBuf[BufTransCnt++] = BufTransUpper(0x0e, (DSOCtlBase | 0x01));
		HexBuf[BufTransCnt++] = BufTransLower(0x0e, (DSOCtlBase | 0x01));
		HexBuf[BufTransCnt++] = 0x7E;
		write(fd_w,HexBuf,BufTransCnt);  
		close(fd_w);
	}
}

//-----------------------------------------------------
//returns trigstat with open/close fd_w
unsigned char CheckTriggerStatus()
{
	int lcnt = 0;
//	int res;
	unsigned char buf[10];
	int bytes;

	fd_w = open(line,O_RDWR | O_NOCTTY |  O_NDELAY);	//opening serial port device for write action

	if(fd_w < 0)
	{
		printf("Serial error TS\n");
		return fd_w;
	}
	else
	{
		portconfig(&fd_w,1);

		BufTransCnt = 0;
		BufTransCnt = SerBufInit(BufTransCnt);
		HexBuf[BufTransCnt++] = BufTransUpper(0x02, 0x00);
		HexBuf[BufTransCnt++] = BufTransLower(0x02, 0x00);
		HexBuf[BufTransCnt++] = 0x7E;
		write(fd_w,HexBuf,BufTransCnt);  
		usleep(5);



		STOP = FALSE;
		while (STOP==FALSE){       // loop for input 
			ioctl(fd_w, FIONREAD, &bytes);
			if(bytes >= 1) {
//				res = read(fd_w,buf,bytes);
				read(fd_w,buf,bytes);
				STOP = TRUE;
			}
			else
			{
				lcnt++;
			}
				usleep(5);
		}

 		tcflush(fd_w, TCIOFLUSH);

		close(fd_w);
		return buf[0];
	}
}
//-----------------------------------------------------
static long RotateSid(long sidT)
{
	long Ret;
	Ret = sidE;
	sidE=sidD;
	sidD=sidC;
	sidC=sidB;
	sidB=sidA;
	sidA=sidT;
	return Ret;
}
//-----------------------------------------------------
void ArmMSO()
{
	fd_w = open(line,O_RDWR | O_NOCTTY |  O_NDELAY);	//opening serial port device for write action
	if(fd_w < 0)
	{
         printf("Serial error ArmMSO\n");
	}
	else
	{
		portconfig(&fd_w,1);
		DSOCtlBase = 0x10;
		if(ACDCMode[0]) DSOCtlBase |= 0x80;//set DC mode, default is AC
		
		BufTransCnt = 0;
		BufTransCnt = SerBufInit(BufTransCnt);
		HexBuf[BufTransCnt++] = BufTransUpper(0x0e, (DSOCtlBase | 0x01));
		HexBuf[BufTransCnt++] = BufTransLower(0x0e, (DSOCtlBase | 0x01));
		HexBuf[BufTransCnt++] = BufTransUpper(0x0e, (DSOCtlBase | 0x02));
		HexBuf[BufTransCnt++] = BufTransLower(0x0e, (DSOCtlBase | 0x02));
		HexBuf[BufTransCnt++] = BufTransUpper(0x0e, (DSOCtlBase));
		HexBuf[BufTransCnt++] = BufTransLower(0x0e, (DSOCtlBase));
		HexBuf[BufTransCnt++] = 0x7E;

		write(fd_w,HexBuf,BufTransCnt);  
		close(fd_w);
	}
}
//-----------------------------------------------------
void ConfigureThresholdLevel()
{
	//unsigned char MSB, LSB;	
	unsigned short tmp = 0x87ff;
    switch (LogFam)
    {
        case 0: tmp = 0x8600; break;    //1.2V Logic 0x8600
        case 1: tmp = 0x8770; break;    //1.5V Logic 0x8770
        case 2: tmp = 0x88ff; break;    //1.8V Logic 0x88ff
        case 3: tmp = 0x8c70; break;    //2.5V Logic 0x8c70
        case 4: tmp = 0x8eff; break;    //3.0V Logic	0x8eff
        default:
        case 5: tmp = 0x8fff; break;    //3.3V Logic family   0x8fff
    }

//    LSB = tmp & 0x00ff;
//    MSB = (tmp & 0xff00) >> 8;
    DAC_Out(tmp);//Logic Threshold

}
//-----------------------------------------------------

void DAC_Out(unsigned short DacVal)
{

    unsigned char MSB, LSB;

    LSB = (unsigned char)(DacVal & 0x00ff);
    MSB = (unsigned char)((DacVal & 0xff00) >> 8);

	fd_w = open(line,O_RDWR | O_NOCTTY |  O_NDELAY);	//opening serial port device for write action
	if(fd_w < 0){
         printf("Serial error ClkRate\n");
	}
	else
	{
		portconfig(&fd_w,1);
		DSOCtlBase = 0x10;
		BufTransCnt = 0;
		BufTransCnt = SerBufInit(BufTransCnt);
		HexBuf[BufTransCnt++] = BufTransUpper(0x0c, MSB);//DAC MSB
		HexBuf[BufTransCnt++] = BufTransLower(0x0c, MSB);
		HexBuf[BufTransCnt++] = BufTransUpper(0x0d, LSB);//DAC LSB
		HexBuf[BufTransCnt++] = BufTransLower(0x0d, LSB);
		HexBuf[BufTransCnt++] = BufTransUpper(0x0e, (DSOCtlBase|0x20));
		HexBuf[BufTransCnt++] = BufTransLower(0x0e, (DSOCtlBase|0x20));
		HexBuf[BufTransCnt++] = 0x7E;
		write(fd_w,HexBuf,BufTransCnt);  
		close(fd_w);
	}
}
//-----------------------------------------------------
//Read msocfg.bin file 
void ReadMso19Param()
{
	FILE *fp;
	/* open the file */
	fp = fopen("mso19sn.txt", "rb");
	if(fp)
	{
		fread(FBuf,sizeof(FBuf[0]),30,fp);
		fclose(fp);
	}
}
//-----------------------------------------------------
void InitSettings()
{
	vDiv[0] = 500;			vDiv[1] = 500;
	//vDiv
	ProbeAttn[0] = 1;  	ProbeAttn[1] = 1;
	//ProbeAttn
	ACDCMode[0] = 0;		ACDCMode[1] = 0;
	//ACDCMode
	OffsetDbl[0] = 0;	   OffsetDbl[1] = 0;
	//OffsetDbl
	LogFam = 5;
	//LogFam
	sampleInterval = 10;
	//sampleInterval
	TrigLevel[0] = 0;	   	TrigLevel[1] = 0;
	//TrigLevel
//	printf("nv3.1= %s %s\n",name_val_pairs[0].name,name_val_pairs[0].value);

	TrigSlope[0] = SLOPE_RISING;
	TrigSlope[1] = SLOPE_RISING;
	TrigSlope[2] = SLOPE_RISING;
	//TrigSlope	
	TrigLAWord[0] = 0;
	strcat(TrigLAWord,"XXXXXXXX");
//	printf("nv3.2= %s %s\n",name_val_pairs[0].name,name_val_pairs[0].value);
	//TrigLAWord
	TrigPosition = 500;
	//TrigPosition
	TrigSPIWd[0] = 0;
	strcat(TrigSPIWd,"XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX");
	//TrigSPIWd
	TrigI2CWd[0] = 0;
	strcat(TrigI2CWd,"XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX");
//	printf("nv3.3= %s %s\n",name_val_pairs[0].name,name_val_pairs[0].value);

	//TrigI2CWd
	TrigSPIMode = 0;
	//TrigSPIMode
	TrigWidth[0] = 0;
	TrigWidth[1] = 0;
	//TrigWidth
	TrigModeDSO[0] = TRIG_DSO_EDGE;
	TrigModeDSO[1] = TRIG_DSO_EDGE;
	//TrigModeDSO
	TrigChan = TRIG_CHAN_DSO;
	TrigOutSrc = TrigOut;
	//TrigChan
}
//write settings.bin file

//-----------------------------------
void WriteMsoSettings()
{
	FILE *fp;
	/* open the file */
	fp = fopen("msoset.txt", "wb");
	if(fp)
	{
		fprintf(fp,"vDiv0,%d\n",vDiv[0]);
		fprintf(fp,"vDiv1,%d\n",vDiv[1]);
		fprintf(fp,"ProbeAttn0,%d\n",ProbeAttn[0]);
		fprintf(fp,"ProbeAttn1,%d\n",ProbeAttn[1]);
		fprintf(fp,"ACDCMode0,%d\n",ACDCMode[0]);
		fprintf(fp,"ACDCMode1,%d\n",ACDCMode[1]);
		fprintf(fp,"OffsetDbl0,%d\n",OffsetDbl[0]);
		fprintf(fp,"OffsetDbl1,%d\n",OffsetDbl[1]);
		fprintf(fp,"LogFam,%d\n",LogFam);
		fprintf(fp,"sampleInterval,%d\n",sampleInterval);
		fprintf(fp,"TrigLevel0,%d\n",TrigLevel[0]);
		fprintf(fp,"TrigLevel1,%d\n",TrigLevel[1]);
		fprintf(fp,"TrigSlope0,%d\n",TrigSlope[0]);
		fprintf(fp,"TrigSlope1,%d\n",TrigSlope[1]);
		fprintf(fp,"TrigLAWord,%s\n",TrigLAWord);
		fprintf(fp,"TrigLASlope,%d\n",TrigSlope[2]);
		fprintf(fp,"TrigPosition,%d\n",TrigPosition);
		fprintf(fp,"TrigSPIWd,%s\n",TrigSPIWd);
		fprintf(fp,"TrigI2CWd,%s\n",TrigI2CWd);
		fprintf(fp,"TrigSPIMode,%d\n",TrigSPIMode);
		fprintf(fp,"TrigWidth0,%d\n",TrigWidth[0]);
		fprintf(fp,"TrigWidth1,%d\n",TrigWidth[1]);
		fprintf(fp,"TrigModeDSO0,%d\n",TrigModeDSO[0]);
		fprintf(fp,"TrigModeDSO1,%d\n",TrigModeDSO[1]);
		fprintf(fp,"TrigChan,%d\n",TrigChan);
		fprintf(fp,"TrigOutSrc,%d\n",TrigOutSrc);
		fclose(fp);
	}
}
//-----------------------------------

int ReadMsoSettings19()
{
	char *fBuf;
	char vBuf[512],pBuf[512],tBuf[512];
	FILE *fp;
	int ii,jj;
	int i;
//	int n;

	/* open the file */
	fp = fopen("msoset.txt", "rb");
	if(fp){
		i=0;
		fBuf = fgets(tBuf,512,fp);
		while(fBuf){
			//n = strlen(fBuf);

			pBuf[0]=0;
			vBuf[0]=0;
			ii = 0;
			jj = 0;
			while(fBuf[ii]!=','){
				pBuf[ii]=fBuf[ii];
				ii++;
			}
			pBuf[ii]=0;
			ii++;
			while(fBuf[ii]!=0){
				vBuf[jj]=fBuf[ii];
				ii++;
				jj++;
			}
			vBuf[jj]=0;
			if(strcmp(pBuf,"TrigI2CWd")==0) {
				TrigI2CWd[0]=0;
				strncat(TrigI2CWd,vBuf,32);
			}
			if(strcmp(pBuf,"TrigSPIWd")==0) {
				TrigSPIWd[0]=0; 
				strncat(TrigSPIWd,vBuf,32);
			}
			if(strcmp(pBuf,"vDiv0")==0) vDiv[0]=atoi(vBuf);
			if(strcmp(pBuf,"vDiv1")==0) vDiv[1]=atoi(vBuf);
			if(strcmp(pBuf,"ProbeAttn0")==0) ProbeAttn[0]=atoi(vBuf);
			if(strcmp(pBuf,"ProbeAttn1")==0) ProbeAttn[1]=atoi(vBuf);
			if(strcmp(pBuf,"ACDCMode0")==0) ACDCMode[0]=atoi(vBuf);
			if(strcmp(pBuf,"ACDCMode1")==0) ACDCMode[1]=atoi(vBuf);
			if(strcmp(pBuf,"OffsetDbl0")==0) OffsetDbl[0]=atoi(vBuf);
			if(strcmp(pBuf,"OffsetDbl1")==0) OffsetDbl[1]=atoi(vBuf);
			if(strcmp(pBuf,"LogFam")==0) LogFam=atoi(vBuf);
			if(strcmp(pBuf,"sampleInterval")==0) sampleInterval=atoi(vBuf);
			if(strcmp(pBuf,"TrigLevel0")==0) TrigLevel[0]=atoi(vBuf);
			if(strcmp(pBuf,"TrigLevel1")==0) TrigLevel[1]=atoi(vBuf);
			if(strcmp(pBuf,"TrigSlope0")==0) TrigSlope[0]=atoi(vBuf);
			if(strcmp(pBuf,"TrigSlope1")==0) TrigSlope[1]=atoi(vBuf);
			if(strcmp(pBuf,"TrigLAWord")==0) {TrigLAWord[0]=0; strncat(TrigLAWord,vBuf,8);}
			if(strcmp(pBuf,"TrigLASlope")==0) TrigSlope[2]=atoi(vBuf);
			if(strcmp(pBuf,"TrigPosition")==0) TrigPosition=atoi(vBuf);
			if(strcmp(pBuf,"TrigSPIMode")==0) TrigSPIMode=atoi(vBuf);
			if(strcmp(pBuf,"TrigWidth0")==0) TrigWidth[0]=atoi(vBuf);
			if(strcmp(pBuf,"TrigWidth1")==0) TrigWidth[1]=atoi(vBuf);
			if(strcmp(pBuf,"TrigModeDSO0")==0) TrigModeDSO[0]=atoi(vBuf);
			if(strcmp(pBuf,"TrigModeDSO1")==0) TrigModeDSO[1]=atoi(vBuf);
			if(strcmp(pBuf,"TrigChan")==0) TrigChan=atoi(vBuf);
			if(strcmp(pBuf,"TrigOutSrc")==0) TrigOutSrc=atoi(vBuf);
			
			fBuf[0]=0;
			fBuf = fgets(tBuf,512,fp);
			i++;
		}


		fclose(fp);
		return 1;
	}//fp
	else return 0;
}

//-----------------------------------------------------
char CalcRateMSBLSB(int nsRate)
{
    char ret = 0;
    SlowMode = 0x00;
    switch (nsRate)
    {
        case 1: //RIS mode 1GSa
        case 5: ClkRateMSB = 0x02; ClkRateLSB = 0x05; break;//Tstg = "200Msa/S";
        case 10: ClkRateMSB = 0x01; ClkRateLSB = 0x03; break;//Tstg = "100Msa/S";
        case 20: ClkRateMSB = 0x03; ClkRateLSB = 0x02; break;//Tstg = "50Msa/S";
        case 50: ClkRateMSB = 0x03; ClkRateLSB = 0x08; break;//Tstg = "20Msa/S" 3;
        case 100: ClkRateMSB = 0x03; ClkRateLSB = 0x12; break;//Tstg = "10Msa/S" 8;
        case 200: ClkRateMSB = 0x03; ClkRateLSB = 0x26; break;//Tstg = "5Msa/S" 18;
        case 500: ClkRateMSB = 0x03; ClkRateLSB = 0x62; break;//Tstg = "2Msa/S" 48;
        case 1000: ClkRateMSB = 0x03; ClkRateLSB = 0xc6; break;//Tstg = "1Msa/S" 98; 
        case 2000: ClkRateMSB = 0x07; ClkRateLSB = 0x8e; break;//Tstg = "500Ksa/S" 198;
        case 5000: ClkRateMSB = 0x0f; ClkRateLSB = 0xe6; break;//Tstg = "200Ksa/S" 498; 
        case 10000: ClkRateMSB = 0x1f; ClkRateLSB = 0xce; break;//Tstg = "100Ksa/S" 998;
        case 20000: ClkRateMSB = 0x3f; ClkRateLSB = 0x9e; break;//Tstg = "50Ksa/S" 1998; 
        case 50000: ClkRateMSB = 0x9f; ClkRateLSB = 0x0e; break;//Tstg = "20Ksa/S" 4998;
        //0322612 slow rate
		case 100000: ClkRateMSB = 0x03; ClkRateLSB = 0xc6; SlowMode = 0x20; break;//Tstg = "10Ksa/S" 9998;
        case 200000: ClkRateMSB = 0x07; ClkRateLSB = 0x8e; SlowMode = 0x20; break;//Tstg = "5Ksa/S" 199;  
        case 500000: ClkRateMSB = 0x0f; ClkRateLSB = 0xe6; SlowMode = 0x20; break;//Tstg = "2Ksa/S" 499;
        case 1000000: ClkRateMSB = 0x1f; ClkRateLSB = 0xce; SlowMode = 0x20; break;//Tstg = "1Ksa/S" 999;
        case 2000000: ClkRateMSB = 0x3f; ClkRateLSB = 0x9e; SlowMode = 0x20; break;//Tstg = "500sa/S" 1999; 
        case 5000000: ClkRateMSB = 0x9f; ClkRateLSB = 0x0e; SlowMode = 0x20; break;//Tstg = "20sa/S" 4999;
 //       case 10000000: ClkRateMSB = 0x9f; ClkRateLSB = 0x0f; SlowMode = 0x20; break;//Tstg = "10sa/S" 9999;
        default:
            ret = -1;
            break;
    }
    return ret;
}
//-----------------------------------------------------
void ClkRate_Out(unsigned char MSB, unsigned char LSB)
{
	fd_w = open(line,O_RDWR | O_NOCTTY |  O_NDELAY);	//opening serial port device for write action
	if(fd_w < 0){
         printf("Serial error ClkRate\n");
	}
	else
	{
		portconfig(&fd_w,1);
		DSOCtlBase = 0x10;
		BufTransCnt = 0;
		BufTransCnt = SerBufInit(BufTransCnt);
		HexBuf[BufTransCnt++] = BufTransUpper(0x09, MSB);
		HexBuf[BufTransCnt++] = BufTransLower(0x09, MSB);
		HexBuf[BufTransCnt++] = BufTransUpper(0x0a, LSB);
		HexBuf[BufTransCnt++] = BufTransLower(0x0a, LSB);
		HexBuf[BufTransCnt++] = 0x7E;
		write(fd_w,HexBuf,BufTransCnt);  
		close(fd_w);
	}
}
//-----------------------------------------------------
double GetOffsetVBit(unsigned char Chan)
{
	int tmp;
	if(sampleInterval >= 10)	tmp = OffsetVBit[Chan];
	else tmp = OffsetVBit200[Chan];
	return tmp;
	
}
//-----------------------------------------------------

double GetOffsetCenterVal(unsigned char Chan)
{
	int tmp;
	if(sampleInterval >= 10) tmp = OffsetCenterVal[Chan];
	else tmp = OffsetCenterVal200[Chan];
	return tmp;	
}
//-----------------------------------------------------
double GetVbit(unsigned char Chan)
{
	int tmp;
	if(sampleInterval >= 10) tmp = vbit[Chan];
	else tmp = vbit200[Chan];
	return tmp;	
}
//-----------------------------------------------------
unsigned short CalcOffsetRawValueFromVoltage(double volts,unsigned char Chan)
{ //don't need to adjust for probe atten
    unsigned short DacVal;
    int DacTmp;

	DacTmp = (int)(GetOffsetCenterVal(Chan) - ((volts / ProbeAttn[Chan]) / GetOffsetVBit(Chan)));
	
	if (DacTmp < 0) DacVal = 0x0000;
    else if (DacTmp > 0x0fff) DacVal = 0x0fff;
    else DacVal = (DacTmp);

    return (unsigned short) DacVal;
}
 
//-----------------------------------------------------
			
unsigned short CalcRawValueFromVoltage(double val, unsigned char Chan)
{
    return (unsigned short)(512 - (unsigned short)((val/ProbeAttn[Chan]) / GetVbit(Chan)));
}

//-----------------------------------------------------
double CalcVoltageFromRawValue(unsigned short pt, double vbit, int ProbeAttn)
{
	double ret;
    double Vtmp, Vtmp2; 
    Vtmp = pt;
    Vtmp2 = (1023.0 - Vtmp) - VGND;
    ret = Vtmp2 * vbit * ProbeAttn;
    return ret;
}

//-----------------------------------------------------
void VoltageConvert()
{
    int ii; //, PageSize = 5;
    double VbitA,VbitB;
    int PAtnA,PAtnB;	

    VbitA=GetVbit(0);
    VbitB=GetVbit(1);
    PAtnA=ProbeAttn[0];
    PAtnB=ProbeAttn[1];


    for (ii = 0; ii < 1024; ii++)
        {
            AnalogVoltageDataA[ii] =
                CalcVoltageFromRawValue(AnalogDataA[ii],
                VbitA,
                PAtnA);

            AnalogVoltageDataB[ii] =
                CalcVoltageFromRawValue(AnalogDataB[ii],
                VbitB,
                PAtnB);
        }
}
//-----------------------------------------------------
void ConfigureTriggerPosition()
{
    unsigned short LocalTrigPos;
	unsigned char MSB, LSB;

	fd_w = open(line,O_RDWR | O_NOCTTY |  O_NDELAY);	//opening serial port device for write action
	if(fd_w < 0)
	{
         printf("Serial error TrigPos\n");
	}
	else
	{
        if (TrigPosition >= 0)
        {
            LocalTrigPos = TrigPosition;
            LocalTrigPos &= 0X7fff; //turn off holdoff counter
        }
        else
        {
            LocalTrigPos = TrigPosition * -1;
            LocalTrigPos |= 0X8000; //turn on holdoff counter
        }
        if (sampleInterval == 5) LocalTrigPos += 11;//6
        else if (sampleInterval == 10) LocalTrigPos += 9;//6
        else if (sampleInterval == 20) LocalTrigPos += 9;
        else LocalTrigPos += 3;

        LSB = LocalTrigPos & 0x00ff;
        MSB = (LocalTrigPos & 0xff00) >> 8;

        portconfig(&fd_w,1);
		DSOCtlBase = 0x10;
		BufTransCnt = 0;
		BufTransCnt = SerBufInit(BufTransCnt);
		HexBuf[BufTransCnt++] = BufTransUpper(0x07, LSB);
		HexBuf[BufTransCnt++] = BufTransLower(0x07, LSB);
		HexBuf[BufTransCnt++] = BufTransUpper(0x08, MSB);
		HexBuf[BufTransCnt++] = BufTransLower(0x08, MSB);
		HexBuf[BufTransCnt++] = 0x7E;
		write(fd_w,HexBuf,BufTransCnt);  
		close(fd_w);
	}
}

//-----------------------------------------------------
void ConfigureTriggerHardware19()
{
    unsigned char MSB, LSB, SPIDataSource;
    unsigned short TrigVal[2];
//    unsigned char TrigChSel=0;
    int ii;
    unsigned char logTrigWd = 0, logTrigIgn = 0;
    char tmpTrigWord[8]; // = "XXXXXXXX";

	strcpy(tmpTrigWord,TrigLAWord);
    for (ii = 0; ii < 8; ii++)
    {
        if (tmpTrigWord[ii] == '1')
            logTrigWd |= (0x01 << ii);
        if (tmpTrigWord[ii] == 'x' || tmpTrigWord[ii] == 'X')
            logTrigIgn |= (0x01 << ii);
    }
	

	TrigVal[0] = CalcRawValueFromVoltage(
        TrigLevel[TRIG_CHAN_DSO] + OffsetDbl[TRIG_CHAN_DSO], 0);


	if (TrigChan == TRIG_CHAN_DSO){
		TrigVal[0] &= 0x03ff;  //Clear TrigVal
		if (TrigSlope[0] == SLOPE_FALLING) TrigVal[0] |= 0x0400;
		else TrigVal[0] &= 0xfbff;
		if (TrigModeDSO[0] == TRIG_DSO_GE)
		{
		    TrigVal[0] |= 0x4000;  //TRIG_DSO_GE
		}
		if (TrigModeDSO[0] == TRIG_DSO_LT)
		{
		    TrigVal[0] |= 0x2000;  //TRIG_DSO_LT
		}
	}
    else if (TrigChan == TRIG_CHAN_LA){
        TrigVal[0] &= 0x03ff;  //clear trigger val
        TrigVal[0] |= 0xe000;  //set la trigger
        if (TrigSlope[2] == SLOPE_RISING)
            TrigVal[0] |= 0x0400;
        else
            TrigVal[0] &= 0xfbff;
	}
    else if (TrigChan == TRIG_CHAN_SER_I2C){
        TrigVal[0] &= 0x03ff;  //clear trigger val
        TrigVal[0] |= 0xa000;  //set I2C trigger
	}
    else if (TrigChan == TRIG_CHAN_SER_SPI){
        SPIDataSource = 0x00;//Trig on SI=0  s0=0x04
        TrigVal[0] &= 0x03ff;  //clear trigger val
        TrigVal[0] |= 0x8000;  //set SPI trigger
        if (TrigSPIMode < 0 || TrigSPIMode > 3)
            TrigSPIMode = 0;
        TrigSPIMode |= SPIDataSource;

	}

	TrigVal[0] &= 0xe7ff; //Clear TrigOutSrc set it to Gnd
	if (TrigOutSrc == FuncGenOut)
		TrigVal[0] |= 0x0800;
	else if (TrigOutSrc == OutPGBit7)
		TrigVal[0] |= 0x1000;
	else if (TrigOutSrc == WhiteNoise)
		TrigVal[0] |= 0x1800;

	


	fd_w = open(line,O_RDWR | O_NOCTTY |  O_NDELAY);	//opening serial port device for write action
	if(fd_w < 0)
	{
    	printf("Serial error TrgHdwr\n");
	}
	else
	{
		portconfig(&fd_w,1);
		DSOCtlBase = 0x10;
		BufTransCnt = 0;
		BufTransCnt = SerBufInit(BufTransCnt);

		HexBuf[BufTransCnt++] = BufTransUpper(0x05, logTrigWd);
		HexBuf[BufTransCnt++] = BufTransLower(0x05, logTrigWd);
		HexBuf[BufTransCnt++] = BufTransUpper(0x06, logTrigIgn);
		HexBuf[BufTransCnt++] = BufTransLower(0x06, logTrigIgn);

		LSB = (unsigned char)(TrigVal[0] & 0x00ff);
		MSB = (unsigned char)((TrigVal[0] & 0xff00) >> 8);
		HexBuf[BufTransCnt++] = BufTransUpper(0x03, LSB);
		HexBuf[BufTransCnt++] = BufTransLower(0x03, LSB);
		HexBuf[BufTransCnt++] = BufTransUpper(0x04, MSB);
		HexBuf[BufTransCnt++] = BufTransLower(0x04, MSB);

		//Width first, TrigVal second

		//---------  Ch0 trig settings
		ii = (unsigned char)(TrigWidth[0] / sampleInterval);
		if (ii > 3)
		    ii -= 3;
		else
		    ii = 3;

		HexBuf[BufTransCnt++] = BufTransUpper(0x0b, ii);
		HexBuf[BufTransCnt++] = BufTransLower(0x0b, ii);

		HexBuf[BufTransCnt++] = BufTransUpper(0x0f, (0x02 | SlowMode));//Alt Page 2, SerTrig
		HexBuf[BufTransCnt++] = BufTransLower(0x0f, (0x02 | SlowMode));

		int Dex = 0;
		unsigned char SerTrigWd[4];
		unsigned char SerIgnWd[4];
		char chtmp;
		
    if (TrigChan == TRIG_CHAN_SER_SPI){
		for (ii = 0; ii < 4; ii++){
		    SerTrigWd[ii] = 0;
		    SerIgnWd[ii] = 0;
		    for (Dex = 0; Dex < 8; Dex++){
				chtmp = TrigSPIWd[((3-ii)*8)+Dex];
				if (chtmp == '1')
		            SerTrigWd[ii] |= (0x01 << Dex);
				else if((chtmp =='X')||(chtmp == 'x'))
		            SerIgnWd[ii] |= (0x01 << Dex);
			}
		}//SPI Parsing
		
		while(SerIgnWd[0] == 0xff){
			SerTrigWd[0] = SerTrigWd[1];
			SerTrigWd[1] = SerTrigWd[2];
			SerTrigWd[2] = SerTrigWd[3];
			SerTrigWd[3] = 0x00;
			SerIgnWd[0] = SerIgnWd[1];
			SerIgnWd[1] = SerIgnWd[2];
			SerIgnWd[2] = SerIgnWd[3];
			SerIgnWd[3] = 0xff;

		}//SPI trigword position swap
	}//SPI
	else if (TrigChan == TRIG_CHAN_SER_I2C){
		for (ii = 0; ii < 4; ii++){
		    SerTrigWd[ii] = 0;
		    SerIgnWd[ii] = 0;
		    for (Dex = 0; Dex < 8; Dex++){
				chtmp = TrigI2CWd[((3-ii)*8)+Dex];
				if (chtmp == '1')
		            SerTrigWd[ii] |= (0x01 << Dex);
				else if((chtmp =='X')||(chtmp == 'x'))
		            SerIgnWd[ii] |= (0x01 << Dex);
			}
		}//I2C Parsing
		
		while(SerIgnWd[0] == 0xff){
			SerTrigWd[0] = SerTrigWd[1];
			SerTrigWd[1] = SerTrigWd[2];
			SerTrigWd[2] = SerTrigWd[3];
			SerTrigWd[3] = 0x00;
			SerIgnWd[0] = SerIgnWd[1];
			SerIgnWd[1] = SerIgnWd[2];
			SerIgnWd[2] = SerIgnWd[3];
			SerIgnWd[3] = 0xff;

		}//I2C trigword position swap
			

		}//I2C
		

		//Need to add I2C trigwd


		//serial trigger word [17] .. [24]  - must translate from menu
		HexBuf[BufTransCnt++] = BufTransUpper(0x00, SerTrigWd[3]);//SerTrigWd1
		HexBuf[BufTransCnt++] = BufTransLower(0x00, SerTrigWd[3]);
		HexBuf[BufTransCnt++] = BufTransUpper(0x01, SerTrigWd[2]);//
		HexBuf[BufTransCnt++] = BufTransLower(0x01, SerTrigWd[2]);
		HexBuf[BufTransCnt++] = BufTransUpper(0x02, SerTrigWd[1]);
		HexBuf[BufTransCnt++] = BufTransLower(0x02, SerTrigWd[1]);
		HexBuf[BufTransCnt++] = BufTransUpper(0x03, SerTrigWd[0]);
		HexBuf[BufTransCnt++] = BufTransLower(0x03, SerTrigWd[0]);

		//serial trigger word [25] .. [32] ingnore bits X   - must translate from menu
		HexBuf[BufTransCnt++] = BufTransUpper(0x04, SerIgnWd[3]);
		HexBuf[BufTransCnt++] = BufTransLower(0x04, SerIgnWd[3]);
		HexBuf[BufTransCnt++] = BufTransUpper(0x05, SerIgnWd[2]);
		HexBuf[BufTransCnt++] = BufTransLower(0x05, SerIgnWd[2]);
		HexBuf[BufTransCnt++] = BufTransUpper(0x06, SerIgnWd[1]);
		HexBuf[BufTransCnt++] = BufTransLower(0x06, SerIgnWd[1]);
		HexBuf[BufTransCnt++] = BufTransUpper(0x07, SerIgnWd[0]);//SerTrigIgn1
		HexBuf[BufTransCnt++] = BufTransLower(0x07, SerIgnWd[0]);

		HexBuf[BufTransCnt++] = BufTransUpper(0x08, TrigSPIMode);//bit 0,1 = SPI mode
		HexBuf[BufTransCnt++] = BufTransLower(0x08, TrigSPIMode);

		HexBuf[BufTransCnt++] = BufTransUpper(0x0f, (0x00 | SlowMode));
		HexBuf[BufTransCnt++] = BufTransLower(0x0f, (0x00 | SlowMode));

		//To turn on probe cal signal set (0x0f, 0x10)

		HexBuf[BufTransCnt++] = 0x7E;

		write(fd_w,HexBuf,BufTransCnt);  
		close(fd_w);
	}//fd_w
}
//-----------------------------------------------------

void ConfigureHardwareMSO19()
{
	CalcRateMSBLSB(sampleInterval);
	ClkRate_Out(ClkRateMSB, ClkRateLSB);

	OffsetDacVal[0] = CalcOffsetRawValueFromVoltage(OffsetDbl[0], 0);

    DAC_Out(OffsetDacVal[0]);
	ConfigureThresholdLevel();

    ConfigureTriggerHardware19();
	ConfigureTriggerPosition();
    
}
//-----------------------------------------------------
void ReadBuffer19()
{
	unsigned char bbb[3100];
	int	Cnt;
	int BytesToRead, AdjAddr;
	unsigned short ii, jj, mm, kk;
	int bytes;
	int lcnt = 0;


	fd_w = open(line,O_RDWR | O_NOCTTY |  O_NDELAY);	//opening serial port device for write action
	if(fd_w < 0){
         printf("Serial error RB\n");
	}
	else
	{
		portconfig(&fd_w,4090);
		tcflush(fd_w, TCIOFLUSH);

		DSOCtlBase = 0x10;
		BufTransCnt = 0;
		BufTransCnt = SerBufInit(BufTransCnt);
		HexBuf[BufTransCnt++] = BufTransUpper(0x01, 0x00);
		HexBuf[BufTransCnt++] = BufTransLower(0x01, 0x00);
		HexBuf[BufTransCnt++] = 0x7E;
		write(fd_w,HexBuf,BufTransCnt);  
		usleep(5);

		lcnt = 0;
		STOP = FALSE;
		while ((STOP==FALSE)&&(lcnt<2000)){       // loop for input 
			ioctl(fd_w, FIONREAD, &bytes);
			if(bytes >= 3070) {
				BytesToRead = read(fd_w,bbb,bytes);
				STOP = TRUE;
			}
			else{
			}
			usleep(5);
			lcnt++;
		}

		tcflush(fd_w, TCIOFLUSH);

		close(fd_w);


		AdjAddr = 0;
	int CntPos;
	int CntAdj;
        for (Cnt = AdjAddr; Cnt < (BytesToRead / 3); Cnt++)//x4
        {
            CntPos= Cnt*3;
			CntAdj=Cnt-AdjAddr;
			ii = bbb[CntPos]&0x3f;              //dso 1
            jj = (bbb[CntPos + 1] & 0x0f) << 6; //dso1

            kk = (bbb[CntPos + 1] & 0x30) >> 4; //dso2
            mm = (bbb[CntPos + 2] & 0x3f) << 2; //dso2

            AnalogDataA[CntAdj] = jj | ii;
            LogicData[CntAdj] = kk | mm;
        }
	}
}

//-----------------------------------------------------

int WriteMsoData19()
{
	FILE *fp;
	int Cnt;
	int ret;
	char sBuf[512];
	
	/* open the file */
	
	sprintf(sBuf,"tmp/mso19data%d.csv",(int)CurrSid);
	fp = fopen(sBuf, "wb");
	if(fp)
	{

        for (Cnt = 0; Cnt < 1023; Cnt++)
			fprintf(fp,"%f\t%d\n",AnalogVoltageDataA[Cnt],LogicData[Cnt]); 
		
		fclose(fp);
		ret = 1;
	}
	else ret = 0;

	printf("%d\n%d\n",29,CurrSid);
	return ret;

}
//-----------------------------------
void send_error(char *error_text)
{
    printf("Content-type: text/plain\r\n");
    printf("\r\n");
    printf("Woops:- %s\r\n", error_text);
}
//-----------------------------------------------------
/* this routine borrowed from the examples that come with the NCSA server */
char x2c(char *what)
{
    register char digit;

    digit = (what[0] >= 'A' ? ((what[0] & 0xdf) - 'A')+10 : (what[0] - '0'));
    digit *= 16;
    digit += (what[1] >= 'A' ? ((what[1] & 0xdf) - 'A')+10 : (what[1] - '0'));
    return(digit);
}


//--------------------------------

/* this routine borrowed from the examples that come with the NCSA server */
void unescape_url(char *url)
{
    int x,y;

    for (x=0,y=0; url[y]; ++x,++y) 
	{
        if ((url[x] = url[y]) == '%')
		{
            url[x] = x2c(&url[y+1]);
            y += 2;
        }
    }
    url[x] = '\0';
}
//-----------------------------------------------------

/* Assumes name_val_pairs array is currently full of NULL characters */
void load_nv_pair(char *tmp_buffer, int nv_entry)
{
    int chars_processed = 0;
    char *src_char_ptr;
    char *dest_char_ptr;

    /* get the part before the '=' sign */
    src_char_ptr = tmp_buffer;
    dest_char_ptr = name_val_pairs[nv_entry].name;
    while(*src_char_ptr && *src_char_ptr != '=' &&
          chars_processed < FIELD_LEN)
	{
        /* Change a '+' to a ' ' */
        if (*src_char_ptr == '+')
            *dest_char_ptr = ' ';
        else
            *dest_char_ptr = *src_char_ptr;
        dest_char_ptr++;
        src_char_ptr++;
        chars_processed++;
    }

    /* skip the '=' character */
    if (*src_char_ptr == '=')
	{
        /* get the part after the '=' sign */
        src_char_ptr++;
        dest_char_ptr = name_val_pairs[nv_entry].value;
        chars_processed = 0;
        while(*src_char_ptr && *src_char_ptr != '=' &&
               chars_processed < FIELD_LEN)
		{
            /* Change a '+' to a ' ' */
            if (*src_char_ptr == '+')
                *dest_char_ptr = ' ';
            else
                *dest_char_ptr = *src_char_ptr;
            dest_char_ptr++;
            src_char_ptr++;
            chars_processed++;
        }
    }

    /* Now need to decode %XX characters from the two fields */
    unescape_url(name_val_pairs[nv_entry].name);
    unescape_url(name_val_pairs[nv_entry].value);
}


//-----------------------------------------------------
int Get_input(void)
{
    int nv_entry_number = 0;
    int got_data = 0;
    char *ip_data = 0;
    int ip_length = 0;
    char tmp_buffer[(FIELD_LEN * 2) + 2];
    int tmp_offset = 0;
    char *tmp_char_ptr;
    int chars_processed = 0;

    tmp_char_ptr = getenv("REQUEST_METHOD");
    if (tmp_char_ptr)
	{
        if (strcmp(tmp_char_ptr, "POST") == 0)
		{
            tmp_char_ptr = getenv("CONTENT_LENGTH");
            if (tmp_char_ptr)
			{
                ip_length = atoi(tmp_char_ptr);
                ip_data = malloc(ip_length + 1); /* allow for NULL character */
                if (fread(ip_data, 1, ip_length, stdin) != ip_length)
				{
                    send_error("Bad read from stdin");
                    return(0);
                }
                ip_data[ip_length] = '\0';
                got_data = 1;        
            }
        }
    }

    tmp_char_ptr = getenv("REQUEST_METHOD");
    if (tmp_char_ptr)
	{
        if (strcmp(getenv("REQUEST_METHOD"), "GET") == 0)
		{
            tmp_char_ptr = getenv("QUERY_STRING");
            if (tmp_char_ptr) 
			{
                ip_length = strlen(tmp_char_ptr);
                ip_data = malloc(ip_length + 1); /* allow for NULL character */
                strcpy(ip_data, getenv("QUERY_STRING"));
                ip_data[ip_length] = '\0';
                got_data = 1;
            }
        }
    }

    if (!got_data) 
	{
        send_error("No data received");
        return(0);
    }

    if (ip_length <= 0)
	{
        send_error("Input length not > 0");
        return(0);
    }
	
//	printf("QUERY_STRING 2= ");
//	printf("%s\n",tmp_char_ptr);

    memset(name_val_pairs, '\0', sizeof(name_val_pairs));
    tmp_char_ptr = ip_data;
    while (chars_processed <= ip_length && nv_entry_number < NV_PAIRS)
	{
        /* copy a single name=value pair to a tmp buffer */
        tmp_offset = 0;
        while (*tmp_char_ptr && *tmp_char_ptr != '&' &&
               tmp_offset < FIELD_LEN)
		{
            tmp_buffer[tmp_offset] = *tmp_char_ptr;
            tmp_offset++;
            tmp_char_ptr++;
            chars_processed++;
        }
        tmp_buffer[tmp_offset] = '\0';

        /* decode and load the pair */
        load_nv_pair(tmp_buffer, nv_entry_number);

        /* move on to the next name=value pair */
        tmp_char_ptr++;
        nv_entry_number++;
    }
//	printf("NV= %s %s\n",name_val_pairs[0].name,name_val_pairs[0].value);
	

	return(1);
}
//----------------------------------
//----------------------------------
//----------------------------------
//----------------------------------
//-----------------------------------
void ReadQueryString()
{
	int nv_entry_number = 0;
	unsigned char i;
	unsigned char s;
//	unsigned char wd=0;
	char sBuf[512];
	long sidTmp;


	while(name_val_pairs[nv_entry_number].name[0] != '\0')
	{
		if(strcmp(name_val_pairs[nv_entry_number].name,"sid")==0)
			{
				sid= atol(name_val_pairs[nv_entry_number].value);
			}

			
		if(strcmp(name_val_pairs[nv_entry_number].name,"VDIV0")==0){
			vDiv[0] = atoi(name_val_pairs[nv_entry_number].value);
			SetChanged++;
		}
		if(strcmp(name_val_pairs[nv_entry_number].name,"VDIV1")==0){
			vDiv[1] = atoi(name_val_pairs[nv_entry_number].value);
			SetChanged++;
		}
		//vDiv
		if(strcmp(name_val_pairs[nv_entry_number].name,"PATTN0")==0){
			ProbeAttn[0] = atoi(name_val_pairs[nv_entry_number].value);
			SetChanged++;
		}

		if(strcmp(name_val_pairs[nv_entry_number].name,"PATTN1")==0){
			ProbeAttn[1] = atoi(name_val_pairs[nv_entry_number].value);
			SetChanged++;
		}

		//ProbeAttn
		if(strcmp(name_val_pairs[nv_entry_number].name,"ACDC0")==0)
		{
			if(strcmp(name_val_pairs[nv_entry_number].value,"DC")==0)
				ACDCMode[0] = 1;
			else
				ACDCMode[0] = 0;
			SetChanged++;
		}
		if(strcmp(name_val_pairs[nv_entry_number].name,"ACDC1")==0)
		{
			if(strcmp(name_val_pairs[nv_entry_number].value,"DC")==0)
				ACDCMode[1] = 1;
			else
				ACDCMode[1] = 0;
			SetChanged++;
		}
		//ACDCMode
		if(strcmp(name_val_pairs[nv_entry_number].name,"VOFF0")==0){
			OffsetDbl[0] = atof(name_val_pairs[nv_entry_number].value);
			SetChanged++;
		}
		if(strcmp(name_val_pairs[nv_entry_number].name,"VOFF1")==0){
			OffsetDbl[1] = atof(name_val_pairs[nv_entry_number].value);
			SetChanged++;
		}
		//OffsetDbl
		if(strcmp(name_val_pairs[nv_entry_number].name,"LAFM")==0){
			LogFam = atoi(name_val_pairs[nv_entry_number].value);
			SetChanged++;
		}
		//LogFam
		if(strcmp(name_val_pairs[nv_entry_number].name,"TSAMP")==0){
			sampleInterval = atoi(name_val_pairs[nv_entry_number].value);
			SetChanged++;
		}
		//sampleInterval
		if(strcmp(name_val_pairs[nv_entry_number].name,"TRIGV0")==0){
			TrigLevel[0] = atof(name_val_pairs[nv_entry_number].value);
			SetChanged++;
		}
		if(strcmp(name_val_pairs[nv_entry_number].name,"TRIGV1")==0){
			TrigLevel[1] = atof(name_val_pairs[nv_entry_number].value);
			SetChanged++;
		}
		//TrigLevel
		if(strcmp(name_val_pairs[nv_entry_number].name,"TRSLP0")==0)
		{
			if(strcmp(name_val_pairs[nv_entry_number].value,"R")==0)
				TrigSlope[0] = SLOPE_RISING;
			else
				TrigSlope[0] = SLOPE_FALLING;
			SetChanged++;
		}
		if(strcmp(name_val_pairs[nv_entry_number].name,"TRSLP1")==0)
		{
			if(strcmp(name_val_pairs[nv_entry_number].value,"R")==0)
				TrigSlope[1] = SLOPE_RISING;
			else
				TrigSlope[1] = SLOPE_FALLING;
			SetChanged++;
		}
		//TrigSlope	
		if(strcmp(name_val_pairs[nv_entry_number].name,"TRLAWD")==0){
			strncpy(TrigLAWord,name_val_pairs[nv_entry_number].value,8);
			SetChanged++;
		}
		//TrigLAWord	
		if(strcmp(name_val_pairs[nv_entry_number].name,"TRLASLP")==0)
		{
			if(strcmp(name_val_pairs[nv_entry_number].value,"T")==0)
				TrigSlope[2] = SLOPE_RISING;
			else
				TrigSlope[2] = SLOPE_FALLING;
			SetChanged++;
		}
		//Logic trig slope
		if(strcmp(name_val_pairs[nv_entry_number].name,"TRPOS")==0){
			TrigPosition = atoi(name_val_pairs[nv_entry_number].value);
			SetChanged++;
		}
		//TrigPosition

		if(strcmp(name_val_pairs[nv_entry_number].name,"TRSPIWD")==0){
			strncpy(TrigWdtmp,name_val_pairs[nv_entry_number].value,32);
			strncpy(TrigSPIWd,TrigWdtmp,8);
			strncpy(&TrigSPIWd[8],&TrigWdtmp[8],8);
			strncpy(&TrigSPIWd[16],&TrigWdtmp[16],8);
			strncpy(&TrigSPIWd[24],&TrigWdtmp[24],8);
			TrigSPIWd[32]=0;
			SetChanged++;
		}
		//TrigSPIWd	
		if(strcmp(name_val_pairs[nv_entry_number].name,"TRI2CWD")==0){
			strncpy(TrigWdtmp,name_val_pairs[nv_entry_number].value,32);
			strncpy(TrigI2CWd,TrigWdtmp,8);
			strncpy(&TrigI2CWd[8],&TrigWdtmp[8],8);
			strncpy(&TrigI2CWd[16],&TrigWdtmp[16],8);
			strncpy(&TrigI2CWd[24],&TrigWdtmp[24],8);
			TrigI2CWd[32]=0;
			SetChanged++;
		}
		//TrigI2CWd	


		if(strcmp(name_val_pairs[nv_entry_number].name,"SPIMODE")==0){
			TrigSPIMode = atof(name_val_pairs[nv_entry_number].value);
			SetChanged++;
		}
		//TrigSPIMode
		if(strcmp(name_val_pairs[nv_entry_number].name,"TRIGWD0")==0){
			TrigWidth[0] = atof(name_val_pairs[nv_entry_number].value);
			SetChanged++;
		}
		if(strcmp(name_val_pairs[nv_entry_number].name,"TRIGWD1")==0){
			TrigWidth[1] = atof(name_val_pairs[nv_entry_number].value);
			SetChanged++;
		}
		//TrigWidth
		if(strcmp(name_val_pairs[nv_entry_number].name,"TRMODE0")==0)
		{
			TrigModeDSO[0] = atoi(name_val_pairs[nv_entry_number].value);

			SetChanged++;
		}
		if(strcmp(name_val_pairs[nv_entry_number].name,"TRMODE1")==0)
		{
			TrigModeDSO[1] = atoi(name_val_pairs[nv_entry_number].value);
			SetChanged++;
		}
		//TrigModeDSO	
		if(strcmp(name_val_pairs[nv_entry_number].name,"TRGCH")==0)
		{
			if(strcmp(name_val_pairs[nv_entry_number].value,"A0")==0)
				TrigChan = TRIG_CHAN_DSO;
			else if(strcmp(name_val_pairs[nv_entry_number].value,"A1")==0)
				TrigChan = TRIG_CHAN_DSO1;
			else if(strcmp(name_val_pairs[nv_entry_number].value,"LA")==0)
				TrigChan = TRIG_CHAN_LA;
			else if(strcmp(name_val_pairs[nv_entry_number].value,"I2C")==0)
				TrigChan = TRIG_CHAN_SER_I2C;
			else if(strcmp(name_val_pairs[nv_entry_number].value,"SPI")==0)
				TrigChan = TRIG_CHAN_SER_SPI;
			else
				TrigChan = TRIG_CHAN_DSO;
			SetChanged++;
		}
		//TrigChan
		if(strcmp(name_val_pairs[nv_entry_number].name,"w")==0)
		{
			s = atoi(name_val_pairs[nv_entry_number].value);
			usleep(s*10000);
		}
		//wait

		if(strcmp(name_val_pairs[nv_entry_number].name,"i")==0)
		{
			GetSerPort19();
			if(strcmp(name_val_pairs[nv_entry_number].value,"I")==0)
			{
				ResetADC();
		//		SPI_Read_Buf_Page(2, &MBuf[0], 0);
		//		SPI_Read_Buf_Page(2, &MBuf[0], 1);
		//		WriteMsoParam();
		//		LEDOff();
				MsoBusy=0;
				printf("P_Rdy\n%d\n",sid);	
			}
			else if(strcmp(name_val_pairs[nv_entry_number].value,"C")==0
			        ||strcmp(name_val_pairs[nv_entry_number].value,"c")==0)
			{
				GetUSBVid();
				ResetADC();
				ReadMso19Param();
				Parse19Sn(&FBuf[0]);
				sprintf(sBuf,"rm tmp/mso19data*\n");
				system(sBuf);
				MsoBusy=0;

				if(strcmp(name_val_pairs[nv_entry_number].value,"c")==0){
		//			PrintMsoParam();
					printf("%d\n",sid);
				}
				else
					printf("Connected\n%d\n",sid);	

			}
			else if(strcmp(name_val_pairs[nv_entry_number].value,"a")==0)
			{
				if(!MsoBusy){
					//ResetFSM();
					PrevSid=CurrSid;
					MsoBusy=1;
					sidTmp = RotateSid(PrevSid);
					if(sidTmp!=0){
						sprintf(sBuf,"rm tmp/mso19data%d.csv\n",(int)sidTmp);
						system(sBuf);
					}

					ArmMSO();
					CurrSid=sid;
	//				printf("Armed\n\n");	
					printf("%d\n%d\n",28,CurrSid);
				}
				else printf("%d\n%d\n",36,CurrSid);

			}
			else if(strcmp(name_val_pairs[nv_entry_number].value,"A")==0)
			{
				if(!MsoBusy){
					PrevSid=CurrSid;
					MsoBusy=1;
					sidTmp = RotateSid(PrevSid);
					if(sidTmp!=0){
						sprintf(sBuf,"rm tmp/mso19data%d.csv\n",(int)sidTmp);
						system(sBuf);
					}
					if(SetChanged){
						WriteMsoSettings();//write to msoset.txt
						SetChanged = 0;
					}
					ReadMso19Param();
					Parse19Sn(&FBuf[0]);
					InitSettings();
					ReadMsoSettings19();//read back from msoset.txt
					ConfigureHardwareMSO19();
					CurrSid=sid;
					printf("%d\n%d\n",27,CurrSid);	
					ArmMSO();
	//				printf("Setup & Armed\n\n");	
				}
				else printf("%d\n%d\n",37,CurrSid);	

			}
			else if(strcmp(name_val_pairs[nv_entry_number].value,"M")==0)
			{
				ReadMso19Param();
				Parse19Sn(&FBuf[0]);
				InitSettings();
				ReadMsoSettings19();//read back from msoset.txt
				ConfigureHardwareMSO19();
//				printf("MSO Setup\n\n");	
				printf("%d\n%d\n",26,sid);	
			}
			else if(strcmp(name_val_pairs[nv_entry_number].value,"F")==0)
			{
				ResetFSM();
		//		printf("ResetFSM\n\n");	
				MsoBusy=0;
				printf("%d\n%d\n",30,sid);	
			}
			else if(strcmp(name_val_pairs[nv_entry_number].value,"X")==0)
			{
		//		ForceCapture();
//				printf("ForceCap\n\n");	
				MsoBusy=0;
				printf("%d\n%d\n",30,sid);	
			}

			else if(strcmp(name_val_pairs[nv_entry_number].value,"Q")==0)
			{
		//		LEDOff();
//				printf("Off\n\n");	
	
				printf("Bye\n%d\n",sid);	
			}
			else if(strcmp(name_val_pairs[nv_entry_number].value,"p")==0)
			{
				ReadMso19Param();
				Parse19Sn(&FBuf[0]);
		//		PrintMsoParam();
				InitSettings();
				ReadMsoSettings19();//read back from msoset.txt
		//		PrintMSOSettings();	
				printf("%d\n",sid);
			}
			else if((strcmp(name_val_pairs[nv_entry_number].value,"T")==0)||
			        (strcmp(name_val_pairs[nv_entry_number].value,"t")==0))
			{
				i = CheckTriggerStatus();
				i&=0x1f;
//				printf("TrigStat = %x\n\n",i);	
				if(strcmp(name_val_pairs[nv_entry_number].value,"T")==0)
					printf("%d\n%d\n",i,sid);
				else{
					if(i==22){
						if(sid==CurrSid){
						ReadBuffer19();
						VoltageConvert();
						WriteMsoData19();
		//				wd =1;
						MsoBusy=0;
						printf("%d\n%d\n",29,CurrSid);
						}
						else printf("%d\n%d\n",35,CurrSid);
					}//22 waiting for Readback
					else printf("%d\n%d\n",i,sid);
				}//"t"

			} //"t or T"
			else if((strcmp(name_val_pairs[nv_entry_number].value,"B")==0)||
				(strcmp(name_val_pairs[nv_entry_number].value,"b")==0))
			{
				ReadMso19Param();
				Parse19Sn(&FBuf[0]);
				ReadBuffer19();

				if(strcmp(name_val_pairs[nv_entry_number].value,"b")==0){
					for(i=0;i<2;i++) printf("vbit[%d]=%f\n",i,vbit[i]);
					for(i=0;i<2;i++) printf("OffsetVBit[%d]=%f\n",i,OffsetVBit[i]);
					for(i=0;i<2;i++) printf("OffsetCenterVal[%d]=0x%x\n",i,OffsetCenterVal[i]);
					printf("\n");
					for(i=0;i<2;i++) printf("vbit200[%d]=%f\n",i,vbit200[i]);
					for(i=0;i<2;i++) printf("OffsetVBit200[%d]=%f\n",i,OffsetVBit200[i]);
					for(i=0;i<2;i++) printf("OffsetCenterVal200[%d]=0x%x\n",i,OffsetCenterVal200[i]);
				}
				VoltageConvert();
				WriteMsoData19();
				MsoBusy=0;
			}
		} 
	   nv_entry_number++;
	}

}
//-----------------------------------

int main()
{
	line = (char *) malloc(18);
	printf("Hello world\n");
	GetUSBVid();
//	GetSerPort19();

	printf("%s\n",line);
	InitSettings();
	printf("ReadMsoSettings %d\n",ReadMsoSettings19());//read back from msoset.txt//
//	ResetADC();

	while (FCGI_Accept() >= 0) {
	printf("FCGI_in\n");

		if (!Get_input())
		{
        		exit(EXIT_FAILURE);
		}
			printf("Content-Type: text/plain\n");
			printf("Cache-Control: no-cache,no-store\n");
			printf("Status: 200 OK\n\n");
			SetChanged = 0;
			ReadQueryString();
			if(SetChanged)	WriteMsoSettings();//write to msoset.txt
	//		printf("\n\n");
	} // while 
	
	free(line);	return (0);
}


