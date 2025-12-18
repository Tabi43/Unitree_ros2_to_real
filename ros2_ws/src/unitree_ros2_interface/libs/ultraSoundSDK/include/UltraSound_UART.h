#ifndef ULTRASOUND_UART
#define ULTRASOUND_UART

#include <string>
#include <stdint.h>
#include <vector>

#include <iostream>
#include <sys/time.h>
#include <unistd.h>

//时间戳  微秒级， 需要#include <sys/time.h> 
inline long long getSystemTime(){
    struct timeval t;  
    gettimeofday(&t, NULL);
    return 1000000 * t.tv_sec + t.tv_usec;  
}

enum class UltraSoundHardware{
	doubleProbe,
	singleProbe
};

struct HardwareSetting{
	HardwareSetting(UltraSoundHardware type, uint freqPosID);

	uint8_t TVGAIN0;
	uint8_t TVGAIN1;
	uint8_t TVGAIN2;
	uint8_t TVGAIN3;
	uint8_t TVGAIN4;
	uint8_t TVGAIN5;
	uint8_t TVGAIN6;

	uint8_t INIT_GAIN;
	uint8_t FREQUENCY;
	uint8_t DEADTIME;
	uint8_t PULSE_P1;
	uint8_t PULSE_P2;
	uint8_t CURR_LIM_P1;
	uint8_t CURR_LIM_P2;

	uint8_t REC_LENGTH;
	uint8_t FREQ_DIAG;
	uint8_t SAT_FDIAG_TH;
	uint8_t FVOLT_DEC;
	uint8_t DECPL_TEMP;

	uint8_t DSP_SCALE;
	uint8_t TEMP_TRIM;
	uint8_t P1_GAIN_CTRL;
	uint8_t P2_GAIN_CTRL;
	uint8_t EE_CRC;
	uint8_t EE_CNTRL;

	uint8_t P1_THR_0;
	uint8_t P1_THR_1;
	uint8_t P1_THR_2;
	uint8_t P1_THR_3;
	uint8_t P1_THR_4;
	uint8_t P1_THR_5;
	uint8_t P1_THR_6;
	uint8_t P1_THR_7;
	uint8_t P1_THR_8;
	uint8_t P1_THR_9;
	uint8_t P1_THR_10;
	uint8_t P1_THR_11;
	uint8_t P1_THR_12;
	uint8_t P1_THR_13;
	uint8_t P1_THR_14;
	uint8_t P1_THR_15;

	uint8_t P2_THR_0;
	uint8_t P2_THR_1;
	uint8_t P2_THR_2;
	uint8_t P2_THR_3;
	uint8_t P2_THR_4;
	uint8_t P2_THR_5;
	uint8_t P2_THR_6;
	uint8_t P2_THR_7;
	uint8_t P2_THR_8;
	uint8_t P2_THR_9;
	uint8_t P2_THR_10;
	uint8_t P2_THR_11;
	uint8_t P2_THR_12;
	uint8_t P2_THR_13;
	uint8_t P2_THR_14;
	uint8_t P2_THR_15;
};

/* positionID: 0=head, 1=left, 2=right, 3=tail */
/* All the UltraSound have to run under same thread */
class UltraSound{
public:
	UltraSound(uint8_t positionID);
    ~UltraSound(){};

	bool measureDistance(double &distance);
	void ultrasonicCmd(int preSet);
	bool pullUltrasonicMeasResult(double &dis);
private:
	int _fd, _epfd;
	int _gpio18;	// wiringPi Pin for gpio18, only used on Raspberry Pi
	int _gpio23;	// wiringPi Pin for gpio23, only used on Raspberry Pi

	uint8_t _positionID;
	HardwareSetting *_st;
	void open_set(char* dev_name);
	int setPort();			// only used on Raspberry Pi, to set the gpio18 and gpio23
	int serialSend(uint8_t* msg, int msgLen);
	bool serialRead(uint8_t* rsv, int rsvLen);
	bool burnEEPROM();
	double printUltrasonicMeasResult(uint8_t umr);
	double printUltrasonicMeasResultExt(uint8_t umr, int speedSound);
	double runDiagnostics(uint8_t run, uint8_t diag);

	uint8_t calcChecksum(uint8_t cmd);	

	// void tciRecord(uint8_t numObj);
	// void tciuint8_tToggle(uint8_t data, uint8_t zeroPadding);
	// void tciIndexRW(uint8_t index, bool write);
	// void tciCommand(uint8_t cmd);
	// void spiTransfer(uint8_t* mosi, uint8_t size);
	// void spiMosiIdle(uint8_t size);

	uint8_t ultraMeasResult[34+3]; 	// data byte array for cmd5 and tciB+L return

	uint8_t syncByte = 0x55; 		// data byte for Sync field set UART baud rate of UltraSound
	uint8_t uartAddr = 0; 			// UltraSound UART device address (0-7). '0' is factory default address
	uint8_t ChecksumInput[44]; 		// data byte array for checksum calculator
	uint8_t numObj = 1; 			// number of objects to detect
	uint8_t tempOrNoise = 0; 			// data byte to determine if temp or noise measurement is to be performed
	uint8_t regAddr = 0x00; 		// data byte for Register Address
	uint8_t regData = 0x00; 		// data byte for Register Data

	uint8_t P1BL;
	uint8_t P2BL;
	uint8_t P1LO;
	uint8_t P2LO;
	uint8_t TNLM;
	uint8_t UMR;
	uint8_t TNLR;
	uint8_t TEDD;
	uint8_t SD;
	uint8_t SRR;
	uint8_t SRW;
	uint8_t EEBR;
	uint8_t EEBW;
	uint8_t TVGBR;
	uint8_t TVGBW;
	uint8_t THRBR;
	uint8_t THRBW;

	uint8_t USER_DATA1 = 0x00;
	uint8_t USER_DATA2 = 0x00;
	uint8_t USER_DATA3 = 0x00;
	uint8_t USER_DATA4 = 0x00;
	uint8_t USER_DATA5 = 0x00;
	uint8_t USER_DATA6 = 0x00;
	uint8_t USER_DATA7 = 0x00;
	uint8_t USER_DATA8 = 0x00;
	uint8_t USER_DATA9 = 0x00;
	uint8_t USER_DATA10 = 0x00;
	uint8_t USER_DATA11 = 0x00;
	uint8_t USER_DATA12 = 0x00;
	uint8_t USER_DATA13 = 0x00;
	uint8_t USER_DATA14 = 0x00;
	uint8_t USER_DATA15 = 0x00;
	uint8_t USER_DATA16 = 0x00;
	uint8_t USER_DATA17 = 0x00;
	uint8_t USER_DATA18 = 0x00;
	uint8_t USER_DATA19 = 0x00;
	uint8_t USER_DATA20 = 0x00;

};

class UltraSoundGroup{
public:
	UltraSoundGroup(std::vector<uint8_t> id);
	std::vector<double> getDistance();
private:
	std::vector<UltraSound*> _probeGroup;
	std::vector<double> _distance;
	std::vector<bool> _detected;
};

#endif	// ULTRASOUND_UART
