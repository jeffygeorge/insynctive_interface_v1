//============================================================================
// Name        : ztest.cpp
// Author      : 
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
using namespace std;
typedef unsigned char uint8_t;

int fd ;

#define SmartSync_POLY 0x85
/************* SmartSync_CRC8_byte *******************
*   Implementing George's CRC calculation in C for SmartSync's RF CRC
*   @param  uint8_t crcVal  This iteration of CRC calculations will use this value to start out
*   @param  uint8_t data    This is the data that will generate the resulting CRC
*   @return uint8_t crcVal  The resulting CRC
**************************************************/
uint8_t SmartSync_CRC8_byte( uint8_t crcVal, uint8_t data ){

	uint8_t carryBit = 0;
        uint8_t bitMask = 0x80;
        
	for(; bitMask != 0; bitMask >>= 1){

                carryBit = crcVal & 0x80;

                crcVal <<= 1;

		if( ( bitMask & data ) != 0 ){               //bit is a 1

			if( carryBit == 0 ){

				crcVal ^= SmartSync_POLY;

			}

		}else{                                      //bit is a 0

			if( carryBit ){                     //note: carryBit is either a 0 (false) or an 0x80 which is a logical (true) since it's just NOT zero

				crcVal ^= SmartSync_POLY;

			}

		}

	}

	return crcVal;

}

/************* SmartSync_CRC8_byteBuffer *******************
*   Implementing George's CRC calculation in C for SmartSync's RF CRC
*   @param  uint8_t* byteBuffer  A pointer to an array of bytes that the crc will iterate through to calculate the CRC
*   @param  uint8_t bufferLength The size of the array.
*   @return uint8_t The result of the CRC calculation
********************************************************/
uint8_t SmartSync_CRC8_byteBuffer(const uint8_t* byteBuffer, const uint8_t bufferLength){

    uint8_t crcResult = 0;
    int i = 0;

    printf("length: %d\n", bufferLength);
    for(; i < bufferLength ; ++i ){

        crcResult = SmartSync_CRC8_byte( crcResult, byteBuffer[i] );
        printf("buff[%d]: %d crcResult:  %x\n", i, byteBuffer[i], crcResult);

    }

    printf("%s crc returned: %d\n", __FUNCTION__, crcResult);

    return crcResult;
}
#define SEND()  \
{\
    unsigned char pkt[40] = {1, 0x41, 2};  \
    test[test[0]] = SmartSync_CRC8_byteBuffer((const uint8_t*)test+1, test[0]-1); \
    cout << "Sending response num : " <<  test[0]+1 << endl; \
    int num = 3;\
    for(int j = 0; j < test[0]+1; j++) \
    { \
       if(test[j] == 0x1b || test[j] == 1 || test[j]==2 || test[j] == 3)\
            pkt[num++] = 0x1b;\
        pkt[num++] = test[j];\
    }  \
    pkt[num++] = 3;\
    \
    for(int j = 0; j < num; j++) \
    { \
        printf(" %2x", pkt[j]); \
        write(fd, &pkt[j], 1); \
    } \
    printf("\n"); \
}

void Sensor(int argc, char *argv[])
{

    unsigned char test[] = {
    7,
    0,  //device type + sn
    0,  // sn
    0,  // sn
    0xd,  // data
    0x0, 0x0,  //rolling code
    0x0};  //crc
    
    int val;
    cout << "Device type + sn : " ;
    cin >> hex >> val;
    test[1] = (char)((val & 0xff0000) >> 16);
    
    cout << endl << "test[1]: "<< hex << test[1];
    
    test[2] = (char)((val & 0x00ff00) >> 8);
    cout << endl << "test[2]: "<< hex << test[2];
    
    test[3] = (char)(val & 0xff);
    cout << endl << "test[3]: "<< hex << test[3];
    
    cout << endl << "data:";
    cin >> hex >> val;
    test[4] = val;
    
    SEND();
}

void ShadeLearn(int argc, char *argv[])
{

    unsigned char test[] = {
    5,
    0,  //device type + sn
    0,  // sn
    0,  // sn
    0x7e,  // data
    0x0};  //crc
    
    int val;
    cout << "Device type + sn : " ;
    cin >> hex >> val;
    test[1] = (char)((val & 0xff0000) >> 16);
    
    
    test[2] = (char)((val & 0x00ff00) >> 8);
    
    test[3] = (char)(val & 0xff);
    
    SEND();
}

void ShadeStatus(int argc, char *argv[])
{

    unsigned char test[] = {
    6,
    0,  //device type + sn
    0,  // sn
    0,  // sn
    0x0,  // cmd
    0x0,  //data
    0x0};  //crc
    
    int val;
    cout << "Device type + sn : " ;
    cin >> hex >> val;
    test[1] = (char)((val & 0xff0000) >> 16);
    
    
    test[2] = (char)((val & 0x00ff00) >> 8);
    
    test[3] = (char)(val & 0xff);
    
    cout << endl << "data:";
    cin >> hex >> val;
    test[4] = val;
    
    cout << endl << "value:" ;
    cin >> hex >> val;
    test[5] = val;
    cout <<"Got: " << (int)test[5] << endl;
    
    SEND();
}

void SMLearn(int argc, char *argv[])
{

    unsigned char test[] = {
    5,
    0,  //device type + sn
    0,  // sn
    0,  // sn
    0x40,  // data
    0x0};  //crc
    
    int val;
    cout << "Device type + sn : " ;
    cin >> hex >> val;
    test[1] = (char)((val & 0xff0000) >> 16);
    test[2] = (char)((val & 0x00ff00) >> 8);
    test[3] = (char)(val & 0xff);
    
    SEND();
}

void SMStatus(int argc, char *argv[])
{

    unsigned char test[] = {
    5,
    0,  //device type + sn
    0,  // sn
    0,  // sn
    0x40,  // data
    0x0};  //crc
    
    int val;
    cout << "Device type + sn : " ;
    cin >> hex >> val;
    test[1] = (char)((val & 0xff0000) >> 16);
    test[2] = (char)((val & 0x00ff00) >> 8);
    test[3] = (char)(val & 0xff);
    
    cout << endl << "Status byte: " ;
    cin >> hex >> val;
    test[4] = val;
    
    SEND();
}


void  (*funcPtr[])(int argc, char *argv[]) =
{
&Sensor,
&ShadeLearn,
&ShadeStatus,
&SMLearn,
&SMStatus
};

void UartRead();

int main(int argc, char *argv[]) {
	int port = atoi(argv[1]);
    char tmp[20] ;
    #ifdef x86
	sprintf(tmp, "/dev/pts/%d", port);
    #endif
	sprintf(tmp, "/dev/ttySP%d", port);
	fd = open(tmp, O_RDWR);
	
    cout << "port: " << port << std::endl << "argv[2]: " << argv[2] << std::endl;
	cout << "num args: " << argc << endl;
	if(argv[2][0] == 'w')
	{
	    int i = atoi(argv[3]);
	    cout << "i: " << i << std::endl;
        funcPtr[i](argc, argv);
	}
	else	
	{
	    UartRead();
	}

	cout << "Done..." << endl; // prints !!!Hello World!!!
	return 0;
}

enum PACKET_FIELD
{
    SOH = 0x01,
    STX = 0x02,
    ETX = 0x03,
    ESC = 0x1b
};
void UartRead()
{
    char tmp[20];
    int num,i,j;
    int numPackets = 0;
    #define FILE_NAME "/bridge/log/itest_r.log"
    FILE* logfd = fopen(FILE_NAME, "w+");
    
    while(1)
    {
        num = read(fd, tmp, 20);
        i = 2;
        if(tmp[i] == STX && tmp[i-2]==SOH)
        {
           while( i < num && tmp[i] != ETX)
           {
               i++;
                if(tmp[i] == ESC)
                {
                     if(tmp[i+1] == ETX)
                         i++;
                }
           }
           if(tmp[i++] == ETX)
           {
               numPackets ++;
               fprintf(logfd, "Num Packets: %d [", numPackets);
               printf("Num Packets: %d [", numPackets);
               for(j = 0; j < i; j++)
               {
                   fprintf(logfd, "%02x ", tmp[j]);
                   printf("%02x ", tmp[j]);
               }
               fprintf(logfd, "]\n");
               printf("]\n");
               fflush(logfd);
           }
           if(i < num)
           {
               j = i;
               for(i = 0; j < num; i++,j++)
                    tmp[i]=tmp[j];
           }
        }
    }    
    fclose(logfd);    
}
                   
