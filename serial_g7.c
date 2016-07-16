
/*
 * G7 PM2.5传感器C语言实现版本
 * bruce.zhu@2015.11.02
 * linuxpro@2016.07.16
 *
 */

#include <stdlib.h>
#include <stdio.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>

#include "serial_helper.h"

#define SERIAL_PORT "/dev/ttyUSB0"
#define PACKAGE_LEN 32

static void dump_package(char *buf, int len);

static void
handle_package(char *package)
{
    //printf("-----------------\n");
    dump_package(package, 24);

    // check data package length, should be 20
    int package_length = package[2] * 256 + package[3];
    if (package_length != 28) {
        printf("RECV data package length error[20, %d]\n", package_length);
        return;
    }

    // check CRC
    int crc = 0;
    int i;
    for (i = 0; i < PACKAGE_LEN - 2; i++) {
        crc += package[i];
    }
    crc = crc % (256*256);
    int package_crc = package[30] * 256 + package[31];
    if (package_crc != crc) {
        printf("data package crc error[%d, %d]\n", package_crc, crc);
        return;
    }

    // all is OK, let's get real value
    int index = 4;
    if (package[0] == 0x42 && package[1] == 0x4d) {
        // PM1.0(CF=1)
        int pm1_0 = package[4] * 256 + package[5];
        // PM2.5(CF=1)i
        int pm2_5 = package[6] * 256 + package[7];
        // PM10(CF=1)
        int pm10 = package[8] * 256 + package[9];
        //printf("(CF=1) -> [%d, %d, %d]\n", pm1_0, pm2_5, pm10);
        printf("(CF=1) -> [%d]\n", pm2_5);
	sleep(1);

        // PM1.0(大气环境下)
        int pm_air_1_0 = package[10] * 256 + package[11];
        // PM2.5(大气环境下)
        int pm_air_2_5 = package[12] * 256 + package[13];
        // PM10(大气环境下)
        int pm_air_10 = package[14] * 256 + package[15];
        //printf("大气环境 -> [%d, %d, %d]\n", pm_air_1_0, pm_air_2_5, pm_air_10);
      //  printf("大气环境 -> [%d]\n", pm_air_2_5);

	sleep(1);
        int pm0_3 = package[16] * 256 + package[17];
        int pm0_5 = package[18] * 256 + package[19];
        int Pm1_0 = package[20] * 256 + package[21];
        int Pm2_5 = package[22] * 256 + package[23];
        int pm5_0 = package[24] * 256 + package[25];
        int pm10_0 = package[26] * 256 + package[27];
        //printf("0.1升空气中个数分别是0.3um,0.5um,1.0um,2.5um,5.0um,10um -> [%d, %d, %d, %d, %d, %d]\n", pm0_3, pm0_5, Pm1_0, Pm2_5, pm5_0, pm10_0);
        printf("0.1升空气中个数是2.5um -> [%d]\n",Pm2_5);

	
	char buf[1024];
	sprintf(buf , "curl --request POST http://www.lewei50.com/api/V1/Gateway/UpdateSensors/01 --data \"[{'Name':'dust','Value':%d},{'Name':'AQI','Value':%d}]\" --header \"userkey:XXXX\"", pm2_5, pm_air_2_5);
	system(buf);

	sleep(12);


	int file;
	char *bus = "/dev/i2c-1";
	if ((file = open(bus, O_RDWR)) < 0) 
	{
		printf("Failed to open the bus. \n");
		exit(1);
	}
	// Get I2C device, SHT30 I2C address is 0x44(68)
	ioctl(file, I2C_SLAVE, 0x44);

	// Send measurement command(0x2C)
	// High repeatability measurement(0x06)
	char config[2] = {0};
	config[0] = 0x2C;
	config[1] = 0x06;
	write(file, config, 2);
	sleep(1);

	// Read 6 bytes of data
	// Temp msb, Temp lsb, Temp CRC, Humididty msb, Humidity lsb, Humidity CRC
	char data[6] = {0};
	if(read(file, data, 6) != 6)
	{
		printf("Erorr : Input/output Erorr \n");
	}
	else
	{
		// Convert the data
		int temp = (data[0] * 256 + data[1]);
		float cTemp = -45 + (175 * temp / 65535.0);
		float fTemp = -49 + (315 * temp / 65535.0);
		float humidity = 100 * (data[3] * 256 + data[4]) / 65535.0;

		// Output data to screen
		printf("Relative Humidity : %.2f RH \n", humidity);
		printf("Temperature in Celsius : %.2f C \n", cTemp);
		printf("Temperature in Fahrenheit : %.2f F \n", fTemp);

		char buf[1024];
		sprintf(buf , "curl --request POST http://www.lewei50.com/api/V1/Gateway/UpdateSensors/01 --data \"[{'Name':'T1','Value':%f},{'Name':'H1','Value':%f}]\" --header \"userkey:XXX\"", cTemp,humidity);
		system(buf);
		sleep(12);
		
	}



    } else {
        printf("package length error\n");
    }
}

static int
serial_callback(int fd, char* buf, int len)
{
    static int package_index = 0;
    static char whole_package[PACKAGE_LEN];

    //printf("package len = %d\n", len);
    dump_package(buf, len);

    int i;
    for (i = 0; i < len; i++) {
        if (package_index == 0) {
            if (buf[i] == 0x42 && buf[i+1] == 0x4d) {
                whole_package[package_index++] = buf[i];
            }
        } else if (package_index < PACKAGE_LEN) {
            whole_package[package_index++] = buf[i];
        }

        if (package_index == PACKAGE_LEN) {
            handle_package(whole_package);
            package_index = 0;
        }
    }

    return 0;
}

static void
dump_package(char *buf, int len)
{
    int i;
   // for (i = 0; i < len; i++) {
        printf("hold");
        //printf("%x ", buf[i]);
   // }

    printf("\n");
}


static struct serial_helper_t serial = {
    .dev_path = SERIAL_PORT,
    .baud_rate = 9600,
    .data_bits = 8,
    .serial_notification = serial_callback
};


int main()
{
    int ret = serial_helper_register(&serial);
    if (ret) printf("serial open failed\n");
    else printf("serial open success\n");

    for (;;) {
        sleep(1);
    }
}



