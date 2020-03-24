#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

#include <stdio.h>

#include "vtx_tramp.h"

#include "io/vtx_control.h"
#include "drivers/vtx_common.h"

#include <signal.h>

#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

typedef void* serialPort_t;
typedef struct
{
    int identifier;
} serialPortConfig_t;

static serialPortConfig_t spc = {
    0
};

serialPortConfig_t *findSerialPortConfig(int x)
{
    return &spc;
}
typedef enum
{
    SERIAL_BIDIR = 0,
    SERIAL_UNIDIR
} portOptions_e;

int set_interface_attribs (int fd, int speed, int parity)
{
        struct termios tty;
        if (tcgetattr (fd, &tty) != 0)
        {
                printf("error %d from tcgetattr", errno);
                return -1;
        }

        cfsetospeed (&tty, speed);
        cfsetispeed (&tty, speed);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
        // disable IGNBRK for mismatched speed tests; otherwise receive break
        // as \000 chars
        tty.c_iflag &= ~IGNBRK;         // disable break processing
        tty.c_lflag = 0;                // no signaling chars, no echo,
                                        // no canonical processing
        tty.c_oflag = 0;                // no remapping, no delays
        tty.c_cc[VMIN]  = 0;            // read doesn't block
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

        tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                        // enable reading
        tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
        tty.c_cflag |= parity;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
        {
                printf("error %d from tcsetattr", errno);
                return -1;
        }
        return 0;
}

void set_blocking (int fd, int should_block)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
                printf("error %d from tggetattr", errno);
                return;
        }

        tty.c_cc[VMIN]  = should_block ? 1 : 0;
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
                printf("error %d setting term attributes", errno);
}

int fd;

serialPort_t *openSerialPort(int id, int func, void* x, void* y, int speed, int mode, portOptions_e opts)
{
    const char *portname = "/dev/tty.usbmodem0x80000001";

    printf("open serial port...\n");
    fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC | O_NONBLOCK);
    printf("FD: %i\n", fd);

    if (fd < 0)
    {
            printf("error %d opening %s: %s", errno, portname, strerror(errno));
            return NULL;
    }

    set_interface_attribs(fd, B115200, 0);  // set speed to 115,200 bps, 8n1 (no parity)
    set_blocking(fd, 0);                // set no blocking

    return (serialPort_t*)(void*)1;
}

void serialWriteBuf(serialPort_t* port, uint8_t* buf, int size)
{
    static int failcount = 0;
    if (failcount > 0 && buf[1] == 0x49)
    {
        failcount--;
        printf("blocked write: %i\n", 20-failcount);
        return;
    }

    printf("Wrote:");
    for (int i = 0; i < size; i++)
        printf(" %02X", buf[i]);
    printf("\n");
    write(fd, buf, size);

    return;
}

char c;

int serialRxBytesWaiting(serialPort_t* port)
{
    return read(fd, &c, sizeof(c));
}

uint8_t serialRead(serialPort_t* port)
{
    printf("Read: %02X\n", (unsigned char)c);
    return c & 0xff;
}

void vtxInit()
{
    return;
}

vtxConfig_t vtxConfig_System = {
    .halfDuplex = 0
};

int vtxTablePowerLevels;
uint32_t vtxTablePowerValues[10];
const char* vtxTablePowerLabels[10];

bool vtxCommonLookupPowerValue(const vtxDevice_t *vtxDevice, int index, uint16_t *pPowerValue)
{
    return false;
}

static vtxDevice_t *tramp;

void vtxCommonSetDevice(vtxDevice_t *vtxDevice)
{
    tramp = vtxDevice;

    return;
}

void my_handler(int s)
{
    printf("Caught signal %d\n",s);
    exit(1);
}

int main(int argc, char **argv)
{
    int time = 0;

    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = my_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, NULL);

    if (vtxTrampInit())
    {
        tramp->vTable->setPitMode(tramp, 1);

        while(true)
        {
            time += 200 * 1000;
            tramp->vTable->process(tramp, time);
            usleep(200 * 1000);
        }
    }

    return 0;
}
