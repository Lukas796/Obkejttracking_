// serial_utils.cpp
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <string.h>

// Diese Funktion initialisiert den seriellen Port.
// device: z. B. "/dev/ttyACM0" oder "/dev/ttyUSB0"
// baud_rate: z. B. 9600
int serial_init(const char *device, int baud_rate) {
    int fd = open(device, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd < 0) {
        perror("serial_init: open failed");
        return -1;
    }
    // Setze den Dateideskriptor in den blockierenden Modus
    fcntl(fd, F_SETFL, 0);

    struct termios options;
    if (tcgetattr(fd, &options) < 0) {
        perror("serial_init: tcgetattr failed");
        close(fd);
        return -1;
    }

    speed_t speed;
    switch (baud_rate) {
        case 9600:   speed = B9600; break;
        case 19200:  speed = B19200; break;
        case 38400:  speed = B38400; break;
        case 57600:  speed = B57600; break;
        case 115200: speed = B115200; break;
        default:     speed = B9600; break;
    }
    cfsetispeed(&options, speed);
    cfsetospeed(&options, speed);

    // 8 Datenbits, keine Parität, 1 Stop-Bit (8N1)
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cflag |= (CLOCAL | CREAD);

    // Setze den "Raw"-Modus: Keine Kanonisierung, kein Echo, etc.
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_oflag &= ~OPOST;

    tcflush(fd, TCIFLUSH);

    if (tcsetattr(fd, TCSANOW, &options) < 0) {
        perror("serial_init: tcsetattr failed");
        close(fd);
        return -1;
    }
    return fd;
}

// Diese Funktion gibt den seriellen Port wieder frei.
void serial_deinit(int fd) {
    close(fd);
}