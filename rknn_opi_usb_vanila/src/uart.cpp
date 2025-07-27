#include "uart.hpp"
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <iostream>
#include <cstring>

UART::UART(const std::string& device, int baudrate) {
    serial_fd = open(device.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    connected = (serial_fd >= 0);

    if (!connected) {
        std::cerr << "Fehler beim Öffnen von " << device << std::endl;
        return;
    }

    termios tty{};
    if (tcgetattr(serial_fd, &tty) != 0) {
        std::cerr << "Fehler beim Lesen der UART-Konfiguration\n";
        connected = false;
        return;
    }

    cfsetispeed(&tty, baudrate);
    cfsetospeed(&tty, baudrate);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8 Datenbits
    tty.c_iflag &= ~IGNBRK;                         // kein Break ignorieren
    tty.c_lflag = 0;                                // kein Canonical Mode
    tty.c_oflag = 0;                                // keine Verarbeitung
    tty.c_cc[VMIN] = 0;                             // kein Mindestinput
    tty.c_cc[VTIME] = 10;                           // Timeout 1s

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);         // kein Flow-Control
    tty.c_cflag |= (CLOCAL | CREAD);                // lokal, Lesen aktivieren
    tty.c_cflag &= ~(PARENB | PARODD);              // keine Parität
    tty.c_cflag &= ~CSTOPB;                         // 1 Stopbit
    tty.c_cflag &= ~CRTSCTS;                        // kein Hardware-Flow

    tcsetattr(serial_fd, TCSANOW, &tty);
}

UART::~UART() {
    if (connected) {
        close(serial_fd);
    }
}

bool UART::isOpen() const {
    return connected;
}

void UART::send(const std::string& message) {
    if (connected) {
        write(serial_fd, message.c_str(), message.length());
    }
}