#include "uart.hpp"
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <iostream>
#include <cstring>
#include <chrono>

int setupSerial(const char* device) {
    int serial_port = open(device, O_RDWR | O_NOCTTY);
    if (serial_port < 0) {
        std::cerr << "❌ Fehler beim Öffnen von " << device << std::endl;
        return -1;
    }

    termios tty{};
    if (tcgetattr(serial_port, &tty) != 0) {
        std::cerr << "❌ Fehler beim Abrufen der TTY-Eigenschaften\n";
        return -1;
    }

    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);

    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_cc[VMIN] = 0;    // nicht blockierend
    tty.c_cc[VTIME] = 10;  // 1 Sekunde Timeout (10 x 100ms)

    tcsetattr(serial_port, TCSANOW, &tty);
    return serial_port;
}

std::string readLine(int fd, int timeout_ms) {
    std::string result="";
    char ch;
    auto start = std::chrono::steady_clock::now();

    while (true) {
        int n = read(fd, &ch, 1);
        if (n == 1) {
            if (ch == '\n') break;
            result += ch;
        }

        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count() > timeout_ms) {
            std::cerr << "⚠️ Timeout beim Lesen der Zeile\n";
            break;
        }
    }

    return result;
}
