#include <iostream>
#include <fstream>
#include <string>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

int main() {
    const char* device = "/dev/ttyACM1";  // Anpassen, falls bei dir anders
    int serial_port = open(device, O_RDWR | O_NOCTTY);

    if (serial_port < 0) {
        std::cerr << "Fehler beim Öffnen von " << device << std::endl;
        return 1;
    }

    termios tty{};
    if (tcgetattr(serial_port, &tty) != 0) {
        std::cerr << "Fehler beim Abrufen der TTY-Eigenschaften\n";
        return 1;
    }

    cfsetispeed(&tty, B9600);
    cfsetospeed(&tty, B9600);

    tty.c_cflag |= (CLOCAL | CREAD);    // Empfangen erlauben
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;                 // 8 Datenbits
    tty.c_cflag &= ~PARENB;             // Keine Parität
    tty.c_cflag &= ~CSTOPB;             // 1 Stopbit
    tty.c_cflag &= ~CRTSCTS;            // Kein Hardware-Flowcontrol
    tty.c_lflag = 0;                    // Kein Canonical Mode
    tty.c_oflag = 0;
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 0;

    tcsetattr(serial_port, TCSANOW, &tty);

    std::string command;
    std::cout << "Befehl an Arduino senden (z. B. LED ON): ";
    std::getline(std::cin, command);
    command += "\n";  // Wichtig für readStringUntil('\n') auf Arduino

    write(serial_port, command.c_str(), command.length());

    // Antwort lesen
    char buffer[256];
    int n = read(serial_port, buffer, sizeof(buffer) - 1);
    buffer[n] = '\0';
    std::cout << "Antwort vom Arduino: " << buffer << std::endl;

    close(serial_port);
    return 0;
}