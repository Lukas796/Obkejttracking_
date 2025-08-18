#include <iostream>
#include <fstream>
#include <string>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <chrono>
#include <cstdlib>  // für rand()
#include <csignal>


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


int main() {
    const char* device = "/dev/ttyACM0";
    int serial_port = setupSerial(device);
    if (serial_port < 0) return 1;

    std::cout << "✅ UART geöffnet: " << device << std::endl;

    while (true) {
        int x = 100 + rand() % 300;
        int y = 50 + rand() % 200;
        std::string posString = "X:" + std::to_string(x) + ",Y:" + std::to_string(y) + "\n";
         // Startzeitpunkt erfassen
        auto start = std::chrono::steady_clock::now();
        write(serial_port, posString.c_str(), posString.length());
        // Startzeitpunkt 2erfassen
        auto start2 = std::chrono::steady_clock::now();
        std::cout << "📤 [Auto] Gesendet: " << posString;

        // Antwort (falls vorhanden)
        char buffer[256] = {0};
        int n = read(serial_port, buffer, sizeof(buffer) - 1);
          // Endzeitpunkt erfassen
        auto end = std::chrono::steady_clock::now();
        // Dauer berechnen (in Millisekunden)
         auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
        std::cout << "Dauer: " << duration << " us" << std::endl;

        auto duration2 = std::chrono::duration_cast<std::chrono::microseconds>(end - start2).count();
        std::cout << "Dauer2: " << duration2 << " us" << std::endl;
        if (n > 0) {
            buffer[n] = '\0';
            std::cout << "📥 Antwort: " << buffer << std::endl;
        }

    }
    
    close(serial_port);
    std::cout << "\n🚪 UART Verbindung geschlossen. Programm beendet.\n";
    return 0;
}