#include <iostream>
#include <fstream>
#include <string>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <thread>
#include <chrono>
#include <atomic>
#include <cstdlib>  // für rand()
#include <csignal>

// Globale Variable, um Loop zu stoppen
std::atomic<bool> running(true);

// Signal-Handler für sauberen Abbruch mit Ctrl+C
void signalHandler(int signum) {
    running = false;
}

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

    cfsetispeed(&tty, B9600);
    cfsetospeed(&tty, B9600);

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

// Thread: automatische Positionssende-Schleife
void sendPositions(int serial_port) {
    while (running) {
        int x = 100 + rand() % 300;
        int y = 50 + rand() % 200;
        std::string posString = "X:" + std::to_string(x) + ",Y:" + std::to_string(y) + "\n";

        write(serial_port, posString.c_str(), posString.length());
        std::cout << "📤 [Auto] Gesendet: " << posString;

        // Antwort (falls vorhanden)
        char buffer[256] = {0};
        int n = read(serial_port, buffer, sizeof(buffer) - 1);
        if (n > 0) {
            buffer[n] = '\0';
            std::cout << "📥 Antwort: " << buffer << std::endl;
        }

        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

int main() {
    signal(SIGINT, signalHandler);  // Ctrl+C abfangen

    const char* device = "/dev/ttyACM0";
    int serial_port = setupSerial(device);
    if (serial_port < 0) return 1;

    std::cout << "✅ UART geöffnet: " << device << std::endl;
    std::cout << "✏️  Du kannst jederzeit manuell Befehle eingeben (z.B. 'LED ON'):\n\n";

    // Starte Positions-Thread
    std::thread sender(sendPositions, serial_port);

    // Haupteingabe: manuelle Eingaben wie "LED ON"
    std::string input;
    while (running) {
        std::getline(std::cin, input);
        if (!running) break;
        if (input.empty()) continue;

        input += "\n";
        write(serial_port, input.c_str(), input.length());

        char buffer[256] = {0};
        int n = read(serial_port, buffer, sizeof(buffer) - 1);
        if (n > 0) {
            buffer[n] = '\0';
            std::cout << "📥 Antwort: " << buffer << std::endl;
        }
    }

    running = false;
    if (sender.joinable()) sender.join();
    close(serial_port);
    std::cout << "\n🚪 UART Verbindung geschlossen. Programm beendet.\n";
    return 0;
}