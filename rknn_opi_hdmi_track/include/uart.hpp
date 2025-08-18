#ifndef UART_HPP
#define UART_HPP

#include <string>

// Öffnet eine serielle Schnittstelle und konfiguriert sie
// Rückgabewert: Filedescriptor (>= 0) oder -1 bei Fehler
int setupSerial(const char* device);

// Liest eine Zeile von der seriellen Schnittstelle
// fd: Filedescriptor der offenen UART
// timeout_ms: Timeout in Millisekunden (Standard: 1000 ms)
// Rückgabe: Empfangene Zeile (ohne \n)
std::string readLine(int fd, int timeout_ms = 1000);

#endif // UART_HPP