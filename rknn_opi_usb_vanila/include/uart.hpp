#ifndef UART_HPP
#define UART_HPP

#include <string>

class UART {
public:
    UART(const std::string& device, int baudrate);
    ~UART();

    bool isOpen() const;
    void send(const std::string& message);

private:
    int serial_fd;
    bool connected;
};

#endif