#ifndef __USB_PROTOCOL_H__
#define __USB_PROTOCOL_H__

#include "icommunication_protocol.hpp"
#include "communication_packets.h"
#include "serialib.h"

class UsbProtocol : public ICommunicationProtocol {
public:
    UsbProtocol() = default;

    bool open(const std::string& port) override;

    bool is_open() override;

    int write_bytes(void* buffer, const unsigned int n_bytes) override;

    int read_bytes(void* buffer, int max_length, std::chrono::milliseconds timeout) override;

    int write_string(const std::string& msg) override;

    std::string read_string(std::chrono::milliseconds timeout) override;

    void close() override;

    ~UsbProtocol() override;

private:
    serialib serial_ {};
    char read_buffer_[1024] {};
    bool is_open_ { false };
};



#endif // __USB_PROTOCOL_H__