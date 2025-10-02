#ifndef __ICOMMUNICATION_PROTOCOL_H__
#define __ICOMMUNICATION_PROTOCOL_H__

#include <string>
#include <chrono>

class ICommunicationProtocol {
public:
    virtual ~ICommunicationProtocol() = default;

    virtual bool open(const std::string& dev) = 0;

    virtual int write_bytes(void* buffer, const unsigned int n_bytes) = 0;

    virtual int read_bytes(void* buffer, int max_length, std::chrono::milliseconds timeout) = 0;

    virtual int write_string(const std::string& msg) = 0;

    virtual std::string read_string(std::chrono::milliseconds timeout) = 0;
};

#endif // __ICOMMUNICATION_PROTOCOL_H__