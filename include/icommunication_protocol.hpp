#ifndef __ICOMMUNICATION_PROTOCOL_H__
#define __ICOMMUNICATION_PROTOCOL_H__

#include <string>
#include <chrono>
#include <communication_packets.h>

class ICommunicationProtocol {
public:
    
    virtual bool open(const std::string& dev) = 0;

    virtual bool is_open() = 0;

    virtual int write_bytes(void* buffer, const unsigned int n_bytes) = 0;

    virtual int read_bytes(void* buffer, int max_length, std::chrono::milliseconds timeout) = 0;

    virtual bool send_command_packet(MobilityCommandPacket& packet);

    virtual bool send_init_packet(MobilityInitPacket& packet);

    virtual bool receive_motor_feedback_bytes(MobilityFeedbackPacket* feedback, std::chrono::milliseconds timeout);

    virtual void close() = 0;

    virtual ~ICommunicationProtocol() = 0;
};

#endif // __ICOMMUNICATION_PROTOCOL_H__