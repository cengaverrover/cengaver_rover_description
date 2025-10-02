/**
 *@file icommunication_protocol.cpp
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2025-10-02
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "icommunication_protocol.hpp"
#include <vector>

ICommunicationProtocol::~ICommunicationProtocol() {}

bool ICommunicationProtocol::send_command_packet(MobilityCommandPacket& packet) {
    packet.frame_start = FRAME_START;
    packet.frame_end = FRAME_END;
    int ret = write_bytes((void*) &packet, sizeof(packet));
    //RCLCPP_INFO(this->get_logger(), "Sent bytes: %d/%zu", ret, sizeof(packet));
    if (ret == sizeof(packet)) {
        return true;
    } else {
        return false;
    }
}

bool ICommunicationProtocol::send_init_packet(MobilityInitPacket& packet) {
    packet.frame_start = FRAME_START;
    packet.frame_end = FRAME_END;
    int ret = write_bytes((void*) &packet, sizeof(packet));
    //RCLCPP_INFO(this->get_logger(), "Sent bytes: %d/%zu", ret, sizeof(packet));
    if (ret == sizeof(packet)) {
        return true;
    } else {
        return false;
    }
}

bool ICommunicationProtocol::receive_motor_feedback_bytes(MobilityFeedbackPacket* feedback) {
    if (!feedback) return false;

    static std::vector<uint8_t> buffer;
    buffer.reserve(2 * sizeof(MobilityFeedbackPacket));

    uint8_t temp[64]; // read in small chunks
    int bytes_read = read_bytes(temp, sizeof(temp), std::chrono::milliseconds(1));

    if (bytes_read <= 0) return false;

    // Append new data to sliding buffer
    buffer.insert(buffer.end(), temp, temp + bytes_read);

    while (buffer.size() >= sizeof(MobilityFeedbackPacket)) {
        // Check for FRAME_START at current position
        uint32_t frame_start = buffer[0] | (buffer[1] << 8) | (buffer[2] << 16) | (buffer[3] << 24);
        if (frame_start == FRAME_START) {
            // Possible start of packet, check if we have enough bytes
            if (buffer.size() < sizeof(MobilityFeedbackPacket)) break;

            MobilityFeedbackPacket* pkt = reinterpret_cast<MobilityFeedbackPacket*>(buffer.data());
            if (pkt->frame_end == FRAME_END) {
                // Valid packet, copy to feedback
                *feedback = *pkt;
                // Remove consumed bytes
                buffer.erase(buffer.begin(), buffer.begin() + sizeof(MobilityFeedbackPacket));
                return true;
            } else {
                // FRAME_END mismatch, discard first byte and resync
                buffer.erase(buffer.begin());
            }
        } else {
            // FRAME_START not found, discard first byte
            buffer.erase(buffer.begin());
        }
    }
    return false;
}