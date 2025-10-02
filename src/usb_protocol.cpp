/**
 *@file usb_protocol.cpp
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2025-10-02
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "usb_protocol.hpp"
#include <vector>

bool UsbProtocol::open(const std::string& port) {
    char errorOpening = serial_.openDevice(port.c_str(), 115200);
    if (errorOpening != 1) {
        is_open_ = false;
    } else {
        is_open_ = true;
    }
    return is_open_;
}

bool UsbProtocol::is_open() {
    return serial_.isDeviceOpen() && is_open_;
}

int UsbProtocol::write_bytes(void* buffer, const unsigned int n_bytes) {
    unsigned num_bytes = 0;
    int ret = serial_.writeBytes(buffer, n_bytes, &num_bytes);
    if (ret == 1) {
        is_open_ = true;
        return (int) num_bytes;
    } else {
        is_open_ = false;
        return 0;
    }
}

int UsbProtocol::read_bytes(void* buffer, int max_length, std::chrono::milliseconds timeout) {
    int ret = serial_.readBytes(buffer, max_length, timeout.count());
    if (ret >= 0) {
        is_open_ = true;
        return ret;
    } else {
        is_open_ = false;
        return 0;
    }
}

int UsbProtocol::write_string(const std::string& msg) {
    int ret = serial_.writeString(msg.c_str());
    return ret;
}

std::string UsbProtocol::read_string(std::chrono::milliseconds timeout) {
    int numChars = serial_.readString(read_buffer_, '\n', 1023, timeout.count());
    if (numChars == 0) {
        return std::string();
    } else if (numChars > 0) {
        return std::string(read_buffer_, numChars);
    } else {
        throw std::runtime_error("USB Read Error");
    }
}

void UsbProtocol::close() {
    serial_.closeDevice();
    is_open_ = false;
}


UsbProtocol::~UsbProtocol() {
    close();
}
