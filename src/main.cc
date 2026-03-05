
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <memory>
#include <mutex>
#include <new>
#include <string>
#include <thread>
#include <csignal>

#include "quadruped_hardware/quadruped_sdk2_bridge.h"
#include <pthread.h>
#include "yaml-cpp/yaml.h"

struct RealRobotConfig
  {
    int domain_id = 1;
    std::string interface = "lo";

  } config;

std::atomic<bool> running(true);

void signalHandler(int signum) {
    running = false;
    std::cerr << "\nInterrupt signal (" << signum << ") received.\n";
}

Journaller* gJournal = 0; // Xsens IMU related

int main(int argc, char **argv)
{
    std::signal(SIGINT, signalHandler);
    // *** Get USB2CANFD device ID *** //
    libusb_context* context = nullptr;
    int result = libusb_init(&context);
    if (result < 0) {
        std::cerr << "Failed to initialize libusb: " << libusb_error_name(result) << std::endl;
        return 1;
    }

    // get device list
    libusb_device** devices;
    ssize_t count = libusb_get_device_list(context, &devices);
    if (count < 0) {
        std::cerr << "Failed to obtain device list: " << libusb_error_name(count) << std::endl;
        libusb_exit(context);
        return 1;
    }

    // get serial number
    char serial_number[256] = {0};
    // search for all
    for (int i = 0; devices[i]; i++) {
        libusb_device* device = devices[i];
        
        libusb_device_descriptor desc;
        result = libusb_get_device_descriptor(device, &desc);
        if (result < 0) {
            std::cerr << "Failed to obtain device descriptor: " << libusb_error_name(result) << std::endl;
            continue;
        }
        
        if (desc.idVendor != 0x34B7 || desc.idProduct != 0x6877) {
            continue;
        }
        
        // open device
        libusb_device_handle* handle = nullptr;
        result = libusb_open(device, &handle);
        if (result != LIBUSB_SUCCESS) {
            std::cerr << "Failed to open device: " << libusb_error_name(result) << std::endl;
            return 0;
        }
        
        if (desc.iSerialNumber > 0) {
            result = libusb_get_string_descriptor_ascii(
                handle, 
                desc.iSerialNumber,
                reinterpret_cast<unsigned char*>(serial_number),
                sizeof(serial_number)
            );
            
            if (result < 0) {
                std::cerr << "Failed to obtain serial number: " << libusb_error_name(result) << std::endl;
                serial_number[0] = '\0';
                return 0;
            }
        }
        
        std::cout << "U2CANFD_DEV " << i << ":" << std::endl;
        std::cout << "  VID: 0x" << std::hex << desc.idVendor << std::endl;
        std::cout << "  PID: 0x" << std::hex << desc.idProduct << std::endl;
        std::cout << "  SN: " << (serial_number[0] ? serial_number : "[No serial number]") << std::endl;
        std::cout << std::endl;
        
        libusb_close(handle);
    }
    
    // clear resource 
    libusb_free_device_list(devices, 1);
    libusb_exit(context);

    // Load parameter
    YAML::Node yaml_node = YAML::LoadFile("../config/config.yaml");
    config.domain_id = yaml_node["domain_id"].as<int>();
    config.interface = yaml_node["interface"].as<std::string>();
    // Main function
    ChannelFactory::Instance()->Init(config.domain_id, config.interface);
    QuadrupedSdk2Bridge quadruped_interface(serial_number);
    
    while (running) 
    {
        sleep(2);
    }
    
    return 0;
}
