// Standalone utility: list the serial numbers of all connected USB2CANFD
// devices so they can be filled into the dev_sn field of the robot configs.
#include <cstdio>
#include <iostream>
#include <libusb-1.0/libusb.h>

#define USB2CANFD_VID 0x34B7
#define USB2CANFD_PID 0x6877

int main()
{
    libusb_context* context = nullptr;
    int result = libusb_init(&context);
    if (result < 0) {
        std::cerr << "Failed to initialize libusb: " << libusb_error_name(result) << std::endl;
        return 1;
    }

    libusb_device** devices;
    ssize_t count = libusb_get_device_list(context, &devices);
    if (count < 0) {
        std::cerr << "Failed to obtain device list: " << libusb_error_name(count) << std::endl;
        libusb_exit(context);
        return 1;
    }

    int found = 0;
    for (int i = 0; devices[i]; i++) {
        libusb_device* device = devices[i];

        libusb_device_descriptor desc;
        result = libusb_get_device_descriptor(device, &desc);
        if (result < 0) {
            std::cerr << "Failed to obtain device descriptor: " << libusb_error_name(result) << std::endl;
            continue;
        }

        if (desc.idVendor != USB2CANFD_VID || desc.idProduct != USB2CANFD_PID) {
            continue;
        }

        libusb_device_handle* handle = nullptr;
        result = libusb_open(device, &handle);
        if (result != LIBUSB_SUCCESS) {
            std::cerr << "Found USB2CANFD device but failed to open it: "
                      << libusb_error_name(result) << " (try sudo)" << std::endl;
            continue;
        }

        char serial_number[256] = {0};
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
            }
        }

        std::cout << "USB2CANFD device " << found << ":" << std::endl;
        std::cout << "  bus " << static_cast<int>(libusb_get_bus_number(device))
                  << ", port " << static_cast<int>(libusb_get_port_number(device)) << std::endl;
        std::cout << "  SN: " << (serial_number[0] ? serial_number : "[No serial number]") << std::endl;
        std::cout << std::endl;
        found++;

        libusb_close(handle);
    }

    if (found == 0) {
        std::cout << "No USB2CANFD device found (VID 0x34B7, PID 0x6877)." << std::endl;
    } else {
        std::cout << found << " device(s) found. Copy each SN into the dev_sn field "
                  << "of the matching robot config yaml." << std::endl;
    }

    libusb_free_device_list(devices, 1);
    libusb_exit(context);
    return 0;
}
