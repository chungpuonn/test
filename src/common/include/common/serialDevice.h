#pragma once
#ifndef MOBILEMAN_SERRIALDEVICE_HPP
#define MOBILEMAN_SERRIALDEVICE_HPP
#include <unistd.h>
#include <stdlib.h>
#include <iostream>
#include <time.h>
#include <unistd.h>
#include <string.h>

#include <SerialStream.h>
#include <SerialPort.h>


#include "../../mobileman_navigation/include/mobileman_navigation/system_log.hpp"
using namespace LibSerial;
namespace mobileman
{
    class SerialDevice
    // \brief Provide interface to send and recive data from arduino
    {
        public:
        SerialDevice(const std::string , int);
        ~SerialDevice();
        std::string sendData(std::string);
        std::string readData();

        private:
        
        std::string read_data;
        LibSerial::SerialStream device;

    };
    SerialDevice::SerialDevice(const std::string port_name, int baurate  = 9600)
    {
        device.Open(port_name);
        device.SetVTime(1);
        device.SetVMin(100);
        device.SetCharSize(SerialStreamBuf::CharSizeEnum::CHAR_SIZE_8);
        // device.SetBaudRate(LibSerial::BaudRate::BAUD_9600);
        device.SetBaudRate(SerialStreamBuf::BAUD_9600) ;      

        do
        {
            /* code */
            usleep(10000);  //sleep for 10ms if the the device port is not open
            std::cout<<"device is not open" <<std::endl;
        } 
        while (!device.IsOpen());
        sleep(3); //wait for device port to be initialized
        // moibleman::Logger::log_message("Serial Port Open\n");

 
    }
    SerialDevice::~SerialDevice()
    {
        device.Close();
    }
    std::string SerialDevice::sendData(std::string data )
    {
        
            device << data;

            usleep(5000); //wait 1 milisecond for responese
        
            return readData();

    

    }
    std::string SerialDevice::readData()
    {
        char response[200];
        try
        {
            memset(response,0, sizeof(response));
            device.read(response,100);
        }
        // catch(const LibSerial::ReadTimeout )
        catch (const SerialPort::ReadTimeout)
        {
            std::cerr << "The Read() call has timed out." << std::endl ;;
        }
        
        return std::string(response);
    }
}
#endif