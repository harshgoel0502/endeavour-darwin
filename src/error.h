#include <exception>
#include <iostream>
#include <string>

class ErrorState : public std::exception {
    private:
        std::string message;
    
    public:
        ErrorState(const char* msg) : message(msg)
        {
        }

        const char* what() const throw(){
            return message.c_str();
        }
};

#ifndef ERROR_H
#define ERROR_H

enum Error{
    NO_ERROR,
    UNDEFINED,
    IMU_ERROR,
    BARO_ERROR,
    GPS_ERROR,
    SD_ERROR,
    FLASH_ERROR,
    RADIO_ERROR
};

#endif