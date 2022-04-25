/**
 * @file system_log.hpp
 * @author bingbingLI (bingbing.li@ntu.edu.sg)
 * @brief  A log class to log the issues during for testings. 
 * @version 0.1
 * @date 2021-03-24
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef COMMON_SYSTEM_LOG_HPP
#define COMMON_SYSTEM_LOG_HPP

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>

#ifdef WIN32
#include <windows.h>
#endif 

#include "tools.h"

namespace mobileman {

class Logger {
public:
  /**
   * @brief Default constructor of logger, the log file is put in to home folder by default
  */
  Logger();
  /**
   * @brief  Alternative constructor of logger 
   * @param  dir directory where the log file is put
  */
  Logger(std::string dir);
  ~Logger();

  /**
   * @brief Log the warring message 
   * @param  warning_code warning code of the robot
   * @return 0 if successful
  */
  inline int log_warning(int);

  /**
   * @brief log the error to a file
   * @param  error_code error code of the robot
   * @return o if successful
  */
  inline int log_error(int);
  /**
   * @brief log message to a file
   * @param  message message to be logged
   * @return 0 if suceessful
  */
  inline int log_message(std::string);

private:
  std::ofstream log_file;
};

Logger::Logger() {
    std::string logfile = "%USERNAME%\\log";
    logfile.append(__DATE__);
    logfile.append(__TIME__);
    logfile.append(".log");
    
#ifdef __linux__
    logfile.assign(tools::getWorkingDirecotry());
    logfile.append("/");
    logfile.append(tools::getCurrentDateTime());
    logfile.append(".log");
#endif // __linux__

   
   log_file.open(logfile, std::ofstream::out | std::ofstream::ate);
  if (log_file.fail()) {
    std::cerr << "Log File cannot be initialized" << std::endl;
    exit(1);
  }
  else {
    std::cout << "log file generated:" << logfile << std::endl;
  }
}
inline Logger::Logger(std::string dir)
{
    log_file.open(dir.append("log.txt"), std::ofstream::out);
    if (log_file.fail()) {
        std::cerr << "Log file cannot be initialized" << std::endl;
    }
  std::cout <<"log file generated" <<std::endl;

}
Logger::~Logger()
{
    log_file.close();
}

int Logger::log_warning(int warning_code) {
  log_file <<"["<<tools::getCurrentDateTime<<"]: " <<"Warning: " << warning_code << std::endl;
  return 0;
}

int Logger::log_error(int error_code) {
    log_file << "[" << tools::getCurrentDateTime << "]: " << "Error: " << error_code << std::endl;
    return 0;
}

int Logger::log_message(std::string message) {
    log_file << "[" << tools::getCurrentDateTime << "]: " << "Message: " << message << std::endl;
    return 0;
}

} // namespace mobileman

#endif