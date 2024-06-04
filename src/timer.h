#ifndef _TIMER_H_
#define _TIMER_H_
#include <chrono>
#include <iostream>

namespace timer {
void tic();
double toc(bool flag_verbose);
const std::string currentDateTime();
};  // namespace timer
#endif