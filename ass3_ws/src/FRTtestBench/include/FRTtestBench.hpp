#ifndef FRTTESTBENCH_HPP
#define FRTTESTBENCH_HPP

#include "XenoFrt20Sim.hpp"
#include "LoopController.h"



class FRTtestBench : public XenoFrt20Sim
{
public:
    FRTtestBench(uint write_decimator_freq, uint monitor_freq);
    ~FRTtestBench();
private:
    XenoFileHandler file;
    LoopController controller;

    double u[0+1];
    double y[0+1];
protected:
    //Functions
    int initialising() override;
    int initialised() override;
    int run() override;
    int stopping() override;
    int stopped() override;
    int pausing() override;
    int paused() override;
    int error() override;

    // current error
    int current_error = 0;
};

#endif // FRTTESTBENCH_HPP