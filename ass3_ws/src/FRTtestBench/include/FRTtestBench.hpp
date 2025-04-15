#ifndef FRTTESTBENCH_HPP
#define FRTTESTBENCH_HPP

#include "XenoFrt20Sim.hpp"
#include "LoopController.h"

#pragma pack (1)    //https://carlosvin.github.io/langs/en/posts/cpp-pragma-pack/#_performance_test
struct ThisIsAStruct
{
    int this_is_a_int = 0;
    double this_is_a_double = 100.0;
    float this_is_a_float = 10.0;
    char this_is_a_char = 'R';
    bool this_is_a_bool = false;
};

#pragma pack(0)

class FRTtestBench : public XenoFrt20Sim
{
public:
    FRTtestBench(uint write_decimator_freq, uint monitor_freq);
    ~FRTtestBench();
private:
    XenoFileHandler file;
    struct ThisIsAStruct data_to_be_logged;
    LoopController controller;

    double u[4+1];
    double y[2+1];

    int old_encoder_left;
    int old_encoder_right;
    const int encoder_max = 16383;
    int wrap_counter_left = 0;
    int wrap_counter_right = 0;
    const float d_wheel = 0.1;
    const float pi = 3.14159265358979323846;
    const int count_p_turn = 1024;
    const float gear_ratio = 15.58;
    const int quad_counter_ratio = 4;
    bool first_time = true;


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