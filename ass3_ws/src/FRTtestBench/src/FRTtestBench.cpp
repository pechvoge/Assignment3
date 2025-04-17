#include "FRTtestBench.hpp"

FRTtestBench::FRTtestBench(uint write_decimator_freq, uint monitor_freq) :
    XenoFrt20Sim(write_decimator_freq, monitor_freq, file, &data_to_be_logged),
    file(1,"./xrf2_logging/TEMPLATE","bin"), // change template to your project name
    controller()
{
     printf("%s: Constructing rampio\n", __FUNCTION__);
    
    // To infinite run the controller, uncomment line below
    controller.SetFinishTime(0.0);
}

FRTtestBench::~FRTtestBench()
{
    
}

int FRTtestBench::initialising()
{
    // Set physical and cyber system up for use in a 
    // Return 1 to go to initialised state

    evl_printf("Hello from initialising\n");      // Do something

    // The logger has to be initialised at only once
    logger.initialise();
    // The FPGA has to be initialised at least once
    ico_io.init();

    return 1;
}

int FRTtestBench::initialised()
{
    // Keep the physical syste in a state to be used in the run state
    // Call start() or return 1 to go to run state

    evl_printf("Hello from initialised\n");       // Do something

    return 1;
}

int FRTtestBench::run()
{
    // Do what you need to do
    // Return 1 to go to stopping state

    // Start logger
    logger.start();                             
    monitor.printf("Hello from run\n");  

    // Printf encoder 1 to 4 data
    monitor.printf("Encoder 1 value : %d\n",sample_data.channel1);
    monitor.printf("Encoder 2 value : %d\n",sample_data.channel2);

    // Get the current encoder values
    int current_encoder_left = sample_data.channel1;
    int current_encoder_right = sample_data.channel2;

    // Set the old encoder values to the current encoder values if it is the first time
    if (first_time)
    {
        old_encoder_left = current_encoder_left;
        old_encoder_right = current_encoder_right;
        first_time = false;
    }

    // Calculate the difference between the old and current encoder values
    int difference_left = old_encoder_left - current_encoder_left;
    int difference_right = old_encoder_right - current_encoder_right;

    // Set the current encoder values to the old encoder values for next iteration
    old_encoder_left = current_encoder_left;
    old_encoder_right = current_encoder_right;
 

    // Update wrap counters according to the difference between the old and current encoder values
    if(difference_left > encoder_max / 2) 
    {
        wrap_counter_left++;
    }
    else if(difference_left < -encoder_max / 2)
    {
        wrap_counter_left--;
    }
    if(difference_right > encoder_max / 2)
    {
        wrap_counter_right++;
    }
    else if(difference_right < - encoder_max / 2)
    {
        wrap_counter_right--;
    }

   
    // Compute the unwrapped encoder values and print them
    int unwrapped_encoder_left = wrap_counter_left*(encoder_max + 1) + sample_data.channel1;
    int unwrapped_encoder_right = wrap_counter_right*(encoder_max + 1) + sample_data.channel2;
    monitor.printf("Unwrapped Encoder 1 value : %d\n",unwrapped_encoder_left);
    monitor.printf("Unwrapped Encoder 2 value : %d\n",unwrapped_encoder_right);


    // Set motor outputs to setpoint velocities
    actuate_data.pwm1 = 2047.0 * ros_msg.left_motor_setpoint_vel;
    actuate_data.pwm2 = -2047.0 * ros_msg.right_motor_setpoint_vel;

    controller.Calculate(u, y);
    if(controller.IsFinished())
        return 1;


    return 0;
}

int FRTtestBench::stopping()
{
    // Bring the physical system to a stop and set it in a state that the system can be deactivated
    // Return 1 to go to stopped state
    logger.stop();                                // Stop logger
    evl_printf("Hello from stopping\n");          // Do something

    return 1;
}

int FRTtestBench::stopped()
{
    // A steady state in which the system can be deactivated whitout harming the physical system

    monitor.printf("Hello from stopping\n");          // Do something

    return 0;
}

int FRTtestBench::pausing()
{
    // Bring the physical system to a stop as fast as possible without causing harm to the physical system

    evl_printf("Hello from pausing\n");           // Do something
    return 1 ;
}

int FRTtestBench::paused()
{
    // Keep the physical system in the current physical state

    monitor.printf("Hello from paused\n");            // Do something
    return 0;
}

int FRTtestBench::error()
{
    // Error detected in the system 
    // Can go to error if the previous state returns 1 from every other state function but initialising 

    monitor.printf("Hello from error\n");             // Do something

    return 0;
}
