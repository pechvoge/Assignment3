#include "FRTtestBench.hpp"

FRTtestBench::FRTtestBench(uint write_decimator_freq, uint monitor_freq) :
    XenoFrt20Sim(write_decimator_freq, monitor_freq, file, &data_to_be_logged),
    file(1,"./xrf2_logging/TEMPLATE","bin"), // change template to your project name
    controller()
{
     printf("%s: Constructing rampio\n", __FUNCTION__);
    // Add variables to logger to be logged, has to be done before you can log data
    // logger.addVariable("this_is_a_int", integer);
    // logger.addVariable("this_is_a_double", double_);
    // logger.addVariable("this_is_a_float", float_);
    // logger.addVariable("this_is_a_char", character);
    // logger.addVariable("this_is_a_bool", boolean);
    
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
    // logger.initialise();
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
    // logger.start();                               
    //  Change some data for logger            
    // data_to_be_logged.this_is_a_bool = !data_to_be_logged.this_is_a_bool;
    // data_to_be_logged.this_is_a_int++;
    // if(data_to_be_logged.this_is_a_char == 'R')
    //     data_to_be_logged.this_is_a_char = 'A';
    // else if (data_to_be_logged.this_is_a_char == 'A')
    //     data_to_be_logged.this_is_a_char = 'M';
    // else
    //     data_to_be_logged.this_is_a_char = 'R';
    // data_to_be_logged.this_is_a_float = data_to_be_logged.this_is_a_float/2;
    // data_to_be_logged.this_is_a_double = data_to_be_logged.this_is_a_double/4; 

    // Printf encoder 1 to 4 data
    // monitor.printf("Encoder 1 value : %d\n",sample_data.channel1);
    // monitor.printf("Encoder 2 value : %d\n",sample_data.channel2);
    // monitor.printf("Encoder 3 value : %d\n",sample_data.channel3);
    // monitor.printf("Encoder 4 value : %d\n",sample_data.channel4);

    int current_encoder_left = sample_data.channel1;
    int current_encoder_right = sample_data.channel2;
    
    if (first_time)
    {
        old_encoder_left = current_encoder_left;
        old_encoder_right = current_encoder_right;
        first_time = false;
        monitor.printf("Hello from run\n");
    }

    int difference_left = old_encoder_left - current_encoder_left;
    int difference_right = old_encoder_right - current_encoder_right;
    old_encoder_left = current_encoder_left;
    old_encoder_right = current_encoder_right;

    if(difference_left > encoder_max/2) 
    {
        wrap_counter_left++;
    }
    else if(difference_left < -encoder_max/2)
    {
        wrap_counter_left--;
    }
    if(difference_right > encoder_max/2) 
    {
        wrap_counter_right++;
    }
    else if(difference_right < -encoder_max/2)
    {
        wrap_counter_right--;
    }
    // monitor.printf("Wrap counter left : %d\n",wrap_counter_left);
    // monitor.printf("Wrap counter right : %d\n",wrap_counter_right);
    
    int unwrapped_encoder_left = wrap_counter_left*(encoder_max + 1) + sample_data.channel1;
    int unwrapped_encoder_right = wrap_counter_right*(encoder_max + 1) + sample_data.channel2;
    // monitor.printf("Unwrapped Encoder 1 value : %d\n",unwrapped_encoder_left);
    // monitor.printf("Unwrapped Encoder 2 value : %d\n",unwrapped_encoder_right);

    // Set motor outputs to setpoint velocities
    u[0] = unwrapped_encoder_left*pi*d_wheel/(count_p_turn*gear_ratio*quad_counter_ratio);		/* PosLeft (in m) */
	u[1] = -unwrapped_encoder_right*pi*d_wheel/(count_p_turn*gear_ratio*quad_counter_ratio);		/* PosRight (in m)*/
	u[2] = -ros_msg.left_motor_setpoint_vel;		/* SetVelLeft (in m/s)*/
	u[3] = -ros_msg.right_motor_setpoint_vel;		/* SetVelRight (in m/s) */
    monitor.printf("PosLeft : %f\n",u[0]);
    monitor.printf("PosRight : %f\n",u[1]);
    monitor.printf("SetVelLeft : %f\n",u[2]);
    monitor.printf("SetVelRight : %f\n",u[3]);

    // Calculate the control output
    controller.Calculate(u, y);
    monitor.printf("Controller output : %f\n",y[0]);
    monitor.printf("Controller output : %f\n",y[1]);

    // Saturating the controller output to the range [-1, 1]
    if (y[0] > 100.0){
        y[0] = 100.0;
    } else if (y[0] < -100.0){   
        y[0] = -100.0;
    }
    if (y[1] > 100.0){
        y[1] = 100.0;
    } else if (y[1] < -100.0){   
        y[1] = -100.0;
    }


    // Set motor outputs to setpoint velocities
    actuate_data.pwm1 = 2047.0 * y[0]/100.0; // left motor
    actuate_data.pwm2 = -2047.0 * y[1]/100.0; // right motor (minus sign to rotate in positive direction)
    if(controller.IsFinished())
        return 1;


    return 0;
}

int FRTtestBench::stopping()
{
    // Bring the physical system to a stop and set it in a state that the system can be deactivated
    // Return 1 to go to stopped state
    // logger.stop();                                // Stop logger
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
