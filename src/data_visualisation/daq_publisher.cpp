#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <stdio.h>
#include <NIDAQmx.h>
#define DAQmxErrChk(functionCall) if( DAQmxFailed(error=(functionCall)) ) goto Error; else

int main(int argc, char **argv) //argv and argc are how command line arguments are passed to main() in C and C++.
{
    rclcpp::init(argc, argv, "your_sensor_node");
    auto node = rclcpp::Node::make_shared("your_sensor_node");

    // Create a ROS publisher
    auto fz_pub = node->create_publisher<std_msgs::msg::Float64>("/Fz", 10);
    auto mz_pub = node->create_publisher<std_msgs::msg::Float64>("/Mz", 10);

    // Create a rate
    rclcpp::Rate rate(100);//100 Hz

    /*--------------------------
    Paramétrage DAQmx :
    ----------------------------*/
    int32       error=0;
    TaskHandle  taskHandle=0;
    int32       read;
    float64     data[2];//Pour de la commande en temps réel, on va privilégier de faire des petits paquets de données qu'on envoit rapidement avec ROS
    char        errBuff[2048]={'\0'};

    // DAQmx analog voltage channel and timing parameters :
    DAQmxErrChk (DAQmxCreateTask("", &taskHandle));
    //This assigned a name to the task with an output referencing the task created.

    //This function then configured a virtual voltage channel:
    DAQmxErrChk(DAQmxCreateAIVoltageChan(taskHandle, "cDAQ2Mod1/ai6", "", DAQmx_Val_Cfg_Default, -10.0, 10.0, DAQmx_Val_Volts, NULL));

    DAQmxErrChk(DAQmxCreateAIVoltageChan(taskHandle, "cDAQ2Mod1/ai7", "", DAQmx_Val_Cfg_Default, -10.0, 10.0, DAQmx_Val_Volts, NULL));

    //After configuring the virtual voltage channels, a sample clock setting function specified the sampling rate, sample mode, and number of samples to read:
    DAQmxErrChk(DAQmxCfgSampClkTiming(taskHandle, "", 100.0, DAQmx_Val_Rising, DAQmx_Val_ContSamps, 10));

    // DAQmx Start Code
    DAQmxErrChk(DAQmxStartTask(taskHandle));
    //-------------------------------------

    RCLCPP_INFO(node->get_logger(), "Acquisition DAQ démarrée, publication à 100 Hz.");

    while (rclcpp::ok())
    {
        //The DAQmxReadAnalogF64 reads multiple floating-point samples from a task that contains one or more analog input channels as shown in the function call.
        DAQmxErrChk(DAQmxReadAnalogF64(taskHandle, 1, 10.0, DAQmx_Val_GroupByChannel, data, 2, &read, NULL)); //1 sample per chain, 10s timeout (standard), array size = 2 car deux channels
        // fonction : int32 DAQmxReadAnalogF64 (TaskHandle taskHandle, int32 numSampsPerChan, float64 timeout, bool32 fillMode, float64 readArray[], uInt32 arraySizeInSamps, int32 *sampsPerChanRead, bool32 *reserved);

        std_msgs::Float64 fz_msg, mz_msg;
        fz_msg.data = data[0];
        mz_msg.data = data[1];

        fz_pub->publish(fz_msg);
        mz_pub->publish(mz_msg);

        ros::spinOnce() // ros::spinOnce() will call all the callbacks waiting to be called at that point in time.
        rclcpp::spin_some(node); //Create a default single-threaded executor and execute any immediately available work. 
        rate.sleep();
    }

    Error:
       if( DAQmxFailed(error)) {
        DAQmxGetExtendedErrorInfo(errBuff,2048);
        ROS_ERROR("DAQmx Error: %s", errBuff);
       }
             
       if( taskHandle!=0 )  {
              DAQmxStopTask(taskHandle);
              DAQmxClearTask(taskHandle);
       }

        return 0;
}
    