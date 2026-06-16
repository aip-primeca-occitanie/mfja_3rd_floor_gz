#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <stdio.h>
#include <NIDAQmx.h>
#define DAQmxErrChk(functionCall) if( DAQmxFailed(error=(functionCall)) ) goto Error; else

int main(int argc, char **argv) //argv and argc are how command line arguments are passed to main() in C and C++.
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("daq_publisher_node");

    // Create a ROS publisher
    auto fz_pub = node->create_publisher<std_msgs::msg::Float64>("/Fz", 100);
    auto mz_pub = node->create_publisher<std_msgs::msg::Float64>("/Mz", 100);

    // Create a rate
    //this->declare_parameter<int>("FREQ", 250); // Hz
    //int freq = this->get_parameter("FREQ").as_int(); //convert to class
    rclcpp::Rate rate(250);//reading too slowly might make the node crash

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

    char buffer[1024];
    DAQmxGetSysDevNames(buffer, 1024);
    std::cout << "Devices: " << buffer << std::endl;

    //This function then configured a virtual voltage channel:
    DAQmxErrChk(DAQmxCreateAIVoltageChan(taskHandle, "cDAQ1Mod1/ai7", "", DAQmx_Val_Cfg_Default, -10.0, 10.0, DAQmx_Val_Volts, NULL));

    DAQmxErrChk(DAQmxCreateAIVoltageChan(taskHandle, "cDAQ1Mod1/ai6", "", DAQmx_Val_Cfg_Default, -10.0, 10.0, DAQmx_Val_Volts, NULL));

    //After configuring the virtual voltage channels, a sample clock setting function specified the sampling rate, sample mode, and number of samples to read:
    DAQmxErrChk(DAQmxCfgSampClkTiming(taskHandle, "", 100.0, DAQmx_Val_Rising, DAQmx_Val_ContSamps, 10));

    // DAQmx Start Code
    DAQmxErrChk(DAQmxStartTask(taskHandle));
    //-------------------------------------

    RCLCPP_INFO(node->get_logger(), "Acquisition DAQ démarrée, publication à 10 Hz.");

    while (rclcpp::ok())
    {
        //The DAQmxReadAnalogF64 reads multiple floating-point samples from a task that contains one or more analog input channels as shown in the function call.
        DAQmxErrChk(DAQmxReadAnalogF64(taskHandle, 1, 10.0, DAQmx_Val_GroupByChannel, data, 2, &read, NULL)); //1 sample per chain, 10s timeout (standard), array size = 2 car deux channels
        // fonction : int32 DAQmxReadAnalogF64 (TaskHandle taskHandle, int32 numSampsPerChan, float64 timeout, bool32 fillMode, float64 readArray[], uInt32 arraySizeInSamps, int32 *sampsPerChanRead, bool32 *reserved);

        std_msgs::msg::Float64 fz_msg, mz_msg;
        //vérification sur les paramètres de config sur WITIS, Fz correspond à AI7 et Mz à AI6
        fz_msg.data = data[0] * 20.864489; //N = k * V, k was computed with samplings
        mz_msg.data = data[1]; //coeff to be determined

        fz_pub->publish(fz_msg);
        mz_pub->publish(mz_msg);

        rclcpp::spin_some(node); //Create a default single-threaded executor and execute any immediately available work. 
        rate.sleep();
    }
    
    Error:
       if( DAQmxFailed(error)) {
        DAQmxGetExtendedErrorInfo(errBuff,2048);
        RCLCPP_ERROR(node->get_logger(), "DAQmx Error: %s", errBuff);
       }
             
       if( taskHandle!=0 )  {
              DAQmxStopTask(taskHandle);
              DAQmxClearTask(taskHandle);
       }

        return 0;
}
    