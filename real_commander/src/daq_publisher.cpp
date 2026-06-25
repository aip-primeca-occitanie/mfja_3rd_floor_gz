#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <stdio.h>
#include <iostream>
#include <chrono>
#include <NIDAQmx.h>

#define DAQmxErrChk(functionCall) if( DAQmxFailed(error=(functionCall)) ) goto Error; else

namespace real_commander {

class DaqPublisher : public rclcpp::Node
{
public:
    DaqPublisher()
        : rclcpp::Node("daq_publisher_node")
        {
        // Create a ROS publisher
        fz_pub = this->create_publisher<std_msgs::msg::Float64>("/Fz_raw", 100);
        mz_pub = this->create_publisher<std_msgs::msg::Float64>("/Mz_raw", 100);

            // Create a rate
        this->declare_parameter<int>("FREQ", 250); // Hz
        int freq = this->get_parameter("FREQ").as_int(); //reading too slowly might make the node crash

        // Le bloc DAQmx (avec son goto Error) est isolé dans initDaq() : il ne contient
        // que des variables C/POD, donc le goto ne traverse jamais l'initialisation
        // d'un objet C++ non trivial (ce qui causait le warning sur "period").
        if (!initDaq())
            {
            return; // erreur déjà loggée dans initDaq(), tâche nettoyée, pas de timer créé
            }

        RCLCPP_INFO(this->get_logger(), "Acquisition DAQ démarrée, publication à %d Hz.", freq);

        // Remplace l'ancienne boucle while(rclcpp::ok()){...; rate.sleep();} par un timer ROS,
        // pour pouvoir laisser main() faire un simple rclcpp::spin(node).
        auto period = std::chrono::duration<double>(1.0 / static_cast<double>(freq));
        timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&DaqPublisher::readAndPublish, this));
            }

    ~DaqPublisher() { // équivalent du nettoyage qui se faisait par "fall-through" au label Error dans l'ancien main(), une fois la boucle terminée.
        if( taskHandle_!=0 )  {
            DAQmxStopTask(taskHandle_);
            DAQmxClearTask(taskHandle_);
            }
        }

private:
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr fz_pub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr mz_pub;
    rclcpp::TimerBase::SharedPtr timer_;
    TaskHandle taskHandle_{0};

    // Contient tout le paramétrage DAQmx d'origine, avec le goto Error tel quel.
    // Cette méthode ne déclare QUE des variables C/POD (int32, TaskHandle, char[]),
    // donc le goto ne traverse jamais l'initialisation d'un objet C++ non trivial.
    bool initDaq() {
        /*--------------------------
        Paramétrage DAQmx :
        ----------------------------*/
        int32       error=0;
        TaskHandle  taskHandle=0;
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

        taskHandle_ = taskHandle; // on garde le handle pour le timer de lecture et le destructeur
        return true;

        Error:
        if( DAQmxFailed(error)) {
        DAQmxGetExtendedErrorInfo(errBuff,2048);
        RCLCPP_ERROR(this->get_logger(), "DAQmx Error: %s", errBuff);
            }
        if( taskHandle!=0 )  {
        DAQmxStopTask(taskHandle);
        DAQmxClearTask(taskHandle);
            }
        return false;
        }

    void readAndPublish() {
        int32       error=0;
        int32       read;
        float64     data[2];//Pour de la commande en temps réel, on va privilégier de faire des petits paquets de données qu'on envoit rapidement avec ROS
        char        errBuff[2048]={'\0'};

        //The DAQmxReadAnalogF64 reads multiple floating-point samples from a task that contains one or more analog input channels as shown in the function call.
        DAQmxErrChk(DAQmxReadAnalogF64(taskHandle_, 1, 10.0, DAQmx_Val_GroupByChannel, data, 2, &read, NULL)); //1 sample per chain, 10s timeout (standard), array size = 2 car deux channels
        // fonction : int32 DAQmxReadAnalogF64 (TaskHandle taskHandle, int32 numSampsPerChan, float64 timeout, bool32 fillMode, float64 readArray[], uInt32 arraySizeInSamps, int32 *sampsPerChanRead, bool32 *reserved);
        // La création/publication des messages (objets C++ non triviaux) est déportée dans
        // publishData(), pour ne pas être traversée par le goto Error ci-dessous.
        publishData(data);
        return;

        Error:
        if( DAQmxFailed(error)) {
            DAQmxGetExtendedErrorInfo(errBuff,2048);
            RCLCPP_ERROR(this->get_logger(), "DAQmx Error: %s", errBuff);
                        // on arrête le timer pour ne pas spammer l'erreur en boucle, et on libère la tâche
            timer_->cancel();
            if( taskHandle_!=0 ) {
                DAQmxStopTask(taskHandle_);
                DAQmxClearTask(taskHandle_);
                taskHandle_ = 0;
                }
            }
        }

    void publishData(const float64 data[2]) {
        std_msgs::msg::Float64 fz_msg, mz_msg;
        //vérification sur les paramètres de config sur WITIS, Fz correspond à AI7 et Mz à AI6
        fz_msg.data = data[0] * 20.864489; //N = k * V, k was computed with samplings
        mz_msg.data = data[1]; //coeff to be determined
        fz_pub->publish(fz_msg);
        mz_pub->publish(mz_msg);
        }
    };

} // namespace real_commander

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<real_commander::DaqPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}