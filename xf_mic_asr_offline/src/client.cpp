/*
 * client.cpp
 */
#include "user_interface.h"
#include <string>
#include <locale>
#include <codecvt>
#include <ctime>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <xf_mic_asr_offline/Get_Offline_Result_srv.h>
#include <xf_mic_asr_offline/Set_Major_Mic_srv.h>
#include <xf_mic_asr_offline/Set_Led_On_srv.h>
#include <xf_mic_asr_offline/Get_Major_Mic_srv.h>
#include <xf_mic_asr_offline/Pcm_Msg.h>
#include <xf_mic_asr_offline/Start_Record_srv.h>
#include <xf_mic_asr_offline/Set_Awake_Word_srv.h>
#include <xf_mic_asr_offline/Get_Awake_Angle_srv.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int32.h>
#include <sys/stat.h>
#include <iostream>
#include <std_srvs/Trigger.h>

using namespace std;

int awake_angle = -1;
int flag=0;
int shut_flag=0;

/*void awake_angle_Callback(std_msgs::Int32 msg)
{
    awake_angle = msg.data;
}*/

void cmd_callback(const geometry_msgs::Twist& vel)
{
    if(vel.linear.x>0 || vel.linear.y>0)
    {
        shut_flag = 1;
    }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "client_node");
    ros::NodeHandle nh;

    /*离线命令词识别*/
    ros::ServiceClient get_offline_recognise_result_client = 
    nh.serviceClient<xf_mic_asr_offline::Get_Offline_Result_srv>("xf_asr_offline_node/get_offline_recognise_result_srv");
    /*设置主麦克风*/
    ros::ServiceClient Set_Major_Mic_client =
    nh.serviceClient<xf_mic_asr_offline::Set_Major_Mic_srv>("xf_asr_offline_node/set_major_mic_srv");
    /*获取主麦克风*/
    ros::ServiceClient Get_Major_Mic_client =
    nh.serviceClient<xf_mic_asr_offline::Get_Major_Mic_srv>("xf_asr_offline_node/get_major_mic_srv");
    /*修改唤醒词*/
    ros::ServiceClient Set_Awake_Word_client =
    nh.serviceClient<xf_mic_asr_offline::Set_Awake_Word_srv>("xf_asr_offline_node/set_awake_word_srv");
    /*订阅唤醒角度*/
    //ros::Subscriber awake_angle_sub = nh.subscribe("/mic/awake/angle", 1, awake_angle_Callback);
    
    ros::ServiceClient send_goal_client = 
    nh.serviceClient<std_srvs::Trigger>("/send_goal_srv");
    
    xf_mic_asr_offline::Get_Offline_Result_srv GetOfflineResult_srv;
    xf_mic_asr_offline::Set_Major_Mic_srv SetMajorMic_srv;
    xf_mic_asr_offline::Get_Major_Mic_srv GetMajorMic_srv;
    xf_mic_asr_offline::Set_Awake_Word_srv SetAwakeWord_srv;
    std_srvs::Trigger SendGoal_srv;

    while(ros::ok())
    {
        // if(flag==0)send_goal_client.call(SendGoal_srv);
        if(flag==0)
        {
            if (send_goal_client.call(SendGoal_srv))
            {
                flag=1;
                ROS_INFO("succeed to call service \"SendGoal_srv\"!");
                //std::cout << "result: " << GetOfflineResult_srv.response.result << endl;
                //std::cout << "fail reason: " << GetOfflineResult_srv.response.fail_reason << endl;
                //std::cout << "text: " << GetOfflineResult_srv.response.text << endl;
                //system("play -t raw -r 16k -e signed -b 16 -c 1 '/home/ucar/ucar_ws/src/xf_mic_asr_offline/audio/vvui_deno.pcm' -q --no-show-progress");
            }
        }
        if(shut_flag)
        {
            ros::shutdown();
        }
    }
    
    ROS_INFO("CALL OK!");

    ros::Subscriber vel_sub =nh.subscribe("/cmd_vel", 10, cmd_callback);
    ros::spin();

    /*
    std::string word = "";
    int recognize_fail_count;
    int recognize_fail_count_threshold;
    int confidence_threshold;
    int time_per_order;
    */
/*
    SetMajorMic_srv.request.mic_id = 2;
	Set_Major_Mic_client.call(SetMajorMic_srv);
    if(Set_Major_Mic_client.call(SetMajorMic_srv))
    {
        ROS_INFO("succeed to call service \"set_major_mic_srv\"!");
        std::cout << "result: " << SetMajorMic_srv.response.result << endl;
        std::cout << "fail reason: " << SetMajorMic_srv.response.fail_reason << endl;
    }
    else
    {
        ROS_INFO("failed to call service \"set_major_mic_srv\"!");     
    }
*/
    /*sleep(2);*/

	/*GetOfflineResult_srv.request.offline_recognise_start = 1;
    GetOfflineResult_srv.request.confidence_threshold = 0;
    GetOfflineResult_srv.request.time_per_order = 5;*/

    /*
	if(if_awake)
	{
        get_offline_recognise_result_client.call(GetOfflineResult_srv);
		if(get_offline_recognise_result_client.call(GetOfflineResult_srv))
		{
			ROS_INFO("succeed to call service \"get_offline_recognise_result_srv\"!");
			//std::cout << "result: " << GetOfflineResult_srv.response.result << endl;
			//std::cout << "fail reason: " << GetOfflineResult_srv.response.fail_reason << endl;
			//std::cout << "text: " << GetOfflineResult_srv.response.text << endl;
			//system("play -t raw -r 16k -e signed -b 16 -c 1 '/home/ucar/ucar_ws/src/xf_mic_asr_offline/audio/vvui_deno.pcm' -q --no-show-progress");
		}
		else
		{
			ROS_INFO("failed to call service \"get_offline_recognise_result_srv\"!");
		}
	}*/

    return 0;
}
