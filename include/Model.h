/*****************
 * project:IntelligentPerceptionServer
 * source:Model.h
 * author:FEX
 * time:2023-03-24
 * description:
 * 数据模型类定义
 *	定义程序中用到的数据类型
 * ***************/
 
#ifndef MODEL_H
#define MODEL_H


#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <assert.h>
#include <errno.h>
#include <stdlib.h>
#include <time.h>
#include <fcntl.h>
#include <dirent.h>

#include <pthread.h>

#include <sys/stat.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h> 
#include <arpa/inet.h>
#include <sys/msg.h>

#include <signal.h>	
#include <semaphore.h>


#include <iostream>
#include <fstream>
#include <vector>
#include <string>

#include <thread>
#include <mutex>

#include "types.h"




using namespace std;

namespace IPSERVER
{
	//回调函数，主要用于处理消息
	using CALLBACK_FUNC =  void*(*)(void*);

    //定义 IP地址长度
    const std::uint16_t IP_LEN(32);
    //定义 port
    const std::uint8_t PORT_LEN(8);
    //定义socket消息最大长度
    const std::uint32_t MSG_LEN(1024);
    //定义程序运行路径字符串最大长度
    const std:: uint16_t MAX_DIRPATH(1024);

    //定义视频采集及推流子系统运行状态情况
    typedef enum
    {
        e_None=0,//失效
        e_Ready,//就绪

        e_StatusMax
    } enum_Status;

    //下发命令定义
    typedef enum
    {
        e_HeartBeat = 0,            //心跳包
        e_StartRTSP,				//开始建图命令
        e_StopRTSP,                 //停止建图命令
        e_SaveMap,                  //保存地图命令
        e_ExportMap,				//导出地图命令
        e_Move,                     //移动缩放图像（详细移动参数在st_msg中）
        e_ViewpointSwitch,          //视角切换
        e_CoordinateSystemSwitch,	//坐标系切换

        e_CommandMax
    }enum_Command;


    #pragma pack(1)
    ////////
    ////////TCP通信消息结构定义 start

    //消息头定义
    typedef struct _ST_HEADER
    {
        uint64_t        st_len;//消息体长度，不包括头和尾长度
        enum_Command    st_command;//指令代号 占4字节
    }ST_HEADER;

    //消息尾定义
    typedef struct _ST_TAIL
    {
        uint16_t    st_crc16;//定义crc16校验
    }ST_TAIL;

    //TCP通信消息结构定义
    typedef struct _ST_MSG
    {
        ST_HEADER   st_head;
        uint8_t     *st_msg;
        ST_TAIL     st_tail;
    }ST_MSG;



    //服务器-》客户端，以下指令服务器应答客户端，st_msg结构定义
    //回应开始建图指令、停止建图指令、保存地图指令、导出地图指令
    //视角切换指令、坐标系切换指令、移动缩放图像、
    typedef struct _ST_RESPONSE
    {
        uint8_t  st_result;    //指令执行结论：0-成功，其他值-失败
        char st_date[19];          //时间戳，格式为字符串YYYY-MM-DD hh:mm:ss

        //返回的字符串信息，限制长度不超过512字节
        //当服务器回馈导出地图命令的时候，这里保存服务器上点云地图文件绝对路径
        char*    st_msg;

    }ST_RESPONSE;


    //每种类型的命令 服务器收到客户端的请求消息 消息体st_msg定义
    //
    //
    //心跳命令e_HeartBeat消息体
    //时间戳，格式为字符串YYYY-MM-DD hh:mm:ss
    typedef struct _ST_HEARTBEAT
    {
        char st_date[19];
    }ST_HEARTBEAT;

    
    //开始建图命令e_StartRTSP
    typedef struct _ST_START_RTSP
    {
        uint8_t st_ViewpointType;      //视角类型，0x01-FPS，0x02-TopDown
        uint8_t  st_CoodinateSystemType;    //坐标系类型，0x01-map，0x02-odom
    }ST_START_RTSP;
	
    //停止建图命令e_StopRTSP消息体，客户端-》服务器，st_msg字段为NULL
    //保存地图命令e_SaveMap消息体，客户端-》服务器，st_msg字段为NULL
    //导出地图命令e_ExportMap消息体，客户端-》服务器，st_msg字段为NULL

    //移动缩放图像命令e_Move
    typedef struct _ST_MOVE
    {
        uint8_t  st_MoveType;       //类型：0-按钮操作，1-鼠标操作，2-触屏操作
        uint8_t  st_MouseBtnType;   //使用鼠标移动时，按下的是左键还是右键，0x00-未按键，0x01-鼠标右键，0x10-鼠标左键
        int32_t st_X;		//当前点位X（缩放前的中心点位X）
		int32_t st_Y;		//当前点位Y（缩放前的中心点位X）
		int32_t  st_moveX;          //垂直移动距离，单位：m/单位：像素，上正下负
        int32_t  st_moveY;          //水平移动距离，单位：m/单位：像素，右正左负
        float    st_ScaleFactor;    //缩放因子，
    }ST_MOVE;

    //视角切换命令e_ViewpointSwitch
    typedef struct _ST_VIEWPOINT_SWITCH
    {
        uint8_t  st_ViewpointType;      //视角类型，0x01-FPS，0x02-TopDown
    }ST_VIEWPOINT_SWITCH;

    //坐标系切换命令e_CoordinateSystemSwitch
    typedef struct _ST_COORDINATESYSTEM_SWITCH
    {
        uint8_t  st_CoodinateSystemType;    //坐标系类型，0x01-map，0x02-odom
    }ST_COORDINATESYSTEM_SWITCH;


    ////////
    ////////TCP通信消息结构定义 end


    
    ////////
    ////////消息队列通信消息结构定义 start

    //命令定义
    typedef enum
    {
        e_MQ_None = 0,
        e_MQ_SearchStatus,  //状态查询命令
        e_MQ_TargetDetectionData,  //目标检测数据推送命令
        e_MQ_Set_Tracking_Mode,//设置跟踪模式
        e_MQ_Set_Identify_Mode,//设置识别模式
        e_MQ_Set_Fusion_Mode,//设置图像类型
        e_MQ_Set_Plugflow_Mode,//设置推流状态
        e_MQ_Set_Draw_Cros_Mode,//设置十字分划叠加

        e_MQ_Command_Max

    } enum_MQ_Command;

    
    typedef struct _ST_MQ_ALGOSEND_MSG
    {
        long int st_MQ_Type;//消息长度（不包含此字段）
		key_t st_Resp_Key;//待回应的消息队列key值，0-表示无需回应，其他-表示需要回应
        enum_MQ_Command st_Command;//命令类型
        
        uint8_t st_Tracking_Status; //跟踪，0-否，1-是
        uint8_t st_Identify_Status; //识别，0-否，1-是
        uint8_t st_Fusion_Status;	//融合，0-否，1-是
        uint8_t st_Plugflow_Status;	//推流，0-否，1-是

        int32_t		st_MissX;//脱靶量X
        int32_t		st_MissY;//脱靶量Y

        int16_t st_Tracking_Target_ID;
        uint8_t st_Tracking_Gate_Size;  //0-small,1-mid,2-large
        uint8_t st_Draw_Cross;


        uint32_t	st_Objects_Num;//目标数量
        bbox_t      st_Objects_Content[IP_LEN];//目标bbox数据

        // std::vector<bbox_t> st_Objects_Content;//目标bbox数据

        // long int st_MQ_Type;//消息长度（不包含此字段）
		// key_t st_Resp_Key;//待回应的消息队列key值，0-表示无需回应，其他-表示需要回应
        // enum_MQ_Command st_Command;//命令类型
        
        // uint8_t st_Tracking_Status; //跟踪，0-否，1-是
        // uint8_t st_Identify_Status; //识别，0-否，1-是
        // uint8_t st_Fusion_Status;	//融合，0-否，1-是
        // uint8_t st_Plugflow_Status;	//推流，0-否，1-是

        // int32_t		st_MissX;//脱靶量X
        // int32_t		st_MissY;//脱靶量Y

        // uint32_t	st_Objects_Num;//目标数量
        // std::vector<bbox_t> st_Objects_Content;//目标bbox数据

    }ST_MQ_ALGOSEND_MSG;

    
    typedef struct _ST_MQ_COMSEND_MSG
    {
        long int st_MQ_Type;//消息长度（不包含此字段）
		key_t st_Resp_Key;//待回应的消息队列key值，0-表示无需回应，其他-表示需要回应
        enum_MQ_Command st_Command;//命令类型
        uint8_t st_Tracking_Status; //跟踪，0-否,1-center, 2-id
        uint8_t st_Identify_Status; //识别，0-否，1-是
        uint8_t st_Fusion_Status;	//像类型，0-可见光，1-红外，2-融合
        uint8_t st_Plugflow_Status;	//推流，0-否，1-是

        int16_t st_Tracking_Target_ID;
        uint8_t st_Tracking_Gate_Size;  //0-small,1-mid,2-large
        uint8_t st_Draw_Cross;
        uint16_t st_Draw_Cross_Point[2];
        uint16_t st_Tracking_Point[2];
        uint32_t st_ZoomScale;

    }ST_MQ_COMSEND_MSG;


    // //状态下发消息结构定义 ST_MQ_MSG::st_msg 定义为空

    // //状态回馈消息结构定义 ST_MQ_MSG::st_msg 定义为ST_STATUS_RESPOND
    // //详细字段
    // typedef struct _ST_STATUS_RESPOND
    // {
    //     uint8_t st_Tracking_Status; //跟踪，0-否，1-是
    //     uint8_t st_Identify_Status; //识别，0-否，1-是
    //     uint8_t st_Fusion_Status;	//融合，0-否，1-是
    //     uint8_t st_Plugflow_Status;	//推流，0-否，1-是

    // }ST_STATUS_RESPOND;


    // //目标检测数据推送指令回馈消息结构定义
    // //ST_MQ_MSG::st_msg 定义为ST_DATA_RESPOND

    // struct bbox_t 
    // {
    //     unsigned int x, y, w, h;       // (x,y) - top-left corner, (w, h) - width & height of bounded box
    //     float prob;                    // confidence - probability that the object was found correctly
    //     unsigned int obj_id;           // class of object - from range [0, classes-1]
    //     unsigned int track_id;         // tracking id for video (0 - untracked, 1 - inf - tracked object)
    //     unsigned int frames_counter;   // counter of frames on which the object was detected
    //     float x_3d, y_3d, z_3d;        // center of object (in Meters) if ZED 3D Camera is used
    // };


    // //详细字段
    // typedef struct _ST_DATA_RESPOND
    // {
    //     uint32_t		st_Objects_Num;//目标数量
    //     std::vector<bbox_t> st_Objects_Content;//目标bbox数据
    //     int32_t		st_MissX;//脱靶量X
    //     int32_t		st_MissY;//脱靶量Y

    // }ST_DATA_RESPOND;



    ////////
    ////////消息队列通信消息结构定义 end
	
	
}//end namespace IPSERVER



#endif
