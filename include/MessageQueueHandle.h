/*****************
 * project:IntelligentPerceptionServer
 * source:MessageQueueHandle.h
 * author:FEX
 * time:2022-10-19
 * description:
 *	单例类
 * 和底层数据回放进程通信的消息队列定义
 * 包括：从消息队列取数据，向消息队列发送数据，分析数据三个线程
 *	
 * copyright:
 * 
 *	请求应答模式：首先上层应用（IntelligentPerceptionServer进程）发送请求消息给底层应用（VideoProcessingServer进程），
 *				  然后上层应用（当前进程）再从底层应用（回放进程fpga_playback）接收消息回馈
 *	推送模式：底层应用（VideoProcessingServer进程）在回放文件数据过程中会推送回放进度到上层应用（当前进程）
 *				
 *	设计：创建一个独立运行的线程，用于接收消息，接收线程接收到消息后直接处理，
 *			需要发送给底层的消息统一调用消息发送接口
 * ***************/
 
#ifndef MESSAGEQUEUEHANDLE_H
#define MESSAGEQUEUEHANDLE_H

#include "Common.h"

using namespace std;

namespace IPSERVER
{
	//接收消息的线程
	//void* OnMSGRecieve(void* arg);
	
	class MessageQueueHandle
	{
		private:
			int m_RecvKey;//接收消息队列Key值，默认8898
			int m_SendKey;//发送消息队列Key值，默认8899
			
			int m_RecvMsgID;//接收信息的消息队列通道操作符
			int m_SendMsgID;//发送信息的消息队列通道操作符
			
			//线程
			//pthread_t	p_Recv;
			
			//状态机,当前接收消息通信状态：1-正在通信, 0-通道正常，-1-断开通信
			int m_RunStatus;
			
			
			//默认无参构造函数
			MessageQueueHandle();
			
			//构造函数重载
			//MessageQueueHandle(const int recvKey=8898, const int sendKey=8899);
			
			
			//析构函数
			//~MessageQueueHandle();
			
			//单例对象定义，私有静态成员对象
			static MessageQueueHandle* m_Object;
			
			//socket通信对象
			//FSocket* m_socket;

            //使用C++11标准类thread创建线程，注意需要gcc版本支持c++11
			//从客户端接收数据执行体
			void OnReceive();
			
			//向客户端发送数据执行体
			//void OnSend();
			
			//解析数据执行体
			//void OnAnalysis();
			
			std::thread r_thread;
			//std::thread s_thread;
			//std::thread a_thread;

			void (*algoCallback)(IPSERVER::ST_MQ_COMSEND_MSG &st_msg);


			
		public:
		
			//获取单例对象
			static MessageQueueHandle* GetInstance();

            
			//私有拷贝构造函数，
			MessageQueueHandle(MessageQueueHandle& object) = delete;
			
			//重载"="运算符，返回值依旧为原始对象的引用
		    MessageQueueHandle& operator=(MessageQueueHandle& object) = delete;
			
            
			//设置接收消息队列key值
			const int SetRecv_Key(const int recvKey);
			//设置发送消息队列key值
			const int SetSend_Key(const int sendKey);
			//获取接收消息队列key值
			const int GetRecv_Key() const;
			//获取发送消息队列key值
			const int GetSend_Key() const;
			
			//设置状态机信息
			const int SetStatus(const int status);
			//获取状态机信息
			const int GetStatus() const;
			
			//发送消息
			const int SendMsg(const IPSERVER::ST_MQ_ALGOSEND_MSG& msg);
			
			//启动线程,
            //返回值1-正在通信, 0-通道正常，准备开启接收消息的线程，-1-断开通信，通信通道不正常，需要重新设置通信MSG_Key值
			int OnStart();
			int OnStart(void(*callback)(IPSERVER::ST_MQ_COMSEND_MSG &st_msg));

            
			//等待线程
			//int OnWait();

            //获取消息的最大长度
			
			//回调函数，回调socket对象里向发送消息队列添加元素
			//int (*OnAddItemToRecvVector)(const IPSERVER::ST_RECV_MSG& msg);
			
			
	};//end class MessageQueueHandle
	
	
}//end namespace IPSERVER
 
 
 
#endif