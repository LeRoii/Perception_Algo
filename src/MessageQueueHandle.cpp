#include "MessageQueueHandle.h"

namespace IPSERVER
{
	
	// //接收消息的线程
	// void* OnMQReceive(void* arg)
	// {
	// 	printf("OnMQReceive thread start!\n");
		
	// 	if(NULL == arg)
	// 	{
	// 		printf("OnMQReceive thread end where parameter is NULL\n");
	// 		return NULL;
	// 	}
		
	// 	MessageQueueHandle* msgHandle = (MessageQueueHandle*)arg;
	// 	if(NULL == msgHandle)
	// 	{
	// 		printf("OnMQReceive thread end where parameter is NULL\n");
	// 		return NULL;
	// 	}
		
	// 	if(msgHandle->GetRecv_Key() <= 0)
	// 	{
	// 		printf("OnMQReceive thread end where Recv ID <= 0\n");
	// 		return NULL;
	// 	}
		
	// 	//接收消息接口调用返回值
	// 	size_t recvSize = 0;
		
	// 	//接收消息的长度
	// 	ST_MQ_MSG msg;
	// 	const size_t len = sizeof(ST_MQ_MSG);
	// 	//消息类型，0-代表返回消息队列中最老的消息，不考虑消息类型
	// 	long msgType = 0;
		
        

	// 	//循环接收消息队列中的数据，每5ms接收一次
	// 	while(msgHandle->GetStatus() == 0)
	// 	{
	// 		recvSize = msgrcv(msgHandle->GetRecv_Key(), (void*)&msg, len, msgType, IPC_NOWAIT);
	// 		if(recvSize == -1)
	// 		{
	// 			printf("接收消息失败！\n");
	// 		}
	// 		else
	// 		{
	// 			printf("从回放进程接收到的消息信息如下：\n");
	// 			printf("\n\n消息长度：%ld\n命令号：%d\n\n\n",
	// 					msg.st_MQ_Type, msg.st_MQ_Type);
						
	// 			//判断从底层回放进程fpga_playback返回的消息是否需要回馈给前端
	// 			switch(msg.st_MQ_Type)
	// 			{
    //                 //状态查询指令
	// 				case IPSERVER::e_MQ_SearchStatus:
	// 				{
	// 					std::cout<<"从消息队列接收到状态查询指令回馈消息！"<<std::endl;
	// 					break;
	// 				}
    //                 //目标检测数据推送
	// 				case IPSERVER::e_MQ_TargetDetectionData:
	// 				{
	// 					std::cout<<"从消息队列接收到目标检测数据推送消息！"<<std::endl;
						
	// 					break;
	// 				}
	// 				default:
	// 					break;
	// 			}
	// 		}
			
	// 		usleep(5000);
	// 	}
		
	// 	printf("OnMQReceive thread end!\n");
	// 	return NULL;
	// }


    //使用C++11标准类thread创建线程，注意需要gcc版本支持c++11
    //从客户端接收数据执行体
    void MessageQueueHandle::OnReceive()
	{
		printf("MessageQueueHandle  OnReceive thread start!\n");		
		//接收消息接口调用返回值
		ssize_t recvSize = 0;
		
		//接收消息的长度
		IPSERVER::ST_MQ_COMSEND_MSG st_msg;
		const size_t len = sizeof(ST_MQ_COMSEND_MSG);
		//消息类型，0-代表返回消息队列中最老的消息，不考虑消息类型
		long msgType = 0;

		//循环接收消息队列中的数据，每5ms接收一次
		while(this->GetStatus() == 1)
		{
            //接收消息队列中队首的第一个消息，也就是最古老的消息
			recvSize = msgrcv(this->m_RecvMsgID, (void*)&st_msg, len, msgType, IPC_NOWAIT);
			if(recvSize == -1)
			{
				//printf("MessageQueueHandle  OnReceive thread 接收消息失败！\n");
			}
			else
			{
				printf("MessageQueueHandle  OnReceive thread 接收到的消息信息如下：\n");
				algoCallback(st_msg);


				// printf("\n\n消息长度:%ld\n回馈消息的KEY值\n命令号:%d\n\
                //             跟踪状态：%ld\n识别状态:%ld\n融合状态:%ld\n推流状态:%ld\n\
                //             目标检测有效对象数量：%ld\n脱靶量X:%ld\n脱靶量Y:%ld\n\
                //             \n\n",\
				// 		st_msg->st_MQ_Type, st_msg.st_Resp_Key, st_msg->st_Command,\
                //         st_msg->st_Tracking_Status, st_msg->st_Identify_Status,st_msg->st_Fusion_Status,\
                //         st_msg->st_Plugflow_Status, st_msg->st_Objects_Num, st_msg->st_MissX, st_msg->st_MissY);
						
				//判断从底层进程 返回的消息
				/*switch(st_msg->st_Command)
				{
                    //状态查询指令
					case IPSERVER::e_MQ_SearchStatus:
					{
						std::cout<<"从消息队列接收到状态查询指令回馈消息！"<<std::endl;
                        // IPSERVER::ST_STATUS_RESPOND *st_respond = (IPSERVER::ST_STATUS_RESPOND*)(st_msg->st_msg);

                        // if(NULL != st_respond && nullptr != st_respond)
                        // {
                        //     std::cout<<"跟踪状态："<<st_respond->st_Tracking_Status<<std::endl;
                        //     std::cout<<"识别状态："<<st_respond->st_Identify_Status<<std::endl;
                        //     std::cout<<"融合状态："<<st_respond->st_Fusion_Status<<std::endl;
                        //     std::cout<<"推流状态："<<st_respond->st_Plugflow_Status<<std::endl;
                        // }

						break;
					}
                    //目标检测数据推送
					case IPSERVER::e_MQ_TargetDetectionData:
					{
						std::cout<<"从消息队列接收到目标检测数据推送消息！"<<std::endl;
                        //IPSERVER::ST_DATA_RESPOND *st_respond;

						
						break;
					}
					default:
						break;
				}*/
			}
			
			usleep(5000);
		}
		
		printf("MessageQueueHandle  OnReceive thread end!\n");

        this->SetStatus(0);
		return;
	}
	
	
	//静态对象初始化
	MessageQueueHandle* MessageQueueHandle::m_Object = new MessageQueueHandle();
	
	//默认无参构造函数
	MessageQueueHandle::MessageQueueHandle()
	{
		this->m_RecvKey = 8899;
		this->m_SendKey = 8898;
		
		this->m_RunStatus = 0;//正常通信
		//this->p_Recv = -1;//
		
		this->m_RecvMsgID = msgget(this->m_RecvKey, 0666 | IPC_CREAT);
		if(this->m_RecvMsgID == -1)
		{
			printf("和底层进程进行消息队列通信时创建接收消息的队列失败！！！\n\n");
			this->m_RunStatus = -1;
		}
        
		printf("和底层进程进行消息队列通信时创建接收消息的队列成功！！！\n\n");
		
		this->m_SendMsgID = msgget(this->m_SendKey, 0666 | IPC_CREAT);
		if(this->m_SendMsgID == -1)
		{
			printf("和底层进程进行消息队列通信时创建发送消息的队列失败！！！\n\n");
			this->m_RunStatus = -1;
		}	
		printf("和底层进程进行消息队列通信时创建发送消息的队列成功！！！\n\n");	
	}
	
	
	
	//私有拷贝构造函数，
	// MessageQueueHandle(MessageQueueHandle& object)
	// {
		
	// }
	
	//重载"="运算符，返回值依旧为原始对象的引用
	// MessageQueueHandle& operator=(MessageQueueHandle& object)
	// {
		
	// }
	
	//构造函数重载
	/*MessageQueueHandle::MessageQueueHandle(const int recvKey, const int sendKey)
	{
		this->m_RecvKey = recvKey;
		this->m_SendKey = sendKey;
		
		this->m_RunStatus = 0;
		
		
		this->p_Recv = -1;
		
		this->m_RecvMsgID = msgget(this->m_RecvKey, 0666 | IPC_CREAT);
		if(this->m_RecvMsgID == -1)
		{
			printf("和底层进程进行消息队列通信时创建接收消息的队列失败！！！\n\n");
			this->m_RunStatus = -1;
		}
		
		this->m_SendMsgID = msgget(this->m_SendKey, 0666 | IPC_CREAT);
		if(this->m_SendMsgID == -1)
		{
			printf("和底层进程进行消息队列通信时创建发送消息的队列失败！！！\n\n");
			this->m_RunStatus = -1;
		}
	}
	
	//析构函数
	MessageQueueHandle::~MessageQueueHandle()
	{
		
	}*/
	
	
	
	//获取单例对象
	MessageQueueHandle* MessageQueueHandle::GetInstance()
	{
		if(NULL == MessageQueueHandle::m_Object)
		{
			MessageQueueHandle::m_Object = new MessageQueueHandle();
		}
		
		return MessageQueueHandle::m_Object;
	}
	
	//设置接收消息队列key值
	const int MessageQueueHandle::SetRecv_Key(const int recvKey)
	{
		this->m_RecvKey = recvKey;
		return 0;
	}
	
	//设置发送消息队列key值
	const int MessageQueueHandle::SetSend_Key(const int sendKey)
	{
		this->m_SendKey = sendKey;
		return 0;
	}
	
	//获取接收消息队列key值
	const int MessageQueueHandle::GetRecv_Key() const
	{
		return this->m_RecvKey;
	}
	
	//获取发送消息队列key值
	const int MessageQueueHandle::GetSend_Key() const
	{
		return this->m_SendKey;
	}
	
	//设置状态机信息
	const int MessageQueueHandle::SetStatus(const int status)
	{
		this->m_RunStatus = status;
		return 0;
	}
	
	//获取状态机信息
	const int MessageQueueHandle::GetStatus() const
	{
		return this->m_RunStatus;
	}
	
	//发送消息
	const int MessageQueueHandle::SendMsg(const IPSERVER::ST_MQ_ALGOSEND_MSG& msg)
	{
		//发送消息的消息队列通道创建成功
		if(this->m_SendMsgID != -1)
		{
            int ret = msgsnd(this->m_SendMsgID, &msg, sizeof(IPSERVER::ST_MQ_ALGOSEND_MSG)-sizeof(msg.st_MQ_Type), IPC_NOWAIT);
			if(ret == -1)
			{
				printf("向底层进程发送消息失败！！！\n\n\n");
			}
			else
			{
				printf("向底层进程发送消息成功！！！\n\n\n");
				
				// //准备从底层获取消息				
				// //接收消息接口调用返回值
				// size_t recvSize = 0;
				
				// //接收消息的长度
				// ST_MQ_ALGOSEND_MSG st_msg;
				// const size_t len = sizeof(ST_MQ_ALGOSEND_MSG);
				// //消息类型，0-代表返回消息队列中最老的消息，不考虑消息类型
				// long msgType = 0;
				// //IPC_NOERROR
				// recvSize = msgrcv(this->m_RecvMsgID, (void*)&st_msg, len, msgType, IPC_NOWAIT);
				// if(recvSize == -1)
				// {
				// 	printf("下发命令，接收底层回馈消息失败！\n");
				// 	ret = -1;
				// }
				// else
				// {
				// 	printf("下发命令，接收底层回馈消息成功！\n");

                //     // printf("\n\n消息长度:%ld\n回馈消息的KEY值\n命令号:%d\n
                //     //         跟踪状态：%ld\n识别状态:%ld\n融合状态:%ld\n推流状态:%ld\n
                //     //         目标检测有效对象数量：%ld\n脱靶量X:%ld\n脱靶量Y:%ld\n
                //     //         \n\n",
				// 	// 	st_msg->st_MQ_Type, st_msg.st_Resp_Key, st_msg->st_Command,
                //     //     st_msg->st_Tracking_Status, st_msg->st_Identify_Status,st_msg->st_Fusion_Status,
                //     //     st_msg->st_Plugflow_Status, st_msg->st_Objects_Num, st_msg->st_MissX, st_msg->st_MissY);
					
					
				// }
			}
			return ret;
		}
		else
		{
			return -1;
		}
	}
	
	//启动线程,
    //返回值1-正在通信, 0-通道正常，准备开启接收消息的线程，-1-断开通信，通信通道不正常，需要重新设置通信MSG_Key值
	int MessageQueueHandle::OnStart()
	{
		if(this->m_RunStatus == 0)
		{
            //设置状态运行
            this->SetStatus(1);

			// //创建接收消息的线程
			// ret = pthread_create(&this->p_Recv, NULL, OnMQReceive, this);
			// if(ret != 0)
			// {
			// 	perror("create OnMQReceive thread fail\n");
			// 	return -1;
			// }
			// else
			// {
			// 	printf("create OnMQReceive thread succeed\n");
			// }

            //使用C++11标准类thread创建线程，注意需要gcc版本支持c++11
            //创建接收消息的线程
            this->r_thread = std::thread(&MessageQueueHandle::OnReceive, this);
            r_thread.detach();//主线程与子线程分离，保证主线程结束不影响子线程，确保子线程在后台运行
            printf("MessageQueueHandle 创建接收数据的线程成功\n");
            
		}
        else if(this->m_RunStatus == 1)
        {
            printf("MessageQueueHandle 接收数据的线程已经在运行\n");
        }
        else
        {
            printf("MessageQueueHandle 创建接收数据的线程失败，通道未初始化好\n");
        }
		return this->m_RunStatus;
	}
	//启动线程,
    //返回值1-正在通信, 0-通道正常，准备开启接收消息的线程，-1-断开通信，通信通道不正常，需要重新设置通信MSG_Key值
	int MessageQueueHandle::OnStart(void(*callback)(IPSERVER::ST_MQ_COMSEND_MSG &st_msg))
	{
		algoCallback = callback;
		if(this->m_RunStatus == 0)
		{
            //设置状态运行
            this->SetStatus(1);

			// //创建接收消息的线程
			// ret = pthread_create(&this->p_Recv, NULL, OnMQReceive, this);
			// if(ret != 0)
			// {
			// 	perror("create OnMQReceive thread fail\n");
			// 	return -1;
			// }
			// else
			// {
			// 	printf("create OnMQReceive thread succeed\n");
			// }

            //使用C++11标准类thread创建线程，注意需要gcc版本支持c++11
            //创建接收消息的线程
            this->r_thread = std::thread(&MessageQueueHandle::OnReceive, this);
            r_thread.detach();//主线程与子线程分离，保证主线程结束不影响子线程，确保子线程在后台运行
            printf("MessageQueueHandle 创建接收数据的线程成功\n");
            
		}
        else if(this->m_RunStatus == 1)
        {
            printf("MessageQueueHandle 接收数据的线程已经在运行\n");
        }
        else
        {
            printf("MessageQueueHandle 创建接收数据的线程失败，通道未初始化好\n");
        }
		return this->m_RunStatus;
	}

    
    //等待线程
    // int MessageQueueHandle::OnWait()
    // {
    //     // void *ret;
    //     // pthread_join(p_Recv, &ret);

    //     // std::cout<<"消息队列接收消息线程结束！"<<std::endl;

    //     while(this->m_RunStatus == 1)
    //     {
    //         usleep(500000);
    //     }

    //     return 0;
    // }
	
}//end namespace IPSERVER