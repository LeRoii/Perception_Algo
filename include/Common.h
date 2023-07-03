/*****************
 * project:IntelligentPerceptionServer
 * source:common.h
 * author:FEX
 * time:2022-08-20
 * description:
 * 公共处理类定义
 * copyright:
 *
 *
 * ***************/

#ifndef COMMON_H
#define COMMON_H

#include "Model.h"

using namespace std;
using namespace IPSERVER;

namespace IPSERVER
{
	class Common
	{
	private:
			//状态机标志，表示当前执行状态，当前设备运行状态（无效、就绪、正在记录数据、正在回放数据、正在查询目录）
			static IPSERVER::enum_Status m_RunStatus;
			
	public:
		Common();
		//~Common();
		
		//读取配置文件中的配置信息，key-要查询的关键字，value-查询到的值，length-value能容纳的最大字符串长度
		static const int GetValueByKeyFromConfig(const char* key, char* value, const unsigned int length);
		
		//read from  fd
		//输入参数：fd-文件描述符，buffer-存放数据的缓冲区，maxSize-最大读取数据量，timeout-超时时长（ms）
		//输出参数：size-本次读取数据量
		static const int64_t Read(const int fd, uint8_t* buffer, const uint64_t maxSize, uint64_t* size, const uint32_t timeout);

		//write to fd
		//输入参数：fd-文件描述符，buffer-存放数据的缓冲区，maxSize-最大发送数据量，timeout-超时时长（ms）
		//输出参数：size-本次写入的数据量
		static const int64_t Write(const int fd, const uint8_t* buffer, const uint64_t maxSize, uint64_t* size, const uint32_t timeout);
		
		//判断system指令是否执行成功
		//输入参数：result调用system函数返回的值
		//返回值：0~成功，-1~失败
		static const int SystemCheck(int result);
		
		//异或校验，校验值占一个字节（8位）
		//输入参数：buffer-校验数据（含末尾校验值位置），len-需要校验的数据长度（不包括末尾校验值）
		//输出参数：校验值
		//返回值：异或校验结果
		static const uint8_t XOR(uint8_t* buffer, const uint32_t len);
		
		//异或校验检查
		//输入参数：buffer-校验检查数据（包含最后一个校验字节在内），len-数据长度（包含最后一个校验字节在内）
		//输出参数:无
		//返回值：0~成功，-1~失败
		static const int ChechXOR(const uint8_t* buffer, const uint32_t len);
		
		//校验计算
		//输入参数：buffer-校验数据（含末尾校验值位置），len-需要校验的数据长度（不包括末尾校验值）
		//输出参数：无
		//返回值：校验值
		static const int GetCheckSB(uint8_t* buffer, const int32_t len);
		
		//校验检查
		//输入参数：buffer-校验检查数据（包含最后四个校验字节在内），len-数据长度（包含最后四个校验字节在内）
		//输出参数：无
		//返回值：0~成功，-1~失败
		static const int CheckSB(const uint8_t* buffer, const int32_t len);
		
		
		//设置状态信息
		//输入参数：status状态值
		//输出参数：无
		//返回值：0~成功，-1~失败
		static const int SetStatus(const IPSERVER::enum_Status status);
		
	
		//获取状态信息
		//输入参数：无
		//输出参数：无
		//返回值：状态标志值
		static const IPSERVER::enum_Status GetStatus();

        
        //将TCP服务消息转换成字符串
        //msg-消息结构体，str-转换后的字符串保存位置，maxLen-str缓存的最大容量
        //返回str发送内容的总长度
        static int ST_ConvertTo_Char(const ST_MSG &msg, char* str, const uint32_t maxLen);

        
        
        //将IPC消息转换成字符串
        //msg-消息结构体，str-转换后的字符串保存位置，maxLen-str缓存的最大容量
        //返回str发送内容的总长度
        // static int ST_ConvertTo_Char(const ST_MQ_MSG &msg, char* str, const uint32_t maxLen);
		


	};//end class Common

}//end namespace IPSERVER

#endif // COMMON_H
