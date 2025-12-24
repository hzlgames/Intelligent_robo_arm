#pragma once

#include "pch.h"
#include "framework.h"
#include <Windows.h>

//=======1控制舵机转到指定位置==========
//=======命令格式说明===================
//0-1  帧头  0x55 0x55----固定不变
//2    数据长度=控制舵机数*3+5
//3    指令  操作舵机指令0x03----固定不变
//4    要控制的舵机数
//5-6  时间，低8位，高8位
//7-9  舵机ID，角度位置低8位，角度位置高8位
//...  格式与7-9位类似，不同舵机ID
//======================================
//=======举例说明=======================
// 命令     0x55 0x55 0x0b 0x03 0x02 0x20 0x03 0x02 0x20 0x03 0x06 0x20 0x03
// 序号       0    1    2    3    4    5    6    7    8    9    10   11   12 
// 1.第2字节，数据长度11，指从0x0b往后的数据个数（包括0x0b)
// 2.第4字节，控制舵机数2
// 3.第5-6字节，时间800ms，表示800ms内控制机械臂到位
// 4.第7-9字节，舵机2，角度位置800
// 5.第10-12字节，舵机6，角度位置800
// 命令功能：控制舵机2和6在800ms内分别转到800和800的位置
//======================================

//=======2读取舵机角度位置==============
//=======命令格式说明===================
//0-1  帧头  0x55 0x55----固定不变
//2    数据长度=控制舵机数+3
//3    指令  读取舵机角度指令0x15----固定不变
//4    要控制的舵机数
//5... 舵机ID（数目与舵机数对应）
//======================================
//=======返回数据格式说明===============
//0-1  帧头  0x55 0x55----固定不变
//2    数据长度=控制舵机数*3+3
//3    指令  操作舵机指令0x15----固定不变
//4    要控制的舵机数
//5-7  舵机ID，角度位置低8位，角度位置高8位
//...  格式与5-7位类似，不同舵机ID
//======================================
//=======举例说明=======================
// 命令     0x55 0x55 0x05 0x15 0x02 0x01 0x03
// 序号       0    1    2    3    4    5    6 
// 命令功能：读取1、3号舵机的角度位置
// 
// 返回数据   0x55 0x55 0x09 0x15 0x02 0x01 0xf4 0x01 0x03 0xf4 0x01
// 序号       0    1    2    3    4    5    6    7    8    9    10
// 结果：1和3号舵机角度位置均为500
//======================================


//串口类
class SerialPort
{
private:
	HANDLE hSerial = NULL;   //串口句柄
	bool enabled = false;    //串口是否可用
	CString name;            //串口名称

public:
	//串口初始化
	void Initialize(CString comName)
	{
		hSerial = CreateFile(comName, GENERIC_READ | GENERIC_WRITE,
			0, 0, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, 0);
		if (hSerial == INVALID_HANDLE_VALUE)
		{
			enabled = false;
		}
		else
		{
			DCB dcb = { 0 };
			dcb.DCBlength = sizeof(dcb);
			dcb.BaudRate = CBR_9600;    //波特率9600
			dcb.ByteSize = 8;           //数据位数8位
			dcb.StopBits = ONESTOPBIT;  //停止位1位
			dcb.Parity = NOPARITY;      //优先级无
			if (!SetCommState(hSerial, &dcb))
			{
				enabled = false;
			}
			else
			{
				name = comName;
				enabled = true;
			}
		}
	}

	//发送数据
	void Send(char* data, DWORD len)
	{
		if (enabled)
		{
			DWORD bytesWrite;
			bool res = WriteFile(hSerial, data, len, &bytesWrite, NULL);
		}
	}

	//读取数据
	char* Read(DWORD& len)
	{
		if (enabled)
		{
			char* buffer = (char*)malloc(1024 * sizeof(char));
			bool res = ReadFile(hSerial, buffer, sizeof(buffer), &len, NULL);
			return buffer;
		}
	}
};