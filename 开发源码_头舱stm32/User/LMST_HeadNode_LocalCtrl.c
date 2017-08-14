/*
 * LMST_HeadNode_LocalCtrl.c
 *
 * Created: 2016/9/2 
 *  Author: YJ
 * Email: huangyj@lzrobot.com
 */
 
#include "cJSON.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "SmartTuna.h"
#include "LMST_HeadNode_LocalCtrl.h"
#include "LMST_NodeCommon.h"

/* ============================================
		向尾舱发送控制命令

		param
		int32_t speed	 尾鳍速度  范围 0 ~ 15
		int32_t direct 尾鳍方向  范围 0 ~ 14  
=============================================*/
static void TailCtrl(int32_t speed, int32_t direct)
{
	char tmp[50] = {0};
	char buf[60];
	int32_t ret;
	
	snprintf(tmp,50, "{\"c\": \"SetSpeed\", \"n\": 0, \"value\": %d}", speed);
	ret = YJ_PackedApl(buf, 3, tmp, strlen(tmp));
	YJ_CudpSendTo(g_CudpCore, buf, ret, LMST_TAIL_NODE);
  //mDEBUG("%s",tmp);
	
	snprintf(tmp,50, "{\"c\": \"SetDirection\", \"n\": 0, \"value\": %d}", direct);
  //mDEBUG("%s",tmp);
	ret = YJ_PackedApl(buf, 3, tmp, strlen(tmp));
	YJ_CudpSendTo(g_CudpCore, buf, ret, LMST_TAIL_NODE);
}

/* ============================================
		向舱螺旋桨推进舱发送控制命令

		param
	int32_t Servo_l		左舵机	范围 0 ~ 14 	
	int32_t Motor_l		左电机	范围 0 ~ 14
	int32_t Servo_r		右舵机	范围 0 ~ 14
	int32_t Motor_r 	右电机	范围 0 ~ 14
=============================================*/
static void ServoMotorCtrl(int32_t Servo_l, int32_t Motor_l, int32_t Servo_r, int32_t Motor_r )
{
	char tmp[60] = {0};
	char buf[70];
	int32_t ret;
	
	snprintf(tmp,60, "{\"c\": \"SetSteerDirection_l\", \"n\": 0, \"value\": %d}", Servo_l);
	ret = YJ_PackedApl(buf, 3, tmp, strlen(tmp));
	YJ_CudpSendTo(g_CudpCore, buf, ret, LMST_SERVO_MOTOR_NODE);
  //mDEBUG("%s",tmp);
	
	snprintf(tmp,60, "{\"c\": \"SetMotorSpeed_l\", \"n\": 0, \"value\": %d}", Motor_l);
	ret = YJ_PackedApl(buf, 3, tmp, strlen(tmp));
	YJ_CudpSendTo(g_CudpCore, buf, ret, LMST_SERVO_MOTOR_NODE);
	//mDEBUG("%s",tmp);
	
	snprintf(tmp,60, "{\"c\": \"SetSteerDirection_r\", \"n\": 0, \"value\": %d}", Servo_r);
	ret = YJ_PackedApl(buf, 3, tmp, strlen(tmp));
	YJ_CudpSendTo(g_CudpCore, buf, ret, LMST_SERVO_MOTOR_NODE);
  //mDEBUG("%s",tmp);
	
	snprintf(tmp,60, "{\"c\": \"SetMotorSpeed_r\", \"n\": 0, \"value\": %d}", Motor_r);
	ret = YJ_PackedApl(buf, 3, tmp, strlen(tmp));
	YJ_CudpSendTo(g_CudpCore, buf, ret, LMST_SERVO_MOTOR_NODE);
	//mDEBUG("%s",tmp);
}

/* ============================================
		获取mpu 数据
			return 
				1		成功
				0		失败

		param
float data[5] : pich roll yaw pressure temp
=============================================*/
static int32_t GetSensorData(float data[5])
{
	char tmp[] =  "{\"c\": \"HeartBeat\", \"n\": 0, \"value\": 0}";
	char buf[LM_MAX_CAN_PAYLOAD];
	int32_t ret, SrcAddr, i;
	cJSON *json = NULL , *s = NULL, *v = NULL;
	const portTickType xDelay = pdMS_TO_TICKS(50);  
	
	ret = YJ_PackedApl(buf, 3, tmp, strlen(tmp));
	YJ_CudpSendTo(g_CudpCore, buf, ret, LMST_SENSOR_NODE);
	
	for(i = 0; i < 20; i++)
	{
			vTaskDelay( xDelay );
	
		if((ret = YJ_CudpRecv(g_CudpCore, buf, LM_MAX_CAN_PAYLOAD))  > 0)
		{
			if((SrcAddr = YJ_AplGetAddr(buf)) != LMST_SENSOR_NODE || YJ_AplGetType(buf) != 3)
				continue;
			
			do
			{
				// 解析数据包
				json = cJSON_Parse(buf + 8);
				//mDEBUG("%s",buf + 8);
				if (!json)
				{
					mERR("cJSON_Parse fail");
					break;
				}
				else
				{
					s = cJSON_GetObjectItem( json , "c");
					if( s == NULL || s->type != cJSON_String || (strcmp(s->valuestring, "HeartBeat") != 0))
					{
						mERR("error c");
						break;
					}
					
					if((v = cJSON_GetObjectItem( json , "pich")) == NULL )
						break;
					
					//mDEBUG("%s , %lf %d",buf + 8, v->valuedouble, v->valueint);
					
					data[0] = v->valuedouble;		
						
					if((v = cJSON_GetObjectItem( json , "roll")) == NULL )
						break;
					
					data[1] = v->valuedouble;	

					if((v = cJSON_GetObjectItem( json , "yaw")) == NULL )
						break;
					
					data[2] = v->valuedouble;				
					
					if((v = cJSON_GetObjectItem( json , "press")) == NULL )
						break;
					
					data[3] = v->valuedouble;	
					
					if((v = cJSON_GetObjectItem( json , "temp")) == NULL )
						break;
					
					data[4] = v->valuedouble;	
										
					cJSON_Delete(json);
					return 1;
					//mDEBUG("c: %d", s->valueint);
				}
				
			}while(0);
			 
			if(json)
			 cJSON_Delete(json);
		}
	}
	return 0;
}

static void ReplyHeatBeat(char *msg)
{
	char *tmp = "{\"c\":\"HeartBeat\",\"n\":0,\"s\":1,\"temp\":%d,\"humidity\":%d}";
	char buf[240];
	
	snprintf(buf, 240, tmp, 0, 0);
	YJ_SendMsg(buf, 3);
}

static void ProcMsg(char *msg)
{
		cJSON *json = NULL , *s = NULL;
						
	 do{
			// 解析数据包
			//json = cJSON_Parse("{\"c\":\"SetMotorSpeed_l\",\"n\":0,\"value\":7}");
		 json = cJSON_Parse(msg);
			if (!json)
			{
				mERR("cJSON_Parse fail");
				break;
			}
			else
			{
				s = cJSON_GetObjectItem( json , "c");
				if( s == NULL || s->type != cJSON_String)
				{
					mERR("have not c");
					break;
				}
				
				if(strcmp(s->valuestring, "OpenLight") == 0)
				{
					GPIO_SetBits(GPIOA,GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7);			
				}
				else if(strcmp(s->valuestring, "CloseLight") == 0)
				{
					GPIO_ResetBits(GPIOA,GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7);
				}
				else if(strcmp(s->valuestring, "HeartBeat") == 0)
				{
					ReplyHeatBeat(msg);
				}
				else if(strcmp(s->valuestring, "Reboot") == 0)
				{
					mDEBUG("Reboot");
					YJ_UnInitSystem();
					YJ_ReBoot();
				}
				else
				{
					mERR("error c");
					break;
				}
				
				//mDEBUG("c: %d", s->valueint);
			}
			
		}while(0);
	 
		if(json)
		 cJSON_Delete(json);
}


void YJ_HeadNodeTask__( void *parameters ) 
{
	int32_t 					ret = 1; 
	char       				* tmp = pvPortMalloc(LM_MAX_CAN_PAYLOAD);
	const portTickType 	xDelay = pdMS_TO_TICKS(3);  //宏pdMS_TO_TICKS用于将毫秒转成节拍数
		
	while(1)
	{			
		vTaskDelay( xDelay);
		//接收上位机命令
		if((ret = YJ_UudpRecv(g_UudpCore, tmp, LM_MAX_CAN_PAYLOAD)) > 0)
		{
			if(YJ_AplGetAddr(tmp) == LMST_HEAD_NODE)
			{
				//mERR("reboot-------------------------------");
				ProcMsg(tmp + 8);//除去8字节的应用层头
			}
		}
	}
}

//头舱自动控制demo任务
void YJ_HeadNodeTask_LocalCtrl( void *parameters ) 
{
		int32_t 		ret = 1; 
	  int32_t   	SrcAddr;
		//YJ_MpuParam	p;
		float data[5];
	
	  const portTickType xDelay = pdMS_TO_TICKS(3);  //宏pdMS_TO_TICKS用于将毫秒转成节拍数
		const portTickType xDelay_5s = pdMS_TO_TICKS(5000);  
		const portTickType xDelay_1s = pdMS_TO_TICKS(1000);  
	
	//创建用于接收上位机命令的线程
		xTaskCreate( YJ_HeadNodeTask__, ( signed portCHAR * ) "YJ_HeadNodeTask", 
		256, NULL, 
			LMST_NODE_TASK_PRIORITY, NULL);
		
		mDEBUG("boot ----> user");
		while(1)
		{			
				
			//尾舱控制
			TailCtrl(15, 7);
			vTaskDelay( xDelay_5s );
			TailCtrl(0, 7);
			vTaskDelay( xDelay_1s );
			
			//螺旋桨推进舱控制
			ServoMotorCtrl(8, 10, 8, 10);
			vTaskDelay( xDelay_5s );
			ServoMotorCtrl(7, 7, 7, 7);
			vTaskDelay( xDelay_1s );
			
			//mpu数据获取
			if(!GetSensorData(data))
				continue;
			
			//打印调试信息到pc端
			mDEBUG("mpu data -> pich: %f roll: %f yaw: %f press: %d temp: %f", data[0],  data[1], data[2],  (int32_t)data[3], data[4]/10);
		}
}