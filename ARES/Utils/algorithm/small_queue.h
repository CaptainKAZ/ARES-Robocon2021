/**
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  *                                 ___     ____   ______ _____
  *                                /   |   / __ \ / ____// ___/
  *                               / /| |  / /_/ // __/   \__ \
  *                              / ___ | / _, _// /___  ___/ / 
  *                             /_/  |_|/_/ |_|/_____/ /____/ 
  *                        Association of Robotics Engineers at SUSTech
  * 
  * @file     small_queue.h
  * @author   Zou Yuanhao (11810102@mail.sustech.edu.cn)
  * @brief    小型队列的标志位处理
  * @version  0.1
  * @date     2021-09-02
  * 
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  */

/**
 *  @warn: 线程不安全！！！！
  * 使用说明：
  * 先初始化SmallQueue_init(SmallQueue *smallQueue, uint8_t size) 指定队列最大长度
  * 入队：
  * 获得入队index：SmallQueue_getEnqueueIndex(SmallQueue *smallQueue)
  * 对index对应的数组进行操作
  * 然后再调用：SmallQueue_enqueue(SmallQueue *smallQueue) 更新队列
  * 出队：
  * 获得出队index：SmallQueue_getDequeueIndex(SmallQueue *smallQueue)
  * 对index对应的数组进行操作
  * 然后再调用：SmallQueue_dequeue(SmallQueue *smallQueue) 更新队列
  * @note: 队列满了以后入队会直接覆盖最后一个 队列空了以后出队会返回最后一个index
  */

#ifndef SAMALL_QUEUE
#define SMALL_QUEUE
#include "main.h"

typedef struct {
  uint8_t front;
  uint8_t rear;
  uint8_t totalSize : 7;
  uint8_t isFull : 1;
} SmallQueue;

extern void    SmallQueue_init(SmallQueue *smallQueue, uint8_t size);
extern uint8_t SmallQueue_getEnqueueIndex(SmallQueue *smallQueue);
extern void    SmallQueue_enqueue(SmallQueue *smallQueue);
extern uint8_t SmallQueue_getDequeueIndex(SmallQueue *smallQueue);
extern void    SmallQueue_dequeue(SmallQueue *smallQueue);
extern uint8_t SmallQueue_isFull(SmallQueue *smallQueue);
extern uint8_t SmallQueue_isEmpty(SmallQueue *smallQueue);
#endif
