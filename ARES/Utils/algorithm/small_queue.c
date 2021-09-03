/**
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  *                                 ___     ____   ______ _____
  *                                /   |   / __ \ / ____// ___/
  *                               / /| |  / /_/ // __/   \__ \
  *                              / ___ | / _, _// /___  ___/ / 
  *                             /_/  |_|/_/ |_|/_____/ /____/ 
  *                        Association of Robotics Engineers at SUSTech
  * 
  * @file     small_queue.c
  * @author   Zou Yuanhao (11810102@mail.sustech.edu.cn)
  * @brief    小型队列标志位处理
  * @version  0.1
  * @date     2021-09-02
  * 
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  */
#include "small_queue.h"

void SmallQueue_init(SmallQueue *smallQueue, uint8_t size) {
  smallQueue->front = smallQueue->rear = 0;
  smallQueue->totalSize                = size;
  smallQueue->isFull                   = 0;
}

uint8_t SmallQueue_isFull(SmallQueue *smallQueue) { return smallQueue->isFull; }

uint8_t SmallQueue_isEmpty(SmallQueue *smallQueue) { return (!smallQueue->isFull) && (smallQueue->front == smallQueue->rear); }

uint8_t SmallQueue_getEnqueueIndex(SmallQueue *smallQueue) {
  if (!smallQueue->isFull) {
    return (smallQueue->rear + 1) % smallQueue->totalSize;
  } else {
    return smallQueue->rear;
  }
}

void SmallQueue_enqueue(SmallQueue *smallQueue) {
  if (!SmallQueue_isFull(smallQueue)) {
    smallQueue->rear++;
    smallQueue->rear %= smallQueue->totalSize;
    if (smallQueue->rear == smallQueue->front) {
      smallQueue->isFull = 1;
    }
  }
}

uint8_t SmallQueue_getDequeueIndex(SmallQueue *smallQueue) {
  if(!SmallQueue_isEmpty(smallQueue)){
    return (smallQueue->front + 1) % smallQueue->totalSize;
  }else{
    return smallQueue->front;
  }
}

void SmallQueue_dequeue(SmallQueue *smallQueue) {
  if (!SmallQueue_isEmpty(smallQueue)) {
    smallQueue->front++;
    smallQueue->front %= smallQueue->totalSize;
    if (smallQueue->isFull) {
      smallQueue->isFull = 0;
    }
  }
}
