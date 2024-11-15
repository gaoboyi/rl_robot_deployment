/*
 * @Description:
 * @Author: kx zhang
 * @Date: 2022-09-20 09:46:42
 * @LastEditTime: 2022-09-20 11:23:43
 */
#pragma once

#include <stdio.h>
#include <iostream>
#define PI 3.14159265359f
#define KP_MINX 0.0f 
#define KP_MAXX 500.0f
#define KD_MINX 0.0f
#define KD_MAXX 5.0f
#define POS_MINX -12.5f
#define POS_MAXX 12.5f
#define SPD_MINX -18.0f
#define SPD_MAXX 18.0f


// 定义执行器类型常量
#define LSG_20_90_7090 0
#define LSG_10_414 1
#define LSG_17_80_6070_new 2
#define LSG_14_70_5060 3

// 定义执行器参数结构体
typedef struct {
    int actuatorType;
    int defRatio;
    float defKT;
    float tMinX;
    float tMaxX;
    float iMinX;
    float iMaxX;
} ActuatorParams;

// 初始化执行器参数
ActuatorParams actuators[] = {
    {LSG_20_90_7090, 101, 0.118f, -32.0f * 2, 32.0f * 2, -22.0f, 22.0f},
    {LSG_10_414, 51, 0.175f, -94.0f * 2, 94.0f * 2, -40.0f, 40.0f},
    {LSG_17_80_6070_new, 51, 0.096f, -19.8f * 2, 19.8f * 2, -20.0f, 20.0f},
    {LSG_14_70_5060, 51, 0.089f, -6.6f * 2, 6.6f * 2, -9.0f, 9.0f}
};


// float fmaxf(float x, float y);
// float fminf(float x, float y);
float fmaxf3(float x, float y, float z);
float fminf3(float x, float y, float z);
void limit_norm(float* x, float* y, float limit);
int float_to_uint(float x, float x_min, float x_max, int bits);
float uint_to_float(int x_int, float x_min, float x_max, int bits);

void float32_to_float16(float* float32, unsigned short int* float16);
void float16_to_float32(unsigned short int* float16, float* float32);

float fmaxf(float x, float y) {
    return (((x) > (y)) ? (x) : (y));
}

float fminf(float x, float y) {
    return (((x) < (y)) ? (x) : (y));
}

float fmaxf3(float x, float y, float z) {
    return (x > y ? (x > z ? x : z) : (y > z ? y : z));
}

float fminf3(float x, float y, float z) {
    return (x < y ? (x < z ? x : z) : (y < z ? y : z));
}

void limit_norm(float* x, float* y, float limit) {
    float norm = sqrt(*x * *x + *y * *y);
    if (norm > limit) {
        *x = *x * limit / norm;
        *y = *y * limit / norm;
    }
}

int float_to_uint(float x, float x_min, float x_max, int bits) {
    float span = x_max - x_min;
    float offset = x_min;
    return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

float uint_to_float(int x_int, float x_min, float x_max, int bits) {
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;  
}

union F32 {
    float v_float;
    unsigned int v_int;
    unsigned char buf[4];
} f32;

void float32_to_float16(float* float32, unsigned short int* float16) {
    unsigned short int temp = 0;
    f32.v_float = *float32;
    temp = (f32.buf[3] & 0x7F) << 1 | ((f32.buf[2] & 0x80) >> 7);
    temp -= 112;
    *float16 = temp << 10 | (f32.buf[2] & 0x7F) << 3 | f32.buf[1] >> 5;
    *float16 |= ((f32.v_int & 0x80000000) >> 16);
}

void float16_to_float32(unsigned short int* float16, float* float32) {
    unsigned short int temp2 = 0;
    f32.v_int = 0;
    temp2 = (((*float16 & 0x7C00) >> 10) + 112);
    f32.buf[3] = temp2 >> 1;
    f32.buf[2] = ((temp2 & 0x01) << 7) | (*float16 & 0x03FC) >> 3;
    f32.buf[1] = (*float16 & 0x03) << 6;
    f32.v_int |= ((*float16 & 0x8000) << 16);
    *float32 = f32.v_float;
}




