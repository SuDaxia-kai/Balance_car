#ifndef __VECTOR3_H__
#define __VECTOR3_H__

#include <stdint.h>

typedef struct {
    float x;
    float y;
    float z;
} Vector3f_t;

typedef struct {
    double x;
    double y;
    double z;
} Vector3d_t;

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} Vector3i_t;

typedef struct {
    int32_t x;
    int32_t y;
    int32_t z;
} Vector3l_t;
#endif


