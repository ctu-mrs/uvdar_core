#pragma once

#ifndef _GL32_UTILS_H_
#define _GL32_UTILS_H_

#include <stdio.h>
#include <GLES3/gl32.h>

typedef struct {
    uint8_t r;
    uint8_t g;
    uint8_t b;
    uint8_t a;
} rgba_t;

typedef struct {
    int32_t x;
    int32_t y;
} ivec2_t;

typedef struct {
    int32_t x;
    int32_t y;
    int32_t z;
} ivec3_t;

typedef struct {
    float x;
    float y;
    float z;
} vec3_t;

typedef struct {
    int32_t x;
    int32_t y;
    int32_t z;
    int32_t w;
} ivec4_t;

typedef struct {
    float x;
    float y;
    float z;
    float w;
} vec4_t;


const char* gl32_get_define_name(unsigned long define_value);
size_t gl32_get_type_size(GLenum type);
uint32_t gl32_get_image_format_num_components(GLenum format);

#endif
