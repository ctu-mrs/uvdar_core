#pragma once

#ifndef COMPUTE_LIB_H_
#define COMPUTE_LIB_H_

#include <EGL/egl.h>
#include <EGL/eglext.h>
#include <assert.h>
#include <fcntl.h>
#include <gbm.h>
#include <stdbool.h>
#include <string.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>

#include "gl32_utils.h"
#include "queue.h"

#define __GET_DEFINE_VALUE(def) #def
#define __PRINT_DEFINE(def) "#define " #def " " __GET_DEFINE_VALUE(def) "\n"

typedef struct {
    uint32_t lib_id;
    char* message;
    int message_len;
    GLenum source;
    GLenum type;
    GLuint id;
    GLenum severity;
} compute_lib_error_t;

typedef struct {
    bool initialised;
    int32_t fd;
    struct gbm_device* gbm;
    EGLDisplay dpy;
    EGLContext ctx;
    compute_lib_error_t* last_error;
    uint32_t error_total_cnt;
    queue_t* error_queue;
} compute_lib_instance_t;

typedef struct {
    compute_lib_instance_t* lib_inst;
    char* source;
    GLuint handle;
    GLuint shader_handle;
} compute_lib_program_t;

typedef struct {
    GLenum attachment;
    GLuint handle;
} compute_lib_framebuffer_t;

typedef struct {
    const char* uniform_name;
    GLenum texture;
    GLsizei width;
    GLsizei height;
    GLenum internal_format;
    GLenum access;
    GLfloat texture_wrap;
    GLfloat texture_filter;
    GLenum format;
    GLenum type;
    GLuint handle;
    GLuint location;
    uint32_t data_size;
    uint32_t px_size;
    compute_lib_framebuffer_t* framebuffer;
} compute_lib_image2d_t;

typedef struct {
    const char* name;
    GLenum type;
    GLenum usage;
    GLuint handle;
    GLuint index;
} compute_lib_acbo_t;

typedef struct {
    const char* name;
    GLenum type;
    GLenum usage;
    GLuint handle;
    GLuint index;
    GLuint binding;
} compute_lib_ssbo_t;

typedef struct {
    const char* name;
    GLuint location;
    GLuint size;
    GLenum type;
    GLuint index;
} compute_lib_uniform_t;


#define COMPUTE_LIB_INSTANCE_NEW ((compute_lib_instance_t) {.initialised = false, .fd = 0, .gbm = NULL, .dpy = NULL, .ctx = EGL_NO_CONTEXT, .last_error = 0, .error_total_cnt = 0, .error_queue = NULL})
#define COMPUTE_LIB_PROGRAM_NEW(lib_inst_, source_) ((compute_lib_program_t) {.lib_inst = (lib_inst_), .source = (source_), .handle = 0, .shader_handle = 0})
#define COMPUTE_LIB_IMAGE2D_NEW(uniform_name_, texture_, width_, height_, internal_format_, access_, texture_wrap_, texture_filter_, format_, type_) ((compute_lib_image2d_t) {.uniform_name = (uniform_name_), .texture = (texture_), .width = (width_), .height = (height_), .internal_format = (internal_format_), .access = (access_), .texture_wrap = (texture_wrap_), .texture_filter = (texture_filter_), .format = (format_), .type = (type_), .handle = 0, .location = 0, .data_size = 0, .px_size = 0, .framebuffer = NULL})
#define COMPUTE_LIB_SSBO_NEW(name_, type_, usage_) ((compute_lib_ssbo_t) {.name = (name_), .type = (type_), .usage = (usage_), .handle = 0, .index = 0, .binding = 0})
#define COMPUTE_LIB_ACBO_NEW(name_, type_, usage_) ((compute_lib_acbo_t) {.name = (name_), .type = (type_), .usage = (usage_), .handle = 0, .index = 0})
#define COMPUTE_LIB_UNIFORM_NEW(name_) ((compute_lib_uniform_t) {.name = (name_), .location = 0, .size = 0, .type = 0, .index = 0})


#define COMPUTE_LIB_GPU_DRI_BACKUP_PATH "/dev/dri/renderD128"
#define COMPUTE_LIB_GPU_DRI_PATH "/dev/dri/card0"
#define COMPUTE_LIB_MAX_INFO_LOG_LEN 4096

#define COMPUTE_LIB_SEVERITY_PRINT_NOTIFICATION false


enum {
    COMPUTE_LIB_ERROR_NO_ERROR                          = 0,
    COMPUTE_LIB_ERROR_GROUP_INIT_FN                     = -100,
    COMPUTE_LIB_ERROR_ALREADY_INITIALISED               = -101,
    COMPUTE_LIB_ERROR_GPU_DRI_PATH                      = -102,
    COMPUTE_LIB_ERROR_GPU_DRI_BACKUP_PATH               = -152,
    COMPUTE_LIB_ERROR_CREATE_GBM_CTX                    = -103,
    COMPUTE_LIB_ERROR_EGL_PLATFORM_DISPLAY              = -104,
    COMPUTE_LIB_ERROR_EGL_INIT                          = -105,
    COMPUTE_LIB_ERROR_EGL_EXTENSION_CREATE_CTX          = -106,
    COMPUTE_LIB_ERROR_EGL_EXTENSION_KHR_CTX             = -107,
    COMPUTE_LIB_ERROR_EGL_CONFIG                        = -108,
    COMPUTE_LIB_ERROR_EGL_BIND_API                      = -109,
    COMPUTE_LIB_ERROR_EGL_CREATE_CTX                    = -110,
    COMPUTE_LIB_ERROR_EGL_MAKE_CURRENT                  = -111,
    COMPUTE_LIB_ERROR_GROUP_GL_ERROR                    = 0x0500
};

int compute_lib_init(compute_lib_instance_t* inst);
void compute_lib_deinit(compute_lib_instance_t* inst);
int compute_lib_error_queue_flush(compute_lib_instance_t* inst, FILE* out);

bool compute_lib_gl_error_occured(void);
void compute_lib_error_str(int err_code, char* target_str, int* len);

bool compute_lib_program_init(compute_lib_program_t* program);
char* compute_lib_program_get_log(compute_lib_program_t* program, int* output_len);
char* compute_lib_program_get_shader_log(compute_lib_program_t* program, int* output_len);
bool compute_lib_program_print_resources(compute_lib_program_t* program);
bool compute_lib_program_dispatch(compute_lib_program_t* program, GLuint num_groups_x, GLuint num_groups_y, GLuint num_groups_z);
bool compute_lib_program_destroy(compute_lib_program_t* program, bool free_source);

bool compute_lib_framebuffer_init(compute_lib_program_t* program, compute_lib_framebuffer_t* framebuffer);
bool compute_lib_framebuffer_destroy(compute_lib_program_t* program, compute_lib_framebuffer_t* framebuffer);

bool compute_lib_image2d_init(compute_lib_program_t* program, compute_lib_image2d_t* image2d, GLenum framebuffer_attachment);
bool compute_lib_image2d_destroy(compute_lib_program_t* program, compute_lib_image2d_t* image2d);
void* compute_lib_image2d_alloc(compute_lib_image2d_t* image2d);
bool compute_lib_image2d_reset(compute_lib_program_t* program, compute_lib_image2d_t* image2d, void* px_data);
bool compute_lib_image2d_reset_patch(compute_lib_program_t* program, compute_lib_image2d_t* image2d, void* px_data, int x_min, int x_max, int y_min, int y_max);
bool compute_lib_image2d_write(compute_lib_program_t* program, compute_lib_image2d_t* image2d, void* image_data);
bool compute_lib_image2d_read(compute_lib_program_t* program, compute_lib_image2d_t* image2d, void* image_data);
bool compute_lib_image2d_read_patch(compute_lib_program_t* program, compute_lib_image2d_t* image2d, void* image_data, int x_min, int x_max, int y_min, int y_max, bool render);

bool compute_lib_acbo_init(compute_lib_program_t* program, compute_lib_acbo_t* acb, void* data, int len);
bool compute_lib_acbo_destroy(compute_lib_program_t* program, compute_lib_acbo_t* acb);
bool compute_lib_acbo_write(compute_lib_program_t* program, compute_lib_acbo_t* acb, void* data, int len);
bool compute_lib_acbo_write_uint_val(compute_lib_program_t* program, compute_lib_acbo_t* acb, GLuint value);
bool compute_lib_acbo_read(compute_lib_program_t* program, compute_lib_acbo_t* acb, void* data, int len);
bool compute_lib_acbo_read_uint_val(compute_lib_program_t* program, compute_lib_acbo_t* acb, GLuint* value);

bool compute_lib_ssbo_init(compute_lib_program_t* program, compute_lib_ssbo_t* ssbo, void* data, int len);
bool compute_lib_ssbo_destroy(compute_lib_program_t* program, compute_lib_ssbo_t* ssbo);
bool compute_lib_ssbo_write(compute_lib_program_t* program, compute_lib_ssbo_t* ssbo, void* data, int len);
bool compute_lib_ssbo_read(compute_lib_program_t* program, compute_lib_ssbo_t* ssbo, void* data, int len);

bool compute_lib_uniform_init(compute_lib_program_t* program, compute_lib_uniform_t* uniform);
bool compute_lib_uniform_write(compute_lib_program_t* program, compute_lib_uniform_t* uniform, void* data);

#endif
