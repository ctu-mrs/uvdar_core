#include "compute_lib.h"


static const EGLint egl_config_attribs[] = { EGL_RENDERABLE_TYPE, EGL_OPENGL_ES3_BIT_KHR, EGL_NONE };
static const EGLint egl_ctx_attribs[] = { EGL_CONTEXT_CLIENT_VERSION, 3, EGL_NONE };


static void compute_lib_gl_callback(GLenum source, GLenum type, GLuint id, GLenum severity, GLsizei length, const GLchar* message, const void* inst)
{
    if (severity == GL_DEBUG_SEVERITY_NOTIFICATION && !COMPUTE_LIB_SEVERITY_PRINT_NOTIFICATION) {
        return;
    }
    
    compute_lib_error_t* err = (compute_lib_error_t*) calloc(1, sizeof(compute_lib_error_t));
    err->message_len = length;
    if (message[length-1] == '\n') err->message_len -= 1;
    err->message = (char*) calloc(err->message_len + 1, sizeof(char));
    memcpy(err->message, message, err->message_len);
    err->lib_id = ((compute_lib_instance_t*) inst)->error_total_cnt;
    err->id = id;
    err->type = type;
    err->severity = severity;
    err->source = source;

    if (severity == GL_DEBUG_SEVERITY_LOW || severity == GL_DEBUG_SEVERITY_MEDIUM || severity == GL_DEBUG_SEVERITY_HIGH) {
        queue_push(((compute_lib_instance_t*) inst)->error_queue, (void*) err);
        ((compute_lib_instance_t*) inst)->error_total_cnt++;
        ((compute_lib_instance_t*) inst)->last_error = err;
    }
}

int compute_lib_init(compute_lib_instance_t* inst)
{
  fprintf(stderr, "[ComputeLib]: Checking if initialized... " );
    if (inst->initialised == true) {
        return COMPUTE_LIB_ERROR_ALREADY_INITIALISED;
    }

    fprintf(stderr, "[ComputeLib]: Opening GPU DRI PATH... " );
    inst->fd = open(COMPUTE_LIB_GPU_DRI_PATH, O_RDWR);
    if (inst->fd <= 0) {
        compute_lib_deinit(inst);
        return COMPUTE_LIB_ERROR_GPU_DRI_PATH;
    }

    fprintf(stderr, "[ComputeLib]: Creating BGM device... " );
    inst->gbm = gbm_create_device(inst->fd);
    if (inst->gbm == NULL) {
        compute_lib_deinit(inst);
        return COMPUTE_LIB_ERROR_CREATE_GBM_CTX;
    }

    fprintf(stderr, "[ComputeLib]: Getting EGL platform display... " );
    inst->dpy = eglGetPlatformDisplay(EGL_PLATFORM_GBM_MESA, inst->gbm, NULL);
    if (inst->dpy == NULL) {
        compute_lib_deinit(inst);
        return COMPUTE_LIB_ERROR_EGL_PLATFORM_DISPLAY;
    }

    fprintf(stderr, "[ComputeLib]: Initializing EGL... " );
    if (!eglInitialize(inst->dpy, NULL, NULL)) {
        compute_lib_deinit(inst);
        return COMPUTE_LIB_ERROR_EGL_INIT;
    }

    fprintf(stderr, "[ComputeLib]: Getting EGL extensions... " );
    const char* egl_extension_st = eglQueryString(inst->dpy, EGL_EXTENSIONS);
    if (strstr(egl_extension_st, "EGL_KHR_create_context") == NULL) {
        compute_lib_deinit(inst);
        return COMPUTE_LIB_ERROR_EGL_EXTENSION_CREATE_CTX;
    }
    if (strstr(egl_extension_st, "EGL_KHR_surfaceless_context") == NULL) {
        compute_lib_deinit(inst);
        return COMPUTE_LIB_ERROR_EGL_EXTENSION_KHR_CTX;
    }

    EGLConfig egl_cfg;
    EGLint egl_count;
    fprintf(stderr, "[ComputeLib]: Choosing EGL config... " );
    if (!eglChooseConfig(inst->dpy, egl_config_attribs, &egl_cfg, 1, &egl_count)) {
        compute_lib_deinit(inst);
        return COMPUTE_LIB_ERROR_EGL_CONFIG;
    }
    fprintf(stderr, "[ComputeLib]: Binding EGL API... " );
    if (!eglBindAPI(EGL_OPENGL_ES_API)) {
        compute_lib_deinit(inst);
        return COMPUTE_LIB_ERROR_EGL_BIND_API;
    }

    fprintf(stderr, "[ComputeLib]: Crating EGL context... " );
    inst->ctx = eglCreateContext(inst->dpy, egl_cfg, EGL_NO_CONTEXT, egl_ctx_attribs);
    if (inst->ctx == EGL_NO_CONTEXT) {
        compute_lib_deinit(inst);
        return COMPUTE_LIB_ERROR_EGL_CREATE_CTX;
    }

    fprintf(stderr, "[ComputeLib]: Making EGL current... " );
    if (!eglMakeCurrent(inst->dpy, EGL_NO_SURFACE, EGL_NO_SURFACE, inst->ctx)) {
        compute_lib_deinit(inst);
        return COMPUTE_LIB_ERROR_EGL_MAKE_CURRENT;
    }

    inst->last_error = 0;
    inst->error_total_cnt = 0;
    inst->error_queue = queue_create(64);

    fprintf(stderr, "[ComputeLib]: Enabling GL output... " );
    glEnable(GL_DEBUG_OUTPUT);
    fprintf(stderr, "[ComputeLib]: Setting message callback... " );
    glDebugMessageCallback(compute_lib_gl_callback, (const void*) inst);
    
    fprintf(stderr, "[ComputeLib]: Initialized... " );
    inst->initialised = true;

    return 0;
}


void compute_lib_deinit(compute_lib_instance_t* inst)
{
    if (inst->ctx != EGL_NO_CONTEXT && inst->dpy != NULL) {
        eglDestroyContext(inst->dpy, inst->ctx);
    }
    inst->ctx = EGL_NO_CONTEXT;

    if (inst->dpy != NULL) {
        eglTerminate(inst->dpy);
    }
    inst->dpy = NULL;

    if (inst->gbm != NULL) {
        gbm_device_destroy(inst->gbm);
    }
    inst->gbm = NULL;

    if (inst->fd > 0) {
        close(inst->fd);
    }
    inst->fd = 0;

    compute_lib_error_queue_flush(inst, NULL);
    queue_delete(inst->error_queue);

    inst->initialised = false;
}

int compute_lib_error_queue_flush(compute_lib_instance_t* inst, FILE* out)
{
    int i = 0;
    compute_lib_error_t* err;
    while (queue_size(inst->error_queue) > 0) {
        err = (compute_lib_error_t*) queue_pop(inst->error_queue);
        if (out != NULL) {
            fprintf(stderr, "compute_lib: GL error #%d: %s (0x%X), severity: %s (0x%X), message = %.*s\n", err->lib_id, gl32_get_define_name(err->type), err->type, gl32_get_define_name(err->severity), err->severity, err->message_len, err->message);
        }
        free(err->message);
        free(err);
        i++;
    }
    return i;
}

void compute_lib_error_str(int err_code, char* target_str, int* len)
{
    switch (err_code) {
        case COMPUTE_LIB_ERROR_NO_ERROR:
            *len += sprintf(target_str, "compute_lib_init error: no error.\r\n");
            break;
        case COMPUTE_LIB_ERROR_GROUP_INIT_FN:
            *len += sprintf(target_str, "compute_lib error: occurs at: int compute_lib_init(compute_lib_instance_t* inst);\r\n");
            break;
        case COMPUTE_LIB_ERROR_ALREADY_INITIALISED:
            *len += sprintf(target_str, "compute_lib_init error: already initialised!\r\n");
            break;
        case COMPUTE_LIB_ERROR_GPU_DRI_PATH:
            *len += sprintf(target_str, "compute_lib_init error: could not open '%s'!\r\n", COMPUTE_LIB_GPU_DRI_PATH);
            break;
        case COMPUTE_LIB_ERROR_CREATE_GBM_CTX:
            *len += sprintf(target_str, "compute_lib_init: error: could not create GBM context!\r\n");
            break;
        case COMPUTE_LIB_ERROR_EGL_PLATFORM_DISPLAY:
            *len += sprintf(target_str, "compute_lib_init error: could not get platform display!\r\n");
            break;
        case COMPUTE_LIB_ERROR_EGL_INIT:
            *len += sprintf(target_str, "compute_lib_init error: could not initialise EGL!\r\n");
            break;
        case COMPUTE_LIB_ERROR_EGL_EXTENSION_CREATE_CTX:
            *len += sprintf(target_str, "compute_lib_init error: could not locate extension: EGL_KHR_create_context!\r\n");
            break;
        case COMPUTE_LIB_ERROR_EGL_EXTENSION_KHR_CTX:
            *len += sprintf(target_str, "compute_lib_init error: could not locate extension: EGL_KHR_surfaceless_context!\r\n");
            break;
        case COMPUTE_LIB_ERROR_EGL_CONFIG:
            *len += sprintf(target_str, "compute_lib_init error: could not choose EGL configuration!\r\n");
            break;
        case COMPUTE_LIB_ERROR_EGL_BIND_API:
            *len += sprintf(target_str, "compute_lib_init error: could not bind EGL_OPENGL_ES_API!\r\n");
            break;
        case COMPUTE_LIB_ERROR_EGL_CREATE_CTX:
            *len += sprintf(target_str, "compute_lib_init error: could not create EGL context!\r\n");
            break;
        case COMPUTE_LIB_ERROR_EGL_MAKE_CURRENT:
            *len += sprintf(target_str, "compute_lib_init error: could not make current EGL context!\r\n");
            break;
        case COMPUTE_LIB_ERROR_GROUP_GL_ERROR:
            *len += sprintf(target_str, "compute_lib error: occured at GL library, see inst->error_queue!\r\n");
            break;
        default:
            *len += sprintf(target_str, "compute_lib error: undefined error (%d)!\r\n", err_code);
            break;
    }
}

bool compute_lib_gl_error_occured(void)
{
    bool res = false;
    while (glGetError() != GL_NO_ERROR) {
        res = true;
    }
	return res;
}

bool compute_lib_program_init(compute_lib_program_t* program)
{

    program->shader_handle = glCreateShader(GL_COMPUTE_SHADER);
    if (glGetError() != GL_NO_ERROR) {
        return compute_lib_program_destroy(program, false);
    }

    glShaderSource(program->shader_handle, 1, (const char* const *) &(program->source), NULL);
    if (glGetError() != GL_NO_ERROR) {
        return compute_lib_program_destroy(program, false);
    }

    GLsizei len;
    GLint is_compiled;
    glCompileShader(program->shader_handle);
    glGetShaderiv(program->shader_handle, GL_COMPILE_STATUS, &is_compiled);
    if (glGetError() != GL_NO_ERROR || is_compiled != GL_TRUE) {
        return compute_lib_program_destroy(program, false) | !is_compiled;
    }

    program->handle = glCreateProgram();
    if (program->handle == 0) {
        return compute_lib_program_destroy(program, false);
    }

    glAttachShader(program->handle, program->shader_handle);
    if (glGetError() != GL_NO_ERROR) {
        return compute_lib_program_destroy(program, false);
    }

    GLint is_linked;
    glLinkProgram(program->handle);
    glGetProgramiv(program->handle, GL_LINK_STATUS, &is_linked);
    if (glGetError() != GL_NO_ERROR || is_linked != GL_TRUE) {
        return compute_lib_program_destroy(program, false) | !is_linked;
    }
    
    if (glGetError() != GL_NO_ERROR) {
        return compute_lib_program_destroy(program, false);
    }

    return compute_lib_gl_error_occured();
}

char* compute_lib_program_get_log(compute_lib_program_t* program, int* output_len) 
{
    if (program->handle == 0) return NULL;
    glGetProgramiv(program->handle, GL_INFO_LOG_LENGTH, output_len);
    *output_len += 1;
    char* output_str = (char*) calloc(*output_len, sizeof(char));
    glGetProgramInfoLog(program->handle, *output_len, NULL, output_str);
    return output_str;
}

char* compute_lib_program_get_shader_log(compute_lib_program_t* program, int* output_len)
{
    if (program->shader_handle == 0) return NULL;
    glGetShaderiv(program->shader_handle, GL_INFO_LOG_LENGTH, output_len);
    *output_len += 1;
    char* output_str = (char*) calloc(*output_len, sizeof(char));
    glGetShaderInfoLog(program->shader_handle, *output_len, NULL, output_str);
    return output_str;
}

bool compute_lib_program_dispatch(compute_lib_program_t* program, GLuint num_groups_x, GLuint num_groups_y, GLuint num_groups_z)
{
    glUseProgram(program->handle);
    glDispatchCompute(num_groups_x, num_groups_y, num_groups_z);
    glMemoryBarrier(GL_ALL_BARRIER_BITS);
    glUseProgram(0);
    return compute_lib_gl_error_occured();
}

bool compute_lib_program_print_resources(compute_lib_program_t* program)
{
    GLint i;
    GLint count;

    GLint size;
    GLenum type;
    GLint name_max_len = 0;
    GLsizei name_len;
    GLint index, location;

    GLenum binding_prop = GL_BUFFER_BINDING, location_prop = GL_LOCATION;
	GLuint binding;

    // get maximum name length
    glGetProgramiv(program->handle, GL_ACTIVE_ATTRIBUTE_MAX_LENGTH, &name_len);
    if (name_len > name_max_len) name_max_len = name_len;

    glGetProgramiv(program->handle, GL_ACTIVE_UNIFORM_MAX_LENGTH, &name_len);
    if (name_len > name_max_len) name_max_len = name_len;

    glGetProgramInterfaceiv(program->handle, GL_BUFFER_VARIABLE, GL_MAX_NAME_LENGTH, &name_len);
    if (name_len > name_max_len) name_max_len = name_len;

    glGetProgramInterfaceiv(program->handle, GL_SHADER_STORAGE_BLOCK, GL_MAX_NAME_LENGTH, &name_len);
    if (name_len > name_max_len) name_max_len = name_len;

    GLchar name[name_max_len];

    // get active attributes
    glGetProgramiv(program->handle, GL_ACTIVE_ATTRIBUTES, &count);
    printf("Active Attributes: %d\n", count);

    for (i = 0; i < count; i++) {
        glGetActiveAttrib(program->handle, (GLuint)i, name_max_len, &name_len, &size, &type, name);

        printf("Attribute #%d Type: %s (0x%04X) Name: %s\n", i, gl32_get_define_name(type), type, name);
    }

    // get active uniforms
    glGetProgramiv(program->handle, GL_ACTIVE_UNIFORMS, &count);
    printf("Active Uniforms: %d\n", count);

    for (i = 0; i < count; i++) {
        glGetActiveUniform(program->handle, (GLuint)i, name_max_len, &name_len, &size, &type, name);
        index = glGetProgramResourceIndex(program->handle, GL_UNIFORM, name);
		location = glGetUniformLocation(program->handle, name);
		if (type == GL_UNSIGNED_INT_ATOMIC_COUNTER) {
			location_prop = GL_ATOMIC_COUNTER_BUFFER_INDEX;
        	glGetProgramResourceiv(program->handle, GL_UNIFORM, index, 1, &location_prop, sizeof(location), NULL, &location);
		}
        printf("Uniform #%d Type: %s (0x%04X) Name: %s Index: %d Location: %d Size: %d\n", i, gl32_get_define_name(type), type, name, index, location, size);
    }

    // get active SSBOs
    glGetProgramInterfaceiv(program->handle, GL_SHADER_STORAGE_BLOCK, GL_ACTIVE_RESOURCES, &count);
    printf("Active SSBOs: %d\n", count);

    for (i = 0; i < count; i++) {
        glGetProgramResourceName(program->handle, GL_SHADER_STORAGE_BLOCK, i, name_max_len, &name_len, name);
        index = glGetProgramResourceIndex(program->handle, GL_SHADER_STORAGE_BLOCK, name);
        glGetProgramResourceiv(program->handle, GL_SHADER_STORAGE_BLOCK, index, 1, &binding_prop, sizeof(binding), NULL, &binding);
        printf("SSBO #%d: Index: %d Name: %.*s Binding: %d\r\n", i, index, name_len, name, binding);
    }

    return compute_lib_gl_error_occured();
}

bool compute_lib_program_destroy(compute_lib_program_t* program, bool free_source) 
{
    if (free_source) {
        free(program->source);
    }

    if (program->shader_handle != 0) {
        glDeleteShader(program->shader_handle);
    }
    program->shader_handle = 0;

    if (program->handle != 0) {
        glDeleteProgram(program->handle);
    }
    program->handle = 0;

    return compute_lib_gl_error_occured();
}


bool compute_lib_framebuffer_init(compute_lib_program_t* program, compute_lib_framebuffer_t* framebuffer)
{
    glGenFramebuffers(1, &(framebuffer->handle));
    return compute_lib_gl_error_occured();
}

bool compute_lib_framebuffer_destroy(compute_lib_program_t* program, compute_lib_framebuffer_t* framebuffer)
{
    if (framebuffer != NULL) {
        glDeleteFramebuffers(1, &(framebuffer->handle));
    }
    return compute_lib_gl_error_occured();
}


bool compute_lib_image2d_init(compute_lib_program_t* program, compute_lib_image2d_t* image2d, GLenum framebuffer_attachment)
{
    
    image2d->location = glGetUniformLocation(program->handle, image2d->uniform_name);
    glGenTextures(1, &(image2d->handle));
    glActiveTexture(image2d->texture);
    glBindTexture(GL_TEXTURE_2D, image2d->handle);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, image2d->texture_wrap);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, image2d->texture_wrap);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, image2d->texture_filter);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, image2d->texture_filter);
    glTexStorage2D(GL_TEXTURE_2D, 1, image2d->internal_format, image2d->width, image2d->height);
    glBindImageTexture(image2d->location, image2d->handle, 0, GL_FALSE, 0, image2d->access, image2d->internal_format);
    size_t type_size = gl32_get_type_size(image2d->type);
    uint32_t num_ch = gl32_get_image_format_num_components(image2d->format);
    image2d->px_size = type_size * num_ch;
    image2d->data_size = image2d->px_size * image2d->width * image2d->height;
    if (framebuffer_attachment != 0) {
        image2d->framebuffer = (compute_lib_framebuffer_t*) malloc(sizeof(compute_lib_framebuffer_t));
        image2d->framebuffer->attachment = framebuffer_attachment;
        compute_lib_framebuffer_init(program, image2d->framebuffer);
        glBindFramebuffer(GL_FRAMEBUFFER, image2d->framebuffer->handle);
        glFramebufferTexture2D(GL_FRAMEBUFFER, framebuffer_attachment, GL_TEXTURE_2D, image2d->handle, 0);
    } else {
        image2d->framebuffer = NULL;
    }
    
    return compute_lib_gl_error_occured();
}

bool compute_lib_image2d_destroy(compute_lib_program_t* program, compute_lib_image2d_t* image2d)
{
    compute_lib_framebuffer_destroy(program, image2d->framebuffer);
    free(image2d->framebuffer);
    glDeleteTextures(1, &(image2d->handle));
    
    return compute_lib_gl_error_occured();
}

void* compute_lib_image2d_alloc(compute_lib_image2d_t* image2d) 
{
    return malloc(image2d->data_size);
}

bool compute_lib_image2d_reset(compute_lib_program_t* program, compute_lib_image2d_t* image2d, void* px_data)
{
    int i;
    void* image_data = compute_lib_image2d_alloc(image2d);
    for (i = 0; i < image2d->data_size; i += image2d->px_size) {
        memcpy(image_data + i, px_data, image2d->px_size);
    }
    glBindTexture(GL_TEXTURE_2D, image2d->handle);
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, image2d->width, image2d->height, image2d->format, image2d->type, image_data);
    free(image_data);
    
    return compute_lib_gl_error_occured();
}

bool compute_lib_image2d_reset_patch(compute_lib_program_t* program, compute_lib_image2d_t* image2d, void* px_data, int x_min, int x_max, int y_min, int y_max)
{
    int patch_width = x_max - x_min;
    int patch_height = y_max - y_min;
    int x, y;
    void* image_data = compute_lib_image2d_alloc(image2d);
    for (y = 0; y < patch_height; y++) {
        for (x = 0; x < patch_width; x++) {
            memcpy(image_data + (image2d->px_size * ((y_min + y) * image2d->width + x_min)) + x*image2d->px_size, px_data, image2d->px_size);
        }
    }
    glBindTexture(GL_TEXTURE_2D, image2d->handle);
    glTexSubImage2D(GL_TEXTURE_2D, 0, x_min, y_min, patch_width, patch_height, image2d->format, image2d->type, image_data);
    free(image_data);
    
    return compute_lib_gl_error_occured();
}

bool compute_lib_image2d_write(compute_lib_program_t* program, compute_lib_image2d_t* image2d, void* image_data)
{
    glBindTexture(GL_TEXTURE_2D, image2d->handle);
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, image2d->width, image2d->height, image2d->format, image2d->type, image_data);
    
    return compute_lib_gl_error_occured();
}

bool compute_lib_image2d_read(compute_lib_program_t* program, compute_lib_image2d_t* image2d, void* image_data)
{
    if (image2d->framebuffer != NULL) {
        
        glBindTexture(GL_TEXTURE_2D, image2d->handle);
        glBindFramebuffer(GL_FRAMEBUFFER, image2d->framebuffer->handle);
        glFramebufferTexture2D(GL_FRAMEBUFFER, image2d->framebuffer->attachment, GL_TEXTURE_2D, image2d->handle, 0);
        glReadPixels(0, 0, image2d->width, image2d->height, image2d->format, image2d->type, image_data);
    }
    return compute_lib_gl_error_occured();
}

bool compute_lib_image2d_read_patch(compute_lib_program_t* program, compute_lib_image2d_t* image2d, void* image_data, int x_min, int x_max, int y_min, int y_max, bool render)
{
    if (image2d->framebuffer != NULL) {
        glBindTexture(GL_TEXTURE_2D, image2d->handle);
        glBindFramebuffer(GL_FRAMEBUFFER, image2d->framebuffer->handle);
        if (render) {
            glFramebufferTexture2D(GL_FRAMEBUFFER, image2d->framebuffer->attachment, GL_TEXTURE_2D, image2d->handle, 0);
        }
        int patch_width = x_max - x_min;
        int patch_height = y_max - y_min;
        int y;
        void* tmp = malloc(image2d->px_size * patch_width * patch_height);
        glReadPixels(x_min, y_min, patch_width, patch_height, image2d->format, image2d->type, tmp);
        for (y = 0; y < patch_height; y++) {
            memcpy(image_data + (image2d->px_size * ((y_min + y) * image2d->width + x_min)), tmp + (image2d->px_size * (y * patch_width)), image2d->px_size * patch_width);
        }
        free(tmp);
    }
    return compute_lib_gl_error_occured();
}

bool compute_lib_acbo_init(compute_lib_program_t* program, compute_lib_acbo_t* acb, void* data, int len)
{
     
    glGenBuffers(1, &(acb->handle));
    acb->index = glGetProgramResourceIndex(program->handle, GL_UNIFORM, acb->name);
    if (len > 0) compute_lib_acbo_write(program, acb, data, len);
    
    return compute_lib_gl_error_occured();
}

bool compute_lib_acbo_destroy(compute_lib_program_t* program, compute_lib_acbo_t* acb)
{
     
    glDeleteBuffers(1, &(acb->handle));
    
    return compute_lib_gl_error_occured();
}

bool compute_lib_acbo_write(compute_lib_program_t* program, compute_lib_acbo_t* acb, void* data, int len)
{
     
    glBindBuffer(GL_ATOMIC_COUNTER_BUFFER, acb->handle);
    glBufferData(GL_ATOMIC_COUNTER_BUFFER, gl32_get_type_size(acb->type)*len, data, acb->usage);
    glBindBufferBase(GL_ATOMIC_COUNTER_BUFFER, acb->index, acb->handle);
    //glBufferSubData(GL_ATOMIC_COUNTER_BUFFER, 0, gl32_get_type_size(acb->type)*len, data);
    
    return compute_lib_gl_error_occured();
}

bool compute_lib_acbo_write_uint_val(compute_lib_program_t* program, compute_lib_acbo_t* acb, GLuint value)
{
    compute_lib_acbo_write(program, acb, (uint8_t*) &value, 1);
    return compute_lib_gl_error_occured();
}

bool compute_lib_acbo_read(compute_lib_program_t* program, compute_lib_acbo_t* acb, void* data, int len)
{
     
    glBindBuffer(GL_ATOMIC_COUNTER_BUFFER, acb->handle);
    GLsizei size = gl32_get_type_size(acb->type)*len;
    memcpy(data, glMapBufferRange(GL_ATOMIC_COUNTER_BUFFER, 0, size, GL_MAP_READ_BIT), size);
    glUnmapBuffer(GL_ATOMIC_COUNTER_BUFFER);
    
    return compute_lib_gl_error_occured();
}

bool compute_lib_acbo_read_uint_val(compute_lib_program_t* program, compute_lib_acbo_t* acb, GLuint* value)
{
    compute_lib_acbo_read(program, acb, (void*) value, 1);
    return compute_lib_gl_error_occured();
}


bool compute_lib_ssbo_init(compute_lib_program_t* program, compute_lib_ssbo_t* ssbo, void* data, int len)
{
    GLenum binding_prop = GL_BUFFER_BINDING;
     
    glGenBuffers(1, &(ssbo->handle));
    ssbo->index = glGetProgramResourceIndex(program->handle, GL_SHADER_STORAGE_BLOCK, ssbo->name);
    glGetProgramResourceiv(program->handle, GL_SHADER_STORAGE_BLOCK, ssbo->index, 1, &binding_prop, sizeof(ssbo->binding), NULL, &(ssbo->binding));
    if (len > 0) compute_lib_ssbo_write(program, ssbo, data, len);
    
    return compute_lib_gl_error_occured();
}

bool compute_lib_ssbo_destroy(compute_lib_program_t* program, compute_lib_ssbo_t* ssbo)
{
     
    glDeleteBuffers(1, &(ssbo->handle));
    
    return compute_lib_gl_error_occured();
}

bool compute_lib_ssbo_write(compute_lib_program_t* program, compute_lib_ssbo_t* ssbo, void* data, int len)
{
     
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, ssbo->handle);
    glBufferData(GL_SHADER_STORAGE_BUFFER, gl32_get_type_size(ssbo->type)*len, data, ssbo->usage);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, ssbo->binding, ssbo->handle);
    //glBufferSubData(GL_ATOMIC_COUNTER_BUFFER, 0, gl32_get_type_size(ssbo->type)*len, data);
    
    return compute_lib_gl_error_occured();
}

bool compute_lib_ssbo_read(compute_lib_program_t* program, compute_lib_ssbo_t* ssbo, void* data, int len)
{
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, ssbo->handle);
    int size = gl32_get_type_size(ssbo->type)*len;
    memcpy(data, glMapBufferRange(GL_SHADER_STORAGE_BUFFER, 0, size, GL_MAP_READ_BIT), size);
    glUnmapBuffer(GL_SHADER_STORAGE_BUFFER);
    
    return compute_lib_gl_error_occured();
}

bool compute_lib_uniform_init(compute_lib_program_t* program, compute_lib_uniform_t* uniform)
{
    glGetUniformIndices(program->handle, 1, (const GLchar **) &(uniform->name), &(uniform->index));
    uniform->location = glGetUniformLocation(program->handle, uniform->name);
    glGetActiveUniform(program->handle, uniform->index, 0, NULL, &(uniform->size), &(uniform->type), NULL);
    return compute_lib_gl_error_occured();
}

bool compute_lib_uniform_write(compute_lib_program_t* program, compute_lib_uniform_t* uniform, void* data)
{
    glUseProgram(program->handle);
    switch (uniform->type) {
        case GL_FLOAT:
            glUniform1fv(uniform->location, uniform->size, (const GLfloat*) data);
            break;
        case GL_UNSIGNED_INT:
            glUniform1uiv(uniform->location, uniform->size, (const GLuint*) data);
            break;
        case GL_INT:
            glUniform1iv(uniform->location, uniform->size, (const GLint*) data);
            break;
        case GL_FLOAT_VEC2:
            glUniform2fv(uniform->location, uniform->size, (const GLfloat*) data);
            break;
        case GL_UNSIGNED_INT_VEC2:
            glUniform2uiv(uniform->location, uniform->size, (const GLuint*) data);
            break;
        case GL_INT_VEC2:
            glUniform2iv(uniform->location, uniform->size, (const GLint*) data);
            break;
        case GL_FLOAT_VEC3:
            glUniform3fv(uniform->location, uniform->size, (const GLfloat*) data);
            break;
        case GL_UNSIGNED_INT_VEC3:
            glUniform3uiv(uniform->location, uniform->size, (const GLuint*) data);
            break;
        case GL_INT_VEC3:
            glUniform3iv(uniform->location, uniform->size, (const GLint*) data);
            break;
        case GL_FLOAT_VEC4:
            glUniform4fv(uniform->location, uniform->size, (const GLfloat*) data);
            break;
        case GL_UNSIGNED_INT_VEC4:
            glUniform4uiv(uniform->location, uniform->size, (const GLuint*) data);
            break;
        case GL_INT_VEC4:
            glUniform4iv(uniform->location, uniform->size, (const GLint*) data);
            break;
        case GL_FLOAT_MAT2:
            glUniformMatrix2fv(uniform->location, uniform->size, false, (const GLfloat*) data);
            break;
        case GL_FLOAT_MAT3:
            glUniformMatrix3fv(uniform->location, uniform->size, false, (const GLfloat*) data);
            break;
        case GL_FLOAT_MAT4:
            glUniformMatrix4fv(uniform->location, uniform->size, false, (const GLfloat*) data);
            break;
        case GL_FLOAT_MAT2x3:
            glUniformMatrix2x3fv(uniform->location, uniform->size, false, (const GLfloat*) data);
            break;
        case GL_FLOAT_MAT3x2:
            glUniformMatrix3x2fv(uniform->location, uniform->size, false, (const GLfloat*) data);
            break;
        case GL_FLOAT_MAT2x4:
            glUniformMatrix2x4fv(uniform->location, uniform->size, false, (const GLfloat*) data);
            break;
        case GL_FLOAT_MAT4x2:
            glUniformMatrix4x2fv(uniform->location, uniform->size, false, (const GLfloat*) data);
            break;
        case GL_FLOAT_MAT3x4:
            glUniformMatrix3x4fv(uniform->location, uniform->size, false, (const GLfloat*) data);
            break;
        case GL_FLOAT_MAT4x3:
            glUniformMatrix4x3fv(uniform->location, uniform->size, false, (const GLfloat*) data);
            break;
    }
    glUseProgram(0);
    return compute_lib_gl_error_occured();
}



