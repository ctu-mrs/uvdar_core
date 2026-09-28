#include "uvdar_core/helpers/compute_shader.hpp"

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>

#include <filesystem>
#include <fstream>
#include <sstream>
#include <algorithm>

namespace uvdar_core::helpers::compute_shader {

namespace {

    /**
     * @brief Replace all occurrences of a token in shader source text.
     */
    void replaceAll(std::string& source, const std::string& token, const std::string& value)
    {
        std::size_t offset = 0;
        while ((offset = source.find(token, offset)) != std::string::npos) {
            source.replace(offset, token.size(), value);
            offset += value.size();
        }
    }

} // namespace

/**
 * @brief Push an error entry to the queue.
 */
void ErrorQueue::push(std::unique_ptr<Error> error)
{
    queue_.push(std::move(error));
}

/**
 * @brief Return number of currently queued errors.
 */
std::size_t ErrorQueue::size() const
{
    return queue_.size();
}

/**
 * @brief Pop and return one queued error, or `nullptr` when empty.
 */
std::unique_ptr<Error> ErrorQueue::pop()
{
    if (queue_.empty()) {
        return nullptr;
    }
    auto result = std::move(queue_.front());
    queue_.pop();
    return result;
}

/**
 * @brief Clear all queued errors.
 */
void ErrorQueue::clear()
{
    while (!queue_.empty()) {
        queue_.pop();
    }
}

/**
 * @brief Create compute context instance with chosen DRM path.
 */
Instance::Instance(const std::string& path)
    : dri_path(path)
{
}

/**
 * @brief Destroying instance releases all resources.
 */
Instance::~Instance()
{
    deinit();
}

/**
 * @brief Enumerate available render/card devices from `/dev/dri`.
 */
std::vector<std::string> Instance::render_devices()
{
    std::vector<std::string> devices;
    const std::filesystem::path dri_dir("/dev/dri");
    if (!std::filesystem::exists(dri_dir)) {
        return devices;
    }

    for (const auto& entry : std::filesystem::directory_iterator(dri_dir)) {
        const std::string name = entry.path().filename().string();
        if (name.rfind("renderD", 0) == 0) {
            devices.push_back(entry.path().string());
        }
    }
    for (const auto& entry : std::filesystem::directory_iterator(dri_dir)) {
        const std::string name = entry.path().filename().string();
        if (name.rfind("card", 0) == 0) {
            devices.push_back(entry.path().string());
        }
    }
    std::sort(devices.begin(), devices.end());
    return devices;
}

/**
 * @brief Return the first available render instance.
 */
Instance Instance::first_available()
{
    const auto devices = render_devices();
    if (devices.empty()) {
        throw std::runtime_error("Error: No render device available!");
    }
    return Instance(devices.front());
}

/**
 * @brief OpenGL debug callback for queueing GL errors.
 */
void Instance::gl_debug_callback(GLenum source, GLenum type, GLuint id, GLenum severity, GLsizei length, const GLchar* message, const void* user_param)
{
    auto* instance = static_cast<Instance*>(const_cast<void*>(user_param));
    if (instance == nullptr || !instance->error_queue) {
        return;
    }
    if (severity > instance->verbosity) {
        return;
    }

    const std::string msg(message, length > 0 ? static_cast<std::size_t>(length) : std::strlen(message));
    instance->error_queue->push(std::make_unique<Error>(instance->error_total_cnt, msg, length, source, type, id, severity));
    instance->error_total_cnt++;
}

/**
 * @brief Initialize EGL/GBM/OpenGL context.
 */
GLint Instance::init()
{
    if (initialised) {
        return -101;
    }

    fd = open(dri_path.c_str(), O_RDWR);
    if (fd <= 0) {
        std::fprintf(stderr, "compute_shader: open('%s') failed: %s\n", dri_path.c_str(), std::strerror(errno));
        deinit();
        return -102;
    }

    gbm = gbm_create_device(fd);
    if (gbm == nullptr) {
        deinit();
        return -103;
    }

    dpy = eglGetPlatformDisplay(EGL_PLATFORM_GBM_MESA, gbm, nullptr);
    if (dpy == EGL_NO_DISPLAY) {
        deinit();
        return -104;
    }

    if (!eglInitialize(dpy, nullptr, nullptr)) {
        deinit();
        return -105;
    }

    const char* extensions = eglQueryString(dpy, EGL_EXTENSIONS);
    if (extensions == nullptr || std::strstr(extensions, "EGL_KHR_create_context") == nullptr) {
        deinit();
        return -106;
    }
    if (std::strstr(extensions, "EGL_KHR_surfaceless_context") == nullptr) {
        deinit();
        return -107;
    }

    static const EGLint egl_config_attribs[] = {
        EGL_RENDERABLE_TYPE,
        EGL_OPENGL_ES3_BIT_KHR,
        EGL_NONE,
    };
    static const EGLint egl_ctx_attribs[] = {
        EGL_CONTEXT_CLIENT_VERSION,
        3,
        EGL_NONE,
    };

    EGLConfig egl_config;
    EGLint egl_count = 0;
    if (!eglChooseConfig(dpy, egl_config_attribs, &egl_config, 1, &egl_count)) {
        deinit();
        return -108;
    }
    if (!eglBindAPI(EGL_OPENGL_ES_API)) {
        deinit();
        return -109;
    }

    ctx = eglCreateContext(dpy, egl_config, EGL_NO_CONTEXT, egl_ctx_attribs);
    if (ctx == EGL_NO_CONTEXT) {
        deinit();
        return -110;
    }

    if (!eglMakeCurrent(dpy, EGL_NO_SURFACE, EGL_NO_SURFACE, ctx)) {
        deinit();
        return -111;
    }

    last_error.reset();
    error_total_cnt = 0;
    error_queue     = std::make_unique<ErrorQueue>();
    glEnable(GL_DEBUG_OUTPUT);
    glDebugMessageCallback(reinterpret_cast<GLDEBUGPROC>(Instance::gl_debug_callback), this);
    initialised = true;
    return 0;
}

/**
 * @brief Deinitialize resources and clear state.
 */
void Instance::deinit()
{
    if (ctx != EGL_NO_CONTEXT && dpy != EGL_NO_DISPLAY) {
        eglDestroyContext(dpy, ctx);
    }
    ctx = EGL_NO_CONTEXT;

    if (dpy != EGL_NO_DISPLAY) {
        eglTerminate(dpy);
    }
    dpy = EGL_NO_DISPLAY;

    if (gbm != nullptr) {
        gbm_device_destroy(gbm);
    }
    gbm = nullptr;

    if (fd > 0) {
        close(fd);
    }
    fd = 0;

    if (error_queue) {
        flush_errors(stderr);
        error_queue.reset();
    }
    initialised = false;
}

/**
 * @brief Flush queued GL errors to output.
 */
GLuint Instance::flush_errors(FILE* out)
{
    if (!error_queue) {
        return 0;
    }

    GLuint count = 0;
    while (error_queue->size() > 0) {
        auto error = error_queue->pop();
        if (out != nullptr && error != nullptr) {
            std::fprintf(
                out,
                "gpu: GL error #%u: %s (0x%X), severity: %s (0x%X), message = %s\n",
                error->err_id,
        uvdar_core::helpers::gles::get_define_name(error->type),
                error->type,
        uvdar_core::helpers::gles::get_define_name(error->severity),
                error->severity,
                error->message.c_str());
        }
        count++;
    }
    return count;
}

/**
 * @brief Print a readable error message for initialization error codes.
 */
void Instance::print_error(GLint err_code, FILE* out)
{
    switch (err_code) {
    case 0:
        std::fprintf(out, "compute_lib_init error: no error.\r\n");
        break;
    case -100:
        std::fprintf(out, "gpu error: occurs at: GLint compute_lib_init(compute_lib_instance_t* inst);\r\n");
        break;
    case -101:
        std::fprintf(out, "compute_lib_init error: already initialised!\r\n");
        break;
    case -102:
        std::fprintf(out, "compute_lib_init error: could not open GPU direct rendering infrastructure!\r\n");
        break;
    case -103:
        std::fprintf(out, "compute_lib_init: error: could not create GBM context!\r\n");
        break;
    case -104:
        std::fprintf(out, "compute_lib_init error: could not get platform display!\r\n");
        break;
    case -105:
        std::fprintf(out, "compute_lib_init error: could not initialise EGL!\r\n");
        break;
    case -106:
        std::fprintf(out, "compute_lib_init error: could not locate extension: EGL_KHR_create_context!\r\n");
        break;
    case -107:
        std::fprintf(out, "compute_lib_init error: could not locate extension: EGL_KHR_surfaceless_context!\r\n");
        break;
    case -108:
        std::fprintf(out, "compute_lib_init error: could not choose EGL configuration!\r\n");
        break;
    case -109:
        std::fprintf(out, "compute_lib_init error: could not bind EGL_OPENGL_ES_API!\r\n");
        break;
    case -110:
        std::fprintf(out, "compute_lib_init error: could not create EGL context!\r\n");
        break;
    case -111:
        std::fprintf(out, "compute_lib_init error: could not make current EGL context!\r\n");
        break;
    case 0x0500:
        std::fprintf(out, "gpu error: occured at GL library, see inst->error_queue!\r\n");
        break;
    default:
        std::fprintf(out, "gpu error: undefined error (%d)!\r\n", err_code);
        break;
    }
}

/**
 * @brief Return count of pending GL errors.
 */
GLuint Instance::gl_errors_count()
{
    GLuint count = 0;
    while (glGetError() != GL_NO_ERROR) {
        count++;
    }
    return count;
}

/**
 * @brief Compute optimal local work-group dimensions for image size.
 */
std::tuple<unsigned, unsigned, unsigned> Instance::get_local_sizes(unsigned image_width, unsigned image_height)
{
    (void)image_width;
    (void)image_height;
    GLint max_invocations  = 256;
    GLint max_local_size_x = 32;
    GLint max_local_size_y = 32;
    GLint max_local_size_z = 1;
    glGetIntegerv(GL_MAX_COMPUTE_WORK_GROUP_INVOCATIONS, &max_invocations);
    glGetIntegeri_v(GL_MAX_COMPUTE_WORK_GROUP_SIZE, 0, &max_local_size_x);
    glGetIntegeri_v(GL_MAX_COMPUTE_WORK_GROUP_SIZE, 1, &max_local_size_y);
    glGetIntegeri_v(GL_MAX_COMPUTE_WORK_GROUP_SIZE, 2, &max_local_size_z);

    // A balanced two-dimensional group keeps neighboring image accesses in
    // the same group and avoids the poorly occupied 128x2 shape that results
    // from selecting dimensions only for exact divisibility. Dispatch already
    // rounds up and the shaders bounds-check edge invocations.
    const unsigned local_x = std::max(
        1U,
        std::min(16U, static_cast<unsigned>(max_local_size_x)));
    const unsigned local_y = std::max(
        1U,
        std::min({
            16U,
            static_cast<unsigned>(max_local_size_y),
            static_cast<unsigned>(max_invocations) / local_x}));
    return { std::max(1u, local_x), std::max(1u, local_y), 1u };
}

/**
 * @brief Create context wrapper around an `Instance`.
 */
Context::Context()
    : instance_(std::make_unique<Instance>())
{
}

/**
 * @brief Destroy context and release resources.
 */
Context::~Context()
{
    deinit();
}

/**
 * @brief Initialize first available render device.
 */
bool Context::initFirstAvailable()
{
    const auto devices = Instance::render_devices();
    if (devices.empty()) {
        std::fprintf(stderr, "compute_shader: no DRM render/card devices found in /dev/dri.\n");
        return false;
    }

    for (const auto& device : devices) {
        std::fprintf(stderr, "compute_shader: trying render device '%s'\n", device.c_str());
        instance_->deinit();
        instance_->dri_path = device;
        const GLint init_error = instance_->init();
        if (init_error == 0) {
            return true;
        }
        std::fprintf(stderr, "compute_shader: render device '%s' init failed with code %d.\n", device.c_str(), init_error);
        Instance::print_error(init_error, stderr);
        if (instance_->error_queue) {
            instance_->flush_errors(stderr);
        }
    }
    std::fprintf(stderr, "compute_shader: no render device could be initialized.\n");
    return false;
}

/**
 * @brief Make context current on caller thread.
 */
bool Context::makeCurrent()
{
    if (!isInitialized()) {
        return false;
    }
    if (eglGetCurrentContext() == instance_->ctx) {
        return true;
    }
    return eglMakeCurrent(instance_->dpy, EGL_NO_SURFACE, EGL_NO_SURFACE, instance_->ctx) == EGL_TRUE;
}

/**
 * @brief Release current context binding from caller thread.
 */
bool Context::releaseCurrent()
{
    if (!isInitialized()) {
        return false;
    }
    if (eglGetCurrentContext() != instance_->ctx) {
        return true;
    }
    return eglMakeCurrent(instance_->dpy, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT) == EGL_TRUE;
}

/**
 * @brief Deinitialize owned instance.
 */
void Context::deinit()
{
    if (instance_) {
        instance_->deinit();
    }
}

/**
 * @brief Check whether context has been initialized.
 */
bool Context::isInitialized() const
{
    return instance_ && instance_->initialised;
}

/**
 * @brief Get local work-group sizes via context.
 */
std::tuple<unsigned, unsigned, unsigned> Context::getLocalSizes(unsigned image_width, unsigned image_height) const
{
    return Instance::get_local_sizes(image_width, image_height);
}

/**
 * @brief Access mutable underlying instance.
 */
Instance& Context::instance()
{
    return *instance_;
}

/**
 * @brief Access const underlying instance.
 */
const Instance& Context::instance() const
{
    return *instance_;
}

/**
 * @brief Base resource descriptor constructor.
 */
Resource::Resource(const std::string& name_, GLuint type_)
    : name(name_)
    , type(type_)
    , value(-1)
{
}

/**
 * @brief Create framebuffer binding target.
 */
Framebuffer::Framebuffer(GLenum attachment_)
    : attachment(attachment_)
    , handle(0)
{
}

/**
 * @brief Destroy framebuffer resources.
 */
Framebuffer::~Framebuffer()
{
    destroy();
}

/**
 * @brief Allocate framebuffer handle.
 */
GLuint Framebuffer::init()
{
    glGenFramebuffers(1, &handle);
    return Instance::gl_errors_count();
}

/**
 * @brief Delete framebuffer handle.
 */
GLuint Framebuffer::destroy()
{
    if (handle != 0) {
        glDeleteFramebuffers(1, &handle);
    }
    handle = 0;
    return Instance::gl_errors_count();
}

/**
 * @brief Construct 2D image resource descriptor.
 */
Image2D::Image2D(const std::string& name, GLenum texture_, GLsizei width_, GLsizei height_, GLenum access_, GLuint num_components_, GLenum type_)
    : resource(name, GL_IMAGE_2D)
    , texture(texture_)
    , width(width_)
    , height(height_)
    , access(access_)
    , texture_wrap(GL_CLAMP_TO_EDGE)
    , texture_filter(GL_LINEAR)
    , type(type_)
    , num_components(num_components_)
    , format(0)
    , internal_format(0)
    , compatibility_format(0)
    , handle(0)
    , data_size(0)
    , px_size(0)
    , framebuffer(0)
{
}

/**
 * @brief Destroy image resource on teardown.
 */
Image2D::~Image2D()
{
    destroy();
}

/**
 * @brief Resolve OpenGL image formats for this type/component combination.
 */
void Image2D::setup_format()
{
    switch (type) {
    case GL_UNSIGNED_BYTE:
        switch (num_components) {
        case 1:
            internal_format = GL_R8UI;
            format          = GL_RED_INTEGER;
            break;
        case 2:
            internal_format = GL_RG8UI;
            format          = GL_RG_INTEGER;
            break;
        case 3:
            internal_format = GL_RGB8UI;
            format          = GL_RGB_INTEGER;
            break;
        case 4:
            internal_format = GL_RGBA8UI;
            format          = GL_RGBA_INTEGER;
            break;
        default:
            break;
        }
        break;
    case GL_BYTE:
        switch (num_components) {
        case 1:
            internal_format = GL_R8I;
            format          = GL_RED_INTEGER;
            break;
        case 2:
            internal_format = GL_RG8I;
            format          = GL_RG_INTEGER;
            break;
        case 3:
            internal_format = GL_RGB8I;
            format          = GL_RGB_INTEGER;
            break;
        case 4:
            internal_format = GL_RGBA8I;
            format          = GL_RGBA_INTEGER;
            break;
        default:
            break;
        }
        break;
    case GL_UNSIGNED_SHORT:
        switch (num_components) {
        case 1:
            internal_format = GL_R16UI;
            format          = GL_RED_INTEGER;
            break;
        case 2:
            internal_format = GL_RG16UI;
            format          = GL_RG_INTEGER;
            break;
        case 3:
            internal_format = GL_RGB16UI;
            format          = GL_RGB_INTEGER;
            break;
        case 4:
            internal_format = GL_RGBA16UI;
            format          = GL_RGBA_INTEGER;
            break;
        default:
            break;
        }
        break;
    case GL_SHORT:
        switch (num_components) {
        case 1:
            internal_format = GL_R16I;
            format          = GL_RED_INTEGER;
            break;
        case 2:
            internal_format = GL_RG16I;
            format          = GL_RG_INTEGER;
            break;
        case 3:
            internal_format = GL_RGB16I;
            format          = GL_RGB_INTEGER;
            break;
        case 4:
            internal_format = GL_RGBA16I;
            format          = GL_RGBA_INTEGER;
            break;
        default:
            break;
        }
        break;
    case GL_UNSIGNED_INT:
        switch (num_components) {
        case 1:
            internal_format = GL_R32UI;
            format          = GL_RED_INTEGER;
            break;
        case 2:
            internal_format = GL_RG32UI;
            format          = GL_RG_INTEGER;
            break;
        case 3:
            internal_format = GL_RGB32UI;
            format          = GL_RGB_INTEGER;
            break;
        case 4:
            internal_format = GL_RGBA32UI;
            format          = GL_RGBA_INTEGER;
            break;
        default:
            break;
        }
        break;
    case GL_INT:
        switch (num_components) {
        case 1:
            internal_format = GL_R32I;
            format          = GL_RED_INTEGER;
            break;
        case 2:
            internal_format = GL_RG32I;
            format          = GL_RG_INTEGER;
            break;
        case 3:
            internal_format = GL_RGB32I;
            format          = GL_RGB_INTEGER;
            break;
        case 4:
            internal_format = GL_RGBA32I;
            format          = GL_RGBA_INTEGER;
            break;
        default:
            break;
        }
        break;
    case GL_HALF_FLOAT:
        switch (num_components) {
        case 1:
            internal_format = GL_R16F;
            format          = GL_RED;
            break;
        case 2:
            internal_format = GL_RG16F;
            format          = GL_RG;
            break;
        case 3:
            internal_format = GL_RGB16F;
            format          = GL_RGB;
            break;
        case 4:
            internal_format = GL_RGBA16F;
            format          = GL_RGBA;
            break;
        default:
            break;
        }
        break;
    case GL_FLOAT:
        switch (num_components) {
        case 1:
            internal_format = GL_R32F;
            format          = GL_RED;
            break;
        case 2:
            internal_format = GL_RG32F;
            format          = GL_RG;
            break;
        case 3:
            internal_format = GL_RGB32F;
            format          = GL_RGB;
            break;
        case 4:
            internal_format = GL_RGBA32F;
            format          = GL_RGBA;
            break;
        default:
            break;
        }
        break;
    default:
        break;
    }
    compatibility_format = uvdar_core::helpers::gles::get_image2d_compatibility_format(internal_format);
}

/**
 * @brief Allocate texture storage and optional framebuffer attachment.
 */
GLuint Image2D::init(GLenum framebuffer_attachment)
{
    glGenTextures(1, &handle);
    glActiveTexture(texture);
    glBindTexture(GL_TEXTURE_2D, handle);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, texture_wrap);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, texture_wrap);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, texture_filter);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, texture_filter);
    glTexStorage2D(GL_TEXTURE_2D, 1, internal_format, width, height);
    glBindImageTexture(resource.value, handle, 0, GL_FALSE, 0, access, compatibility_format);
    const std::size_t type_size = uvdar_core::helpers::gles::get_type_size(type);
    px_size                     = static_cast<GLuint>(type_size * num_components);
    data_size                   = px_size * width * height;
    if (framebuffer_attachment != 0) {
        framebuffer.attachment = framebuffer_attachment;
        framebuffer.init();
        glBindFramebuffer(GL_FRAMEBUFFER, framebuffer.handle);
        glFramebufferTexture2D(GL_FRAMEBUFFER, framebuffer_attachment, GL_TEXTURE_2D, handle, 0);
    }
    return Instance::gl_errors_count();
}

/**
 * @brief Build GLSL image layout qualifier string.
 */
std::string Image2D::glsl_layout()
{
    char* str        = nullptr;
    const int result = asprintf(
        &str,
        "layout(%s, binding=%d) %s uniform highp %s %s",
        uvdar_core::helpers::gles::get_glsl_image2d_format_qualifier(compatibility_format),
        resource.value,
        uvdar_core::helpers::gles::get_glsl_image2d_access(access),
        uvdar_core::helpers::gles::get_glsl_image2d_type(compatibility_format),
        resource.name.c_str());
    std::string out = (result < 0 || str == nullptr) ? std::string { } : std::string(str);
    if (str != nullptr) {
        std::free(str);
    }
    return out;
}

/**
 * @brief Destroy image texture and framebuffer linkage.
 */
GLuint Image2D::destroy()
{
    framebuffer.destroy();
    if (handle != 0) {
        glDeleteTextures(1, &handle);
    }
    handle = 0;
    return Instance::gl_errors_count();
}

/**
 * @brief Replace all pixels with provided value.
 */
GLuint Image2D::reset(const void* px_data)
{
    void* image_data = std::malloc(data_size);
    for (GLuint index = 0; index < data_size; index += px_size) {
        std::memcpy(static_cast<char*>(image_data) + index, px_data, px_size);
    }
    glBindTexture(GL_TEXTURE_2D, handle);
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, width, height, format, type, image_data);
    std::free(image_data);
    return Instance::gl_errors_count();
}

/**
 * @brief Reset rectangular patch with provided value.
 */
GLuint Image2D::reset_patch(const void* px_data, GLint x_min, GLint x_max, GLint y_min, GLint y_max)
{
    const GLint patch_width  = x_max - x_min;
    const GLint patch_height = y_max - y_min;
    void* image_data         = std::malloc(px_size * patch_width * patch_height);
    for (GLint y = 0; y < patch_height; ++y) {
        for (GLint x = 0; x < patch_width; ++x) {
            std::memcpy(static_cast<char*>(image_data) + (px_size * (y * patch_width + x)), px_data, px_size);
        }
    }
    glBindTexture(GL_TEXTURE_2D, handle);
    glTexSubImage2D(GL_TEXTURE_2D, 0, x_min, y_min, patch_width, patch_height, format, type, image_data);
    std::free(image_data);
    return Instance::gl_errors_count();
}

/**
 * @brief Upload full texture contents.
 */
GLuint Image2D::write(const void* image_data)
{
    glBindTexture(GL_TEXTURE_2D, handle);
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, width, height, format, type, image_data);
    return Instance::gl_errors_count();
}

/**
 * @brief Read back full image content.
 */
GLuint Image2D::read(void* image_data)
{
    if (framebuffer.handle != 0) {
        glBindTexture(GL_TEXTURE_2D, handle);
        glBindFramebuffer(GL_FRAMEBUFFER, framebuffer.handle);
        glFramebufferTexture2D(GL_FRAMEBUFFER, framebuffer.attachment, GL_TEXTURE_2D, handle, 0);
        glReadPixels(0, 0, width, height, format, type, image_data);
    }
    return Instance::gl_errors_count();
}

/**
 * @brief Read back a rectangular patch.
 */
GLuint Image2D::read_patch(void* image_data, GLint x_min, GLint x_max, GLint y_min, GLint y_max, bool render)
{
    if (framebuffer.handle != 0) {
        glBindTexture(GL_TEXTURE_2D, handle);
        if (render) {
            glBindFramebuffer(GL_FRAMEBUFFER, framebuffer.handle);
            glFramebufferTexture2D(GL_FRAMEBUFFER, framebuffer.attachment, GL_TEXTURE_2D, handle, 0);
        }
        const GLint patch_width  = x_max - x_min;
        const GLint patch_height = y_max - y_min;
        void* tmp                = std::malloc(px_size * patch_width * patch_height);
        glReadPixels(x_min, y_min, patch_width, patch_height, format, type, tmp);
        for (GLint y = 0; y < patch_height; ++y) {
            std::memcpy(
                static_cast<char*>(image_data) + (px_size * ((y_min + y) * width + x_min)),
                static_cast<char*>(tmp) + (px_size * (y * patch_width)),
                px_size * patch_width);
        }
        std::free(tmp);
    }
    return Instance::gl_errors_count();
}

/**
 * @brief Construct atomic counter buffer object.
 */
ACBO::ACBO(const std::string& name, GLenum type_, GLenum usage_)
    : resource(name, GL_ATOMIC_COUNTER_BUFFER)
    , type(type_)
    , usage(usage_)
    , handle(0)
{
}

/**
 * @brief Destroy ACBO resources.
 */
ACBO::~ACBO()
{
    destroy();
}

/**
 * @brief Allocate and optionally initialize ACBO.
 */
GLuint ACBO::init(const void* data, GLint len)
{
    glGenBuffers(1, &handle);
    if (len > 0) {
        write(data, len);
    }
    return Instance::gl_errors_count();
}

/**
 * @brief Deallocate ACBO storage.
 */
GLuint ACBO::destroy()
{
    if (handle != 0) {
        glDeleteBuffers(1, &handle);
    }
    handle = 0;
    return Instance::gl_errors_count();
}

/**
 * @brief Upload ACBO data.
 */
GLuint ACBO::write(const void* data, GLint len)
{
    glBindBuffer(GL_ATOMIC_COUNTER_BUFFER, handle);
    glBufferData(GL_ATOMIC_COUNTER_BUFFER, uvdar_core::helpers::gles::get_type_size(type) * len, data, usage);
    glBindBufferBase(GL_ATOMIC_COUNTER_BUFFER, resource.value, handle);
    return Instance::gl_errors_count();
}

/**
 * @brief Upload single unsigned integer value.
 */
GLuint ACBO::write_uint_val(GLuint value)
{
    return write(&value, 1);
}

/**
 * @brief Read ACBO data to host memory.
 */
GLuint ACBO::read(void* data, GLint len)
{
    glBindBuffer(GL_ATOMIC_COUNTER_BUFFER, handle);
    const GLsizei size = uvdar_core::helpers::gles::get_type_size(type) * len;
    const void* ptr = glMapBufferRange(GL_ATOMIC_COUNTER_BUFFER, 0, size, GL_MAP_READ_BIT);
    if (ptr == nullptr) {
        std::fprintf(stderr, "Failed to map ACBO '%s'!\n", resource.name.c_str());
        return Instance::gl_errors_count();
    }
    std::memcpy(data, ptr, size);
    glUnmapBuffer(GL_ATOMIC_COUNTER_BUFFER);
    return Instance::gl_errors_count();
}

/**
 * @brief Read one unsigned integer value from ACBO.
 */
GLuint ACBO::read_uint_val(GLuint* value)
{
    return read(value, 1);
}

/**
 * @brief Construct SSBO descriptor.
 */
SSBO::SSBO(const std::string& name, GLenum type_, GLenum usage_)
    : resource(name, GL_SHADER_STORAGE_BUFFER)
    , type(type_)
    , usage(usage_)
    , handle(0)
{
}

/**
 * @brief Destroy SSBO resources.
 */
SSBO::~SSBO()
{
    destroy();
}

/**
 * @brief Allocate and optionally initialize SSBO.
 */
GLuint SSBO::init(const void* data, GLint len)
{
    glGenBuffers(1, &handle);
    if (len > 0) {
        write(data, len);
    }
    return Instance::gl_errors_count();
}

/**
 * @brief Deallocate SSBO storage.
 */
GLuint SSBO::destroy()
{
    if (handle != 0) {
        glDeleteBuffers(1, &handle);
    }
    handle = 0;
    return Instance::gl_errors_count();
}

/**
 * @brief Generate SSBO GLSL buffer declaration.
 */
std::string SSBO::glsl_layout()
{
    char* str        = nullptr;
    const int result = asprintf(
        &str,
        "layout(std430, binding=%d) buffer %s { %s %s_data[]; }",
        resource.value,
        resource.name.c_str(),
        uvdar_core::helpers::gles::get_glsl_data_type(type),
        resource.name.c_str());
    std::string out = (result < 0 || str == nullptr) ? std::string { } : std::string(str);
    if (str != nullptr) {
        std::free(str);
    }
    return out;
}

/**
 * @brief Write data into SSBO.
 */
GLuint SSBO::write(const void* data, GLint len)
{
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, handle);
    glBufferData(GL_SHADER_STORAGE_BUFFER, uvdar_core::helpers::gles::get_type_size(type) * len, data, usage);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, resource.value, handle);
    return Instance::gl_errors_count();
}

/**
 * @brief Read SSBO data into host memory.
 */
GLuint SSBO::read(void* data, GLint len)
{
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, handle);
    const GLint size = uvdar_core::helpers::gles::get_type_size(type) * len;
    const void* ptr = glMapBufferRange(GL_SHADER_STORAGE_BUFFER, 0, size, GL_MAP_READ_BIT);
    if (ptr == nullptr) {
        std::fprintf(stderr, "Failed to map SSBO '%s'!\n", resource.name.c_str());
        return Instance::gl_errors_count();
    }
    std::memcpy(data, ptr, size);
    glUnmapBuffer(GL_SHADER_STORAGE_BUFFER);
    return Instance::gl_errors_count();
}

/**
 * @brief Uniform handle descriptor constructor.
 */
Uniform::Uniform(const std::string& name_)
    : name(name_)
    , location(0)
    , size(0)
    , type(0)
    , index(0)
{
}

/**
 * @brief Destructor tears down program resources.
 */
Program::~Program()
{
    destroy(false);
}

/**
 * @brief Compile and link compute shader source.
 */
bool Program::init()
{
    const char* src_ptr = source.c_str();
    shader_handle       = glCreateShader(GL_COMPUTE_SHADER);
    GLuint errors_count = glGetError();
    if (errors_count != GL_NO_ERROR) {
        destroy(false);
        return false;
    }

    glShaderSource(shader_handle, 1, &src_ptr, nullptr);
    errors_count = glGetError();
    if (errors_count != GL_NO_ERROR) {
        destroy(false);
        return false;
    }

    glCompileShader(shader_handle);
    GLint is_compiled = GL_FALSE;
    glGetShaderiv(shader_handle, GL_COMPILE_STATUS, &is_compiled);
    errors_count = glGetError();
    if (errors_count != GL_NO_ERROR || is_compiled != GL_TRUE) {
        destroy(false);
        return false;
    }

    handle = glCreateProgram();
    if (handle == 0) {
        destroy(false);
        return false;
    }
    glAttachShader(handle, shader_handle);
    errors_count = glGetError();
    if (errors_count != GL_NO_ERROR) {
        destroy(false);
        return false;
    }

    glLinkProgram(handle);
    GLint is_linked = GL_FALSE;
    glGetProgramiv(handle, GL_LINK_STATUS, &is_linked);
    errors_count = glGetError();
    if (errors_count != GL_NO_ERROR || is_linked != GL_TRUE) {
        destroy(false);
        return false;
    }

    return true;
}

/**
 * @brief Initialize program from shader file path.
 */
/**
 * @brief Initialize program from inline shader source.
 */
bool Program::init(
    const Context& context,
    const std::filesystem::path& shader_path,
    unsigned local_size_x_,
    unsigned local_size_y_,
    unsigned local_size_z_,
    const std::vector<std::pair<std::string, std::string>>& replacements)
{
    if (!context.isInitialized()) {
        return false;
    }
    local_size_x = local_size_x_;
    local_size_y = local_size_y_;
    local_size_z = local_size_z_;
    auto merged  = replacements;
    merged.emplace_back("{{LOCAL_SIZE_X}}", std::to_string(local_size_x));
    merged.emplace_back("{{LOCAL_SIZE_Y}}", std::to_string(local_size_y));
    merged.emplace_back("{{LOCAL_SIZE_Z}}", std::to_string(local_size_z));
    source = loadShaderSource(shader_path, merged);
    return init();
}

bool Program::init(
    const Context& context,
    const std::string& shader_source,
    unsigned local_size_x_,
    unsigned local_size_y_,
    unsigned local_size_z_,
    const std::vector<std::pair<std::string, std::string>>& replacements)
{
    if (!context.isInitialized()) {
        return false;
    }
    local_size_x = local_size_x_;
    local_size_y = local_size_y_;
    local_size_z = local_size_z_;
    auto merged  = replacements;
    merged.emplace_back("{{LOCAL_SIZE_X}}", std::to_string(local_size_x));
    merged.emplace_back("{{LOCAL_SIZE_Y}}", std::to_string(local_size_y));
    merged.emplace_back("{{LOCAL_SIZE_Z}}", std::to_string(local_size_z));
    source = loadShaderSource(shader_source, merged);
    return init();
}

/**
 * @brief Return compute shader local-size layout qualifier.
 */
std::string Program::glsl_layout() const
{
    char* str        = nullptr;
    const int result = asprintf(&str, "layout (local_size_x = %u, local_size_y = %u, local_size_z = %u) in", local_size_x, local_size_y, local_size_z);
    std::string out  = (result < 0 || str == nullptr) ? std::string { } : std::string(str);
    if (str != nullptr) {
        std::free(str);
    }
    return out;
}

/**
 * @brief Dispatch compute workgroups and wait for completion.
 */
bool Program::dispatch(unsigned width, unsigned height, unsigned depth) const
{
    glUseProgram(handle);
    glDispatchCompute(
        (width + std::max(1u, local_size_x) - 1) / std::max(1u, local_size_x),
        (height + std::max(1u, local_size_y) - 1) / std::max(1u, local_size_y),
        (depth + std::max(1u, local_size_z) - 1) / std::max(1u, local_size_z));
    glMemoryBarrier(GL_ALL_BARRIER_BITS);
    GLsync completion = glFenceSync(GL_SYNC_GPU_COMMANDS_COMPLETE, 0);
    if (completion != 0) {
        GLenum wait_status = glClientWaitSync(completion, GL_SYNC_FLUSH_COMMANDS_BIT, 1000000000ull);
        while (wait_status == GL_TIMEOUT_EXPIRED) {
            wait_status = glClientWaitSync(completion, GL_SYNC_FLUSH_COMMANDS_BIT, 1000000000ull);
        }
        glDeleteSync(completion);
        if (wait_status == GL_WAIT_FAILED) {
            glUseProgram(0);
            return false;
        }
    }
    glUseProgram(0);
    return Instance::gl_errors_count() == GL_NO_ERROR;
}

/**
 * @brief Delete program/shader resources.
 */
GLuint Program::destroy(bool free_source)
{
    if (free_source) {
        source.clear();
    }
    if (shader_handle != 0) {
        glDeleteShader(shader_handle);
    }
    shader_handle = 0;
    if (handle != 0) {
        glDeleteProgram(handle);
    }
    handle = 0;
    return Instance::gl_errors_count();
}

/**
 * @brief Resolve and cache resource binding information.
 */
GLuint Program::find_resource(Resource& resource) const
{
    if (resource.value >= 0) {
        return 0;
    }

    GLenum binding_property = GL_BUFFER_BINDING;
    GLuint index            = 0;
    switch (resource.type) {
    case GL_IMAGE_2D:
        resource.value = glGetUniformLocation(handle, resource.name.c_str());
        break;
    case GL_ATOMIC_COUNTER_BUFFER:
        resource.value = static_cast<GLint>(glGetProgramResourceIndex(handle, GL_UNIFORM, resource.name.c_str()));
        break;
    case GL_SHADER_STORAGE_BUFFER:
        index = glGetProgramResourceIndex(handle, GL_SHADER_STORAGE_BLOCK, resource.name.c_str());
        glGetProgramResourceiv(handle, GL_SHADER_STORAGE_BLOCK, index, 1, &binding_property, sizeof(GLint), nullptr, &resource.value);
        break;
    default:
        return static_cast<GLuint>(-1);
    }
    return Instance::gl_errors_count();
}

/**
 * @brief Initialize uniform metadata.
 */
GLuint Program::uniform_init(Uniform& uniform) const
{
    GLuint index     = 0;
    const char* name = uniform.name.c_str();
    glGetUniformIndices(handle, 1, &name, &index);
    uniform.index    = index;
    uniform.location = glGetUniformLocation(handle, name);
    glGetActiveUniform(handle, uniform.index, 0, nullptr, reinterpret_cast<GLint*>(&uniform.size), &uniform.type, nullptr);
    return Instance::gl_errors_count();
}

/**
 * @brief Write uniform by type into active shader.
 */
GLuint Program::uniform_write(const Uniform& uniform, const void* data) const
{
    glUseProgram(handle);
    switch (uniform.type) {
    case GL_FLOAT:
        glUniform1fv(uniform.location, uniform.size, static_cast<const GLfloat*>(data));
        break;
    case GL_UNSIGNED_INT:
        glUniform1uiv(uniform.location, uniform.size, static_cast<const GLuint*>(data));
        break;
    case GL_INT:
        glUniform1iv(uniform.location, uniform.size, static_cast<const GLint*>(data));
        break;
    case GL_FLOAT_VEC2:
        glUniform2fv(uniform.location, uniform.size, static_cast<const GLfloat*>(data));
        break;
    case GL_UNSIGNED_INT_VEC2:
        glUniform2uiv(uniform.location, uniform.size, static_cast<const GLuint*>(data));
        break;
    case GL_INT_VEC2:
        glUniform2iv(uniform.location, uniform.size, static_cast<const GLint*>(data));
        break;
    case GL_FLOAT_VEC3:
        glUniform3fv(uniform.location, uniform.size, static_cast<const GLfloat*>(data));
        break;
    case GL_UNSIGNED_INT_VEC3:
        glUniform3uiv(uniform.location, uniform.size, static_cast<const GLuint*>(data));
        break;
    case GL_INT_VEC3:
        glUniform3iv(uniform.location, uniform.size, static_cast<const GLint*>(data));
        break;
    case GL_FLOAT_VEC4:
        glUniform4fv(uniform.location, uniform.size, static_cast<const GLfloat*>(data));
        break;
    case GL_UNSIGNED_INT_VEC4:
        glUniform4uiv(uniform.location, uniform.size, static_cast<const GLuint*>(data));
        break;
    case GL_INT_VEC4:
        glUniform4iv(uniform.location, uniform.size, static_cast<const GLint*>(data));
        break;
    case GL_FLOAT_MAT2:
        glUniformMatrix2fv(uniform.location, uniform.size, GL_FALSE, static_cast<const GLfloat*>(data));
        break;
    case GL_FLOAT_MAT3:
        glUniformMatrix3fv(uniform.location, uniform.size, GL_FALSE, static_cast<const GLfloat*>(data));
        break;
    case GL_FLOAT_MAT4:
        glUniformMatrix4fv(uniform.location, uniform.size, GL_FALSE, static_cast<const GLfloat*>(data));
        break;
    case GL_FLOAT_MAT2x3:
        glUniformMatrix2x3fv(uniform.location, uniform.size, GL_FALSE, static_cast<const GLfloat*>(data));
        break;
    case GL_FLOAT_MAT3x2:
        glUniformMatrix3x2fv(uniform.location, uniform.size, GL_FALSE, static_cast<const GLfloat*>(data));
        break;
    case GL_FLOAT_MAT2x4:
        glUniformMatrix2x4fv(uniform.location, uniform.size, GL_FALSE, static_cast<const GLfloat*>(data));
        break;
    case GL_FLOAT_MAT4x2:
        glUniformMatrix4x2fv(uniform.location, uniform.size, GL_FALSE, static_cast<const GLfloat*>(data));
        break;
    case GL_FLOAT_MAT3x4:
        glUniformMatrix3x4fv(uniform.location, uniform.size, GL_FALSE, static_cast<const GLfloat*>(data));
        break;
    case GL_FLOAT_MAT4x3:
        glUniformMatrix4x3fv(uniform.location, uniform.size, GL_FALSE, static_cast<const GLfloat*>(data));
        break;
    default:
        break;
    }
    glUseProgram(0);
    return Instance::gl_errors_count();
}

/**
 * @brief Load shader source and apply replacements.
 */
/**
 * @brief Load shader source text from file and apply replacements.
 */
std::string loadShaderSource(
    const std::string& shader_source,
    const std::vector<std::pair<std::string, std::string>>& replacements)
{
    std::string source = shader_source;
    for (const auto& [token, value] : replacements) {
        replaceAll(source, token, value);
    }
    return source;
}

std::string loadShaderSource(
    const std::filesystem::path& shader_path,
    const std::vector<std::pair<std::string, std::string>>& replacements)
{
    std::ifstream input(shader_path);
    if (!input.is_open()) {
        throw std::runtime_error("Failed to open shader file: " + shader_path.string());
    }

    std::stringstream buffer;
    buffer << input.rdbuf();
    return loadShaderSource(buffer.str(), replacements);
}

} // namespace uvdar_core::helpers::compute_shader
