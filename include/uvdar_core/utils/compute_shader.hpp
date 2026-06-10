#pragma once

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <memory>
#include <queue>
#include <sstream>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include <EGL/egl.h>
#include <EGL/eglext.h>
#include <GLES3/gl32.h>
#include <gbm.h>

#include "uvdar_core/utils/gles.hpp"

namespace uvdar_core::utils::compute_shader {

/**
 * @brief Error payload emitted from GL debug callback.
 */
class Error {
public:
    GLuint err_id;
    std::string message;
    GLint message_len;
    GLenum source;
    GLenum type;
    GLuint id;
    GLenum severity;

    Error(GLuint err_id_, const std::string& message_, GLint message_len_, GLenum source_, GLenum type_, GLuint id_, GLenum severity_)
        : err_id(err_id_)
        , message(message_)
        , message_len(message_len_)
        , source(source_)
        , type(type_)
        , id(id_)
        , severity(severity_)
    {
    }
};

/**
 * @brief Thread-safe queue for debug/error messages.
 */
class ErrorQueue {
public:
    /**
     * @brief Push new GL error.
     */
    void push(std::unique_ptr<Error> error);
    /**
     * @brief Current queue length.
     */
    std::size_t size() const;
    /**
     * @brief Pop oldest error if available.
     */
    std::unique_ptr<Error> pop();
    /**
     * @brief Clear all queued errors.
     */
    void clear();

private:
    std::queue<std::unique_ptr<Error>> queue_;
};

class Instance {
public:
    /**
     * @brief Construct compute instance using optional custom DRM path.
     */
    explicit Instance(const std::string& path = "");
    /**
     * @brief Tear down EGL/GBM context.
     */
    ~Instance();

    Instance(const Instance&)                = delete;
    Instance& operator=(const Instance&)     = delete;
    Instance(Instance&&) noexcept            = default;
    Instance& operator=(Instance&&) noexcept = default;

    /**
     * @brief Enumerate available DRM render/card devices.
     */
    static std::vector<std::string> render_devices();
    /**
     * @brief Return first available rendering device.
     */
    static Instance first_available();
    /**
     * @brief GL debug callback that stores messages in the associated Instance queue.
     */
    static void gl_debug_callback(GLenum source, GLenum type, GLuint id, GLenum severity, GLsizei length, const GLchar* message, const void* user_param);

    /**
     * @brief Initialize EGL + GL + GBM resources.
     */
    GLint init();
    /**
     * @brief Release all resources owned by this instance.
     */
    void deinit();
    /**
     * @brief Drain queued GL messages and print to stream.
     */
    GLuint flush_errors(FILE* out);
    /**
     * @brief Print a human-readable error code explanation.
     */
    static void print_error(GLint err_code, FILE* out);
    /**
     * @brief Drain GL error flags and return count.
     */
    static GLuint gl_errors_count();
    /**
     * @brief Derive compute dispatch local sizes from image shape.
     */
    static std::tuple<unsigned, unsigned, unsigned> get_local_sizes(unsigned image_width, unsigned image_height);

    std::string dri_path;
    bool initialised = false;
    int fd           = 0;
    gbm_device* gbm  = nullptr;
    EGLDisplay dpy   = EGL_NO_DISPLAY;
    EGLContext ctx   = EGL_NO_CONTEXT;
    std::unique_ptr<Error> last_error;
    GLuint error_total_cnt = 0;
    std::unique_ptr<ErrorQueue> error_queue;
    GLenum verbosity = GL_DEBUG_SEVERITY_LOW;
};

class Context {
public:
    /**
     * @brief Construct wrapper with lazy default instance.
     */
    Context();
    /**
     * @brief Deinitialize owned compute instance.
     */
    ~Context();

    Context(const Context&)            = delete;
    Context& operator=(const Context&) = delete;

    /**
     * @brief Try to initialize first available render device.
     */
    bool initFirstAvailable();
    /**
     * @brief Bind this context on current thread.
     */
    bool makeCurrent();
    /**
     * @brief Unbind context from current thread.
     */
    bool releaseCurrent();
    /**
     * @brief Deinitialize context and backing instance.
     */
    void deinit();
    /**
     * @brief Check whether instance is initialized.
     */
    bool isInitialized() const;
    /**
     * @brief Compute preferred local invocation sizes.
     */
    std::tuple<unsigned, unsigned, unsigned> getLocalSizes(unsigned image_width, unsigned image_height) const;
    /**
     * @brief Mutable access to backing instance.
     */
    Instance& instance();
    /**
     * @brief Read-only access to backing instance.
     */
    const Instance& instance() const;

private:
    std::unique_ptr<Instance> instance_;
};

class Resource {
public:
    /**
     * @brief Construct named GL resource identifier.
     */
    Resource(const std::string& name_, GLuint type_);

    std::string name;
    GLuint type;
    GLint value;
};

class Framebuffer {
public:
    /**
     * @brief Construct framebuffer with optional attachment.
     */
    explicit Framebuffer(GLenum attachment_ = 0);
    /**
     * @brief Delete framebuffer if created.
     */
    ~Framebuffer();

    /**
     * @brief Create GL framebuffer object.
     */
    GLuint init();
    /**
     * @brief Delete GL framebuffer object.
     */
    GLuint destroy();

    GLenum attachment;
    GLuint handle;
};

class Image2D {
public:
    /**
     * @brief Construct 2D image resource.
     */
    Image2D(const std::string& name, GLenum texture, GLsizei width, GLsizei height, GLenum access, GLuint num_components, GLenum type);
    /**
     * @brief Destroy image resources.
     */
    ~Image2D();

    /**
     * @brief Configure GLES format/layout fields from type/size.
     */
    void setup_format();
    /**
     * @brief Allocate texture storage and optional framebuffer attachment.
     */
    GLuint init(GLenum framebuffer_attachment);
    /**
     * @brief Build GLSL image layout declaration.
     */
    std::string glsl_layout();
    /**
     * @brief Delete texture and framebuffer resources.
     */
    GLuint destroy();
    /**
     * @brief Upload full image buffer.
     */
    GLuint reset(const void* px_data);
    /**
     * @brief Upload a patch of the image.
     */
    GLuint reset_patch(const void* px_data, GLint x_min, GLint x_max, GLint y_min, GLint y_max);
    /**
     * @brief Upload raw image bytes.
     */
    GLuint write(const void* image_data);
    /**
     * @brief Read whole image back to host memory.
     */
    GLuint read(void* image_data);
    /**
     * @brief Read a patch from host memory.
     */
    GLuint read_patch(void* image_data, GLint x_min, GLint x_max, GLint y_min, GLint y_max, bool render);

    Resource resource;
    GLenum texture;
    GLsizei width;
    GLsizei height;
    GLenum access;
    GLfloat texture_wrap;
    GLfloat texture_filter;
    GLenum type;
    GLuint num_components;
    GLenum format;
    GLenum internal_format;
    GLenum compatibility_format;
    GLuint handle;
    GLuint data_size;
    GLuint px_size;
    Framebuffer framebuffer;
};

class ACBO {
public:
    ACBO()
        : resource("", GL_ATOMIC_COUNTER_BUFFER)
        , type(GL_UNSIGNED_INT)
        , usage(GL_DYNAMIC_DRAW)
        , handle(0)
    {
    }

    ACBO(const std::string& name, GLenum type, GLenum usage);
    /**
     * @brief Destroy atomic counter buffer.
     */
    ~ACBO();

    /**
     * @brief Allocate and optionally initialize ACBO.
     */
    GLuint init(const void* data, GLint len);
    /**
     * @brief Delete buffer.
     */
    GLuint destroy();
    /**
     * @brief Upload full counter payload.
     */
    GLuint write(const void* data, GLint len);
    /**
     * @brief Write one unsigned counter.
     */
    GLuint write_uint_val(GLuint value);
    /**
     * @brief Read full payload.
     */
    GLuint read(void* data, GLint len);
    /**
     * @brief Read one unsigned counter.
     */
    GLuint read_uint_val(GLuint* value);

    Resource resource;
    GLenum type;
    GLenum usage;
    GLuint handle;
};

class SSBO {
public:
    SSBO()
        : resource("", GL_SHADER_STORAGE_BUFFER)
        , type(GL_UNSIGNED_INT)
        , usage(GL_DYNAMIC_DRAW)
        , handle(0)
    {
    }

    SSBO(const std::string& name, GLenum type, GLenum usage);
    /**
     * @brief Destroy SSBO resources.
     */
    ~SSBO();

    /**
     * @brief Allocate and optionally initialize SSBO.
     */
    GLuint init(const void* data, GLint len);
    /**
     * @brief Delete SSBO handle.
     */
    GLuint destroy();
    /**
     * @brief GLSL std430 declaration for this SSBO.
     */
    std::string glsl_layout();
    /**
     * @brief Upload data to SSBO.
     */
    GLuint write(const void* data, GLint len);
    /**
     * @brief Read data from SSBO.
     */
    GLuint read(void* data, GLint len);

    Resource resource;
    GLenum type;
    GLenum usage;
    GLuint handle;
};

class Uniform {
public:
    /**
     * @brief Create uniform helper by GLSL name.
     */
    explicit Uniform(const std::string& name_);

    std::string name;
    GLuint location;
    GLuint size;
    GLenum type;
    GLuint index;
};

class Program {
public:
    /**
     * @brief Construct empty program shell.
     */
    Program() = default;

    template <typename... Args>
    Program(std::tuple<unsigned, unsigned, unsigned> local_sizes, const std::string& src, Args&&... extra_args)
        : source(src)
        , local_size_x(0)
        , local_size_y(0)
        , local_size_z(0)
        , handle(0)
        , shader_handle(0)
    {
        const auto [lx, ly, lz] = local_sizes;
        local_size_x            = lx;
        local_size_y            = ly;
        local_size_z            = lz;

        char* formatted  = nullptr;
        const int result = asprintf(&formatted, src.c_str(), lx, ly, lz, std::forward<Args>(extra_args)...);
        if (result < 0) {
            throw std::runtime_error("Error when formatting compute shader source");
        }
        source = std::string(formatted);
        std::free(formatted);
    }

    ~Program();

    Program(const Program&)            = delete;
    Program& operator=(const Program&) = delete;

    /**
     * @brief Build compute shader source with local sizes and extra replacements.
     */
    bool init();
    /**
     * @brief Load, patch and compile shader from file path.
     */
    bool init(
        const Context& context,
        const std::filesystem::path& shader_path,
        unsigned local_size_x,
        unsigned local_size_y,
        unsigned local_size_z,
        const std::vector<std::pair<std::string, std::string>>& replacements = { });
    /**
     * @brief Load, patch and compile shader from string source.
     */
    bool init(
        const Context& context,
        const std::string& shader_source,
        unsigned local_size_x,
        unsigned local_size_y,
        unsigned local_size_z,
        const std::vector<std::pair<std::string, std::string>>& replacements = { });
    /**
     * @brief Return GLSL local_size qualifier layout string.
     */
    std::string glsl_layout() const;
    /**
     * @brief Dispatch compute work for width/height/depth domain.
     */
    bool dispatch(unsigned width, unsigned height, unsigned depth) const;
    /**
     * @brief Delete program/shader objects.
     */
    GLuint destroy(bool free_source = false);
    /**
     * @brief Resolve buffer/image binding points for resource.
     */
    GLuint find_resource(Resource& resource) const;
    /**
     * @brief Query uniform location/type metadata.
     */
    GLuint uniform_init(Uniform& uniform) const;
    /**
     * @brief Upload values into uniform.
     */
    GLuint uniform_write(const Uniform& uniform, const void* data) const;

    std::string source;
    GLuint local_size_x  = 0;
    GLuint local_size_y  = 0;
    GLuint local_size_z  = 0;
    GLuint handle        = 0;
    GLuint shader_handle = 0;
};

std::string loadShaderSource(
    const std::string& shader_source,
    const std::vector<std::pair<std::string, std::string>>& replacements = { });
/**
 * @brief Load shader source and apply replacement tokens.
 */
std::string loadShaderSource(
    const std::filesystem::path& shader_path,
    const std::vector<std::pair<std::string, std::string>>& replacements = { });

} // namespace uvdar_core::utils::compute_shader
