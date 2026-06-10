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

class ErrorQueue {
public:
    void push(std::unique_ptr<Error> error);
    std::size_t size() const;
    std::unique_ptr<Error> pop();
    void clear();

private:
    std::queue<std::unique_ptr<Error>> queue_;
};

class Instance {
public:
    explicit Instance(const std::string& path = "");
    ~Instance();

    Instance(const Instance&)                = delete;
    Instance& operator=(const Instance&)     = delete;
    Instance(Instance&&) noexcept            = default;
    Instance& operator=(Instance&&) noexcept = default;

    static std::vector<std::string> render_devices();
    static Instance first_available();
    static void gl_debug_callback(GLenum source, GLenum type, GLuint id, GLenum severity, GLsizei length, const GLchar* message, const void* user_param);

    GLint init();
    void deinit();
    GLuint flush_errors(FILE* out);
    static void print_error(GLint err_code, FILE* out);
    static GLuint gl_errors_count();
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
    Context();
    ~Context();

    Context(const Context&)            = delete;
    Context& operator=(const Context&) = delete;

    bool initFirstAvailable();
    bool makeCurrent();
    bool releaseCurrent();
    void deinit();
    bool isInitialized() const;
    std::tuple<unsigned, unsigned, unsigned> getLocalSizes(unsigned image_width, unsigned image_height) const;
    Instance& instance();
    const Instance& instance() const;

private:
    std::unique_ptr<Instance> instance_;
};

class Resource {
public:
    Resource(const std::string& name_, GLuint type_);

    std::string name;
    GLuint type;
    GLint value;
};

class Framebuffer {
public:
    explicit Framebuffer(GLenum attachment_ = 0);
    ~Framebuffer();

    GLuint init();
    GLuint destroy();

    GLenum attachment;
    GLuint handle;
};

class Image2D {
public:
    Image2D(const std::string& name, GLenum texture, GLsizei width, GLsizei height, GLenum access, GLuint num_components, GLenum type);
    ~Image2D();

    void setup_format();
    GLuint init(GLenum framebuffer_attachment);
    std::string glsl_layout();
    GLuint destroy();
    GLuint reset(const void* px_data);
    GLuint reset_patch(const void* px_data, GLint x_min, GLint x_max, GLint y_min, GLint y_max);
    GLuint write(const void* image_data);
    GLuint read(void* image_data);
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
    ~ACBO();

    GLuint init(const void* data, GLint len);
    GLuint destroy();
    GLuint write(const void* data, GLint len);
    GLuint write_uint_val(GLuint value);
    GLuint read(void* data, GLint len);
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
    ~SSBO();

    GLuint init(const void* data, GLint len);
    GLuint destroy();
    std::string glsl_layout();
    GLuint write(const void* data, GLint len);
    GLuint read(void* data, GLint len);

    Resource resource;
    GLenum type;
    GLenum usage;
    GLuint handle;
};

class Uniform {
public:
    explicit Uniform(const std::string& name_);

    std::string name;
    GLuint location;
    GLuint size;
    GLenum type;
    GLuint index;
};

class Program {
public:
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

    bool init();
    bool init(
        const Context& context,
        const std::filesystem::path& shader_path,
        unsigned local_size_x,
        unsigned local_size_y,
        unsigned local_size_z,
        const std::vector<std::pair<std::string, std::string>>& replacements = { });
    bool init(
        const Context& context,
        const std::string& shader_source,
        unsigned local_size_x,
        unsigned local_size_y,
        unsigned local_size_z,
        const std::vector<std::pair<std::string, std::string>>& replacements = { });
    std::string glsl_layout() const;
    bool dispatch(unsigned width, unsigned height, unsigned depth) const;
    GLuint destroy(bool free_source = false);
    GLuint find_resource(Resource& resource) const;
    GLuint uniform_init(Uniform& uniform) const;
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
std::string loadShaderSource(
    const std::filesystem::path& shader_path,
    const std::vector<std::pair<std::string, std::string>>& replacements = { });

} // namespace uvdar_core::utils::compute_shader
