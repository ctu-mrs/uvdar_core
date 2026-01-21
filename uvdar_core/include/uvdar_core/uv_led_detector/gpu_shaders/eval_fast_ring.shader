#version 450

layout (local_size_x = 1) in;

layout(binding = 0) buffer bufA { float a[]; };
layout(binding = 1) buffer bufB { float b[]; };
layout(binding = 2) buffer bufOut { float o[]; };

void main() {
    uint index = gl_GlobalInvocationID.x;

    o[index] = a[index] * b[index];
}
