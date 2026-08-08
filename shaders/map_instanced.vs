#version 330

// Instanced map cells. One draw call per colour bucket per chunk; the per-cell
// transform arrives as a mat4 vertex attribute.

in vec3 vertexPosition;
in vec3 vertexNormal;
in mat4 instanceTransform;

uniform mat4 mvp;

out vec3 fragNormal;
out float fragFacing;

void main()
{
    vec4 world = instanceTransform * vec4(vertexPosition, 1.0);

    // Normals only need the rotation part, and the transforms are uniform
    // scales, so the upper 3x3 is safe to use directly.
    mat3 rot = mat3(instanceTransform);
    fragNormal = normalize(rot * vertexNormal);

    // A fixed key light keeps cell faces distinguishable without a full
    // lighting pass; occupancy already carries meaning through opacity.
    const vec3 key = normalize(vec3(0.4, 1.0, 0.25));
    fragFacing = clamp(dot(fragNormal, key) * 0.45 + 0.65, 0.0, 1.0);

    gl_Position = mvp * world;
}
