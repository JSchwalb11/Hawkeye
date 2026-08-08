#version 330

in vec3 fragNormal;
in float fragFacing;

uniform vec4 colDiffuse;

out vec4 finalColor;

void main()
{
    // Alpha carries confidence, so it must survive the shading untouched.
    finalColor = vec4(colDiffuse.rgb * fragFacing, colDiffuse.a);
}
