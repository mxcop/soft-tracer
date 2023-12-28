/**
 * This shader is for debugging purposes only!
 * Used to display a buffer to the screen with a screen quad.
 */
#version 400 core

out vec4 frag_color;
in vec2 texcoord;

uniform sampler2D buffer_sampler;

void main()
{
    frag_color = vec4(texture(buffer_sampler, texcoord).rgb, 1.0);
    // frag_color = vec4(1.0);
}
