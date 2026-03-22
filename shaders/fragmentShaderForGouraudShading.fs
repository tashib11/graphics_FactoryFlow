#version 330 core
out vec4 FragColor;

in vec4 vertexColor;
in vec2 TexCoords;

struct Material {
    sampler2D diffuse;
};
uniform Material material;

void main()
{
    FragColor = vertexColor * texture(material.diffuse, TexCoords);
}
