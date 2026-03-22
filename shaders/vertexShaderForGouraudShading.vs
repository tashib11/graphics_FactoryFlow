#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aNormal;
layout (location = 2) in vec2 aTexCoords;

out vec4 vertexColor;
out vec2 TexCoords;

struct DirLight {
    vec3 direction;
    vec3 ambient;
    vec3 diffuse;
    vec3 specular;
};

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;
uniform vec3 viewPos;
uniform DirLight dirLight;
uniform bool dirLightOn;
uniform bool ambientOn;
uniform bool diffuseOn;
uniform bool specularOn;

void main()
{
    vec3 FragPos = vec3(model * vec4(aPos, 1.0));
    vec3 Normal = mat3(transpose(inverse(model))) * aNormal;  
    
    gl_Position = projection * view * vec4(FragPos, 1.0);
    TexCoords = aTexCoords;
    
    vec3 norm = normalize(Normal);
    vec3 viewDir = normalize(viewPos - FragPos);
    vec3 result = vec3(0.0);
    
    if (dirLightOn) {
        vec3 lightDir = normalize(-dirLight.direction);
        float diff = max(dot(norm, lightDir), 0.0);
        vec3 reflectDir = reflect(-lightDir, norm);
        float spec = pow(max(dot(viewDir, reflectDir), 0.0), 32.0);
        
        vec3 ambient  = dirLight.ambient;
        vec3 diffuse  = dirLight.diffuse * diff;
        vec3 specular = dirLight.specular * spec;
        
        if(!ambientOn) ambient = vec3(0.0);
        if(!diffuseOn) diffuse = vec3(0.0);
        if(!specularOn) specular = vec3(0.0);
        
        result += (ambient + diffuse + specular);
    } else {
        if(ambientOn) result += vec3(0.1); 
    }
    
    vertexColor = vec4(result, 1.0);
}
