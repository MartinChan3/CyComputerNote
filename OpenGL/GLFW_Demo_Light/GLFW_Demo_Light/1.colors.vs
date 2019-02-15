#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aNormal;

out vec3 Normal;
out vec3 FragPos;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

void main()
{
	FragPos = vec3(model * vec4(aPos, 1.0f)); //将坐标内容变换到世界坐标，用于接下来计算漫反射光源的大小，留意这里的写法
	Normal =  mat3(transpose(inverse(model))) * aNormal;

	gl_Position = projection * view * model * vec4(aPos, 1.0);
}