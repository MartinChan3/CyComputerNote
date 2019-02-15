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
	FragPos = vec3(model * vec4(aPos, 1.0f)); //���������ݱ任���������꣬���ڽ����������������Դ�Ĵ�С�����������д��
	Normal =  mat3(transpose(inverse(model))) * aNormal;

	gl_Position = projection * view * model * vec4(aPos, 1.0);
}