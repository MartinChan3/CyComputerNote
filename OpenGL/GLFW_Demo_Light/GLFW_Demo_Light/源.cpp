#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <learnopengl/shader_m.h>
#include <learnopengl/camera.h>

#include <iostream>   

//回调函数
void framebuffer_size_callback(GLFWwindow* window, int width, int height);
void mouse_callback(GLFWwindow* window, double xpos, double ypos);
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);
void processInput(GLFWwindow *window);

//设置
const unsigned int SCR_WIDTH = 800;
const unsigned int SCR_HEIGHT = 600;

//摄像机
Camera camera(glm::vec3(0.0f, 0.0f, 3.0f));
float lastX = SCR_WIDTH / 2.0f;
float lastY = SCR_HEIGHT / 2.0f;
bool firstMouse = true;

//计时
float deltaTime = 0.0f;
float lastFrame = 0.0f;

//光照
glm::vec3 lightPos(1.2f, 1.0f, 2.0f);

int main()
{
	//glfw初始化
	glfwInit();
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

	//创建glfw窗口
	GLFWwindow* window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "LearnOpenGL", NULL, NULL);
	if (window == NULL)
	{
		std::cout << "Failed to create GLFW window" << std::endl;
		glfwTerminate();
		return -1;
	}
	glfwMakeContextCurrent(window);
	glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
	glfwSetCursorPosCallback(window, mouse_callback);
	glfwSetScrollCallback(window, scroll_callback);

	//让glfw捕获鼠标
	glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

	//载入glad封装的opengl函数
	if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
	{
		std::cout << "Failed to initialize GLAD" << std::endl;
		return -1;
	}

    //设置全局
	glEnable(GL_DEPTH_TEST);

	//编译自身的着色器
	Shader lightingShader("1.colors.vs", "1.colors.fs");
	Shader lampShader("1.lamp.vs", "1.lamp.fs");

	//多边形顶点数据
    float vertices[] = {
		-0.5f, -0.5f, -0.5f, 0.0f, 0.0f, -1.0f,
		0.5f, -0.5f, -0.5f, 0.0f, 0.0f, -1.0f,
		0.5f, 0.5f, -0.5f, 0.0f, 0.0f, -1.0f,
		0.5f, 0.5f, -0.5f, 0.0f, 0.0f, -1.0f,
		-0.5f, 0.5f, -0.5f, 0.0f, 0.0f, -1.0f,
		-0.5f, -0.5f, -0.5f, 0.0f, 0.0f, -1.0f,

		-0.5f, -0.5f, 0.5f, 0.0f, 0.0f, 1.0f,
		0.5f, -0.5f, 0.5f, 0.0f, 0.0f, 1.0f,
		0.5f, 0.5f, 0.5f, 0.0f, 0.0f, 1.0f,
		0.5f, 0.5f, 0.5f, 0.0f, 0.0f, 1.0f,
		-0.5f, 0.5f, 0.5f, 0.0f, 0.0f, 1.0f,
		-0.5f, -0.5f, 0.5f, 0.0f, 0.0f, 1.0f,

		-0.5f, 0.5f, 0.5f, -1.0f, 0.0f, 0.0f,
		-0.5f, 0.5f, -0.5f, -1.0f, 0.0f, 0.0f,
		-0.5f, -0.5f, -0.5f, -1.0f, 0.0f, 0.0f,
		-0.5f, -0.5f, -0.5f, -1.0f, 0.0f, 0.0f,
		-0.5f, -0.5f, 0.5f, -1.0f, 0.0f, 0.0f,
		-0.5f, 0.5f, 0.5f, -1.0f, 0.0f, 0.0f,

		0.5f, 0.5f, 0.5f, 1.0f, 0.0f, 0.0f,
		0.5f, 0.5f, -0.5f, 1.0f, 0.0f, 0.0f,
		0.5f, -0.5f, -0.5f, 1.0f, 0.0f, 0.0f,
		0.5f, -0.5f, -0.5f, 1.0f, 0.0f, 0.0f,
		0.5f, -0.5f, 0.5f, 1.0f, 0.0f, 0.0f,
		0.5f, 0.5f, 0.5f, 1.0f, 0.0f, 0.0f,

		-0.5f, -0.5f, -0.5f, 0.0f, -1.0f, 0.0f,
		0.5f, -0.5f, -0.5f, 0.0f, -1.0f, 0.0f,
		0.5f, -0.5f, 0.5f, 0.0f, -1.0f, 0.0f,
		0.5f, -0.5f, 0.5f, 0.0f, -1.0f, 0.0f,
		-0.5f, -0.5f, 0.5f, 0.0f, -1.0f, 0.0f,
		-0.5f, -0.5f, -0.5f, 0.0f, -1.0f, 0.0f,

		-0.5f, 0.5f, -0.5f, 0.0f, 1.0f, 0.0f,
		0.5f, 0.5f, -0.5f, 0.0f, 1.0f, 0.0f,
		0.5f, 0.5f, 0.5f, 0.0f, 1.0f, 0.0f,
		0.5f, 0.5f, 0.5f, 0.0f, 1.0f, 0.0f,
		-0.5f, 0.5f, 0.5f, 0.0f, 1.0f, 0.0f,
		-0.5f, 0.5f, -0.5f, 0.0f, 1.0f, 0.0f
	};

	//首先，配置好立方体的VAO和VBO
	unsigned int VBO, cubeVAO;
	glGenVertexArrays(1, &cubeVAO);
	glGenBuffers(1, &VBO);

	glBindBuffer(GL_ARRAY_BUFFER, VBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

	glBindVertexArray(cubeVAO);

	//位置属性
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
	glEnableVertexAttribArray(1);

	//第二步，配置光源的vao(其坐标仍然和相同)
	unsigned int lightVAO;
	glGenVertexArrays(1, &lightVAO);
	glBindVertexArray(lightVAO);

	//我们只需要在需要的时候绑定VBO（这里其实不需要重复绑定）
	glBindBuffer(GL_ARRAY_BUFFER, VBO);

	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
	glEnableVertexAttribArray(0);

	//渲染循环
	while (!glfwWindowShouldClose(window))
	{
		//计算deltaTime
		float currentFrame = glfwGetTime();
		deltaTime = currentFrame - lastFrame;
		lastFrame = currentFrame;

		//输入
		processInput(window);

		//渲染  
		glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		//确定使用正确的着色器当设置uniform变量和渲染一帧内容时
		lightingShader.use();
		lightingShader.setVec3("objectColor", 1.0f, 0.5f, 0.31f);
		lightingShader.setVec3("lightColor" , 1.0f, 1.0f, 1.0f);
		lightingShader.setVec3("lightPos", lightPos);
		lightingShader.setVec3("viewPos", camera.Position);

		//设定view和projection矩阵
		glm::mat4 projection = glm::perspective(glm::radians(camera.Zoom), (float)SCR_WIDTH / (float)SCR_HEIGHT, 0.1f, 100.0f);
		glm::mat4 view = camera.GetViewMatrix();
		lightingShader.setMat4("projection", projection);
		lightingShader.setMat4("view", view);

		//世界坐标转换(model矩阵)
		glm::mat4 model = glm::mat4(1.0f);
		lightingShader.setMat4("model", model);

		//渲染立方体
		glBindVertexArray(cubeVAO);
		glDrawArrays(GL_TRIANGLES, 0, 36);

		//绘制lamb内容
		lampShader.use();
		lampShader.setMat4("projection", projection);
		lampShader.setMat4("view", view);
		model = glm::mat4(1.0f);
		model = glm::translate(model, lightPos);
		model = glm::scale(model, glm::vec3(0.2f));
		lampShader.setMat4("model", model);

		glBindVertexArray(lightVAO);
		glDrawArrays(GL_TRIANGLES, 0, 36);   

		//glfw 
		glfwSwapBuffers(window);
		glfwPollEvents();
	}

	//可选的内容
	glDeleteVertexArrays(1, &cubeVAO);
	glDeleteVertexArrays(1, &lightVAO);
	glDeleteBuffers(1, &VBO);

	glfwTerminate();
	return 0;
}

// process all input: query GLFW whether relevant keys are pressed/released this frame and react accordingly
// ---------------------------------------------------------------------------------------------------------
void processInput(GLFWwindow *window)
{
	if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
		glfwSetWindowShouldClose(window, true);

	if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
		camera.ProcessKeyboard(FORWARD, deltaTime);
	if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
		camera.ProcessKeyboard(BACKWARD, deltaTime);
	if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
		camera.ProcessKeyboard(LEFT, deltaTime);
	if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
		camera.ProcessKeyboard(RIGHT, deltaTime);
	if (glfwGetKey(window, GLFW_KEY_Q) == GLFW_PRESS)
		camera.ProcessKeyboard(UP, deltaTime);
	if (glfwGetKey(window, GLFW_KEY_E) == GLFW_PRESS)
		camera.ProcessKeyboard(DOWN, deltaTime);
}

// glfw: whenever the window size changed (by OS or user resize) this callback function executes
// ---------------------------------------------------------------------------------------------
void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
	// make sure the viewport matches the new window dimensions; note that width and 
	// height will be significantly larger than specified on retina displays.
	glViewport(0, 0, width, height);
}


// glfw: whenever the mouse moves, this callback is called
// -------------------------------------------------------
void mouse_callback(GLFWwindow* window, double xpos, double ypos)
{
	if (firstMouse)
	{
		lastX = xpos;
		lastY = ypos;
		firstMouse = false;
	}

	float xoffset = xpos - lastX;
	float yoffset = lastY - ypos; // reversed since y-coordinates go from bottom to top

	lastX = xpos;
	lastY = ypos;

	camera.ProcessMouseMovement(xoffset, yoffset);
}

// glfw: whenever the mouse scroll wheel scrolls, this callback is called
// ----------------------------------------------------------------------
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
{
	camera.ProcessMouseScroll(yoffset);
}