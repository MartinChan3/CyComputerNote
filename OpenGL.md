# OpenGL及其可编程渲染管线   

[LearnOpenGL CN](https://learnopengl-cn.github.io/)

## 第一章 介绍   
	1. 状态机(Status Machine)：OpenGL本身是一个巨大的状态机，**即通过一系列变量来描述OpenGL来如何运行**。OpenGL的状态往往被称为**上下文(Context)**。我们通常使用以下方法来改变OpenGL的状态：**设置选项→操作缓冲→使用当前OpenGL上下文渲染**。
	例如我们如果想要绘制的是线段而不是三角形，我们会通过改变一些上下文变量来改变OpenGL的状态，从而告诉OpenGL如何去绘图，一旦我们改变了OpenGL的状态为绘制线段，那么下一个绘制命令画出来就会画出线段而不是三角形；
	OpenGL主要包含两类函数：
	> 状态设置(State-changing Function)函数:改变上下文用
	> 状态使用(State-using Function)函数：根据当前OpenGL状态执行一些操作

	2. 对象概念：OpenGL中对象指一些选项的集合，代表了OpenGL状态的一个子集，例如可以用一个对象来代表绘图窗口的设置，之后就可以针对它的大小、支持的颜色位数等内容进行设置，可以把对象视作为一个C风格的结构体(Struct)：
	```
	struct object_name{
		float option1;
		float option2;
		char[] name;
	};
	```
	使用对象的好处在于在一个程序中，可以定义不止一个对象，然后设置其选项，每个对象都可以是不同的设置；在我们执行一个使用OpenGL状态操作时，只需要绑定含有需要设置的对象即可；	*比如说我们有一些作为3D模型数据（一栋房子或一个人物）的容器对象，在我们想绘制其中任何一个模型的时候，只需绑定一个包含对应模型数据的对象就可以了（当然，我们需要先创建并设置对象的选项）。拥有数个这样的对象允许我们指定多个模型，在想画其中任何一个的时候，直接将对应的对象绑定上去，便不需要再重复设置选项了。*
	
## 入门
	### 创建窗口
	1. GLFW库：提供渲染上下文的最简易的接口；
	2. GLAD：针对OpenGL提供简化的OpenGL规范接口函数；
	
	### 你好，窗口
	1. GLFW是利用渲染循环来接触所有事件；
	
	### 你好，三角形
	1. 三个基本名词，牢记：   
	> 顶点数组对象: Vertex Array Object, VAO
	> 顶点缓冲对象：Vertex Buffer Object, VBO
	> 索引缓冲对象：Element Buffer Object, EBO或者IBO
	2. OpenGL中任何事物都在3D空间中，但是屏幕和窗口却是2D像素数组，这引出了**图形渲染管线(Graphics Pipeline)的概念**，主要划分为两个部分：**1) 把3d坐标转换为2d坐标；2) 将2d坐标转换为实际有颜色的像素；**这个流程依赖于GPU数以万计的小核心来进行并行运算，实现该流程的函数称之为着色器；
	> Tip:2D坐标和像素也是不同的，前者表示一个精准的位置(理论坐标)，后者指的是依赖于屏幕/窗口分辨率的坐标近似值。
	3. GLSL：蓝色部分为可以自定义着色器的部分   
	![GLSL着色器流程](https://learnopengl-cn.github.io/img/01/04/pipeline.png)
	
	3. **着色器数据处理流程**：首先，以数组形式传递3个3d坐标作为图形渲染管线的输入，该数组称为顶点数据(Vertex Data);顶点数据是一系列顶点的集合。一个顶点(Vertex)是一个3d坐标数据的集合，而顶点数据是用顶点属性(Vertex Attribute)表示的数据，可以包含任何我们想用的数据；   
	
	> Tips: 为了让OpenGL知道我们坐标和颜色构成的到底是什么，OpenGL需要我们制定这些数据表示的渲染类型（例如只是渲染点？还是直线？还是一系列三角形？）。这种提示称之为**图元**(Primitive)，任何一个绘制指令调用都将把图元传递给OpenGL，例如GL_POINTS、GL_LINE_STRIP;   
	
	**顶点着色器阶段**：将单独顶点作为输入，将3d坐标转换为另一种坐标，同时也可以对顶点属性进行一些基本处理；   
	图元装配阶段：将顶点着色器输出的顶点作为输入，并组成指定图元的形状；例如本例中是一个三角形；  
	几何着色器阶段：以图元装配结果为输入，通过构造新的顶点构造出(或者其他的)图元来生成形状；本例生成了另一个三角形；   
	光栅化阶段：图元会最终被映射到屏幕上对应的像素，生成供片段着色器使用的片段(Fragment)。在片段着色器执行之前会执行裁切(Clipping)，这会丢弃超过你视图外的所有像素，用来提升效率；   
	
	片段着色器阶段：主要是计算一个像素的最终颜色，**也是所有OpenGL高级效果产生的地方**。通常，片段着色器包含3D场景的数据(例如光照、阴影、光的颜色等等)，这些数据可以被用来计算最终像素的颜色；   
	
	Alpha测试和混合(Blending)阶段：这是最后一个阶段，主要用来检测深度(DPETH)和模板(STENCIL)值，用来判断这个像素是物体的前面还是后面，决定是否对其进行抛弃；这个阶段也会检查alpha值(透明度)，并且对物体的颜色进行混合。所以即使片段着色器计算出了一个像素的输出颜色，在渲染多个三角形时输出的颜色也可能完全不同；   
	
	4. 顶点输入：
	实际当中OpenGL只处理**标准化设备坐标**，即xyz三轴只在-1.0到1.0的范围之内才处理，这之间就会涉及到转换问题；
	> 标准化设备坐标(Normalized Device Coordinates,NDC):一旦顶点坐标在**顶点着色器**中被处理过，它们就应该是标准化设备坐标。与通常屏幕坐标不同的是，y轴方向朝上，而(0,0)是这个图像的中心，而非左上角。*所有变换过的坐标都在这个空间中，否则不可见*；接着该标准化设备坐标会变为屏幕空间坐标(Screen-space Coordinates),这是由于你使用了glViewport设定了视口决定的。所得的屏幕空间坐标又会被变换为片段输入到片段着色器中。   
	在顶点数据定义后，我们会将其发送到顶点着色器进行处理：**它会在GPU上创建内存用来存储顶点数据，还会配置OpenGL如何来解释这部分内存，并且指定它如何发送给显卡。顶点着色器接着会处理我们在内存中指定数量的顶点。**
	我们通过**顶点缓冲对象(VBO)**来管理这个内存，它会在GPU内存(我们称之为显存)存储大量的顶点，使用这些缓冲对象的好处在于可以一次性的发送大批数据到显卡上，而不是每个顶点发送一次。CPU发送数据到显卡是相对较慢的，所以我们要一次性尽可能发送多的数据。当数据发送到显卡的内存中后，顶点着色器几乎能够立即访问顶点，这个过程非常快；
	
	复制数据时存在三种形式：GL_STATIC_DRAW数据几乎不变、GL_DYNAMIC_DRAW数据会改变很多以及GL_STREAM_DRAW数据每次绘制时都会改变；
	
	简单的顶点着色器：   
	```
	#version 330 core
	layout (location = 0) in vec3 aPos; //in关键字声明所有的输入顶点属性

	void main()
	{
		//为了设置顶点输出，必须赋给预定义的gl_Positioin变量
		gl_Position = vec4(aPos.x, aPos.y, aPos.z, 1.0);
	}
	```
	如上文所说，这一部分值得注意的是真正的程序里输入的往往不是标准化设备坐标，必须先要将其转化到OpenGL的可视区域内；
	
	简单的片段着色器：
	```
	#version 330 core
	out vec4 FragColor;

	void main()
	{
		FragColor = vec4(1.0f, 0.5f, 0.2f, 1.0f);
	} 
	```
	3.3OpenGL版本之后只限定了gl_Position这一固定参数，而着色器输出颜色进行了开放，使用``out``关键字来进行输出变量声明；
	
	5. 着色器程序：着色器对象(Shader Program Object)是多个着色器合并之后最终链接完成的版本。如果需要使用刚刚编译的着色器则我们必须把它们**链接(Link)**为一个着色器程序对象，然后在渲染对象的时候激活这个着色器程序。已激活的着色器程序的程序将在我们发送渲染调用的时候被使用；
	
	6. **链接顶点属性**：顶点着色器允许以任何形式的顶点属性输入数据，同时也意味着我们要手动指定输入数据的哪一部分对应顶点着色器的哪一个顶点属性；所以，我们必须在渲染前指定OpenGL如何解释顶点数据，这依赖于函数``glVertexAttribPointer``；
	![VBO数据构成解释](https://learnopengl-cn.github.io/img/01/04/vertex_attribute_pointer.png)
	```
	glVertexAttribPointer(
		int location, //对应顶点属性位置，例如顶点着色器中的0
		int size, //顶点属性大小
		GLTYPE type, //数据类型，例如GL_FLOAT,
		GL_BOOL normalized, //设定是否会被标准化
		int stride, //步长，揭示连续的顶点数据之间的间隔，
		void* offset, //起始位置的偏移量
	)
	```
	
	> Tips: 每个顶点属性从一个VBO管理的内存中获取它的数据，而具体从哪个VBO（程序中往往有多个）获取，则是由**调用glVertexAttributePointer时绑定到GL_ARRAY_BUFFER的VBO**决定的。由于在调用glVetexAttribPointer之前绑定的是先前定义的VBO对象，顶点属性0现在会链接到它的顶点数据。
	
	至此，一个大致的完整的在OpenGL中绘制物体的框架已经浮上水面：
	```
	// 0. 复制顶点数组到缓冲中供OpenGL使用
	glBindBuffer(GL_ARRAY_BUFFER, VBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);
	// 1. 设置顶点属性指针
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
	glEnableVertexAttribArray(0);
	// 2. 当我们渲染一个物体时要使用着色器程序
	glUseProgram(shaderProgram);
	// 3. 绘制物体
	someOpenGLFunctionThatDrawsOurTriangle();
	```
	以上流程同时带来一个新的问题：每当我们绘制一个物体的时候，都必须重复这一个过程，画一个物体还好，但是如果要画超过5个顶点属性，同时有上百个不同物体呢？
	
	
	