# OpenGL及其可编程渲染管线   

[LearnOpenGL CN](https://learnopengl-cn.github.io/)

# 入门
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
以上流程同时带来一个新的问题：每当我们绘制一个物体的时候，都必须重复这一个过程，画一个物体还好，但是如果要画超过5个顶点属性，同时有上百个不同物体呢？很明显就需要引入一个机制，来使得同一个绘制任务，可以方便的进行存储和复现；

7. 顶点数组对象：VAO这时候隆重登场，在其被绑定后，所有的顶点属性调用都会包含在这个VAO当中。这带来的好处是显而易见的：绘制同一类物体时，你只需要绑定配置一次，下次一再需要进行绘制时，你只需要绑定对应的VAO就可以，而在不同的顶点数据和属性配置之间切换，你只需要绑定不同的VAO。
> Tip:OpenGL的核心模式**要求**我们使用VAO，这样它才会知道如何处理顶点输入。如果绑定失败，则OpenGL会拒绝绘制任何东西；
一个顶点数组对象会储存以下这些内容：
	- glEnableVertexAttribArray和glDisableVertexAttribArray的调用。
	- 通过glVertexAttribPointer设置的顶点属性配置。	- 通过glVertexAttribPointer调用与顶点属性关联的顶点缓冲对象。
	
8. 索引缓冲对象：EBO用于存储(VBO中数据的)索引，OpenGL调用这些顶点的索引来决定绘制哪个顶点。有一个很明显的特征是使用了``glDrawElements``来进行绘制；   
这样进一步完善了整个绘制流程的内容；  
!(VAO、VBO和EBO的关系)[https://learnopengl-cn.github.io/img/01/04/vertex_array_objects_ebo.png]
```
// ..:: 初始化代码 :: ..
// 1. 绑定顶点数组对象
glBindVertexArray(VAO);
// 2. 把我们的顶点数组复制到一个顶点缓冲中，供OpenGL使用
glBindBuffer(GL_ARRAY_BUFFER, VBO);
glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);
// 3. 复制我们的索引数组到一个索引缓冲中，供OpenGL使用
glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices, GL_STATIC_DRAW);
// 4. 设定顶点属性指针
glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
glEnableVertexAttribArray(0);

[...]

// ..:: 绘制代码（渲染循环中） :: ..
glUseProgram(shaderProgram);
glBindVertexArray(VAO);
glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0)
glBindVertexArray(0);
```

### 着色器
1. GLSL：下面给出一个典型的GLSL着色器的形式
```
#version version_number
in type in_variable_name;
in type in_variable_name;

out type out_variable_name;

uniform type uniform_name;

int main()
{
  // 处理输入并进行一些图形操作
  ...
  // 输出处理过的结果到输出变量
  out_variable_name = weird_stuff_we_processed;
}
```
2. 数据类型：类似C语言，有着大部分语言默认的数据类型int/float/double/uint/bool,同时具有着向量变量vecn(默认为float类型)，使用.xyzw(坐标)、.rgba(颜色)、.stpq(纹理)作为固定的后缀；同时允许和进行重组和截取；   
3. 输入与输出：GLSL提供了in和out关键字来进行数据输入和输出的管理；输入方面，使用了layout(location = 0)关键字指定输入变量的位置。此外颜色输出器必须有个颜色分量的输出，否则OpenGL会将结果渲染为黑色（或者白色）；   
如果需要两个着色器之间的数据有联系，则需要在两个着色器之间声明一个相同名称和类型的变量，这样（在程序链接阶段），两者之间就可以发送数据；

3. Uniform:Uniform是指一种从CPU向GPU中着色器发送数据方式，但是Uniform形式和顶点属性略有不同，首先Uniform形式是**全局的**，这意味着在每个着色器程序中，uniform变量都是独一无二的，而且可以在着色器的任意阶段被访问；第二就是无论把uniform设定为射门，uniform变量会一直保留其数据，直到被重置或者更新；
简单三角形随着时间颜色变化的例子显示了uniform变量的有效性。但是同时也暴露了一个问题：这样需要针对每个点都有一个uniform变量；   
更为通用的方法是VBO中每个点除了基本的坐标数据，也包含颜色分量数据，使用在顶点着色器的顶点属性项；

### 纹理   
纹理本身的处理是为了解决大量点数据的输入问题，艺术家通常使用2D（其实也有1D和3D的纹理），用来添加物体的细节；
为了能够把纹理**映射(Map)**到三角形，我们需要指定的三角形的每个顶点各自对应纹理的哪个部分。这样每个顶点就会关联着一个**纹理坐标**，用来告诉着色器从纹理的图像的哪个部分采样，之后其他片段将会类似着色盘一样进行片段插值；   
纹理坐标：纹理坐标在x轴和y轴上，范围为0到1之间(注意使用的是2d纹理)。使用纹理坐标获取纹理颜色称为**采样(Sampling)**。纹理坐标始于(0, 0),即纹理图片的左下角，终始于(1, 1), 即图片的右上角。   
![纹理坐标](https://learnopengl-cn.github.io/img/01/06/tex_coords.png)   
1. 纹理环绕方式：OpenGL对于纹理采样的解释非常宽松，它可以采用几种不同的插值方式，所以需要指定OpenGL如何进行纹理采样，主要有以下几种方式：  
> GL_REPEAT:默认模式，重复纹理图像；
> GL_MIRRORED_REPEAT:同上，但纹理本身镜像倒置；
> GL_CLAMP_TO_EDGE:纹理坐标被约束在0到1之间，但是超出的部分会重复纹理坐标的边缘，形成一种拉伸的效果；   
> GL_CLAMP_TO_BORDER:超出的坐标为用户指定的边缘颜色；   
![纹理插值形式](https://learnopengl-cn.github.io/img/01/06/texture_wrapping.png)   
2. 纹理过滤：纹理坐标不会依赖于分辨率，可以是任意的浮点值，所以OpenGL需要知道如何将纹理像素(Texture Pixel, 也称为Texel)映射到纹理坐标；这种现象尤其在一个很大的物体中，但是纹理分辨率很低的情况下显得尤其重要；所以这里引入了**纹理过滤(Texture Filter)**的选项。主要包含两种：``GL_NEAREST``和``GL_LINEAR``。   
> GL_NEAREST: 邻近过滤，是默认的OpenGL纹理过滤方式；以最邻近于该点坐标的内容替代；（通常显现为颗粒感）   
> GL_LINEAR: 线性过滤，是一个基于附近值的插值计算结果;
Tip: 当进行放大(Magnify)和缩小(Minify)操作时可以设置纹理过滤的选项，例如在纹理被缩小时使用邻近过滤，被放大的使用线性过滤（这样的处理非常类似于很多图片查看软件的处理方式）；   
3. 多级渐远的纹理：想象下在一个包含着上千物品的大房间，每个物体上都有纹理。由于近远物体拥有着相同分辨率的纹理，由于远处物体可能只需要产生很少的纹理片段，OpenGL从高分辨率纹理中为这些片段获取正确的颜色值就变成了一个很难的事情；这种情况在小物体上显得非常明显；OpenGL引入**多级渐远纹理(Mipmap)**来解决这个问题，简单来说就是生成了一系列纹理图像，后一个纹理图像是前一个的二分之一大小；多级渐远的理念非常简单：当应用纹理的物体距离观察者超过一定阈值，OpenGL会使用不同的多级渐远纹理，同时保持着超级好的性能；OpenGL提供了``glGenerateMipmaps``函数来处理这类事；      
同时在切换多级渐远纹理级别时，OpenGL在两个不同级别的多级渐远纹理层之间会场产生生硬的边界。类似普通纹理过滤，切换多级纹理级别时也可以使用NEAREST和LINEAR进行过滤。存在以下四个类型进行多级渐远纹理过滤：
> GL_NEAREST_MIPMAP_NEAREST：使用最邻近的多级渐远纹理来匹配像素大小，并使用邻近插值进行纹理的采样；
> GL_LINEAR_MIPMAP_NEAREST: 使用最邻近多级渐远纹理级别，并且使用线性插值进行采样；
> GL_NEAREST_MIPMAP_LINEAR: 在两个最匹配像素大小的多级渐远纹理之间进行线性插值，使用邻近插值进行采样；
> GL_LINEAR_MIPMAP_LINEAR: 在两个邻近的多级渐远纹理之间进行线性插值，并使用线性插值进行采样；

4. 生成纹理：标准的纹理设定流程类似如下   
```
unsigned int texture;
glGenTextures(1, &texture);
glBindTexture(GL_TEXTURE_2D, texture);
// 为当前绑定的纹理对象设置环绕、过滤方式
glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);   
glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
// 加载并生成纹理
int width, height, nrChannels;
unsigned char *data = stbi_load("container.jpg", &width, &height, &nrChannels, 0);
if (data)
{
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, data);
    glGenerateMipmap(GL_TEXTURE_2D);
}
else
{
    std::cout << "Failed to load texture" << std::endl;
}
stbi_image_free(data);
```

5. 应用纹理：类似于先前告知OpenGL中VBO中数据的组织形式，直接告知每个坐标最后两个坐标为纹理坐标，并依次调整补偿为8；同时，需要在着色器中告诉OpenGL具体使用哪个纹理，这里在片段着色器中使用**采样器(Sampler2D)**，以纹理类型作为后缀(sampler2D),在使用VAO绑定绘制之前，调用``glBindTexture(GL_TEXTURE_2D, texture);``来启用2d纹理（牢记，包括2d纹理在内都是状态机的概念）。   
6. 纹理单元：这里指出为何sampler2D是uniform但是不使用普通的glUniform对其进行赋值；转而使用glUniform1i，**可以给纹理采样器分配一个位置值**，这样我们能够在一个片段着色器中设置多个纹理。一个纹理的位置值通常我们称为纹理单元(Texture Unit)。一个纹理的默认纹理单元是0，它是默认激活的纹理单元；   
纹理单元的核心目标是让用户能够使用多于一个的纹理；如果我们想要使用多个纹理，可以使用激活纹理单元的方法，（类似状态机似的）再去绑定对应的纹理； 
```
glActiveTexture(GL_TEXTURE0); //绑定前激活纹理单元
glBindTexture(GL_TEXTURE_2D, texture);
```  
Tip: OpenGL保证了至少有16个纹理单元可以使用，可以激活从GL_TEXTURE0到GL_TEXTURE15。可以使用类似指针的方式调用邻近的纹理；   

### 变换   
1. 向量： 方向+大小；
2. 向量和标量计算；
3. 向量取反；
4. 向量加减；
5. 向量长度：引入了单位向量概念；
6. 向量乘法：两种乘法——点乘和叉乘；
> 点乘：a * b = XaXb + YaYb + ZaZb = 模a * 模b * cosθ
> 叉乘：a · b = (YaZb - ZaYb, ZaXb - XaZb, XaYb - YaXb)，方向由右手定则决定
7. 矩阵的运算；
###  矩阵和向量相乘：向量的本质可以理解为N*1大小的矩阵，在OpenGL中可以表示颜色或者纹理坐标，**即只有一列的坐标**。这样，我们有了**变换(Transform)**概念的计算基础，这样极大的方便了使用；  
1. 单位矩阵；
2. 缩放：往往使用不均匀缩放，矩阵如下   
$$
\begin{bmatrix} \color{red}{S_1} & \color{red}0 & \color{red}0 & \color{red}0 \\ \color{green}0 & \color{green}{S_2} & \color{green}0 & \color{green}0 \\ \color{blue}0 & \color{blue}0 & \color{blue}{S_3} & \color{blue}0 \\ \color{purple}0 & \color{purple}0 & \color{purple}0 & \color{purple}1 \end{bmatrix} \cdot \begin{pmatrix} x \\ y \\ z \\ 1 \end{pmatrix} = \begin{pmatrix} \color{red}{S_1} \cdot x \\ \color{green}{S_2} \cdot y \\ \color{blue}{S_3} \cdot z \\ 1 \end{pmatrix}
$$   
3. 位移：在原有基础进行位移，矩阵实现形式如下
$$
\begin{bmatrix}  \color{red}1 & \color{red}0 & \color{red}0 & \color{red}{T_x} \\ \color{green}0 & \color{green}1 & \color{green}0 & \color{green}{T_y} \\ \color{blue}0 & \color{blue}0 & \color{blue}1 & \color{blue}{T_z} \\ \color{purple}0 & \color{purple}0 & \color{purple}0 & \color{purple}1 \end{bmatrix} \cdot \begin{pmatrix} x \\ y \\ z \\ 1 \end{pmatrix} = \begin{pmatrix} x + \color{red}{T_x} \\ y + \color{green}{T_y} \\ z + \color{blue}{T_z} \\ 1 \end{pmatrix}
$$   
Tip：引入的第四维向量称作为**齐次坐标(Homogeneous Coordinates)**, 后面可以看到，当w为1.0时，是允许该矩阵往3d向量上进行位移，为0的时候就不允许，并且退化为方向向量；   
4. 旋转，基本的思路是先以特定的旋转轴（xyz三轴）为基础，进行最基本的旋转，然后三者叠加相乘；以下先给出基于z轴的旋转矩阵计算结果：   
$$
\begin{bmatrix} \color{red}{\cos \theta} & - \color{red}{\sin \theta} & \color{red}0 & \color{red}0 \\ \color{green}{\sin \theta} & \color{green}{\cos \theta} & \color{green}0 & \color{green}0 \\ \color{blue}0 & \color{blue}0 & \color{blue}1 & \color{blue}0 \\ \color{purple}0 & \color{purple}0 & \color{purple}0 & \color{purple}1 \end{bmatrix} \cdot \begin{pmatrix} x \\ y \\ z \\ 1 \end{pmatrix} = \begin{pmatrix} \color{red}{\cos \theta} \cdot x - \color{red}{\sin \theta} \cdot y  \\ \color{green}{\sin \theta} \cdot x + \color{green}{\cos \theta} \cdot y \\ z \\ 1 \end{pmatrix}
$$   
这样的旋转模型已经很完善，但是可能会导致另一个问题——万向节锁死。更优化的矩阵是沿着任意一个轴，进行旋转而不是对一系列旋转轴进行复合。但是其表达相当复杂（而且在数学上无法完全解决万向节锁死人问题）；   
重点来了，真正的万向节死锁解决方法为**四元数(Quaternion)**，兼具有高效率和安全性，后面会进行讨论；   
### 矩阵的组合   
可以通过组合方式实现变化(例如先缩放和再位移)   
### 实践   
1. GLM: 引入GLM库
2. 基本运算；
3. 牢记变换的顺序会影响最终缩放的效果；   

## 坐标系统   
引言：前文已经强调过，顶点着色器会将xyz三轴坐标转换为标准化设备坐标，进而传输数据进光栅，转换为最终的二维坐标或者像素；   
从标准化设备坐标到最终的像素坐标，仍然需要经历几个过程(被变换到多个坐标系统);这主要是由于，某些特殊的操作需要在特定的坐标系下操作更方便；这样的坐标系一共有五个：
> **局部空间(Local Space)**,或被称为物体空间；
> **世界空间(World Space)**
> **观察空间(View Space)**，或被称为视觉空间；
> **裁剪空间(Clip Space)**
> **屏幕空间(Screen Space)**  
### 概述    
为了将坐标从一个坐标系变换到另一个坐标系，分别需要使用三个重要的坐标系**模型(Model)**、**观察(View)**、**投影(Projection)**,下面这张图非常好的解释了各个坐标和变换之间的关系   
![空间坐标系变换](https://learnopengl-cn.github.io/img/01/08/coordinate_systems.png)   
- 局部坐标是指物体相对于局部原点的坐标，也是物体起始的坐标；（很像是3d零件中单独part绘制）   
- 下一步是变换为世界空间坐标(使用Model矩阵)，即将所有部件放在一个更大的范围内进行绘制；（类似3d装配中导入各个零件）   
- 世界坐标转换为观察坐标(使用View矩阵)，使得每个坐标都是从摄像机或者说观察者的角度进行观察的；  
- 坐标到达观察空间后，我们需要将其投影到裁剪坐标，裁剪坐标会被处理到1.0到-1.0的范围内，并判断哪些点会出现在屏幕上；   
- 最后我们将裁剪坐标变换为屏幕坐标，我们会使用一个**视口变换(Viewport Transform)**来把-1.0到1.0的内容发送到光栅器，将其转换为片段；      
至此为止可以解释上述的为何使用多个坐标系方便，例如当需要对物体本身修改，则在局部空间中才说得通；如果要对一个物体相对于其他物体相对位置改变，则在世界坐标系中才说的通；

### 局部空间   
局部空间指的是物体所在的做包空空间；

### 世界空间   
为了将各个零件从局部空间上区分开来，从物体的坐标从局部变换到世界空间，这种变换即依赖模型矩阵；   
模型矩阵是一种变换矩阵，它能够通过对物体进行位移、缩放、旋转来使它放置于本应在的位置或者朝向；   
   
### 观察空间   
观察空间经常被人们称为OpenGL的**摄像机(Camera)**，所以有时候也被称为摄像机空间或者视觉空间；观察空间是将世界空间坐标转换为用户视野前方的坐标而产生的结果。这通常是由**一系列的位移和旋转**组成的，这样的变换组合使得特定的对象被变换到摄像机前方；这样的组合我们存储在一个观察矩阵中，我们将在下一节仔细讨论其实现；   

### 裁剪空间   
OpenGL一向希望所有坐标落在一个特定的范围内，范围之外的点都会被裁剪掉; 因为所有可见的坐标在-1.0到1.0之间不是特别直观，所以我们会指定自己的坐标集(Coordinate Set)并将它变换回标准化设备坐标系；  
为了将顶点坐标从观察空间变换到裁剪空间，我们需要定义一个**投影矩阵(Projection Matrix)**，它指定了一个范围的坐标，比如每个维度上的-1000到1000。投影矩阵接着会在这个指定范围内容将坐标变换为标准化设备坐标(-1.0,1.0)的范围内。所有在这个范围外的坐标会被裁减掉。
> Tip: 如果只是图元(Primitive), 例如三角形的一部分超出裁剪体积(Clipping Volume), 则OpenGL会重新构建这个三角形或者多个三角形，来让其能够适合这个裁剪范围；   

由投影矩阵创建的**观察箱(Viewing Box)**被称为平截头体(Frustum), 每个出现在平截头体范围内的坐标都会出现在用户屏幕上； 将特定范围内的坐标转换到标准化设备坐标系过程称之为投影(Projection),因为使用投影矩阵能将3D坐标投影(Project)到很容易映射到2D的标准化设备中；   
一旦所有顶点被变换到裁剪空间，最终的操作**透视除法(Perspective Division)**会被执行，整个过程我们将位置向量的x、y、z分量分别处以向量的齐次w分量；透视除法是将4d裁剪空间坐标转换为3d标准化设备坐标的过程，这一步会在每一个顶点着色器运行的最后自动执行；   
进行过该阶段后，最终坐标会被映射到屏幕空间(使用glViewport),并被赋予为片段；
将观察坐标变换为裁剪坐标的投影坐标分为两种形式，分别对应两种不同的平截头体，分别称之为**正射投影矩阵**和**透视投影矩阵**；   
  
### 正射投影   
正射投影矩阵定义了一个**类似立方体的平截头箱**，定义了一个裁剪空间，在这个空间之外的顶点都会被裁剪掉。   
![正射投影]()https://learnopengl-cn.github.io/img/01/08/orthographic_frustum.png   
平截头体定义了宽、高、近和远四个平面，任何超出这个立方体四个平面的内容都会被舍弃掉。正射平截头体将平截头体内部的所有坐标映射为标准化设备坐标， 因为每个向量的w分量没有改变；如果w分量等于1.0，透视除法则则不会改变这个坐标；   
使用``glm::ortho``来创建一个正射投影矩阵，例如：    
```
glm::ortho(0.0f, 800.0f, 0.0f, 600.0f, 0.1f, 100.0f);
```
前两个参数指定平截头体左右坐标，第三第四参数指定了平截头体的底部和顶部，最后两个参数指定了近平面和远平面的距离；   
正射投影能将坐标映射到2D平面中，但其实它忽略了一个问题：一个直接的投影矩阵会产生不真实的结果，因为这个投影没有把透视考虑进去；   

### 透视投影
透视的基本例子来源于生活，例如双眼所观察到的铁路在远处相交，它是借由**透视投影矩阵**完成的。这个投影矩阵将给定的平截头体范围映射到裁剪空间，*除此之外还修改了每个顶点坐标的w值，从而使得离观察者越远的顶点坐标w分量越大*。被变换到裁剪空间的坐标都会在-w到w的范围之内（任何大于这个范围坐标的内容都会被裁剪掉）。OpenGL要求所有的可见坐标都落在-1.0到1.0之间的范围，所以作为顶点着色器的输出结果，一旦坐标在裁剪空间之后，透视除法就会被应用到裁剪空间坐标上：``out = (x/w, y/w, z/w)``。顶点坐标的每个分量都会除以其w分量，距离观察者越远，其坐标就会越小。   
GLM中按照下面例子创建一个透视投影矩阵：
```
glm::mat4 proj = glm::perspective(glm::radians(45.0f), (float)width/(float)heigth, 0.1f, 100.0f);
```
![透视平截头体](https://learnopengl-cn.github.io/img/01/08/perspective_frustum.png)   
该平截头体第一个参数为fov，我们称之为**视野(Field of View)**，并设定了观察空间的大小。如果想要一个真实的观察效果，通常会被设定为45.0f，如果要一末日风格，则可以设置的更大。第二个参数设定了宽高比，由视口的宽除以高所得，第三个参数决定了平截头体的近和远平面，我们通常设置0.1f和100.0f。所有处于近平面和远平面之间，并且位于平截头体以内的顶点都会被渲染。   
如果near值设的过大(例如10.0f)，则会导致先前部分的内容会被完全裁剪掉，会类似游戏中离物体太近，视线会直接穿过模型。   
![两种投影方式的区别](https://learnopengl-cn.github.io/img/01/08/perspective_orthographic.png)
Tip： 正射投影往往应用于二维渲染以及一些建筑或者工程的程序，在这种情况下用户不希望观察内容被透视所干扰；   

### **组合**
综上所述，我们已经得到三个变换矩阵：模型矩阵、观察矩阵和投影矩阵，一个顶点坐标将会被按照以下过程变换到裁剪坐标：   
![从原始坐标到裁剪坐标的矩阵](https://s2.ax1x.com/2019/01/29/kQ3p6g.png)   
记住矩阵运算顺序和书写顺序是相反的（由于矩阵乘法无结合律决定）上述式子的结果被顶点着色器赋予给着色器中的gl_Position,OpenGL会进行自动的透视除法和裁剪；   
Tip: 再之后的流程会对裁剪坐标进行透视除法从而将其变为标准化设备坐标，然后使用``glViewport``来将标准化设备坐标映射到屏幕坐标，每个坐标都关联了屏幕上的一个点，以上过程称为视口变换；   

### 进入3D   
1) 创建模型矩阵，让矩形在世界坐标中“躺下”:围绕x轴旋转   
```
glm::mat4 model;
model = glm::rotate(model, glm::radians(-55.0f), glm::vec3(1.0f, 0.0f, 0.0f));
```   
2) 创建观察矩阵，可以观察到物体：首先想下**将摄像机向后移动，等价于将整个场景向前移动**，那么结论非常明显，我们只需要以相反于摄像机移动方向的方向来移动整个场景就好。根据OpenGL符合右手坐标系，正Z方向是朝外的。   
![OpenGL中的右手坐标系](https://learnopengl-cn.github.io/img/01/08/coordinate_systems_right_handed.png)
Tip: *DirectX和OpenGL的标准化坐标系中，使用的是左手坐标系；*   
这样我们可以得到我们的观察矩阵；   
```
glm::mat4 view;
// 注意，我们将矩阵向我们要进行移动场景的反方向移动。
view = glm::translate(view, glm::vec3(0.0f, 0.0f, -3.0f));
```   
3) 创建投影矩阵：使用透视投影
```
glm::mat4 projection;
projection = glm::perspective(glm::radians(45.0f), screenWidth / screenHeight, 0.1f, 100.0f);
```

### Z缓冲   
有了以上的数据和类型，我们已经能够正确的绘制矩形，但是能很明显看出来OpenGL绘制结果很奇怪，这是由于绘制的结果存在一个覆盖的效果，需要我们进行**深度测试(Depth Testing)**。通过``glEnable(GL_DEPTH_TEST)``来打开，同时我们需要在每一次渲染之前清除深度缓冲``(glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT))``。   

### 更多的立方体   
这一小节极大的显示出了之前VAO和多种矩阵的价值：如果需要绘制多个相同的模型，不需要重新构建，只需要在模型矩阵中对每个对象变换到世界坐标的各个位置中； 


## 摄像机    
OpenGL中本身没有摄像机(Camera)的概念，但是可以通过把场景所有物体向相反方向移动的方式来模拟出摄像机，产生一种观察者在移动的感觉。   
### 摄像机/观察空间    
当我们讨论摄像机/观察空间(Camera/View Space)时，本质上都在讨论**以摄像机视角为原点下， 场景中所有顶点的坐标**。观察矩阵(即上一章中的View Matrix)把所有的世界坐标变换为相对于摄像机位置与方向的观察坐标。  
**要定义一个摄像机，需要：1）该摄像机在世界空间中的位置；2）观察的方向；3）一个指向它右侧的向量以及一个指向它上方的向量；**    
![Camera](https://learnopengl-cn.github.io/img/01/09/camera_axes.png)   
1. 摄像机位置：   
直接用世界空间中指向摄像机位置的一个向量来代替摄像机位置，如下   
```
glm::vec3 cameraPos = glm::vec3(0.0f, 0.0f, 1.0f);
```
> Tip: 不要忘记z轴是从屏幕指向于你的，如果摄像机向后移动，我们就沿着z轴的正方向移动；   
2. 摄像机方向：   
摄像机方向，指的是摄像机朝向哪个方向。本例中使摄像机指向场景原点，根据矢量减法原则，我们可以获得一个指向Z轴负方向的方向向量(正交化)；   
```
glm::vec3 cameraTarget = glm::vec3(0.0f, 0.0f, 0.0f);
glm::vec3 cameraDirection = glm::normalize(cameraPos - cameraTarget);
```   
> Tip: **方向**向量(Direction Vector)并不是最好的名字，因为它实际上指向从它到目标向量的相反方向（译注：注意看前面的那个图，蓝色的方向向量大概指向z轴的正方向，与摄像机实际指向的方向是正好相反的）。*(简单来说，方向向量是由摄像机指向目标点的相反向量，用摄像机坐标减去指向点坐标获得)*   
3. 右轴：   
我们需要另一个向量称之为**右向量(Right Vector)**，它代表摄像机空间x轴的方向。为了获取右向量，我们使用一个小技巧：先定义一个上向量(Up vector)。接下来将上向量和第二步获得的方向向量进行叉乘，即可获得指向x轴正方向的那个向量（如果交换叉乘顺序就会得到相反指向x负方向的向量）
```
glm::vec3 up = glm::vec3(0.0f, 1.0f, 0.0f);
glm::vec3 cameraRight = glm::normalize(glm::cross(up, cameraDirection));
```   
4. 上轴：   
现在已经有x轴和z轴向量，那么获取摄像机真正意义上的正y轴向量就显得非常简单，直接用右向量和方向向量叉乘：   
```
glm::vec3 cameraUp = glm::cross(cameraDirection, cameraRight);
```   
这样的流程称作为**格拉姆-施密特正交化**，这些摄像机向量我们可以创建一个LookAt矩阵。   
### LookAt   
使用矩阵的好处之一在于，如果你使用了三个相互垂直(或者非线性)的轴定义了一个空间，你可以使用这三个轴外加一个平移的向量来创建一个矩阵，并且你可以用这个矩阵乘以任何向量来变换到那个坐标空间，而这就是LookAt矩阵所完成的工作，其形式如下：   
![screenShot.png](https://i.loli.net/2019/02/12/5c623f5453366.png)   
其中R是右向量，U是上向量，D是方向向量，而P是摄像机位置。值得注意的是，位置向量始终是相反的，因为我们最终希望吧世界平移到与我们自身移动相反方向。使用这个LookAt矩阵作为观察矩阵，可以方便的将所有世界坐标变换到刚刚定义的观察空间。LookAt矩阵就类似其名字表达的一样：他会创建一个看着给定目标的观察矩阵；   
GLM方便的提供了lookAt函数来创建LookAt矩阵，只需要定义**1) 摄像机位置；2) 目标位置；3) （世界空间中的）上向量（创建右向量时）**，形式如下：   
```
glm::mat4 view;
view = glm::lookAt(glm::vec3(0.0f, 0.0f, 3.0f),
				   glm::vec3(0.0f, 0.0f, 0.0f),
				   glm::vec3(0.0f, 1.0f, 0.0f));
```   
### 自由移动    
接下来考虑自定义的移动情况，首先我们需要设定一个摄像机系统，这需要在程序开始定义三个摄像机变量：   
```
glm::vec3 cameraPos   = glm::vec3(0.0f, 0.0f,  3.0f);
glm::vec3 cameraFront = glm::vec3(0.0f, 0.0f, -1.0f);
glm::vec3 cameraUp    = glm::vec3(0.0f, 1.0f,  0.0f);
```   
LookAt函数变为了：   
```
view = glm::lookAt(cameraPos, cameraPos + cameraFront, cameraUp);
```   
（这里的cameraFront代指的是摄像机自身的瞄准点相对坐标，即向下看）   
最后，需要在processInput中对各个按键进行捕获和处理：   
```
void processInput(GLFWwindow *window)
{
    ...
    float cameraSpeed = 0.05f; // adjust accordingly
    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
        cameraPos += cameraSpeed * cameraFront;
    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
        cameraPos -= cameraSpeed * cameraFront;
    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
        cameraPos -= glm::normalize(glm::cross(cameraFront, cameraUp)) * cameraSpeed;
    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
        cameraPos += glm::normalize(glm::cross(cameraFront, cameraUp)) * cameraSpeed;
}
```   
这里如果希望摄像机前后移动，则直接把位置向量加上或者减去方向向量，如果希望左右移动，则先通过叉乘来计算右向量(标准化，否则朝不同方向的速度不同)，并沿着其移动即可；   
### 移动速度   
目前乍一看，移动看上去没啥问题，但是实际上根据处理器处理能力不同，有些情况可能会绘制更多帧数/秒，即以更高的频率调用processInput函数。结果就是，根据配置不同，有些人移动很快，而有些人很慢。这种情况不是我们愿意看到的，接下来讨论如何在任何硬件上保持同一移动速度；   
图形程序和游戏通常会跟踪一个时间差(deltaTime)变量，它存储了上一帧所需要的渲染时间。我们用所有的速度都去乘以deltaTime值，结果就是，如果我们的deltaTime很大，那么意味着我们上一帧花了相当长的时间，所以这一帧的速度需要变得更高来平衡渲染所用的时间。使用这种方法，无论你的电脑是快还是慢，摄像机的速度都会相应的平衡，这样每个用户的体验都是一样的。   
![screenShot.png](https://i.loli.net/2019/02/12/5c626462ab80e.png)   

### **视角移动**   
只有键盘的移动了无生趣，是时候表演鼠标的功能了！~为了能够改变视角，我们核心是针对cameraFront变量进行修改。   
#### 欧拉角   
**欧拉角(Euler Angle)**是可以表示3D空间任意旋转的3个值，由（牛逼炸了的）莱昂哈德·欧拉在18世纪提出。一共有三种欧拉角：**俯仰角(Pitch)/偏航角(Yaw)/滚转角(Roll)**   
![三种欧拉角](https://learnopengl-cn.github.io/img/01/09/camera_pitch_yaw_roll.png)   
三种角分别绕xyz三轴旋转，俯仰角描述如何往上或者往下看，偏航角描述了向左向右看的程度，滚转角代表我们如何翻滚摄像机，通常在太空飞船的摄像机中使用。有了这三个欧拉角，我们就能计算3D空间中的任何旋转向量了。   
通常摄像机系统只会讨论俯仰角和偏航角。   
公式推导：正常情况下，以斜边为1的三角形，可以相当容易的推出，三角形对边长度为sinθ，而邻边长度为cosθ。   
![三角推导](https://learnopengl-cn.github.io/img/01/09/camera_triangle.png)   
先来讨论俯仰角： 想象自己站在xz平面上，望向y轴，基本上可以得到旋转pitch大小的角度变化会影响一根长度为1的线段3个分量投影为：
```
direction.y = sin(glm::radians(pitch));
direction.x = cos(glm::radians(pitch));
direction.z = cos(glm::radians(pitch));
```   
![俯仰角分量结果](https://learnopengl-cn.github.io/img/01/09/camera_pitch.png)    
看看我们能否为偏航角找到需要的分量：  
![偏航角分量结果](https://learnopengl-cn.github.io/img/01/09/camera_yaw.png)   
非常容易的看到，其只会影响x轴和z轴的分量大小，所以得到了最后的表达形式为：  
```
direction.x = cos(glm::radians(pitch)) * cos(glm::radians(yaw)); // 译注：direction代表摄像机的前轴(Front)，这个前轴是和本文第一幅图片的第二个摄像机的方向向量是相反的
direction.y = sin(glm::radians(pitch));
direction.z = cos(glm::radians(pitch)) * sin(glm::radians(yaw));
```
两个欧拉角准备完毕，唯一的问题就在于如何获得俯仰角和偏航角了。    
### 鼠标输入   
偏航角和俯仰角是通过鼠标获得，**水平的移动影响偏航角，竖直的移动影响俯仰角**。它的基本原理就是，存储上一帧鼠标的位置，在当前帧中我们计算鼠标位置和上一帧的差值是多少。越大的差值，意味着越大的移动距离。   
首先我们要告诉GLFW，它应该隐藏光标，并且**捕获（capture）**它。捕捉光标的意思是，如果焦点在你的程序上，光标应该停留在窗口中（除非程序失去焦点或者退出）。我们可以用一个简单的配置调用来完成：   
```
glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
```   
在调用完整个函数后，无论怎么去移动鼠标，光标都不会显示，也不会离开窗口。这对于整个FPS摄像机系统非常完美，为了计算俯仰角和偏航角，我们需要让GLFW监听鼠标移动事件。和键盘输入类似，我们会引入一个回调函数来完成，函数的原型如下：   
```
void mouse_callback(GLFWwindow* window, double xpos, double ypos);
```
这里的xpos，ypos代表了鼠标当前的位置。当我们使用GLFW注册了回调函数之后，鼠标一移动mouse_callback就会被调用：``glfwSetCursorPosCallBack(window, mouse_callback);``  
在处理FPS风格的摄像机鼠标输入时，我们必须在获取最终方向向量之前做以下几步：   
> 1. 计算鼠标距上一帧的偏移量；   
> 2. 把偏移量添加到摄像机的俯仰角和偏航角当中；   
> 3. 把偏航角和俯仰角进行最大和最小值的限制；   
> 4. 计算方向向量；   
第一步是计算偏移量，我们首先要在程序中存储上一帧的位置，我们设初始值为屏幕中心(屏幕尺寸为800*600)：
```
float lastX = 400, lastY = 300;
```   
然后在回调函数中我们计算当前帧和上一帧鼠标位置的偏移量：   
```
float xoffset = xpos - lastX;
float yoffset = lastY - ypos; //注意，这里是相反的，因为y坐标是从底部往顶部依次增大的？？？（这里测试下）   
lastX = xpos;
lastY = ypos;
float sensitivity = 0.05f;
xoffset *= sensitivity;
yoffset *= sensitivity;
```   
灵敏度值影响着鼠标移动的大小，可以适当进行调整；   
接下来我们将偏移量加到全局变量pitch和yaw上：   
```
yaw   += xoffet;
pitch += yoffset;
```
第三步我们希望给摄像机添加一些限制，这样摄像机就不会发生奇怪的移动了，例如限制用户不能看超过高度89度（或低于89度）的内容，这样俯仰角限制的形式类似于：   
```
if (pitch > 89.0f)
	pitch = 89.0f;
if (pitch < -89.0f)
	pitch = 89.0f;
```   
偏航角我们这里没有进行设定，是因为我们不希望限制用户的水平旋转，当然如果你希望进行限制也非常容易，模仿上述形式即可。    
第四步，就是通过俯仰角和偏航角的计算来获得真正的方向向量：   
```
glm::vec3 front;
front.x = cos(glm::radians(pitch)) * cos(glm::radians(yaw));
front.y = sin(glm::radians(pitch));
front.z = cos(glm::radians(pitch)) * sin(glm::radians(yaw));
cameraFront = glm::normalize(front);
```
如果现在运行代码，会出现在窗口第一次获取焦点的时候摄像机会调一下，这是因为初始化的原因。这里使用一个简单的bool变量来判断是否是第一次进入循环，进而避免这种问题；   
最后代码以这样的形式呈现：   
```
void mouse_callback(GLFWwindow* window, double xpos, double ypos)
{
    if(firstMouse)
    {
        lastX = xpos;
        lastY = ypos;
        firstMouse = false;
    }

    float xoffset = xpos - lastX;
    float yoffset = lastY - ypos; 
    lastX = xpos;
    lastY = ypos;

    float sensitivity = 0.05;
    xoffset *= sensitivity;
    yoffset *= sensitivity;

    yaw   += xoffset;
    pitch += yoffset;

    if(pitch > 89.0f)
        pitch = 89.0f;
    if(pitch < -89.0f)
        pitch = -89.0f;

    glm::vec3 front;
    front.x = cos(glm::radians(yaw)) * cos(glm::radians(pitch));
    front.y = sin(glm::radians(pitch));
    front.z = sin(glm::radians(yaw)) * cos(glm::radians(pitch));
    cameraFront = glm::normalize(front);
}
```   

### 缩放   
同样比较常见的操作是实现**缩放(Zoom)**功能。之前我们曾经强调透视视图中的视野（FOV，Field of View）决定了我们在场景中可以看到多大的范围。当视野变小，投影出来的空间就会减小，产生了放大(Zoom In)的感觉。我们使用鼠标的滚轮来放大。和鼠标移动以及键盘输入一样，我们使用一个鼠标滑轮的回调函数：   
```
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
{
  if(fov >= 1.0f && fov <= 45.0f)
    fov -= yoffset;
  if(fov <= 1.0f)
    fov = 1.0f;
  if(fov >= 45.0f)
    fov = 45.0f;
}
```   
当滚动鼠标滚轮时，yoffset值代表数值滚动大小，我们限制缩放级别在1.0f到45.0f之间；   
我们必须把每一帧的透视投影矩阵上传到GPU，但是是使用fov作为它的视野：   
```
projection = glm::perspective(glm::radians(fov), 800.0f / 600.0f, 0.1f, 100.0f);   
```
最后注册回调函数：
glfwSetScrollCallback(window, scroll_callback);

# 光照   
## 颜色   
现实世界有无数种颜色，常见的颜色分量可以由RGB三分量构成。现实当中肉眼所见的颜色并非实际物体的颜色，而是由物体反射一定光源投入后的颜色。   
类似的原理被运用到图形学当中，我们引入白色的光源，用其和本身的颜色进行分量相乘，得到的就是最终自身的颜色：   
```
glm::vec3 lightColor(1.0f, 1.0f, 1.0f);
glm::vec3 toyColor(1.0f, 0.5f, 0.31f);
glm::vec3 result = lightColor * toyColor; // = (1.0f, 0.5f, 0.31f);
```   
由此看到，颜色的本质在于物体吸收了白色光源很大把一部分颜色，即**颜色是物体从一个光源反射的各个颜色分量的大小**。例如使用绿色光源，这个玩具颜色就会变成深绿色：   
```
glm::vec3 lightColor(0.0f, 1.0f, 0.0f);
glm::vec3 toyColor(1.0f, 0.5f, 0.31f);
glm::vec3 result = lightColor * toyColor; // = (0.0f, 0.5f, 0.0f);
```   
## 创建光照场景   
我们仍然使用一个标准的立方体来进行投光的实验，这次我们不使用纹理。我们仍然从最精简的顶点着色器开始：
```
#version 330 core
layout (location = 0) in vec3 aPos;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

void main()
{
    gl_Position = projection * view * model * vec4(aPos, 1.0);
}
```   
与先前不同的是，我们这次需要一个表示光源的立方体，我们会专门创建一个VAO。当然我们比较简单的能想到的，是让这个灯和其他物体使用同一个VAO，同时简单的对它的model（模型）矩阵进行一些变换。本文中由于灯本身数据简单，而且可能我们会对箱子的内容进行频繁更改，我们决定给灯一个新的vao。   
```
unsigned int lightVAO;
glGenVertexArrays(1, &lightVAO);
glBindVertexArray(lightVAO);
// 只需要绑定VBO不用再次设置VBO的数据，因为箱子的VBO数据中已经包含了正确的立方体顶点数据
glBindBuffer(GL_ARRAY_BUFFER, VBO);
// 设置灯立方体的顶点属性（对我们的灯来说仅仅只有位置数据）
glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
glEnableVertexAttribArray(0);
```   
接下来需要一个片段着色器：  
```
#version 330 core
out vec4 FragColor;

uniform vec3 objectColor;
uniform vec3 lightColor;

void main()
{
    FragColor = vec4(lightColor * objectColor, 1.0);
}
```   
这个片段着色器从uniform变量中接受物体的颜色和光源的颜色。正如本文一开始讨论的那样，我们将光源的颜色和物体反射的颜色相乘。我们将物体颜色设置为珊瑚红色，光源为白色。   
```
// 在此之前不要忘记首先 use 对应的着色器程序（来设定uniform）
lightingShader.use();
lightingShader.setVec3("objectColor", 1.0f, 0.5f, 0.31f);
lightingShader.setVec3("lightColor",  1.0f, 1.0f, 1.0f);
```   
值得注意的是，当我们修改顶点或者着色器片段后，灯的位置和颜色也会随之改变，这并不是我们想要的效果，所以我们需要另外绘制为灯专门设计的着色器，其顶点着色器和当前顶点着色器一样，但是片段着色器给灯定义了一个不变的白色常量，确保灯是一直亮着的：   
```
#version 330 core
out vec4 FragColor;

void main()
{
    FragColor = vec4(1.0); // 将向量的四个分量全部设置为1.0
}
```   
当我们绘制物体时，我们会使用刚刚定义的光照着色器来绘制箱子（或者是其他物体）。当我们想要绘制灯的时候，我们会使用灯的着色器。之后的教程我们会逐渐更新这个光照着色器，让其能够慢慢地实现更加真实的效果。   
使用这个灯立方体的意义，主要是为了让我们知道光源在场景中的具体位置，我们通常在场景中定义一个光源的位置，但这只是一个位置，并没有视觉意义。为了显示真正的灯，我们将表示光源的立方体绘制在和光源相同的地方，一直以纯白绘制，不受场景当中的光照影响；   
我们声明一个全局vec3变量来表示光源在场景的世界空间坐标中的位置：   
```
glm::vec3 ligthPos(1.2f, 1.0f, 2.0f);
```   
然后我们把灯移动到这里，并且缩小一点：   
```
model = glm::mat4();   
model = glm::translate(model, lightPos);
model = glm::scale(model, glm::vec3(0.2f));
```
绘制灯立方体的代码大约如下：   
```
lampShader.use();
// 设置模型、视图和投影矩阵uniform
...
// 绘制灯立方体对象
glBindVertexArray(lightVAO);
glDrawArrays(GL_TRIANGLES, 0, 36);
```
[本例](https://learnopengl.com/code_viewer_gh.php?code=src/2.lighting/1.colors/colors.cpp)中有非常完善的双shader的应用，请仔细观察VAO、VBO的绑定，以及对应的变量写入以及变化，进一步感知OpenGL的运行机制；   

## 基础光照   
现实的光照极其复杂，并且会受到诸多因素的影响，这是实际计算机无法模拟的。OpenGL因此引入的都是简化的光照模型，这些模型都是基于人类对于光的物理特性的理解。其中一个最基础的模型称之为**冯氏光照模型(Phong Lighting Model)**。冯氏光照模型由三个分量组成：**1）环境(Ambient); 2）漫反射(Diffuse); 3）镜面(Specular)**，如下图所示：   
![冯氏光照模型](https://learnopengl-cn.github.io/img/02/02/basic_lighting_phong.png)   
> 环境光照：即使在黑暗情况下，世界上也会有一些微弱的光（月光、远处的光），所以物体不会完全黑暗。为了模拟这种情况，我们会使用一个环境模拟光照，它使得物体会永远有一些颜色；   
> 漫反射光照：模拟光源对物体的方向性影响。它是冯氏光照模型中视觉上最明显的分量。物体的某一部分越是正对着光源，物体就越亮；   
> 镜面光照：模拟有光泽的物体的表面上出现的亮点，镜面光照的颜色相比物体的颜色会更接近光源的颜色；   
为了创建有趣的视觉场景，我们会尽量模拟这三种分量，先从最简单的环境光照开始；   
### 环境光照   
光通常都不是来自于同一个光源，而是来自身边很多分散的光源，即使他们可能本身不是那么显而易见。光的一个属性是，它可以向很多地方发散并且反弹，从而达到不是非常邻近的点。所以，光能够在其它表面上反射，对一个物体产生间接的影响。这样的算法称之为全局照明，但是它开销巨大并且极其复杂；   
我们先从简单开始，它的添加非常简单，只需要用光的颜色乘以一个非常小的环境因子就可以：   
```
void main()
{
    float ambientStrength = 0.1;
    vec3 ambient = ambientStrength * lightColor;

    vec3 result = ambient * objectColor;
    FragColor = vec4(result, 1.0);
}
```   
### 漫反射光照    
漫反射光照使得物体上与光线方向更接近的片段能够从光源处获得更多的亮度。为了更好的理解漫反射光照，请看下图：  
![漫反射示意图](https://learnopengl-cn.github.io/img/02/02/diffuse_light.png)   
图左上方有一个光源，它所发出的光线落在物体的一个片段上，我们需要测量这个光线是以什么角度接触到这个片段的，如果光线是以垂直角度于物体表面，那么这束光对物体的影响会最大化（更亮）。为了测量光线和片段的角度，我们会引入一个叫做法向量的东西，它是垂直于片段表面的一个向量（黄色箭头）。这两个向量之间的角度会很容易通过点乘计算出来。   
点乘返回一个标量，我们可以用它计算光线对片段颜色的影响。不同的片段朝向光源的方向不同，这些片段被照亮的情况也不同。所以计算漫反射光照需要什么？   
- 法向量：一个垂直于顶点表面的向量；   
- 定向的光线：作为光源位置与片段的位置之间向量差的方向向量。为了计算这个光线，我们需要光的位置向量和片段的位置向量；   
#### 法向量    
法向量是一个垂直于顶点表面的单位向量。由于顶点本身没有表面（它只是空间中一个独立的点），我们利用它周围的顶点来计算出这个顶点的表面。我们一般使用的技巧是叉乘立方体所有顶点来计算法向量。但是由于3d立方体还不算复杂，我们索性在vbo里把所有法向量全指定出来了；  
由于添加了额外的数据，我们应该更新光照的顶点着色器：   
```
#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aNormal;
//...
```   
现在我们向每个顶点添加了一个法向量并且更新了顶点着色器，但是同时需要注意灯是无需使用新的法向量的，新的步长变为6；   
> 虽然对灯的着色器使用不能完全利用的顶点数据看起来不是那么高效，但这些顶点数据已经从箱子对象载入后开始就储存在GPU的内存里了，所以我们并不需要储存新数据到GPU内存中。这实际上比给灯专门分配一个新的VBO更高效了。   
    
### 计算漫反射光照    
目前我们所有的顶点都有了法向量，但是我们仍然需要光源的位置向量和片段的位置向量（来计算相对位置向量）。光源位置是一个静态向量，所以我们使用uniform变量：   
```
uniform vec3 lightPos;
```   
然后放在渲染循环中更新uniform即可。通过``lightingShader.setVec3("lightPos", lightPos);``   
最后，我们还需要片段的位置。我们会在世界空间中进行所有的光照运算，因此我们需要一个在世界空间中的定点位置。我们可以通过把顶点位置属性乘以矩阵模型（不是观察和投影矩阵）来把它变换到世界空间坐标，这个工作在顶点着色器中非常容易完成：   
```
out vec3 FragPos;  
out vec3 Normal;

void main()
{
    gl_Position = projection * view * model * vec4(aPos, 1.0);
    FragPos = vec3(model * vec4(aPos, 1.0));
    Normal = aNormal;
}
```   
再在片段着色器中添加对应的输入变量``in vec3 FragPos;``即可。
   
现在所有需要的变量都已经设置好，我们可以着手在片段着色器中添加光照计算了。  
第一件事是计算光源和片段位置之间的方向向量。这个非常简单，只需要计算一个向量差即可；
```
vec3 norm = normalize(Normal);
vec3 lightDir = normalize(lightPos - FragPos);
```   
> 注意：当计算光照时我们通常不关心一个向量的模长或它的位置，我们只关心它们的方向。所以，几乎所有的计算都使用单位向量完成，因为这简化了大部分的计算（比如点乘）。所以当进行光照计算时，确保你总是对相关向量进行标准化，来保证它们是真正地单位向量。忘记对向量进行标准化是一个十分常见的错误。     
   
下一步，我们会对norm和lightDir进行点乘，计算光源对于当前片段实际的漫反射影响。其结果再乘以光的颜色，得到漫反射分量。两个向量之间的角度越大，漫反射分量就越小：   
```
float diff = max(dot(norm, lightDir), 0.0);
vec3 diffuse = diff * lightColor;
```   
如果两个向量之间的角度大于90度，那么点乘的结果会变成负数，这样会导致漫反射分量变成负数。为此我们使用max函数来返回两个参数之间较大的参数，从而保证漫反射分量不会变成负数。负数颜色的光照是没有定义的，最好要避免它，除非你是个古怪的艺术家……   
有了环境分量和漫反射分量，我们把它们相加，最后把结果乘以物体颜色，就可以得到输出颜色：   
```
vec3 result = (ambient + diffuse) * objectColor;
FragColor = vec4(result, 1.0);
```   

### 最后一件事   
现在我们已经把法向量从顶点着色器传到了片段着色器。到那时目前片段着色器的计算都是在世界空间坐标中完成的，所以我们是不是应该把法向量也转换为世界坐标？没错，基本正确，但是这不是一个简单的乘以一个矩阵模型就能完成的。   
首先，法向量只是空间中一个方向向量，它本身并不可以替代一个具体的空间位置。同时，法向量没有齐次坐标（即没有w分量），这意味着位移不应该影响到法向量。因此，如果我们准备把法向量乘以一个矩阵模型，我们首先需要从矩阵中移除掉平移的部分，而只选用左上角的3*3模型部分（或者更简单的方法是，**将w分量设置为0，再乘以4*4的矩阵**）。说白了对于法向量，我们只希望对它实施缩放和旋转变换；   

其次，另一个值得注意的方面，是如果模型矩阵执行了不等比缩放，这会导致法向量实际上也会发生方向上的变化，如图所示：   
![不等比缩放](https://learnopengl-cn.github.io/img/02/02/basic_lighting_normal_transformation.png)   
如图所示，每次我们应用一个不等比缩放的时候，法向量就不会垂直于对应的表面了，因为这样光照就会被破坏。   
修复这个行为的诀窍在于使用一个专门为法向量指定的模型矩阵，我们称之为**法线矩阵(Normal Matrix)**，这个矩阵使用一系列线性代数操作来移除对法向量错误缩放的影响。具体的推断方法见[链接](http://www.lighthouse3d.com/tutorials/glsl-12-tutorial/the-normal-matrix/)   
法向量矩阵被定义为模型矩阵左上角的逆矩阵的转置矩阵。值得注意的是，大部分的资源操作都会将法线矩阵定义为应用到模型-观察矩阵上的操作，但是由于我们只在世界空间内进行操作，所以我们只使用观察模型。  
顶点着色器中我们使用inverse和transpose函数来生成这个法线矩阵。这两个函数对所有的类型矩阵都有效。注意我们还要把被处理过的矩阵强制转换为3*3的矩阵，来确保其是去了位移属性以及能够乘以vec3的法向量；   
```
Normal = mat3(transpose(inverse(model))) * aNormal;   
```
目前漫反射光照部分没有问题，是因为我们没有对物体本身进行任何缩放操作，所以并不是必须要使用一个法线矩阵，仅仅让模型矩阵乘以法线也可以。但是如果进行了不等比缩放，用法线矩阵去乘以法向向量就是必不可少的了。   
> Tip：即使对于着色器来说，逆矩阵的计算也是个开销比较大的运算，因此要尽可能避免在着色器中进行逆矩阵运算。尽可能在CPU中进行运算，然后使用uniform把值传递给着色器（像模型矩阵一样）。   

### 镜面光照   
最后需要考虑的就是**镜面高光(Specular Highlight)**，这样冯氏光照模型才算完整。和漫反射光照一样，镜面光照也是依据光的反射特性。如果想象物体表面像一面镜子，无论从哪个表面观察那个表面所反射的光，镜面光照都会最大化。类似下图：   
![镜面反射原理图](https://learnopengl-cn.github.io/img/02/02/basic_lighting_specular_theory.png)   
我们通过反射法向量周围光的方向来计算反射向量。然后我们计算反射向量和视线方向的角度差，如果夹角越小，那么镜面光的影响会越大。它的效果在于，当我们去看光被物体所反射的那个方向，我们会得到一个高光。   
观察向量是镜面光照附加的一个向量，我们可以用观察者世界空间位置和片段来计算它。然后，我们计算镜面光强度，用它乘以光源的颜色，再将它加上环境光和漫反射分量。   
> 本文主要选择在世界空间进行光照计算，但是绝大多数人都会选择在观察空间进行光照计算。这是因为：观察者的位置总是(0,0,0)，这样就直接获得了观察者的位置。但是根据统计，在学习时在世界空间中计算光照更符合直觉。如果需要在观察空间计算光照的话，则需要将所有相关的向量都是用观察矩阵进行变换。（同时对法线矩阵也进行变换）   
为了获得观察者世界空间坐标，我们简单实用摄像机对象的位置坐标，使用一个uniform变量添加到片段着色器：   
```
uniform vec3 viewPos;

lightingShader.setVec3("viewPos", camera.Position);
```
获得位置信息后，可以计算高光强度了。首先定义一个镜面强度变量，给镜面高光一个中等亮度颜色，不要让它过度影响本什么颜色；   
```
float specularStrength = 0.5f;
```
然后计算视线方向向量，和对应的沿着法线轴的反射向量：   
```
vec3 viewDir = normalize(viewPos - FragPos);
vec3 reflectDir = reflect(-lightDir, norm);
```
注意，这里因为reflect函数要求第一个向量是由光源指向片段位置，所以我们对lightDir进行了取反。第二个参数要求是一个法向量。   
剩下下的镜面分量计算，下面的代码完成了该项工作：   
```
float spec = pow(max(dot(viewDir, reflectDir), 0.0), 32);
vec3 specular = specularStrength * spec * lightColor;
```   
我们先计算视线方向和反射方向的点乘（交角，同时保证不是负值），然后取其32次幂。这个32称之为高光的**反光度(Shininess)**。一个物体的反光度越高，反射光的能力就越强，散射的就越少，高光点就越小。   
> 着色器早期，开发者曾在顶点着色器中实现冯氏光照模型。其优势在于相比片段着色器，顶点要少得多，因此更加高效，所以开销会减少。但是顶点着色器中的颜色仅仅只是那个顶点的颜色，片段的颜色则是由插值光照颜色得来，结果就是片段的着色器比较真实和平滑。前者则被称为Gouraud着色。   
![颜色着色器](https://learnopengl-cn.github.io/img/02/02/basic_lighting_gouruad.png)    


## 材质   
现实世界各种材质对光的反应也有所不同。例如钢比陶瓷瓶闪闪发亮，木头箱子不会像钢皮箱子一样有很强的反射。每个物体对镜面高光也有不同反应。有些物体反射光的时候不会有太多的散射(Scatter)，因此会产生一个较小的高光点。有些物体却有着更大的散射，会有一个更大半径的高光点；所以，我们必须给每个物体定义一个材质属性来模拟多种类型的物体；   
上一节已经讲了一个物体的光和颜色以及结合环境光以及镜面强度分量，来定义物体的视觉输出；当描述一个物体时，我们可以定义一个材质颜色：基本上就由环境光照、漫反射光照和镜面光照组成。通过为每个分量制定一个颜色，我们就可以对颜色输出有一个精细的控制；现在我们再添加一个反光度(shininess)分量到上述三个颜色中去，这就有了我们的材质属性：   
```
#version 330 core
struct Material{
	vec3 ambient;
	vec3 diffuse;
	vec3 specular;
	float shininess;
};

uniform Material material;
```    
在片段着色器中，我们创建一个结构体来存储物体的材质属性。首先我们先声明结构体的布局(layout)，然后用刚刚创建的结构体为类型，简单的声明一个uniform变量。   
可以看到，我们给每个冯氏光照模型的分量都定义了一个颜色分量：   
- ambient: 定义了材质在环境光照下这个物体反射的是什么颜色，通常是和该物体颜色相同的颜色；   
- diffuse: 定义了材质在漫反射下物体的颜色，（和环境光照一样）漫反射颜色也要设置为我们需要的物体颜色;   
- specular: 设置的是镜面光照对物体的颜色影响（或者甚至可能反射一个物体特定的镜面高光颜色）。   
- shininess影响镜面高光的散射/半径；   
常见的材质在白光下的情况：   
![常见材质](https://learnopengl-cn.github.io/img/02/03/materials_real_world.png)  
 
### 设置材质   
我们在片段着色器中可以直接由material的属性来访问：   
```
void main()
{    
    // 环境光
    vec3 ambient = lightColor * material.ambient;

    // 漫反射 
    vec3 norm = normalize(Normal);
    vec3 lightDir = normalize(lightPos - FragPos);
    float diff = max(dot(norm, lightDir), 0.0);
    vec3 diffuse = lightColor * (diff * material.diffuse);

    // 镜面光
    vec3 viewDir = normalize(viewPos - FragPos);
    vec3 reflectDir = reflect(-lightDir, norm);  
    float spec = pow(max(dot(viewDir, reflectDir), 0.0), material.shininess);
    vec3 specular = lightColor * (spec * material.specular);  

    vec3 result = ambient + diffuse + specular;
    FragColor = vec4(result, 1.0);
}
```   
可以看到，我们访问了材质结构中的所有属性，并且根据材质的颜色计算最终的输出颜色。物体的每个材质属性都乘上了它们对应的光照分量。   
现在就可以在程序中设置适当的uniform了，GLSL在程序中设置对应的uniform进行设置，需要对每一个分量进行单独的设置，方式如下：   
```
lightingShader.setVec3("material.ambient",  1.0f, 0.5f, 0.31f);
lightingShader.setVec3("material.diffuse",  1.0f, 0.5f, 0.31f);
lightingShader.setVec3("material.specular", 0.5f, 0.5f, 0.5f);
lightingShader.setFloat("material.shininess", 32.0f);
```
![结果](https://learnopengl-cn.github.io/img/02/03/materials_with_material.png)   

### 光的属性    
这个物体太亮了。物体太郎是因为环境光、漫反射和镜面光这三个颜色对任意一个光源都会去权利反射。光源对环境光、漫反射和镜面光分量也有着不同的强度。前面的教程，我们通过一个强度值改变环境光和镜面光强度的方式解决了这个问题。我们做一个类似的系统，但是这次是为各个光照分量指定一个强度向量。我们假设lightColor是vec3(1.0)，代码会变成这样：   
```
vec3 ambient  = vec3(1.0) * material.ambient;
vec3 diffuse  = vec3(1.0) * (diff * material.diffuse);
vec3 specular = vec3(1.0) * (spec * material.specular);
```   
很明显可以看见，只要调小1.0，就会获得比较满意的效果，这里引出了类似材质属性的**光照属性**，它的定义如下：   
```
struct Light {
    vec3 position;

    vec3 ambient;
    vec3 diffuse;
    vec3 specular;
};

uniform Light light;
```   
这样可以类似的更新片段着色器和程序中对应的代码片段：   
```
vec3 ambient  = light.ambient * material.ambient;
vec3 diffuse  = light.diffuse * (diff * material.diffuse);
vec3 specular = light.specular * (spec * material.specular);
```   
```
lightingShader.setVec3("light.ambient",  0.2f, 0.2f, 0.2f);
lightingShader.setVec3("light.diffuse",  0.5f, 0.5f, 0.5f); // 将光照调暗了一些以搭配场景
lightingShader.setVec3("light.specular", 1.0f, 1.0f, 1.0f);
```   
结果perfect！   
![光照材质](https://learnopengl-cn.github.io/img/02/03/materials_light.png)


## 光照贴图   
上一章中我们将整体物体材质定义为一个整体，但是现实世界中的物体通常并不只包含一种材质，而是由多种材质组成。想象一辆汽车：外壳有光泽，车窗会反射周围的环境，车胎不会有光泽，所以它没有镜面高光。而车轮毂会很亮。这个例子反映了实际情况中，不同的部分有着不同的环境光/漫反射颜色。总之，这样的物体在不同的部件上有着不同的材质属性；   
我们在本章引入**漫反射**和**镜面光**贴图。这样，我们队物体的反射分量（以及间接的对环境分量，它们总是一样的）和镜面光分量有着更精确的控制；   

### 漫反射贴图    
我们的目标是通过某种方式，对物体的每个单独片段设置反射颜色，这就是让我们根据片段在物体上的位置来获取颜色的系统，我们已经见过类似的形式，它就是纹理；在光照场景中，它通常叫做**漫反射贴图(Diffuse Map)**，它是一个表现了物体所有漫反射颜色的纹理图像；    
构建漫反射贴图的方法和纹理教程中的方式是完全一样的，但是这次我们会将纹理存储为Material结构体中的一个sampler2D。我们将之前定义的vec3漫反射变量替换为该漫反射贴图；   
> Tip:注意sampler2D其实是所谓的不透明类型（Opaque Type，指的是内容被封装），即类型不能被实例化，只能通过uniform变量来定义它。如果使用非设定uniform变量的办法，GLSL则会抛出一些奇怪的错误。    
我们同样移除了环境光材质颜色向量，因为环境光颜色在几乎所有情况下都等于漫反射颜色，所以我们不需要将它们分开存储：   
```
struct Material {
	sampler2D diffuse;
	vec3 specular;   
	float shininess;
};

in vec2 TexCoords;
```
注意，我们要在着色器片段中再次需要纹理坐标，所以我们会声明一个额外的输入变量。接下来我们只需要从纹理中采样片段的漫反射颜色值就可以：   
```
vec3 diffuse = light.diffuse * diff * vec3(texture(material.diffuse, TexCoords));
```   
同时将环境光的材质颜色设置为漫反射材质颜色同样的值。   
```
vec3 ambient = light.ambient * vec3(texture(material.diffuse, TexCoords));
```   
这样我们就可以基本获得试用漫反射贴图的全部步骤了，别看它简单，但是能极大的提升视觉品质。为了其正常工作，我们还需要使用纹理坐标来更新顶点数据，并将它们作为顶点数据传输到片段着色器，加载材质并且绑定合适的材质到纹理单元。   
现在将更新后的数据传递到片段着色器内；   
```
#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aNormal;
layout (location = 2) in vec2 aTexCoords;
...
out vec2 TexCoords;

void main()
{
    ...
    TexCoords = aTexCoords;
}
```   
同时记得去更新两个VAO的顶点属性指针来匹配新的顶点数据，并且加载箱子图像为一个纹理。在绘制箱子之前，我们希望将要用的纹理单元赋值到material.diffuse这个uniform采样器当中，并且绑定箱子的纹理到这个纹理单元；   
```
lightShader.setInt("material.diffuse", 0);
...
glActiveTexture(GL_TEXTURE0);
glBindTexture(GL_TEXTURE_2D, diffuseMap);
```
   
### 镜面高光贴图    
你可能会注意到，镜面高光看上去有些奇怪——因为木头本身不该有这么高的镜面高光的。这里就揭示出，只有钢铁才应该有一些镜面高光的。所以我们需要让物体的某些部分以不同的强度显示镜面高光，这个问题看上去和漫反射贴图特别相似；   
我们同样可以使用一个专门用于镜面高光的纹理贴图。这也就意味着我们需要生成一个黑白的（如果你想得话也可以是彩色的）纹理，来定义物体每部分的镜面光强度。   
镜面高光的强度可以通过图像的每个像素来进行获取，镜面光贴图上的每个像素都可以由一个颜色向量来代替，例如黑色代表颜色向量vec3(0,0),灰色代表颜色向量vec3(0.5)。在片段着色器中，我们接下来取样对应的颜色值，并且将它乘以光源的镜面强度。一个像素越“白”，那么乘积会越大，物体的镜面分量就会越大。   
由于箱子大部分由木头组成，木头材质本身没有镜面高光，所以漫反射纹理的木头部分都被换成了黑色。而箱子钢制边框镜面强度是有细微变化的，钢铁本身比较容易受到镜面高光的影响，而裂缝不会。   
使用Photoshop和Gimp之类的工具将漫反射纹理转换为镜面光纹理还是比较容易的，只需要做一些剪除操作，再转个黑白，加个对比度。    
#### 采样镜面光贴图   
镜面光贴图和其它的纹理非常类似，所以代码也和漫反射贴图的代码很类似。记得要保证正确地加载图像并生成一个纹理对象。由于我们正在同一个片段着色器中使用另一个纹理采样器，我们必须要对镜面光贴图使用一个不同的纹理单元（见纹理），所以我们在渲染之前先把它绑定到合适的纹理单元上：   
```
lightingShader.setInt("material.specular", 1);
...
glActiveTexture(GL_TEXTURE1);
glBindTexture(GL_TEXTURE_2D, specularMap);
```
接下来更新片段着色器的材质属性，让其接受一个sampler2D而不是vec3作为镜面光分量：   
```
struct Material {
    sampler2D diffuse;
    sampler2D specular;
    float     shininess;
};
```
最后我们希望采样镜面光贴图，来获取片段所对应的镜面光强度：   
```
vec3 ambient  = light.ambient  * vec3(texture(material.diffuse, TexCoords));
vec3 diffuse  = light.diffuse  * diff * vec3(texture(material.diffuse, TexCoords));  
vec3 specular = light.specular * spec * vec3(texture(material.specular, TexCoords));
FragColor = vec4(ambient + diffuse + specular, 1.0);
```   
通过使用镜面光贴图我们可以对物体设置大量的细节，尤其例如物体哪些部分需要有闪闪发光的属性。我们甚至可以设置它们对应的强度。   
> 镜面光为什么没有使用实际的颜色？是因为实际当中镜面光颜色往往是由光源本身来决定大部分（甚至是全部）的，并不能生成真实的视觉效果（这就是为什么镜面光贴图通常都是白色，因为我们只关心强度）；   
通过使用漫反射和镜面光贴图，我们可以给相对简单的物体添加大量的细节。我们甚至可以使用法线/凹凸贴图(Normal/Bump Map)或者反射贴图(Reflection Map)给物体添加更多的细节，但这些将会留到之后的教程中。   

## 投光物    
我们目前使用的光照都来自空间的一个点，但是现实当中存在比较复杂的投光物(Light Caster)来进行投射(Cast)。我们分别讨论定向光(Directional Light)、点光源(Point Light)以及聚光(SpotLight)。至于如何将这些光整合到一起，我们将会在下一节讨论；   
### 平行光   
当光源处于非常远的地方时，每条来自光源的光线就会类似平行的。这样揭示了平行光源的特性：**无限远处**，它也被称为定向光，这种光与光源的位置无关，例如太阳。   
![太阳光线](https://learnopengl-cn.github.io/img/02/05/light_casters_directional.png)  
也正是因为光线是平行的，所有物体和光源的相对位置都不是最重要的，而因为对场景中每一个物体光方向一致，因为对 每个物体的光照的计算也会类似。   
我们定义一个光线方向向量而不是位置向量来模拟一个定向光。着色器的计算基本保持不变，但是这次我们直接使用光的direction向量而不是通过direction来计算lightDir向量。   
```
struct Light {
    // vec3 position; // 使用定向光就不再需要了
    vec3 direction;

    vec3 ambient;
    vec3 diffuse;
    vec3 specular;
};
...
void main()
{
  vec3 lightDir = normalize(-light.direction);
  ...
}
```   
注意这里我们对light.direction的向量进行了取反，我们目前需要的是一个**从片段到光源**的方向向量，但是人为的更习惯于定义一个**从光源到片段**的全局方向。记得对其进行标准化。   
结果如图：   
![平行光源效果](https://learnopengl-cn.github.io/img/02/05/light_casters_directional_light.png)   
> Tip:齐次向量再次在这里起到一个神奇的功效——很多人喜欢吧光的位置向量定义为vec4，这样的话潜在带来的问题就是，如果表示位置向量，需要把w分量表示为1.0，这样变换和投影的时候才能够正确应用。但是如果定义一个方向向量为vec4时，我们不想让位移很有任何效果，所以w分量会为0.0。因此不言自明的w分量的一个作用——区分位置向量和方向向量。（用它来进行定向光照和位置光源的方法，这正是老版本OpenGL使用的判断方法）   

### 点光源   
现实中同样常见的是点光源，点光源会朝着所有方向放光，但是光强会随着距离逐渐衰减，例如灯和火把。   
![点光源](https://learnopengl-cn.github.io/img/02/05/light_casters_point.png)   
我们先前绘制过lamp，唯一的问题在于，它是个类似恒定光源的光源，永远不会衰减。而更逼真的模拟，应该是光源仅仅照亮附近的区域，而不是所有的区域。这里我们就需要先讨论点光源的衰减规律。   
#### 衰减   
随着光线传播距离增长而逐渐削减光的强度，我们称之为衰减(Attenuation)。最简单的形式是用一个线性公式表示光强的衰减。但是实际当中，我们的灯光普遍遵循着近处极亮，但是随着距离的增加的光源亮度一开始下降极快，在远处又下降速率衰减。我们需要另一种描述灯光光强的方式。   
![screenShot.png](https://i.loli.net/2019/02/19/5c6bbe32adb0b.png)   
三个参数：常数项Kc、一次项Kl和二次项Kq。   
- 常数项为1.0，主要避免分母比1小，出现光强增大的情况；   
- 一次向与距离相乘，以线性减少光强。   
- 二次项与距离平方相乘，让光源以二次递减的方式减少强度。二次项在距离很小的情况影响比一次项小得多，但是较大时则有较明显的影响。   
其大约的效果如下图所示：   
![点光源衰弱](https://learnopengl-cn.github.io/img/02/05/attenuation.png)   

#### 选择正确的值   
常见的三参数值表如下(覆盖范围)：   
距离|常数项|一次项|二次项
-------|------|-----|------
7|1.0|0.7|1.8
13|1.0|0.35|0.44
20|1.0|0.22|0.20
32|1.0|0.14|0.07
50|1.0|0.09|0.032
65|1.0|0.07|0.017
100|1.0|0.045|0.0075
160|1.0|0.027|0.0028
200|1.0|0.022|0.0019
325|1.0|0.014|0.0007
600|1.0|0.007|0.0002
3250|1.0|0.0014|0.000007   
#### 实现衰减   
为了实现衰减，在片段着色器中我们需要公式当中的常数项、一次项和二次项。应当都存储在Light结构体当中。这里我们换成使用位置光的算法，而不是平行光。   
```
struct Light {
    vec3 position;  

    vec3 ambient;
    vec3 diffuse;
    vec3 specular;

    float constant;
    float linear;
    float quadratic;
};
```   
我们通过获取片段和光源间的向量差，并且获取结果向量的长度作为距离项，这个可以借助GLSL的length函数来完成：   
```
float distance = length(light.position - FragPos);
float attenuation = 1.0 / (light.constant + light.linear * distance + light.quadratic * (distance * distance));
```   
![衰减之后的光线的情况](https://learnopengl-cn.github.io/img/02/05/light_casters_point_light.png)   

### 聚光   
最后一种需要进行讨论的光的类型是聚光(SpotLight)。聚光是位于环境中某个位置的光源，它只朝着一个特定的方向而不是所有方向照射光线。这样带来的结果就是只在聚光方向的特定半径内物体才会被照亮。






