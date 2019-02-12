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
