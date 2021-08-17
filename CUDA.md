# CUDA学习笔记
1. 一个典型的CUDA编程结构包含：
> 1. 分配GPU内存；
> 2. 从CPU内存中拷贝数据到GPU；
> 3. 调用CUDA内核函数来完成程序制定的运算；
> 4. 将数据从GPU拷贝回CPU内存；
> 5. 释放GPU空间；
   
2. 数据的局部性：```数据局部性```是存在于并行编程中的一个重要概念，数据局限性指的是数据重用，来降低内存访问的延迟。数据的局部性分时间局限性（相对较短时间内数据或资源的重用）和空间局限性（相对较近的存储空间内数据元素重用）两种。
内存层次和线程层次则是CUDA中的数据局限性的体现；前者例如有共享内存，通过共享内存可以直接控制代码局部性；   

3. CUDA努力抽象了硬件细节，摆脱了应用程序映射到传统API上。CUDA中有三个抽象：
> - 线程组层次结构
> - 内存层次结构
> - 障碍同步

4. ![20210802180703](https://i.loli.net/2021/08/02/UpnScyLxvVYlh4C.png)   
以上图为程序与编程模型之间的抽象结构。   
并行计算可以从以下三个层面来看待：   
> - 领域层：如何解析数据，以便在并行运行环境中能够正确高效地解决问题；
> - 逻辑层：重点转向如何组织并发线程；    
> - 硬件层

5. CUDA编程结构：每个GPU和CPU内存都由一条PCIE总线分隔开，因此需要区分以下内容：   
> 主机：CPU及其内存（主机内存），h_为前缀
> 设备：GPU及其内存（设备内存），d_为前缀    
内核（Kernal）是CUDA模型的一个重要组成部分，其代码是要在GPU上运行。开发者可以串行执行该函数；
绝大多数情况下，主机可以独立地对设备进行操作。内核一旦被启动，管理权会被返回给主机，释放CPU来执行有设备上运行的代码来实现的额外的任务。CUDA的编程模型主要是异步的，因此GPU上进行的运算可以和主机-设备通信重叠。一个典型的CUDA程序包含由并行代码互补的串行代码。   
![20210803092014](https://i.loli.net/2021/08/03/1UMEvGnjQeiyxaX.png)   
如上图，串行代码（及任务并行代码）在主机CPU上执行，而并行代码在GPU上执行。主机代码按照ANSI C标准进行编写，而设备代码使用CUDA C进行编写。代码可以放在一个文件里，也可以放在不同的源文件里由nvcc来进行编译；   

6. 内存管理：cudaMalloc函数原型为   ``cudaError_t cudaMalloc {void** devPtr, size_t size}``    

 标准的C函数  |  CUDA C函数 | 标准的C函数 | CUDA C函数    
:--:|:--:|:--:|:--:   
malloc|cudaMalloc|memset|cudaMemset   
memcpy|cudaMemcpy|free|cudaFree   

该函数负责向设备分配一定字节的线性内存，并以devPtr的形式返回指向所分配内存的指针。   
cudaMemcpy函数负责主机和设备之间的数据传输，其函数原型为：   
``cudaError_t cudaMemcpy{void* dat, const void* src, size_t count, cudaMemcpyKind kind}``   
此函数从src指向的源存储区复制一定数量的字节到dst指向的目标存储区，复制方向由Kind指定，有以下几种：
```
cudaMemcpyHostToHost
cudaMemcpyHostToDevice
cudaMemcpyDeviceToHost
cudaMemcpyDeviceToDevice
```
该函数是通过同步方式执行，因为在cudaMemcpy函数返回以及传输操作完成之前主机应用程序是阻塞的。除了内核启动之外的CUDA调用都会返回一个枚举类型cudaError_t。如果GPU内存分配成功，函数返回``cudaSuccess``,否则返回``cudaErrorMemoryAllocation``，可以用``char* cudaGetErrorString (cudaError_t error)``将错误代码转换为错误消息。   
CUDA编程模型从GPU架构中抽象出一个内存层次结构，如下图所示，包含全局内存和共享内存，前者类似CPU的系统内存，而后者类似CPU的缓存，但是GPU的共享内存可由CUDA C内核直接控制。     
![20210803095616](https://i.loli.net/2021/08/03/PJEnbo1iTe4HDAl.png)     
尤其要注意CUDA C进行编程的人最常犯的错误就是对不同内存空间的不恰当引用，尤其是对于GPU上被分配的内存来说，设备指针在主机代码中可能并没有被引用，如果进行了错误的内存分配，例如``gpuRef = d_C``，而不用``cudaMemcpy(gpuRef, d_C, nBytes, cudaMemcpyDeviceToHost)``应用程序在运行时将会崩溃。所以为了避免这个错误，CUDA 6.0提出统一寻址，会使用一个指针来访问CPU和GPU的内存。   

7. 线程管理：当核函数在主机端启动，它的执行会移动到设备上，此时设备中会产生大量的线程并且每个线程都执行有核函数制定的语句。   
![20210803111438](https://i.loli.net/2021/08/03/eMsxWH1r2uYPDXE.png)    
CUDA明确了线程层次抽象概念以方便于组织线程，这是一个两层的线程层次结构，由线程块和线程块网络构成。   
由一个内核所启动所有线程称作为一个网格，同一个网格中的所有线程共享相同的全局内存空间。一个网格由多个线程块构成，而一个线程块包含一组线程，同一个线程块内的线程写作可作为以下方式实现：
> 同步
> 共享内存
而不同块内的线程是不能协作的，而线程依靠以下两个坐标变量来区分彼此：   
> -blockldx(线程块在线程格内的索引)
> -threadldx(块内的线程索引)   
这些变量是核函数中需要预初始化的内置变量。当执行一个核函数时，CUDA运行时为每个线程分配坐标变量blockldx和threadldx。基于这些坐标，可以将部分数据分配给不同的线程。这个坐标都是基于uint3(3个无符号整形数据组成)来定义，可通过x、y、z三个字段来制定。   
```
blockIdx.x
blockIdx.y
blockIdz.z
threadIdx.x
threadIdx.y
threadIdx.z
```   
CUDA可以组织三维的网络和块，图2-5展示了一个线程层次结构的示例，它的结构是一个包含二位块的二位网格。而网格和块的维度由以下两个内置变量指定。   
> blockDim(线程块的维度，用每个线程块中线程数来表示)
> gridDim(线程格的维度，用每个线程格中的线程数来表示)
它们是dim3类型的变量，是基于uint3定义的整数型向量，用来表示维度。当定义一个dim3类型的变量时，所有未制定的元素都被初始化为1。dim3类型变量中的每个组件可以通过它的x、y、z字段获得，类似：
```
blockDim.x
blockDim.y
blockDim.z
```   
通常一个线程格(Grid)会被组织成线程块的二维数组形式，一个线程块（Block）会被组成成线程的三维数组形式。线程格和线程块均使用3格dim3类型的无符号整形字段，而未使用的字段将会被初始化为1且忽略不计。   
在CUDA程序中由两组不同的网格和块变量，手动定义的dim3数据类型和预定义的uint3数据类型。在主机端，作为内核调用的一部分，你可以使用dim3数据类型定义一个网格和块的维度。当执行核函数时，CUDA运行时会生成相应的内预置初始化的网格、块和线程变量，它们在核函数内均可以访问到且为unit3类型。手动定义的dim3类型网格和快变量仅在主机端课件，而unit3类型的内置初始化的网格和块变量仅在设备端可见。    
以下为一个例子：
```
int nElem = 6;
//定义块的尺寸，并基于块和数据的大小计算网格尺寸
//下面例子定义了一个包含3个线程的一维线程块，以及1个基于块和数据大小定义的一定数量线程块的一维线程网格
dim3 block(3);
dim3 grid((nElem + block.x - 1) / block.x);
```
你会发现网格的大小是块大小的倍数。在下一章中你必须了解这样计算网格大小的原因。以下为完整例子：   
```cpp
#include <cuda_runtime.h>
#include <stdio.h>

__global__ void checkIndex(void) {
    printf("threadIdx:(%d, %d, %d) blockIdx:(%d, %d, %d) blockDim(%d, %d, %d) "
    "gridDim:(%d, %d, %d)\n", threadIdx.x, threadIdx.y, threadIdx.z,
    blockIdx.x, blockIdx.y, blockIdx.z, blockDim.x, blockDim.y, blockDim.z, 
    gridDim.x, gridDim.y, gridDim.z);
}

int  main(int argc, char **argv) {
    //define total data element 
    int nElem = 6;
    
    //define grid and block structure
    dim3 block(3);
    dim3 grid ((nElem + block.x - 1) / block.x);

    //check grid and block from host side 
    printf("grid.x %d grid.y %d grid.z %d\n", grid.x, grid.y, grid.z);
    printf("block.x %d block.y %d block.z %d\n", block.x, block.y, block.z);

    //check grid and block dimension from device side 
    checkIndex <<<grid, block>>> ();
    
    //reset device before you leave 
    cudaDeviceReset();

    return 0;
}
```   
结果为：   
```
grid.x 2 grid.y 1 grid.z 1
block.x 3 block.y 1 block.z 1
threadIdx:(0, 0, 0) blockIdx:(0, 0, 0) blockDim(3, 1, 1) gridDim:(2, 1, 1)
threadIdx:(1, 0, 0) blockIdx:(0, 0, 0) blockDim(3, 1, 1) gridDim:(2, 1, 1)
threadIdx:(2, 0, 0) blockIdx:(0, 0, 0) blockDim(3, 1, 1) gridDim:(2, 1, 1)
threadIdx:(0, 0, 0) blockIdx:(1, 0, 0) blockDim(3, 1, 1) gridDim:(2, 1, 1)
threadIdx:(1, 0, 0) blockIdx:(1, 0, 0) blockDim(3, 1, 1) gridDim:(2, 1, 1)
threadIdx:(2, 0, 0) blockIdx:(1, 0, 0) blockDim(3, 1, 1) gridDim:(2, 1, 1)
```    
默认情况下，nvcc是会产生最低版本GPU架构的代码。可以从上述内容看到，每个线程都有自己的坐标，但所有线程都由相同的块维度和网格维度。   

下面讨论如何从主机端和设备端访问网络/块变量：
区分以上两者非常重要，例如主机端的块变量，可按照以下内容对其进行访问:   
```
block.x, block.y, and block.z
```   
在设备端，则已经预定义了内置块变量的大小：   
```
blockDim.x, blockDim.y, and blockDim.z
```   
总之，在启动内核之前就定义了主机端的网格和块变量，并从主机端通过由x、y、z三个字段决定的矢量结构来访问它们。当内容启动时，可以使用内核中预初始化的内置变量。   
对于一个给定的数据大小，确定网格和块尺寸的一般步骤为：
- 确定块的大小
- 在已知数据大小和块大小的基础上计算网络维度。要确定块尺寸，通常要考虑：
    - 内核的性能特征
    - GPU资源的限制
下面给出了一个使用一维网格和一个一维块来说明块大小改变时，网格的尺寸也会随之改变：
![20210803173232](https://i.loli.net/2021/08/03/7rRQfCAgJIn3NLS.png)   
![20210803173358](https://i.loli.net/2021/08/03/HydipLSsT1MF5rI.png)

下面讨论线程层次结构：
CUDA的特点就是通过编程模型揭示了一个两层的线程层次结构。由于一个内核启动的网格和块的维数会影响性能。而网络和块的维度存在几个限制因素，最主要的一个限制因素就是可利用的计算资源：寄存器、共享内存等。某些限制可以通过查询GPU设备返回。   

8. 启动一个CUDA核函数   
C语言函数调用是``function_name(argument list);``而CUDA内核调用是对C预研函数调用语句的衍生，**<<<>>>**运算符内是核函数的执行配置``kernel_name <<<grid, block>>>(arguement list);``执行配置第一个值是网格维度，也就是启动块的数目。第二个值是块维度，也就是每个块中线程的数目。通过指定网格和块的维度，可以配置：   
- 内核中线程的数目
- 内核中使用的线程布局
**同一个块中的线程之间可以相互协作，不同块内的线程不能协作**。对于一个给定的问题，可以使用不同的网格和块布局来组织线程。例如，如果有32个数据元素用于计算，每8个元素一个块，需要启动4个块：``kernal_name<<<4, 8>>>(arguement list);``下面这个图说明了线程布局：   
![20210803174946](https://i.loli.net/2021/08/03/DHOQpohYyqljdsT.png)   
由于数据在全局内存中是线性存储的，因此可用变量blockIdx.x和threadIdx.x来进行以下操作：   
- 在网格中标识一个唯一的线程；
- 建立线程和数据元素之间的映射关系；   
同理，建立一个块里有32个元素``kernel_name<<<1, 32>>>(argument list);``,或者每个块都只有一个元素``kernel_name<<<32, 1>>>(argument list);``   
**核函数的调用与主机线程是异步的**，核函数调用结束以后，控制权立刻返回给主机端，可以调用``cudaError_t cudaDeviceSynchronize(void);``来强制主机端程序等待所有的核函数执行结束。   
而有些CUDA运行的API是隐式同步的（主机和设备之间），例如cudaMemcpy，主机端隐式同步，即主机端程序必须等待数据拷贝完成之后才能继续执行程序；   

下面讨论异步行为：   
不同于C语言函数调用，所有CUDA核函数启动都是异步的。CUDA内核调用完成后，控制权会立刻返回给CPU；

下面讨论核函数的编写：
类似之前讨论，核函数的声明是用形式：``__global__ void kernel_name(argument list);``，而**其必须有一个返回类型void**,下表对函数类型限定符做了一个总结：   
限定符| 执行| 调用| 备注
:--:|:--:|:--:|:--:
__global__ | 在设备端执行 | 可从主机端调用，也可以从计算能力为3的设备中调用 | 必须有一个void返回类型   
__device__ | 在设备端执行 | 仅能从设备端调用 |   
__host__   | 在主机端执行 | 仅能从主机端调用 | 可省略   

其中__device__和__host__可以一起使用，这样函数可以同时在主机和设备端进行编译   

CUDA核函数的限制： 
> 只能访问设备内存；
> 必须有void返回类型；
> 不支持可变数量的参数；
> 不支持静态变量；
> 显示异步行为；

这里再回到我们一开始的例子，将两个向量大小为N的向量A和B相加，主机端的向量加法如下： ![20210803185023](https://i.loli.net/2021/08/03/gNHTWJuQXjlUoh3.png)   
然后换成普通核函数计算，会变成： 
```
__global__ void sumArraysOnGPU(float *A, float *B, float *C){
    int i = threadIdx.x;
    C[i] = A[i] + B[i];
}
```   
很明显，循环体消失了，内置的线程坐标替换了数组索引，由于N是隐式被定义来启动N个线程的，所以N并没有什么参考价值；

9. 验证核函数：一般会写一个主机函数来验证核函数的结果，例如：   
![20210803185545](https://i.loli.net/2021/08/03/IKn9hABSzF5iPaX.png)
此外可以使用printf、设置参数为<<<1,1>>>来调试结果；   

10. 处理错误：由于CUDA的调用是异步的，这导致可能很确定某个错误是由哪一步导致的，故定义一个错误处理宏封装所有的CUDA API调用是很有用的：   
![20210803185831](https://i.loli.net/2021/08/03/UjbHpecuGvAa3Oo.png)   
例如，可以使用该宏去检查拷贝是否错误： ``CHECK(cudaMemcpy(d_C, gpuRef, nBytes, cudaMemcpyHostToDevice))``，如果出错则直接exit并报出错误信息；另一个例子是:   
![20210803190044](https://i.loli.net/2021/08/03/RV2S3Hnq5wuvfPU.png)   
会阻塞主机端线程运行直到所有请求任务都结束。但是注意，如果处理不好，因为阻塞主机端线程，该检查点会成为整个程序的全局障碍。   
以下为完整的例子：   
```cpp
#include <cuda_runtime.h>
#include <stdio.h>

#define check(call)                                                      \
{                                                                        \
    const cudaError_t error = call;                                      \
    if(error!=cudaSuccess)                                               \
    {                                                                    \
        printf("Error: %s:%d, "__FILE__,__LINE__);                       \
        printf("code:%d, reason: %s\n",error,cudaGetErrorString(error)); \
        exit(1);                                                         \
    }                                                                    \
}

void checkResult(float *hostRef, float *gpuRef, const int N)
{
	double epslion = 1.0E-8;
	bool match = 1;
	for (int i = 0; i < N; i++) {
		if (abs(hostRef[i] - gpuRef[i]) > epslion) {
			match = 0;
			printf("Arrays do not match!\n");
			printf("host %5.2f gpu %5.2f at current %d\n", hostRef[i], gpuRef[i], i);
			break;
		}
	}
	if (match) printf("Arrays match.\n\n");
}

void initialData(float *ip, int size) {
	//生成随机数据
	time_t t;
	srand((unsigned int) time(&t));

	for (int i = 0; i < size; i++)
	{
		ip[i] = (float)(rand() & 0xFF) / 10.0f;
	}
}

void sumArraysOnHost(float *A, float *B, float *C, const int N) {
	for (int idx = 0; idx < N; idx++)
		C[idx] = A[idx] + B[idx];
}

__global__ void sumArraysOnGPU(float *A, float *B, float *C) {
    int i = threadIdx.x;
    C[i] = A[i] + B[i];

    /*
    //一般情况下，可基于给定的一维网格和块的信息来计算全局数据访问的唯一索引
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    C[i] = A[i] + B[i];
    */
}

int main(int argc, char **argv) {
    printf("%s Strating..\n", argv[0]);

    //set up device
    int dev = 0;
    cudaSetDevice(dev);

    //set up data size of vectors
    int nElem = 32;
    printf("Vector size %d\n", nElem);

    //malloc host memory
    size_t nBytes = nElem * sizeof(float);

    float *h_A, *h_B, *hostRef, *gpuRef;
    h_A     = (float*)malloc(nBytes);
    h_B     = (float*)malloc(nBytes);
    hostRef = (float*)malloc(nBytes);
    gpuRef  = (float*)malloc(nBytes);

    //initialize data at host side 
    initialData(h_A, nElem);
    initialData(h_B, nElem);

    memset(hostRef, 0, nBytes);
    memset(gpuRef,  0, nBytes);

    //malloc device global memory
    float *d_A, *d_B, *d_C;
    cudaMalloc((float**)&d_A, nBytes);
    cudaMalloc((float**)&d_B, nBytes);
    cudaMalloc((float**)&d_C, nBytes);

    //transfer data from host to device
    cudaMemcpy(d_A, h_A, nBytes, cudaMemcpyHostToDevice);
    cudaMemcpy(d_B, h_B, nBytes, cudaMemcpyHostToDevice);

    //invoke kernel at host side 
    dim3 block (nElem);
    dim3 grid  (nElem / block.x);

    sumArraysOnGPU<<<grid, block>>>(d_A, d_B, d_C);
    printf("Execution configuration <<<%d, %d>>>\n", grid.x, block.x);

    //copy kernel result back to host side
    cudaMemcpy(gpuRef, d_C, nBytes, cudaMemcpyDeviceToHost);

    //add vector at host side for result checks
    sumArraysOnHost(h_A, h_B, hostRef, nElem);

    //check device results
	checkResult(hostRef, gpuRef, nElem);

    //free device global memory
    cudaFree(d_A);
    cudaFree(d_B);
    cudaFree(d_C);

    //free host memory
    free(h_A);
    free(h_B);
    free(hostRef);
    free(gpuRef);

    return 0;
}
```   

11. 给核函数计时：通常使用gettimeofday来创建一个CPU计时器，以获取系统的时钟时间，它会返回1970年1月1日零点以来到现在的秒数，其代码如下：   
```
double cpuSecond() {
    struct timeval tp;
    gettimeofday(&tp, NULL);

    return ((double)tp.tv_sec + (double)tp.tv_usec * 1.e-6);
}

double iStart = cpuSecond();
kernel_name<<<grid, block>>>(argument list);
cudaDeviceSynchronize();
double iElaps = cpuSecond() - iStart;
```   
需要使用cudaDeviceSynchronize函数来等待所有GPU线程运行结束，而最后iElaps就是表示运行的时间。   
由于GPU的可扩展性，需要借助块和线程的索引来计算一个按行有优先的数组索引i，并且对核函数进行修改，添加限定条件来检索索引值是否越界。
```
int i = blockIdx.x * blockDim.x + threadIdx.x;
if (i < N) C[i] = A[i] + B[i];
```
有了这些更改，局可以使用不同的执行配置来衡量核函数。为了解决创建的线程总数大于向量元素总数的情况，你需要限制内核不能非法访问全局内存。   
![20210804154632](https://i.loli.net/2021/08/04/8C6x1GzFQdjYsAi.png)   
在调整执行配置时了解的一个关键点是对网格和块维度的限制。线程层次结构中每个层级的最大尺寸取决于设备，CUDA提供了通过查询GPU来了解这些限制的能力，后文有详细的介绍。   
**对于Fermi设备，每个块最大线程数是1024，且网格的x、y、z三个方向上的维度最大都为65535。**    

除了上述的方法，CUDA（5.0向后）提供了nvprof指令来进行来获取CPU和GPU的活动信息，包含内核执行、内存传输以及CUDA API的调用，其方法如下：    
```
nvprof [nvprof_args] <application> [application_args]
```
可以很明显的注意到，我们传输数据的时间，是远大于数据的处理时间的：   
![20210804173542](https://i.loli.net/2021/08/04/yhvHT83VXWj26Sq.png)   
对于HPC(High Performance Computer，高性能工作电脑)工作负载，理解程序中的通信比的计算非常重要好。如果你的应用程序计算时间大于数据传输所用的时间，那么或许可以压缩这些操作，并且完全隐藏与数据传输有关的延迟。如果应用程序用于计算的时间少于数据传输所用的时间，那么需要尽量减少主机和设备之间的传输。CUDA在后文中介绍CUDA流和事件来压缩计算量和通信量。绝大部分HPC工作负载是受到内存带宽的限制。    

12. 组织并行线程：前面的例子已经可看出合适的网格和块大小对内核性能有着巨大的影响。前文里面使用基于块大小和向量数据大小计算出了网格大小，实现了最佳性能。   
现在通过矩阵加法的例子来说明，我们从以下三个角度：
- 由二维线程块构成的二维网格
- 由一维线程块构成的一维网络
- 由一维线程块构成的二维网格   

通常情况下，一个矩阵用行优先的方法在全局内存中进行线性存储。下面给出了一个8*6矩阵的小例子   
![20210805103218](https://i.loli.net/2021/08/05/QLMihSTm9ke4CtY.png)   
在加法核函数中，一个线程通常被分配一个数据元素处理，首先就是使用块和线程索引从全局内存中访问指定的数据。通常对于一个二维示例，需要管理三种索引：   
- 线程和块索引
- 矩阵中给定点的坐标
- 全局线性内存中的偏移量   
对于一个给定的线程，首先可以通过把线程和块索引映射到矩阵坐标上来获取线程块和线程索引的全局内存偏移量，然后将这些矩阵坐标映射到全局内存的存储单元中。    
第一步，利用以下公式把线程和块索引映射到矩阵坐标上：   
```
ix = threadIdx.x + blockIdx.x * blockDim.x
iy = threadIdx.y + blockIdx.y * blockDim.y
```   
第二步，利用以下公式把矩阵坐标映射到全局内存中的索引/存储单位上：   
```
idx = iy * nx + ix
```   
以下说明了块和线程索引、矩阵坐标以及线性全局内存索引之间的对应关系：   
![20210805104418](https://i.loli.net/2021/08/05/5qsSiHzlycamYFA.png)   

下面使用一个二维网格和二维块进行求和操作：   
其内存映射关系如下：   
![20210805132531](https://i.loli.net/2021/08/05/wAhn28bQlptRExM.png)   
代码如下： 
```cpp
#include "cuda_runtime.h"
#include "stdio.h"
//#include "sys/time.h"
#include "time.h"

#define CHECK(call)                                                             \
{                                                                               \
    const cudaError_t error = call;                                             \
    if (error != cudaSuccess){                                                  \
        printf("Error: %s: %d, ", __FILE__, __LINE__);                          \
        printf("code: %d, reason: %s \n", error, cudaGetErrorString(error));    \
        exit(1);                                                                \
    }                                                                           \
}

double cpuSecond(){
    // struct timeval tp; 
    // gettimeofday(&tp, NULL);
    // return ((double)tp.tv_sec + (double)tp.tv_usec*1.e-6);

    return clock();
}

void printMatrix(float *C, const int nx, const int ny){
    float *ic = C; 
    printf("\n Matrix: (%d, %d)\n", nx, ny);
    for(int iy=0; iy<ny; iy++){
        for(int ix=0; ix<nx; ix++){
            printf("%.3f ", ic[ix]);
        }
        ic += nx; 
        printf("\n");
    } 
    printf("\n");
}

void checkResult(float *hostRef, float *gpuRef, const int N){
    double epsilon = 1e-8;
    bool match = 1; 
    for(int i=0; i<N; i++){
        if(abs(hostRef[i] - gpuRef[i]) > epsilon) {
            match = 0; 
            printf("Arrays do not match! \n");
            printf("host %5.2f gpu %5.2f at current %d \n", hostRef[i], gpuRef[i], i);
            break;
        }
    }
    if (match) { 
        printf("Arrays match! \n");
    }
}

void initialData(float *ip, int size){
    // Generate different seed for random number
    time_t t; 
    srand((unsigned int) time(&t));
    for(int i=0; i<size; i++){
        ip[i] = (float)(rand() &0xFF) /10.0f; 
    }
}

void sumMatrixOnHost(float *A, float *B, float*C, const int nx, const int ny){
    float *ia = A;
    float *ib = B;
    float *ic = C;
    
    for(int iy=0; iy<ny; iy++){
        for(int ix=0; ix<nx; ix++){
            ic[ix] = ia[ix] + ib[ix];
        }
        ia += nx; 
        ib += nx;
        ic += nx;
    }
}

__global__ void sumMatrixOnDevice(float *MatA, float *MatB, float *MatC, const int nx, const int ny){
    unsigned int ix = threadIdx.x + blockIdx.x*blockDim.x; 
    unsigned int iy = threadIdx.y + blockIdx.y*blockDim.y; 
    unsigned int idx = iy*nx + ix;  

    // Whether remove this if ? 
    if(ix < nx && iy < ny){
        MatC[idx] = MatA[idx] + MatB[idx];
    }
}

int main(){
    printf("Starting... \n");

    // set up device 
    int dev = 0; 
    cudaDeviceProp deviceProp;
    CHECK(cudaGetDeviceProperties(&deviceProp, dev));
    printf("Using Device %d:  %s \n", dev, deviceProp.name);
    cudaSetDevice(dev);

    // set matrix dimension 
    int nx = 1<<14;
    int ny = 1<<14; 
    int nxy = nx*ny; 
    int nBytes = nxy*sizeof(float);

    // malloc host memory 
    float *h_A, *h_B, *host_ref, *gpu_ref; 
    h_A = (float *)malloc(nBytes);
    h_B = (float *)malloc(nBytes);
    host_ref = (float *)malloc(nBytes);
    gpu_ref = (float *)malloc(nBytes);


    // initialize host matrix
    initialData(h_A, nxy);
    // printMatrix(h_A, nx, ny);

    initialData(h_B, nxy);
    // printMatrix(h_B, nx, ny);

    double iStart = cpuSecond();
    sumMatrixOnHost(h_A, h_B, host_ref, nx, ny);
    double iElaps = cpuSecond();
    double cpu_elapse = iElaps-iStart;
    printf("SumMatrixOnCPU2D elapsed %f sec \n", cpu_elapse);
    // printMatrix(host_ref, nx, ny);
    // malloc device memory
    float *d_MatA, *d_MatB, *d_MatC;
    cudaMalloc((void **)&d_MatA, nBytes);
    cudaMalloc((void **)&d_MatB, nBytes);
    cudaMalloc((void **)&d_MatC, nBytes);

    // transfer data from host to device 
    cudaMemcpy(d_MatA, h_A, nBytes, cudaMemcpyHostToDevice);
    cudaMemcpy(d_MatB, h_B, nBytes, cudaMemcpyHostToDevice);

    // set up execution configuration
    dim3 block(32, 16); 
    // dim3 block(32, 32); 
    // dim3 block(16, 16); 

    dim3 grid((nx+block.x-1)/block.x, (ny+block.y-1)/block.y);

    iStart = cpuSecond();
    //invoke the kernel
    sumMatrixOnDevice<<<grid, block>>>(d_MatA, d_MatB, d_MatC, nx, ny);
    cudaDeviceSynchronize();
    iElaps = cpuSecond();
    double gpu_elapse = iElaps - iStart ; 
    printf("SumMatrixOnGPU2D <<<grid(%d %d), block(%d, %d)>>> elapsed %f sec \n", grid.x, grid.y, \
            block.x, block.y, gpu_elapse);
    printf("The accelerate: %f \n", cpu_elapse/gpu_elapse);
    cudaMemcpy(gpu_ref, d_MatC, nBytes, cudaMemcpyDeviceToHost);
    // printMatrix(gpu_ref, nx, ny);
    // free host and device memory 
    cudaFree(d_MatA);
    cudaFree(d_MatB);
    cudaFree(d_MatC);

    free(h_A);
    free(h_B);
    free(host_ref);
    free(gpu_ref);
    cudaDeviceReset();
    return 0;
}
```   
![20210805155726](https://i.loli.net/2021/08/05/ng1yWuUht9mFjXl.png)
![20210805161919](https://i.loli.net/2021/08/05/GELxpHRYTalQmNP.png)   


### 3.2.1 线程束和线程块    
一个线程块中线程束的数量=向正无穷取整(一个线程块中线程的数量/线程束大小)    

从逻辑角度，线程块是线程的集合，它们可以被组织为一维、二维或者三维布局。   
从硬件角度，线程块是一维线程束的集合。在线程块中线程被组织成一维布局，每32个连续线程组成一个线程束。   

### 3.2.2 线程束分化   
为了获取最佳性能，应避免同一线程束当中有不同的执行路径。
分支效率被定义未分化的分支与全部分支之比，用下列公式计算：   
$$ 分支效率=100*(分支数-分化分支数)/分支数 $$   
100%即为最高分支效率。   
一段很长的代码路径一定会导致线程分束。   
   
### 3.2.3 资源分配   
对于一个给定的内核，同时存在于同一个SM中的线程块和线程束的数量取决于在SM中可用的且内核所需的寄存器和共享内存的数量。   
故在CUDA编程中需要关注计算资源的分配：计算资源限制了活跃的线程束的数量，因此必须了解由硬件产生的限制和内核用到的资源。为了最大程度利用GPU，需要最大化活跃的线程束数量。    

### 3.2.4 延迟隐藏   
在指令发出与完成之间的时钟周期被称作为指令延迟。当每个时钟周期中所有的线程调度器都有一个符合条件的线程束时，可以达到计算资源的完全利用。这就可以保证通过在其他常驻线程束中发布其他指令，可以隐藏每个指令的延迟。   
考虑至零延迟，可以被分为两个类型：   
- 算术指令：10~20周期   
- 内存指令：400~800周期   
利特尔法则提供一个合理的近似值用于估算隐藏延迟所需要的活跃线程束的数量：   
$$所需的线程束数量=延迟*吞吐量$$   

带宽通常描述单位时间内最大可能的数据传输量，而吞吐量是用来描述单位时间内任何形式的信息或操作的执行速度，例如每个周期完成多少个指令。   
有两种方法提高并行：    
- 指令级并行（ILP）：一个线程中有很多独立的指令   
- 线程级并行（TLP）：很多并发的符合条件的线程
而对内存操作来说，其所需的并行可以表示为在每个周期内隐藏内存延迟所需的字节数。   

网格和线程块大小的准则：   
- 保持每个块中线程数量是线程束大小(32)的倍数；   
- 避免块大小：每个块至少有128或者256个线程；   
- 根据内核资源的需求调整块大小；   
- 块的数量要远远多于SM的数量，从而在设备中显示有足够的并行；   
- 通过实验得到最佳执行配置和资源的使用情况；   

### 3.2.6 同步   
前文已讲到通常使用``cudaError_t cudaDeviceSynchronize(void);``来实现阻塞主机完成所有的CUDA操作。在线程块中，CUDA提供了一个使用块局部栅栏来同步他们执行的功能：``__device__ void __syncthreads(void);``   
线程块中的线程可通过共享内存和寄存器来共享数据。当线程之间共享数据时，要避免竞争条件。例如线程间的无序读写。不同块中的线程不允许相互同步，因此GPU可以以任意顺序执行块，这使得CUDA在大规模并行GPU上是可扩展的。    
     
### 3.2.7 可扩展性   
![20210810185934](https://i.loli.net/2021/08/10/jfUWOtC1vkumbZS.png)   

### 3.3.3 增大并行性   
最好的配置既不具有最高的可实现占用率，也不具有最高的吞吐量   

## 3.4避免分支分化   
### 3.4.1 并行规约问题   
并行规约问题设计用于解决大规模运算（类似于分冶）：
- 将输入向量划分到更小的数据库中；    
- 用一个线程计算一个数据块的部分和；   
- 对每个数据块的部分和再求和得出最终结果；
可分为相邻配对（log2N）与交错配对两种方式：    
![](https://files.catbox.moe/d4cyq6.png)   
下面是个交错配对法c的实现：   
```cpp
int recursiveReduce(int *data, int const size)
{
    //terminate check
    if (size == 1) return data[0];

    //renew the stride
    int const stride = size / 2;
    
    //in-place reduction   
    for (int i = 0; i < stride; i++) {
        data[i] += data[i + stride];   
        //这里满足交换律和结合律的任意运算都可以替代加法，例如max求最大值、min、平均值、乘积
        //f在想两种执行满足交换律和结合律的运算，被称为规约问题。
    }

    //call recursively 
    return recursiveReduce(data, stride);
}
```    

### 3.4.2 并行规约中的分化   
以下是一个相邻配对法的核函数写法（一个大数组用来存整个数组，进行规约，拎一个小数组用来存放每个线程块的和。__syncthreads保证线程块中任意一个线程在进入下一次迭代之前，在当前迭代里每个线程的所有部分和都被保存在全局内存中，进入下一次的迭代的所有的所有线程都使用上一步产生的数值，最后一个循环后，整个线程块的和被保存进全局内存中）：   
![](https://files.catbox.moe/q56e1e.png)   
```cpp
__global__ void reduceNeighbored(int *g_idata, int *g_odata, unsigned int n)
{
    //set thread ID 
    unsigned int tid = threadIdx.x;
    
    //convert global data pointer to the local pointer of this block
    int *idata = g_idata + blockIdx.x * blockDim.x;

    //boundary check
    if (idx >= n) return;  //现在可以理解，是因为线程束的原因不能超界

    //in-place reduction in global memory 
    for (int stride = 1; stride < blockDim.x; stride *= 2) {
        if ((tid % (2 * stride)) == 0) {
            idata[tid] += idata[tid + stride];
        }

        //synchronize within block
        __syncthreads();  //同步
    }

    //write result for this block to global mem
    if (tid == 0) g_odata[blockIdx.x] = idata[0];
}
```    
其根本思路类似下图：    
![](https://files.catbox.moe/13novi.png)   

### 3.4.3 改善并行归约的分化   
上面的方法，单个线程束当中存在大量没有利用的线程，线程利用率（其实是数据吞吐量）被浪费了，应该考虑让执行优先填满线程束；

### 3.4.4 交错配对的归约
交错归约的工作线程没有变化，但是每个线程在全局内存中的加载/存储位置是不同的。   
交错归约的内核代码：   
```cpp
__global__ void reduceInterleaved(int *g_idata, int *g_odata, unsigned int n){
    // set thread ID
    int tid = threadIdx.x;
    int idx = blockIdx.x*blockDim.x + threadIdx.x;

    // convert global data pointer to the local pointer of this block 
    int *idata = g_idata + blockIdx.x*blockDim.x; 

    // boudary check 
    if(idx >= n) return; 

    // in-place reduction in global memory 
    // >>= 1 等价于 /2
    for(int stride = blockDim.x / 2; stride > 0; stride >>=1){
        if(tid < stride){
            idata[tid] += idata[tid+stride];
        }
        __syncthreads();
    }
    if(tid == 0) g_odata[blockIdx.x] = idata[0];
}
```   

## 3.5 展开循环   
循环展开是一个尝试通过减少分支出现的频率和循环维护指令来优化循环的技术。在循环展开中，循环主题在代码中要多次编写，而不是只编写一次循环主体再使用另一个循环反复执行的。任何的封闭循环可以将它的迭代次数减少或者完全删除。循环体复制数量称为循环展开因子：   
```cpp
for (int i = 0; i < 100; i++)
{
    a[i] = b[i] + c[i];
}

for (int i = 0; i < 100; i+=2) 
{
    a[i] = b[i] + c[i];
    a[i + 1] = b[i + 1] + c[i + 1];
}
```   
CPU语言里循环展开提高不明显，但是对于CUDA来说至关重要。CUDA目标一直是：通过减少指令消耗和增加更多的独立调度指令来提高性能。   

### 3.5.1 展开的规约    
在一个线程中有更多的独立内存加载/存储操作会产生更好的性能，因为内存延迟可以被隐藏起来。可用设备内存吞吐量来衡量性能提高的原因；

### 3.5.2 展开线程的规约   
``__syncthreads``是用于块同步的，在归约核函数中，它用来确保在线程进入下一轮之前，每一轮中所有线程已将局部结果写入全局内存中。    
考虑剩32个或者更小的线程（即一个线程束）的情况。因为线程束的执行是SIMT的（单指令多线程）的，每条指令之后有隐式的线程束内同步过程。因此，归约循环的最后6个迭代可以用下述语句来展开：   
```cpp
if (tid < 32) {
    volatile int *vem = idata;
    vmem[tid] += vmem[tid + 32];
    vmem[tid] += vmem[tid + 16];
    vmem[tid] += vmem[tid + 8];
    vmem[tid] += vmem[tid + 4];
    vmem[tid] += vmem[tid + 2];
    vmem[tid] += vmem[tid + 1];
}
```   
该线程束展开避免了执行循环控制和线程同步逻辑。    
注意，这里vmem适合volatile修饰符一起被声明的，它告诉编辑器每次赋值必须将vmem[tid]的值存回全局内存中。如果省略了volatile修饰符，这段代码将不能正常工作，因为编译器或缓存可能对全局或共享内存优化读写。如果位于全局或共享内存中的变量有volatile修饰符，编译器会假定其值可以被其他线程在任何时间修改或者使用。因此，任何参考volatile修饰符的变量强制直接读或者写内存，而不是简单的读写缓存或者寄存器。    
通过实验，证明``__syncthreads``能减少新的核函数中的阻塞。   

### 3.5.3 完全展开的规约   
如果编译时已知一个循环中的迭代次数，就可以把循环完全展开，由于在Fermi或Kepler架构中，每个块的最大线程为1024，并且在这些规约核函数中循环迭代次数是基于一个线程块维度的，所以完全展开规约循环是完全可能的。    
    
### 3.5.4 函数模板的规约     
虽然可以手动展开循环，但是使用模板函数可以进一步减少分支消耗。在设备函数上CUDA支持模板函数，如以下代码，可指定块的大小作为模板函数的参数：    
```cpp
template<int iBlocksize>
__global__ void reduceCompleteUnroll(int *g_idata, int *g_odata,
    int n){
    // set thread ID
    int tid = threadIdx.x;
    int idx = blockIdx.x * blockDim.x * 8 + threadIdx.x;

    // convert global data pointer to the local pointer of this block
    int *idata = g_idata + blockIdx.x * blockDim.x * 8;

    // unrolling 8
    if (idx + 7 * blockDim.x < n)
    {
        int a1 = g_idata[idx];
        int a2 = g_idata[idx + blockDim.x];
        int a3 = g_idata[idx + 2 * blockDim.x];
        int a4 = g_idata[idx + 3 * blockDim.x];
        int b1 = g_idata[idx + 4 * blockDim.x];
        int b2 = g_idata[idx + 5 * blockDim.x];
        int b3 = g_idata[idx + 6 * blockDim.x];
        int b4 = g_idata[idx + 7 * blockDim.x];
        g_idata[idx] = a1 + a2 + a3 + a4 + b1 + b2 + b3 + b4;
    }

    __syncthreads();

    // in-place reduction and complete unroll
    if (iBlocksize >= 1024 && tid < 512) idata[tid] += idata[tid + 512];

    __syncthreads();

    if (iBlocksize >= 512 && tid < 256) idata[tid] += idata[tid + 256];

    __syncthreads();

    if (iBlocksize >= 256 && tid < 128) idata[tid] += idata[tid + 128];

    __syncthreads();

    if (iBlocksize >= 128 && tid < 64) idata[tid] += idata[tid + 64];

    __syncthreads();

    // unrolling warp
    if (tid < 32)
    {
        volatile int *vsmem = idata;
        vsmem[tid] += vsmem[tid + 32];
        vsmem[tid] += vsmem[tid + 16];
        vsmem[tid] += vsmem[tid +  8];
        vsmem[tid] += vsmem[tid +  4];
        vsmem[tid] += vsmem[tid +  2];
        vsmem[tid] += vsmem[tid +  1];
    }

    // write result for this block to global mem
    if (tid == 0) g_odata[blockIdx.x] = idata[0];
}
```
   
模板类替换块大小，使得块大小在编译时就会被评估，如果这一个条件为false，那么编译的时候它会被删除，使得内循环更有效率。
注意，该核函数一定要从switch-case结构中被调用，这会允许编译器为特定的线程块大小自动优化代码。但是这也意味着它只对特定块大小下启动reduceCompleteUnroll有效：   
以下总结了规约内核的性能和加载/存储效率：   
![20210813152608](https://i.loli.net/2021/08/13/sJRD7aNu145Qp3z.png)   
![20210813152651](https://i.loli.net/2021/08/13/yTpRxAHlzgBFjVr.png)   
 
 ## 3.6 动态并行    
 目前为止，所有核函数都是从主机线程中被调用。而GPU的工作负载完全在CPU的控制下。CUDA的动态并行允许在GPU端直接创建和同步新的GPU内核，体现在可在一个核函数中的任意点动态增加GPU应用程序的并行性。    
 GPU端直接创建工作的能力可以减少在主机和设备之间传输执行控制和数据的需求，因为在设备上执行的线程可以在运行时可以决定启动配置。    
 本节使用动态并行实现递归规约核函数的例子，对如何利用动态并行有一个基本了解。    
     
### 3.6.1 嵌套执行   
在动态并行中，内核执行分为两种类型：父母和孩子。父线程、父线程块或者父网格启动一个新的网格，即子网格。子线程、子线程块或子网格被父母启动。子网格必须在父线程、父线程块或者副网格完成之前完成，只有在所有的子网格完成之后，父母才会完成。    
下图说明了父网格和子网格的适用范围。主机线程配置和启动父网格，父网格配置和启动子网格，子网格的调用和完成必须适当的进行嵌套，这意味着在线程创建的所有子网格都完成后，父网格才会完成。如果调用的线程没有显式的同步启动子网格，那么运行时保证父母和孩子之间的隐式同步。下图三中在父线程中设置了栅栏，从而可以与其子网格显式的同步。    
 ![20210813154845](https://i.loli.net/2021/08/13/uyght9dqeD372Ul.png)    
 设备线程中的网格启动，在线程块间是可见的。这意味着线程可能与由该线程启动的或由相同线程块中其他线程启动的子网格同步。在线程块中，只有当所有线程创建的所有子网格完成之后，线程块的执行才会完成。    
当父母启动一个子网格，父线程块与孩子显式同步之后，孩子才能开始执行。    
父网格和子网格共享相同全局和常量内存存储，但他们有不同的局部内存和共享内存。有了孩子和父母之间的弱一致性作为保证，父网格和子网格可以对全局内存并发存取。有两个时刻，子网格和它的父线程见到的内存完全相同：子网格开始时和子网格完成时。当父线程由于子网格调用时，所有的全局内存操作必须要保证对子网格是可见的。当父母在子网格完成时进行同步操作后，子网格所有的内存操作应对父母是可见的。    
共享内存和局部内存分别对于线程块或线程来说是私有的，同时，在父母和孩子之间不是可见或者一致的。局部内存对线程来说是私有存储，并且对该线程外部不可见。当启动一个子网格时，**向局部内存传递一个指针作为参数是无效的** 。    

### 3.6.2 在GPU上嵌套Hello World   
```cpp
__global__ void nestedHelloWorld(int const iSize, int iDepth) {
    int tid = threadIdx.x;
    printf("Recursion=%d:Hello World from thread %d block %d", iDepth, tid, blockIdx.x);    
    
    //condition to stop recursive execution 
    if (iSize == 1) return;

    //reduce block size to half
    int nthreads = iSize >> 1;
    
    //thread 0 launches child grid recursively
    if (tid == 0 && nthreads > 0)
    {
        nestedHelloWorld<<<1, nthreads>>>(nthreads, ++iDepth);
        printf("---------> nested execution depth: %d\n", iDepth);
    }
}
```   
![20210813164314](https://i.loli.net/2021/08/13/iEyb8oHz2JRDkjV.png)    

它使用以下代码进行编译：   
``nvcc -arch=sm_35  -rdc=true nestedHelloWorld.cu -o nestedHelloWorld -lcudadevrt``   
由于动态并行是由设备运行时库所支持的，所以必须在命令行使用-lcudadevrt明确链接。   
当-rdc标志为true时，它强制生成可重定位的设备代码，这是动态并行的一个要求。   
上图显示父网格会等待它的子网格执行结束，空白处说明内核在等待子网格执行结束。   

### 3.6.3 嵌套规约   
```cpp
__global__ void gpuRecursiveReduce(int *g_idata, int *g_odata, int isize){
    int tid = threadIdx.x; 
    int *idata = g_idata + blockIdx.x*blockDim.x; 
    int *odata = &g_odata[blockIdx.x]; 

    // stop condition 
    if (isize == 2 && tid == 0){
        g_odata[blockIdx.x] = idata[0] + idata[1];
        return; 
    }

    // nested invocation 
    int istride = isize >> 1; 
    if(istride > 1 && tid < istride){
        idata[tid] += idata[tid+istride];
    }

    // sync
    __syncthreads();

    // nested invocation to generate child grids
    if(tid == 0){
        gpuRecursiveReduce<<<1, istride>>>(idata, odata, istride);

        // sync all child grids
        cudaDeviceSynchronize();
    }
    __syncthreads();
}
```   
以上嵌套规约方法极慢，原因是最初2048个线程块，因为每个线程执行8次递归，所以共创建了16384个子线程，且``__syncthreads``函数也被调用了16384次。    
当一个子网格被调用后，它所共有的内存和父线程是完全一致的。因为每一个子线程只需要父线程的数值来指导归约。因此每个子网格启动前执行线程块内部的同步是没有必要的。去除所有同步操作会产生如下核函数：   
```cpp
__global__ void gpuRecursiveReduceNosync (int *g_idata, int *g_odata,
        unsigned int isize)
{
    // set thread ID
    unsigned int tid = threadIdx.x;

    // convert global data pointer to the local pointer of this block
    int *idata = g_idata + blockIdx.x * blockDim.x;
    int *odata = &g_odata[blockIdx.x];

    // stop condition
    if (isize == 2 && tid == 0)
    {
        g_odata[blockIdx.x] = idata[0] + idata[1];
        return;
    }

    // nested invoke
    int istride = isize >> 1;

    if(istride > 1 && tid < istride)
    {
        idata[tid] += idata[tid + istride];

        if(tid == 0)
        {
            gpuRecursiveReduceNosync<<<1, istride>>>(idata, odata, istride);
        }
    }
}
```
结束后效率有明显提升，大约变为原时间的1/3。   
![20210816145142](https://i.loli.net/2021/08/16/XHmufZaWSBtCsKY.png)    
![20210816145213](https://i.loli.net/2021/08/16/1KDRBWk6xvM2duX.png)   
然而相较于相邻配对内核，它性能依旧较差，需要考虑如何减少由大量自网络启动引起的消耗
。当前实现中每个线程块都产生一个子网个，并引起了大量的调用。如果用上图2的方法，当创建子网格的数量减少时，那么每个子网格中的线程块数量将会增加，保持了相同数量的并行性。    
以下核函数实现了这个方法：通过比较核函数特征码（父线程块的维度会传递给子线程块），这样子线程块中的线程可以精准计算出小号部分的全局内存偏移地址。因为这个实现是所有空线程都是在每次内核启动时被移除的，而对于第一次实现而言，每个嵌套曾内核执行过程都有一半的线程空下来。这样的改变会释放一半的被第一个和函数消耗的计算资源，可以让更多的线程块活跃起来：   
```cpp
__global__ void gpuRecursiveReduce2(int *g_idata, int *g_odata, int iStride, int const iDim) {
// convert global data pointer to the local pointer of this block
int *idata = g_idata + blockIdx.x*iDim;
// stop condition
if (iStride == 1 && threadIdx.x == 0) {
g_odata[blockIdx.x] = idata[0]+idata[1];
return;
}

// in place reduction
idata[threadIdx.x] += idata[threadIdx.x + iStride];
// nested invocation to generate child grids
if(threadIdx.x == 0 && blockIdx.x == 0) {gpuRecursiveReduce2 <<<gridDim.x,iStride/2>>>(g_idata,g_odata,iStride/2,iDim);}
}
```   
这种递归归约核函数极大的减少了时间的占用，因为调用了较少的子网格。因为只调用了8个子网格。     
总而言之，一方面需要避免大量的嵌套调用，另一方面需要减少线程块内部的同步次数（每一个嵌套蹭上设备运行时系统都要保留额外的内存），都是提升速度的关键。   
   
# 4 全局内存   
先前例子中，如果把一个线程块最内层维度设为线程束大小的一半，这会导致内存负载效率的大幅下降，这种性能的损失不能用线程束调度或者并行性来解释，真正原因是全局内存访问的模式。    
## 4.1 CUDA内存模型概述   
数据存在两种不同类型的局部性：   
- 时间局部性：一个数据位置被引用，那么该数据在较短的时间周期内很可能再次被引用，而随着时间流逝，改数据被引用的可能性逐渐降低；   
- 空间局部性： 一个内存位置被引用，那么临近的位置也可能被引用。   
通常的存储器如下图：   
![20210816183350](https://i.loli.net/2021/08/16/JLke2lAGgnUOzNv.png)   
CPU和GPU主存都是用DRAM，而低延迟内存（如CPU一级缓存）使用SRAM。GPU与CPU在内存层次结构设计中都是用相似的准则和模型，其主要区别在于CUDA编程模型可以使内存层次更好的呈现给客户。    

### 4。1.2 CUDA内存模型   
CPU内存结构中，一级缓存及二级缓存都是不可编程的存储器，而CUDA就比较多了：   
> 寄存器   
> 共享内存
> 本地内存
> 常量内存
> 纹理内存
> 全局内存   
如下图所示：   
![20210816184114](https://i.loli.net/2021/08/16/jkz95ReEJbdIFZM.png)   
一个核函数中的线程都由自己私有的本地内存，一个线程块有自己的共享内存，并且对统一而线程块中所有线程都可见，且其内容持续了线程块的整个生命周期。所有线程都可以访问全局内存。所有线程都可以访问全局内存。所有线程都能访问的只读内存空间有：常量内存空间和纹理内存空间。全局内存、常量内存和纹理内存有着不同的用途。