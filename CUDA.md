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

### 4.1.2 CUDA内存模型   
CPU内存结构中，一级缓存及二级缓存都是不可编程的存储器，而CUDA就比较多了：   
> 寄存器   
> 共享内存
> 本地内存
> 常量内存
> 纹理内存
> 全局内存   
如下图所示：   
![20210816184114](https://i.loli.net/2021/08/16/jkz95ReEJbdIFZM.png)   
一个核函数中的线程都由自己私有的本地内存，一个线程块有自己的共享内存，并且对统一而线程块中所有线程都可见，且其内容持续了线程块的整个生命周期。所有线程都可以访问全局内存。所有线程都可以访问全局内存。所有线程都能访问的只读内存空间有：常量内存空间和纹理内存空间。全局内存、常量内存和纹理内存有着不同的用途。纹理内存为各种不同的数据布局提供了不同的寻址模式和滤波模式。对于一个应用程序来说，全局内存、常量内存和纹理内存中的内容有相同的生命周期。    

#### 4.1.2.1 寄存器   
寄存器是GPU中运行速度最快的内存空间，核函数中声明的一个没有其他修饰符的自变量，会通常存储在寄存器中。如果用于该数组的索引是常量且在编译时能够确定，那么该数组也存储在寄存器中。    
寄存器是一个在SM中由活跃线程束划分出的较少资源。Fermi架构由63个寄存器，Kepler架构有255个。在核函数中使用越少的寄存器就可以在SM上有更多的常驻线程块，每个SM上并发线程块越多，使用率和性能也就越高。   
如果一个核函数使用了超过硬件限制数量的寄存器，则会用本地内存替代多占用的寄存器。这种寄存器溢出会给性能带来不利影响。nvcc编译器使用启发式策略来最小化寄存器的使用，来避免寄存器溢出，我们可在代码中为每个核函数显式地加上额外的信息来帮助编译器进行优化：   
```cpp
__global__ void __launch__bounds__(maxThreadPerBlock, minBlocksPreMultiprocessor)
kernel(...) {
    //your kernel body
}
```   
maxThreadsPerBlock指出了每个线程块可以包含的最大线程数，这个线程块由核函数来启动。minBlockPerMultiprocessor是可选参数，指明了在每个SM中预期的最小的常驻线程块数量。对于给定的核函数，最优的启动边界会因主要架构的版本不同而有所不同。   
也可以使用maxregcount编译器选项。   

#### 4.1.2.2 本地内存   
核函数中符合存储在寄存器中但不能进入被该核函数分配的寄存器空间中的变量将溢出到本地内存中。编译器可能存放到本地内存中的变量有：   
- 在编译时使用未知索引引用的本地数组；
- 可能会占用大量寄存器空间的较大本地结构体或数组；
- 任何不满足函数寄存器限定条件的变量；   

#### 4.1.2.3 共享内存   
在核函数中使用``__shared__``修饰符放在共享内存中。   
因为共享内存为片上内存，相较于本地内存或全局内存，它具有更高的带宽与更低的延迟。它使用类似于CPU一级缓存，但它是可编程的。   
每一个SM都有一定数量的由线程分配的共享内存。因此必须非常小心不要过度使用共享内存，否则将会在不经意间限制了活跃线程束的数量。   
共享内存在核函数的范围内声明，其生命周期伴随着整个线程块。当一个线程块执行结束后，其分配的共享内存将被释放并重新分配给其他线程块。    
共享内存是线程之间通信的基本方式，一个块内的线程可以通过共享内存中的数据来相互合作。访问共享内存必须同步使用如下调用，该命令是在之前章节中介绍过CUDA运行时调用:``void __syncthread();``   
SM中的一级缓存和共享内存都使用64KB的片上内存，它通过静态划分，但是在运行时可以通过``cudaError_t cudaFuncSetCacheConfig(const void* func, enum cudaFuncCache cacheConfig);``来进行动态配置。    
该函数在每个函数的基础上配置了片上内存的划分，为func指定的核函数设置了配置。支持的缓存配置如下：   
```cpp
cudaFuncCachePreferNone;    //无参考值（默认）
cudaFuncCachePreferShared;  //建议48KB的共享内存和16KB的一级缓存
cudaFuncCachePreferL1;      //建议48KB的一级内存和16KB的共享内存   
cudaFuncCachePreferEqual;   //建议相同尺寸的一级缓存和共享内存，都是32KB
```        
Fermi支持前三种配置，Kepler设备支持所有配置。    

#### 4.1.2.4 常量内存   
常量内存驻留在设备内存中，并且在每个SM专用的常量缓存中缓存。常量变量用如下修饰符来修饰：   
``__constant__``    
常量变量必须在全局空间内以及所有核函数之外进行声明。对于所有计算能力的设备，都只可以声明64KB的常量内存。常量内存是静态声明的，并且对同一编译单元中的所有核函数都可见。   
核函数只能从常量内存中读取数据，因此常量内存必须在主机端使用下面函数来初始化：   
``cudaError_t cudaMemcpyToSymbol(const void * symbol, const void* src, size_t count);``   
这个函数将count个字节从src指向的内存复制到symbol指向的内存中，这个变量存放在设备的全局内存或常量内存中，在大多数情况下这个函数是同步的。    
线程束当中所有线程从相同内存地址中读取数据的时候，常量内存表现的最好。例如数学公式中的系数就是典型的常量内存。   

#### 4.1.2.5 纹理内存   
纹理内存驻留在设备内存中，并在每个SM的只读缓存中缓存。纹理内存是一种通过指定的只读缓存访问的全局内存。只读缓存包括硬件滤波的支持，它可以将浮点插入作为读过程的一部分来执行。纹理内存是对二维空间局部性的优化，所以线程数理使用纹理内存访问二维数据可以达到最优性能。但这是对于一些应用程序是这样，并且由于缓存和滤波硬件的支持所以有较好的性能又是，然而对于另一些应用程序，相较于全局内存，使用纹理内存更慢。   

#### 4.1.2.6 全局内存   
全局内存是GPU中最大、延迟最高并且最常使用的内存。global指的是其作用域和生命周期。它的声明可以在任何SM设备上被访问到，并且贯穿应用程序的整个生命周期。   
一个全局内存变量可以被静态或者动态声明。你可以使用如下修饰符在设备代码中静态地声明一个变量：   
``__device__``    
前文已提过cudaMalloc分配全局内存，cudaFree函数释放全局内存。从多个线程访问全局内存时必须注意，因为线程执行不能跨线程块同步，不同线程块内的线程并发地修改全局内存的同一位置可能会出现问题，这会导致一个未定义的程序行为。    
全局内存驻于设备内存中，可通过32字节、64字节或者128字节的内存倍数进行访问，也就是首地址必须是32字节、64字节、128字节的倍数。当一个线程束执行内存加载/存储时，需要满足的传输数量取决于以下两个因素：   
- 跨线程的内存地址分布   
- 每个事务内存地址的对齐方式    
一般情况下，用来满足内存请求的事务越多，未使用的字节被传输回的可能性就越低，这就造成了数据吞吐率的降低。    

#### 4.1.2.7 GPU缓存    
跟CPU缓存一样，GPU缓存是不可编程的内存，总共有4种：    
> 一级缓存   
> 二级缓存   
> 只读常量缓存   
> 只读纹理缓存   
每个SM都有一个一级缓存，所有SM共享一个二级缓存。一级和二级缓存都被用来在存储本地内存和全局内存中的数据，也包含寄存器溢出的部分。对于Fermi GPU和Kepler K40或期后发布的GPU来说，CUDA允许我们配置读操![20210817180945](https://i.loli.net/2021/08/17/hnrjAMTHGcdvJeP.png)作的数据是使用一级缓存和二级缓存，还是只是用二级缓存。    
在CPU上，内存的加载和存储都可以被缓存，但是在GPU上只有内存加载操作可以被缓存。但是GPU上只有内存加载操作可以被缓存，内存存储操作不能被缓存。   
每个SM也有一个只读常量缓存和只读纹理缓存，它们用于在设备内存中提高来自于各自内存空间内的读取性能。     

#### 4.1.2.8 CUDA变量声明总结    
![20210817180922](https://i.loli.net/2021/08/17/Sq86bY1iOZpIoyH.png)   
![20210817181701](https://i.loli.net/2021/08/17/QaKfmTil9H3U5tq.png)   

#### 4.1.2.9 静态全局内存

```cpp
#include "cuda_runtime.h"
#include "stdio.h"

__device__ float devData; 

__global__ void checkGlobalVariable(){
    printf("Device: the value of the global variable is: %f\n", devData);
    devData += 2.0f;
}

int main(void){
    float value = 3.14f; 
    cudaMemcpyToSymbol(devData, &value, sizeof(float));
    printf("Host: copied %f to the global variable \n", value);

    checkGlobalVariable<<<1, 1>>>();

    // copy back 
    cudaMemcpyFromSymbol(&value, devData, sizeof(float));
    printf("Host the value changed by the kernel to %f \n", value);

    cudaDeviceReset(); 
    return EXIT_SUCCESS; 
}
```    
在CUDA编程中，你需要控制主机和设备这两个地方的操作。一般情况下，设备核函数不能访问主机变量，并且主机函数也不能访问设备变量，即使这些变量在同一文件作用域内被声明。   

## 4.2 内存管理     
CUDA编程的内存管理与C语言类似，需要显式地管理主机与设备之间的数据的移动。随着CUDA版本的升级，NVIDIA正在系统地实现主机和设备内存的空间同一，但对于大多数应用程序来说，仍需要手动移动程序。现在介绍如何使用CUDA来显式的管理内存及数据移动：   
- 分配和释放内存   
- 在主机和设备之间传输数据     

### 4.2.1 内存分配和释放    
以下函数在主机上分配全局内存：   
```cpp
cudaError_t cudaMalloc(void **devPtr, size_t count);
```
用以下函数从主机上传输数据来填充所分配的全局内存（或初始化）：   
```cpp
cudaError_t cudaMemset(void *devPtr, int value, size_t count);
```    
该函数用存储在变量value中的值来填充从设备内存地址devPtr处开始的count字节。    
以下代码释放该内存空间：   
```cpp
cudaError_t cudaFree(void *devPtr);
```    
该函数释放devPtr指向的全局内存。注意，不管是未分配的内存进行分配，还是二次释放，都会返回一个cudaErrorInvalidDevicePointer。如果地址空间已被释放，那么cudaFree也会返回一个错误。   
设备内存放分配和释放的操作成本高，所以应该重利用设备内存，减少对整体性能的影响。   

### 4.2.2 内存传输   
创建好全局内存后，使用以下函数实现主机向设备传输数据：   
```cpp
cudaError_t cudaMemcpy(void *dst, const void *src, size_t count, enum cudaMemcpyKind kind);
```    
包含下列取值：   
![20210818133102](https://i.loli.net/2021/08/18/1QXD7eZSKAkCuI3.png)    

图示为CPU内存与GPU内存之间的连接性能呢，理论上来说GPU的板载内存性能要远高于CPU和GPU之间的PCIe Gen2总线性能   
![20210818133436](https://i.loli.net/2021/08/18/Erc3PWKYL6bJxzn.png)   

### 4.2.3 固定内存     
分配的主机内存默认是pageable(可分页)，它的意思是因页面错误而导致的操作，该操作按照操作系统的要求，将主机虚拟内存上的数据移动到不同的物理位置。虚拟内存给人一种比实际可用内存大得多的假象，就如同一级缓存好像比实际可用的片上内存大得多一样。    
GPU不能在可分页主机上安全的访问数据，因为当主机操作系统在物理位置上移动该数据时，它是无法控制的。当从可分页内存传输数据到设备内存时，CUDA驱动程序首先分配临时页面来锁定或者固定住主机内存，并将主机源数据复制到固定内存中，然后从固定内存传输数据给设备内存，如下图左侧所示：   
![20210818134253](https://i.loli.net/2021/08/18/FtW48LwSuyJ675A.png)   
CUDA使用``cudaError_t cudaMallocHost(void **devPtr, size_t count);``来直接分配固定主机内存。    
以上内存对设备来说可访问，由于可由设备直接读写，因此它能用比可分页内存高得多的带宽进行读写。但是，分配过多的固定内存可能会降低主机系统的性能，因为它减少了用于存储虚拟内存数据的可分页内存的数量，其中分页内存对主机系统是可用的。     

#### 主机与设备间的内存传输 
与可分页内存相比，固定内存的分配和释放成本更高，但是它为大规模数据传输提供了更高的吞吐量。   
相较于可分页内存，使用固定内存获得的加速取决于设备计算能力。例如当传输超过10M的数据时，在Fermi设备上使用固定内存往往是更优选择。    
将许多小的传输批处理为一个更大的传输能提高性能，因为它减少了单位传输消耗。   

### 4.2.4 零拷贝内存   
通常主机不能直接访问设备变量，同时设备也不能直接访问主机变量，但是有一个例外：零拷贝内存。主机和设备都可以访问零拷贝内存。    
GPU线程可以直接访问零拷贝内存。在CDUA核函数中使用零拷贝内存有以下优势：   
> 当设备内存不足时可以使用主机内存；   
> 避免主机和设备间的显式数据传输；    
> 提高PCIe的传输率；   
当使用零拷贝内存来共享主机和设备间的设备时，你必须同步主机和设备间的访问内存，同事更改主机和设备零拷贝内存中的数据将会导致不可预知的后果。    
领拷贝内存是固定（不可分页）内存，该内存映射到设备地址空间中，可以通过下列函数来chuangjyige到固定内存的映射：    
```cpp
cudaError_t cudaHostAlloc(void **pHost, size_t count, unsigned int flags); 
```    
flags参数可以进一步配置：    
·cudaHostAllocDefalt
·cudaHostAllocPortable
·cudaHostAllocWriteCombined
·cudaHostAllocMapped    
cudaHostAllocDefault会使得cudaHostAlloc与cudaMallocHost函数一致。设置cudaHostAllocPortable函数可以返回能被所有CUDA上下文使用的固定内存，而不仅是执行内存分配的哪一个。标志cudaHostAllocWriteCombined返回写结合内存，该内存可以在某些系统配置上通过PCIe总线上更快的传输，但是它在大部分主机上不能够被有效的读写。因此，写结合内存对缓冲区来说是一个很好的选择，该内存通过设备使用映射的固定内存或主机到设备的传输。零拷贝内存的最明显标志是cudaHostAllocMapped，该标志返回，可以实现主机写入和设备读取被映射到设备地址空间中的主机内存。   
可使用以下函数获取映射到固定内存的设备指针：   
```cpp
cudaError_t cudaHostGetDevicePointer(void **pDevice, void *pHost, unsigned int flags);
```   
该函数返回了一个在pDevice中的设备指针，该指针可以在设备上被引用以访问映射得到的固定主机内存，如果设备不支持映射得到的固定内存，该函数将会失效。flag将留作以后使用。但现在，它必须被置为0。    
进行频繁的读写操作时，使用零拷贝内存作为设备内存的补充，就会显著的降低性能，因为每次映射到内存的操作都需要通过PCIe总线。     
从试验结果来看，如果主机和设备端共享少量数据，零拷贝内存可能是一个不错的选择（1KB时为1.84的减速比），因为它简化了编程并且有较好的性能。但是对于PCIe总线连接的离散GPU上更大数据集来说就划不来了，会导致明显的性能下降。    
值得一提的是，两种常见的易购计算机架构：集成架构和离散架构。前者把CPU和GPU集成在一个芯片上，并且可以在物理地址上共享主存。这种情况由于无序通过PCIe总线上备份，所以零拷贝内存在性能和可编程方面表现更佳。     

### 4.2.5 统一虚拟寻址    
计算能力在2.0以上版本的设备支持一种特殊的寻址方式，称为统一虚拟寻址（UVA），从CUDA 4.0中被引入，并且支持64位Linux系统。有了UVA，主机内存和设备内存可以共享一个虚拟地址空间，如下图所示：    
![20210818152117](https://i.loli.net/2021/08/18/e7TMgRX9vScGO45.png)
在UVA之前，用户需要管理哪些指针指向逐级内存，哪些指向设备内存，有了UVA之后，有指针指向的内存空间对应用程序来说是透明的。    
通过UVA，由cudaHostAlloc分配的固定主机内存具有相同的主机和设备指针。因此，可以将返回的指针直接传递给核函数。上一节当中的例子就可以被改写：   
![20210818152914](https://i.loli.net/2021/08/18/a9ehMglmCL7FTAO.png)   
执行与前一节相同的测试产生相同结果，使用了更少的代码取得了相同的结果，这就提高了应用程序的可读性和可维护性。    

### 4.2.6 统一内存寻址   
在CUDA 6.0当中，引入了“统一内存寻址”这一个新特性，它用于简化CUDA编程模型中的内存管理。统一内存创建了一个托管内存池，内存池中已分配的空间可以用相同的内存地址（即指针）在CPU和GPU上进行访问。    
统一内存寻址依赖于UVA的支持，但是它们是完全不同的技术。UVA为系统中的所有处理器提供了一个单一的虚拟内存地址空间。但是，UVA不会自动将数据从一个物理位置转移到另一个位置，这是统一内存寻址的一个特有功能。而且统一内存寻址提供一个“单指针到数据”的模型，在概念上它类似零拷贝内存。    
托管内存指的是有底层系统自动分配的统一内存，与特定设备的分配内存可以互相操作，流入他们的创建都是使用cudaMalloc程序。因此，可以在核函数中可以使用两种类型的内存：由系统控制的托管内存，以及由应用程序明确分配和调用的未托管内存。所有在设备内存上有效的CUDA操作也同样是用于托管内存。其主要区别是主机也能够引用和访问托管内存。    
托管内存可以被静态分配也可以被动态分配，可以通过添加_managed_注释，静态声明一个设备变量作为托管变量。但该操作只能在文件范围和全局范围内进行。该变量可以从主机或设备代码中直接被引用：    
```cpp
_device_ _managed_ int y;
```   
还可以使用以下函数在CUDA运行时动态分配托管内存：    
```cpp
cudaError_t cudaMallocManaged(void **devPtr, size_t size, const unsigned int flags = 0);
```    
注意CUDA 6.0中，设备代码不能调用cudaMallocManaged函数，所有托管内存都必须在主机端动态声明或在全局范围内静态声明。     

## 4.3 内存访问模式     
最大限度利用全局内存带宽是调控核函数性能的基本（因为大部分数据端访问是从全局内存开始，并且多数GPU内存易受内存带宽的限制）。    
CUDA执行模型的最显著特征就是指令必须以线程束为单位进行发布与执行。存储操作也累死，在执行内存指令时，线程束中每个线程都提供了一个正在加载或者存储的内存地址。在线程束的32个线程中，每个线程都提出了一个包含请求地址的单一内存访问请求。在线程束的32个线程中，每个线程都提出一个包含请求地址的单一内存访问请求。它由一个或多个设备内存传输提供服务。根据线程束中内存地址的分布，内存访问可被分为不同的模式。     
### 4.3.1 对齐与合并访问    
![20210818165338](https://i.loli.net/2021/08/18/hXAK1LUp3wb5cfg.png)    
如上图所是，全局内存通过缓存来实现加载/存储。全局内存是一个逻辑内存空间，用户可通过核函数访问它。所有的应用程序数据最初存在DRAM上，即物理设备内存中。核函数的内存请求通常是在DRAM设备和片上内存间以128字节或者32字节内存事务来实现的。    
对全局内存的访问都会通过二级缓存，也会有不少通过一级缓存。这会由访问类型和GPU架构决定。如果两级缓存都被用到，那么内存访问是由一个128字节的内存十五实现的。如果只使用了二级缓存，那么这个内存访问是由一个32字节的内存事务实现的。对全局内存缓存其架构，如果允许使用一级缓存，所以可在编译时选择启用或者禁用一级缓存。    
一行一级缓存是128个字节，它会映射到设备内存中一个128字节的对齐段。如果线程束中的每个线程请求一个4字节的值，那么每次请求就会获取128字节的数据，这恰好与缓存行和设备内存段的大小相契合。     
因此通常在优化应用程序时，需要注意设备访问内存的两个特性：   
> 对齐内存访问
> 合并内存访问    
当设备内存事务的第一个地址是用于事务服务的缓存粒度的偶数倍时（32字节的二级缓存或128字节的一级缓存），就会出现对齐内存访问。运行非对齐的加载会造成带宽的浪费。         
当一个线程束中全部32个线程会访问一个连续的内存块时，就会出现合并内存访问。    
![对齐访问](https://i.loli.net/2021/08/18/4nswarXGK9Zqj7d.png)
![非对齐访问](https://i.loli.net/2021/08/18/X46O1tlZfDJihRy.png)     
      
### 4.3.2 全局内存读取     
SM中，数据通过以下3种缓存/缓冲路径进行传输，具体使用何种方式取决于使用了哪种类型的设备内存：    
- 一级和二级缓存
- 常量缓存    
- 只读缓存    
一/二级缓存是默认路径。想通过其他两种路径传输数据需要应用程序显式的说明，但想要提升性能还是要取决于使用的访问模式。全局内存加载操作是否会通过一级缓存取决于两个因素：   
- 设备的计算能力   
- 编译器选项    

Fermi之后的家欧冠，可通过下列标志通知编译器:    
``-Xptxas -dlcm=cg``   
如果一级缓存被禁用，所有对全局内存的加载请求将会直接进入到二级缓存；如果二级缓存缺失，则由DRAM完成请求。每一次内存事务可以由一个、两个或者四个部分执行，每个部分有32个字节。一级缓存也可使用下列标识符直接启用：    
``-Xptxas -dlcm=ca``    
设置这个标志后，全局内存加载请求首先尝试通过一级缓存。如果一级缓存缺失，则该请求会转向二级缓存。如果二级缓存缺失，则请求由DRAM完成。在这种模式下，一个内存加载请求由一个128字节的设备内存事务实现。    

内存加载可以分为两类：    
- 缓存加载（启用一级缓存）    
- 没有缓存的加载（禁用一级缓存）    
内存加载的访问模式有以下特点：    
- 有缓存与没有缓存：如果启用一级内存，则内存加载被缓存    
- 对齐与非对齐：如果内存访问的第一个地址是32字节的倍数，则对齐加载   
- 合并与未合并：如果线程束访问一个连续的数据块，则加载合并    

#### 4.3.2.1 缓存加载     
缓存加载操作经过一级缓存，在粒度为128字节的一级缓存行上由设备内存事务进行传输。缓存加载可以分为对齐/非对齐及合并/非合并。    
最理想的情况如下：对齐与合并内存访问。    
![20210819103129](https://i.loli.net/2021/08/19/PUltWIpc5dbMkmF.png)    
另一种情况是访问是对齐的，但引用地址不是连续的线程ID，而是128字节范围内的随机值。总线程利用率仍然是100%。
![20210819103409](https://i.loli.net/2021/08/19/KForGqxnvektJE4.png)     
如果请求连续32个连续4字节的非对齐数据元素，恰好在全局内存线程束的线程请求地址落在两个128字节范围内。所以当启用一级缓存时，由SM执行的物理加载操作必须在128字节的界线上对齐，所以要求有两个128字节的事务来执行这段内存加载操作，总线利用率为50%。    
![20210819103757](https://i.loli.net/2021/08/19/9SFsilmjOv5VrJe.png)    
另一种情况是线程束中所有线程请求同一个地址，这种情况总线利用率极低，只有4字节/128字节=3.125%
![20210819131813](https://i.loli.net/2021/08/19/L39DEi7p6dsxUvn.png)     
最糟糕的情况是线程束中的请求分散于全局内存中的32个4字节的地址。尽管线程束请求的字节总数仅为128个字节，但是地址需要占用N个缓存行（0<N<=32）。完成一次内存加载操作需要申请B次内存事务。    
![20210820093745](https://i.loli.net/2021/08/20/LxzIhY1M8TBjRHU.png)     

#### 4.3.2.2 没有缓存的加载     
没有缓存的加载不经过一级缓存，它的表现和通过一级缓存的表现类似。    

#### 4.3.2.3 非对齐读取的示例    
> 全局加载效率=请求的全局内存加载吞吐量/所需的全局内存加载吞吐量     
通过观察以全局加载效率为指标的结果，可以验证这些非对齐访问就是性能损失的原因。而请求的全局内存加载吞吐量包括重新执行的内存加载指令，这个指令不只需要一个内存事务，而请求的全局内存加载吞吐量不需要如此。     
对于非对齐的情况，禁用一级缓存使得加载效率得到了提高。（128字节的粒度缩为32字节的粒度）      

#### 4.3.2.4 只读缓存    
只读缓存加载粒度为32个字节，通常对于分散读取来说，这些更细的粒度加载要优于一级缓存。    
有两种方式可以指导内存对只读缓存进行读取：    
- 使用函数__ldg
- 在间接引用的指针上使用修饰符    
例如，已下为拷贝核函数：    
```cpp
__global__ void copyKernel(int *out, int *in)
{
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    out[idx] = in[idx];    
}
```    
你可以使用内部函数__ldg来通过只读缓存直接对数组进行读取访问：    
```cpp
__global__ void copyKernel(int *out, int *in)
{
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    out[idx] = __ldg(&in[idx]);    
}
```     
我们还可使用``__restrict__``修饰符应用到指针上。这些修饰符帮助nvcc编译器识别无别名的指针（即专门用来访问特定数组的指针）。nvcc将自动通过只读缓存来指导无别名指针的加载：    
```cpp
__global__ void copyKernel(int * __restrict__  out, int * __restrict__ in)
{
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    out[idx] = in[idx];    
}
```     

#### 4.3.3 全局内存写入     
内存写入操作相对读取操作相对简单。至少在Fermi或者Kepler GPU上是无法直接操作一级缓存的，只能操作二级缓存。存储操作在32个字节段的粒度上进行。内存事务可以被划分为1段、2段以及4段。如果两个地址同属于一个128字节区域，但是不属于一个对齐的64字节区域，则会执行一个四段事务（这意味着执行一个四段事务比执行两个一段事务更好）。    
先来看一个理想情况：    
![20210820154249](https://i.loli.net/2021/08/20/UfSQLDJg8VicbCl.png)     
下图反映了内存访问是对齐的，但是地址分散在一个192字节范围内的情况，存储请求会由3个一段事务来实现：    
![20210820154410](https://i.loli.net/2021/08/20/PkQXYAgKnpEq1FW.png)     
如果内存地址是对齐的，并且地址访问在一个连续的64字节范围内的情况。这种存储请求由一个两段事务来完成：    
![20210820154605](https://i.loli.net/2021/08/20/fHzs3ZhE6L12ucx.png)     

### 4.3.4 结构体数组与数组结构体     
对于C程序员来说有两种熟悉的数据组织方式：数组结构体（AoS）和结构体数组（SoA）。    
下面先举一个例子，先考虑这些成对的数据元素如何使用AoS方法进行存储，如定义以下结构：   
```cpp
struct innerStruct {
    float x;
    float y;
};
```    
然后按照下列方法定义这些结构体数组，这是利用Aos方式来组织数据的，它存储的是空间上相邻的数据(例如x和y)，这在CPU上会有良好的缓存局部性。    
```cpp
struct innerStruct myAos[N];
```     
接下来考虑使用SoA方法来存储数据：    
```cpp
struct innerArray {
    float x[N];
    float y[N];
};
```    
这里，在原结构体中每个字段的所有值都会被分到各自的数组中，这不仅能够将相邻数据点紧密的存储起来，也能够将数据的独立数据点存储起来。你可以使用如下结构体定义一个变量：   
```cpp
struct innerArray moa;
```     
下图说明了AoS和SoA方法的内存布局：     
![20210820155813](https://i.loli.net/2021/08/20/mOzSJTalBVci9tW.png)     
用AOS方法在GPU上存储示例数据并执行一个只有x字段的应用程序，会导致50%的带宽损失，因为y值在每32节字节段或128个字节缓存行上被隐式的加载。AoS格式也在不需要的y值上浪费了二级缓存空间。     
而SOA方式则充分利用了GPU的内存带宽，由于没有相同字段元素的交叉存取，GPU上的SoA布局提供了合并内存访问。     

许多并行编程范式，尤其是SIMD范式，更倾向于使用SoA。CUDA C中也普遍倾向于SoA， 因为数据元素是为全局内存的有效合并访问而预先准备好的，而被相同内存操作引用的通字段数据元素在存储时时彼此相邻的。     
通过实验证明，AoS比SoA的确带宽利用率低一半。     

### 4.3.5 性能调整      
优化设备内存带宽利用率有两个目标：    
- 对齐及合并内存访问，以减少带宽的浪费；    
- 足够的并发内存操作，以隐藏内存延迟；     
在上一节中，已经学习了如何组织内村访问模式来实现对齐合并的内存访问。这样做在设备DRAM和SM上内存或寄存器之间能确保有效利用字节移动。    
而第3章中讨论了优化指令吞吐量的核函数。回忆一下，实现并发内存访问最大化是通过以下方式获得的：   
- 增加每个线程中执行独立内存操作的数量；    
- 对核函数启动的执行配置进行实验，以充分体现每个SM的并行性；    

#### 4.3.5.1 展开技术     
考虑之前的readSegment示例。按照如下方式修改readOffset核函数，使得每个线程都执行4个独立地内存操作，因为每个加载过程都是独立地，因此可调用更多的并发内存访问。    
```cpp
__global__ void readOffsetUnroll4(float *A, float *B, float *C, const int n, int offset) 
{
    unsigned int i = blockIdx.x * blockDim.x * 4 + threadIdx.x;
    unsigned int k = i + offset;
    if (k + 3 * blockDim.x < n)
    {
        C[i] = A[k];
        C[i + blockDim.x] = A[k + blockDim.x] + B[k + blockDim.x];
        C[i + 2 * blockDim.x] = A[k + 2 * blockDim.x] + B[k + 2 * blockDim.x];
        C[i + 3 * blockDim.x] = A[k + 3 * blockDim.x] + B[k + 3 * blockDim.x];
    }
}
```    
实验证明上述代码对于性能提升有着非常明显的作用，甚至比地址对齐还要好。相对于原来无循环展开的readSegment示例，循环展开技术实现了3.04~3.17倍的加速。对于一个IO密集型核函数来说，充分说明**内存访问并行性有很高的优先级**。    
但是展开并不影响内存操作的数量（只影响并发执行的数量）。在非对齐情况，原始核函数明显比展开核函数读写事务数量多。     

### 4.3.5.2 增大并行性     
为了充分体现并行，回忆上一章当中做的关于核函数配置的实验。    
现在当非对齐访问执行时，可以验证线程块大小对性能的影响。以下结果和对齐访问示例产生的结果类似，这表明无论访问是否对齐，每个SM中相同的硬件资源限制仍然会影响核函数的性能。     

为了最大化带宽利用率，影响设备内存操作性能的因素主要有两个：    
- 有效利用设备DRAM和SM上内存之间的字节移动：为了避免设备内存带宽的浪费，内存访问模式应是对齐和合并的；    
- 当前的并发内存操作数：可通过以下两点实现最大化当前存储器操作数。1) 展开，每个线程产生更多的独立内存访问；2）修改核函数启动的执行配置来使每个SM有更多的并行性。     

## 4.4 核函数可到达的带宽     
分析核函数性能时，需要注意内存延迟，即完成一次独立内存的时间；内存带宽，即SM访问设备内存的速度，它以每单位时间的字节数进行测量。     
虽然已经尝试使用各种方法改进核函数性能，但问题是往往问题的本质就是一个不好的访问模式。对于这样一个核函数，怎么样的性能才是足够好的？在次理想的情况下可达到的最理想性能又是什么呢？我们通过矩阵转置的例子展示，即使一个原本不好的访问模式，仍然可以通过重新设计核函数的几个部分来实现良好的性能。      

### 4.4.1 内存带宽      
一般有如下两种类型的带宽：     
- 理论带宽
- 有效带宽    
前者是硬件可以实现的绝对最大带宽：
``有效带宽(GB/s)=(读写字数+写字结束)*10^(-9)/运行时间``

### 4.4.2 矩阵转置问题     
以下是一个基于主机的单精度浮点值的错位转置算法：    
```cpp
void transposeHost(float *out, float *in, int nx, int ny)
{
    for (int iy = 0; iy < ny; ++iy)
    {
        for (int ix = 0; ix < nx; ++ix)
        {
            out[ix * ny + iy] = in[iy * nx + ix];
        }
    }
}
```   
![20210820181542](https://i.loli.net/2021/08/20/pXzNGwZ1nhIPy8m.png)
![20210820181629](https://i.loli.net/2021/08/20/nBpGZzswhuYlOMP.png)     
观察输入和输出的布局，你会发现：    
- 读：通过原矩阵和行进行访问，结果为合并访问；    
- 写：通过转置矩阵的列进行访问，结果为交叉访问；    
交叉访问是使得GPU性能变得最差的内存访问模式。但是这么操作似乎不可避免。本文剩余部分将侧重于使用两种专制和核函数来提高带宽的利用率，一种是按行读取按列存储，另一种则按列读按行存储。     
如果禁用一级缓存，那么两种实现的性能在理论上是相同的，但是如果启用一级缓存，第二种实现的性能会更好。    

#### 4.4.2.1 为转置核函数设置性能的上限和下限    
在执行矩阵转置核函数之前，可以先创建两个拷贝核函数来粗略计算所有转置核函数性能的上限和下限：   
- 通过加载和存储行来拷贝矩阵（上限）。这样将模拟执行相同数量的内存操作作为转置，但是只能使用合并访问；
- 通过加载和存储列来拷贝矩阵（下限）。这样将模拟执行相同数量的内存操作作为转置，但是只能使用交叉访问；    
核函数的实现：     
```cpp
__global__ void copyRow(float *row, float *in, const int nx, const int ny)
{
    unsigned int ix = blockDim.x * blockIdx.x + threadIdx.x;
    unsigned int iy = blockDim.y * blockIdx.y + threadIdx.y;
    if (ix < nx && iy < ny)
    {
        out[iy * nx + ix] = in[iy * nx + ix];
    }
}

__global__ void copyCol(float *row, float *in, const int nx, const int ny)
{
    unsigned int ix = blockDim.x * blockIdx.x + threadIdx.x;
    unsigned int iy = blockDim.y * blockIdx.y + threadIdx.y;
    if (ix < nx && iy < ny)
    {
        out[iy * ny + iy] = in[ix * ny + iy];
    }
}
```
![20210823092723](https://i.loli.net/2021/08/23/oOyiDsEWNXzvcMl.png)     
![20210823094124](https://i.loli.net/2021/08/23/biJoDKFl8uYO2LW.png)     

这些测试给出了上限和下限，分别是理论峰值的70%和33%。    
![20210823094918](https://i.loli.net/2021/08/23/WvrdU7peG1yqc5s.png)     

#### 4.4.2.2 朴素转置：读取行与读取列     
基于行的朴素转置是基于主机实现的，这种转置按行加载按列存储：    
```cpp
__global__ void transposeNativeRow(float *out, float *in, const int nx, const int ny)
{
    unsigned int ix = blockDim.x * blockIdx.x + threadIdx.x;
    unsigned int iy = blockDim.y * blockIdx.y + threadIdx.y;
    if (ix < nx && iy < ny)
    {
        out[iy * nx + ix] = in[iy * nx + ix];
    }
}
```     
通过互换索引和写索引，就生成了基于列的朴素转置核函数。这种转置按列加载按行存储：    
```cpp
__global__ void transposeNativeCol(float *out, float *in, const int nx, const int ny)
{
    unsigned int ix = blockDim.x * blockIdx.x + threadIdx.x;
    unsigned int iy = blockDim.y * blockIdx.y + threadIdx.y;
    if (ix < nx && iy < ny)
    {
        out[iy * ny + iy] = in[ix * ny + iy];
    }
}
```      
内核switch语句添加了以下几种情况：     
```cpp
case 2: 
    kernel = &transposeNativeRow;
    kernelName = "NativeRow     ";
    break;
case 3:
    kernel = &transposeNativeCol;
    kernelName = "NativeCol     ";
    break;
```    
![20210823100403](https://i.loli.net/2021/08/23/GF6t75NbhHjqu41.png)     
结果表明使用NaiveCol方法比NaiveRow方法性能表现的更好。如前面所述，这种性能提升的原因可能是在缓存中执行了交叉读取。如果尝试禁用一级缓存，可以看到禁用缓存对交叉读取访问模式有着显著的影响。     
![20210823100908](https://i.loli.net/2021/08/23/phJCKFofY3tTX5c.png)      
结果表明，对于NaiveCol实现而言，由于合并写入，存储请求从未被重复执行，但是由于交叉读取，多次重复执行了加载请求。这证明了即使是较低的加载效率，一级缓存中的缓存架在也可以限制交叉加载对性能的负面负面影响。      

#### 4.4.2.3 展开转置：读取行与读取列    
接下来尝试使用展开技术来提高转置内存带宽的利用率。这个例子中，展开的目的是为了每个线程分配更独立的任务，从而最大化当前内存请求。      
以下是一个展开因子是4的基于行的实现，这里引入两个新的数组索引，一个用于行访问，一个用于列访问：    
```cpp
__global__ void transposeUnroll14Row(float *out, float *in, const int nx, const int ny) 
{
    unsigned int ix = blockDim.x * blockIdx.x * 4 + threadIdx.x;
    unsigned int iy = blockDim.y * blockIdx.y + threadIdx.y;
     
    unsigned int ti = iy * nx + ix;   //access in rows
    unsigned int to = ix * ny + iy;   //access in columns

    if (ix + 3 * blockDim.x < nx && iy < ny)
    {
        out[to]                       = in[ti];
        out[to + ny * blockDim.x]     = in[ti + blockDim.x];
        out[to + ny * 2 * blockDim.x] = in[ti + 2 * blockDim.x];
        out[to + ny * 3 * blockDim.x] = in[ti + 3 * blockDim.x];
    }
}
```     
以下是类似的基于列的实现：    
```cpp
__global__ void transposeUnroll14Col(float *out, float *in, const int nx, const int ny) 
{
    unsigned int ix = blockDim.x * blockIdx.x * 4 + threadIdx.x;
    unsigned int iy = blockDim.y * blockIdx.y + threadIdx.y;
     
    unsigned int ti = iy * nx + ix;   //access in rows
    unsigned int to = ix * ny + iy;   //access in columns

    if (ix + 3 * blockDim.x < nx && iy < ny)
    {
        out[ti]                  = in[to];
        out[ti + blockDim.x]     = in[to + blockDim.x * ny];
        out[ti + 2 * blockDim.x] = in[to + 2 * blockDim.x * ny];
        out[ti + 3 * blockDim.x] = in[to + 2 * blockDim.x * ny];
    }
}
```     
![20210823103850](https://i.loli.net/2021/08/23/icforW2FlNRXHpY.png)      
通过启用一级缓存，按列加载和按行存储获得了更好的有效带宽和整体执行时间。      

#### 4.4.2.4 对角转置：读取行与读取列     
当启用一个线程块网格时，线程块会被分配给SM。编程模型抽象可能用一个一维或者二维布局来表示该网格。但是从硬件的角度看，所有块都是一维的。每个线程块都有其唯一的标识符bid，它可以用网格中的线程块按行优先顺序计算得出：    
``int bid = threadIdx.y * gridDim.x + blockIdx.x;``
![20210823110024](https://i.loli.net/2021/08/23/6bPVUkhJsHu2B3C.png)    
当启用一个核函数时，线程块被分配给SM的顺序由块ID来确定。一旦所有的SM被完全占用，所有剩余的线程都保持直到当前的执行被完成。一旦一个线程块执行结束，将为该SM分配另一个线程块。由于线程块完成的速度和顺序是不确定的，随着内核进程的执行，起初通过bid相连的活跃线程块会变得不太连续了。      
尽管无法直接调控线程块的顺序，但是可以灵活的使用块坐标blockIdx.x和blockIdx.y。上一张图说明了笛卡尔坐标系下的块坐标，而下图展示了使用对角块坐标系的方法：     
![20210823111148](https://i.loli.net/2021/08/23/v5ELANmt3ZWIuDG.png)     
对角坐标系用于确定一维线程块的ID，对于数据访问，仍需使用笛卡尔坐标系。因此使用对角坐标系表示块ID时，需要将对角坐标映射到笛卡尔坐标中，以便可以访问到正确的数据块。对于一个方阵来说，这个映射可以通过下列方程式计算的得出：    
```cpp
block_x = (blockIdx.x + blockIdx.y) % gridDim.x;
block_y = blockIdx.x;
```    
这里的blockIdx.x和blockIdx.y为对角坐标。block_x和block_y是它们对应的笛卡尔坐标。下面的核函数号使用了对角线程序块坐标，它借助合并读取和交叉写入实现了矩阵的转置：    
```cpp
__global__ void transposeDiagonalRow(float *out, float *in, const int nx, const int ny)
{
    unsigned int blk_y = blockIdx.x;
    unsigned int blk_x = (blockIdx.x + blockIdx.y) % gridDim.x;

    unsigned int ix = blockDim.x * blk_x + threadIdx.x;
    unsigned int iy = blockDim.y * blk_y + threadIdx.y;

    if (ix < nx && iy < ny)
    {
        out[ix * ny + iy] = in[iy * nx + ix];
    }
}
```
使用基于列的对角坐标的核函数如下所示：    
```cpp
__global__ void transposeDiagonalCol(float *out, float *in, const int nx, const int ny)
{
    unsigned int blk_y = blockIdx.x;
    unsigned int blk_x = (blockIdx.x + blockIdx.y) % gridDim.x;

    unsigned int ix = blockDim.x * blk_x + threadIdx.x;
    unsigned int iy = blockDim.y * blk_y + threadIdx.y;

    if (ix < nx && iy < ny)
    {
        out[iy * nx + ix] = in[ix * nx + iy];
    }
}
```
性能结果如下所示：    
![20210823115211](https://i.loli.net/2021/08/23/POlzXpqR7wYMCnc.png)     
通过使用对角坐标来修改线程块的执行顺序，这是基于行的核函数性能得到了大大的提升。但是，基于列的核函数在使用笛卡尔坐标系仍然比使用对角块坐标系表现的更好。对角核函数的实现可以通过展开块得到更大的提升，但是这种实现不像使用基于笛卡尔坐标系的核函数那样简单直接。     
这种性能提升的原因与DRAM的并行访问有关。发送给全局内存的请求由DRAM分区完成。设备内存中的连续256字节区域被分配到连续的分区。当使用笛卡尔坐标将线程块映射到数据块时，全局内存访问可能无法均匀的被分配到整个DRAM从分区中，这时候就可能发生“分区冲突”的现象。发生分区冲突时，内存请求在某些分区中排队等候，而另一些分区则一直未被调用。因为对角坐标映射造成了从线程块到待处理数据块的非线性映射，所以交叉访问不太可能会落入到一个独立地分区中，并且会带来性能的提升。    
对于最佳性能来说，被所有活跃的线程束并发访问的全局内存应该在分区中被均匀的划分。下图是一个简化的可视化模型：    
![](https://files.catbox.moe/y6b4u7.png)     
![](https://files.catbox.moe/ky0ohl.png)    
简化后的模型使用对角坐标来表示块ID。这种情况下，需要使用两个分区为0、1.2.3个线程块加载和存储数据。加载和存储请求在两个分区之间被均匀分配。这个例子说明了为什么对角核函数能够获得更好的性能。    

#### 使用瘦块来增加并行性     
增加并行性最简单方式是调整块的大小，之前几节内容已经证明了这种简单技术对提高性能的有效性。我们对基于列的NaiveCol核函数的块大小进行实验，测试结果如下：    
![](https://files.catbox.moe/nl9fui.png)     
明显看到，性能最佳的为瘦块(8,32)，虽然它的并行性是与(16,16)的块相同，但是这种性能提升是由“瘦的”块带来的。     
![](https://files.catbox.moe/s256t1.png)    

## 4.5 使用统一内存的矩阵加法     
在第2章中已经学了如何在GPU中添加两个矩阵。为了简化主机和设备内存空间的管理，提高这个CUDA程序的可读性和易维护性，可以使用统一内存来将以下解决方案添加到矩阵加法的主函数中：    
- 用托管内存分配来替换主机和设备内存分配，以消除重复指针；     
- 删除所有显式的内存副本；     
实验结果表明，使用托管内存的核函数速度与显式地在主机和设备之间复制数据几乎一样快，并且很明显它需要更少的编程工作。      


# 第5章 共享内存和常量内存     
由于非合并访问是无法避免的，许多情况下，使用共享内存来提高全局内存合并访问是有可能的。共享内存是很多高性能计算应用程序的关键驱动力。      

## 5.1 CUDA共享内存概述    
GPU中共有两种类型的内存：     
- 板载内存    
- 片上内存     
全局内存是较大的板载内存，具有相对较高的延迟。共享内存是较小的片上内存，具有相对较低的延迟，并且共享内存可以提供比全局内存高得多的带宽。可以把它当做一个可编程管理的缓存。共享内存通常的用途有：     
- 块内线程通信的通道    
- 用于全局内存数据的可编程管理的缓存
- 高速暂存存储器，用于转换数据来优化全局内存访问模式     

### 5.1.1 共享内存     
共享内存是GPU的一个关键部件，五里上每个SM都有一个小的低延迟内存池，这个内存池被当前正在SM上执行的线程块中的所有线程所共享。共享内存使同一个线程块中的线程能够相互协作便于重用偏上的数据，并且可以大大降低核函数所需要的全局内存带宽。      
Fermi和Kepler具有相似的内存层次结构。全局内存中所有加载和存储请求都要经过二级缓存，这是SM单元之间数据统一的基本点。注意，相较于二级缓存和全局内存，共享内存和一级缓存在五里上更接近SM。因此，共享内存相较于全局内存而言，延迟要低大约20~30倍，而带宽高其大约10倍。        
![](https://files.catbox.moe/5ujgod.png)     
当每个线程块开始执行时，会分配给它一定数量的共享内存。这个共享内存的地址空间会被线程块中所有的线程共享。它的内容和创建时所在的线程块具有相同生命周期。每个线程发出共享内存的访问请求。在理想情况下，每个被线程束共享内存访问的请求在一个事务中完成。最坏的情况下，每个共享内存的请求在32个不同的事务中顺序执行。如果多个线程访问共享内存中的同一个字，一个线程读取该字后，通过多播把它发送给其他线程。     
共享内存被SM中所有的常驻线程块划分，因此共享内存时限制设备并行性的关键资源。一个核函数使用的共享内存越多，处于并发活跃状态的线程块就会越少。     
CUDA中允许手动管理共享内存。     

### 5.1.2 共享内存分配     
有多种方法可以用来分配或声明由应用程序锁清秋所决定的共享内存变量。可以静态或者动态的分配共享内存变量。在CUDA的源文件中，共享内存可以被声明为一个本地的CUDA核函数或者是一个全局的核函数。CUDA支持一维、二维或者三维共享内存数组的声明。     
使用``__shared__``对共享内存变量进行修饰。     
下面的代码静态声明了一个共享内存的二维浮点数组。如果在核函数中进行声明，那么这个变量的作用于就局限在该内核中。如果在文件的任何核函数外进行声明，那么这个变量的作用域对于所有核函数都是全局的。    
``__shared__ float title[size_y][size_x];``    
如果共享内存的大小在编译时是未知的，那么可以用extern关键字声明一个未知大小的数组。例如下面的代码声明了共享内存中一个未知大小的一维整形数组。这个声明可以在某个核函数的内部或者所有核函数的外部进行。     
``extern __shared__ int tile[];``     
因为该数组大小在编译时是未知的，所以在每个核函数被调用时，需要动态分配共享内存，将所需的大小按字节作为三重括号内的第三个参数，如下所示：    
``kernel<<<grid, block, isize * sizeof(int)>>>(...)``    
注意，只能动态声明一维数组。     

### 5.1.3 共享内存存储体和访问模式    
优化内存要度量的两个属性是：延迟和带宽。第4章解释了由不同的内存访问模式引起的延迟和带宽对核函数性能的影响。共享内存可以用来隐藏全局内存延迟和带宽对性能的影响。要想充分理解这些资源，了解共享内存时如何被安排的会对齐有帮助；      

#### 5.1.3.1 内存存储体     
为了获得高内存带宽，共享内存被分为32个同样大小的内存模型，它们被称为存储体，它们可以同时被访问。有32个存储体是因为在一个线程束中有32个线程。共享内存是一个一维地址空间。根据GPU的计算能力，共享内存的地址在不同模式下会映射到不同的存储体中。如果通过线程束发布共享内存加载或存储操作，且每个存储体上只访问不多于一个的内存只，那么该操作可由一个内存事务来完成。否则，该操作由多个内存事务来完成，这就降低了内存带宽的利用率。     

#### 5.1.3.2 存储体冲突     
当线程发出共享内存请求时，有以下3种典型的模式：    
- 并行访问：多个地址访问多个存储体
- 串行访问：多个地址访问同一个存储体
- 广播访问：单一地址读取单一存储体     
并行最优，串行最差，广播是带宽利用率很差。    
以下三图分别表现出并行访问、串行访问以及广播访问的效率：    
![](https://files.catbox.moe/b5fi9h.png)    
![](https://files.catbox.moe/ljxpt6.png)    
![](https://files.catbox.moe/q0u6u4.png)      

#### 5.1.3.3 访问模式      
共享内存存储体的宽度规定了共享内存地址与共享内存存储体之间的对应关系。内存存储体的宽度随设备计算能力的不同而变化。有两种不同的存储体宽度：    
- 计算能力2.x的设备中为4字节（32位）
- 计算能力3.x的设备中为8字节（64位）    
对于Fermi设备，存储体的宽度是32位且有32个存储体。每个存储体在每两个时钟周期内都由32位的带宽。连续的32字映射到连续的存储体重。因此，从共享内存地址到存储体索引的映射可按照如下公式：    
> 存储体索引=(字节地址÷4字节/储存体)%32存储体    
字节地址除以4转换为一个4字节索引，然后进行模32操作，将4字节自索引转换为存储体索引。    
![](https://files.catbox.moe/z20qip.png)      
上图显示了从Fermi设备中从字节地址到字索引的映射。      
当来自相同线程束中的两个线程访问相同的地址时，不会发生存储体冲突。这种情况下对于读访问，这个字请求被广播到请求的线程中，对于写访问这个字只能由其中一个线程写入，而这个写入操作的线程是不确定的。     
而对于Kepler设备，共享内存由32个存储体，它们有32位或64位两种模式。     

#### 5.1.3.4 内存填充     
内存填充是避免存储体冲突的一种方法。
![](https://files.catbox.moe/o5vej4.png)     
为了解决一个五项的存储体冲突，方法是在每N个元素之后添加一个字。这里的N是存储体的数量，也改变了字到存储体的映射。      
填充的内存不能用于数据存储，唯一作用是移动数据元素，以便将原来属于同一个存储体中的数据分散到不同存储体中。填充之后，还需要重新计算数组索引以确保能访问到正确的数据元素。     
这里要注意由于Fermi和Kepler都有32个存储体，但是因为存储体宽度不同，填充内存时必须要小心。Fermi架构中的某些内存填充模式可能会导致Kepler中的存储体冲突。      

#### 5.1.3.5 访问模式配置     
之前提到过，Kepler设备支持4字节和8字节的共享内存访问模式。默认是4字节模式。可采用以下CUDA运行时API来查询访问模式：    
``cudaError_t cudaDeviceGetSharedMemConfig(cudaSharedMemConfig *pConfig);``    
结果返回到pConfig，返回的存储体配置可以是以下值中的一个：    
```
cudaSharedMemBankSizeFourByte
cudaSharedMemBankSizeEightByte
```    
在可配置共享内存存储体的设备上，可以使用以下功能设置一个新的存储体代销：    
```
cudaError_t cudaDeviceSetSharedMemConfig(cudaSharedMemConfig config);    
```
支持的存储体配置为：    
```
cudaSharedMemBankSizeDefault
cudaSharedMemBankSizeFourByte
cudaSharedMemBankSizeEightByte
```     
在不同的核函数启动之间更改共享内存配置可能需要一个隐式的设备同步点。更改共享内存存储体的大小不会增加共享内存的使用量，也不会影响核函数的占用率，但是它对性能有重大影响。      

### 5.1.4 配置共享内存量     
每个SM都有64KB的片上内存，共享内存和一级缓存共享该硬件资源。CUDA为配置一级缓存和共享内存的大小提供了两种方法：    
- 按设备进行配置
- 按核函数进行配置    
使用下述的运行时函数，可以为在设备上启动的核函数配置一级缓存和共享内存大小：    
```
cudaError_t cudaDeviceSetCacheConfig(cudaFuncCache cacheConfig);
```    
参数cacheConfig指明，在当前的CUDA设备上，片上内存时如何在一级缓存和共享内存间进行划分的。所支持的参数如下：     
> cudaFuncCachePreferNone:      no preference(default)
> cudaFuncCachePreferShared:    prefer 48KB shared memory and 16KB L1 cache
> cudaFuncCachePerferL1:        perfer 48KB L1 cache and 16KB shared memory
> cudaFuncCachePreferEqual:     prefer 32KB L1 cache and 32KB shared memory
哪种模式更好，这取决于核函数中使用了多少共享内存，典型情况如下：    
- 当核函数使用较多的共享内存时，倾向于使用更多的共享内存
- 当核函数使用更多的寄存器时，倾向于使用更多的一级缓存     
CUDA运行时会尽可能使用请求设备的片上内存配置，但是如果需要执行一个核函数，它可自由地选择不同的配置，每个核函数的配置可以覆盖设备范围的设置。可以使用以下运行时函数进行设置：    
```
cudaError_t cudaFuncSetCacheConfig(const void *func, enum udaFuncCache chcheConfig);
```     
核函数使用的这种配置是由核函数指针func指定的。启动一个不同优先级的内核比启动有最近优先级设置的内核可能更会导致设备同步。对于每个核，只需要调用一次这个函数，而每个核函数启动时，片上内存中的配置不需要重新设定。     
虽然一级缓存和共享缓存位于相同的片上硬件中，但是在某些方面它们却不太相同。共享内存是通过32个存储体进行访问的。而一级缓存则是通过缓存进行访问的。使用共享内存，对存储内容和存放位置会有完全的控制权，而使用一级缓存，数据删除工作是由硬件完成的。     

### 5.1.5 同步     
并行线程间的同步是所有并行计算语言的重要机制。CUDA提供了两种方法：    
- 障碍
- 内存栅栏    
障碍意味着所有调用的线程等待其余调用的线程到达障碍点。在内存栅栏中所有调用的线程必须等到全部内存修改对其余调用线程可见时才能继续执行。     

#### 5.1.5.1 弱排序内存模型     
现代的内存架构有一个宽松的内存模型，这意味着内存访问不一定按照它们在程序中出现的顺序进行执行。CUDA采用弱排序内存模型从而优化了更多激进的编译器。     
GPU线程在不同的内存中写入数据的顺序，不一定和这些数据在源代码中访问的顺序相同。一个线程的写入顺序对其他线程可见时，它可能和写操作被执行的实际顺序不一致。     
如果指令之间是相互独立的，线程从不同内存中读取数据的顺序和读指令在程序中出现的顺序不一定相同。     
为了显式的强制程序以一个确切的顺序执行，必须在应用程序代码中插入内存栅栏和障碍。这是保证与其他线程共享资源的核函数行为正确的唯一途径。     

#### 5.1.5.2 显式障碍     
在CUDA中，障碍只能在同一线程块的线程间执行。在核函数中，可以通过调用下面的函数来指定一个障碍点：    
``void __syncthreads();``     
__syncthreads()用于协调同一块中线程间的通信，当块中的某些线程访问共享内存或全局内存中的同一地址时，会有潜在问题（写后读、读后写、写后写），这将会导致在那些内存位置产生未定义的应用程序和未定义的状态。可以通过利用冲突访问间的同步线程来避免这种情况。     
在条件代码中使用__syncthreads()时必须要小心。如果一个条件能保证对整个线程块进行同等评估，则它是调用__syncthreads的唯一条件。否则执行很可能会挂起或产生意想不到的问题。例如下面的代码可能会导致块中的线程无限期的等待对方，因为块中的所有线程没有到达相同的障碍点。     
```
if (threadId % 2 == 0)
{
    __syncthreads();
}
else 
{
    __syncthreads();
}
```     
如果不允许跨线程块同步，线程块可能会以任何顺序、并行、串行的顺序在任何SM上执行。线程块执行的独立性使得CUDA编程在任意数量的核心中都是可扩展的。如果一个CUDA核函数要求跨线程块全局同步，那么通过在同步点分割核函数并执行多个内核启动可能会达到预期的效果。因为每个连续的内核必须等待之前的内核启动完成，这会产生一个隐式的全局障碍。      

#### 5.1.5.3 内存栅栏
内存栅栏功能可确保栅栏前的任何内容对栅栏后的其他线程都是网络的。根据所需范围，有3中内存栅栏：块、网格或系统。       
通过以下固有函数可以在线程块内创建内存栅栏：     
``void __threadfence_block();``     
__threadfence_block保证了栅栏前被调用线程产生的对共享内存和全局内存的所有写操作对栅栏后的同一块中的其他线程都是可见的。回想一下，内存栅栏不执行任何线程同步，所以对于一个块中所有线程来说，没有必要实际执行这个命令。      
使用下面固有函数来创建网格级内存栅栏：    
``void __threadfence();``     
__threadfence挂起调用的线程，直到全局内存中的写操作对于相同网格内的所有线程都是可见的。    
使用下面的函数可以跨系统（包括主机和设备）设置内存栅栏：    
``void __threadfence_system();``    
__threadfence_system挂起调用的线程，以确保该线程对全局内存、锁页主机内存和其他设备内存中所有写操作对全部设备中的线程和主机线程是可见的。      

#### 5.1.5.4 Volatile修饰符     
在全局或共享内存中使用volatile修饰符声明一个变量，可以防止编译器优化，编译器优化可能会将数据暂时缓存在寄存器或者本地内存中。
当使用volatile修饰符时，编译器假定其他任何线程在任何时间都可以更改或使用该变量的值，因此，这个变量的任何引用都会直接被编译到全局内存读指令或全局内存写指令中，它们都会忽略缓存。    
##### 共享内存与全局内存    
GPU全局内存常驻在设备内存（DRAM）上，它比GPU的共享内存访问慢得多。相较于DRAM，共享内存存在以下几个特点：    
- DRAM比其高20~30倍的延迟    
- 比DRAM大10倍的带宽    

## 5.2 共享内存的数据布局    
为了全面了解如何有效的使用共享内存本节将会使用共享内存研究几个简单的例子。当这些概念了然于心时，就可以设计一个高效的核函数了，它可以避免存储体冲突，并且充分利用共享内存的优势。    

### 5.2.1 方形共享内存     
![](https://files.catbox.moe/z8mpno.png)     
以上二维共享内存变量可用以下静态声明：    
```
__shared__ int tile[N][N];
```    
因为该共享内存是方形的，所以可以选择一个二维线程块来访问它，在x或者y维度上通过相邻线程访问临近元素：    
```
tile[threadIdx.y][threadIdx.x]
tile[threadIdx.x][threadIdx.y]
```     
最好是有访问共享内存连续位置的线程，且该线程带有连续的threadIdx.x值。由此可以得出结论，第一存储模式（块[threadIdx.y][threadIdx.x]）将会比第二存储模式（块[threadIdx.x][threadIdx.y]）呈现出更好的性能和更少的存储体冲突，因为邻近线程在最内层数组上访问相邻的阵列单元。    

#### 5.2.1.1 行主序访问和列主序访问     
考虑一个例子：      
```cpp
#define BDIMX 32 
#define BDIMY 32

dim3 block(BDIMX, BDIMY);
dim3 grid (1, 1);

//核函数有两个简单操作：1) 将全局线程索引按行写入到一个二维共享内存数组中；2) 从共享内存中按行主序读取这些值并且将它们存储到全局内存中    

__shared__ int tile[BDIMY][BDIMX];

//由于只有1个线程块，索引简化为    
unsigned int idx = threadIdx.y * blockDim.x + threadIdx.x;     

//将全局索引写入到共享内存块，可按照如下方式进行
tile[threadIdx.y][threadIdx.x] = idx;

//一旦达到同步点(使用syncthreads函数)，所有线程必须将存储的数据送到共享内存块中，这样就可以按照行主序从共享内存给全局内存赋值，如下所示：    
out[idx] = tile[threadIdx.y][threadIdx.x];

//核函数代码如下：    
__global__ void setRowReadRow(int *out)
{
    __shared__ int tile[BDIMY][BDIMX];

    unsigned int idx = threadIdx.y * blockDim.x + threadIdx.x;     

    tile[threadIdx.y][threadIdx.x] = idx;

    __syncthreads();

    out[idx] = tile[threadIdx.y][threadIdx.x];

}
```     
因为相同线程束中的线程有连续的threadIdx.x值，并且可以使用threadIdx.x索引共享内存数组tile的最内层维度，所以核函数无存储体冲突。     
Tesla K40c， 具有4字节共享内存访问模式的结果如下，这清楚展示了按行共享内存可以提高性能，因为相邻线程引用相邻字。     
![](https://files.catbox.moe/65ai56.png)     
nvprof的结果显示，在setRowReadRow中，线程束的存储和加载请求由一个事务完成，而相同的请求在setColReadCol核函数中由16个事务完成。这证实了在Kepler设备上，当使用8字节共享内存存储体时，核函数会有16路存储体冲突。     
![](https://files.catbox.moe/ekjcsm.png)     

#### 5.2.1.2 按行主序写和按列主序读      
![](https://files.catbox.moe/8rmgws.png)     
用nvprof检查该内核的内存事务后，报出以下指标：     
![](https://files.catbox.moe/6wgdx0.png)     
存储操作是无冲突的，但是加载操作显示有16路冲突。     

#### 5.2.1.3 动态共享内存     
可以动态声明共享内存，从而实现这些相同的核函数。可以在核函数外声明动态共享内存，使它的作用域为整个文件，也可以在核函数内声明动态共享内存，将其作用域限制在该内核之内。动态共享内存必须声明为一个未定大小的一维数组，因此，需要基于二维线程索引来计算内存访问索引。因为要这个核函数中按行主序写入，按列主序读取，所以需要保留以下两个索引：     
> row_idx：根据二维线程索引计算出的一维行主序内存偏移量    
> col_idx: 根据二维线程索引计算出的一维列主序内存偏移量     
代码如下：    
```cpp
__global__ void setRowReadColDyn(int *out)
{
    extern __shared__ int tile[];

    unsigned int row_idx = threadIdx.y * blockDim.x + threadIdx.x;
    unsigned int col_idx = threadIdx.x * blockDim.y + threadIdx.y;

    tile[row_idx] = row_idx;

    __syncthreads();

    out[row_idx] = tile[col_idx];
}
```     
在启动内核时，必须指定共享内存的大小，如下所示：    
```
setRowReadColDyn<<<grid, block, BDIMX * BDIMY * sizeof(int)>>>(d_C);
```
这些结果和前面setRowReadCol例子相同，但是它却是使用了由一维数组索引计算出的动态声明的共享内存。写操作是无冲突的，但是读是报告了16路冲突。     

#### 5.2.1.4 填充静态声明的共享内存    
前文已描述，填充数组是避免存储体冲突的一种方法。填充静态声明的共享内存很简单，只需要将1列添加到二维共享内存分配中，如下：    
``__shared__ int tile[BDIMY][BDIMX + 1];``
下面的核函数是setRowReadCol核函数的修改版。通过在每行添加一个元素，列元素就分不在了不同的存储体中，因此读和写的操作都是无冲突的：    
```cpp
__global__ void setRowReadColPad(int *out){
    __shared__ int tile[BDIMY][BDIMX+IPAD]; //IPAD = 1
    unsigned int idx = threadIdx.y * blockDim.x + threadIdx.x;
    tile[threadIdx.y][threadIdx.x] = idx;

    __syncthreads();
    out[idx] = tile[threadIdx.y][threadIdx.x];
}
```    
对于Fermi设备，需要增加一列来解决存储体冲突；对于Kepler设备，并非总是如此，每行填充元素数量取决于二维共享内存的大小。因此需要在Kepler设备中进行更多的测试，以便为64位访问模式确定合适的填充数量元素。    

#### 5.2.1.5 填充动态声明的共享内存    
对于填充动态声明的内存数组更复杂些，比许对每一行必须跳过一个填充的内存空间，代码如下：   
```
unsigned int row_idx = threadIdx.y * (blockDim.x + 1) + threadIdx.x;
unsigned int col_idx = threadIdx.x * (blockDim.x + 1) + threadIdx.y;
```   
以下为图示：    
![](https://files.catbox.moe/b2vqai.png)     
其nvprof效率结果和填充静态声明的共享内存结果是一致的。    

#### 5.1.2.6 方形共享内核性能的比较     
到目前为止，可以看出：    
> 使用填充的内容可填充内容，因为它减少了存储体冲突；    
> 带有动态声明共享内存的内核增加了少量的消耗；     

### 5.2.2 矩形共享内存      
矩形内存是一个更普遍的内存形式。这里仍以上文中内容建立两个核函数： 
```
__global__ void setRowReadRow(int *out);
__global__ void setColReadCol(int *out);    
```    
首先遇到的问题是每个内核中矩形共享内存数组的声明。在setRowReadRow当中：   
``__shared__ int tile[BDIMY][BDIMX];``
而setColReadCol中相反：     
``__shared__ int tile[BDIMX][BDIMY];``     
![](https://files.catbox.moe/d9gtya.png)      
以上结果显式在Kepler K40的存储体宽度是8个字节，一列16个4字节的数据元素被安排到8个存储体中，因此该操作有个8路冲突。     

#### 5.2.2.2 行主序写操作和列主序读操作    
```cpp
__global__ void setRowReadCol(int *out) 
{
     __shared__ int tile[BDIMY][BDIMX];

     unsigned int idx = threadIdx.y * blockDim.x + threadIdx.x;

     unsigned int irow = idx / blockDim.y;
     unsigned int icol = idy % blockDim.y;
     
     tile[threadIdx.y][threadIdx.x] = idx;

     __syncthreads();

     out[idx] = tile[icol][irow];
}
```    
该存储操作时是无冲突的，加载操作则有一个8路冲突。      

#### 5.2.2.3 动态声明的共享内存     
类似之前块动态声明共享内存分配，其效率和上述效率描述一样。     

#### 5.2.2.4 填充静态声明的共享内存     
对于Kepler设备，必须计算出需要多少填充元素来解决存储体冲突的问题，为了便于编程，使用以下宏来添加填充列的数量：    
```cpp
#define NPAD 2
```     
用nvprof检测内存事务，结果报告如下图所示：     
![](https://files.catbox.moe/tqpxcq.png)      
如果将填充数据元素从2改到1，那么nvprof报告有两个事务完成内存的加载操作，即会发生一个双向存储体冲突。鼓励你用不同数值的NPAD来进行实验。     

#### 5.2.2.5 填充动态声明的共享内存    
同上    

#### 5.2.2.6 矩形共享内存内核性能的比较     
使用动态共享内核的核函数会显示有少量的消耗      

## 5.3 减少全局内存访问      
使用共享内存的原因之一就是要缓存片上数据，从而减少核函数中全局内存访问的次数。第3章介绍了使用全局内存的并行规约核函数，并集中解释了以下几个问题：     
> 如何重新安排数据访问模式来避免线程束分化；    
> 如何展开循环以保证有足够的操作使指令和内存带宽饱和；     

### 5.3.1 使用共享内存的并行规约      
使用共享内存的核函数比只是用全局内存的核函数快了1.84倍，这是因为使用共享内存明显的降低了全局内存访问：      

### 5.3.2 使用展开的并行规约     
在前面的核函数中，每个线程块处理一个数据块。在第三章中，可以通过一次运行多个I/O操作，展开线程块来提高内核性能。以下内核展开了4个线程块，及每个线程处理来自4个数据块的数据元素。通过展开，以下优势是可以预期的：     
- 通过在每个线程中提供更多的并行I/O，增加全局内存的吞吐量    
- 全局内存存储事务减少了1/4     
- 整体内核性能的提升     
展开的并行规约比原非展开快了2.76倍，主要是因为展开的存储事务减少了1/4，但是加载事务的数量保持不变，最后检查全局内存吞吐量，结果加载吞吐量增加了2.57倍儿存储吞吐量下降了1.56倍。后者的下降是因为较少的存储请求使总线达到了饱和。      

### 5.3.3 使用动态内存的并行规约     
并行规约核函数可以使用动态内存     

### 5.3.4 有效带宽
由于归约核函数是受内存带宽约束的，所以评估时所用的适当性能指标是有效带宽：     
> 有效带宽= (读字节+写字节)÷(运行时间*109)GB/s     

## 5.4 合并的全局内存访问     
使用共享内存也会帮助避免对未合并的全局内存的访问。矩阵转置就是个典型的例子：读操作被自然合并。但是它的写操作是按照交叉访问的。    
### 5.4.1 基准转置内核    
上一章中有提到naiveGmem是整个性能的下限，本节将会对其进行逐步的优化。    
以执行合并访问为目的的的更改操作会生成副本内核。因为读写操作将会被合并，但是仍然会执行相同数量的I/O，所以copyGmem会成为一个性能近似的上界：    

### 5.4.2 使用共享内存的矩阵转置    
为了避免交叉全局内存访问，可以使用二维共享内存来缓存原始矩阵的数据。下图显示了在矩阵转置中是如何使用共享内存的：    
![](https://files.catbox.moe/z2w1mj.png)     
```
__global__ void transposeSmem(float *out, float *in, int nx, int ny)
{
    __shared__ float tile[BDIMY][BDIMX];

    unsigned int ix, iy, ti, to;
    ix = blockIdx.x * blockDim.x + threadIdx.x;
    iy = blockIdx.y * blockDim.y + threadIdx.y;

    ti = iy * ny + ix;    
    
    unsigned int bidx, irow, icol;
    bidx = threadIdx.y * blockDim.x + threadIdx.x;
    irow = bidx / blockDim.y;
    icol = bidx % blockDim.y;

    ix = blockIdx.y * blockDim.y + icol;
    iy = blockIdx.x * blockDim.x + irow;

    to = iy * ny + ix;

    if (ix < nx && iy < ny)
    {
        tile[threadIdx.y][threadIdx.x] = in[ti];

        _syncthreads();    

        out[to] = tile[icol][irow];
    }
}
```      
综上所述，总共包含以下几个步骤：     
1. 线程束执行合并读取一行，该行存储在全局内存中的原始矩阵块中；
2. 然后，该线程束按行主序写入共享内存中，所以该写操作没有存储体冲突；    
3. 因为线程块的读写操作都是同步的，所以会有一个填满全局内存数据的二维共享内存数组；    
4. 该线程从二维共享内存数组中读取一列。由于共享内存没有被填充，所以会发生存储体冲突。    
5. 然后该线程结束执行数据的合并写入操作，将其写入到全局内存的转置矩阵中的某行。     
![](https://files.catbox.moe/urxy70.png)      
结果如下所示： 
![](https://files.catbox.moe/zby19l.png)     

### 5.4.3 使用填充共享内存的矩阵转置     
通过给二维共享内存数组tile的每一行添加列填充，可以将原矩阵相同列中的数据元素均匀的划分到共享内存的存储体中。需要填充的列数取决于设备的计算能力和线程块的大小。对于一个大小为32*16的线程块被测试内核来说，在Tesla K40中必须增加两列填充，而在Tesla M2090中必须增加一列填充。在Tesla K40中，下面的填充语句声明了填充的共享内存：     
``__shared__ float tile[BDIMY][BDIMX + 2];``
此外，对tile的存储和加载必须被转化对每行中的额外两列负责。填充列会提供额外的加速：    
![](https://files.catbox.moe/mmj63p.png)     
从实验结果看，在共享内存数组中添加列填充消除了所有的存储体冲突。     

### 5.4.4 使用展开的转置矩阵    
![](https://files.catbox.moe/m5jvst.png)     
从实验结果来看，展开提供了显著的性能改善：      
![](https://files.catbox.moe/q7gqw6.png)
在Tesla K40上使用nvprof的结果总结如下，展开内核的吞吐量提高近1.5倍。      

### 5.4.5 增大并行性     
通常需要分析，与内存吞吐量相比，该内核受到全局内存吞吐量约束更多。      

## 5.5 常量内存     
常量内存是一种专用内存，它用于只读数据和统一访问线程中线程的数据。常量内存对内核代码而言是只读的，但它对主机而言是可读又是可写的。       
常量内存位于设备的DRAM上（和全局内存一样），并且有一个专用的片上缓存。和一级缓存与共享内存一样，从每个SM的常量缓存中读取的延迟，比直接从常量内存中读取的低得多。每个SM常量内存的缓存大小的限制为64KB。     
到目前为止相较于其他内存，常量内存有一个不同的最优访问模式。按常量内存中，如果线程束中的所有线程都访问相同的位置，那么这个访问的模式就是最优的。如果线程束中的线程访问不同的地址，则访问就需要串行。因此一个常量内存读取的成本与线程束中线程读取唯一地址的数量呈线性关系。     
全局作用域中使用``__constant__``来修饰常量变量。     
常量内存变量的生存期与应用程序的生存期相同，其对网格内的所有线程都是可访问的，并且通过运行时函数对主机可访问。当使用CUDA独立编译能力时，常量内存变量跨多个源文件是可见的。因为设备只能读取常量内存，所以常量内存中的值必须使用以下运行时函数进行初始化：    
```cpp
cudaError_t cudaMemcpyToSymbol(const void* symbol, const void *src, size_t count, size_t offset, cudaMemcpyKind kind)
```
cudaMemcpyToSymbol函数将src指向的数据复制到设备上由symbol指定的常量内存中。枚举变量kind指定了传输方向，默认情况下，kind是cudaMemcpyHostToDevice。       

### 5.5.1 使用常量内存实现一维模板      
在数值分析中，模板计算在集合点集合上应用函数，并用输出更新单一点的值。模板是求解许多偏微分方程的基础，在一维中，在位置x周围的九点模板会给这些位置上的值应用一些函数：    
![](https://files.catbox.moe/7dk7vi.png)     
它是用于实变量函数f在x上一阶导数的第八阶中心差分公式。理解这个公式的应用不重要，只要简单地了解到它会将上述的九点作为输入并产生单一输出。在本节中该公式将作为一个示例模板。     
![](https://files.catbox.moe/i710qs.png)       
理论上，系数c0、c1、c2和c3在所有线程中都是相同的且不会被修改，这使得它们成为常量内存的最优候选。因为它们是只读的，并将呈现一个广播式的访问模式：线程束中每个线程同时引用相同的常量内存地址。     
下面的内核实现了基于上述公式的一维模板计算，由于每个线程都需要9个点来计算一个点，所以要使用共享内存来缓存数据，从而减少全局内存的冗余访问。    
``__shared__ float smem[BDIM + 2 * RADIUS];``    
RADIUS定义了点x两侧的点的数量，这些点被用于计算x的值。在这个例子中，为了形成一个九点模板，RADIUS被定义为4:x两侧各有4个点加上位置x的值。在每个块的左、右边界上各需要一个RADIUS个元素的“光环”。     
![](https://files.catbox.moe/a8rcqe.png)      
注意，这里使用#pragma unroll是为了提示CUDA编辑器，表明这个循环会被自动展开。     
完整核函数如下：    
```cpp
__global__  void stencil_ld(float *in, float *out)
{
    __shared__ float smem[BDIM + 2 * RADIUS];
    
    int idx = threadIdx.x + blockIdx.x * blockDim.x;

    int sidx = threadIdx.x + RADIUS；

    smem[sidx] = in[idx];

    if (threadIdx.x < RADIUS)
    {
        smem[sidx - RADIUS] = in[idx - RADIUS];
        smem[sidx + BDIM] = in[idx + BDIM];
    }

    __syncthreads();

    float tmp = 0.0f;
    #pragma unroll
    for (int i = 1; i <= RADIUS; i++)
    {
        tmp += coef[i] * (smem[sidx + i] - smem[sidx - i]);
    }

    out[idx] = tmp;
}
```

在常量内存中声明coef数组，代码如下：   
``__constant__ float coef[RADIUS + 1];``
然后使用cudaMemcpyToSymbol的CUDA API调用从主机端初始化的常量内存：     
```cpp
void setup_coef_constant(void)
{
    const float h_coef[] = {a0, a1, a2, a3, a4};
    cudaMemcpyToSymbol(coef, h_coef, (RADIUS + 1) * sizeof(float));
}
```     

### 5.5.2 与只读缓存的比较      
Kepler GPU添加了一个功能，即使用GPU纹理流水线作为只读缓存，用于存储全局内存中的数据。因为这是一个独立的只读缓存，它带有从标准全局内存读取的独立内存带宽，所以使用此功能可以为带宽限制提供性能优势。     
每个Kepler SM都有48KB的只读缓存。一般来说，只读缓存在分散读取方便比一级缓存更好，当线程束中的线程都读取相同的地址时，不应该使用只读缓存。只读缓存的粒度为32字节。     
当通过只读缓存访问全局内存时，需要向编译器指出在内核的持续时间里数据是只读的。有两种方法可以实现这一点：     
- 使用内部函数_ldg     
- 全局内存的限定指针     
内部函数_ldg用于代替标准指针解引用，并且强制加载通过只读数据缓存，如下面的代码片段所示：    
```cpp
__global__ void kernel(float *output, float *input)
{
    //...
    output[idx] += __ldg(&input[idx]);
    //...
}
```
可以限定指针为const__restrict__,以表明它们应该通过只读缓存被访问：    
```cpp
void kernel(float *output, const float* __restrict__ input) 
{
    //...
    output[idx] = input[idx];
}
```    
很明显，内部函数__ldg是一个更好的选择。     
只读缓存是独立的，而且区别于常量缓存。通过常量缓存加载的数据必须是相对较小的，而且访问必须一致以获得良好的性能（一个线程束内所有线程在任何给定时间内应该都访问相同的位置）。而通过只读缓存加载数据是可以比较大的，而且能够在一个非统一的模式下进行访问。     
使用了新的核函数（只是在函数声明部分加restrict限制），使用nvprof测试结果表明，对此应用程序使用只读内存时性能实际上会降低。这是由于coef数组使用了广播访问模式。相比于只读缓存，该模式更适用于常量内存。      
常量缓存和只读缓存的特点：    
- 在设备上常量缓存和只读缓存都是只读的。     
- 每个SM资源都有限：常量缓存是64KB，而只读缓存是48KB。   
- 常量缓存在同一读取中可以更好的执行（统一读取是线程束中每一个线程都访问相同的地址）
- 只读缓存更适合分散读取。     

## 5.6 线程束洗牌指令      
在本章，已经介绍了如何使用共享内存执行线程块中线程间低延迟数据的交换。从Kepler系列开始（计算能力为3.0或更高）开始，洗牌指令(shuffle instruction)作为一种机制被加入其中，只要2个线程在相同的线程束中，那么就允许这两个线程直接读取另一个线程的寄存器。     
洗牌指令使得线程束中的线程彼此之间可以直接交换数据，而非通过共享内存或内全局内存来进行。洗牌指令比共享内存有更低的延迟，并且该指令在执行数据交换时不消耗额外的内存。因此洗牌指令非常有吸引力。     
先引入束内线程（lane）概念，简单来说一个束内线程指的是线程束内的单一线程。线程束内每个束内线程是[0,31]是束内线程索引的唯一标识。线程束中每个线程都由一个唯一的束内线程索引，并且统一线程块中的多个线程可以有相同的束内线程索引（就像同一网格中多个线程可以有相同的threadIdx.x值一样）。然而，束内线程索引没有内置变量，因为线程索引有内置变量，在一维线程块中，对于一个给定线程的线程束可以按照如下公式进行计算：    
```cpp
laneID = threadIdx.x % 32;
wrapID = threadIdx.x / 32;
```
例如线程块中的线程1和线程33都由束内线程ID 1，但它们有不同的线程束ID。对于二维线程块，可以将二维线程坐标转为一维线程索引，并且应用前面的公式来确定束内线程和线程束的索引。      

### 5.6.1 线程束洗牌指令的不同形式     
有两组洗牌指令















