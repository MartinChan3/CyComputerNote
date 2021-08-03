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
```
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




