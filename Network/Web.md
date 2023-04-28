# Socket通讯问题（(CSDN)[https://blog.csdn.net/jiushimanya/article/details/82684525 ]）

1. 网络中进程如何通讯？
本地进程通信（IPC）可以通过消息传递（管道、FIFO、消息队列）、同步（互斥量、条件变量、读写锁、文件读写记录锁、信号量）、共享内存（匿名的和具名的）、远程过程调用；
虽然网络进程的通讯和本地进程没啥关系，但是首先需要解决**唯一标识一个进程**的问题。本地使用一个PID来表示，网络中TCP/IP协议帮我们解决了这个问题：**网络层ip地址可以唯一标识主机，而传输层的"协议+端口"可以唯一标识主机中的应用程序。**这样意味着三元组（ip地址+协议+端口）就可以标识网络的进程了。
而UNIX BSD的套接字（socket）已经是TCP/IP实际当中的垄断性编程接口，所以会说：“一切皆socket”。

2. 什么是Socket？
socket起源于Unix，是模仿Unix哲学“一切皆文件”，可以用“open→write/read→close”模式来操作。即socket其实也是抽象的一个IODevice。

3. Socket()函数：
3.1 int socket(int domain, int type, int protocol);
socket函数对应普通文件的普通操作，和std中普通文件打开返回一个句柄类似，socket()用于创建一个socket描述符，它会标识唯一一个socket，它类似文件描述字，后续操作都会有用。
domain:协议域，也称为协议族(family)。常用的协议族有AF_INET、AF_INET6、AF_LOCAL、AF_ROUTE等，协议族决定了socket的地址类型，在通讯中必须使用相应的地址，例如AF_INET要使用32位的ipv4地址与16位的端口号组合，而AF_UNIX则需要决定一个绝对路径名作为地址；
type:指定socket类型，常见的有SOCK_STREAM、SOCK_DGRAM、SOCK_RAW、SOCK_PACKET、SOCK_SEQPACKET等；
protocol：指定协定，常用的IPPROTO_TCP、IPPROTO_UDP、SCTP、TCIP等。
注意：并不是所有type和protocol可以任意组合，当protocol为0，会自动选择type的类型；
此外，当我们利用socket()创建一个socket时，返回的socket描述子会描述它具体存在于协议族的空间，但是并不会给出一个具体地址，如果需要附一个地址，则必须调用bind()函数，否则在调用connect()、listen()时会自动分配一个端口。
3.2 bind()函数：
例如对应AF_INET、AF_INET6就是把一个ipv4地址或ipv6地址加端口号赋给socket;

int bind(int sockfd, const sockaddr* addr, socklen_t addrlen);
sockfd: 即socket描述字，上面说过用socket()创建，唯一标识一个socket(),bind()就是要将这个描述字绑定一个名字；
addr:一个cosnt struct sockaddr*指针，指向要绑定给sockfd的协议地址，这个地址结构根据创建socket时的地址协议族的不同而不同，例如ipv4对应：
```
struct sockaddr_in {
sa_family_t sin_family;
in_port_t sin_port;
struct in_addr sin_addr;
};

struct in_addr {
uint32_t s_addr;
};
```
addrlen: 对应的是地址的长度；
通常服务器在启动的时候会绑定一个众所周知的地址（例如ip地址+端口号）来提供服务，用户就可以通过它来连接服务器。而客户端就不用指定啦，会有系统自动分配一个端口号和自身的ip地址组合。这就是为什么通常服务器在listen前会调用bind(),而客户端不会，转而会在connect()时随机生成一个。

网络字节序与主机字节序
主机字节序即使我们平时说的大端和小端模式：不同CPU有着不同字节保存序列的类型，叫做主机序，标准的Big-Endian和Little-Endian定义如下：Little-Endian就是低字节在内存低地址端，Big-Endian是反过来
网络字节序：4个字节的32bit值以下面的次序传输，首先是0~7bit,其次8~15bit，然后是16~32bit。由于TCP/IP中所有二进制整数在网络中传输都要以这种次序，因此它又称作为网络字节序。
所以在将把一个地址绑定到socket的时候，必须将主机字节序转为网络字节序，不能假设主机字节序都是使用的Big-Endian。

3.3 listen()、connect()函数
如果作为一个服务器，在调用socket()、bind()之后就会调用listen()来监听这个socket，如果此时客户端发出connect()的请求，服务器端就会接到这个请求。

int listen(int sockfd, int backlog);
int connect(int sockfd, const sockaddr *addr, socklen_t addrlen);
listen函数的第一个参数即为要监听的socket描述字，第二个参数为相应socket可以排队的最大连接个数。socket()函数创建的socket默认是一个主动类型的，listen函数将socket变为被动类型的，等待客户的连接请求。
connect函数的第一个参数即为客户端的socket描述字，第二参数为服务器的socket地址，第三个参数为socket地址的长度，客户端通过connect函数来建立与TCP服务器的连接。

3.4 accept()函数
TCP服务器端依次会调用socket()、bind()、listen()之后，就会监听指定的socket地址。TCP客户端依次调用socket()、connect()之后就想TCP服务器发送了一个连接请求。TCP服务器监听到这个请求之后，就会依次调用accept()函数取接收请求，这样就建立好了。之后就可以开始网络I/O操作了，即实现了网络中不同进程的通信。网络I/O操作有以下几组：
```
read()/write()
recv()/send()
readv()/writev()
recvmsg()/sendmsg()
recvfrom()/sendto()
```
按照原文中笔者意思，推荐使用recvmsg()/sendmsg()函数，是最通用的I/O函数。
```
ssize_t read(int fd, void *buf, size_t count);
ssize_t write(int fd, const void *buf, size_t count);
ssize_t send(int sockfd, const void *buf, size_t len, int flags);
ssize_t recv(int sockfd, void *buf, size_t len, int flags);

ssize_t sendto(int sockfd, const void *buf, size_t len, int flags,
               const struct sockaddr *dest_addr, socklen_t addrlen);
ssize_t recvfrom(int sockfd, void *buf, size_t len, int flags,
                 struct sockaddr *src_addr, socklen_t *addrlen);
ssize_t sendmsg(int sockfd, const struct msghdr *msg, int flags);
ssize_t recvmsg(int sockfd, struct msghdr *msg, int flags);

```
read函数负责从fd中读取内容，如果成功，read会返回实际所读的字节数，如果返回的值是0表示已经读到文件的结束，小于0表示出现了错误。如果错误为EINTR说明读是由中断引起，如果是ECONNREST表示网络连接出了问题。
write函数buf中的nbytes字节写入文件描述符fd,成功时返回写的字节数。失败时返回-1，并设置errno变量。在网络程序中，当我们想套接字文件描述符有两种可能：1) write的返回值大于0，表示写了部分或者是全部的数据；2) 返回的值小于0，此时出现了错误，我们要根据错误类型来进行处理，如果错误为EINTR表示写的时候出现了中断错误，如果为EPIPE则表示网络出现了问题（对方已关闭连接）。

3.6 close()函数
进行完读写操作后就需要关闭相应的socket描述字。

int close(int fd);
close一个TCP socket的缺省行为应该把socket标记为已关闭，然后立即返回到调用进程，该描述字不能再由调用进程使用，也就睡说不能再作为read或者write的第一个参数。
> Tip: close操作只是使相应的socket描述字的引用计数-1，只有当引用计数为0的时候，才会触发TCP客户端向服务器发送终止连接的请求。

4. Socket中TCP的三次握手建立连接：
我们知道tcp的连接需要进行三次握手，即交换三个分组，大致流程如下：
客户端向服务器发送一个SYN J
服务器向客户端响应一个SYN K,并对SYN J进行确认ACK J+1
客户端再向服务器发一个确认ACK K+1
![握手图](https://img-blog.csdn.net/20180913105048585?watermark/2/text/aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L2ppdXNoaW1hbnlh/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70 )
当客户调用connect时，触发了连接请求，向服务器发送了SYN J包，这时候connect进入了阻塞状态；服务器监听到了连接请求，即受到了SYN J包，调用accept函数接收请求向客户端发送SYN K,ACK J+1，这时accept进入阻塞状态；客户端收到服务器的SYN K,ACK J+1之后，此时connect返回，并对SYN K进行确认；服务器收到ACK K+1时,accpet返回，至此三次握手完毕，连接建立。
> 总结：客户端的connect在三次握手的第二次返回，而服务器端的accept在三次握手的第三次返回。

5. socket中TCP四次握手释放
![四次释放](https://img-blog.csdn.net/20180913105058539?watermark/2/text/aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L2ppdXNoaW1hbnlh/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70 )
某个应用进程首先调用close主动关闭连接，这时TCP发送一个FIN M;
另一端接收到FIN M之后，执行被动关闭，对这个FIN进行确认。它的接收也作为文件结束符传递给应用进程，因为FIN的接收意味着应用进程在相应的连接上再也接收不到额外数据；
一段时间后，接收到文件结束符的应用进程调用close关闭它的socket。这导致它的TCP也发送一个FIN N;
接收到这个FIN的源发送端TCP对它进行确认。
这样每个方向上都有一个FIN和ACK。




