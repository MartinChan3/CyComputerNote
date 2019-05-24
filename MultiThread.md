# 多线程

## 多线程
### 定义   
进程：是并发执行的程序在执行过程中分配和管理资源的基本单位，是一个动态概念。   
线程：是进程的一个执行单元，是进程内调度的实体。它是比进程更小的独立运行的基本单位。线程也被称为轻量级线程。   
区别：    
- 地址空间： 同一进程的线程共享本进程的地址空间，而进程之间是独立的地址空间。   
- 资源拥有： 同一进程内的线程共享本进程的资源如内存、I/O、CPU等，但是进程之间的资源是相互独立的。    
一个进程崩溃后，在保护模式下不会对其他进程造成影响，但是一个线程崩溃后，整个进程都会崩溃。所以多进程比多线程要健壮。   
- 执行过程： 每个独立的进程有一个程序运行的入口、顺序执行序列和程序入口。但是线程不能独立执行，必须依存在应用程序中，由应用程序提供多个线程执行的控制。   
- 线程是处理器调度的基本单位，但是进程不是。   
- 两者都可以并发执行。   
### GUI线程和工作者线程   
如上所述，每个程序启动后就会拥有一个线程，该线程称为“主线程”（在Qt程序中称为GUI线程）。QtGUI必须运行在此线程上。其所拥有的部件和几个相关的类，例如QPixmap，是不可以工作在次线程中。次线程通常称为“工作者线程”，主要用于处理从主线程中卸下的一部分工作。   
### 数据的同步访问   
每个线程都有自己的栈，这意味着每个线程都拥有自己的调用历史和本地变量。不同于进程，线程共享相同的地址空间。下图显示了内存中的线程块图。非活跃线程的程序计数器和相关寄存器通常保存在内核空间中，对于每个线程来说，存在一个共享代码片段和一个单独的栈。     
![多线程存储分配示意图](https://img-blog.csdn.net/20160727163519029)    
Tip: User Space指的是用户程序所占用的空间， Kernal Space是指系统程序所占用的空间[阮一峰](http://www.ruanyifeng.com/blog/2016/12/user_space_vs_kernel_space.html)    
如果两个线程拥有一个指向相同对象的指针，那么两个线程可以同时取访问对象，这可能会导致一定的错误。   
有时候，从不同线程访问同一个对象是不可避免的，那么就需要一定的方法进行所谓的同步操作。那么怎样做才算安全呢？在一个线程中创建的所有对象在线程内部使用是安全的，前提是其他线程没有引用该线程中创建的一些对象并且这些对象与其它线程之间没有隐性耦合关系。当数据作为静态成员、单例或者全局变量时，这种隐性的耦合是可能存在的。    
### 何时不应该使用线程   
线程的不确定性，决定了其实不使用多线程会更加的安全。例如以下几个替代方案：   
name | 价格   
-|-   
QEventLoop::processEvents()|在一个耗时的计算中反复调用来避免GUI阻塞，但是这种方式有个缺点是不能够很好的扩展，因为可能调用太过于频繁或者不够，这取决于硬件。   
QTimer | 有时候，在后台进程中使用一个计时器来调度将来某个事件点运行一段程序非常方便。超时时间为0的计时器将在事件处理完后立即触发。   
QSocketNotifier QNetworkAccessManager QIODevice::readyRead() | 当在一个低速的网络连接上进行阻塞读的时候，可以不使用多线程，只要对一块网络数据的计算可以很快的执行。只要对一块网络数据的计算可以很快的运行，那么这种交互式的设计比线程中的同步等待要好一些。交互式设计比多线程要不容易出错而且更加有效。许多情况下也有性能上的提升。   

一般情况下，建议只使用安全的而且已经被验证过的路径，避免引入线程的概念。QtConcurrent提供了一种简单的接口，来将工作分配到所有的处理器的核上。线程相关代码已经完全隐藏在QtConcurrent框架中，开发者也无需关心对应的细节。但是QtConcurrent不能用于需要和运行中的线程通信的情形，也不能用于阻塞操作。      
### 处理异步执行   
一种获得工作者线程的结果就是等待该线程停止。然而在很多情况下，阻塞的等待是不可接受的。另一种方式是通过发送事件或者queued信号和槽来获得异步的结果。这产生了一些开销，因为一个操作的结果并不是出现在下一个代码行，而是在另一个位于其他地方的槽中。Qt开发者对这种执行模式应该是习惯的，因为它和GUI应用程序中事件驱动的方式极其类似。   

## Qt多线程类概览   
### QThread 具有可选事件循环的低级API   
QThread是Qt当中所有线程控制的基础，实例化的QThread提供了一个并行事件循环，允许在次线程中调用QObject的槽函数。子类化QThread提供了一个并行事件循环，允许在次线程中调用QObject的槽函数（**这说明事件循环还是多线程的实现基础**）。子类化QThread允许应用程序在开始其事件循环之前初始化新的线程，或者运行没有事件循环的并行代码。    
### QRunnable和QThreadPool 重用线程   
经常创建和销毁线程代价比较巨大，而现有县城可以用于新任务，QThreadPool就是可重用的QThreads的集合。   
每个Qt程序都有一个全局的线程池，通过QThreadPool::globalInstance()进行访问。此全局线程池根据CPU中的核心数目自动维护最佳线程数，但是同时可以显式的创建和管理单独的QThreadPool。   
### QtConcurrent 高级API   
Qt Concurrent模块提供了处理一些常见的并行运算的高级函数：map、filter和reduce。和使用QThread和QRunnable不同，这些函数不需要使用低级线程的语言，例如互斥锁或者信号量。相反，他们需要返回一个QFuture对象，当它们准备就绪时，用于检索函数的结果。QFuture也可以用于查询并且暂停/恢复/取消计算。为了方便起见，QFutureWatcher通过信号与槽和QFutures进行交互。   
Qt Concurrent的map、filter和reduce算法在所有的可用的处理器核心之间自动分配计算，所以如今编写的应用程序将在未来具有更多核心的系统上部署时扩展。   
该模块还提供了QtConcurrent::run()方法，可以在另一个线程中运行任何函数。但是该函数只能支持用于map/filter/reduce函数的一个子集。QFuture可以用于检索函数的返回值，并且检查线程是否在运行。但是对QtConcurrent::run()的调用仅使用一个线程，不能被暂停/恢复/取消，而且不能接受被查询进度。   
### WorkerScript QML中的进程   
QML中提供了WorkerScript类型允许js代码和GUI线程并行运行。   
每个WorkerScript实例可以有一个js脚本附加到它，当调用WorkerScript::sendMessage()时，脚本将在单独的线程(和单独的QML上下文)中运行，当脚本完成运行时，它可以发送一个恢复给GUI线程，它将调用WorkerScript::onMessage()信号处理程序。类似于QOject的worker模式。   
[一去二三里](https://blog.csdn.net/liang19890820/article/details/52943811 )给出了各类多线程使用的注意点。

## Qt的可重入和线程安全   
可重入：   
- 一个线程安全的函数可以同时被多个线程调用，甚至共享数据也没有问题，因为对共享数据的访问是串行的。   
线程安全：   
- 一个可重入函数可以同时被多个线程调用，但是每个调用者只能使用自己的数据。   
因此一个线程安全的函数总是可重入的，但是一个可重入的函数未必线程安全。   
扩展开来，一个可重入的类，指的是它的成员函数可以被多个线程安全地调用，只要每个线程使用者类不同的对象。而一个线程安全的类，指的是它的成员函数能够被多线程安全的调用，哪怕共用一个实例。   
### 可重入性：   
C++的类往往是可重入的，这只是因为它们只能访问自己的数据。任何线程都能访问一个可重入类实例的一个成员函数，只要同一时间没有其他线程调用该实例的成员函数；例如下面这个例子就是可重入的：   
```
class Counter
{
public:
    Counter() { n = 0; }

    void increment() { ++n; }
    void decrement() { --n; }
    int value() const { return n; }

private:
    int n;
};
```
但是该类不是线程安全的，因为如果多个线程试图修改数据成员n，该结果是不确定的。这是因为++和--操作不总是原子性的。事实上，大部分时候它们被拆为三条机器码：   
1. 将变量值装入寄存器；   
2. 增加或者减少寄存器内的值；  
3. 将寄存器中的值写回内存；   
如果线程A和B同时把旧的变量装入寄存器，再加，再装回去，那么会最终覆盖，结果只加了一次。   
### 线程安全   
显然，访问应当是串行的：即线程A必须在无中断的情况下执行完三个步骤（原子性），然后线程B才能够执行。最简单的方法就是使用一个QMutex来保护数据成员：   
```
class Counter 
{
public: 
	Counter() {n = 0};

	void increment() { QMutexLocker locker(&mutex); ++n;} 
	void decrement() { QMutexLocker locker(&mutex); --n;}
	int value() const { QMutexLocker locker(&mutex); return n;}

private: 
	mutable QMutex mutex; //mutable: 与const相反，可变的，常用于在const函数中作为一个被修改的无关量；
	int n;
}
```   
### Qt类的注意事项   
许多Qt类都是可重入的，但是不总是线程安全的，因为线程安全意味着锁定与解锁一个QMutex增加了额外的开销。例如QString是可重入的，但是却不是线程安全的，你能够从多个线程访问不同的QString，但是在没有QMutex保护的情况下多个线程访问QString的同一个实例。   
有些Qt的类和函数都是线程安全的，他们主要是线程相关的类(QMutex)以及一些基本函数(postEvent（）)

## Qt的多线程
1. 正常多线程（Worker-Controller模型）：注意，在Worker的构造函数中构建的内容（new），并不属于次线程，而全部属于主线程。若需要在次线程中构建，则需要在主函数中分配资源；   

2. QRunnable和QThreadPool:   
QRunnable类是接口，表示一个任务或者需要执行的代码，需要重新实现run()函数。   
QThreadPool用于管理和循环使用单独的QThread对象，来帮助程序减少创建线程的成本。每个Qt应用程序都有一个全局的QThreadPool对象，可以通过调用静态函数globalInstance()来访问；    
QThread支持多次执行相同的QRunnable，通过调用QThreadPool::tryStart(this)从run()函数内。如果启用了autoDelete,当最后一个函数退出run()函数，QRunnable将会被删除；多次调用QThreadPool::start()使用相同的QRunnable，当启用autoDelete时，会创建一个竞争条件，不推荐使用；   
一定时间未使用线程将会到期，默认到期的超时为30s，可以使用setExpiryTimeout()来进行改变，设定一个负值，则会禁用到期机制；   
调用maxThreadCount()可以查询线程的最大数量，如果需要，可以使用setMaxThreadCount()来进行更改。默认情况下，maxThreadCount()是QThread::idealThreadCount()。activeThreadCount()函数返回当下正在工作的线程数量；   
reverseThread()函数用于存储一个线程用于外部使用。当线程完成以后，使用releaseThread()，以方便它可以重新被使用。从本质上来说，这些函数会暂时增加或者减少活跃线程的数量，并且当实现耗时操作时对QThreadPool是不可见的，这一点非常有用。   
总体来说QThreadPool是一个管理线程的低级类，高级替代品可以使用Qt Concurrent模块。   

### 基本使用   
子类化QRunnable并实现run()虚函数，然后创建一个对象，将其传递给QThreadPool::start()，这样会把可运行对象的拥有权赋给Qt全局线程池，并且让它可以运行；
```
class HelloWorldTask : public QRunnable
{
	void run() {
		qDebug() << "Hello world from thread" << QThread::currentThread();
	}
}

HelloWorldTask *hello = new HelloWorldTask();

//QThreadPool取得所有权，并且自动删除hello
QThreadPool::globalInstance()->start(hello);
```
默认的情况下，在执行结束后这个Hello实例自动删除了，不过如果你不想删，可以使用QRunnable::setAutoDelete(false)来实现。   

### 自定义信号和槽   
打开QRunnable的头文件，会发现它其实并不继承QObject，也就无法使用QObject的一些特性（信号、槽、事件等）。为了方便使用，我们选择继承QObject的方法：   
```
class HelloWorldTask : public QObject, public QRunnable
{
	Q_OBJECT
	
//自定义信号
signals: 
	void finished();
	
public: 
	void run() {
		qDebug() << "Hello thread:" << QThread::currentThreadID();
		emit finished();
	}
}
```    
使用的时候，连接对应的信号和槽就可以：
```
class MainWindow : public QMainWindow {
	Q_OBJECT
	
public: 
	explicit MainWindow(QWidget *parent = 0)
		: QMainWindow(parent)
	{
		qDebug() << "Main Thread:" << QThread::currentThreadID();
		
		HelloWorldTask *hello = new HelloWorldTask;
		connect(hello, SIGNAL(finished()), this , SLOT(onFinished()));
		QThreadPool::globalInstance()->start(hello);
	}
	
protected:
	void closeEvent(QCloseEvent* event) {
		if (QThreadPool::globalInstance()->activeThreadCount())
			QThreadPool::globalInstance()->waitForDone();
			
		evnet->accepte();
	}
	
private slots: 
	void onFinished() {
		qDebug() << "SLOT Thread:" << QThread::currentThreadID();
	}
}
```
一般为了做到安全清除，对于多线程的应用程序来说，最好在终止程序之前，停掉方所有的辅助线程。我们在closeEvent中已经做到了这一点，确保在允许终止动作之前让任何活动的线程先结束掉。调试结果如下：     
```
Main Thread : 0xb308
Hello Thread : 0xb33c
SLOT Thread : 0xb308
```    
显然，槽函数所在线程和主线程相同。如果需要槽函数在次线程中执行，只需要改变信号槽的连接方式：
```
connect(hello, SIGNAL(finished()), this, SLOT(onFinished()), Qt::DirectConnection);
```
Tip: DirectConnection指的是slot会被立刻调用，而其所在线程会取决于信号线程。正常都是AutoConnection，即同线程的时候立刻调用，非通线程默认使用QueueConnection（只有当控制权回到接受槽函数所在事件循环中时才开始调用，是在接受槽函数所在线程中被调用的）。还有BlockingQueuedConnection(直到槽函数执行完之前block)和UniqueConnection。   

事件传递方式很多，除了信号和槽以外，可以使用1. 自定义事件，然后通过调用QApplication::sendEvent()或者QApplication::postEvent()发送； 2. 使用QMetaObject::invokeMethod()都可以。    

### Threads和QObjects
QThread继承自QObjec，它通过发射信号表明线程执行的开始或者结束，并且提供了一些有用的槽函数；    
QObjects可以在多线程中使用，发射信号后在其他线程中调用槽函数，并且向“**存活**”于其他线程中的对象发送事件(post event)。这种事情是可能的，完全依赖于每个线程都有自身的“事件循环”；   

#### QObject的可重入性   
QObject是可重入的，它的绝大多数非GUI子类，例如QTimer、QTcpSocket、QUdpSocket和QProcess，也都是可重入的；
Tip: 可重入性(Reenterable)指的是一个功能模块是否可以被中断（，然后在任何时段中断它，转入OS调度下去执行另一段代码，而返回的时候没有错误）；   
注意，这些类是在单一线程中被创建和使用的，在一个线程创建对象而在另一个线程中调用该对象的函数，不能保证行得通，需要有三个约束:     
-  一个QObejct类型的孩子必须总是被创建在它父亲所被创建的线程中，这意味着，除了别的以外，永远不要把QThread对象(this)作为该线程中创建的一个对象的父亲(因为QThread对象自身永远被创建在另一个线程中);   
- 事件驱动的对象可能只能被用在一个单线程中。特别是，这适用于计时器机制和网络模块；例如：你不能在不属于这个对象的线程中启动一个定时器或者连接一个socket，必须保证在删除QThread之前删除所有创建在这个线程的对象。在run()函数中，通过在栈中创建这些对象，可以轻松做到这一点；   
- 虽然QObject是可重入的，但是GUI类，尤其是QWidget及其所有子类都是不可重入的，它们只能被用在主线程中。如前面所述，QCoreAppliction::exec()也必须从那个线程被调用;      
在实践当中，只能在主线程而不能在其他线程使用GUI的类这一点，可以轻松的被解决：将耗时的操作放在一个单独worker线程中，当worker线程结束后在主线程中由屏幕显示结果，这在QtExamples中有所体现。   
一般来说，在QApplication之前就创建QObject是不行的，会导致奇怪的崩溃或者退出，这取决于平台。这意味着，也不能支持QObject的静态实例。一个单线程或者多线程的应用程序应该先创建QApplication，并在最后销毁QObject。    

#### 每个线程的事件循环     
每个线程都有他自己的事件循环，初始线程通过QCoreApplication::exec()来启动它的事件或者对于单对话框的GUI应用程序，有些时候用QDialog::exec()。其他的线程可以用QThread::exec()来启动事件循环。就例如QCoreApplication、QThread都提供了一个exit(int)和quit()槽函数。    
一个线程中的事件循环使得该线程可以利用一些非GUI的，要求有事件循环存在的Qt类(例如QTimer、QTopSocket和QProcess)。它也使得链接一些线程的信号到一个特定线程的槽函数成为可能。   
![事件循环示意图](https://img-blog.csdn.net/20160728131021869 )   
一个QObject实例被称为存活于它所被创建的线程中，关于这个对象的事件被分发到该线程的事件循环中。可以用QObject::thread()方法获取一个QObject所处的线程。
QObject::moveToThread()会改变一个对象和其孩子的线程所属属性。（如果该对象有父亲的话，则不能被移动到其他线程中）   
从另一个线程（不是该QObject对象所属的线程）对该QObject对象调用delete方法是不安全的，除非你能保证该对象在那个时刻不处理事件，使用QObject::deleteLater()更好。一个DeferredDelete类型的事件将会被提交(posted),而该对象的线程的事件循环最终会处理这个事件。默认情况下，拥有一个QObject的线程就是创建QObject的那个线程，而不是QObject::moveToThread()被调用后的。   
如果没有事件循环运行，事件将不会传递给对象。例如：你在一个线程中创建了一个QTimer对象，但是从来没有调用过exec()，那么QTimer永远不会发射timeout()信号，即使调用deleteLater()也不行（这些限制同样适用于主线程）；    
利用线程安全的方法QCoreApplication::postEvent(),你可以在任何时候给任何线程中的任何对象发送事件，这些事件将会自动被分发到该对象所被创建的线程事件循环中去。   
所有的线程都支持事件过滤器，而限制是监控对象必须和被监控对象都要处于相同的线程中。类似的，QCoreApplication::sendEvent()（不同于postEvent()）只能将事件分发到和该函数调用者相同线程中的对象。   
Tip: sendEvent()——同步处理事件，使用notify()函数直接给receiver发送事件；（立刻执行）   
postEvent()——异步处理事件，向事件队列中添加receiver和event；（不知道啥时候执行，所以线程安全）   

#### 从其他线程访问QObject子类   
QObject及其所有子类都不是线程安全的，这包含了整个事件的交付系统，更重要的是，切记当你从另一个线程访问该对象时，事件循环可能正在向你的子类发送事件。    
如果你正在调用一个QObject子类的函数，而该子类对象并不存活于当前的线程中，并且该对象是可以接受事件的，那么你必须用一个mutex保护对该QObject子类的内部数据的所有访问，否则，就可能发生崩溃和非预期的行为。   
和其他对象一样，QThread对象存活于该对象被创造的线程中——而不是QThread::run()被调用时所在的线程。一般来说，在QThread子类中提供槽函数是不安全的，除非用一个Mutex来保护成员变量。   
另一方面，可以在QThread:run()的实现中安全的发射信号，因为信号的发射是线程安全的。   

#### 跨线程的信号和槽    
Qt支持如下的信号-槽连接类型：
- Auto Connection（默认）：如果信号在接收者所依附的线程内发射，则等同于Direct Connection。否则，等同于Queued Connection。   
- Direct Connection：当信号发射后，槽函数立即被调用。槽函数在信号发射者所在的线程中执行，而未必需要在接收者的线程中。
- Queued Connection：当控制权回到接受者所在线程的事件循环时，槽函数被调用。槽函数在接收者的线程中执行。
- Blocking Queued Connection：槽函数的调用情形和Queued Connection相同，不同的是当前的线程会阻塞住，直到槽函数返回。注意：**在同一个线程中使用这种类型进行连接会导致死锁**。
- Unique Connection：行为与Auto Connection相同，但是连接只会在“不会与已存在的连接相同”时建立，也就是：如果相同的信号已经被连接到相同的槽函数，那么连接就不会被再次建立，并且connect()会返回false。    
通过传递一个参数给connet()来指定连接的类型，要知道，如果一个事件循环运行在接收者的线程中，而发送者和接受者位于不同的线程时，使用Direct Connection是不安全的。同样的原因，调用存活于另一个线程中的任何对象的任何函数都是不安全的。   
QObject::connect()本身是线程安全的。   
QtExamples中的两个例子，都是分别使用单独的worker线程内容，一个计算完发送信号，另一个使用单独的线程和TCP Server进行通信。    

## QtConcurrent框架   
使用QtConcurrent需要添加:``QT += concurrent``模块。   
Qt Concurrent包含了函数式编程风格APIs并且用于并行列表处理。包括用于共享内存(非分布式)系统MapReduce和FilterReduce实现，以及用于管理GUI应用程序异步计算的类：   
- Concurrent Map和Map-Reduce
	- QtConcurrent::map():将一个函数应用于一个容器中的每一项，就地修改items；   
	- QtConcurrent::mapped():和map()类似，只是它返回一个包含内容的新容器；
	- QtConcurrent::mappedReduced(): 和mapped()类似，只是修改后的结果减少或组合成一个单一的结果；
- Concurrent Filter和Filter-Reduce   
	- QtConcurrent::filter(): 从一个容器中删除所有的items，基于一个filter函数的结果。   
	- QtConcurrent::filtered(): 和filter()类似，只是它返回一个包含过滤内容的新容器。   
	- QtConcurrent::filteredReduced():和filtered()类似，只是过滤后的结果减少或者组合成了一个单一的结果。   
- Concurrent Run   
	- QtConcurrent::run():在另一个线程中运行一个函数；   
- QFuture: 表示异步计算的结果   
- QFutureIterator: 允许通过QFuture遍历可用的结果   
- QFutureWatcher：允许使用信号槽啦监控一个QFuture
- QFutureSynchronizer:是一个方便的类，用于一些QFutures的自动同步   

Qt Concurrent支持多种兼容STL的容器和迭代器类型，但是最好使用具有随机访问迭代器的Qt容器，例如QVector或者QList。map和fiter函数都接受容器和begin和end迭代器。   
在Qt Concurrent迭代大量轻量级items的情况下，随机访问迭代器可以访问的更快，因为它们允许跳过容器中的任何点。此外使用随机访问迭代器Qt Concurrent通过QFuture::progressValue()和QFutureWatcher::progressValueChanged()来提供进度条信息；    
非就地修改的函数（例如：mapped()和filtered()）在调用时会创建容器的副本，如果正在使用的是stl容器，此赋值操作可能需要一定时间，这种情况下建议使用begin()和end()迭代器。   
### 单词统计   
我们来看一个单词统计的示例：   
```
#include <QList>
#include <QMap>
#include <QTextStream>
#include <QString>
#include <QStringList>
#include <QDir>
#include <QTime>
#include <QApplication>
#include <QDebug>

#include <qtconcurrentmap.h>

using namespace QtConcurrent;

// 递归搜索文件
QStringList findFiles(const QString &startDir, QStringList filters)
{
    QStringList names;
    QDir dir(startDir);

    foreach (QString file, dir.entryList(filters, QDir::Files))
        names += startDir + '/' + file;

    foreach (QString subdir, dir.entryList(QDir::AllDirs | QDir::NoDotAndDotDot))
        names += findFiles(startDir + '/' + subdir, filters);
    return names;
}

typedef QMap<QString, int> WordCount;

// 单线程单词计数器函数
WordCount singleThreadedWordCount(QStringList files)
{
    WordCount wordCount;
    foreach (QString file, files) {
        QFile f(file);
        f.open(QIODevice::ReadOnly);
        QTextStream textStream(&f);
        while (textStream.atEnd() == false)
            foreach (const QString &word, textStream.readLine().split(' '))
                wordCount[word] += 1;

    }
    return wordCount;
}

// countWords 计算单个文件的单词数，该函数由多个线程并行调用，并且必须是线程安全的。
WordCount countWords(const QString &file)
{
    QFile f(file);
    f.open(QIODevice::ReadOnly);
    QTextStream textStream(&f);
    WordCount wordCount;

    while (textStream.atEnd() == false)
        foreach (const QString &word, textStream.readLine().split(' '))
            wordCount[word] += 1;

    return wordCount;
}

// reduce 将 map 的结果添加到最终结果，该函数只能由一个线程一次调用。
void reduce(WordCount &result, const WordCount &w)
{
    QMapIterator<QString, int> i(w);
    while (i.hasNext()) {
        i.next();
        result[i.key()] += i.value();
    }
}

int main(int argc, char** argv)
{
    QApplication app(argc, argv);
    qDebug() << "finding files...";
    QStringList files = findFiles("../../", QStringList() << "*.cpp" << "*.h");
    qDebug() << files.count() << "files";

    int singleThreadTime = 0;
    {
        QTime time;
        time.start();
        // 单线程统计，与 mapreduce 机制实现的作对比
        WordCount total = singleThreadedWordCount(files);
        singleThreadTime = time.elapsed();
        // 打印出所耗费的时间
        qDebug() << "single thread" << singleThreadTime;
    }

    int mapReduceTime = 0;
    {
        QTime time;
        time.start();
        // mappedReduced 方式进行统计
        WordCount total = mappedReduced(files, countWords, reduce);
        mapReduceTime = time.elapsed();
        qDebug() << "MapReduce" << mapReduceTime;
    }
    // 输出 mappedReduced 方式比单线程处理方式要快的倍数
    qDebug() << "MapReduce speedup x" << ((double)singleThreadTime - (double)mapReduceTime) / (double)mapReduceTime + 1;
}
```   
一去二三里的博客中大约快了2.3倍左右,办公室机器为2.3466倍    

## QtConcurrent Map以及Map-Reduce   
上文中已经提到,map/mapped/mappedReduced区别为就地修改/返回新序列/返回单一结果。   
上述每个函数都有一个blocking变量，其最终返回的是最终结果，而不是一个QFuture。可以用和异步变量同样的方式来使用它们；   
```
QList<QImage> imgs = ...;

//每次调用都会被阻塞，直到整个操作完成；   
QList<QImage> future = QtConcurrent::blockingMapped(images, sacled);
QtConcurrent::blockingMap(imgs);
QImage collage = QtConcurrent::blockingMappedReduced(images, scaled, addToCollage);
```   
上述结果类型不是QFuture对象，而是实际类型结果
### QtConcurrent Map   
QtConcurret::mapped()接受一个输入序列和一个map函数，该map函数被序列中每一项调用，并且返回一个包含map函数返回值的新序列。   
map函数必须是以下形式：   
```
U function(const T &t);
```   
T和U可以是任意类型（甚至是同一类型），但是T必须匹配存储在序列中的类型，该函数返回修改或者映射的内容。下面以上述的scale函数作为示例：   
```
QImage scaled(const QImage &image)
{
	return image.scaled(100,100);
}

QList<QImage> images = ...;
QFuture<QImages> thumbnails = QtConcurrent::mapped(images, scaled);
```   
map的结果通过QFuture提供，有关如何在应用程序中使用QFuture的更多信息，需要查阅QFuture和QFutureWatcher的结果。    
如果想要就地修改一个序列，使用QtConcurrent::map()。map必须是以下形式：   
```
U function(T &t);
```
注意：map的返回值、返回类型没有被使用；
使用QtConcurret::map()和使用QtConcurrent::mapped()有些类似：   
```
QImage scaled(const QImage &image)
{
	return image.scaled(100,100);
}

QList<QImage> images = ...;
QFuture<void> future = QtConcurrent::map(images, sacle);   
```
虽然该序列被就地修改，不通过QFuture返回任何结果，但是仍然可以使用QFuture和QFutureWatcher来监控map的状态。   
### Concurrent Map-Reduce   
QtConcurrent::mappedReduced()类似于mapped()，但是不会是返回具有新结果的序列，而是返回一个组合的单个值。reduce函数的形式必须如下：   
```
V function(T &value, const U &intermediate);
```   
T是最终的类型，U是map函数的返回类型。注意：reduce函数的返回值、返回类型没有被使用。   
调用QtConcurrent::mappedReduced()如下图所示：    
```
void addToCollage(QImage &collage, const QImage &thumbnail)
{
	QPainter p(&collage);
	static QPoint offset = QPoint(0, 0);
	p.drawImage(offset, thumbnail);
	offset += ...;
}

QList<QImage> images = ...;
QFuture<QImage> collage = QtConcurret::mappedReduced(images, scaled, addToCollage);
```
对于map函数返回的结果，reduce函数将会被调用一次，并且应该讲中间体合并到结果变量中。QtConcurrent::mappedReduced()保证一次只有一个线程调用reduce，所以没必要用一个mutex锁定结果变量。   
有一个很有意思的参数是QtConcurrent::ReduceOptions枚举量提供了一个控制reduction完成的顺序，如果使用QtConcurrent::UnorderedReduced(默认)，则执行顺序是不确定的。而QtConcurrent::OrderedReduce确保reduction按照原始序列的顺序完成；   

### 附加API功能   
### 使用迭代器而不是序列
上述每个函数都有一个变量，需要一个迭代器范围，而不是一个序列： 
```
QList<QImage> images = ...;
QFuture<QImage> thumbnails = QtConcurrent::mapped(images.constBegin(), images.constEnd(), scaled);   
//map就只能运行在非常迭代器上   
QFuture<void> future = QtConcurrent::map(images.begin(), images.end(), scale);   
QFuture<QImage> collage = QtConcurrent::mappedReduced(images.constBegin(), images.constEnd(), scaled, addToCollage);
```
### Blocking变量   
上面已经提过，每个函数都有一个blocking变量，其最终返回就是其直接的结果，而不是一个QFuture。可以使用异步变量的方式来使用它们。   
``` 
QList<QImage> images = ...;

// 每次调用都会被阻塞，直到整个操作完成
QList<QImage> future = QtConcurrent::blockingMapped(images, scaled);

QtConcurrent::blockingMap(images, scale);

QImage collage = QtConcurrent::blockingMappedReduced(images, scaled, addToCollage);
```   
### 支持成员函数   
map的三种形式接受成员函数的指针，成员函数类型必须与存储在序列中的类型匹配：   
```
//挤压QStringList中的所有字符串   
QStringList strings = ...;
QFuture<void> squeezedStrings = QtConcurrent::map(strings, &QString::squeeze);

//交换images列表中所有图片的rgb值   
QList<QImage> images = ...; 
QFuture<QImage> bgrImatges = QtConcurrent::mapped(images, &QImage::rgbSwapped);

//创建一个列表中所有字符串长度的集合   
QStringList strings;   
QFuture<QSet<int>> wordLengths = QtConcurrent::mappedReduced(strings, &QString::length, &QSet<int>::insert);   
```    
Tip: 当使用QtConcurrent::mappedReduced()的时候，可以自由的混合使用正常函数和成员函数：   
```
// 可以使用 QtConcurrent::mappedReduced() 混合正常函数和成员函数

// 计算字符串列表的平均长度
extern void computeAverage(int &average, int length);
QStringList strings = ...;
QFuture<int> averageWordLength = QtConcurrent::mappedReduced(strings, &QString::length, computeAverage);

// 创建一个列表中所有图像颜色分布的集合
extern int colorDistribution(const QImage &string);
QList<QImage> images = ...;
QFuture<QSet<int> > totalColorDistribution = QtConcurrent::mappedReduced(images, colorDistribution, QSet<int>::insert);
```    

### 使用函数对象   
三种形式同样支持函数对象，可用于向函数调用添加状态。其中result_type typedef必须定义函数调用操作符的类型：
```   
struct Scaled {
	Scaled(int size) : m_size(size) {}
	typedef QImage result_type;

	QImage operator()(const QImage &image)
	{
		return image.scaled(m_size, m_size);
	}
	
	int m_size;
}; 

QList<QImage> images;
QFuture<QImage> thumbnails = QtConcurrent::mapped(images, Scaled(100));
```   

### 使用绑定函数参数   
如果想要使用一个接受多个参数的map函数，可以使用std::bind()将其转换到接受一个参数的函数，如果C++11不可用，可以使用boost::bind()或者std::tr1::bind();   
例如使用QImage::scaleToWidth()，想使用第二参数：   
```
QImage QImage::scaleToWidth(int width, Qt::TransformationMode) const;
```
scaledToWidth接受三个参数（包含this指针），不能直接使用QtConcurrent::mapped()，因为QtConcurrent::mapped()**期望一个只接受一个的参数**。为了QImage::scaledToWidth()和QtConcurrent::mapped()一起使用，必须给width和transformation mode提供一个值：   
```
std::bind(&QImage::scaledToWidth, 100, Qt::SmoothTransoformation);
```
std::bind()返回值是一个具有以下签名的函数对象(functor):   
```
QImage scaledToWidth(const QImage &image)
```
这样符合了mapeed()的要求，示例写为
```
QList<QImage> iamges =...;
QFuture<QImage> thumbnails = QtConcurrent::mapped(images, std::bind(&QImage::scaledToWidth, 100, Qt::Transformation));
```   


## Qt Concurrent Filter和Filter-reduce   
其用法和map类似，同样具有filter()、filtered()和filteredReduced()三个函数，不过都是对一个序列中的内容进行过滤；   

## Qt Concurrent Run  
QtConcurrent::run()函数在一个单独的线程中运行一个函数，函数的返回值通过QFutureAPI提供。   
### 在单独的线程中运行函数   
```
extern void aFunction();
QFuture<void> future = QtConcurrent::run(aFunction);
```   
这将在从默认的QThreadPool获取的单独的线程中运行aFunction，可以使用QFuture和QFutureWathcer监控函数的状态；   
如果要使用专门的线程池，可以传递QThreadPool作为第一个参数： 
```  
extern void aFunction();
QThreadPool pool;
QFuture<void> future = QtConcurrent::run(&pool, aFunction);
```   
### 将参数传递给函数   
将参数传递给函数，是通过将它们添加到QtConcurrent::run()直接调用在函数名之后（类似于arg），例如： 
```
extern void aFunctionWithArguments(int arg1, double arg2, const QString &string);
int integer;
double floatingPoint;
QString string;

QFuture<void> future = QtConcurrent::run(aFunctionWithArguments, integer, floatingPoint, string);
```   
每个参数的副本在调用QtConcurrent::run()的时候产生，而当函数开始执行时，这些值被传递给线程。**调用QtConcurrent::run()后对参数所做的更改对于线程来说不可见。**   
### 从函数返回值   
该函数的任何返回值都可以通过QFuture可见：   
```
extern QString functionReturningAString();
QFuture<QString> future = QtConcurrent::run(functionReturningAString);    
...
QString result = future.result();
```   
传递参数和上文所述一样：   
```
extern QString someFunction(const QByteArray &input);

QByteArray bytearray = ...;

QFuture<QString> future = QtConcurrent::run(someFunction, bytearray);
...
QString result = future.result();
```   
Tip: QFuture::result()函数阻塞并且等待通知结果可用，当函数执行完成并且结果可用时，使用QFutureWatcher获取通知。   

### 附加API功能   
### 使用成员函数   
类似map三人组，run()也接受指向成员函数的指针，第一个参数必须是const引用或者指向类的实例的指针。在调用const成员函数的时候，const引用的传递是很有用的，传递指针对于调用修改实例的非const成员非常有用。 (简单来说就是通过引用和指针来区分是常成员函数还是非常成员函数)     
例如，在单独的线程中调用QByteArray::split()(一个const成员函数)是这样做的：   
```
// 在一个单独的线程中调用 QList<QByteArray> QByteArray::split(char sep) const
QByteArray bytearray = "hello world";
QFuture<QList<QByteArray> > future = QtConcurrent::run(bytearray, &QByteArray::split, ',');
...
QList<QByteArray> result = future.result();
```   
调用非const成员函数是这样：   
```
// 在一个单独的线程中调用 void QImage::invertPixels(InvertMode mode) 
QImage image = ...;
QFuture<void> future = QtConcurrent::run(&image, &QImage::invertPixels, QImage::InvertRgba);
...
future.waitForFinished();
// 此时，image 中的像素已经被反转
```      
### 使用绑定函数参数   
在调用时，可以使用std::bind()将一些参数绑定到函数，如果C++11不可用，boost::bind()或者std::tr1::bind()是合适的替代品。   
有许多需要绑定的原因：    
- 调用需要超过5个参数的函数；   
- 简化使用常量参数调用函数；   
- 更改参数的顺序。   
调用绑定函数是这样：   
```
void someFunction(int arg1, double arg2);
QFuture<void> future = QtConcurrent::run(std::bind(someFunction, 1, 2.0));
```   


## QFuture
QFuture代表一个异步计算的结果，它允许线程与一个或者多个结果同步，而这些结果将在稍后的事件点准备就绪，该结果可以是具有默认构造函数和拷贝构造函数的任何类型。如果一个结果在调用result()、resultAt()或者results()函数不可用的时候，QFuture将进行等待，直到结果可用位置，使用isResultReadyAt()函数可以检测结果是否已经准备就绪；   
进度信息由progressValue()、progressMinimum()、progressMaximun()和progressText()函数提供。   
### 基本使用   
```
#include <QCoreApplication>
#include <QtConcurrent/QtConcurrentRun>
#include <QDebug>

void hello(const QString &name)
{
    qDebug() << "Hello" << name << "from" << QThread::currentThread();
}

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);

    qDebug() << "Main Thread" << QThread::currentThread();

    // 在一个单独的线程中调用 hello()
    QFuture<void> f1 = QtConcurrent::run(hello, QString("Qter"));
    QFuture<void> f2 = QtConcurrent::run(hello, QString("Pythoner"));

    // 阻塞调用线程并等待计算完成，确保所有结果可用
    f1.waitForFinished();
    f2.waitForFinished();
}
```   
输出结果如下所示：   
```
Main Thread QThread(0x398fc0)
Hello “Qter” from QThread(0x39c240, name = “Thread (pooled)”)
Hello “Pythoner” from QThread(0x39c280, name = “Thread (pooled)”)
```   
显然，主线程和另外两个线程均不同。   
QFuture<void>专用于获取不包含任何结果的函数，任何QFuture<T>都可以分配或者复制到QFuture<void>。如果只需要获得状态或者进度信息，而不需要实际结果数据，就一般采用这样的方法。   
### 高级操作   
QFuture最亮眼的，是它提供了一些列与正在进行的异步调用进行交互，例如使用cancel()函数取消计算；要暂停计算，使用setPaused()函数或者pause()、resume()、toogglePaused()函数之一。   
助手中有一句很关键的话：    
```
Be aware that not all asynchronous computations can be canceled or paused. For example, the future returned by QtConcurrent::run() cannot be canceled; but the future returned by QtConcurrent::mappedReduced() can
```   
意思说是并非所有的异步计算都可以被取消，例如QtConcurrent::run()返回的future不能被取消，但是QtConcurrent::mappedReduced()返回的可以。   
来看一组对于图像缩放的示例：   
```
#include <QImage>
#include <QDebug>
#include <QGuiApplication>
#include <QtConcurrent/QtConcurrentMap>

QImage scale(const QImage &image)
{
    qDebug() << "Scaling image in thread" << QThread::currentThread();
    return image.scaled(QSize(100, 100), Qt::IgnoreAspectRatio, Qt::SmoothTransformation);
}

int main(int argc, char *argv[])
{
    QGuiApplication app(argc, argv);

    const int imageCount = 20;

    // 创建一个列表包含 imageCount 个图像
    QList<QImage> images;
    for (int i = 0; i < imageCount; ++i)
        images.append(QImage(1600, 1200, QImage::Format_ARGB32_Premultiplied));

    // 使用 QtConcurrent::mapped 对每个图像应用 scale 函数（缩放）
    QFuture<QImage> thumbnails = QtConcurrent::mapped(images, scale);

    // 暂停计算
    thumbnails.pause();
    qDebug() << "isPaused" << thumbnails.isPaused();

    // 恢复计算
    if(thumbnails.isPaused()) {
        thumbnails.resume();
        qDebug() << "isStarted" << thumbnails.isPaused();
    }

    // 阻塞调用线程并等待计算完成，确保所有结果可用
    thumbnails.waitForFinished();

    // 返回 future 的所有结果
    QList<QImage> list = thumbnails.results();
    foreach (QImage image, list) {
        qDebug() << image;
    }

    qDebug() << "********************";

    // 返回 future 的结果数
    int nCount = thumbnails.resultCount();
    for (int i = 0; i < nCount; ++i) {
        QImage image = thumbnails.resultAt(i);
        qDebug() << image;
    }

    return 0;
}
```   
为了演示效果，调用了mapped()之后，使用pause()暂停计算，然后通过isPaused()来查询QFuture表示的状态。状态可以通过isCanceled()、isStarted()、isFinished()、isRunning()或者isPaused()函数来查询。   
Tip: 即使isPaused()返回为true,计算仍然可能在运行中。   
要访问future中的结果，可以使用索引位置，一些QFuture的成员函数将索引作为它们的第一个参数，例如resultAt()，使得其可以在不使用迭代器的情况下访问。   
对于拥有多个结果的QFuture对象，resultCount()函数返回连续结果的数量，这意味着从0到resultCount()的结果始终是安全的。   
### 迭代器   
QFuture提供了一个Java风格迭代器(QFutureIterator)和一个stl风格迭代器(QFuture::const_iterator):   
- Java风格迭代器：高级易于使用，但是效率略低；   
- STL风格迭代器：低级，实用麻烦，但是效率高；   
使用这两种迭代器是访问QFuture对象中的另一种方法。   
### QFutureIterator   
QFutureIterator为QFuture提供了一个Java风格的const迭代器。   
QFutureIterator<T>允许遍历QFuture<T>，其构造函数使用QFuture作为其参数，构造后就位于列表的开始端，如下：    
```
QFuture<QString> future;
...
QFutureIterator<QString> i(future);
while (i.hasNext())
    qDebug() << i.next();
```
Tip:next()函数从future返回下一个结果（如果需要，要等它变得可用）并且推进迭代器——注意，和C++风格迭代器完全不同的是，Java风格的迭代器指向结果之间，而不是直接指向结果，例如第一次调用next()将迭代器推进到结果1和结果2之间，但是返回第一个结果；   
![Java风格迭代器示意图](https://img-blog.csdn.net/20161025171723591 )
反向的迭代顺序同理：   
```
QFutureIterator<QString> i(future);
i.toBack();
while (i.hasPrevious())
    qDebug() << i.previous();
```    
如果需要查找特定的所有值，可以在循环中使用findNext()或者findPrevious()。   
可以在同一个future中使用多个迭代器，如果future在QFutureIterator处于活动状态时被修改，QFutureIterator将继续迭代原始的future，忽略修改的副本。   
### QFuture::const_iterator   
QFuture::const_iterator类为QFuture提供了一个STL风格的const迭代器。   
默认的QFuture::const_iterator构造函数会创建一个未初始化的迭代器，在开始迭代以前，必须使用QFuture::constBegin()或者QFuture::constEnd()等QFuture函数进行初始化；   
```
QFuture<QString> future = ...;

QFuture<QString>::const_iterator i;
for (i = future.constBegin(); i != future.constEnd(); ++i)
    cout << *i << endl;
```

## Qt之QFutureWatcher   
QFuture表示异步计算的结果，而QFutureWatcher则允许使用信号和槽监视QFuture，也就是说QFutureWatcher是为QFuture而生的。    
### 详细描述    
QFutureWatcher提供了有关QFuture的信息和通知，通过setFuture()函数来监视一个特定的QFuture，函数future()则返回由setFuture()设置的future。   
为了方便，QFuture的很多函数可以直接通过QFutureWatcher来进行访问，例如： progressValue()、progressMinimum()、progressMaximum()、progressText()、isStarted()、isFinished()、isRunning()、isCanceled()、isPaused()、waitForFinished()、result() 和 resultAt()。而 cancel()、setPaused()、pause()、resume() 和 togglePaused() 是 QFutureWatcher 中的槽函数。   
状态更改由 started()、finished()、cancelled()、paused()、resumed()、resultReadyAt() 和 resultsReadyAt() 信号提供，进度信息由 progressRangeChanged()、progressValueChanged() 和progressTextChanged() 信号提供。   
由函数setPendingResultsLimits()提供节流控制，当挂起的resultReadyAt()或resultsReadyAt()信号数量超过限制时，由future表示的计算将被自动节流，一旦挂起的信号数量下降到限制以下时，计算将恢复。   
示例，开始计算并当完成时获取槽回调：   
```
//实例化对象，并且连接到finished()信号   
MyClass myObject;
QFutureWatcher<int> watcher;
connect(&watcher, SIGNAL(finished()), &myObject, SLOT(handleFinished());   

//开始计算   
QFuture<int> future = QtConcurrent::run(...);
watcher.setFuture(future);   
```   
[一去二三里的图片打开程序](https://blog.csdn.net/liang19890820/article/details/52935118 )给出了一个完整的并行示例，并且即时通过QFuture返回处理的内容。     
**总体来说，QtConcurrent非常适合大量同性质的多线程任务的自动分配，提高效率大约在2.3倍左右。**    


## QThread高阶应用    
### 子类化QThread   
定义一个WorkerTherad类，让其继承自QThread，并且重写run()函数，每隔50ms刷新当前的值，然后发射resultReady信号（用于更新进度条）。    
```
#include <QThread>

<<<<<<< HEAD
class WorkerThread : public QThread
{
    Q_OBJECT

public:
    explicit WorkerThread(QObject *parent = 0)
        : QThread(parent)
    {
        qDebug() << "Worker Thread : " << QThread::currentThreadId();
    }

protected:
    virtual void run() Q_DECL_OVERRIDE {
        qDebug() << "Worker Run Thread : " << QThread::currentThreadId();
        int nValue = 0;
        while (nValue < 100)
        {
            // 休眠50毫秒
            msleep(50);
            ++nValue;

            // 准备更新
            emit resultReady(nValue);
        }
    }
signals:
    void resultReady(int value);
};
```   
主界面包含以下内容：   
```
class MainWindow : public CustomWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0)
        : CustomWindow(parent)
    {
        qDebug() << "Main Thread : " << QThread::currentThreadId();

        // 创建开始按钮、进度条
        QPushButton *pStartButton = new QPushButton(this);
        m_pProgressBar = new QProgressBar(this);

        //设置文本、进度条取值范围
        pStartButton->setText(QString::fromLocal8Bit("开始"));
        m_pProgressBar->setFixedHeight(25);
        m_pProgressBar->setRange(0, 100);
        m_pProgressBar->setValue(0);

        QVBoxLayout *pLayout = new QVBoxLayout();
        pLayout->addWidget(pStartButton, 0, Qt::AlignHCenter);
        pLayout->addWidget(m_pProgressBar);
        pLayout->setSpacing(50);
        pLayout->setContentsMargins(10, 10, 10, 10);
        setLayout(pLayout);

        // 连接信号槽
        connect(pStartButton, SIGNAL(clicked(bool)), this, SLOT(startThread()));
    }

    ~MainWindow(){}

private slots:
    // 更新进度
    void handleResults(int value)
    {
        qDebug() << "Handle Thread : " << QThread::currentThreadId();
        m_pProgressBar->setValue(value);
    }

    // 开启线程
    void startThread()
    {
        WorkerThread *workerThread = new WorkerThread(this);
        connect(workerThread, SIGNAL(resultReady(int)), this, SLOT(handleResults(int)));
        // 线程结束后，自动销毁
        connect(workerThread, SIGNAL(finished()), workerThread, SLOT(deleteLater()));
        workerThread->start();
    }

private:
    QProgressBar *m_pProgressBar;
    WorkerThread m_workerThread;
};
```   
很显然，UI界面、Worker构造函数和槽函数同属于一个线程，而run()函数位于另一个线程。   
> Main Thread : 0x34fc
> Worker Thread : 0x34fc
> Worker Run Thread : 0x4038
> Handle Thread : 0x34fc   
其中由于默认的连接方式，槽函数在接受者（主线程）执行。   
### 线程休眠   
QThread可以使用sleep进行所谓的休眠，但是还提供了一系列sleep()、msleep()、usleep()，允许分别以秒、毫秒和微妙来进行睡眠时间控制。有时候为了防止UI刷新太快，还会刻意在次线程加入休眠。   
### 在主线程中更新UI   
如果上述更新UI的槽改为Qt::DirectionConnection，就会报错，因为Qt只允许在主线程中对UI类部件进行操作。   
### 避免多次connect    
为了避免多次重复的connect，我们可以使用QThread::isRunning()方法检查是否在运行中，或者isFinished()来查询线程的状态。，并且将connect挪到构造函数中。   
```
class MainWindow : public CustomWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0)
        : CustomWindow(parent)
    {
        // ...
        connect(&m_workerThread, SIGNAL(resultReady(int)), this, SLOT(handleResults(int)));
    }

    ~MainWindow(){}

private slots:
    // ...
    void startThread()
    {
        if (!m_workerThread.isRunning())
            m_workerThread.start();
    }

private:
    WorkerThread m_workerThread;
};
```   
### 优雅的结束线程   
如果一个主线程想要关闭次线程，往往会引起一些错误，例如如果关闭主界面时，次线程还在运行，就会出现以下提示：   
>  QThread: Destroyed while thread is still running   
这是因为次线程还在运行，就结束了UI主线程，导致了整个事件循环的结束。整个问题在使用线程过程中经常遇到，尤其耗时操作。   
常见的一个方法是直接terminate()掉该线程，但是这个方法非常危险，正确的退出线程的一般思路：    
1. 发出线程等待操作，调用quit()或者exit()。   
2. 等待线程完全停止，删除创建在堆上的对象()。   
3. 适当的使用wait()等待。   
下面介绍两种方式：    
QMutex互斥锁 + bool成员变量 
这种方式是Qt4.x中常用的，主要是利用“QMutex互斥锁+bool成员变量”的方式来保证共享数据的安全性：    
```
#include <QThread>
#include <QMutexLocker>

class WorkerThread : public QThread
{
    Q_OBJECT

public:
    explicit WorkerThread(QObject *parent = 0)
        : QThread(parent),
          m_bStopped(false)
    {
        qDebug() << "Worker Thread : " << QThread::currentThreadId();
    }

    ~WorkerThread()
    {
        stop();
        quit();
        wait();
    }

    void stop()
    {
        qDebug() << "Worker Stop Thread : " << QThread::currentThreadId();
        QMutexLocker locker(&m_mutex);
        m_bStopped = true;
    }

protected:
    virtual void run() Q_DECL_OVERRIDE {
        qDebug() << "Worker Run Thread : " << QThread::currentThreadId();
        int nValue = 0;
        while (nValue < 100)
        {
            // 休眠50毫秒
            msleep(50);
            ++nValue;

            // 准备更新
            emit resultReady(nValue);

            // 检测是否停止
            {
                QMutexLocker locker(&m_mutex);
                if (m_bStopped)
                    break;
            }
            // locker超出范围并释放互斥锁
        }
    }
signals:
    void resultReady(int value);

private:
    bool m_bStopped;
    QMutex m_mutex;
};
```   
为什么要加锁？是为了共享数据段的互斥。   
何时需要加锁？在形成资源竞争时，也就是说，多个线程可能访问同一共享资源的时候。   
   
Qt5以后：requestInterruption() + isInterruptionRequested()   
这两个接口是Qt5.x引入的，使用非常方便：    
```
class WorkerThread : public QThread
{
    Q_OBJECT

public:
    explicit WorkerThread(QObject *parent = 0)
        : QThread(parent)
    {
    }

    ~WorkerThread() {
        // 请求终止
        requestInterruption();
        quit();
        wait();
    }

protected:
    virtual void run() Q_DECL_OVERRIDE {
        // 是否请求终止
        while (!isInterruptionRequested())
        {
            // 耗时操作
        }
    }
};
```      

## Qt之线程同步（生产者消费者模式-QWaitCondition）   
生产者将数据写入到缓冲区，直到它到达缓冲区的末尾，这时，它从开始位置重新启动，覆盖现有数据。消费者线程读取数据并将其写入标准错误。（？？？）   
Wait Condition（等待条件）比单独使用mutex（互斥）有一个更高级的并发性，如果缓冲区的访问由一个QMutex把守，当生产者线程访问缓冲区的时候，消费者线程将会无法访问。然而，两个线程同时访问缓冲区的不同部分是没有坏处的。    
示例包含两个类：Producer和Consumer，均继承QThread。循环缓冲区用于两个类之间的沟通，同步工具用于保护全局变量。    
### 全局变量   
首先是循环缓冲区和相关的同步工具：   
```
const int DataSize = 100000;

const int BufferSize = 8192;
char buffer[BufferSize];

QWaitCondition bufferNoEmpty;
QWaitCondition bufferNotFull;
QMutex mutex;
int numUsedBytes = 0;
```    

DataSize是生产者将生成的数据数量，为了尽可能的让示例简单，将其定义为一个常数。BufferSize是缓冲区的大小，小于DataSize，这意味着在某一时刻生产者将达到缓冲区的末尾，并从开始位重新启动。   
要同步生产者和消费者，需要两个wait条件和一个mutex。当生产者生产了一些数据的时候，bufferNotEmpty条件被发射，告诉消费者可以读取它了；当消费者读取一些数据时，bufferNotFull条件发射，告诉生产者生成更多的数据。numUsedBytes为缓冲区中所包含数据的字节数。   
总之，wait条件、mutex和numUsedBytes计数器确保生产者不会先于消费者超过BufferSize的大小，而消费者永远不会读取生产者尚未生成的数据。    
总之，wait、mutex和numUsedBytes计数器确保生产者不会先于消费者超过BufferSize的大小，而消费者永远不会读取生产者尚未生成的数据。   
### Producer   
Producer类的代码如下：   
```
class Producer : public QThread
{
public:
    Producer(QObject *parent = NULL) : QThread(parent)
    {
    }

    void run() Q_DECL_OVERRIDE
    {
        qsrand(QTime(0,0,0).secsTo(QTime::currentTime()));

        for (int i = 0; i < DataSize; ++i) {
            mutex.lock();
            if (numUsedBytes == BufferSize)
                bufferNotFull.wait(&mutex);
            mutex.unlock();

            buffer[i % BufferSize] = "ACGT"[(int)qrand() % 4];

            mutex.lock();
            ++numUsedBytes;
            bufferNotEmpty.wakeAll();
            mutex.unlock();
        }
    }
};
```
生产者生成了了DataSize字节的数据，在往循环缓冲区内写入一个字节之前，它必须先检测缓缓从曲是否已满(例如，numUsedBytes等于BufferSize),如果缓冲区满了，线程就会在bufferNotFull条件上等待。   
最后，生产者增加numUsedBytes，并且标志bufferNotEmpty条件为true，从而numUsedBytes必然大于0；   
我们使用一个mutex保护numUsedBytes变量的所有访问。此外，QWaitCondition::wait()函数接受一个mutex作为其参数，**当线程被置于休眠状态之前，该mutex被解锁；当线程被唤醒，该mutex被锁定；**此外，为了防止竞争条件发生，从锁定状态到等待状态的过渡具有原子性。   
### Consumer   
现在转向Consumer类：    
```
class Consumer : public QThread
{
    Q_OBJECT
public:
    Consumer(QObject *parent = NULL) : QThread(parent)
    {
    }

    void run() Q_DECL_OVERRIDE
    {
        for (int i = 0; i < DataSize; ++i) {
            mutex.lock();
            if (numUsedBytes == 0)
                bufferNotEmpty.wait(&mutex);
            mutex.unlock();

            fprintf(stderr, "%c", buffer[i % BufferSize]);

            mutex.lock();
            --numUsedBytes;
            bufferNotFull.wakeAll();
            mutex.unlock();
        }
        fprintf(stderr, "\n");
    }

signals:
    void stringConsumed(const QString &text);
};
```   
代码非常类似于生产者，在读取字节之前，需要先检查缓冲区是否为空(numberBytes为0)，而非它是否已满。并且，当它为空时，需要等待bnufferNotEmpty条件。在读取字节以后，减小numUsedBytes(而非增加)，并且标志bufferNotFull条件（并非bufferNotEmpty条件）；   
### main()函数   
在main()函数中，我们创建了两个线程，并且调用了QThread::wait()方法，确保在退出前，这两个线程有时间完成。 （wait函数是用于等待该命令是否执行完，可以赋予一个参数指明等待时间，如果到了等待时间还没返回finished()信号，那么就会返回false）    
```
int main(int argc, char *argv[])
{
    QCoreApplication app(argc, argv);
    Producer producer;
    Consumer consumer;
    producer.start();
    consumer.start();
    producer.wait();
    consumer.wait();
    return 0;
}
```
当运行这个程序，会发生什么？    
最初，生产者是唯一一个可以做任何事情的线程，消费者阻塞并等待bufferNotEmpty条件被发射(numUsedBytes是0)。一旦生产者把一个字节放入缓冲区，numUsedBytes就会变成BufferSize-1，并且bufferNotEmptytiaojian 被发射。这时，可能发生两件事：要么消费者线程接管和读取字节，要么生产者开始生成第二个字节。     
此示例中提出的“生产者-消费者”模式，适用于编写高并发多线程应用，在多处理器的计算机中，程序可能比基于mutex的方案快达两倍至多，因为两个线程可以同一时间在缓冲区的不同部分处于激活的状态。   
但是要知道，这些好处并不能总是实现，例如对一个QMutex加锁和解锁都是需要时间成本的。在实践中，可能需要把缓冲区区分为块，并且针对块进行操作而非单个字节。缓冲区的大小也是一个值得选择的参数，需要基于实践进行选择。   

## Qt之线程同步（生产者消费者模式-QSemaphore）   
其余情况和上一章相同，仍然使用相同的全局变量。   
### 全局变量   
```
const int DataSize = 100000;

const int BufferSize = 8192;
char buffer[BufferSize];

QSemaphore freeBytes(BufferSize);
QSemaphore usedBytes;
```    
信号量模式中，要同步生产者和消费者，需要两个信号量。freeBytes信号量用于控制缓冲区中的"free"区域（生产者尚未填充，或者消费者已经读取过的区域）, usedBytes信号量用于控制缓冲区的"used"区域（生产者已经填充，但是消费者上为读取的区域）。    
总之，这些信号量会确保生产者不会先于消费者超过BufferSize的大小，而消费者永远不会读取生产者尚未生成的数据。   
freeBytes信号量用BufferSize来初始化，因为整个缓冲区是空的。usedBytes信号量初始化为0（默认值）。   
### Producer    
Producer类的代码如下：   
```
class Producer : public QThread
{
public:
    void run() Q_DECL_OVERRIDE
    {
        qsrand(QTime(0,0,0).secsTo(QTime::currentTime()));
        for (int i = 0; i < DataSize; ++i) {
            freeBytes.acquire();
            buffer[i % BufferSize] = "ACGT"[(int)qrand() % 4];
            usedBytes.release();
        }
    }
};
<<<<<<< HEAD
```     
生产者生成了DataSize字节的数据，在往循环缓冲区写入一个字节之前，必须使用freeBytes信号量来获取一个"free"字节。如果消费者没有与生产者保持着同样的速度，QSemaphore::acquire()调用可能会阻塞。   
最后，生产者使用usedBytes信号释放一个字节，该“free”字节已经成功转化为一个"used"字节，准备好供消费者进行读取。   
### Consumer   
```
class Consumer : public QThread
{
    Q_OBJECT
public:
    void run() Q_DECL_OVERRIDE
    {
        for (int i = 0; i < DataSize; ++i) {
            usedBytes.acquire();
            fprintf(stderr, "%c", buffer[i % BufferSize]);
            freeBytes.release();
        }
        fprintf(stderr, "\n");
    }

signals:
    void stringConsumed(const QString &text);

protected:
    bool finish;
};
```   
Consumer和Producer流程正好相反，获得一个used字节并且释放一个free字节。   
### main函数  
```
int main(int argc, char *argv[])
{
    QCoreApplication app(argc, argv);
    Producer producer;
    Consumer consumer;
    producer.start();
    consumer.start();
    producer.wait();
    consumer.wait();
    return 0;
}
```   
最初，生产者是唯一一个可以做任何事情的线程，消费者阻塞并等待usedBytes信号量释放(available()初始数为0)。一旦生产者把一个字节放入缓冲区，freeBytes.available()就会变为BufferSize - 1,并且usedBytes.available()变为1。这时候可能发生两件事，要么消费者线程接管和读取字节，要么生产者开始生产第二个字节。   
=======
```
>>>>>>> bbaaa604500580ffe3cf8a6055015607a47d3cf6
