# 多线程

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
