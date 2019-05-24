#include <QCoreApplication>
#include <QThread>
#include <QDebug>
#include <QWaitCondition>
#include <QMutex>
#include <QTime>
#include <QSemaphore>

#define SEMAPHORE
#ifdef  SEMAPHORE
const int DataSize = 100000;
const int BufferSize = 8192;
char buffer[BufferSize];

QSemaphore freeBytes(BufferSize);
QSemaphore usedBytes;

class Producer : public QThread
{
public:
    void run() Q_DECL_OVERRIDE {
        qsrand(QTime(0,0,0).secsTo(QTime::currentTime()));
        for (int i = 0; i < DataSize; ++i)
        {
            freeBytes.acquire();   //尝试获取一个字节，如果获取不到，可能会阻塞
            buffer[i % BufferSize] = "AGCT"[(int)qrand()%4];
            usedBytes.release();   //释放一个字节给消费者
        }
    }
};

class Consumer : public QThread
{
    //Q_OBJECT

public:
    void run() Q_DECL_OVERRIDE
    {
        for (int i = 0; i < DataSize; ++i)
        {
            usedBytes.acquire(); //和生产者线程恰好相反
            fprintf(stderr, "%c", buffer[i % BufferSize]);
            freeBytes.release();
        }

        fprintf(stderr, "\n");
    }

//signals:
//    void stringConsumed(const QString &text);

protected:
    bool finish;
};

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

#else

const int DataSize = 100000;

const int BufferSize = 8192;
char buffer[BufferSize];

QWaitCondition bufferNotEmpty;
QWaitCondition bufferNotFull;
QMutex mutex;
int numUsedBytes = 0;

class Producer : public QThread
{
public:
    Producer(QObject *parent = NULL): QThread(parent){}

    void run() Q_DECL_OVERRIDE {
        qsrand(QTime(0,0,0).secsTo(QTime::currentTime()));

        for (int i = 0; i < DataSize; ++i)
        {
            mutex.lock();
            //在写入一个新的字节之前，必须先检查缓冲区是否已满
            //如果满了，就会在bufferNotFull条件上等待
            if (numUsedBytes == BufferSize)
                bufferNotFull.wait(&mutex);
            mutex.unlock();

            buffer[i % BufferSize] = "AGCT"[(int)qrand()%4];

            mutex.lock();
            ++numUsedBytes;
            bufferNotEmpty.wakeAll();
            mutex.unlock();
        }
    }
};

class Consumer : public QThread
{
    //Q_OBJECT
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

//signals:
//    void stringConsumed(const QString &text);
};


int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);

    Producer producer;
    Consumer consumer;
    producer.start();
    consumer.start();
    producer.wait();
    consumer.wait();

    return a.exec();
}

#endif
