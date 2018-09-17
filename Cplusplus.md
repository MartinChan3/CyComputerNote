# C++技巧

1. 在类函数定义时，private封装原本的函数类型，而在public中封装一个简单的函数入口；这是一种常用的保护函数内容的方法，更可以在运用递归运算时起到分离总函数和单个函数的作用；
2. 对于成对出现的键值内容，首先需要考虑到pair变量，同时pair变量也起到了多个函数结果返回的作用（不过相对不是特别好用，因为意味着只能传回两个变量）；pair模板变量使用first和second来访问对应的对象；
3. 类中的private、public、protect编写的次数是不固定的，可以出现多次；
4. 在List类的自定义写法中，前++使用了返回自身的引用，而后++使用了返回一个新的对象；
5. 下一次开发新的程序内容时，需要先把数据可能会用到的内容用一个表列出，然后再进行判断是否要进行处理；
6. 在C模板类中定义新的模板类时，其迭代器类应该是public类型，来用于外部进行具体位置的访问；
7. 参数表的初始化定义往往用于结构体内容的初始化当中；
8. 类模板也可以进行继承操作，但是模板参数T必须实例化；
未必，可以使用以下形式，以typename替代写入
```
//从QVector继承写循环列表
template < typename Object>
class LVector: public QVector<Object>
{
public:
    Object next( int tI)
    {
        if ( tI < 0 || tI >= this->size()) return Object();

        if ( tI < this->size() - 1)
            return (*(this))[ tI + 1];
        else
            return (*(this))[ 0];
    }
};
```
这个方法可以用来解决部分vector中没有想要的重载函数的问题；
9. 借助函数模板化的排序算法导入实例。这样在使用时，只需要对比较器类进行带类型继承和isLessThan函数的实例化，就可以达到比较的效果；
```
//比较器基类
template <typename Object>
class Comparator
{
public:
    virtual bool isLessThan( Object*, Object*) = 0;//纯虚函数，迫使后续函数对其进行实现；
};

//模板函数子类
template <typename Object>
void sort( QVector<Object> &vp, Comparator<Object>* fptr)
{
    if ( vp.size() <= 1) return;

    for ( int j = 0; j < vp.size(); j++)
    {
        for ( int i = ( vp.size() - 1); i > j; --i)
        {
            if ( fptr->isLessThan( vp.data() + i, vp.data() + i - 1))
                qSwap( vp[i], vp[i - 1]);
        }
    }
}

//实例化比较器
class QPointComparator : public Comparator<QPoint> //有一个很精彩实例化继承
{
public:
    bool isLessThan(QPoint* p1 , QPoint* p2)
    {
        if ( p1->x() < p2->x())
            return true;
        else
            return false;
    }
};

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);

    QVector<QPoint> tGrp;
    tGrp << QPoint( 4, 4)
         << QPoint( 3, 3)
         << QPoint( 2, 2);
    qDebug() << tGrp;
    QPointComparator comparator;
    sort( tGrp, &comparator);
    qDebug() << tGrp;

    return a.exec();
}

```

10. 可以使用多个函数模板，这样就释放掉了比较函数的比较器；模板其实是使用了动态编译的思想；
9中涉及的模板类使用其比较器在调用时**必须使用指针：因为原始sort函数中导入的是原先的父类对象的内容** ；
如果想要避免这种相对比较复杂的表示方法，可以直接使用双模版的模式，这里给出例子：
```
template <typename Object, typename Comparator>
void sort( QVector<Object> &vp, Comparator cmp)
{
    if ( vp.size() <= 1) return;

    for ( int j = 0; j < vp.size(); j++)
    {
        for ( int i = ( vp.size() - 1); i > j; --i)
        {
            if ( cmp.isLessThan( vp.data() + i, vp.data() + i - 1))
                qSwap( vp[i], vp[i - 1]);
        }
    }
}
```

11. 部分无法访问的错误是因为没有权限访问，需要检查继承的方式是否为public，因为**默认的继承方式为private。**
12. 在类似base基础文件集中的数据结构，应该使用的数据结构为：
0) common类：给出最基本的宏定义、模板类定义（来定义最简单的广义的比较函数等内容）
1) base类：给出最基本的点集定义
2）工具集类：给出基于当前编译库的常用函数(例如Qt、MFC等)
13. 下面这个表达写的很精彩，可以说是遍历过程中遇到循环的一个常用写法；
```
IntPoint ipNext = (i == cnt ? path[0] : path[i]);
```
14. Qt中的两种智能指针：QScopedPointer和QSharedPointer;前者主要应用于特定函数范围内，在函数结构结束的时候会进行自动回收，后者应用于整个程序运用范围，它会在最后一次调用完对象的函数执行后直接回收（内部有个计数器，来计算实例对象被调用的次数）；

15. 友元函数及友元类：C++完善的封装体制限制了用户的自由，故提供了友元类来使得“管中窥豹”成为可能。友元不可以继承，而且性质为单向；

16. 注意，经常在写双重链表的时候，需要使用>结尾时，需要在两个>中间添加空格，否则编译器会将其和>>流符混淆；

17. const常成员对象的重载(例如链表结构中的[])，之所以可以进行，那是因为常量成员函数只能用于常量实例的调用，这种情况在编译器的编译流程中是允许的；

18. C程序是否从main开始，这完全取决于程序是否使用了相应的c runtime程序，一般来说c runtime会在执行完初始程序后寻找main作为**函数入口**。

19. 多项目的管理中，为了子项目生成插件，一般会使用global类来导出，其间使用了dllexport和dllimport来进行使用（__declspec(dllexport)/Q_DECL_EXPORT）

20. 大型Qt文件的组织形式尚未明晰，需要反复确认练习；
- showbuild是有必要的，基本上按照基础的形式就可以；
- 在引用库文件的时候记得改成DESTDIR(目前)
- 分离子项目主要是为了减少编译时间

21. using用于引入一个命名空间或者一个命名空间内的一个方法；例如
```
using std::cout;
void main()
{cout << "U can use cout without std here";}
```

22. 目前QVector、QList等容器类当中，必须添加一部分例如等于符号等的运算符，来使得传统的类型得以添加；

23. 平时编译，出现类定义错误时，一定要检查是否所有的类的头文件声明末尾处加上了“;”;

24. Qt/QML背后的核心系统——元对象(meta-object),例如使用invokeMethod宏来标记那些可以被qml前端调用的函数；
元对象本身主要是指描述另一个对象结构的对象，包含一个对象有多少个成员函数以及有哪些属性（QMetaObject），它是基于QObject、使用Q_OBJECT（进而实现信号与槽机制）以及元对象编译器(Meta-Object Compiler,moc)来实现的；

25. 通过控制台的message输出可以看到
- DESTDIR 目标输出的文件夹，包括lib/dll/exe
- PWD 当前工程文件夹
- OUT_PWD 只shadow-build文件夹

26. qmake中Config配置ordered是指最终实际的编译顺序是按照SUBDIRS给的子项目顺序来进行编译；平时编写类似多文件输出的结构时，也应当按照类似的结构进行处理

27. 时刻牢记：stl风格的vector与list都存在空的表头和表尾，对于QVector和QList也是同理；

28. Qt Creator的debug界面左下角可以看行号，考虑平时报错是否可以以其为判断依据判断运行到具体哪一行；

29. 按照内存分配也需要时间的原则，尽可能在常量操作时，使用引用或者指针的方式进行处理；

30. 注释过多是一种非常愚蠢的行为，一方面会过多暴露写作意图，另一方面增加了代码的冗余性；应当以函数标题和适当的注释来增加代码的可读性；

31. stream类型不可以倒过来写，作为被引入数据的一方，必须放置于之前；例如```byte << stream```不被允许；

32. 小技巧：在数据传输包过程中，常常使用10以上的传输标志位作为信号，这样可以避免和数字混淆；

33. md5和sha256都可以作为文件加密的一部分，来检验文件是否被修改过；

34. 在传递过程中，一定要考虑结构体的灵活使用，这样可以避免输入函数写的过于冗长；

35. 此外xml文件要求完整，不能有多余的数据在尾巴处才可以被QXmlStreamReader正确识别；

36. QDataStream会默认写入QByteArray会默认写入一个字头（表示类型），应当直接用dStream.writeRawData方法来正确写入；

37. 类中类、结构体都是可以自由使用的，只不过限定了其使用域而已，这是在部分比较函数等情况中可以使用的技巧，增加了类的封装性；（貌似Qt debugger当中查看类中类是比较慢的）

38. 在QVariant传输过程中，经常会类似上下位机传输信息的模式，使用文件头来进行数据的传输；

39. 好的代码，可以被1000个人重用；（代码的重用性非常重要）