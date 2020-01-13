# C++技巧

1. 在类函数定义时，private封装原本的函数类型，而在public中封装一个简单的函数入口；这是一种常用的保护函数内容的方法，更可以在运用递归运算时起到分离总函数和单个函数的作用；
2. 对于成对出现的键值内容，首先需要考虑到pair变量，同时pair变量也起到了多个函数结果返回的作用（不过相对不是特别好用，因为意味着只能传回两个变量）；pair模板变量使用first和second来访问对应的对象；
3. 类中的private、public、protect编写的次数是不固定的，可以出现多次；
4. 在List类的自定义写法中，前++使用了返回自身的引用，而后++使用了返回一个新的对象；（20190726：这是前加比后加效率高的一个原因？结论正确）
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
13. 下面这个表达写的很精彩，可以说是遍历过程中遇到循环的一个常用写法；（环形队列遍历）
```
IntPoint ipNext = (i == cnt ? path[0] : path[i]);
```
14. Qt中的两种智能指针：QScopedPointer和QSharedPointer;前者主要应用于特定函数范围内，在函数结构结束的时候会进行自动回收，后者应用于整个程序运用范围，它会在最后一次调用完对象的函数执行后直接回收（内部有个计数器，来计算实例对象被调用的次数）；

15. 友元函数及友元类：C++完善的封装体制限制了用户的自由，故提供了友元类来使得“管中窥豹”成为可能。友元不可以继承，而且性质为单向；

16. 注意，经常在写双重链表的时候，需要使用>结尾时，需要在两个>中间添加空格，否则编译器会将其和>>流符混淆；

17. const常成员对象的重载(例如链表结构中的[])，之所以可以进行，那是因为常量成员函数只能用于常量实例的调用，这种情况在编译器的编译流程中是允许的；

18. C程序是否从main开始，这完全取决于程序是否使用了相应的c runtime程序，一般来说c runtime会在执行完初始程序后寻找main作为**函数入口**。（这与MSVC的四个选项MD/MT/MDd/MTd不谋而合）

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
- OUT_PWD 指的是shadow-build文件夹

26. qmake中Config配置ordered是指最终实际的编译顺序是按照SUBDIRS给的子项目顺序来进行编译；平时编写类似多文件输出的结构时，也应当按照类似的结构进行处理

27. 时刻牢记：stl风格的vector与list都存在空的表头和表尾，对于QVector和QList也是同理；

28. Qt Creator的debug界面左下角可以看行号，考虑平时报错是否可以以其为判断依据判断运行到具体哪一行；

29. 按照内存分配也需要时间的原则，尽可能在常量操作时，使用引用或者指针的方式进行处理；

30. 注释过多是一种非常愚蠢的行为，一方面会过多暴露写作意图，另一方面增加了代码的冗余性；应当以函数标题和适当的注释来增加代码的可读性；

31. stream类型不可以倒过来写，作为被引入数据的一方，必须放置于之前；例如```byte << stream```不被允许，而应当写为``stream >> bytes``；

32. 小技巧：在数据传输包过程中，常常使用10以上的传输标志位作为信号，这样可以避免和数字混淆；

33. md5和sha256都可以作为文件加密的一部分，来检验文件是否被修改过；

34. 在传递过程中，一定要考虑结构体的灵活使用，这样可以避免输入函数写的过于冗长；

35. 此外xml文件要求完整，不能有多余的数据在尾巴处才可以被QXmlStreamReader正确识别；

36. QDataStream会默认写入QByteArray会默认写入一个字头（表示类型），应当直接用dStream.writeRawData方法来正确写入；(非常重要，这意味着QDateStream并不是标准的raw类型写入特征)

37. 类中类、结构体都是可以自由使用的，只不过限定了其使用域而已，这是在部分比较函数等情况中可以使用的技巧，增加了类的封装性；（貌似Qt debugger当中查看类中类是比较慢的）

38. 在QVariant传输过程中，经常会类似上下位机传输信息的模式，使用文件头来进行数据的传输；

39. 好的代码，可以被1000个人重用；（代码的重用性非常重要）

40. 在枚举类型不定义枚举类型的名称时，其等价于define具体的值，好处在于自动补全赋值，而不用一口气连续定义几个值；

41. 日后需要完善库类文件的定义问题：
首先明确一点：**需要导出的结构体、类及方法需要在头部添加好对应的dll生成的宏，否则生成的dll及lib不会提供对应内容的入口，编译时会报对应的函数部分没有实现。** 类不需要对内部的函数结构再进行宏标示。

42. 枚举类型输出时只会输出其实际值，如果要输出其枚举值，只能使用类似“映射”的方法，写一个枚举类型的返回其名的字符串函数；（20190726：Qt里提供了另一个非常好的方法是利用Q_ENUM将其注册到MetaType中，之后就可以使用QMetaEnum::fromType去获取name等内容）

43. 比较语句中的倒序写法的好处，是防止写成等于号，例如```3 == a```可以避免```a = 3```这种少写等于号的错误的情况；

44. Qt中QString存在一个很明显的问题：当在qDebug中输出时，仍然使用双反斜杠来替代实际内存中的单斜杠；（这个斜杠也影响到了QSettings中的输出问题）

45. QString::number中的precision指的是**小数位数**而非数学概念中的有效精度；（小数点后的有效位数）

46. C++中，应当在最基础的结构体或者类型中，只存储最简单的数据结构（例如点集数据），由其本身可以推断出的数据内容，应当作为函数进行推断，这样可以以最简洁的方式传递结构体本身的数据内容，而不影响数据的使用；也可以提高代码的**可重用性**；（或者说一个基本结构的处理集也应该写作为一个类进行处理）

47. 类当中的const函数是好事，但是**const函数无法调用类中非const函数**，这意味着需要非常好的框定好函数本身内容的权限，编译器会报错；

48. C++中首字母不可以是数字，否则编译器会报很诡异的错误；

49. 在Qt乃至std库中目前还没有发现文本读入或者读出模块有“插入字节”的功能，所以一旦涉及到内容的修改，似乎必须会需要重新创造一个文件来代替原来的文件；

50. QFile提供了函数**peek**来进行尝试性的读取，类似于先读取再跳转回去（即read和skipRawData(-num)的组合）；（这个功能天生适合于读取类似cli或者其他格式的数据文件）

51. 注意QFile的方法```skipRawData(int offset)```的输入参数为偏置系数，而不是定位到具体哪一位；

52. Clipper库中图形顺时针和逆时针的顺逆时针准则，主要取决于**始终保持实体在轮廓的左侧**这一定律；此外，需要考虑该点为凹点还是凸点时，需要考虑顺时针和逆时针；   

53. C++中结构体有个很好的特性，它的内部存储结构是连续的，这意味着基本上可以直接将其传递给GLSL，并且方便offsetof函数的使用。   

54. Qt当中可以使用类似下方的语句来锁定OpenGL的版本处理，类似GLFW中的版本控制：   
```
int main(int argc, char *argv[])
{
    QSurfaceFormat fmt;
    fmt.setDepthBufferSize(24);
    if (QOpenGLContext::openGLModuleType() == QOpenGLContext::LibGL) {
        fmt.setVersion(3, 3);
        fmt.setProfile(QSurfaceFormat::CompatibilityProfile);
    } else {
        fmt.setVersion(3, 0);
    }
    QSurfaceFormat::setDefaultFormat(fmt);
    QGuiApplication app(argc, argv);
    ...
}
```   

55. Interview错误：    
1） 重载函数是否可以返回类型不同？（不可以！）   
2） 虚函数能否应用于构造函数或者虚函数？（基类的构造函数必须为非虚函数，而基类的非构造函数必须为虚函数）   
3） 常用设计模式（单例）
4） OpenGL的几何着色器（以图元装配结果为输入，通过构造新的顶点构造出(或者其他的)图元来生成形状）
5） Map二叉树相关内容：搜索速度为logN   
![kXLDPg.png](https://s2.ax1x.com/2019/03/05/kXLDPg.png)   
6） 常用数据结构的底层实现，以及各种排序算法；
7） C++三大特性：多态、继承和封装
> 多态：父类由多个子类继承，然后父类的指针在调用子类的同一名称函数得到不同的结果，这称作为多态；  
> 封装： 将对象的属性方法进行隐藏，只对特定的对象提供访问权限；  
> 继承：以现有类为基础，复制并添加和修改副本来创建新类。继承现有类型的同时，子类拥有父类的所有成员，更重要的是子类复制了基类的接口，即所有可以发送给基类对象的消息同时也可以发送给导出对象，子类的确继承了父类的所有属性和方法，因为权限修饰符的关系，访问也有一些限制；    
8） 智能指针：
9） 野指针：野指针指的是指向合法内存，但是却在内存已经释放的指针。   
> 预防方法：   
> - 记得初始化指针；
> - 指针没有价值时，记得释放；
> - 作参数时，需要先对指针进行参数检查；   
> - 尽量使用引用代替指针；   
> - 使用智能指针(shared_ptr)。每次存取之前针对该指针进行有效性检查；   

56. **构造函数不能是（或者调用）虚函数的原因**：（先见59）VPTR指针指向虚函数列表，执行虚函数的时候，会调用VPTR指针指向虚函数的地址；   
当定义一个对象的时候，首先会分配对象内存空间，然后调用构造函数来初始化对象。VPTR变量是在构造函数中进行初始化的，有因为执行虚函数需要vptr进行调用，这样陷入了鸡生蛋诞生记的循环；   
**基类析构函数必须为虚函数**：我们都知道想要回收一个对象的申请，需要调用析构函数；虽然我们没有显式的调用析构函数，但是编译器会帮我们默认的调用析构函数；   
但我们执行BaseClass* base = new BaseClass(),当执行delete base的时候，会自动调用base的析构函数进行释放；而我们执行BaseClass *sub = new SubClass();如果BaseClass基类的析构函数不是虚函数，如果调用delete sub对象的时候，只会释放BaseClass基类申请的资源，而不会释放SubClass派生类的资源，原因是：   
**基类指针指向了派生类对象，而基类的析构函数是非virtual的，而虚函数是动态绑定的基础，而现在是静态绑定，因此不会调用派生类的析构函数，造成内存泄漏；**   
[csdn](https://blog.csdn.net/gochenguowei/article/details/79682681 )   

57. 智能指针：[博客园](https://www.cnblogs.com/wxquare/p/4759020.html )    
1) 智能指针作用：C++11中引入智能指针概念，方便管理内存，同时避免普通指针容易造成的**堆内存泄漏、二次释放、程序异常时内存泄漏**等问题。   
理解智能指针需要考虑三个方面：   
1. 简单来看，智能指针使用了RAII技术(资源获取即初始化)的技术对普通的指针进行封装，这使得智能指针实质是一个对象，而行为表现类似指针；   
2. 智能指针的作用是防止忘记调用delete释放内存和程序异常进入catch忘记释放内存。另外指针的释放时机也非常考究，多次释放同一个指针会造成程序崩溃。这些都可以通过智能指针进行解决。   
3. 智能指针还有一个作用是把值语义转化为引用语义；众所周知C++和Java、python等语言有个重要的区别就是：   
```
Animal a = new Animal();
Animal b = a;
```
Java中只生成了一个对象，a、b都沦为这个对象的引用。但是C++中这是绝对意义上的深拷贝；   
2) 智能指针的使用   
智能指针在C++11版本中提供，包含在<memory.h>中，包含shared_ptr, unique_ptr和weak_ptr三个。   
2.1 shared_ptr的使用   
shared_ptr多个指针指向相同的对象时，使用了引用计数，每一个shared_ptr的拷贝都指向相同的内存，没使用一次，内部引用计数加1，每析构一次，内部引用计数-1，减为0时，会自动删除所指向的堆内存。**shared_ptr的内部引用计数线程是安全的，但是对象读取需要加锁。**    
- 初始化。智能指针是模板类，可以指定类型，传入指针通过构造函数初始化。也可以使用make_shared函数初始化。注意，不能够把指针直接赋给一个智能指针，一个是类，一个是指针，例如``std::shared_ptr<int> p4 = new int(1);``的写法是错误的（但是反过来可以）；
- 拷贝和赋值。拷贝使得对象引用计数+1,赋值使原对象引用计数-1，当计数为0时，自动释放内存。后来指向的对象引用计数加1，指向后来的对象；    
- get函数获取原始指针；   
- 注意不要使用一个原始指针初始化多个shared_ptr，否则仍将出现二次释放同一块内存；   
- **注意避免循环引用**：shared_ptr的最大一个陷阱在于循环引用，会造成堆栈内存无法正确释放，最终导致内存泄漏。循环引用会在weak_ptr中介绍；   
来看例子：   
```
#include <iostream>
#include <memory>

int main() {
    {
        int a = 10;
        std::shared_ptr<int> ptra = std::make_shared<int>(a); //用栈内容初始化？？
        std::shared_ptr<int> ptra2(ptra); //直接用指针本身赋值
        std::cout << ptra.use_count() << std::endl;  //结果：2

        int b = 20;
        int *pb = &a;
        //std::shared_ptr<int> ptrb = pb;  //error
        std::shared_ptr<int> ptrb = std::make_shared<int>(b);
        ptra2 = ptrb; //assign
        pb = ptrb.get(); //获取原始指针

        std::cout << ptra.use_count() << std::endl;   //结果：1
        std::cout << ptrb.use_count() << std::endl;   //结果：2
    }
}
```
2.2 unique_ptr的使用    
unique_ptr拥有“唯一的”它所指向的对象，同一个时刻只能有一个unique_ptr指向给定的对象(通过禁止拷贝语义、只有移动语义来实现)。相比原始指针，unique_ptr用于RAII的特线性，使得在出现异常的情况下，动态资源能够得到释放；   
unique_ptr指针本身的生命周期：从unique_ptr指针创建开始，直到离开作用域。离开作用域时，若其指向对象，则将其所指的对象销毁（默认使用delete操作符，用户可以指定其他操作）。    
unique_ptr指针与其所指的对象关系：在智能指针生命周期内，可以改变智能指针所指的对象，如创建智能指针时通过构造函数指定、通过reset函数指定、通过release函数释放所有权、通过移动语义转移所有权；   
```
#include <iostream>
#include <memory>

int main() {
	{
		std::unique_ptr<int> uptr(new int(10)); //绑定动态对象（和shared_ptr不同）   
		//std::unique_ptr<int> uptr2 = uptr; //不能赋值
		//std::unique_ptr<int> uptr2(uptr);  //不能拷贝
		std::unique_ptr<int> uptr2 = std::move(uptr); //转换所有权   
		uptr2.release(); //释放所有权
	}
	//超过uptr作用域，内存释放
}
```

58. 多态实现的基础：**1）需要有继承；2）需要有虚函数重写；3）父类指针(引用)指向子类对象；**   
静态联编和动态联编：当使用虚函数的多态，就是所谓的**动态联编，因为编码器并不提前知道调用相同方法的对象产生的结果是哪个产生的**。而普通的函数重载是静态联编的典型形式。   
如果父函数不加virtual函数，那么是单纯的重定义，只有有虚函数标识，则为重写（会发生多态）。多态也是设计模式的基础；      

59. 多态的核心原理： 
```
class Parent{
    public:
        Parent(int a=0){
            this->a = a;}
        virtual void print(){ //地方1  
            cout<<"parent"<<endl;}  
    private:
        int a;
};

class Son:public Parent{
    public:
       Son(int a=0,int b=0):Parent(a){
           this->b = b;}
       void print(){
           cout<<"Son"<<endl;}
    private:
        int b;
    };

  void play(Parent *p){ //地方2  
        p->print();}

    void main(int argc, char const *argv[])
    {
        Parent p; //地方3  
        Son s;
        play(&s)
        return 0;
    }
```
多态的发生其实在3处，真正绑定vptr指针的地方，在创建对象的时候，**C++编译器给了对象添加了一个vptr指针，而类中创建的虚函数会存在一个虚函数列表中，vptr指针就是这个表的首地址**；    
![虚函数分配](https://images2017.cnblogs.com/blog/1136072/201709/1136072-20170903163845968-129670418.png)
![虚函数指针指向对象](https://images2017.cnblogs.com/blog/1136072/201709/1136072-20170903163048280-708265652.png)   
发生多态的地方，编译器根本就不会去进行区分传进来的是子类对象还是父类对象，而是关心print()是否为虚函数，如果为虚函数，就根据不同对象的vptr指针来找到属于自己的函数。而父类对象和子类对象都会有vptr指针，根据传入的对象不同，编译器会根据vptr指针，到属于自己的虚函数列表中找到属于自己的函数，即流程为：vptr->虚函数表->函数的入口地址，从而实现了迟绑定/动态联编（在运行的时候才会去判断）；    
特点1： 通过虚函数列表指针VPTR调用重写函数是在程序运行的时候进行的，因此需要进行寻址操作，这一点决定了没有静态联编的效率高（虚函数效率略低）；
特点2：出于效率考虑，没必要将所有成员函数都声明为虚函数；   
特点3：C++编译器，当执行play函数，无需区分是子对象还是父类对象；    

60. 证明VPTR的存在：    
```
class Parent1{
  public:
        Parent1(int a=0){
            this->a = a;}
        void print(){ 
            cout<<"parent"<<endl;}  
    private:
        int a;};
class Parent2{
   public:
        Parent2(int a=0){
            this->a = a;}
        virtual void print(){ 
            cout<<"parent"<<endl;}  
    private:
        int a;};

    void main(int argc, char const *argv[]){
        cout<<"Parent1"<<sizeof(Parent1)<<endl; //4
        cout<<"Parent2"<<sizeof(Parent2)<<endl; //8
        return 0;
    }
```

61. 03.08 PAY ATTENTION TO:
1. 相对简单的几个设计模式及优缺点；
2. STL格式及ASSIMP格式导入的实现；
3. 光照内容；
4. 入门的几个图形学算法以及水平线扫描法；   
5. 温习下MetaObject的内容及使用注意点； 

62. 问题：在有序序列中插入一个数字，并且返回其索引：
```
//看问题是否能够使用二分法，进而减少比较的次数，即使用计算机中常用的比较方法来进行比较    

```

63. C++本地函数的析构顺序和构造顺序相反；   

64. deleteLater和delete的区别：deleteLater是向事件主循环发送一个event，等到事件主循环到下一次之后再进行删除；   
   
65. C++11新特性总结：（别总结了，先看C++11的书吧）（[链接](https://blog.csdn.net/jiange_zh/article/details/79356417)）   

66. 前加和后加的区别：从标准库模板中来看，前加(++i)效率大于后加(i++)，因为前者是返回引用，后者是返回一个新值；   
![前加后加在模板库中的区别](https://img-blog.csdn.net/20151228000947030 )   

67. 已知空间上有两点坐标，求第三点到这条直线的最短距离：   
使用投影矩阵法，见[csdn](https://blog.csdn.net/wanhongluli/article/details/82992090 )   

68. 0312Interview:   
1) 计算公共父节点（首先学会遍历寻找特定子结点遍历的办法，参见[博客园](http://www.cnblogs.com/neuzk/p/9486730.html )）
>  考虑下对数性指表：a^loga(b)=b；
2) 共线算法（共同矢量）
3) Canvas绘制（几种变换如何进行）
4) 多线程控制输出（互锁机制）
5) 机器语言prase
6) 路径优化

69. 树的遍历：前序遍历、中序、后序三种遍历方式（已实现）

70. 关于多线程和mutex:本质上需要了解生产者和消费者模型的价值；这里需要具体写一章对应的内容；（这里Qt和std实现的方式还不尽相同，需要单独进行学习和处理）      

71. C++11中存在谓词系统（Predicate），用来在类似count_if函数中进行判断返回的条件。   

72. C++11后有了标准的std::thread，其用法值得研究

73. [仿函数](https://www.cnblogs.com/yyxt/p/4986981.html ) ([链接2](https://www.cnblogs.com/decade-dnbc66/p/5347088.html))
：STL提供的各种算法往往有两个版本，其中一个版本表现出最直观的某种运算，另一个版本则会表现出最泛化的演算流程，允许用户通过“以template为参数来指定所要采取的策略”。这个类似于sort()函数，其中第一版本以operator<为排序依据，第二版本则允许任何操作。   
如果要将某种“操作”当做算法的参数，唯一的办法就是将该“操作”（可能拥有数条指令）设计为一个函数，再将函数指针当做算法的一个参数；或是将该“操作”设计为一个所谓的仿函数(从语言层面上来说，是个class), 再以该仿函数产生一个对象，并以此对象作为算法的一个参数；   
根据以上陈述，既然函数指针可以达到类似的效果，为何还需要有所谓的仿函数呢？原因在于**函数指针不能满足STL对抽象的要求，也不能满足软件积木的要求——函数指针无法和STL其他组件搭配，同时函数指针还不能保存信息**。   
就实现观点而言，仿函数其实就是个“行为类似函数的对象”。为了做到这点，其类别定义必须自定义（或说改写或说重载）function call运算子(operator())。拥有这样的算子以后，我们就可以在仿函数的对象后面加上一对小括号，一次调用仿函数所定义的operator()。如下：   
```
#include <iostream>
using namespace std;
int main()
{
	greater<int> ig;
	cout << boolalpha << ig(4,6); //boolalpha标志符，用于输出布尔变量
	cout << greater<int>()(6,4);
}
```
其中第一种用法比较为大家所熟悉，``greater<int>ig``是产生一个名为ig的对象，ig(4,6)则是调用其operator()，并且给予两个参数4,6。第二个用法是一个临时的对象，之后的6和4才是指定的参数。   
上述第二中语法一般不常见，但是在STL应用当中，却是主流用法。（STL仿函数绝大部分采用这种语法）   
STL仿函数的分类，若以操作个数划分，可以分为一元仿函数，二元仿函数；若以功能划分，则可分为算数运算、关系运算、逻辑运算三大类。任何应用程序如果想要使用STL内建仿函数，需要引入``<functional>``头文件。   
二、 可配接的关键   
STL仿函数应该有能力被函数配接器修饰，彼此像积木一样被串接。为了拥有配接能力，每一个仿函数都必须定义自己相应的型别。，就像迭代器如果要融入整个STL大家庭，就必须依照定义定义自己的5个相应型别一样。这些相应的型别是为了让配接器能够取出，获得某些仿函数的某些信息(仿函数能保存信息，但是函数指针不行)。相应的型别都是一些typedef，所有必要的操作都在编译期都已经完成了，对程序的执行效率没有影响，不会带来任何的额外负担。   
仿函数的相应型别主要用来表现函数参数型别和传回值型别。为了方便起见，functional定义了两个classes，分别用来代表一元仿函数和二元仿函数（STL不代表三元仿函数），其中没有任何data members或member functions，唯有一些型别定义。任何仿函数，只要依个人需求选择继承其中一个class，便自动拥有了对应的型别，也就拥有了配接能力。   
1. unary_function:呈现一元函数的参数类别和返回值类型，定义简单：   
```
template <class Arg, class Result>
struct unary_function{
	typedef Arg arguement_type;
	typedef Result result_type;
};
```   
一旦某个仿函数继承了unary_function，其用户便可以渠道该仿函数的参数型别，并且以相同手法获得其返回值的型别；   
```
//以下仿函数继承了unary_function
template<class T>
struct negate: public uanry_function<T,T>
{
	T operator() (const T &x) const {return -x;}
};


// 以下配接器（adapter）用来表示某个仿函数的逻辑负值
template <class Predicate>
class unary_negate
    ...
public:
    // 模板中,需要typename来指明后面的定义是一个类型
    bool operator() (const typename Predicate::argument_type& x) const 
    {
        ....
    }
};
```

2. binary_function: 用来呈现二元函数的第一参数型别，第二参数型别以及返回值型别。定义简单：   
```
// STL规定，每一个Adaptable Binary Function 都应该继承此类别
template <class Arg1, class Arg2, class Result>
struct binary_function
{
    typedef Arg1 first_argument_type;
    typedef Arg2 second_argument_type;
    typedef Result result_type;
};
// 以下仿函数继承了binary_function
template <class T>
struct plus : public binary_function<T, T, T>
{
    T operator() (const T& x, const T& y) const { return x+y; };
};

// 以下配接器（adapter）用来将某个二元仿函数转化为一元仿函数
template <class Operation>
class binder1st
    ....
protected:
    Operation op;
    typename Operation::first_argument_type value;
public:
    // 注意，这里的返回值和参数，都需要加上typename，告诉编译器其为一个类型值
    typename Operation::result_type operator() (const typename Operation::second_argument_type& x) const { ... }
};
```

74. bind2nd [link](https://www.cnblogs.com/renyuan/p/6216375.html )   
bind1st和bind2nd的根本作用在于将二元仿函数化为一元仿函数，来供模板函数进行调用，区别在于1st以一元函数的传递值作为第一参数，2nd是作为第二参数

75. 首先需要明确，C++中的特定多线程操作是一套，Qt中操作是另一套，需要明确：
- 进程与线程关系；   
- 线程在stl中的概念和使用；
- 对应在Qt当中的实现；

76. C/C++被设计为一门由右向左的设计语言，这意味着先从一行的最右侧开始考虑；

77. C++的堆基础知识：
堆(heap)首先不是一个容器，而是一种数据组织方式；堆是一种完全二叉树(叶节点只发生在最后一层，若倒数第二层都有左右子叶，那么算是满二叉树)。   
堆的定义：该堆是一个完全二叉树，每个节点和其子结点位置相对。父节点总是大于或者等于子结点，这种情况被称为**大顶堆**，或者父节点总是小于或等于子结点，这种情况称为**小顶堆**。注意，给定父节点的子结点不一定按顺序排列。   
#### 创建堆：   
algorithm的头文件中包含了创建堆的函数，make_heap()可以对随机访问迭代器指定的一段元素重新排列，生成一个大顶堆，如果没有指定比较函数，则**默认使用<**运算符，如下例：
```
std::vector<double>numbers{2.5,10.0,3.5,6.5,8.0,12.0,1.5,6.0};
std::make_heap(std::begin(numbers), std:rend(numbers));
// Result: 12 10 3.5 6.5 8 2.5 1.5 6
```   
![保存在vector中的堆](http://c.biancheng.net/uploads/allimg/180913/2-1P913154502555.jpg)   
stl本身还提供了priority_queue，但是本身的make_heap()可以有一些又是，例如可以访问堆中的任意元素、可以在提供任何随机访问迭代器的序列容器中创建堆（包含了普通数组、string对象等）  
以下语句生成了一个小顶堆，这里使用了三参数的std::make_heap。    
```
std::vector<double> numbers {2.5, 10.0, 3.5, 6.5, 8.0, 12.0, 1.5, 6.0};
std::make_heap(std::begin(numbers), std::end(numbers), std::greater<>()); 
// Result: 1.5 6 2.5 6.5 8 12 3.5 10
```   
#### 堆操作：   
堆不是容器，而是组织容器元素的一种特别方式。使用push_heap()创建堆的方式，使用它来插入新的元素(先插入元素，再push_heap)：   
```
std::vector<double> numbers {2.5, 10.0, 3.5, 6.5, 8.0, 12.0, 1.5, 6.0};
std::make_heap(std::begin(numbers),std::end(numbers));
// Result: 12 10 3.5 6.5 8 2.5 1.5 6
numbers.push_back(11); // Result: 12 10 3.5 6.5 8 2.5 1.5 6 11
std::push_heap(std::begin(numbers), std::end(numbers));
// Result: 12 11 3. 5 10 8 2. 5 1. 5 6 6. 5
```   
自建的比较函数也是支持的，但是注意需要使用和push_heap()相同的比较函数：   
```
std::vector<double> numbers {2.5, 10.0, 3.5, 6.5, 8.0, 12.0, 1.5, 6.0};
std::make_heap(std::begin(numbers), std::end(numbers),
std::greater<>());//Result: 1.5 6 2.5 6.5 8 12 3.5 10 numbers. push—back(1. 2);
//Result: 1.5 6 2.5 6.5 8 12 3.5 10 1.2
std::push_heap(std::begin(numbers), std::end(numbers),std::greater<>());
//Result: 1.2 1.5 2.5 6 8 12 3.5 10 6.5
```

删除最大元素和添加元素看上去过程类似，但是做的事情相反，先调用pop_heap()，再移除最大的元素：   
```
std::vector<double> numbers{2.5, 10.0, 3.5, 6.5, 8.0, 12.0, 1.5, 6.0};
std::make_heap(std::begin(numbers),std::end(numbers));
//Result:12 10 3.5 6.5 8 2.5 1.5 6
std::pop_heap(std::begin(numbers),std::end(numbers));
// Result:10 8 3.5 6.5 6 2.5 1.5 12
numbers.pop_back();// Result:10 8 3.5 6.5 6 2.5 1.5
```    
pop_heap()将第一个元素移动到最后，并且保证剩余的元素仍然是一个堆。然后使用vector成员函数pop_back()移除最后一个元素。如果make_heap()中使用了比较函数，那么pop_heap()的第三个参数也要是该参数:   
```
std::vector<double> numbers {2.5, 10.0, 3.5, 6.5, 8.0, 12.0, 1.5, 6.0};
std::make_heap(std::begin(numbers),std::end(numbers),std::greater<>());
// Result: 1.5 6 2.5 6.5 8 12 3.5 10
std::pop_heap(std::begin(numbers), std::end(numbers),std:: greater<>());
// Result: 2.5 6 3.5 6.5 8 12 10 1.5
numbers.pop_back();//Result: 2.5 6 3.5 6.5 8 12 10
```
为了防止意外的操作，SLT仍然提供了一个检查序列是否为堆的方法：   
```
if(std::is_heap(std::begin(numbers),std::end(numbers)))
    std::cout << "Great! We still have a heap.\n";
else
    std::cout << "oh bother! We messed up the heap.\n";
```
甚至提供了is_heap_until函数来检查是为堆的区间；   
最后一个stl提供的是堆排序sort_heap()(复杂度O(NlogN))，它要求元素段本身是堆，否则会崩溃；
```
std::vector<double> numbers {2.5, 10.0, 3.5, 6.5, 8.0, 12.0, 1.5, 6.0};
std::make_heap(std::begin(numbers), std::end(numbers));
//Result: 12 10 3.5 6.5 8 2.5 1.5 6
std::sort_heap(std::begin(numbers), std::end(numbers));
// Result: 1.5 2.5 3.5 6 6.5 8 10 12
```
[STL中sort是固定的快速排序吗？](https://blog.csdn.net/qq_35440678/article/details/80147601)
虽然algorithm使用了sort，但是sort_heap()本身使用了堆排序，利用堆的局部有序性让排序变得更快；   

78. 考虑下堆插入的时间复杂度的问题（？）

79. c++中^是异或操作符号

80. C++一概遵循的是左闭右开的原则（这一点可以从迭代器的特性中可以看见）

81. C++11引入了返回类型后置的写法，这样有机会返回一个固定大小的数组。同时提供decltype进行类型的自动推断；（也就是说使用返回类型后置的一个比较重要的场合是可以在模板类中进行返回类型的后置推断）  

82. Git+Qt最正确的编码处理方式：（文件类型）UTF-8带BOM +
 （项目设置）UTF-8 + 如果编码是UTF-8则添加BOM

83. Qt处理密集型耗时的问题不让主界面卡顿的方法：
1) 使用多线程；
2）若不想使用多线程，简单办法就是在文件保存过程中反复调用processEvent,结束后再把使用权返回给调用者（这是由事件循环特性决定的）；
Tip: 如果又希望使用者在过程中不再接受键盘和鼠标的响应，那么可以使用
```
qApp->processEvents(QEventLoop::ExcludeUserInputEvents);//它可以忽略用户的输入（鼠标和键盘事件）。
```     

84. 单例时经常遇到一个问题：提示instance存在链接错误，其实实际情况是类中静态变量需要在类外进行初始化，貌似private和public都是这种情况。（类外其实做的是给静态变量分配内存的工作[链接](https://blog.csdn.net/vict_wang/article/details/80994894 )）     

85. 多线程使用中，只可以在QObject（及其子类）被创建的线程进行其操作，不可以进行跨线程的QObject的添加和修改；
 
86. 二分法并非全部递归——二分法只是略过大部分内容一次，不需要完全遍历

87. 网上一个QML例子给出了较好的以鼠标为中心的缩放办法：
```   
ApplicationWindow {
    visible: true
    width: 640
    height: 480
    title: qsTr("自由缩放")

    // 定义缩放比例系数变量,范围在(-10,10)之间
    property double scaleValue: 1.1
    property int scaleLevel: 0

    function zoomIn(x,y){
        var beforeWidth  = showImg.width
        var beforeHeight = showImg.height
        showImg.width = showImg.width   * scaleValue
        showImg.height = showImg.height * scaleValue
        showImgMouseArea.width = showImg.width
        showImgMouseArea.height = showImg.height

        showImg.x = showImg.x + x - showImg.width  * x / beforeWidth
        showImg.y = showImg.y + y - showImg.height * y / beforeHeight
        scaleLevel++
    }

    function zoomOut(x,y){
        var beforeWidth  = showImg.width
        var beforeHeight = showImg.height
        showImg.width = showImg.width   / scaleValue
        showImg.height = showImg.height / scaleValue
        showImgMouseArea.width = showImg.width
        showImgMouseArea.height = showImg.height
        showImg.x = showImg.x + x - showImg.width  * x / beforeWidth
        showImg.y = showImg.y + y - showImg.height * y / beforeHeight
        scaleLevel--
    }

     Rectangle{
         id:showImg
         width: 100
         height: 100
         color: "lightblue"
     }

     MouseArea{
         id: showImgMouseArea
         anchors.fill: showImg
         //设置拖拽对象以及拖拽区域
         drag.target: showImg
         drag.axis: Drag.XAndYAxis//设置横向纵向拖动

         //设置鼠标悬停以及鼠标响应
         hoverEnabled: true

         // 鼠标滚轮处理函数
         onWheel: {
             if(wheel.angleDelta.y>0&&scaleLevel<=10){//图像放大处理
                 showImg.transformOriginPoint.x = wheel.x
                 showImg.transformOriginPoint.y = wheel.y
                 zoomIn(wheel.x,wheel.y)
             }
             else if(wheel.angleDelta.y<0&&scaleLevel>=-10){//图像缩小处理
                 showImg.transformOriginPoint.x = wheel.x
                 showImg.transformOriginPoint.y = wheel.y
                 zoomOut(wheel.x,wheel.y)
             }
         }
     }
}
```   

88. Java和JS中，双等于号等价于只比较值，不比较类型，三等于号等价于既比较值又比较类型   

89. 反斜杠(\，backslash)C++中既可以表示转义字符，又可以表示下一行和该行的内容连续（类似TrioBasic中的连续符号），常用于较长的宏的定义

90. \#号在预编译中的作用：   
\# 字符化操作   
\## 标记链接符号   

91. pointSize和pointSize: pixelSize是指所占的像素大小，在某些高DPI的设备上，字就会显得比较小
pointSize指的是实际的肉眼字体大小，与显示器无关   

92. QML数据和C++基本数据类型的关系：   
![QML基本数据类型表](https://static.oschina.net/uploads/img/201704/03152700_AyRk.png )   

93. QML中的QtObject是非可视化元素，适合于存储一系列设定的属性内容，可以形成和C++类似的结构体参数。

94. QML同样面临优化的问题：内部使用的大量的相同基础变量应该使用类似单例的编程办法，只在生成时导出一次

95. MVC在Qt中的基本用法：new一个QtStandardItemModel出来，然后往该模型中填充数据；然后绑定到QDataWidgetMapper上去，最后map到界面上的对应内容；

96. QML从C++向QML方向，必须设定C++部分暴露的内容为Q_PROPERTY部分，否则无法正确搞定；

97. QML端的Map信息如何传递：[CSDN](https://blog.csdn.net/fxy0325/article/details/85058662 )    
注意var本身要初始化为{},如果是property还需要对初始键值对类型进行初始化；   

98. QML和C++类型的互相转换：[QtDoc](https://doc.qt.io/qt-5/qtqml-cppintegration-data.html )

99. QML部分存在的属性请不要在QML中以后面带括号的形式调用，本身并不支持类似的操作，尤其是部分可读属性；（例如StackView）

100. 折腾的QML发布：总体来说标准的可以使用的QML发布过程比较麻烦，首先将QML放置于bin文件夹下，再复制bin、plugin、qml以及translation共计4个文件夹，再打开并尝试删除；最终得到的内容即可；

101. 应当广泛的使用正则表达式在网页信息抓取中，这是由html和xml格式决定的；

102. 需要重新审视下XML和JSON文件格式，学会正确的读取二者；

103. 编码问题：从很多api(例如zlib)传递回来的文件编码，是基于本地的GBK16位内码，必须先通过QString::fromLocal8Bit进行转化才可以得到正确的Unicode编码（Qt内部默认使用Unicode编码作为传输文件的基础）。同样的反而行之，如果需要在windows的api中进行操作，应该也要将Unicode编码转为GBK。    
简单来说区别就是：前者使用QString::fromLocal8Bit，后者使用QString::toLocal8Bit()方法。   

104. 常用C++宏定义：   
1) #error message: 类似Q_ASSERT()，不过只要执行了就会弹出报错；message内容不需要添加双引号包围；    
2) #line 用于强制指定新的行号和编译文件名，并且对源程序的代码进行重新编号，用法为#line number newFilename ；   
3) pragma 用于自定义编译消息，其中#pragma message("content...")类似正常的输出内容, #pragma once表示只编译一次，#pragma pack(int n)表示字节对齐方式。

105. 所以do {} while(0)的使用时为了保证宏定义的使用者能无编译错误的用宏。[csdn](https://blog.csdn.net/weibo1230123/article/details/81904498 )  （简单来说不需要加后缀） 

106. QtConcurrent采取了容器处理的方法，会自动分配核心对函数进行处理，这意味着是否非常合适进行类似utk图片输出的情况？（并发式）    

107. Qt事件循环：    
Qt是基于事件驱动的框架，事件和事件传递在其中非常重要。   
事件能够程序内部和外部产生，举个例子：   
- QKeyEvent和QMouseEvent对象代表了一个键盘和鼠标事件，它们从窗口由用户的操作而产生。   
- QTimerEvent对象是当某个事件被激发时投入，它们由操作系统产生。   
- QChildEvent对象是当一个子窗口被添加或者移除时被送入QObject的，它们的源头是Q他程序自己。   
事件的重点是它们产生的时候并不会被直接传递，而是会先进入事件队列，某时刻会被传递。传送者自己循环事件队列并且把事件传递给目标的QObject对象，因此被称作为事件循环。概念上说，时间循环就像这个：   
```
while (is_active)
{
    while (!event_queue_is_empty)
        dispatch_next_event();
    
    wait_for_more_events();
}
```   
我们通过运行QCoreApplication::exec()来进入消息循环，直到这个循环调用了exit()或者quit()被调用时才会被堵塞，然后退出。   
这个“wait_for_more_events()”函数处于堵塞的状态，直到有新的事件被产生。假如我们考虑它，所有在此刻可能产生的事件是外部源头的，那么这个消息循环在以下情况会被唤醒：   
- 窗口管理活动（鼠标按键操作等）；   
- 套接字事件；   
- 定时器事件；   
- 其他线程中投递的事件；    
在Unix-like系统中，窗口管理器通过套接字来通知应用程序，即使客户端使用它们来与x server通讯，如果我们决定用内部的套接字去实现跨线程的事件投递，只剩下以下唤醒条件：   
- 套接字；   
- 定时器；   
这个就是select(2)系统调用所做的，它监视着一系列活动者的描述符，如果它们在一定的时间内没有特定的活动，它最终就超时了。   
一个运行着的事件循环需要什么？    
这个不是完整的列表，但是如果有整体的画面，你将能够去猜测什么类需要一个运行着的事件循环。   
- Widgets的绘画和互动：QWidget::paintEvent()将在传递QPaintEvent对象时被调用，这个对象将会在调用QWidgets::update()或者窗口管理器的时候产生：响应的事件需要一个事件循环去进行分发。   
- Timers: 长话短说，当它们在select(2)或者超时的时候产生，因此它们需要让Qt在返回时间循环的时候进行这些调用。    
- Networking:所有底层Qt网络类（QTcpSocket、QUdpSocket、QTcpServer等）这些都是异步设计的，当你调用了ready()，它们只是返回已经可用的数据，当你调用write()，它们只是将这个操作放入队列，适时的时候会写入。只有当你返回消息循环的时候才是真实的读写速度，写入才会执行。注意它们的确提供了同步的方法，但是它们的用法是不被提倡的，因为它们会堵塞事件循环。QNetworkAccessManager，不再简单的提供一个API，而是需要一个事件循环。   
### 堵塞事件循环法     
在我们讨论为什么你从不堵塞消息循环之前，我们试着分析堵塞的含义。加入你有一个按钮，它会在被点击的时候发出clicked信号；在我们的对象中连接着一个槽函数，当你点击了那个按钮后，栈追踪会像这样：   
```
main(int, char**)
QApplication::exec()
[...]
QWidget::event(QEvnet*)
Button::mousePressEvent(QMousePressEvent*)
Button::clicked()
[...]
Worker::doWork()
```   
**在main函数中我们启动了时间循环，像往常一样调用了exec()。窗口管理器给我们发送了一个鼠标点击事件，它被Qt内核取走，转换为QMouseEvent并被我们送往widget的event()方法，该方法被QApplication::notify()发送。因为按钮没有重写event()，基类的方法将会被调用，QWidget::event()检测到这个事件的确是一个鼠标点击事件，然后调用特定的事件处理函数，那就是Button::mousePressEvent()，我们重写这个方法去发送clicked()信号，那就会调用被连接的槽函数。**     
当该对象处理量很大，那么消息循环在做什么？我们猜测它：什么都不会做！它分发鼠标按下事件，然后就堵塞着等待事件处理函数返。这个就是堵塞了时间循环，它意味着没有消息被分发了，知道我们从槽函数返回了，然后继续处理挂起的信息。   
在消息循环被卡住的情况下，widgets将不能进行自身的更新，不能有更多的互动,timers将不会被激发，网络通讯将会缓慢下来，或者停止。进一步的说，许多窗口管理器将会检测到你的应用程序不再处理事件了，然后告诉用户你的程序没有响应，这就是为什么快速的对事件响应并且即使返回到事件循环是多么的重要。   
### 强制事件分发   
所以，如果我们有一个很长的任务去运行但是又不希望这个消息循环被堵塞，应该如何处理？一个可能的方法是将这个任务移到另一个线程中。当然，我们也能手动强制事件循环去运行，这个方法是通过在堵塞的任务函数中去调用**QCoreApplication::processEvent()**来实现的，QCoreApplication::processEvent()将处理所有在消息队列中的消息并返回给调用者。    
另一个可选的选项是我们可以强制重入事件循环的对象，就是QEventLoop类，当通过调用QEventLoop::exec()我们将重入事件循环，然后我们就能将槽函数QEventLoop::quit()连接到信号上去使它退出。举个例子：   
```
QNetworkAccessManager qnam;
QNetworkReply *reply = qnam.get(QNetworkRequeset(QUrl(...));
QEventLoop loop;
QObject::connect(reply, SIGNAL(finished()), &loop, SLOT(quit()));
loop.exec();
//到这一步时，reply已经全部完成，使用即可   
```
QNetworkReply本身不提供堵塞的API，它要求一个在运行的事件循环，我们进入了一个本地的事件循环，当回复完成的手，这个本地的循环退出了。    
要特别小心的是其他路径下重入事件循环：它极可能导致出现不应该出现的递归。让我们回到前面看看按钮的例子。假如我们在槽函数中调用了QCoreApplication::processEvent(),当用户点击了这个按钮，这个槽函数将被再次调用：    
```
main(int,char**)
QApplication::exec()
[...]
QWidget::event(QEvent*)
Button::mousePressEvent(QMouseEvent*)
Button::clicked()   
[...]
Worker::doWork() //首先，内部调用   
QCoreApplication::processEvent() //手动分发事件   
[...]
QWidget::event(QEvent*) //此刻另一个鼠标点击事件进入了   
Button::mousePressEvent(QMouseEvent*)   
Button::clicked() //此时clicked()信号再次发送   
[...]
Worker::doWork() //GG!此时我们在槽函数中递归了   
```   
一个比较方便的方法是把QEventLoop::ExcludeUserInputEvent传递给QCoreApplication::processEvents(),这会告诉消息循环不要再次分发任何用户的输入事件。   
幸运的是，在检测事件中，这个相同的事件不会被发生。事实上，它们被Qt通过特殊的方法处理了，只有当运行的事件循环有了一个比deleteLater被调用后更小的“nesting”值才会被处理：   
```
QObject *object = new QObject;
object->deleteLater();
QDialgo dialog;
dialog.exec();   
```
将不会使得object成为一个悬空指针，相同的东西被应用到了本地的事件循环中。唯一的一个显著区别是，它在加入没有当前事件循环在运行的时候deleteLater被调用了的条件下，然后第一个消息循环进入后会取走这个事件，然后立即删除这个Object，这个是相当合理的，因为Qt不知道任何外部的循环会最终影响这个检测，因此马上删除了这个object。    
   
108. postEvent()和sendEvent()的源码分析：   
Qt文档中这样解释：   
sendEvent(QObject* receiver, QEvent *event)
使用notify()函数直接给receiver发送事件。   
postEvent(QObject* receiver, QEvent *event)
向事件队列中添加receiver和event。     
简单说，sendEvent是同步事件处理，postEvent是异步事件处理。   
### sendEvent代码分析   
```
inline bool QCoreApplication::sendEvent(QObject *receiver, QEvent *event)  
{  if (event) event->spont = false; return self ? self->notifyInternal(receiver, event) : false; }  
```   
直接调用notifyInternal接口，注意中间设置自发标志位为false，同时还需要判断self是否有效(QCoreApplication是否启动)。   
```
bool QCoreApplication::notifyInternal(QObject *receiver, QEvent *event)  
{  
    // Make it possible for Qt Jambi and QSA to hook into events even  
    // though QApplication is subclassed...  
    bool result = false;  
    void *cbdata[] = { receiver, event, &result };  
    if (QInternal::activateCallbacks(QInternal::EventNotifyCallback, cbdata)) {  
        return result;  
    }  
      
    // Qt enforces the rule that events can only be sent to objects in  
    // the current thread, so receiver->d_func()->threadData is  
    // equivalent to QThreadData::current(), just without the function  
    // call overhead.  
    QObjectPrivate *d = receiver->d_func();  
    QThreadData *threadData = d->threadData;  
    ++threadData->loopLevel;  
      
    bool returnValue = notify(receiver, event);  
    --threadData->loopLevel;  
     return returnValue;  
}  
```   
notifyInternal最重要的作用是activeCallbacks,直接看notify:   
```
bool QCoreApplication::notify(QObject *receiver, QEvent *event)
{  
    Q_D(QCoreApplication);  
    // no events are delivered after ~QCoreApplication() has started  
    if (QCoreApplicationPrivate::is_app_closing)  
        return true;  
    if (receiver == 0) {                        // serious error  
        qWarning("QCoreApplication::notify: Unexpected null receiver");  
        return true;  
    }  
    return receiver->isWidgetType() ? false : d->notify_helper(receiver, event);  
}  
```   
这个接口在Qt文档上有注释，注意其中当receiver为控件时，不进行处理。   
```
bool QCoreApplicationPrivate::notify_helper(QObject *receiver, QEvent * event)  
{  
    // send to all application event filters  
    if (sendThroughApplicationEventFilters(receiver, event))  
        return true;  
    // send to all receiver event filters  
    if (sendThroughObjectEventFilters(receiver, event))  
        return true;  
    // deliver the event  
    return receiver->event(event);  
}  
```    
在这里可以看到事件是如何被处理的：   
**
- 先送入Application事件过滤器，看看是否在事件过滤器中处理；   
- 再看看receiver是否有此事件的过滤器；   
- 最后，将事件送入receiver的event接口；   
**   
从整个过程来看，可以认为是sendEvent直接调用了receiver的event接口，可以认为处理方式是同步处理方式。   
### postEvent分析   
```
void QCoreApplication::postEvent(QObject *receiver, QEvent *event)  
{  
    postEvent(receiver, event, Qt::NormalEventPriority);  
}  
```   

```
void QCoreApplication::postEvent(QObject *receiver, QEvent *event, int priority)  
{  
    ...  
    QThreadData * volatile * pdata = &receiver->d_func()->threadData;  //得到线程信息  
    QThreadData *data = *pdata;  
    if (!data) {  
        // posting during destruction? just delete the event to prevent a leak  
        delete event;  
        return;  
    }  
    
    // lock the post event mutex  
    data->postEventList.mutex.lock();  
    
    // if object has moved to another thread, follow it  
    while (data != *pdata) {                 //在这里判断receiver线程信息是否发生变化。（有可能是另外一个线程调用用receiver->moveToThread）
        data->postEventList.mutex.unlock();  
    
        data = *pdata;  
        if (!data) {  
            // posting during destruction? just delete the event to prevent a leak  
            delete event;  
            return;  
        }  
    
        data->postEventList.mutex.lock();  
    }  
    //这里postEventList还是被锁着的。  
    // if this is one of the compressible events, do compression  
    if (receiver->d_func()->postedEvents  
        && self && self->compressEvent(event, receiver, &data->postEventList)) {  
        data->postEventList.mutex.unlock();//这个事件有可能被压缩（实际上是发现队列中有这个事件还没有被处理，且这个事件是可以被压缩的，例如paintevent）  
        return;  
    }  
    
    event->posted = true;  
    ++receiver->d_func()->postedEvents;  
    if (event->type() == QEvent::DeferredDelete && data == QThreadData::current()) {  
        // remember the current running eventloop for DeferredDelete  
        // events posted in the receiver's thread  
        event->d = reinterpret_cast<QEventPrivate *>(quintptr(data->loopLevel)); //receiver即将被析构？  
    }  
    //将事件添加到postEventList中，注意这里的优先级第一个最高，最后一个优先级最低  
    if (data->postEventList.isEmpty() || data->postEventList.last().priority >= priority) {  
        // optimization: we can simply append if the last event in  
        // the queue has higher or equal priority  
        data->postEventList.append(QPostEvent(receiver, event, priority));  
    } else {  
        // insert event in descending priority order, using upper  
        // bound for a given priority (to ensure proper ordering  
        // of events with the same priority)  
        QPostEventList::iterator begin = data->postEventList.begin()  
                                            + data->postEventList.insertionOffset,  
                                    end = data->postEventList.end();  
        QPostEventList::iterator at = qUpperBound(begin, end, priority);  
        data->postEventList.insert(at, QPostEvent(receiver, event, priority));  
    }  
    data->canWait = false;  
    data->postEventList.mutex.unlock();//在这里解除锁  
    //receiver所在的线程调用eventDispatcher处理postEventList  
    if (data->eventDispatcher)  
        data->eventDispatcher->wakeUp();  
}  
```   
从上面可以看到，postEvent实际上是将事件添加到receiver所在线程的一个队列中，至于整个队列所在的线程什么时候处理这个事件，postEvent是无法处理的。   

109. Qt提供了QSqlRelationTableModel来实现所谓的外键功能。类似你在表格属性中写入0514，它会自动给你转化为扬州。      
外键： 一个表中的属性和另一个表中主键属性的映射功能。   

110. JSON基本的语法规则：
1. 对象表示为键值对；    
2. 数据由逗号分割；    
3. 花括号保存对象；    
4. 方括号保存数组；

111. 编译有时候出现传递参数出现问题的情况（异常字符），可能跟是否使用C++运行时动态库、静态库(Runtime Library)有关（md和mt）
   
112. 左乘和右乘的区别应该如何处理？（待探究的问题）

113. 20190816修改：
1) 计算任意多边形最速公式并非简单的叉积求和，而是Clipper中给出的范例：
```
double Area(const Path &poly)
{
  int size = (int)poly.size();
  if (size < 3) return 0;

  double a = 0;
  for (int i = 0, j = size -1; i < size; ++i)
  {
    a += ((double)poly[j].X + poly[i].X) * ((double)poly[j].Y - poly[i].Y);
    j = i;
  }
  return -a * 0.5;
}
```
效率主要提升在少做了一次乘法
2) 能够以索引进行遍历的，不要轻易使用新结构进行运算；
3) 能够以旋转运算进行遍历的，不要轻易使用单位向量的加和减来进行运算；
4) 整数除以2可以使用右移1位的方法；     

114. 下面给一个实例中关于const变量和指针混合的判断：   
```
int i = 10;
int const * p = &i;   
int * const pp = &i;
const int *ppp = &i;
//下面三个皆为错误，揭示了其本质
*p = 0; 
pp = Q_NULLPTR;
*ppp = 0;
```
从这样看来，仍然遵循**从右至左**的原则，然后以**临近原则**，如果左侧是const，则变量本身是常量，如果左侧是*，则变量本身是指针，然后左侧按照统一处理，如果是int const/const int，则表示指向int常量，如果是int *，则表示为类型为整形指针。   
同理还有一个写法是``int const * const a``,这表示一个指向整形常量的常量整形指针。    

115. [C++中const用于函数重载](https://www.cnblogs.com/qingergege/p/7609533.html )给出了很好的例子，揭示了编译器针对const重载，通常只在以下两种情况可以通过：    
- 类函数的重载，通过对象是否为常量来区分；
- 普通函数的不同形参的重载，这里只能允许底层const来进行区分，如果是顶层const则视为重复    

116. 尽量在类中使用class前置声明，这样的话可以尽量避免编译时间变长并且打开时间也会变长。此外前置声明可以用于**指针和引用**。    

117. 类似宏定义改变变量本身的内容，可以使用函数指针来简化实际代码中的写法；    

118. 在QList、QMap等容器中，如果使用的非指针集合作为类的成员，需要注意提供其拷贝函数（A(const  A&)）的实例化解释，否则可能会报C2248的错误；    

119. _BLOCK_TYPE_IS_VALID 错误经常在**堆区内容被错误释放的情况下报出**，应该检查是否有自我释放的错误空间。[链接](https://blog.csdn.net/zjdnwpu/article/details/70245500)     

120. QPointer的作用是：当QPointer指针引用的对象被销毁时候，p2指针会自动指向NULL，而p1的引用对象如果被销毁，p1则不会自动指向NULL，而是会变成野指针。(用途：避免多次访问时出现问题，此外注意QPointer只能用于QObject的子类)[链接](https://blog.csdn.net/xiezhongyuan07/article/details/80263614)    

121. Qt中QImage部分功能无法正常使用时，需要看下是否使用的QApplication进行的实例化（而非QCoreApplication），如果不是的话可能会出错。   

131. qml有一个陷阱在于，如果函数中间有语句存在执行问题，它虽然会报错，但是并不会中断或者停止，而会结束该函数剩下语句的执行，转而执行剩余未执行完的其他函数。    

132. QMake中的INCLUDEPATH指的是包含路径，而DEPENDENPATH指的是依赖库所在目录；（问题来了，不都能通过LIB直接指定库了吗？为什么还需要DEPENDENPATH？）

133. QtPlugin系统，动态插件只有dll即可动态加载，可在运行时加载，静态插件提前在QMake和main函数中调用（之后使用QtPluginLoader::staticInstance调用出来）    

134. QtPlugin的Plug&Paint例子里面有个很精彩的片段：    
```
void MainWindow::populateMenus(QObject *plugin)
{
    BrushInterface *iBrush = qobject_cast<BrushInterface *>(plugin);
    if (iBrush)
        addToMenu(plugin, iBrush->brushes(), brushMenu, SLOT(changeBrush()),
                  brushActionGroup);

    ShapeInterface *iShape = qobject_cast<ShapeInterface *>(plugin);
    if (iShape)
        addToMenu(plugin, iShape->shapes(), shapesMenu, SLOT(insertShape()));

    FilterInterface *iFilter = qobject_cast<FilterInterface *>(plugin);
    if (iFilter)
        addToMenu(plugin, iFilter->filters(), filterMenu, SLOT(applyFilter()));
}
//! [10]

void MainWindow::addToMenu(QObject *plugin, const QStringList &texts,
                           QMenu *menu, const char *member,
                           QActionGroup *actionGroup)
{
    foreach (QString text, texts) {
        QAction *action = new QAction(text, plugin);
        connect(action, SIGNAL(triggered()), this, member);
        menu->addAction(action);

        if (actionGroup) {
            action->setCheckable(true);
            actionGroup->addAction(action);
        }
    }
}
```    
其中SLOT被解析为字符串，揭露了connect函数中的本质，这也是告诉使用者，可以自由的使用类似字符串的方式去传递槽函数的内容来进行操作，此外应该也可以通过QMetaType（QObject）来获取一个类从类名到函数名的全部信息，贼猛。   

135. 多态的变态之处：接上，观察调用QAction的例子：   
```
void MainWindow::changeBrush()
{
    QAction *action = qobject_cast<QAction *>(sender());
    BrushInterface *iBrush = qobject_cast<BrushInterface *>(action->parent());
    const QString brush = action->text();

    paintArea->setBrush(iBrush, brush);
}
```
这里硬生生从QAction中解析出一个QObject，直接转为Brush的接口拿到数据，将封装、继承和多态发挥的淋漓尽致。   
这种写法非常值得前后端均进行借鉴，一个QObject自由的跨于QAction和自定义的Interface之间，真的方便。     
此外，这种给UI件赋予QObject，然后不用寻找哪个槽函数，只需从sender() 获取返回值的方法非常值得推广，省得每一个index的老方式来进行寻找。      

136. QPluginLoader加载的插件的初始化过程发生于QPluginLoader的加载时期；

137. 再次重复：虚函数的作用并非被重写（非虚函数也可以被重写），虚函数最重要的是能够实现**多态**，即动态执行时，父指针可以实现子对象的重载方法；    

138. 使用QImage进行绘制的时候，需要考虑在比较密集地方**一定记得end掉QPainter**，否则在绘制某些较大图片或者复杂场景处，容易出现创建QImage失败的情况；   

139. 重复定义头文件中的函数的报错解释：**头文件中出现重复定义，简单的方法是使用inline加在之前，更好的方法是囊括到cpp文件中实现，或者在类中实现（隐式内联）**。[解释](https://blog.csdn.net/qq319923400/article/details/80889064 )
    
140. 智能指针一个比较合适使用的场合：一个函数返回智能指针类型，该类型的内存不需要在新的区域指明去进行删除，而是由自身来进行处理。    

141. 



