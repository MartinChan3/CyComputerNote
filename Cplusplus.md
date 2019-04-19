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
- OUT_PWD 指的是shadow-build文件夹

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

40. 在枚举类型不定义枚举类型的名称时，其等价于define具体的值，好处在于自动补全赋值，而不用一口气连续定义几个值；

41. 日后需要完善库类文件的定义问题：
首先明确一点：**需要导出的结构体、类及方法需要在头部添加好对应的dll生成的宏，否则生成的dll及lib不会提供对应内容的入口，编译时会报对应的函数部分没有实现。** 类不需要对内部的函数结构再进行宏标示。

42. 枚举类型输出时只会输出其实际值，如果要输出其枚举值，只能使用类似“映射”的方法，写一个枚举类型的返回其名的字符串函数；

43. 比较语句中的倒序写法的好处，是防止写成等于号，例如```3 == a```可以避免```a = 3```这种少写等于号的错误的情况；

44. Qt中QString存在一个很明显的问题：当在qDebug中输出时，仍然使用双反斜杠来替代实际内存中的单斜杠；（这个斜杠也影响到了QSettings中的输出问题）

45. QString::number中的precision指的是**小数位数**而非数学概念中的有效精度；

46. C++中，应当在最基础的结构体或者类型中，只存储最简单的数据结构（例如点集数据），由其本身可以推断出的数据内容，应当作为函数进行推断，这样可以以最简洁的方式传递结构体本身的数据内容，而不影响数据的使用；也可以提高代码的**可重用性**；

47. 类当中的const函数是好事，但是**const函数无法调用类中非const函数**，这意味着需要非常好的框定好函数本身内容的权限，编译器会报错；

48. C++中首字母不可以是数字，否则编译器会报很诡异的错误；

49. 在Qt乃至std库中目前还没有发现文本读入或者读出模块有“插入字节”的功能，所以一旦涉及到内容的修改，似乎必须会需要重新创造一个文件来代替原来的文件；

50. QFile提供了函数**peek**来进行尝试性的读取，类似于先读取再跳转回去（即read和skipRawData(-num)的组合）；

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
1） 重载函数是否可以返回类型不同？（可以，只需要返回类型）   
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
1. 简单来看，只能只恨使用了RAII技术(资源获取即初始化)的技术对普通的指针进行封装，这使得智能指针实质是一个对象，而行为表现类似指针；   
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
- **注意避免循环引用**：shared_ptr的最大一个陷阱在于玄幻引用，会造成堆栈内存无法正确释放，最终导致内存泄漏。循环引用会在weak_ptr中介绍；   
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
静态联编和动态联编：当使用虚函数的多态，就是所谓的动态联编，因为编码器并不提前知道调用相同方法的对象产生的结果是哪个产生的。而普通的函数重载是静态联编的典型形式。   
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
特点1： 通过虚函数列表指针VPTR调用重写函数是在程序运行的时候进行的，因此需要进行寻址操作，这一点决定了没有静态联编的效率高（虚函数）；
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
   
65. C++11新特性总结：   

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

70. 关于多线程和mutex:本质上需要了解生产者和消费者模型的价值；这里需要具体写一章对应的内容；      

71. C++11中存在谓词系统（Predicate），用来在类似count_if函数中进行判断返回的条件。   

72. C++11后有了标准的std::thread，其用法值得研究

73. [仿函数](https://www.cnblogs.com/yyxt/p/4986981.html )
：STL提供的各种算法往往有两个版本，其中一个版本表现出最直观的某种运算，另一个版本则会表现出最泛华的演算流程，允许用户通过“以template为参数来指定所要采取的策略”。这个类似于sort()函数，其中第一版本以operator<为排序依据，第二版本则允许任何操作。   
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
其中第一种用法比较为大家所熟悉，greater<int>ig是产生一个名为ig的对象，ig(4,6)则是调用其operator()，并且给予两个参数4,6。第二个用法是一个零时的对象，之后的6和4才是指定的参数。
上述第二中语法一般不常见，但是在STL应用当中，却是主流用法。（STL仿函数绝大部分采用这种语法）   
STL仿函数的分类，若以操作个数划分，可以分为一元仿函数，二元仿函数；若以功能划分，则可分为算数运算、关系运算、逻辑运算三大类。任何应用程序如果想要使用STL内建仿函数，需要引入<functional>头文件。   
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

80. C++一概遵循的是左闭右开的原则

81. C++11引入了返回类型后置的写法，这样有机会返回一个固定大小的数组。同时提供decltype进行类型的自动推断；（也就是说使用返回类型后置的一个比较重要的场合是可以在模板类中进行返回类型的后置推断）  

82. Git+Qt最正确的编码处理方式：（文件类型）UTF-8带BOM +
 （项目设置）UTF-8 + 如果编码是UTF-8则添加BOM

83. Qt处理密集型耗时的问题不让主界面卡顿的方法：
1) 使用多线程；
2）若不想使用多线程，简单办法就是在文件保存过程中反复调用processEvent,结束后再把使用权返回给调用者；
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

89. 反斜杠(\，backslash)C++中既可以表示转义字符，又可以表示下一行和该行的内容连续（类似TrioBasic中的连续）

90. 井号在预编译中的作用：   
# 字符化操作   
## 标记链接符号   

91. pointSize和pointSize: pixelSize是指所占的像素大小，在某些高DPI的设备上，字就会显得比较小
pointSize指的是实际的肉眼字体大小，与显示器无关   

92. QML数据和C++基本数据类型的关系：   
![QML基本数据类型表](https://static.oschina.net/uploads/img/201704/03152700_AyRk.png )   

93. QML中的QtObject是非可视化元素，适合于存储一系列设定的属性内容，可以形成和C++类似的结构体参数。

94. QML同样面临优化的问题：内部使用的大量的相同基础变量应该使用类似单例的编程办法，只在生成时导出一次

95. MVC在Qt中的基本用法：new一个QtStandardItemModel出来，然后往该模型中填充数据；然后绑定到QDataWidgetMapper上去，最后map到界面上的对应内容；