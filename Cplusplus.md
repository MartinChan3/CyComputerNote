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

