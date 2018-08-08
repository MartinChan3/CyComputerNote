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