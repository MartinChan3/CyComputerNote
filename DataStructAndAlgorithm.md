#《C算法(第1卷)-基础、数据结构、排序和搜索》

## 第1章 引言
示例：连通性问题
实例化对应案例：1）迷宫；2） PCB板；3）编译器当中的引用；
第一章中主要讲解了“连通性”问题，并对此进行了详细的阐述；

## 第2章 算法统计原理
算法在实际编写过程中，往往会考虑算法性能。此外，实际的系统应当采用一种更具有防错性(defensive programming style)的方式。必须检查和报告出错条件，使得实现的程序可以得到更容易的修改，与系统的其他部分接口完善，并且能够被移植。

## 第3章 数据结构
书中给出了合理的程序组成结构
- 接口(interface)。用来定义数据结构，并且声明来操纵该数据结构的函数；
- 实现(implementation)。给出接口中的函数的实现；
- 客户端程序(client program)。使用接口声明中的函数在较高的抽象层次工作；

接口的设计方法有两种：
1. 将对象的实现隐藏在指针背后，简单来说就是将其分成两个类，一个类只提供接口，另一个负责实现改接口，这种手法成为Pimpl Idiom(Pointer to Implementation)。
2. 将接口定义为抽象类，接口全部被定义为虚函数（纯虚函数无接口体），派生类的成员函数负责实现该接口内容。这种手法被称为Object Interface。
（注意，第二种方法中需要将抽象接口类的析构函数定义为virtual函数，否则会发生函数泄漏）。
[例子链接](https://blog.csdn.net/TAOKONG1017/article/details/79561856)

从下个版本开始，尝试通过接口操作整个点集操作，降低用户直接操作数据的可能性；并且尝试使用分模块编译的方法；

《数据结构与算法分析C++版》 Mark Allen Weiss
## 第一章

1. explicit是用来解决编译器对单变量初始化列表防止隐式转换而存在的；
2. 模板类：是指类似在类内定义多个模板变量或者模板函数的类型，其写法类似模板函数。
```
tmplate <typename Object>
class MemoryCell
{
	public:
		explicit MemoryCell( const Object& initialValue = Object())
		: storedValue( initialValue){}
	const Object & read() const { return storedValue;}
    void write ( const Object & x)
	{ storedValue = x; }
    private:
	Object storedValue;
}
```
使用MemoryCell<int>和MemoryCell<string>是真类，但MemeoryCell本身不是；

在标准c++的ostream中就存在类似qDebug使用的重载形式，说明这种重载形式是通用的写法；

面对实际当中写模板中部分成员变量的比较，书中给出了例子，即重写findMax函数,**使得其可以像接受参数一样接受对象数组和比较函数，并由比较函数来决定哪一个更大或者更小**。就意味着从实际上，数组对象不再知道如何进行相互比较，取而代之，这些信息会完全从数组对象中剥离出来；
为了传递函数，这里采取了一个相对巧妙的方法：定义一个只有一个成员函数的类，然后传递这个类的实例；这个对象往往称为函数对象(function object).为了保证所有的模板正确的运行，则所有的模板都要有``isLessThan``的成员；
```
template < typename Object,  typename Comparator>
const Object & findMax( const vector<Object> &arr, Comparator cmp)
{
	int maxIndex = 0;
	for ( int i = 1; i < arr.size(); i++)
		if ( cmp.isLessThan( arr[ maxIndex], arr[ i]))
			maxIndex = i;
			
		return arr[ maxIndex];	
}

class CaseIntensitiveCompare
{
	public:
		bool isLessThan( const string & lhs, const string &rhs)
		{
			return stricmp( lhs.c_str(), rhs.c_str()) < 0;
		}
};

int main()
{
	vector< string> arr(3);
	arr[0] = "ZEBRA"; arr[1] = "alligator"; arr[2] = "crocodile";
	cout << findMax( arr, CaseInsensitiveCompare()) << endl;
	
	return 0;
}
```
其后可以改写为更贴近函数调用的operator()的形式来传入比较函数，并且通过findMax的重载来实现拓展；

**tip:模板类普遍存在分离编译问题**
即模板类不支持编译器于h和cpp文件进行分开，不然会在分离编译时报错。解决方法为要么老老实实放在.h头文件中（但这样会暴露模板类自身的代码内容），要么引入EXPORT宏（目前大部分编译器不支持），要么使用显式实例化；

自定义matrix类型：
```
#ifndef MATRIX_H
#define MATRIX_H

#include <vector>
using namespace std;

template < typename Object>
class Matrix
{
public:
	matrix( int rows, int cols) : array( rows)
	{
		for ( int i = 0; i < rows; i++)
			array[ i].resize( cols);
	}
	
	const vector<Object> & opeartor[] ( int row) const
	{
		return array[ row];
	}
	vector< Object> & operator []( int row)
	{
		return array[ row];
	}
	
	int numrows() const { return array.size();}
	int numcols() const { return numrows() ? array[0].size() : 0;}
private:
	vector< vector<Object>> array;
}

#endif

```

Attention:通过以上的内容可以看到，模板类的写法非常有意思，是通过定义实体的时候类似vector的写法来指定模板类的类型来决定模板类使用的内容；

## 第二章
1. 又一次给出了O的概念，主要是要建立两个函数之间的**相对增长率**的关系；其中“大O计法”指明了函数增长的一个相对上界，小o则是进一步规定了增长率不可能相等；同样的，下界的定义是由Ω来定义的；
一如既往，需要记住常用的几个量级的排序规律；通常使用极限法搭配洛必达法则来进行两个量级之间相对的增长率判断；

2. 问题：最大子序列和问题给出了四种结果；从结果来看，指明了直接使用大O来规定算法程序的上界；

3. 简单例子，三次方求和：
从该例子可以看到传统的法则规定：
> for循环 至多是循环次数（一般为N） * 循环体时间
> 嵌套循环 两层循环相乘
> 顺序语句 求和
> 判断语句 取判断中最长者
> 递归语句 1）简单的类似循环语句的等同于for循环
2）自身调用的则相对复杂

Attention:书中反复强调了菲波那切数列的递归算法是需要改正的，否则其运行时间为指数级别；接下来来分析最大子序列和问题
分冶：将一个大问题分解为两个小问题（分），再进行合并（冶）。
例如该问题可以推断最终结果只有三个情况：
- 落在前半部分
- 落在后半部分
- 落在前半部分的末端和后半部分的和
给出的最优算法，是线性结果的，这要求对于问题本身有着较为完备的分析；

## 表、栈和队列
1. 概念：抽象数据类型（ADT,Abstract Data Type）
2. C++中引入了标准模板库Standard Template Library,即STL。其中list总是针对双向链表，而表示针对更一般的ADT表。
3. 存在迭代器概念，用于迭代使用；例如QVector<int>::iterator
4. 通过vector和list的算法例子给出了其区别：vector更适合索引，但插入、删除方法较慢（因为存储结构连续）；list则相反（存储结构不连续）；
5. 定常性：例如begin()函数在模板类当中存在以下两个实现；
- iterator begin();
- const_iterator begin() const;
6. typename关键字也可以出现在函数体内（模板类的定义中）来指定后面跟随的量是指模板定义本身而不是变量名（进而来避免编译器的重名错误）
7. C/C++中，数组指针的++操作就意味着指向该数组下一个内容；
8. C++支持嵌套类，即在类中定义新类类型；(值得注意的是，类中定义的类貌似只能由“母类”进行访问，并不能由外部封装直接访问)
9. 讨论List的实现：
1) List本身，包含连接到表两端的链接、大小及一系列方法；
2) Node类，该类看上去像是私有的嵌套类。一个节点包含数据和用来指向其前和其后的节点的指针，以及其适当的构造函数；
3) const_iterator类。该类抽象了位置概念，是一个公有的嵌套类。const_iterator存储指向了当前节点的指针，并且提供了基本迭代器的一系列使用，以及所有的重载操作符（=、==、!=、++）
4) iterator类。该类抽象了位置概念，是一个公有的嵌套类。除了operator*操作返回所指向项的引用，而不是该项的常用引用的功能外，iterator具有和const_iterator相同的功能；
因为迭代器存储指向“当前节点”的指针，并且尾部标志是一个有效位置，这样使得在表的末尾添加一个额外的节点来作为尾部标志成为了可能。进一步的，也可以在表的前端生成一个额外节点，从而作为逻辑上开始的标志。这些额外的节点有时候称作为“哨兵节点”，头部节点称为**表头结点(header node)**，末尾的节点称为**尾结点(tail node)**；
9. 运算符重载前加和后加的区分方式
```
C++编辑器可以通过在运算符函数参数表中是否插入关键字int来区分这两种方式
声明：
X operator++();//前缀方式
X operator++(int);//后缀方式
调用时，参数int一般传递给值0
X ob;
++ob;
ob++;//隐式调用ob.operator++(int)
ob.operator++();
ob.operator++(0);//显式调用ob.operator++(int)，意为ob++
```
10. C系列语言都支持连续赋值a = b = c
11. 异常捕获使用语句try/throw/catch
[msdn教程](https://msdn.microsoft.com/zh-cn/library/wfa0edys.aspx)
12. 栈ADT
栈是限制插入和删除操作只能在一个位置上进行的表，该位置为表的末端，成为**栈顶（top）**；也称作为LIFO表（后进先出，Last In First Out），它基本上只有pop和push两种操作，而且栈顶元素为唯一可见元素；可通过链表和数组两个方式实现；

接下来介绍3个例子来反映栈的应用：
1. 平衡符号：IDE中检查括号是否配对；
> 做一个空栈。读入字符直至文件尾部。如果字符是一个开放符号，则将其压入栈中。如果字符是一个封闭符号，若栈为空，则报错；若不为空，则将栈元素弹出。如果弹出的符号不是对应的开放符号，则报错。在文件尾，若栈非空则报错；
2. 后缀表达式:基于逆波兰记法的科学计算器,其优点在于复杂度为O(N),而且无需知道任何转换优先规则；
3. 中缀到后缀转换：中缀类似于标准表达式
4. 函数调用：在进行新的函数调用时，需要先“保存现场”，来在新函数运行完之后知道向哪里转移；使用一个栈保存所有的活动记录（activation record），或者称作为栈帧，放在堆(pile)顶。
在实际的操作系统中，栈往往是从内存分区的高端向下增长；风险常常在于用尽栈空间（例如递归当中由于递归次数过多，导致忘记了基准情形）
该种情形在“尾递归”中常见，是极差的递归使用方法；

13. 队列ADT（queue）
区别于栈，队列使用的时候是**一端进入，另一端输出**
基本操作：
- enqueue(入队)：在表的末端插入一个元素；
- dequeue(出队)：删除表开头的元素；

相较于之前的结构，queue使用了循环数组(circular array)的概念。即在分配一定空间的情况下，不主动对空间大小进行操作，而是对开头和末尾的标志位进行操作；

队列的应用：
- 打印机队列；
- 买票系统；
- 文件服务器(file server)同样遵循先到先使用的规则；
- 接线员传呼系统；
- 大学资源调度系统；
对应于“排队论”，队列结构在诸多系统中有着极其重要的应用；

## 第4章 树
本章讨论一种大部分操作的访问时间为O(logN)的数据结构——树。这种数据结构称为二叉寻找树(binary search tree)。在很多程序中，它是set和map的实现基础，其实用性极强；
树可以用多种方式定义，比较简单的一种是通过递归：
1. 一棵树是一些节点的集合；该集合可以是空集，若非空集，则由称作根(root)的结点r以及零个或者多个非空(子)树T1、T2……Tk组成；
2. 每个子树中每一棵的根都被来自根r的一条有向的**边(edge)**所连接。
3. 每一棵子树的根叫做根r的儿子(child)，而r是每一棵子树的父亲(parent)。

易推得一棵树是N个结点和N-1条边的集合；
没有儿子的结点称之为**叶(leaf)结点**，具有相同父节点的结点称为**兄弟(siblings)结点**;类似的关系还有**祖父(grandparent)**和**孙子(grandchild)**。
从节点n1到nk的路径(path)被定义为结点n1、n2……nk的一个序列（其中ni是n(i+1)的父亲）。路径的长为路径上边的条数，即k-1。每个节点到自己都有一条长为0的路径。
对于任意节点ni，ni的**深度(depth)**是指从根到ni的唯一路径的长，所以根的深度为0。ni的**高(height)**是指ni到一片树叶的最长路径的长，因此所有树叶的高都为0，而一棵树的高等于它根的高。
易推得：一棵树的深度等于其最深的树叶深度，而该深度总等于该树的高。
如果存在n1到n2的一条路径，那么称为n1是n2的一位**祖先(ancestor)**，而n2是n1的一个后裔(descendant)。如果n1≠n2的话，则可称为**真祖先(proper ancenstor)**和**真后裔(proper descendant)**。

1. 树的定义和应用
1） UNIX文件系统：
例如列出分级文件系统中的伪代码
```
void FileSystem::listAll( int depth = 0) const
{
	printName( depth);  //Print name of object
    if ( isDirectory())
	{
		for each file c in this directory(for each child)
		c.listAll( depth + 1);
	}
}
```
以上的遍历策略称之为前序遍历(preorder traversal)。在前序遍历中，对结点的处理工作都是在它的诸儿子结点被处理之前进行的。每个节点都执行一次，运行时间为O(N),即常数时间；
另一种方法是后序遍历(postorder traversal)。在后序遍历中，在一个结点的工作是它的诸儿子节点被计算后进行的。例如计算磁盘占用的所有体积的算法：
```
int FileSystem::size() const
{
	int totalSize = sizeOfThisFile();
	
	if ( isDirectory())
	{
		for each file c in this directory( for each child)
			totalSize += c.size();
		return totalSize;	
	}
}
```
(Tip:看上去算法是前序的，但是由于该递归本身是从最底层开始写起，所以其实是从低层开始计算器求和的。前序和后序主要取决于前后序执行核心语句的顺序是从根开始还是从子结点开始，例如打印操作、计算size操作)

2. 二叉树
二叉树(binary tree)是指一棵每个节点都不能多于两个儿子的树。其中两个儿子可以为空。
二叉树的一个特点是平均二叉树的深度要比节点个数N小得多，这个性质相当重要，该平均深度为O(N^0.5),对于特殊类型的二叉树，即二叉查找树，其深度平均值为O(logN)。最坏情况的二叉树深度可以到N-1。
实现：因为一个二叉树只有两个儿子，所以可以直接把他们标出来。其形式类似双向链表的声明：
```
struct BinaryNode
{
	Object     element;
	BinaryNode *left;
	BinaryNode *right;
};
```
树一般画为一些直线连起来的圆圈，这是因为本质上二叉树就是图(graph)。当涉及到树的时候，我们也不会显式的画出NULL链，因为具有N个结点的每一棵二叉树都需要N+1个NULL链。
二叉树有着许多和搜索无关的重要应用，例如在编译器当中。

Example: 
1. 表达式树(Expression Tree)
对于一个表达式树，其树叶为操作数（常数或者变量名），其它结点为操作符。限定其所有操作均为2元的，则正好为二叉树。
中序遍历(inorder traversal)：可以递归的产生一个带括号的左表达式，然后在打印出在根出的操作符，然后再递归地产生一个带括号的右表达式从而得到一个(对两个括号内内容进行整体运算的)**中缀表达式(inorder traversal)**。
另一个遍历策略为递归的打印出左子树、右子树，然后再打印操作符，这类似于之提到的后缀表示法（逆波兰表示法）,而这种遍历策略一般称为后序遍历(postorder traversal)。
同时还有一种方法为先打印出操作符，然后递归地打印出左子树和右子树，则为较为少见的前缀（prefix）计法，对应的记为前序遍历(preorder traversal)。
