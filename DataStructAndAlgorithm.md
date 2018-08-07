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
构建表达式树：
> 类似前文中的方法，依次一个符号读入表达式。
> 该符号为操作数，则建立一个单节点树并且将它压入栈中。如果符号为操作符，那么就从栈中弹出两棵树T1和T2(T1先弹出)并且形成一个新的树，该树根就是其操作符。它的左右儿子就是T2和T1，然后将指向该新树的指针压入栈中。
> 书中给出的例子（后缀表达）：
> a b + c d e + * *

3. 查找树ADT
二叉树的一个重要应用在于查找当中。
**Attention:** 使得二叉树成为二叉查找树的性质是，对于树中每个节点X，它的左子树的所有项的值小于X中的项，而右子树中的所有项的值大于X中的项。（注意，是整体符合啊，而不是局部，从最顶层结点开始检查起来）
接下来会给出对二叉查找树进行的操作的简要描述。值得注意的是：1）这些行为通常是通过递归的方式编写；2）因为二叉查找树的平均深度为O(logN)，所以我们一般不必担心栈空间用尽的问题。
给定的例子中，首先需要注意Comparable模板需要提前定义“<”关系。类似先前第一章中对复杂结构体的比较。
在例子结构中，数据成员是指向树根节点的指针，该指针对空树来说为NULL。public类的成员会使用盗用private递归函数的常规技术；
这里的小技巧：几个private成员函数使用了引址调用来传递指针变量的技术，这允许public成员函数将指向树根的指针传递给private递归成员函数。然后递归函数就可以改变根的值，所以root就可以指向其他的结点。
从findMin和findMax可以看到二叉树的好处：一直找左儿子和一直找右儿子即可；值得注意的是要小心处理空树的**退化**，还有注意在函数中是否使用的是指针的副本而非结点本身。（这个问题可以使用const修饰符来进行成员函数后缀修饰）
insert操作：将X插入到树T中，可以类似contain一样寻找。如果找到X，则不作为（即已存在相同元素）；若找不到，则插入到遍历路径的最后一个点上；
Tip：这里涉及到重复元的插入，一般通过在节点记录中保留一个附加字段来指示该数据元出现的频率，虽然该举措增加了空间占用，但是比将重复信息放入树中好（确保树的深度不是很大）。(如果比较运算<只是更大结构的一部分，该方法是行不通的。此时可以吧具有相同键的所有结构保留在一个辅助数据结构中，例如是表或者另一棵查找树)

Tip：C++不支持指针的引用，但是支持引用的指针，例如Node* &t;

remove：诸多数据结构中最难的操作是删除，需要考虑几种情况。
> 1. 如果结点是树叶，则可以被立即删除；
> 2. 如果结点有一个儿子，则可以绕过该结点后删除；
> 3. 如果结点为有两个儿子。一般策略为用其右子树的最小数据代替该节点数据并且递归的删除该结点。

懒惰删除：若删除的次数很少，通常使用懒惰删除准则，让它依然留在树中，只是做一个被删除的记号；该方法在有重复项时候相当有效，因为仅仅是记录频次数减1。如果树当中的实际结点数和“被删除”的结点数相同，那么深度预计只会上升一个很小的常数；因此，只存在一个和懒惰删除相关非常小的时间的损耗。更关键的是，如果删除的项又被重新插入，那么分配一个新单元的开销就可以避免了；

析构函数与复制赋值操作符：析构函数调用makeEmpty。复制操作采取和以往程序一样，使用makeEmpty来回收内存，然后对rhs使用clone递归复制内容；

证明：若所有的插入序列都是等可能的，那么，**树的所有结点的平均深度为O(logN)。**
一棵树所有的结点深度和称为**内部路径长(internal path length)**。下面计算二叉查找树的平均内部路径长。
令D(N)是具有N个结点的某棵树的内部路径长，D(1)=0。一棵N结点树是由一棵i结点左子树和一棵(N-i-1)结点右子树以及深度为0的一个根节点组成，其中0≤i<N,D(i)为根的左子树的内部路径长。但是在原树中，所有这些结点要加深一层。同样结论也适用于右子树。因此我们得到递推关系：
D(N) = D(i) + D(N - i - 1) + N - 1
( N - 1 = N - i - 1 + i)

<img src="http://chart.googleapis.com/chart?cht=tx&chl= \[D(N)=\frac{2}{N}[\sum_{j=0}^{N-1}D(j))]+N-1\]
" style="border:none;">

可以计算出D(N)=O(NlogN),则平均结点深度为O(logN)

“平均”：其大部分计算都是基于假设，而其合理性本身有待考察；但是其结果却又有一定参考价值；

如果向一棵预先排序好的树输入数据，那么一连串的insert操作将会花费二次的时间，而链表实现的代价会非常巨大，因为此时的树将只由没有左儿子的结点组成。一种解决办法就是引入平衡(balance)的附加条件，同时任何结点的深度均不得过深；有许多算法来实现平衡树，但是大部分算法都快比标准二叉树复杂得多= =，而且就更新而言还会花费更长的时间（虽然它们的确可以防止处理起来非常麻烦的一些简单情形）。下面介绍最古老的一种平衡查找树，即AVL树；
新的方法允许放弃平衡条件，允许树有着任意深度，但是每次操作之后会进行调整，使得之后的操作有着更高的效率。一般这样的操作称为自调整，它可以保证连续M次操作在最坏的情形下花费时间O(MlogN)，这就可以防止最坏的情形了。后面书中会讨论一个极其复杂的结构，伸展树(splay tree);

4. AVL树(Adelson-Velskii and Landis Tree)
AVL树是带有平衡条件的二叉查找树；这个平衡条件必须要容易保持，并且必须保持树的深度为O(logN)。最简单的方法是要求左右子树具有相同的高度。值得注意的是，该方法倒并不要求树的深度要浅；（并且仅在跟结点的平衡是远远不够的）
另一种平衡条件是要求每个节点都必须有相同高度的左子树和右子树。如果空子树的高度定义为-1，那么只有具有2^k-1个结点的理想平衡树满足这个条件。所以虽然该条件保证了树的深度小，但是太过严格，需要放宽；
定义：AVL树是指每个结点的左子树和右子树高度最多差1的二叉查找树（空树的高度定义为-1）。所以大约能预估一棵AVL树的高度最多为1.44log(N+2)-1.328。实际高度也就只比logN略高一些；
在AVL树的操作中，除去插入操作，所有树的操作都可以以O(logN)执行。插入操作之所以复杂的原因是有可能威胁到AVL的特性，这样需要引入新的运算来使整体恢复平衡——旋转(rotation);
接下来介绍如何在引入新的节点后，如何重新平衡这棵树：
> 把必须重新平衡的点称为α。由于任意结点至多有两个儿子，因此高度不平衡时，该点的两棵子树高度差为2；它存在以下4种情况
> 1. 对α的左儿子的左子树进行一次插入；
> 2. 对α的左儿子的右子树进行一次插入；
> 3. 对α的右儿子的左子树进行一次插入；
> 4. 对α的右儿子的右子树进行一次插入；

> 情形1和4关于α点对称，2和3也是。因此可以只分为两种情况讨论；
> 第一种——外外边插入，利用一次**单旋转**
> 第二种——内外边插入，利用一次**双旋转**
> 单旋转：局部简单替换原则，不破坏原有高度信息；本质上为改变两个父节点之间的关系；
> 双旋转：针对单旋转无法替代深度较深的情况；分右-左和左-右两种情况；

总结：将一个新节点X插入到一棵AVL树T中去，我们将会递归的将X插入到T的相应子树(称为TLR)中。如果TLR高度不变，那么插入完成。否则，如果在T中出现高度不平衡的情况，那么根据X以及T和TLR中的项做出单旋转或者双旋转，进而更新高度信息并做好与其它点的链接，来完成整个插入工作；

AVL树的删除相对插入会更复杂一些；

5. 伸展树(splay tree)——摊还(amortized)运行时间为O(f(N))的算法；
伸展树基于这样的一个事实：每次二叉树查找最坏情形O(N)并非不好，但是只要它相对来说不是经常发生就可以。二叉查找树的问题主要在于：虽然一系列的访问整体都是不好操作可能发生，但是会非常罕见；此时，累积的运行时间很重要；具有最坏运行时间O(N)但是保证对任意M次连续操作最多花费O(MlogN)运行时间的查找树已经能让人满意了，因为不存在不好的操作序列；
基本想法：一个节点被访问后，它需要经过一系列AVL树的旋转被推到根上。注意，如果一个节点很深，那么其路径上也就存在许多也相对较深的结点，通过重新构造可以使对所有这些结点的进一步访问所花费的时间变少。因此，如果结点过深，那么重新构造应具有平衡这棵树（到某种程度）的作用。除理论上给出算好的时间界以外，这种方法可能还有实际的效用，因为在很多应用中当一个节点被访问的时候，他就很可能不久之后再被访问。研究表明，这种情况发生的频率比人们预料的高得多。此外，伸展树还不要求保留高度或者平衡信息，因此它可以在某种程度上节省空间并且简化代码；
基本方法：将单旋法变为伸展；

6. 树的遍历：(tree traversal)
已知树的一个节点，来进行遍历，总共有三种方法：
1） 先序(preorder):先访问节点，再访问左子树和右子树；
2） 中序(inorder):先访问左子树，再访问节点，再访问右子树；
3） 后序(postorder)：先访问左子树和右子树，再访问节点；

实例：递归树遍历
该递归函数将树的链接看做一个参数，并将树中每个节点作为参数调用函数visit。这种调用方式，函数实现一种前序遍历；如果我们将visit调用移到递归调用之中，就成为中序调用；如果移到递归调用之后，就成了后序遍历；
```
void traverse( link h, void (*visit)(link))
{
	if ( h == NULL) return;
	( *visit)(h);
	traverse( h->l, visit);
	traverse( h->r, visit);
}
```
给出一个计算树高度的函数：
```
int height( BinaryNode* t)
{
	if ( t == NULL)
		return -1;
	else
		return (1 + max( height( t->left),
				 		 height( t->right));
}
```
特点：中序遍历、后序遍历的运行时间始终为O(N)，这是其一大特征；==前序遍历时间略短？？为O(logN)==

7. *B树：数据特别多的情况下，就无法放入内存，只能把数据结构放到磁盘上；而事实上，在传统机械硬盘的访问情况中，CPU运行速度是远快于数据从磁盘传输到内存的速度的；
为了将磁盘访问次数减小到一个非常小的常数，程序员往往愿意写一个复杂的程序来实现它，因为只要不是特别冗长，机器指令基本上是不占时间的；由于典型的AVL树接近最优高度，应该知道的是，二叉查找树基本上不可行；尽量减少深度是最好的选择，如**M叉查找树(M-ary search tree)**有M路分支，深度有着明显的减少。一棵完全二叉树高度大约为logN,一棵完全M二叉树高度大约是logmN。
同时，为了避免出现退化为链表的情况（只有单个子），也要进行一定的处理，这里引出B树的概念；
定义：阶为M的B树是一棵具有下列结构特性的树：
1） 数据项存储在树叶上；
2） 非叶结点存储到M-1个键，来指示搜索的方向；键i代表子树i+1中最小的键；
3） 树的根或者是一片树叶，或其儿子数在2和M之间；  
4） 除了根以外，所有的非树叶结点的儿子数在M/2到M之间；
5） 所有的树叶都在相同的深度上并有L/2和L之间个数据项；

具体B树的方案后面再做详细解释；

8. 标准库中的set与map
传统的stl容器vector和list对于查找来说是不够用的，stl提供了两个附加的容器set和map，这两个容器保证了节本操作(如插入、删除和查找)的对数时间消耗；

set:
set是一个排序后的容器，不接受重复；许多访问vector和list中的项的例程也适用于set。特别的，iterator和const_iterator类型是嵌套于set的，该类型允许遍历set。
set特有操作是高效的插入、删除和执行基本查找；
插入操作：因为set不接受重复，可能会出现插入失败的情况，因此使用bool变量返回来指示该次插入是否成功。
set的insert提供了两种形式(QSet只提供了一种)，如果知道精确的数据结构就可以以线性时间插入数据；
set的erase操作引入了三种形式（QSet两种）
set的find操作是优于contains操作的，该例程会直接返回一个iterator可以指向项的位置（若失败则指向末端的标志符）；
set默认情况下使用less<Object>函数对象来实现，而该函数是通过对Object调用operator<来实现的。另一种可以替代的排序方法是通过**具有函数对象类型的set模板**来举例说明，例如string对象的一个比较；
```
set< string, CaseIntesiveCompare> s;
s.insert("Hello");s.insert("HeLLo");
cout << "The size is:" << s.size() << endl;
```
其输出为1

map:
map是用来存储排序后由键和键值组成的项的集合；其键值必须唯一，但是多个键可以对应同一个值；因此，值不需要唯一；在map中键保持逻辑排序后的顺序；
map的执行类似于用pair例示的set。其中比较函数仅仅涉及到键。因此，map支持begin、end、size和empty,但是基本的迭代器是一个键一值对。换句话说，对iterator itr,*itr是pair<KeyType, ValueType>类型的。map也支持insert、find和erase。对于insert，必须要提供pair<KeyType, ValueType>对象，而find只需要一个键即可，而其返回值是一个pair。
map同时提供了一个额外的操作符来获取索引对应的内容；
```
ValueType & operator[] ( const KeyType &key)
```
如果在map中存在key，就返回指向相应值的引用。如果在map中不存在key，就在map中插入一个默认的值，然后返回指向这个默认值的引用。该默认值是通过应用零参数的构造函数获得的，如果为基本类型，就是0。由于const内部变量operator[]不能用于常量map对象。
例子：
```
map<string,double> salaries;

salaries["Pat"] = 75000.00;
cout << salaries["Pat"] << endl;
cout << salaries["Jan"] << endl;

map< string, double>::const_iterator itr;
itr = salaries.find( "Chris");
if ( itr == salaries.end())
	cout << "Not an empolyee of this company!" << endl;
else
	cout << it->second << endl;
```
set与map的实现：
C++需要set和map支持在最坏的情况下对基本的操作insert、erase和find仅消耗对数时间；相对应的，底层实现的是平衡二叉查找树，典型的情况往往不是使用AVL树，而是使用自顶向下的红黑树（后文讨论）；
实现set和map时一个重要问题是需要提供对于迭代器类的支持。在程序内部迭代器在迭代过程中始终保持一个指针指向于“当前”结点。比较困难的地方时如果将高效的迭代器推进到下一个结点。有好几种方案，其中stl采用了线索树(threaded tree)的方法；

Example(Map):
很多单词通过替换一个字母就可以变为另一个单词，例如wine可以变为dine/wife/wind。现在要求写一个程序，来找到所有的至少可以通过替换其中一个字母来实现；
最直接的方法是使用map，其中键为单词，值为通过对键进行单个字符替换就可以得到那些单词的集合。
```
//查找程序
void printHighChangables( const map< string, vector<string>> & adjWords, int minWords = 15)
{
	map< string, vector<string>>::const_iterator itr;
	
	for ( itr = adjWords.begin(); itr != adjWords.end(); ++itr)
	{
		const pair< string, vector<string>> & entry = *itr;
		const vector< string> & words = entry.second;
		
		if ( word.size() >= minWords)
		{
			cout << entry.first << " ("<< words.size() <<"):";
			for ( int i = 0; i < word.size(); i++)
			{
				cout << " " <<words[i];
			}
			cout << endl;
		}
	}
}

bool oneCharOff( const string & word1,
const string & word2)
{
	if ( word1.length() != word2.length())
		return false;
	
	int diffs = 0;
	
	for ( int i = 0; i < word1.length(); i++)
		if ( ++diffs > 1)
			return false;
			
	return diffs == 1;
} 
```

其核心问题在于从一个包含89000个单词的数组来构造map。
```
//计算一个键值为单词，值为单词组（该单词组只有一个字母不同）
//该蛮力测试使用了4层循环，89000个单词对应运行时间为6.5min
map< string, vector<string>> computeAdjacentWords( const vector<string> & words)
{
	map< string, vector<string>> adjWords;
	
	for ( int i = 0; i < words.size(); i++)
	{
		for ( int j = i + 1; j < words.size(); j++)
		{
			if ( oneCharOff( word[i],
						     word[h]))
			{
				adjWords[ words[i]].push_back( words[j]);
				adjWords[ words[j]].push_back( words[i]);
			}
		}
	}
	
	return adjWords;
}
```

```
//方法二：仍然采用一个四次方的算法，但是通过采用比较其词组长度提高了一点速度，大约为77s
map< string, vector<string>> computeAdjacentWords( const vector<string> & words)
{
	map< string, vector<string>> adjWords;
	map< int , vector<string>> wordsByLength;
	
	//根据数组长度进行分组
	for ( int i = 0; i < word.size(); i++)
		wordsByLength[ words[i].length()].push_back( words[i]);
		
	//对每个组单独处理
	map< int, vector<string>>::const_iterator itr;
	for ( itr = wordsByLength.begin(); itr != wordsByLength.end(); ++itr)
	{
		const vector<string> & groupWords = itr->second;
		
		for ( int i = 0; i < groupWords.size(); i++)
			for ( itn j = i + 1; j < groupWords.size(); j++)
				if ( oneCharOff( groupWords[i], groupWords[j]))
				{
					adjWords[ groupWords[i]].push_back( groupWords[j]);
					adjWords[ groupWords[j]].push_back( groupWords[i]);
				}
	}
	
	return adjWords;
}
```

第三个算法更加复杂一些，使用了附加的map。将单词按照长度分组，然后对每个组进行分别操作；假设目前研究长度为4的单词，首先，我们需要找到wine和nine这种只有一个字符不同的单词对；实现：对每一个长度为4的单词，删除第1个字母，保留剩下三个字母的样本。生成一个map，其中键是这个样本，其值是所有这个样本单词的vector；例如四字母单词组的第一个字母，样本"ine"对应“dine”、“fine”、“nine”等。样本"oot"对应“boot”、“foot”等。最后这个map值，即每一个单独的vector形成了一个单词组，在这个组的每一个单词都可以通过一个字符的替换变成其它单词，于是，最后一个map就构造出来，很容易来遍历和添加项到所处理的原始map中。然后我们使用一个新map来处理4字字母单词的第二个字母，然后是第三个和第四个字母；
一般程序架构如下：
```
for each group g, containing words of length len
	for each position p ( ranging from 0 to len - 1)
	{
		Make an empty map<string, vector<string>> repsToWords
		for each word w
		{
			Obtain w's representative by removing position pair
			Update repsToWords
		}
		Use cliques in repsToWords to update adjWords map
	}
```

```
//Solution 3:Costs O(NlogN)
map< string, vector<string>> computeAdjacentWords( const vector<string> & words)
{
	map< string, vector<string>> adjWords;
	map< int , vector<string>> wordsByLength;
	
	//根据长度分组
	for ( int i = 0; i < word.size(); i++)
		wordByLength[ word[i].length()].push_back( word[i]);
	
	//对每个组单独处理
	map< int, vector<string>>::const_iterator itr;
	for ( itr = wordsByLength.begin(); itr != wordsByLength.end(); ++itr)
	{
		const vector<string> & groupWords = itr->second;
		int groupNum = itr->first;
		
		//对每个组中的每个点进行处理
		for ( int i = 0; i < groupNum; i++)
		{
			//删除指定位置上的一个字符，计算其样本值
			//所有具有相同样本值的单词都是邻近的，所以填充出一个新的map
			map< string, vector<string>> repToWord;
			
			for ( int j = 0; j < groupWords.size(); j++)
			{
				string rep = groupWords[ j];
				rep.erase( i, 1);
				repToWord[ rep].push_back( groupWords[j]);
			}
			
			//对一个单词组在map中进行寻找
			map< string, vector<string>>::const_iterator itr2;
			for ( itr2 = repToWord.begin(); itr2 != repToWord.end(); ++itr2)
			{
				const vector<string> & clique = itr2->second;
				if ( clique.size() >= 2)
					for ( int p = 0; p < clique.size(); p++)
						for ( int q = p + 1; q < clique.size(); q++)
						{
							adjWords[ clique[ p]].push_back( clique[ q]);
							adjWords[ clique[ q]].push_back( clique[ p]);
						}
			}
		}
	}
	
	return adjWords;
}
```	
	
## 第5章 散列
本章讨论散列表(Hash Table)，它只支持一部分二叉查找树的操作。散列表的实现叫做散列(Hashing)。散列是一种以常数平均时间执行插入/查找和删除的技术。但是，散列不会支持那些需要进行排序信息的操作。

1. 基本思想
理想的散列基本结构只不过是一个包含一些项的具有固定大小的数组。先前讨论过，查找一般是针对项的某个部分（即数据成员）进行，这部分称之为**键(key)**。我们将表的大小记录为TableSize,并将其理解为散列数据结构的一部分而不是浮动于全局的某个变量。通常的习惯是让表从0~TableSize-1之间变化（其后会解释原因）。

将每个键映射到从0-TableSize-1范围内的某个数，并且将其放到合适的单元中。则这个映射就称为散列函数(hash function)，理想的情况下它能够保证**两个不同的键映射到不同的单元**。不过相对来说这种可能性极低，因为单元数目有限，而且键其实是用不完的。因此，我们需要寻找一个散列函数，让该函数在单元之间均匀的分配键。
除了以上的思想之外，剩余需要解决**冲突(collision)**问题，即决定当两个键散列到同一个值时应该做什么以及如何确定散列大小。

2. 散列函数：
如果输入是整数键，则一般比较合理的方法是直接返回“Key mod Tablesize”，除非Key碰巧具有某些不理想的性质；在这种情况下，散列函数的选择需要仔细考虑。例如，如果表的大小为10而键的个位都是0，那么以上的标准散列函数就不是一个特别好的选择；其原因我们将在后面介绍。为了避免这种情况，好的方法通常是保证表的大小是素数。当输入的键为随机整数时，散列函数不仅运算比较简单，而且分配也相对均匀。
通常情况下，键是字符串，这种情形下，散列函数需要进行仔细选择；
一种方法：把字符串中的ascii码求和；
```
int hash( const string &key, int tablesize)
{
	int hashVal = 0;
	
	for ( int i = 0; i < key.length(); i++)
		hashVal += key[ i];
		
	return hashVal % tableSize;	
}
```