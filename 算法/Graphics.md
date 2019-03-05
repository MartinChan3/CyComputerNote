# Graphics Note

1. **问题**：求多边形交集
[链接](https://www.cnblogs.com/xmphoenix/p/4508454.html)
**思路**:两个多边形相交后，其顶点只会是其多边形交点 + 多边形内部的点
**步骤**：   
1. 计算两个多边形各条边之间的交点；
2. 计算包含在多边形内部的点；
3. 将以上两者按照逆时针排序（类似求凸包法）， 得出最终点集

```
#include <stdlib>
#include <vector>
using namespace std;
typedef struct Point
{
    int x;
    int y;
}Point;

bool PolygonClip(const vector<Point> &poly1,
                 const vector<Point> &poly2,
                 std::vector<Point> &interPoly)                   
{
    if (poly1.size() < 3 || poly2.size() < 3)
        return false;
    
    long x, y;
    //计算多边形交点(这里选择使用依次求交的方法)
    for (int i = 0; i < poly1.size(); i++)
    {
        int poly1_next_idx = (i + 1) % poly1.size();
		for (int j = 0; j < poly2.size(); j++)
		{
			int poly2_next_idx = (j + 1) % poly2.size();
			if (GetCrossPoint(poly1[i], poly1[poly1_next_idx],
							  poly2[j], poly2[poly2_next_idx]), x, y))
			{
				interPoly.push_back(Point(x, y));
			}
		}
    }

    //计算多边形内部点的（这里使用依次检查是否在相对的多边形中）
    for (int i = 0; i < poly1.size(); i++)
    {
        if (IsPointInPolygon(poly2, poly1[i]))
        {
            interPoly.push_back(poly1[i]);
        }
    }
    for (int i = 0; i < poly2.size(); i++)
    {
        if (IsPointInPolygon(poly1, poly2[i]))
        {
            interPoly.push_back(poly2[i]);
        }
    }

    if (interPoly.size() <= 0)
        return false;

    //点集排序
    ClockwiseSortPoint(interPoly);
}
```

2. 两个多边形求并集


3. 扫描线算法
![扫描线算法](https://images2017.cnblogs.com/blog/1305032/201801/1305032-20180125100949115-966145298.png)

## 简单线、点、方向及多边形关系相关算法
1. 判断点是否在（正）矩形内：
```		
bool IsPointInPolygon(const Rect& rc, const Point& p)
{
	double xr = (p.x - rc.p1.x) * (p.x - rc.p2.x);
	double yr = (p.y - rc.p1.y) * (p.y - rc.p2.y);
	
	return ((xr <= 0.0) && (yr <= 0.0));
}
```
若不考虑乘法运算符的情况，可使用
```
bool IsPointInPolygon(const Rect& rc, const Point& p)
{
	double xl, xr, yt, yb;
	if (rc.p1.x < rc.p2.x)
	{
		xl = rc.p1.x;
		xr = rc.p2.x;
	}
	else
	{
		xl = rc.p2.x;
		xr = rc.p1.x;
	}	
	
	if (rc.p1.y > rc.p2.y)
	{
		yt = rc.p1.y;
		yb = rc.p2.y;
	}
	else
	{
		yt = rc.p2.y;
		yb = rc.p1.y;
	}
	
	return ((p.x >= xl && p.x <= xr) && (p.y >= yb && p.y <= yt));
}
```
2. 矢量的叉乘、点乘分别的意义：    
叉乘：P × Q（右手定则）= -(Q × P)    
> 0: Q在P的逆时针方向上     
< 0: Q在P的顺时针方向上    
= 0：Q与P共线，但是方向不定    
点乘：P · Q  = |P|*|Q|*cosθ    
> 0: P Q所夹角为锐角      
< 0: P Q所夹角为钝角      
= 0：P Q正交   

3. 判断断点与直线关系
**思路**：考虑1）跨列实验；2）叉乘结果是否为零
```
bool IsPointOnLineSegment(const LineSeg& ls, const Point& pt)
{
	Rect rc;
	GetLineSegmentRect(ls, rc);//获取包围盒
	double cp = CrossProduct(ls.pe.x - ls.ps.x, ls.pe.y - ls.ps.y,
						     pt.x - ls.ps.x, pt.y - ls.ps.y);
							 
	return ((IsPointInRect(rc, pt)) //排除实验
			&& IsZero(cp));         //1E-8精度
	
}
```

4. 直线段是否有交线   
**思路**：1） 快速排斥实验（矩形排斥验证）；2）**相互**跨立实验；
跨立实验的基本依据，是如果两线相交，则其中一条线段的两个端点一定位于另一条线段的两侧；最终符合(( P1 - Q1 ) × ( Q2 - Q1 )) * (( P2 - Q1 ) × ( Q2 - Q1 )) < 0类似的形式。**并且注意，这样的跨列一般都是相互的**。
![screenShot.png](https://i.loli.net/2018/12/20/5c1b66201836f.png)    

首先是快速排斥算法:
```
//本质为若两个矩形分离，则一定在各个方向（x与y）的一个矩形的极值必定大于另一个矩形的相反的极值
bool IsRectIntersect(const Rect& rc1, const Rect& rc2)
{
	return (    max(rc1.p1.x, rc1.p2.x) >= min(rc2.p1.x, rc2.p2.x)
			 && max(rc2.p1.x, rc2.p2.x) >= min(rc1.p1.x, rc1.p2.x)
			 && max(rc1.p1.y, rc1.p2.y) >= min(rc2.p1.y, rc2.p2.y)
			 && max(rc2.p1.y, rc2.p2.y) >= min(rc1.p1.y, rc1.p2.y));
}
```
接下来就可以给出完整的算法：
```
bool IsLineSegmentIntersect(const LineSeg& ls1, const LineSeg& ls2)
{
	if(IsLineSegmentExclusive(ls1, ls2)) //排斥实验
    {
        return false;
    }

     //( P1 - Q1 ) ×'a1?( Q2 - Q1 )
     double p1xq = CrossProduct(ls1.ps.x - ls2.ps.x, ls1.ps.y - ls2.ps.y,
                                ls2.pe.x - ls2.ps.x, ls2.pe.y - ls2.ps.y);

    //( P2 - Q1 ) ×'a1?( Q2 - Q1 )
    double p2xq = CrossProduct(ls1.pe.x - ls2.ps.x, ls1.pe.y - ls2.ps.y,
                               ls2.pe.x - ls2.ps.x, ls2.pe.y - ls2.ps.y);

    //( Q1 - P1 ) ×'a1?( P2 - P1 )
    double q1xp = CrossProduct(ls2.ps.x - ls1.ps.x, ls2.ps.y - ls1.ps.y,
                               ls1.pe.x - ls1.ps.x, ls1.pe.y - ls1.ps.y);

    //( Q2 - P1 ) ×'a1?( P2 - P1 )
    double q2xp = CrossProduct(ls2.pe.x - ls1.ps.x, ls2.pe.y - ls1.ps.y,
                               ls1.pe.x - ls1.ps.x, ls1.pe.y - ls1.ps.y);

    return ((p1xq * p2xq <= 0.0) && (q1xp * q2xp <= 0.0));
}
```

5. 点是否位于多边形内还是多边形外还是多边形上
**思路**：射线法，但是需要避免类似以下情况的问题
![](http://img.my.csdn.net/uploads/201112/25/0_1324825108e0jt.gif)     

文章提出3原则：
1. 若P在一条边上， 则判定点在多边形内；
2. 如果P发出的水平射线穿过一个线段的两端，则以**“上闭下开”**为原则进行判断；
3. 若与P1P2平行，则忽略该边
文中给出了基本的流程图：
![](http://img.my.csdn.net/uploads/201112/25/0_1324825164iT00.gif)
基本代码如下：
```
bool IsPointInPolygon(const Polygon& py, const Point& pt)
{
	assert(py.IsValid()); //先考虑可能出现的窄边的情况
	
	int count = 0; 
	LineSeg ll = LineSeg(pt, Point(-INFINITE, pt.y));//构建无限长的射线
	for (int i = 0; i < py.GetPolyCount(); i++)
	{
		//当前和下一个点组成线段P1P2
		LineSeg pp = LineSeg(py.pts[i], py.pts[(i + 1) % py.GetPolyCount()]);
		if (IsPointOnLineSegment(pp,pt))
		{
			return true;
		}
		
		if (!pp.IsHorizontal())
		{
			if (IsSameFloatValue(pp.ps.y, pt.y) && (pp.ps.y > pp.pe.y))
			{
				count++;
			}
			else if (IsSameFloatValue(pp.pe.y, pt.y) && (pp.pe.y > pp.ps.y))
			{
				count++;
			}
			else
			{
				if (IsLineSegmentIntersect(pp, ll))
				{
					count++;
				}
			}
		}
	}
	
	return ((count % 2) == 1);
}
```

6. 直线绘制算法：    

一. DDA
思路：按照斜率步进推进(分斜率大于1与小于1两种情况，按照增长快的方向进行步进)
![DDA算法例子](https://upload-images.jianshu.io/upload_images/11218530-edecd9635ec7b04e?imageMogr2/auto-orient/strip%7CimageView2/2/w/741)
```
void PaintArea::drawLineDDA(QPainter &painter, int x0, int y0, int xEnd, int yEnd)
{
	int dx = xEnd - x0, dy = yEnd - y0, steps, k;
	float xIncrement, yIncrement, x = x0, y = y0;
	if (qAbs(dx > dy))
		steps = qAbs(dx);
	else
		steps = qAbs(dy);
	
	xIncrement = float(dx) / float(steps);
	yIncrement = float(dy) / float(steps);
	
	painter.drawPoint(round(x), round(y)); //首点
	for (k = 0; k < steps; k++)
	{
		x += xIncrement;
		y += yIncrement;
		painter.drawPoint(round(x), round(y));
	}
}
```
    
二. Bresenham法    
思路：按照斜率增长特性决定下一个点的位置
[维基百科](https://zh.wikipedia.org/wiki/%E5%B8%83%E9%9B%B7%E6%A3%AE%E6%BC%A2%E5%A7%86%E7%9B%B4%E7%B7%9A%E6%BC%94%E7%AE%97%E6%B3%95)   
维基百科中给出基本的一个思路是：1）先讨论斜率<=1的情况；2）讨论k>1的情况，只需要意识到是和y=x进行对称即可，即交换x、y坐标的内容；3）负斜率的话需要进行首尾坐标的交换。该过程是一个扩展到一般化的结果的流程。最后为了减少浮点数运算，采取整个浮点数乘以斜率达到化解的方法（改变error初始，以及将error计算由递增改为递减做法）
```
void PaintArea::drawLineBresenham(QPainter & painter, int x0, int y0, int x1, int y1)
{
	int x, y, dx, dy, e;
	if (qAbs(x0 - xEnd) > qAbs(y0 - yEnd))
	{
		dx = xEnd - x0; dy = yEnd - y0; e = -dx;
		x = x0; y = y0; 
		for (int i = 0; i <= dx; i++)
		{
			painter.drawPoint(x, y);
			x++;
			e += 2 * dy; 
			if (e >= 0)
			{
				y += 1;
				e = e - 2 * dx;
			}
		}	
	}
	else
	{
		dx = xEnd - x0; dy = yEnd - y0; e = -dx;
		x = x0; y = y0;
		for (int i = 0; i <= dx; i++)
		{
			painter.drawPoint(x, y);
			y++;
			e += 2 * dx;
			if (e >= 0)
			{
				x += 1;
				e = e - 2 * dy;
			}
		}
	
	}
}
```
      
7. 画圆方法   
画圆算法基本都是讨论8分象限的（x,y）（x=0与y=x在第一象限内所夹）象限内的内容，再扩展到其余共八个象限。       
一.中点画圆法     
核心原理为判断右侧的中点是否为相关的内容，基本思路为构造判别式来比较下一个点的具体位置[csdn](https://blog.csdn.net/zl908760230/article/details/53954746)   
![screenShot.png](https://i.loli.net/2018/12/26/5c2333be2841c.png)   
根据判别式内容可以得到基本的一个构造方法为：
1. 输入圆半径r；
2. 计算初始值d = 1.25 - r，x = 0, y = r,画出点(x,y);
3. 若x < y(不超过y = x)这条线，则继续，否则结束；
4. 求下一个点的d值：若d < 0,则先将d更新为d+2x+3，再将(x,y)更新为(x+1,y+1);否则先将d更新为d+2(x-y)+5，再将(x, y)更新为(x+1, y-1)。
5. 画点(x,y),返回3；   
   
改进的原则仍然为尽可能避免浮点数的运算，所以给出e = d - 0.25,这样可以将所有的d相关浮点数判断和运算替换为e相关的整形数据的运算；   
```
void wholeC(int xc, int yc, int x, int y, int color) {
	putpixel(xc + x, yc + y, color); putpixel(xc + x, yc - y, color);
	putpixel(xc - x, yc + y, color); putpixel(xc - x, yc - y, color);
	putpixel(xc + y, yc + x, color); putpixel(xc + y, yc - x, color);
	putpixel(xc - y, yc + x, color); putpixel(xc - y, yc - x, color);
}
void Mcircle(int xc, int yc, int r, int color) {
	int x = 0, y = r, d = 1 - r;
	wholeC(xc, yc, x, y, color);
	while (x <= y) {
		if (d < 0) {
			d += 2 * x + 3;
			x++;
		}
		else {
			d += 2 * (x - y) + 5;
			x++;
			y--;
		}
		wholeC(xc, yc, x, y, color);
	}
}
```

8. 多边形区域填充算法
一. 递归种子填充(seed filling)；
4-联通算法获得的结果可能反而是正确的（而8-联通算法可能错误）
1) 注入填充算法(Flood Fill Algorithm): 注入填充算法并不强调边界，单纯从指定位置开始，将某种指定颜色替换为对应的颜色；
```
typedef struct tagDIRECTION
{
	int x_offset;
	int y_offset;
}DIRECTION;

DIRECTION direction_8[] = { {-1, 0}, {-1, 1}, {0, 1}, {1, 1}, {1, 0}, {1, -1}, {0, -1}, {-1, -1} };

void FloodSeedFill(int x, int y , int old_color, int new_color)
{
	if (GetPixelColor(x, y) == old_color)
	{
		SetPixelColor(x, y, new_color);
		for (int i = 0; i < COUNT_OF(direction_8); i++)
		{
			FloodSeedFill(x + direction_8[i].x_offset, 
						  y + direction_8[i].y_offset,
						  old_color,
						  new_color);
		}
	}
}
```
2) 边界填充算法(Boundary Fill Algorithm):本质和注入填充相同，只是在于有边界确认，即结束的条件不同；
```
void BoundaryFillAlgorithm(int x, int y , int old_color, int boundary_color)
{
	int curCol = GetPixelColor(x, y);
	if ((curCol != boundary_color) && (curCol != new_color))
	{
		SetPixelColor(x, y, new_color);
		for (int i = 0; i < COUNT_OF(direction_8); i++)
		{
			BoundaryFillAlgorithm(x + direction_8[i].x_offset, 
						          y + direction_8[i].y_offset,
						          old_color,
						          new_color);
		}
	}
}
```

二. 扫描线填充算法     
1) 扫描线种子填充算法[csdn](https://blog.csdn.net/orbit/article/details/7343236)     
为了避免种子填充方式会存在大量的栈空间存储相邻点，而且递归的次数巨大；扫描线种子填充算法采用沿水平扫描线方式填充像素，一段一段的来处理联通的相邻点。这样只需要将每个水平线段的起点像素压入栈，而不需要将当前处理点周围所有相邻点压入栈，极大地节省了栈的空间。是一种避免递归，提高效率的方式；
核心思路：从给定的种子点(x, y)开始向左向右两个方向填充种子点所在扫描线上位于给定区域的一个区段，同时记下这个区段的范围[xLeft, xRight]，然后确定和这一个区段相联通的上、下两条扫描线上位于给定区域内的区段，并依次保存下来。反复这个过程，直到填充结束。
> 扫描线种子填充算法四个步骤：
> 1. 初始化一个空栈存放种子点，将种子点(x, y)压入栈；
> 2. 判断栈是否为空，若为空则结束；否则取出栈顶元素作为当前扫描线的种子点(x,y),y是当前的扫描线；
> 3. 从种子点(x, y)出发，沿当前扫描线向左、右两个方向填充，直到边界。分别标记左右端点的坐标为xLeft和xRight;
> 4. 分别检查与当前扫描线相邻的y-1和y+1两条扫描线在区间[xLeft, yLeft]中的像素，从xLeft开始向xRight方向搜索。若存在非边界且未填充的像素点，则找出这些相邻像素点中**最右边的一个，并将其种子压入栈，然后返回第二步**       

算法：   
```
void ScanLineSeedLine(int x, int y, int new_color, int boundary_color)
{
	std::stack<Point> stk;
	
	stk.push(Point(x, y)); //第1步，种子点入栈
	while (!stk.empty())
	{
		Point seed = stk.top(); //第2步，取当前种子点
		stk.pop();
		
		//第3步，向左右填充
		int count = FillLineRight(seed.x, seed.y, new_color, boundary_color);//向右填充
		int xRight = seed.x + count - 1;
		count = FillLineLeft(seed.x - 1, seed.y, new_color, boundary_color);//向左填充
		int xLeft = seed.x - count;
		
		//第4步，处理相邻两条扫描线
		SearchLineNewSeed(stk, xLeft, xRight, seed.y - 1, new_color, boundary_color);
		SearchLineNewSeed(stk, xLeft, xRight, seed.y + 1, new_color, boundary_color);
	}
}

void SearchLineNewSeed(std::stack<Point> &stk, int xLeft, int xRight,
					   int y, int new_color, int boundary_color)
{
	int xt = xLeft;
	bool findNewSeed = false;
	
	while(xt <= xRight)
	{
		findNewSeed = false;
		while (IsPixelValid(xt, y, new_color, boundary_color) && (xt < xRight))
		{
			findNewSeed = true;
			xt++;
		}
		if (findNewSeed)
		{
			if (IsPixelValid(xt, y, new_color, boundary_color) && (xt == xRight))
				stk.push(Point(xt, y));
			else
				stk.push(Point(xt - 1, y));
		}
	}
	
	//向右跳过内部无效的点(处理内部右侧有障碍点的情况)
	int xspan = SkipInvalidInLine(xt, y, xRight, new_color, boundary_color);
	xt += (xspan == 0) ? 1 : xspan;
}
```
2) 扫描线算法（有序边表法）:适用于矢量图填充，不需要种子点，适合计算机自动图形处理的场合；[csdn](https://blog.csdn.net/orbit/article/details/7368996)      

基本思想：由水平扫描线从上到下(或者从下到上)扫描一个由多条首尾相连的线段构成的多边形，每根扫描线和多边形的边界产生一系列的交点，若将这些交点按照x坐标排序，然后将排序后的点两两成对，作为线段的两个交点，以所填的颜色绘制水平线；多边形被扫描完毕后，整个多边形的填充也就完成了。   
> 步骤：  
> 求交：计算扫描线和多边形交点
> 交点排序：对所得交点按照x值从小到大排序；
> 颜色填充：对排序后的交点两两组成一个水平线段，进行填充；
> 判断是否完成，若否则继续处理   
核心在于第一步，需要以尽量少的计算量求出交点；其次交点的步进计算最好是整数，方便光栅设备输出；  
两大特点：  
- 每次只有相关的几条边可能有交点，不必对所有边进行求交计算；
- 相邻的扫描线和同一直线段的交点存在步进关系，这个关系与直线段所在直线的斜率有关；
第一点显而易见，为了减少计算量，扫描线算法需要维护一张由“活动边”组成的表，称为“活动边表(AET)”。   
第二点可以证明，假设当前扫描线和多边形某一条边的交点已经通过直线段求交算法计算出来，得到的交点坐标若为(x,y)，则下一条扫描线和这条边的交点不需要再进行求交计算。通过步进关系可以直接得到新的交点为(x+δx, y+1);前面提到过，步进关系δx是一个常量，与直线斜率有关。下面进行推导：   
假设多边形某条边所在的直线方程为：ax+by+c = 0，扫描边yi和下一条扫描边yi+1与该边的两个交点分别是(xi, yi)和(xi+1,yi+1)。   
则可以得到以下两个等式：   
axi + byi + c = 0                         (等式 1)   
axi+1 + byi+1 + c = 0                     (等式 2)   
由等式1可以得到等式3：   
xi = -(byi + c) / a                       (等式 3)   
同样，由等式2可以得到等式4：   
xi+1 = -(byi+1 + c) / a                   (等式 4)   
等式4-等式3可得：   
xi+1 – xi = -b (yi+1 - yi) / a = -b/a = δx   
由此可见它为直线斜率的负数。   
“活动边表”是扫描线填充算法的核心，整个算法都是围绕着这张表进行处理的。要定义完整的“活动边表”，需要先定义边的数据结构。每条边都和扫描线有个交点，扫描线填充算法只关注交点的x坐标，每当处理下一条扫描线时，根据δx计算出新扫描线与边的交点x坐标，从而可以避免复杂的求交运算。一条边不会一直待在“活动边表”中，当扫描线与之没有交点时，就要将其从“活动边表”中删除。判断是否有交点的标准就是看扫描线是否大于这条边两个端点的y坐标值，为此，需要记录边的y坐标最大值。根据以上分析，边的数据结构可以定义为：    
```
typedef struct tagEDGE
{
	double xi;
	double dx;
	int    ymax;
}EDGE;
```
[csdn](https://blog.csdn.net/orbit/article/details/7368996)
![活动边表示意图](https://img-my.csdn.net/uploads/201203/19/0_1332138897cTqT.gif)
扫描线（y=4）的活动边表：   
![扫描线4的活动边表](https://img-my.csdn.net/uploads/201203/19/0_133213919970Pv.gif)   
![扫描线7的活动边表](https://img-my.csdn.net/uploads/201203/19/0_1332139249Oi4K.gif)   
前面提到过，扫描线算法的核心就是围绕“活动边表(AET)”展开的，为了方便活动边表的建立与更新，我们为每一条扫描线建立一个“新边表(NET)”，存放该扫描线第一次出现的边。当算法处理到某条扫描线时，就将这条扫描线的“新边表”中所有边逐一插入到“活动边表”中。新边表通常在算法开始的时候建立，建立新边表的规则就在于：**如果某条边的较低端点(y坐标较小的那个点)的y坐标与扫描线y相等，则该边就是扫描线y的新边，应该加入扫描线的新边表。**上图例子中的新边表如下图所示：   
![]()