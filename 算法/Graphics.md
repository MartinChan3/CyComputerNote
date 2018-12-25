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

5. 直线绘制算法：
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