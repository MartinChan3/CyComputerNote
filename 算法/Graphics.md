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


