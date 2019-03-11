/**********************
 * 填充测试：有序边表法
 * 参照案例：https://blog.csdn.net/orbit/article/details/7368996
 * ********************/
#include <iostream>
#include <vector>
#include <list>
#include "assert.h"

typedef struct tagPoint
{
    int x;
    int y;

    tagPoint(int nx, int ny){
        x = nx;
        y = ny;
    }
}Point;

typedef std::vector<Point> Polygon;

typedef struct tagEDGE
{
    double xi;
    double dx;
    int ymax;
}EDGE;

void ScanLinePolygonFill(const Polygon& py, int color);
void GetPolygonMinMax(const Polygon& py, int &ymin, int &ymax);
void InitScanLineNewEdgeTable(std::vector<std::list<EDGE>> &slNet,
                              const Polygon& py, int ymin, int ymax);
void ProcessScanLineFill(std::vector<std::list<EDGE>> &slNet,
                         int ymin, int ymax, int color);

void InsertNetListToAet(std::list<EDGE> &edges, std::list<EDGE> &aet);
void FillAetScanLine(std::list<EDGE> &aet, int y, int color);
void RemoveNonActiveEdgeFromAet(std::list<EDGE>& aet, int y);
void UpdateAndResortAet(std::list<EDGE>& aet);
bool EdgeXiComparator(EDGE& e1, EDGE& e2){return e1.xi <= e2.xi;}

int main()
{
    Polygon py;
    py.push_back(Point(1,1));
    py.push_back(Point(1,3));
    py.push_back(Point(4,4));
    py.push_back(Point(7,3));
    py.push_back(Point(7,2));
    py.push_back(Point(7,1));
    py.push_back(Point(4,2));

    ScanLinePolygonFill(py, 1);
    return 0;
}

//主填充函数
void ScanLinePolygonFill(const Polygon &py, int color){
    assert(py.size());

    int ymin = 0;
    int ymax = 0;
    GetPolygonMinMax(py, ymin, ymax);
    std::vector<std::list<EDGE>> slNet(ymax - ymin + 1);
    InitScanLineNewEdgeTable(slNet, py, ymin, ymax);
    //HorizonEdgeFill(py, color); //水平边填充
    ProcessScanLineFill(slNet, ymin, ymax, color);
}

void GetPolygonMinMax(const Polygon &py, int &ymin, int &ymax){
    ymax = ymin = py.at(0).y;
    for (int i = 0; i < py.size(); i++)
    {
        int ty = py.at(i).y;
        ymin = ty > ymin ? ymin : ty;
        ymax = ty < ymax ? ymax : ty;
    }
}

//初始化边表
void InitScanLineNewEdgeTable(std::vector<std::list<EDGE>> &slNet, const Polygon &py, int ymin, int ymax){
    EDGE e;
    for (int i = 0; i < py.size(); i++)
    {
        const Point& ps = py.at(i);
        const Point& pe = py.at((i + 1) % py.size());
        const Point& pss = py.at((i - 1 + py.size()) % py.size());
        const Point& pee = py.at((i + 2 + py.size()) % py.size());

        if (pe.y != ps.y) //仅处理非水平线
        {
            e.dx = double(pe.x - ps.x) / double(pe.y - ps.y);
            if (pe.y > ps.y)
            {
                e.xi = ps.x;
//                if (pee.y >= pe.y)
//                    e.ymax = pe.y - 1;
//                else
                    e.ymax = pe.y;

                slNet[ps.y - ymin].push_front(e);
            }
            else
            {
                e.xi = pe.x;
//                if (pss.y >= ps.y)
//                    e.ymax = ps.y - 1;
//                else
                    e.ymax = ps.y;
                slNet[pe.y - ymin].push_front(e);
            }
        }
    }
}

//进行非水平线段的填充
void ProcessScanLineFill(std::vector<std::list<EDGE> > &slNet,
                         int ymin, int ymax, int color)
{
    std::list<EDGE> aet;
    for (int y = ymin; y < ymax; y++)
    {
        InsertNetListToAet(slNet[y-ymin], aet);//从新边表中取值给活动边表
        FillAetScanLine(aet, y, color);        //进行填充
        RemoveNonActiveEdgeFromAet(aet, y + ymin);    //删除非活动边(这里应该修复为y+ymin)
        UpdateAndResortAet(aet);               //更新活动边表中的xi值，并根据xi重新排序
    }
}

//从新边表(NET)中构建活动边表(AET)
void InsertNetListToAet(std::list<EDGE> &edges, std::list<EDGE> &aet){
    if (edges.size() == 0) return;

    aet.push_back(*(edges.cbegin()));
    std::list<EDGE>::iterator i;
    for ( i = (++edges.begin()); i != edges.end(); i++)
    {
        double tx = i->xi;
        bool tb = false;
        for (std::list<EDGE>::iterator j = aet.begin(); j != aet.end(); j++)
        {
            if (j->xi > tx)
            {
                aet.insert(j, *i);
                tb = true;
                break;
            }
        }

        if (!tb) aet.push_back(*i);
    }
}

void FillAetScanLine(std::list<EDGE> &aet, int y, int color){
    std::list<EDGE>::iterator itrator = aet.begin();
    std::cout << "In height " << y;
    do{
       std::cout << " " << itrator->xi;
       itrator++;
    }while(aet.end() != itrator);
    std::cout << std::endl;
}

void RemoveNonActiveEdgeFromAet(std::list<EDGE> &aet, int y){
    std::list<EDGE> aetT;
    std::list<EDGE>::iterator i;
    for (i = aet.begin(); i != aet.end(); i++)
    {
        if (i->ymax > y)
            aetT.push_back(*i);
    }
    aet.swap(aetT);
}

void UpdateAndResortAet(std::list<EDGE>& aet){
    std::list<EDGE>::iterator i;
    for (i = aet.begin(); i != aet.end(); i++)
        i->xi += i->dx;
    aet.sort(EdgeXiComparator);
}
