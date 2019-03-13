//问题：给定任意一棵树，给定根节点和两个子结点，要求以最快的方法找到
//两个子结点的最近父节点
#include <iostream>
#include <stack>
#include <vector>
#include <list>
#define  SAFE_DELETE(x) {if(x) delete x; x = nullptr;}
using namespace std;

typedef struct _NODE{
    int   val;
    _NODE *left;
    _NODE *right;

    _NODE(int ti, _NODE* tLeft, _NODE* tRight)
    {
        val = ti;
        left = tLeft;
        right = tRight;
    }
}Node;

//寻找固定子树的固定点（Core）
bool searchNode(Node* root, std::list<Node*>& s, Node* node){
    if (root == nullptr)
        return false;
    s.push_back(root);
    if (root == node)
        return true;
    bool b = false;
    if (root->left != nullptr) b = searchNode(root->left, s, node);//左子树寻找
    if (!b && root->right != nullptr) b = searchNode(root->right, s, node);//左子树寻找不到且右子树不为空的情况停止寻找
    if (!b) s.pop_back(); //左右都找不到，弹出栈顶元素
    return b;
}

Node* findSParent(Node *root, Node *node1, Node *node2)
{
    std::list<Node*> s1, s2;
    searchNode(root, s1, node1);
    searchNode(root, s2, node2);

    int size = s1.size() > s2.size() ? s2.size() : s1.size();
    Node* lastOne(nullptr);
    std::list<Node*>::iterator rIt1, rIt2;
    rIt1 = s1.begin();
    rIt2 = s2.begin();

    for (int i = 0; i < size; i++)
    {
        if (*rIt1 != *rIt2)
        {
            break;
        }
        lastOne = *rIt1;
        rIt1++; rIt2++;
    }

    return lastOne;
}

void main(){
    Node *n1, *n2, *n3, *n4, *n5, *n6, *n7, *n8, *n9;
    n9 = new Node(9, nullptr, nullptr);
    n8 = new Node(8, nullptr, nullptr);
    n7 = new Node(7, nullptr, nullptr);
    n6 = new Node(6, nullptr, nullptr);
    n5 = new Node(5, n8, n9);
    n4 = new Node(4, n6, n7);
    n3 = new Node(3, nullptr, nullptr);
    n2 = new Node(2, n4, n5);
    n1 = new Node(1, n2, n3);

    cout << findSParent(n1, n6, n3)->val;

    SAFE_DELETE(n1);
    SAFE_DELETE(n2);
    SAFE_DELETE(n3);
    SAFE_DELETE(n4);
    SAFE_DELETE(n5);
    SAFE_DELETE(n6);
    SAFE_DELETE(n7);
    SAFE_DELETE(n8);
    SAFE_DELETE(n9);
}
