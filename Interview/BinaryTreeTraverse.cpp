//二叉树的遍历：遍历时间复杂度O(N)
#include <iostream>
#include <stack>
#include <vector>
#include <list>
#include <queue>
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

//先序遍历:先处理完父节点，再处理左结点和右结点
void PreOrder(Node* head){
    if (head == nullptr)
        return;
    cout << head->val << " ";
    PreOrder(head->left);
    PreOrder(head->right);
}

//中序遍历:先处理完左结点，再处理父节点和右结点
void InOrder(Node* head){
    if (head == nullptr)
        return;
    InOrder(head->left);
    cout << head->val << " ";
    InOrder(head->right);
}

//后序遍历：先处理完左结点右结点，再处理父节点
void PosOrder(Node* head){
    if (head == nullptr)
        return;
    PosOrder(head->left);
    PosOrder(head->right);
    cout << head->val << " ";
}

//非递归版本：自己生成堆栈来进行遍历
//先序遍历：先将根节点入栈，从栈中取出栈顶结点，打印该结点
//之后先将右孩子压入栈（因为栈特点是先进后出），再将左孩子
//压入栈，知道栈为空
void PreOrderUnRecur(Node* head)
{
    if (head)
    {
        stack<Node*> Stack;
        Stack.push(head);
        Node* cur;
        while(!Stack.empty())
        {
            cur = Stack.top();
            Stack.pop();
            cout << cur->val << " ";
            //右孩子先进栈，后出
            if (cur->right)
                Stack.push(cur->right);
            if (cur->left)
                Stack.push(cur->left);
        }
    }
}

//中序遍历:1) 先让cur入栈，不停让cur = cur->left，直到cur
//下所有的左孩子结点入栈。2) 从栈中弹出栈顶给cur，打印该结点
//,然后令cur = cur->right，回到不中欧2；
//3) 当栈为空且cur为空时，过程结束；
void InOrderUnRecur(Node* head)
{
    if (head)
    {
        stack<Node*> Stack;
        Node *cur = head;
        while (!Stack.empty() || cur != nullptr)
        {
            if (cur)
            {
                Stack.push(cur);
                cur = cur->left;
            }
            else
            {
                cur = Stack.top();
                Stack.pop();
                cout << cur->val << " ";
                cur = cur->right;
            }
        }
    }
}

//后序遍历:双栈方法，申请两个栈s1、s2，先将头结点压入栈
//从s1中弹出的节点记为cur,然后依次将cur的左孩子、右孩子压入s1，
//在这个过程中每一个从s1弹出的结点均压入s2。当s1为空后，
//从s2中依次弹出的节点便是后序遍历的次序；
//主要就是就是利用两个栈进行“倒腾”，压入s2的顺序为中、右、左，
//那么弹出的顺序就变为左、右、中。
void PosOrderUnRecur(Node* head)
{
    if (head)
    {
       stack<Node*> s1, s2;
       s1.push(head);
       Node *cur;
       while (!s1.empty())
       {
           cur = s1.top();
           s1.pop();
           s2.push(cur);
           if (cur->left)
               s1.push(cur->left);
           if (cur->right)
               s1.push(cur->right);
       }

       while (!s2.empty())
       {
           cout << s2.top()->val << " ";
           s2.pop();
       }
    }
}

//后序遍历2：冷静分析，如果遍历到一个点，如何判断是打印该点
//还是先处理它的孩子？无非分为以下几种情况；
//1. 该结点的左右孩子皆为空，即该结点为叶子节点时，那么该次
//遍历就是打印该点；
//2. 如果上次打印的结点为该结点的右孩子，说明该结点子树处理
//完毕，这次遍历打印就是打印该结点；
//3. 如果上一次打印结点为该结点的左孩子，且右孩子为空，说明
//该结点的子树处理完毕，这次遍历就是打印该点。
//4. 否则说明子树没有被访问过，按照右孩子、左孩子的顺序入栈。
void PosOrderUnRecur2(Node* head)
{
    if (head)
    {
        stack<Node*> s;
        s.push(head);
        Node *top, *last(nullptr);
        while (!s.empty())
        {
            top = s.top();
            if ((top->left == nullptr && top->right == nullptr) ||
                (top->right == nullptr && last == top->left)   ||
                 last == top->right)
            {
                cout << top->val << " ";
                last = top;
                s.pop();
            }
            else
            {
                if (top->right)
                    s.push(top->right);
                if (top->left)
                    s.push(top->left);
            }

        }
    }
}

//层序遍历:队列实现
void LevelOrder(Node* head)
{
    if (head)
    {
        queue<Node*> Queue;
        Queue.push(head);
        Node *cur;
        while (!Queue.empty())
        {
            cur = Queue.front(); //这里非常巧妙的利用了LILO的特点
            Queue.pop();
            cout << cur->val << " ";
            if (cur->left)
                Queue.push(cur->left);
            if (cur->right)
                Queue.push(cur->right);
        }
    }
}

//计算根节点链
void findRootPath(Node* root, Node* n, list<Node*> &path, bool &findIt)
{
    if (!root)
    {
        findIt = false;
        return;
    }
    path.push_back(root);
    if (root == n)
    {
        findIt = true;
        return;
    }

    if (root->left)
        findRootPath(root->left, n, path, findIt);
    if (!findIt && root->right)
        findRootPath(root->right, n, path, findIt);
    if (!findIt)
        path.pop_back();
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

    cout << "Pre "; PreOrder(n1); cout << endl;
    cout << "In  ";  InOrder(n1); cout << endl;
    cout << "Pos "; PosOrder(n1); cout << endl;

    cout << "Pre "; PreOrderUnRecur(n1); cout << endl;
    cout << "In  ";  InOrderUnRecur(n1); cout << endl;
    cout << "Pos "; PosOrderUnRecur(n1); cout << endl;
    cout << "Pos2 "; PosOrderUnRecur2(n1); cout << endl;
    cout << "Level "; LevelOrder(n1); cout << endl;

    list<Node*> nodePath;
    bool findIt(false);
    findRootPath(n1, n9, nodePath, findIt);
    for (list<Node*>::iterator it = nodePath.begin();
         it != nodePath.end(); it++)
    {
        cout << (*it)->val << " ";
    }

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
