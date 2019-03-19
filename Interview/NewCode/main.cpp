#include <stdio.h>
#include <vector>
#include <iostream>
#include <stack>
#include <queue>

//02: replace space
void replaceSpace(char *str, int length){
    int spaceCount = 0;
    for (int i = 0; i < length; i++)
    {
        if (str[i] == ' ')
            spaceCount++;
    }

    if (!spaceCount) return;

    char *newStr = new char[length + 2 * spaceCount];
    int j = 0;
    for (int i = 0; i < length; i++)
    {
        char t = str[i];
        if (t == ' ')
        {
            newStr[j++] = '%';
            newStr[j++] = '2';
            newStr[j++] = '0';
        }
        else
        {
            newStr[j++] = t;
        }
    }

    str = newStr;
    return;
}

//核心：倒序追赶
void replaceSpace2(char *str, int length){
    int blankNumber = 0;
    int oldstringLen;
    for (oldstringLen = 0; str[oldstringLen] != '\0'; oldstringLen++){

        if (str[oldstringLen] == ' ')
            blankNumber++;
    }

    int newstringLen = oldstringLen + blankNumber * 2;
    if (newstringLen > length)
        return;
    str[newstringLen] = '\0';

    //设置两个指针point1和point2分别指向原字符串和新字符串的末尾位置
    int point1 = oldstringLen - 1, point2 = newstringLen - 1;

    while (point1 >= 0 && point2 > point1)
    {
        if (str[point1] == ' ')
        {
            str[point2--] = '0';
            str[point2--] = '2';
            str[point2--] = '%';
        }
        else
            str[point2--] = str[point1];
        point1--;
    }
}

//03 Reverse output a linked list
struct ListNode{
    int val;
    struct ListNode *next;
    ListNode(int x) : val(x), next(nullptr){}
};

void printListFromTailToHead(struct ListNode* head)
{
    std::stack<int> s;
    while (head)
    {
        s.push(head->val);
        head = head->next;
    }

    while(!s.empty())
    {
        std::cout << s.top() << " ";
        s.pop();
    }
}

//Recrusive Tip:it like binary tree to make the travese from bottom
void printListFromTailToHead2(struct ListNode* head)
{
    if (head != nullptr)
    {
        if (head->next != nullptr)
        {
            printListFromTailToHead2(head->next);
        }

        printf("%d ", head->val);
    }
}

//04 Rebuild binary tree
//Pre-Order: First define the root, left part was left child and
//right part was right child
//In-Order: If u know which is root , u can know which is left child
//node and right child node at once;
class TreeNode {
public:
    int val;
    TreeNode *left;
    TreeNode *right;
    TreeNode(int x,
             TreeNode *leftp = nullptr,
             TreeNode *rightp = nullptr) :
        val(x), left(leftp), right(rightp){}
};

void buildBinaryTree(TreeNode* node,
                     std::vector<int> &preOrderH,
                     int pres, int pree,
                     std::vector<int> &posOrderH,
                     int poss, int pose)
{
    int treeVal = node->val;
    int posInPosOrder(pres);
    for (auto i : posOrderH)
    {
        if (i == treeVal)
        {
            break;
        }
        posInPosOrder++;
    }

    pree = posInPosOrder - 1;
    poss = posInPosOrder + 1;

    if (posInPosOrder > pres)
    {
        TreeNode *leftNode = new TreeNode(preOrderH[pres + 1]);
        buildBinaryTree(leftNode, preOrderH, pres + 1, pres + posInPosOrder, posOrderH, pree + 1 - posInPosOrder, pree - 1);
    }
    if (posInPosOrder < pree)
    {
        TreeNode *rightNode = new TreeNode(preOrderH[posInPosOrder + 1]);
        buildBinaryTree(rightNode, preOrderH, pres, pree, posOrderH, poss, pose);
    }
}

TreeNode *reBuildBinaryTree(std::vector<int> &preOrderH,
                            std::vector<int> &posOrderH )
{
    TreeNode *root = new TreeNode(preOrderH[0]);
    buildBinaryTree(root, preOrderH, 0, preOrderH.size(),
                    posOrderH, 0, posOrderH.size());

    return root;
}

//09 Number of 1 in num
int numberOf1(int n)
{
    int num(0);
    while (n)
    {
        if (n & 0x1)
            num++;
        n >>= 1;
    }

    return num;
}

//Return k mininst node in binary search tree
//Core: the binary search tree's in-order traverse
//is from min to max
TreeNode *minNode(TreeNode* root, int &k)
{
    if (root)
    {
        TreeNode *node = minNode(root->left, k);
        if (node != nullptr)
            return node;
        k--;
        if (k == 0)
            return root;
        node = minNode(root->right, k);
        if (node != nullptr)
            return node;
    }
    else
    {
        return nullptr;
    }
}

//Binary num has scequence of 0 num
//Study binary data to 1
int seqOf0(int n)
{
    int maxCount(0); int cur0Count(0);
    while (n != 0)
    {
        if (!(n & 0x1)) //This is the only way to judge whether has option
        {
            cur0Count++;
        }
        else
        {
            if (cur0Count > maxCount)
            {
                maxCount = cur0Count;
            }
            cur0Count = 0;
        }

        n >>= 1;
    }

    return maxCount;
}

//Find the depth of binary tree
void depth(TreeNode* root, int &curDepth)
{
    if (!root)
        return;

    curDepth++;

    int tDepthL(0), tDepthR(0);
    if (root->left)
    {
        depth(root->left, tDepthL);
    }
    if (root->right)
    {
        depth(root->right, tDepthR);
    }

    int maxDepth = tDepthL > tDepthR ? tDepthL : tDepthR ;
    curDepth += maxDepth;
}

//Calculate 1 to n sum(Recrusive)
int sum(int n)
{
    if (n == 0)
        return 0;
    return n + sum(n - 1);
}

//Calculate 1 to n sum(No if)
int sum2(int n)
{
    int result = 0;
    int tmp = 0;
    bool flag = (n > 0) && tmp == (result = sum2(n - 1));
    result += n;
    return result;
}

//Print binary tree from top to bottom
void pushToQueue(TreeNode* node, std::queue<TreeNode*> &q)
{
    if (!node)
        return;
    q.push(node);
    if (node->left)
        pushToQueue(node->left,q);
    if (node->right)
        pushToQueue(node->right,q);
}

void printFromTopToBottom(TreeNode* head)
{
    if (!head)
        return;

    std::queue<TreeNode*> q, qf;
    q.push(head);
    while (!q.empty()) //Core: Level order traverse
    {
        TreeNode* f = q.front();
        q.pop();
        if (f->left)
            q.push(f->left);
        if (f->right)
            q.push(f->right);
        qf.push(f);
    }

    while (!qf.empty())
    {
        std::cout << " " << (qf.front())->val;
        qf.pop();
    }
}

//Print binary tree to multi lines like level-order
void printFromTopToBottomInMultiLines(TreeNode* head)
{
    if (!head)
        return;

    std::queue<TreeNode*> q;
    q.push(head);

    std::vector<std::vector<TreeNode*>> vec2D_Nodes;
    std::vector<TreeNode*> vec_Nodes;

    //Core: use now level flag to show if one layer has ouputed
    int now_level = 1;
    int next_level = 0;

    while (!q.empty())
    {
        TreeNode* f = q.front();
        vec_Nodes.push_back(f);
        q.pop();

        if (f->left)
        {
            q.push(f->left);
            ++next_level;
        }
        if (f->right)
        {
            q.push(f->right);
            ++next_level;
        }
        --now_level;

        if (now_level == 0)
        {
            now_level = next_level;
            next_level = 0;
            vec2D_Nodes.push_back(vec_Nodes);
            vec_Nodes.clear();
        }
    }

    for (auto it1 : vec2D_Nodes)
    {
        for (auto it2 : it1)
        {
            std::cout << it2->val << " ";
        }
        std::cout << std::endl;
    }
}

void main()
{
    //02
    char c[] = "Time flys quickly";
    std::cout << c << std::endl;
    replaceSpace2(&(c[0]), 1000);
    std::cout << c << std::endl;

    //03
    ListNode n1(1), n2(2), n3(3), n4(4);
    n3.next = &n4; n2.next = &n3; n1.next = &n2;
    printListFromTailToHead(&n1);
    printListFromTailToHead2(&n1);

    //04
    std::vector<int> preOrderIntArr = {1,2,4,7,3,5,6,8};
    std::vector<int> inOrderIntArr = {4,7,2,1,5,3,8,6};
    reBuildBinaryTree(preOrderIntArr, inOrderIntArr);

    //09
    std::cout << std::endl << "09 Number of 100 is ";
    std::cout << numberOf1(100);

    //find Kth binary node
    TreeNode b1(1), b2(2), b3(3), b5(5), b6(6), b7(7), b8(8), b9(9);
    b2.left = &b1; b2.right = &b3;
    b7.left = &b6; b7.right = &b8;
    b5.left = &b2; b5.right = &b7;
    int k1 = 3, k2 = 6;
    std::cout << std::endl << "Kth is ";
    std::cout << minNode(&b5, int(k2))->val;

    //Return 0 max sequenced num
    std::cout << seqOf0(0x44);

    //Return depth of binary tree
    b8.right = &b9;
    int depthT = 0;
    depth(&b5, depthT);
    std::cout << std::endl << "Depth of tree is "<< depthT;

    //calculate sum
    std::cout << std::endl << "Recursive sum is " << sum(3);
    std::cout << std::endl << "Recursive sum is " << sum2(3);

    //print from top
    std::cout << std::endl;
    printFromTopToBottom(&b5);

    //print above in multi lines
    std::cout << std::endl;
    printFromTopToBottomInMultiLines(&b5);

    return;
}
