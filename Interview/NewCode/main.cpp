#include <stdio.h>
#include <vector>
#include <iostream>
#include <stack>

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
                     int ls, int le,
                     std::vector<int> &posOrderH,
                     int rs, int re)
{
    int treeVal = node->val;
    int posInPosOrder(0);
    for (auto i : posOrderH)
    {
        if (i == treeVal)
        {
            break;
        }
        posInPosOrder++;
    }

    le = posInPosOrder - 1;
    rs = posInPosOrder + 1;

    TreeNode *leftNode = new TreeNode(preOrderH[ls + 1]);
    TreeNode *rightNode = new TreeNode(preOrderH[posInPosOrder + 1]);
    buildBinaryTree(leftNode, preOrderH, ls, le, posOrderH, rs, re);
    buildBinaryTree(rightNode, preOrderH, ls, le, posOrderH, rs, re);
}

TreeNode *reBuildBinaryTree(std::vector<int> &preOrderH,
                            std::vector<int> &posOrderH )
{
    TreeNode *root = new TreeNode(preOrderH[0]);
    buildBinaryTree(root, preOrderH, 0, preOrderH.size(),
                    posOrderH, 0, posOrderH.size());

    return root;
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

    return;
}
