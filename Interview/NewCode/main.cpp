#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <vector>
#include <iostream>
#include <sstream>
#include <stack>
#include <queue>
#include <functional>
#include <vld.h>
#include <hash_map>
#include <set>

#define MODULE 100000007

template<class T>
void print(std::vector<T> grp)
{
    for (auto i : grp)
    {
        std::cout << i << " ";
    }
}

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


void release(TreeNode* root)
{
    if (!root)
        return;

    if (root->left)
        release(root->left);
    if (root->right)
        release(root->right);

    delete root;
    root = nullptr;
}

void buildBinaryTree(TreeNode* node,
                     std::vector<int> &preOrderH,
                     int pres, int pree,
                     std::vector<int> &inOrderH,
                     int poss, int pose)
{
    int treeVal = node->val;
    int posInPosOrder(pres);
    for (auto i : inOrderH)
    {
        if (i == treeVal)
        {
            break;
        }
        posInPosOrder++;
    }

    if (posInPosOrder > pres)
    {
        TreeNode *leftNode = new TreeNode(preOrderH[pres + 1]);
        buildBinaryTree(leftNode,
                        preOrderH, pres + 1, posInPosOrder - 1,
                        inOrderH, poss + 1, poss + posInPosOrder - 1);
    }
    if (posInPosOrder < pree)
    {
        TreeNode *rightNode = new TreeNode(preOrderH[pres + 1 + posInPosOrder]);
        buildBinaryTree(rightNode,
                        preOrderH, pres + posInPosOrder + 1, pree,
                        inOrderH, poss, pose);
    }
}

TreeNode *reBuildBinaryTree(std::vector<int> &preOrderH,
                            std::vector<int> &inOrderH )
{
    TreeNode *root = new TreeNode(preOrderH[0]);
    buildBinaryTree(root, preOrderH, 0, preOrderH.size(),
                    inOrderH, 0, inOrderH.size());

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

int depth(TreeNode *root)
{
    if (!root)
        return 0;

    int dl(0), dr(0);
    if (root->left)
        dl = depth(root->left);
    if (root->right)
        dr = depth(root->right);
    return dl > dr ? ++dl : ++dr;
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

//Print binary tree in left-right crossed seq
void printFromTopToBottomInMultiLinesCrossed(TreeNode* head)
{
    if (!head)
        return;

    std::queue<TreeNode*> q;
    q.push(head);

    std::vector<std::vector<TreeNode*>> vec2D_Nodes;
    std::vector<TreeNode*> vec_Nodes;

    int now_level = 1;
    int next_level = 0;
    while (!q.empty())
    {
        TreeNode* f = q.front();
        if (!(vec2D_Nodes.size() % 2))
            vec_Nodes.push_back(f);
        else
            vec_Nodes.insert(vec_Nodes.begin(), f);
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

//Print median in data flow
//Use a max heap and a min heap, (Hmax's max
// is less than Hmin's min).
void Insert(int num,
            std::vector<int> &minHeap,
            std::vector<int> &maxHeap)
{
    int data = (minHeap.size() + maxHeap.size()) & 0x1;
    if (data == 0)
    {
        //Even size of previous heaps
        if (!maxHeap.empty() && num <= maxHeap[0])
        {
            maxHeap.push_back(num);
            std::push_heap(maxHeap.begin(), maxHeap.end(), std::less<int>());

            std::pop_heap(maxHeap.begin(), maxHeap.end(), std::less<int>());
            num = *(maxHeap.end() - 1);
            maxHeap.pop_back();
        }

        minHeap.push_back(num);
        std::push_heap(minHeap.begin(), minHeap.end(), std::greater<int>());
    }
    else
    {
        //Odd size of previous heaps
        if (!minHeap.empty() && num > minHeap[0])
        {
            minHeap.push_back(num);
            std::push_heap(minHeap.begin(), minHeap.end(), std::greater<int>());

            std::pop_heap(minHeap.begin(), minHeap.end(), std::greater<int>());
            num = *(minHeap.end() - 1);
            minHeap.pop_back();
        }

        maxHeap.push_back(num);
        std::push_heap(maxHeap.begin(), maxHeap.end(), std::less<int>());
    }
}

double GetMedian(int num,
                 std::vector<int> &minHeap,
                 std::vector<int> &maxHeap)
{
    Insert(num, minHeap, maxHeap);
    if (((minHeap.size() + maxHeap.size()) & 0x1)== 0)
    {
        if (maxHeap.size() == 0 || minHeap.size() == 0)
            return 0;
        else
            return (maxHeap[0] + minHeap[0]) * 1.0 / 2.0;
    }
    else
    {
        return minHeap[0];
    }
}

//Binary tree path sum is a fixed val
void findSPath(TreeNode* node,
               int expectedNum,
               std::vector<TreeNode*> path,
               std::vector<std::vector<TreeNode*>>& paths)
{
    if (node)
    {
        path.push_back(node);
        expectedNum -= node->val;
        if (0 == expectedNum)
        {
            paths.push_back(path);
        }

        if (node->left)
            findSPath(node->left, expectedNum, path, paths);
        if (node->right)
            findSPath(node->right, expectedNum, path, paths);
    }
}

std::vector<std::vector<TreeNode*>> findPaths(TreeNode *node, int expectedNum)
{
    std::vector<std::vector<TreeNode*>> paths;
    std::vector<TreeNode*> sPath;

    findSPath(node, expectedNum, sPath, paths);
    return paths;
}

//Judege tree B is same with tree A
bool isEqual(TreeNode* ta, TreeNode* tb)
{
    if (!ta && !tb)
        return true;
    if (!ta || !tb)
        return false;

    if (ta->val == tb->val)
        return (isEqual(ta->left, tb->left) && isEqual(ta->right, tb->right));
    else
        return false;
}

//Judege tree B is part of tree A
bool isPart(TreeNode* ta, TreeNode* tb) //Same to equal func above
{
    if (!tb)
        return true;
    if (!ta)
        return false;
    if (ta->val != tb->val)
        return false;
    else
        return isPart(ta->left, tb->left) &&
                isPart(ta->right, tb->right);
}

bool isPartTree(TreeNode* ta, TreeNode* tb)
{
    if (!ta && !tb)
        return true;
    if (!ta || !tb)
        return false;

    bool findIt(false);
    if (ta->val == tb->val)
        findIt = isPart(ta, tb);
    if (findIt)
        return true;
    else
        return isPartTree(ta->left, tb) || isPartTree(ta->right, tb);
}

//Mirror the binary tree
void mirror(TreeNode* node)
{
    if (node)
    {
        TreeNode* tt;
        tt = node->left;
        node->left = node->right;
        node->right = tt;
        if (node->left)
            mirror(node->left);
        if (node->right)
            mirror(node->right);
    }
}

//Judge if a list is the result of pos-order traverse of binary search tree
//Core: pos-order has the regular : some-small→some-big→root
bool isBinarySearchTree(std::vector<int> &grp, int start, int end)
{
    if (grp.size() == 0)
        return true;
    if ((end - start) == 0)
        return true;

    int rootVal = grp[end];
    int firstMaxIndex = start;
    while (firstMaxIndex < end && grp[firstMaxIndex] < rootVal)
    {
        firstMaxIndex++;
    }

    bool isBTree(true);
    for (int i = firstMaxIndex; i < end; i++)
    {
        if (grp[firstMaxIndex] <= rootVal)
            isBTree = false;
    }

    if (!isBTree)
        return false;
    else
    {
        if (firstMaxIndex - 1 == start)
            isBTree = true;
        else
            isBTree = isBinarySearchTree(grp, start, firstMaxIndex - 1);
        if (!isBTree)
            return false;
        if (firstMaxIndex == end - 1)
            isBTree = true;
        else
            isBTree = isBinarySearchTree(grp, firstMaxIndex, end - 1);
        if (!isBTree)
            return false;
    }
}

//Transfer a binary search tree to sorted list, and no new nodes
// in the example
void transferBinaryTreeToSortedList(TreeNode* root)
{
    if (root)
    {
        TreeNode* tl, *tr;
        tl = root->left;
        tr = root->right;

        if (tl)
        {
            transferBinaryTreeToSortedList(tl);
            while (tl->right)
                tl = tl->right;
            tl->right = root;
        }

        if (tr)
        {
            transferBinaryTreeToSortedList(tr);
            while (tr->left)
                tr = tr->left;
            tr->left = root;
        }
    }
}

//Judege a b-tree is balanced
//Balanced b-tree: a tree whose left and right depeth
//diff within 1.
int depthMax(TreeNode* head)
{
    return depth(head);
}

int depthMin(TreeNode* head)
{
    if (!head)
        return 0;

    int tl(0), tr(0);
    if (head->left)
        tl = depthMin(head->left);
    if (head->right)
        tr = depthMin(head->right);

    return tl < tr ? ++tl : ++tr;
}

bool isBalanced(TreeNode* head)
{
    if (!head)
        return true;

    int dmax, dmin;
    dmax = depthMax(head);
    dmin = depthMin(head);
    if ((dmax - dmin) > 1)
        return false;
    else
    {
        return isBalanced(head->left) &&
                isBalanced(head->right);
    }
}

//above optimized: No recrusive cal in
//already calculated level by recording depth
bool isBalanced2(TreeNode* head, int &depth)
{
     if (!head)
     {
         depth = 0;
         return true;
     }

     //Pos-order traverse ,and record the current depth
     int left, right;
     if (isBalanced2(head->left, left) &&
             isBalanced2(head->right, right))
     {
         if (abs(right - left) <= 1)
         {
             depth = left > right ? left + 1: right + 1;
             return true;
         }
     }

     return false;
}

//Has a b-tree and a node, find the next in-order traverse
//node, and each node has a ptr to father.
class PTreeNode {
public:
    int val;
    PTreeNode *left;
    PTreeNode *right;
    PTreeNode *parent;
    PTreeNode(int x,
              PTreeNode *leftp = nullptr,
              PTreeNode *rightp = nullptr,
              PTreeNode *parentp = nullptr) :
        val(x), left(leftp), right(rightp), parent(parentp){}
};

PTreeNode* findNextInOrderNode(PTreeNode* root, PTreeNode* n)
{
    if (n->right) //When root has right child
    {
        PTreeNode *tr = n->right;
        while (tr->left)
        {
            tr = tr->left;
        }
        return tr;
    }
    else //don't have right child
    {
        PTreeNode *parent = n->parent;
        PTreeNode *pCur = n;
        //Core: if pCur is his parent right child ,find until top
        while (parent && pCur == parent->right)
        {
            pCur = parent;
            parent = pCur->parent;
        }

        return parent; //two situation: 1. it is a left child;2. find root;
    }
}

//Judge a binary tree whether symmetric
bool isSame(TreeNode* ln, TreeNode* rn)
{
    if (!ln && !rn)
        return true;

    if ((!ln && rn) || (ln && !rn))
        return false;

    if (ln->val != rn->val)
        return false;

    bool fIsSame;
    fIsSame = isSame(ln->left, rn->right);
    if (!fIsSame)
        return false;
    fIsSame = isSame(ln->right, rn->left);
    if (!fIsSame)
        return false;
    return true;
}

bool isSymmetric(TreeNode* root)
{
    if (!(root->left->val == root->right->val))
        return false;

    return isSame(root->left, root->right);
}

//Binary tree serialization and reserialization
std::string to_string(int t)
{
    std::stringstream ss;
    ss << t;
    return ss.str();
}

void serializeHelper(TreeNode *node, std::string& s)
{
    if (node == NULL)
    {
        s.push_back('#');
        s.push_back(',');
        return;
    }
    s += to_string(node->val);
    s.push_back(',');
    serializeHelper(node->left, s);
    serializeHelper(node->right, s);
}
char* Serialize(TreeNode *root)
{
    if (root == NULL)
        return NULL;
    std::string s = "";
    serializeHelper(root, s);

    char *ret = new char[s.length() + 1];
    strcpy(ret, s.c_str());
    return ret;
}

TreeNode *deserializeHelper(std::string &s)
{
    if (s.empty())
        return NULL;
    if (s[0] == '#')
    {
        s = s.substr(2);//trim from pos 2 to end
        return NULL;
    }
    TreeNode *ret = new TreeNode(std::stoi(s));
    s = s.substr(s.find_first_of(',') + 1);
    ret->left = deserializeHelper(s);
    ret->right = deserializeHelper(s);
    return ret;
}

TreeNode* Deserialize(char *str)
{
    if (str == NULL)
        return NULL;
    std::string s(str);
    return deserializeHelper(s);
}

//judge a string whether is a numeric val
bool isNumeric(std::string str)
{
    int size = str.size();
    if (!size)
        return false;
    int si = 0;
    if (str[si] == '-' || str[si] == '+')
        si++;
    int rSi = si;
    bool fFindE(false);
    bool fFindP(false);
    int  ePos;
    for (; si < size; si++)
    {
        if (str[si] == '.')
        {
            if (fFindP)
                return false;
            fFindP = true;
        }
        else if (str[si] == 'E' || str[si] == 'e')
        {
            if (fFindE)
                return false;
            if (si == rSi || si == size - 1)
                return false;
            fFindE = true;
            ePos = ++si;
            break;
        }
        else if (!(str[si] >= '0' && str[si] <= '9'))
        {
            return false;
        }
    }

    if (fFindE)
    {
        if (!(str[si] >= '0' && str[si] <= '9'))
        {
            return false;
        }
    }

    return true;
}

//Return the first no repetition character from a string
char findStr(char* str)
{
   //Sacrifice some rom and record the first index and times
   //Time costs O(n)
   int len=strlen(str),count[26],index[26];
   char result,curr;

   for(int j=0;j<26;j++)
   {
       count[j]=0;
       index[j]=0;
   }

   bool exist=false;

   for(int i = len; i > 0; i--)
   {
       curr=str[i-1];
       count[curr-'a']++;
       index[curr-'a']=i-1;
   }

   int minIndex=len+1;
   for(int i=0;i<26;i++)
   {
       if(count[i]==1)
       {
           if(index[i]<minIndex)
           {
               minIndex=index[i];
           }
       }
   }

   return str[minIndex];
}

//char findStrHashed(const char *str)
//{
//    //use hash table to mark it
//    std::hash_map<int, int> hashMap;
//    int size = strlen(str);

//    for (int i = 0; i < size; i++)
//    {
//        int t = str[i];
//        std::hash_map<int ,int>::const_iterator cit;
//        cit = hashMap.find(t);
//        if (cit == hashMap.cend())
//            (hashMap[t])++;
//        else
//            hashMap.insert(t, 0);
//    }

//     char c = -1;
//     for (auto it = hashMap.begin(); it != hashMap.end(); it++)
//     {
//         if (it->second == 1)
//         {
//             c = it->first;
//             break;
//         }
//     }

//     return c;
//}

//Reverse chars
void reverse(char *str)
{
    int size = strlen(str);
    if (size <= 1)
        return;

    char *l, *r;
    l = str;
    r = str + size - 1;
    char t;
    while (l < r)
    {
        t = *l;
        *l = *r;
        *r = t;
        l++; r--;
    }
}

void reverseWord(char *str, int s, int e)
{
    char *l = str + s;
    char *r = str + e;
    char t;
    while (l < r)
    {
        t = *l;
        *l = *r;
        *r = t;
        l++; r--;
    }
}

void reverseWords(char *str)
{
    std::vector<int> spaceIndexs;
    int size = strlen(str);

    spaceIndexs.push_back(-1);
    for (int i = 0; i < size; i++)
    {
        if (str[i] == ' ')
            spaceIndexs.push_back(i);
    }
    spaceIndexs.push_back(size);

    for (int i = 0; i < spaceIndexs.size() - 1; i++)
    {
        int s = spaceIndexs[i] + 1;
        int e = spaceIndexs[i + 1] - 1;
        if (e > s || e - s > 2)
        {
            reverseWord(str, s, e);
        }
    }
}

//Rotate left of str
void rol(char * str)
{
    int size = strlen(str);
    if (size <= 1)
        return;

    char t = *str;
    for (int i = 0; i < size - 1; i++)
    {
        str[i] = str[i + 1];
    }
    str[size - 1] = t;
}

void rol(char * str, int count)
{
    int size = strlen(str);
    int rMove = count % size;

    for (int i = 0; i < rMove; i++)
        rol(str);
}

//Transfer to int
int toInt(std::string str)
{
    if (str.empty())
        return 0;

    bool fNegative(false);
    int startIndex(0);

    if (str[0] == '+')
    {
        startIndex++;
    }
    else if (str[0] == '-')
    {
        startIndex++;
        fNegative = true;
    }

    std::stack<char> s;
    for (int m = startIndex ; m < str.size(); m++)
    {
        char i = str[m];
        if (i <= '9' && i >= '0')
            s.push(i);
        else
            return 0;
    }

    int r(0);
    int place = 0;
    while (!s.empty())
    {
        int t = s.top();
        s.pop();
        r += int(t - '0') * round(pow(10, place));
        place++;
    }

    return fNegative ? -r : r;
}

//String arrange
//Simple way: move one of the character to top and recurse
//But it can't solve the repetition problem
void permute1(std::string prefix, std::string str)
{
    if (str.length() == 0)
        std::cout << prefix << std::endl;
    else
    {
        for (int i = 0; i < str.length(); i++)
            permute1(prefix + str[i],
                     str.substr(0, i) +
                     str.substr(i + 1, str.length()));
    }
}

void permute1(std::string s)
{
    permute1("", s);
}

//Exchange way: can avoid repetition
void swap(char* x, char* y)
{
    char tmp;
    tmp = *x;
    *x = *y;
    *y = tmp;
}

void permute(char *a, int i, int n)
{
    int j;
    if (i == n)
        printf("%s\n", a);
    else
    {
        for (j = i; j <= n; j++)
        {
            if(a[i] == a[j] && j != i) //avoid repetition here
                continue;
            swap( (a+i), (a+j));
            permute(a, i+1, n);
            swap( (a+i), (a+j)); //backtrack
        }
    }
}

//find first repeated num(time O(nlogn))
int firstRepeatedNum(int *intArr, int size)
{
    std::set<int> s;
    for (int i = 0; i < size; i++)
    {
        if (s.end() != s.find(intArr[i]))
        {
            return intArr[i];
        }
        else
            s.insert(intArr[i]);
    }

    return INT_MIN;
}

//no use group way(save space)
void swap(int* x, int* y)
{
    int tmp;
    tmp = *x;
    *x = *y;
    *y = tmp;
}

int firstRepeatedNum2(int *intArr, int size)
{
    for (int i = 0; i < size; i++ )
    {
        int m = intArr[i];
        if (m != i)
        {
            if (intArr[m] == m) ///???here
                return m;
            else
                swap(intArr + i, intArr + m);
        }
    }

    return INT_MIN;
}

//Construct product arr : bi = a1 * .. * a(i - 1) * a(i + 1).. *a(n)
//Core: to make the matrix and left-behind was an increasing arr and
//same to right-above
std::vector<int> genProduct(std::vector<int> &A)
{
    std::vector<int> B;
    if(A.empty())
        return B;

    int length = A.size();
    B.push_back(1);
    for(int i = 1; i < length;i++)
    {
        B.push_back(A[i-1] * B[i-1]);
    }

    int tmp = 1;
    for(int i = length - 2; i >= 0; i--)
    {
        tmp *= A[i+1];
        B[i] *= tmp;
    }

    return B;
}

//Find one number only show once
//Core: XOR operation has law of commutation, and 0 xor any = any
int findOnceShowed(std::vector<int> &grp)
{
    int num = 0;

    for (int i = 0; i < grp.size(); i++)
    {
        num ^= grp[i];
    }

    return num;
}

//Find two numbers only show once in group
void findOnceShowed(std::vector<int> &grp, int &a, int &b)
{
    int sum = 0;
    for (int i = 0; i < grp.size(); i++)
        sum ^= grp[i];

    sum &= -sum;
    a = 0; b = 0;
    for (int i = 0; i < grp.size(); i++)
    {
        if ((sum & grp[i]) == 0)
            a ^= grp[i];
        else
            b ^= grp[i];
    }
}

//Find two number whose sum is S
//Use like Quick sort to move two guard from left and right
std::vector<int> findTwoNumS(std::vector<int> &grp, int s)
{
    int i = 0, j = grp.size() - 1;
    while (i < j)
    {
        int ts = grp[i] + grp[j];
        if (ts == s)
            break;
        else if (ts < s)
            i++;
        else
            j--;
    }

    std::vector<int> r = {grp[i], grp[j]};
    return r;
}

//Find sequential positive int arr sum is S
//Core: still use way like guards, move from front to the end
int sum(int s, int e)
{
    return (s + e) * (e - s + 1) / 2;
}

void printSeq(int s, int e)
{
    for (int i = s; i <= e; i++)
        std::cout << " " << i;
    std::cout << std::endl;
}

void findSeqs(int S)
{
    int i = 0, j = 1;

    while (1)
    {
        int r = sum(i, j);
        if (S == r)
        {
           printSeq(i, j);
           if (j < S)
               j++;
           else
               i++;
        }
        else if (S < r)
        {
            if (i < j)
                i++;
        }
        else
        {
            if (j < S)
                j++;
        }

        if (i >= S / 2)
            return;
    }
}

//Move odd before even and keep seq of an int grp
//Core: seq is important meaning that can't just exchange
//like 1324578 can't just swap 2 and 5, has to do a rol before swap.
void rol(std::vector<int> &grp, int s, int e)
{
    int t = grp[e];
    for (int i = e; i > s; i--)
        grp[i] = grp[i - 1]; //rol has to reverse the traverse here
    grp[s] = t;
}

void moveOddFirst(std::vector<int> &grp)
{
    for (int i = 0; i < grp.size(); i++)
    {
        int val = grp[i];
        if (!(val % 2))
        {
            bool findOdd(false);
            int j = i + 1;
            for (; j < grp.size(); j++)
            {
                int val2 = grp[j];
                if (val2 % 2)
                {
                    findOdd = true;
                    break;
                }
            }

            if (!findOdd)
                return;
            else
            {
                rol(grp, i, j);
            }
        }
    }
}

//Faster way(O(n)) but space sacrifice
void moveOddFirst2(std::vector<int> &grp)
{
    std::vector<int> nGrpOdd, nGrpEven;
    for (auto i : grp)
    {
        if (i & 1)
            nGrpOdd.push_back(i);
        else
            nGrpEven.push_back(i);
    }

    nGrpOdd.insert(nGrpOdd.end(), nGrpEven.begin(), nGrpEven.end());
    grp.swap(nGrpOdd);
}

//Return the num that more than half length of arr
//1. sort(o(nlgn)): the middle is the most
//2. o(n): pay attention to the times more than any num else
int most(std::vector<int> &grp)
{
    int cNum(grp[0]), cTimes(0);
    for (auto i : grp)
    {
        if (i == cNum)
            cTimes++;
        else
        {
            cTimes--;
            if (!cTimes)
            {
                cNum = grp[1];
                cTimes++;
            }
        }
    }

    //add checkout
    int count(0);
    for (auto i: grp)
    {
        if (i == cNum)
            count++;
    }

    if (count > grp.size() / 2)
        return cNum;
    else
        return 0;
}

//Find the most sub-seq sum
//Core(O(n)): think left sub-seq whether contribute to
//final(> 0 or not),if not then refresh it
int mostSubSeq(std::vector<int> &grp)
{
    if (grp.size() == 0)
        return 0;
    int total = grp[0];
    int maxSum = grp[0];
    for (int i = 1; i < grp.size(); i++)
    {
        if (total >= 0)
            total += grp[i];
        else
            total = grp[i];
        if (total > maxSum)
            maxSum = total;
    }

    return maxSum;
}

//O(nlogn): Divide method recursively, very classic
int mostSubSeq2(std::vector<int> &grp, int s, int e)
{
    if (s == e)
    {
        if (grp[s] > 0)
            return grp[s];
        else
            return 0;
    }

    int middle = (s + e) / 2;
    int maxLeftSum = mostSubSeq2(grp, s, middle);
    int maxRightSum = mostSubSeq2(grp, middle + 1, e);

    int maxLeftBorderSum = 0, leftBoderSum = 0;
    for (int i = middle; i >= s; i-- )
    {
        leftBoderSum += grp[i];
        if (leftBoderSum > maxLeftBorderSum)
            maxLeftBorderSum = leftBoderSum;
    }

    int maxRightBoderSum = 0, rightBorderSum = 0;
    for (int i = middle + 1; i <= e; i++)
    {
        rightBorderSum += grp[i];
        if (rightBorderSum > maxRightBoderSum)
            maxRightBoderSum = rightBorderSum;
    }

    int maxLeftRightSum = maxLeftBorderSum + maxRightBoderSum;

    int maxSubSum = 0;
    maxSubSum = maxLeftSum > maxRightSum ? maxLeftSum: maxRightSum;
    maxSubSum = maxSubSum > maxLeftRightSum ? maxSubSum: maxLeftRightSum;

    return maxSubSum;
}

//Arrange the num arr to minist num
std::stack<int> weishu(int i)
{
    std::stack<int> q;
    do
    {
        q.push( i % 10);
        i /= 10;
    }while(i);
    return q;
}

bool less(int x, int y)
{
    std::stack<int> qx = weishu(x);
    std::stack<int> qy = weishu(y);

    int last = qx.top();
    while (1)
    {
        if (qx.empty() && qy.empty())
            return false;
        if (qx.empty())
        {
            if (qy.top() > last)
                return true;
            else
                return false;
        }
        if (qy.empty())
        {
            if (qx.top() < last)
                return true;
            else
                return false;
        }
        if (qx.top() == qy.top())
        {
            last == qx.top();
            qx.pop();
            qy.pop();
        }
        else
        {
            if (qx.top() < qy.top())
                return true;
            else
                return false;
        }
    }
}

void printToMinist(std::vector<int> &grp)
{
    std::sort(grp.begin(), grp.end(), less);
    print(grp);
}

//Find all reverse-order pairs
//Like merge-sort, divide-and-conquer

//An example of merge-sort
void mergeSortSubSort(std::vector<int> &grp, int s, int e, int middle)
{
    int leftp = s;
    int rightp = middle + 1;

    std::vector<int> sortedIntArr;
    while (rightp <= e)
    {
        if (leftp <= middle)
        {
            if (grp[leftp] <= grp[rightp])
            {
                sortedIntArr.push_back(grp[leftp]);
                leftp++;
            }
            else
            {
                sortedIntArr.push_back(grp[rightp]);
                rightp++;
            }
        }
        else
        {
            sortedIntArr.push_back(grp[rightp++]);
        }
    }

    //Make sure left division and right division was
    //totally concerned
    while (leftp <= middle)
        sortedIntArr.push_back(grp[leftp++]);

    for (int i = 0; i < sortedIntArr.size(); i++)
    {
        grp[i + s] = sortedIntArr[i];
    }
}

void mergeSort(std::vector<int> &grp, int s, int e)
{
    if (e == s)
        return;
    else if (e - s == 1)
    {
        if (grp[s] > grp[e])
            swap(grp.data() + s, grp.data() + e);
    }
    else
    {
        //Divide
        int middle = (s + e) / 2;
        mergeSort(grp, s, middle);
        mergeSort(grp, middle + 1, e);

        //Conquer
        mergeSortSubSort(grp, s, e, middle);
    }
}

void mergeSortSubSortCount(std::vector<int> &grp, int s, int e, int middle, int& count)
{
    int leftp = s;
    int rightp = middle + 1;

    std::vector<int> sortedIntArr;
    while (rightp <= e)
    {
        if (leftp <= middle)
        {
            if (grp[leftp] <= grp[rightp])
            {
                sortedIntArr.push_back(grp[leftp]);
                leftp++;
            }
            else
            {
                sortedIntArr.push_back(grp[rightp]);
                count += (middle - leftp + 1);
                rightp++;
            }
        }
        else
        {
            sortedIntArr.push_back(grp[rightp++]);
        }
    }

    while (leftp <= middle)
        sortedIntArr.push_back(grp[leftp++]);

    for (int i = 0; i < sortedIntArr.size(); i++)
    {
        grp[i + s] = sortedIntArr[i];
    }
}

void mergeSortCount(std::vector<int> &grp, int s, int e, int &count)
{
    if (e == s)
        return;
    else if (e - s == 1)
    {
        if (grp[s] > grp[e])
        {
            swap(grp.data() + s, grp.data() + e);
            count++;
        }
    }
    else
    {
        //Divide
        int middle = (s + e) / 2;
        mergeSortCount(grp, s, middle, count);
        mergeSortCount(grp, middle + 1, e, count);

        //Conquer
        mergeSortSubSortCount(grp, s, e, middle, count);
    }
}

int findAllReverseOrderPairs(std::vector<int> &grp)
{
    int count = 0;
    mergeSortCount(grp, 0, grp.size() - 1, count);
    return count;
}

//Find how many times a num was showed in a seq arr
//Just find the first index and last index is ok, so d-c way again
int findNumNode(std::vector<int> &grp, int s, int e, int targetNum)
{
    int middle = (s + e) / 2;

    if (grp[middle] == targetNum)
        return middle;
    if (s == e)
        return -1;
    int tmp = -1;
    if (grp[middle] > targetNum)
    {
        tmp = findNumNode(grp, s, middle, targetNum);
        if (tmp != -1)
            return tmp;
    }
    else
    {
        tmp = findNumNode(grp, middle, e, targetNum);
        if (tmp != -1)
            return tmp;
    }
}

int times(std::vector<int> &grp, int targetNum)
{
    int cIndex = findNumNode(grp, 0, grp.size() - 1, targetNum);

    if (cIndex == -1)
        return 0;
    int cLeft = cIndex, cRight = cIndex;
    while (grp[cLeft] == targetNum && cLeft >= 0)
        cLeft--;
    while (grp[cRight] == targetNum && cRight <= grp.size() - 1)
        cRight++;

    return  (--cRight - ++cLeft + 1);
}

//Improvement: use before and after way to find first num an last
int getFirstK(std::vector<int> &array, int k, int start, int end)
{
    if (start > end)
        return -1;
    int midIndex = (start + end) / 2;
    int midData = array[midIndex];
    if (midData == k)
    {
        //Judge whether is the first K
        if (midIndex > 0 && array[midIndex - 1] != k || midIndex == 0)
            return midIndex;
        else
            end = midIndex - 1;
    }
    else if (midData > k)
        end = midIndex - 1;
    else
        start = midIndex + 1;
    return getFirstK(array, k, start, end);
}

int getLastK(std::vector<int> &array, int k, int start, int end)
{
    if (start > end)
        return -1;
    int midIndex = (start + end) / 2;
    int midData = array[midIndex];

    if (midData == k)
    {
        if (midIndex < array.size() && array[midIndex + 1] != k || midIndex == array.size())
            return midIndex;
        else
            start = midIndex + 1;
    }
    else if (midData > k)
    {
        end = midIndex - 1;
    }
    else
    {
        start = midIndex + 1;
    }
    return getLastK(array, k, start, end);
}

int times2(std::vector<int> &grp, int targetNum)
{
    int firstK = getFirstK(grp, targetNum, 0, grp.size() - 1);
    int lastK = getLastK(grp, targetNum, 0, grp.size() - 1);
    return lastK - firstK + 1;
}

//Find val of 1+2+3+...+n , no judge no loop
//Core: use a AND opeartion to avoid n < 0, like recurse stops at n = 0
//Like short current circuit of recursion
int SumSolution(int n)
{
    int result = 0;
    int temp = 0;
    bool flag = (n > 0) && (temp == (result = SumSolution(n - 1)));
    result += n;
    return result;
}

//Sum without 4 operations
int sumWithout4Operations(int a, int b)
{
    int ta(a), tb(b);
    int c1(0x1), c0(0x0);
    int r(0x0);
    bool carryBit = false;
    while (ta != 0 || tb!= 0)
    {
        bool cA = ta & 0x1;
        bool cB = tb & 0x1;

        if ((cA && cB) ||
            (!cA && cB && carryBit) ||
            (cA && !cB && carryBit)   )
        {
            if (cA && cB && carryBit)
                r = r | c1;
            else
                r = r | c0;
            carryBit = true; //When carry-over
        }
        else if (!cA && !cB)
        {
            if (carryBit)
                r = r | c1;
            else
                r = r | c0;
            carryBit = false;
        }
        else
        {
            if (carryBit)
                r = r | c0;
            else
                r = r | c1;
            carryBit = false;
        }

        c1 <<= 1;
        c0 <<= 1;
        ta >>= 1;
        tb >>= 1;
    }

    if (carryBit)
        r = r | c1;

    return r;
}

//Improvement: Simulate like what in decimal
//1. Add each bit without carry-over, and bianry addition equals xor operation(^ in C++ and Java)
//2. Cal carry-over ,then lor 1;
//3. Loop the above two result;
int sumWithout4Operations2(int a, int b)
{
    int sum  = a;
    int carry = 0;
    while (b != 0)
    {
        sum = a ^ b;
        carry = (a & b) << 1;
        a = sum;
        b = carry; //End when there no carry
    }

    return sum;
}

int sumWithout4OperationsRecursive(int a, int b)
{
    if (b != 0)
        return sumWithout4OperationsRecursive(a ^ b, (a & b) << 1);
    else
        return a;
}

//Find the smallest val in a lor arr
int findSmallestInLor(std::vector<int> &lorGrp, int start, int end)
{
    if (start > end)
        return INT_MIN;
    if (start == end)
        return lorGrp[start];
    int midIndex = (start + end) / 2;
    if (lorGrp[midIndex] < lorGrp[end])
    {
        if (lorGrp[midIndex - 1] > lorGrp[midIndex])
            return lorGrp[midIndex];
        else
            return findSmallestInLor(lorGrp, start, midIndex);
    }
    else if (lorGrp[midIndex] > lorGrp[end])
    {
        return findSmallestInLor(lorGrp, midIndex, end);
    }
    else
    {
        return lorGrp[end];
    }
}

//Times of cha '1' has between 1 to n
///Wait to see how to solve
int timesOf1(int n)
{
    int low, cur, temp, i = 1;
    int high = n;    //Record high pos
    int total = 0;   //Times of 1
    if (n < 1)
        return 0;
    while (high != 0)
    {
        high = n / pow(10, i);
        temp = n % int(pow(10, i));
        cur = temp / pow(10, i - 1);
        low = temp / pow(10, i - 1);
        if (cur == 1)
            total += high * pow(10, i - 1) + low + 1;
        else if (cur > 1)
            total += (high + 1) * pow(10, i - 1);
        else
            total += high * pow(10, i - 1);

        i++;
    }
    return total;
}

//Judge straight flush
bool straightFlush(std::vector<int> grp)
{
    std::sort(grp.begin(), grp.end());
    if (grp[grp.size() - 2] == 0)
        return true;
    int sIndex = 0;
    for (; sIndex < grp.size(); sIndex++)
    {
        if (grp[sIndex])
            break;
    }

    int dis = grp[grp.size() - 1] - grp[sIndex];
    if (dis > 4)
        return false;

}



void main()
{
    //02 replace space
    char c[] = "Time flys quickly";
    std::cout << c << std::endl;
    replaceSpace2(&(c[0]), 1000);
    std::cout << c << std::endl;

    //03 print list reversed
    ListNode n1(1), n2(2), n3(3), n4(4);
    n3.next = &n4; n2.next = &n3; n1.next = &n2;
    printListFromTailToHead(&n1);
    printListFromTailToHead2(&n1);

    //04
    std::vector<int> preOrderIntArr = {1,2,4,7,3,5,6,8};
    std::vector<int> inOrderIntArr = {4,7,2,1,5,3,8,6};
    //reBuildBinaryTree(preOrderIntArr, inOrderIntArr);

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
    std::cout << std::endl << "Depth of tree is "<< depth(&b5);

    //calculate sum
    std::cout << std::endl << "Recursive sum is " << sum(3);
    std::cout << std::endl << "Recursive sum is " << sum2(3);

    //print from top
    std::cout << std::endl;
    printFromTopToBottom(&b5);

    //print above in multi lines
    std::cout << std::endl;
    printFromTopToBottomInMultiLines(&b5);

    //print above in a crossed way
    std::cout << std::endl << "Crossed way" << std::endl;
    printFromTopToBottomInMultiLinesCrossed(&b5);

    //get median num
    std::cout << std::endl;
    std::vector<int> maxTree, minTree;
    std::cout << GetMedian(2, maxTree, minTree) << std::endl;
    std::cout << GetMedian(4, maxTree, minTree) << std::endl;
    std::cout << GetMedian(6, maxTree, minTree) << std::endl;
    std::cout << GetMedian(8, maxTree, minTree) << std::endl;

    //find sum path
    TreeNode tr1(1), tr2(2), tr3(3), tr4(1), tr5(4), tr6(3), tr7(4);
    tr1.left = &tr2; tr1.right = &tr3;
    tr2.left = &tr4; tr2.right = &tr5;
    tr3.left = &tr6; tr3.right = &tr7;
    std::vector<std::vector<TreeNode*>> sumPaths = findPaths(&tr1, 7);

    //find if tree B is part of tree A
    TreeNode tb1(3), tb2(3), tb3(2);
    tb1.left = &tb2; tb1.right = &tb3;
    std::cout << (isPartTree(&tr1, &tb1)) ? "true" : "false";

    //mirror the binary tree
    mirror(&tr1);
    mirror(&tr1);

    //test a grp of data is seq of pos-order tree
    std::vector<int> posOrderGrp = {1, 3, 2, 6, 8, 7, 6};
    std::cout << std::endl << isBinarySearchTree(posOrderGrp, 0, posOrderGrp.size() - 1);

    //transfer bianry search tree to list
    //transferBinaryTreeToSortedList(&b5);

    //judge a b-tree is balanced or not
    std::cout << std::endl << isBalanced(&b5);
    int dl(0), dr(0);
    std::cout << std::endl << isBalanced2(&b5, dl);

    //Find next node in in-order traverse
    PTreeNode pt1(1), pt2(2), pt3(3), pt4(4), pt5(5), pt6(6), pt7(7);
    pt1.left = &pt2; pt1.right = &pt3;
    pt2.parent = &pt1; pt3.parent = &pt1;
    pt2.left = &pt4; pt2.right = &pt5;
    pt4.parent = &pt2; pt5.parent = &pt2;
    pt5.right = &pt6; pt6.parent = &pt5;
    std::cout << std::endl << "Next node" << findNextInOrderNode(&pt1, &pt2)->val;
    std::cout << std::endl << "Next node" << findNextInOrderNode(&pt1, &pt4)->val;
    std::cout << std::endl << "Next node" << findNextInOrderNode(&pt1, &pt6)->val;
    std::cout << std::endl << "Next node" << findNextInOrderNode(&pt1, &pt3);

    //Judge a tree whether symmetric
    TreeNode st1(1), st2(2), st3(2), st4(3), st5(4), st6(3), st7(3);
    st1.left = &st2; st1.right = &st3;
    st2.left = &st4; st2.right = &st5;
    st3.left = &st6; st3.right = &st7;
    std::cout << std::endl << "is symmetric " << isSymmetric(&st1);

    //serialization and reserialization
    char *str = Serialize(&st1);
    TreeNode *stroot = Deserialize(str);
    if (str)
    {
        delete str;
        str = nullptr;
    }
    release(stroot);

    //judge a string whether a valid numeric val
    std::string str1("-3.3"), str2("+9.0E30"), str3("3.30a");
    std::cout << std::endl << isNumeric(str1) << " "
              << isNumeric(str2) << " "
              << isNumeric(str3) << " ";

    //find the first no repetition character
    char strn[] = "student. a am I";
    std::cout << std::endl << "First valid no rep cha is " << findStr(strn);

    //reverse characters
    std::cout << std::endl << "Before reverse "<< strn;
    reverse(strn); reverseWords(strn);
    std::cout << std::endl << "After reverse " << strn;

    //rol of str
    rol(strn, 5);
    std::cout << std::endl << "result of rol" << strn;

    //str to int
    std::cout << std::endl << "Str to int is " << toInt(std::string("-3209"));

    //full permutation
    char strp[] = "aba";
    std::cout << std::endl << "Full permutation 1 " << std::endl;
    permute1(strp);
    std::cout << std::endl << "Full permutation " << std::endl;
    permute(strp, 0, strlen(strp) - 1);

    //find repeated num
    int intArr[] = {0, 2, 3, 2, 1};
    std::cout << std::endl << "Find first repeated num " << firstRepeatedNum(intArr, sizeof(intArr)/sizeof(int));
    std::cout << std::endl << "Find first repeated num in no group way " << firstRepeatedNum2(intArr, sizeof(intArr)/sizeof(int));

    //cal multiply of vector grp
    std::vector<int> grpA = {1, 2, 3, 4};
    std::vector<int> grpB = genProduct(grpA);
    std::cout << std::endl << "The result of multiply ";
    for (auto i : grpB)
    {
        std::cout << " " << i;
    }

    //find once showed num
    std::vector<int> grpC = {1, 2, 3, 1, 3};
    std::cout << std::endl << "First once showed " << findOnceShowed(grpC);

    //find two once showed num
    std::vector<int> grpD = {1, 2, 3, 1};
    int iF, iS;
    findOnceShowed(grpD, iF, iS);
    std::cout << std::endl << "Two once showed" << iF << " and " << iS;

    //find two sum is s in a grp
    std::vector<int> grpE = {1, 2, 3, 4, 5, 6, 7};
    std::vector<int> grpEr = findTwoNumS(grpE, 8);
    std::cout << std::endl << "Find Two sum in group is " << grpEr[0] << " and " << grpEr[1];

    //Find full permutation of sum
    int sum1 = 200;
    std::cout << std::endl << "Full permutation of sum " << sum1 << std::endl;
    findSeqs(sum1);

    //Move odd to left and keep seq
    std::vector<int> grpH = {1, 3, 2, 4, 5, 7, 8, 9, 9, 10, 12};
    moveOddFirst2(grpH);
    print(grpH);

    //Find the most num in grp
    std::vector<int> grpI = {1, 2, 3, 2, 4, 2, 2, 2, 2, 2, 5, 6, 7, 8, 9};
    std::cout << std::endl << " The most in grp " << " is " << most(grpI);

    //Find the most sub-seq in a seq
    std::vector<int> grpJ = {6,-3,-2,7,-15,1,2,2};
    std::cout << std::endl << "The most sub-seq sum is " << mostSubSeq(grpJ);
    std::cout << std::endl << "The most sub-seq sum is " << mostSubSeq2(grpJ, 0, grpJ.size() - 1);

    //Print the most minist arrangment of grp
    std::vector<int> grpK = {3,32,321};
    std::cout << std::endl << "The minist arrangement is ";
    printToMinist(grpK);

    //merge-sort
    std::vector<int> grpL = {6, 2, 1, 4, 3, 8, 7, 5};
    mergeSort(grpL, 0, grpL.size() - 1);
    std::cout << std::endl << "Merge sort of the grp is ";
    print(grpL);

    //find reversed-pairs number
    std::vector<int> grpM = {1, 2, -1, 3, 0};
    std::cout << std::endl << "All reversed-pairs number is " << findAllReverseOrderPairs(grpM);

    //find times of repeated num in a seq arr
    std::vector<int> grpN = {1, 2, 2, 2, 2, 2, 3, 3, 4};
    std::cout << std::endl << "Repeat times of 2 is " << times(grpN, 2);
    std::cout << std::endl << "Repeat times of 2 is " << times2(grpN, 2);

    //Sum of 1 to n
    std::cout << std::endl << "Sum of 1 to 100 is " << SumSolution(100);

    //Sum with no 4 opeartions
    std::cout << std::endl << "Sum without 4 opeartions is " << sumWithout4Operations(18, 263);
    std::cout << std::endl << "Sum without 4 operations2 is " << sumWithout4Operations2(7, 5);
    std::cout << std::endl << "Sum without 4 operations2 is " << sumWithout4OperationsRecursive(117, 125);

    //Find smallest val in lor array
    std::vector<int> grpO = {61, 2, 3, 4, 5, 6};
    std::cout << std::endl << "The result of LOR array is " << findSmallestInLor(grpO, 0, grpO.size() - 1);

    //Times of 1 in range 1 to n
    std::cout << std::endl << "The result of 1 chars in 1 to 2593 is " << timesOf1(2593);

    std::cout << std::endl << std::endl << std::endl;
    return;
}
