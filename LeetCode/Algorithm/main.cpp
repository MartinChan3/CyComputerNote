#include <string>
#include <iostream>
#include <vector>
#include <stack>
#include <algorithm>
#include <map>
#include <queue>
#include <functional>
#include <unordered_map>

using namespace std;

struct ListNode {
    int val;
    ListNode *next;
    ListNode(int x) : val(x), next(NULL) {}
};

void print(vector<pair<int, int>> vec) {
    for (auto a : vec) {
        cout << a.first << ' ' << a.second<<" | ";
    }
    cout << endl;
}

template<typename T>
void print(vector<T> vec) {
    for (auto a : vec) {
        cout << a << ' ';
    }
    cout << endl;
}

template<typename T>
void print(vector<vector<T>> vec) {
    for (const auto &v: vec) {
        print(v);
    }
    cout << endl;
}

class Solution {
public:
    //005 Find the longest Palindrome
    string longestPalindrome(string s) {
        int len=s.size();
        if(len==0||len==1)
            return s;
        int start=0;//回文串起始位置
        int max=1;//回文串最大长度
        vector<vector<int>>  dp(len,vector<int>(len));//定义二维动态数组
        for(int i=0;i<len;i++)//初始化状态
        {
            dp[i][i]=1;
            if(i<len-1&&s[i]==s[i+1])
            {
                dp[i][i+1]=1;
                max=2;
                start=i;
            }
        }
        for(int l=3;l <= len;l++)//l表示检索的子串长度，等于3表示先检索长度为3的子串
        {
            for(int i=0;i+l-1<len;i++)
            {
                int j=l+i-1;//终止字符位置
                if(s[i]==s[j]&&dp[i+1][j-1]==1)//状态转移
                {
                    dp[i][j]=1;
                    start=i;
                    max=l;
                }
            }
        }
        return s.substr(start,max);//获取最长回文子串
    }

    //006 z-reverse output
    string convert(string s, int numRows) {
        if (numRows == 1) return s;
        string ret;
        int n = s.size();
        int cycleLen = 2 * numRows - 2;

        for (int i = 0; i < numRows; i++) {
            for (int j = 0; j + i < n; j += cycleLen) {
                ret += s[j + i];
                if (i != 0 && i != numRows - 1 && j + cycleLen - i < n)
                    ret += s[j + cycleLen - i];
            }
        }
        return ret;
    }

    //007 int reverse
    //关键在于考虑溢出内容
    int reverse(int x)
    {
        int rev = 0;
        while (x != 0) {
            int pop = x % 10;
            x /= 10;
            if (rev > INT_MAX/10 || (rev == INT_MAX / 10 && pop > 7)) return 0;
            if (rev < INT_MIN/10 || (rev == INT_MIN / 10 && pop < -8)) return 0;
            rev = rev * 10 + pop;
        }
        return rev;
    }

    //008 Atoi
    int myAtoi(string str) {
        unsigned long len = str.length();

        // 去除前导空格
        int index = 0;
        while (index < len) {
            if (str[index] != ' ') {
                break;
            }
            index++;
        }

        if (index == len) {
            return 0;
        }

        int sign = 1;
        // 处理第 1 个非空字符为正负符号，这两个判断需要写在一起
        if (str[index] == '+') {
            index++;
        } else if (str[index] == '-') {
            sign = -1;
            index++;
        }

        int res = 0;
        while (index < len) {
            char curChar = str[index];
            if (curChar < '0' || curChar > '9') {
                break;
            }

            //最重要的就是在这里，需要先判断
            if (res > INT_MAX / 10 || (res == INT_MAX / 10 && (curChar - '0') > INT_MAX % 10)) {
                return INT_MAX;
            }
            if (res < INT_MIN / 10 || (res == INT_MIN / 10 && (curChar - '0') > -(INT_MIN % 10))) {
                return INT_MIN;
            }

            res = res * 10 + sign * (curChar - '0');
            index++;
        }
        return res;
    }

    //009 Judeges whether palindrome
    bool isPalindromeSelf(int x) {
        if (x < 0) return false; //负数不为回文数
        vector<int> grp;
        while (x > 0)
        {
            grp.push_back(x % 10);
            x /= 10;
        }
        if (grp.size() <= 1)  return true;
        for (int i = 0; i <= grp.size() / 2; i++)
            if (grp.at(i) != grp.at(grp.size() - 1 -i)) return false;
        return true;
    }

    //    拿x的前一半与后一半对比。
    //    若x为0-9, true
    //    若x最末位为0, 即 x%10 = 0, false
    //    如果x<0, false
    //    若x位数为偶, 则若前后半段相等 true
    //    若x位数为奇, 则前后半段若较长数/10 = 较短数, true
    //    例 x = 12321, 则分为 123和12, 较长数的末位为对称轴
    bool isPalindrome(int x) {
        if (x < 10 && x >= 0) return true;
        if (x < 0 || x % 10 == 0) return false;
        int val = 0;
        for (; val * 10 <= x; x /= 10) //构造x的前半段与后半段, 前半段为x, 后半段为val
            val = val * 10 + x % 10;
        return (val == x || (val > 9 && val / 10 == x));
    }

    //010 Expression match(classic dynamic programming)
    bool isMatch(string s, string p) {
        if (p.empty()) return s.empty();
        auto first_match = !s.empty() && (s[0] == p[0] || p[0] == '.');
        if (p.length() >= 2 && p[1] == '*') {
            return isMatch(s, p.substr(2)) || (first_match && isMatch(s.substr(1), p));
        } else {
            return first_match && isMatch(s.substr(1), p.substr(1));
        }
    }

    //011 max area of bucket
    int maxAreaSelf(vector<int>& height) {
        unsigned int size = height.size();
        int max = 0;
        for (int width = 1; width <= size - 1; width++)
        {
            for (int startPos = 0; startPos + width < size; startPos++)
            {
                int shorterSize = height[startPos] < height[startPos + width] ?
                            height[startPos] : height[startPos + width];
                if (shorterSize * width > max)
                    max = shorterSize * width;
            }
        }
        return max;
    }

    //分冶算法
    void combination(char* ptr, int n, vector<char> &result)
    {
        if (!ptr) return;

        if (!n)
        {
            auto iter = result.begin();
            for (; iter != result.end(); ++iter)
                cout << *iter;
            cout << endl;
            return;
        }

        if (*ptr == '\0') return;
        result.push_back(*ptr);
        combination(ptr + 1, n - 1, result);  //若把第一个字符放到组合中去，则需要在剩下n-1个字符中选取m-1个字符
        result.pop_back();
        combination(ptr + 1, n, result);
    }

    void combination(char* ptr)
    {
        if (!ptr) return;
        vector<char> result;
        int i, length = strlen(ptr);
        for (i = 1; i <= length; i++)
            combination(ptr, i, result);
    }

    //双指针法
    int maxArea(vector<int>& height) {
        int res = 0;
        int i = 0;
        int j = height.size() - 1;
        while (i < j) {
            int area = (j - i) * min(height[i], height[j]);
            res = max(res, area);
            if (height[i] < height[j]) { //只去动短的一根：因为如果只动长的一根，那么另一边一定不长于先前短的那一边
                i++;
            } else {
                j--;
            }
        }
        return res;
    }

    //012 Int to roman
    string singleToRoman(int s, int place){
        char base1, base5, base10;
        string sRes;
        switch (place)
        {
        case 1: base1 = 'I'; base5 = 'V'; base10 = 'X'; break;
        case 2: base1 = 'X'; base5 = 'L'; base10 = 'C'; break;
        case 3: base1 = 'C'; base5 = 'D'; base10 = 'M'; break;
        case 4: base1 = 'M'; break;
        }

        if (s <= 3)
        {
            while (s--)
                sRes.push_back(base1);
        }
        else if (s < 9)
        {
            int pos = s - 5;
            if (pos < 0)
                sRes.insert(sRes.begin(), base1);
            sRes.push_back(base5);
            while (pos-- > 0)
                sRes.push_back(base1);
        }
        else
        {
            sRes.push_back(base1);
            sRes.push_back(base10);
        }

        return sRes;
    }

    string intToRoman(int num) {
        if (num >= 4000) return string("Above Range");
        string res;
        int x, place = 1;
        while (num)
        {
            x = num % 10;
            num /= 10;
            auto inserted = singleToRoman(x, place++);
            res.insert(res.begin(), inserted.begin(), inserted.end());
        }
        return res;
    }

    //Greed algorithm——always think num is big
    string intToRoman2(int num) {
        int values[] = {1000, 900, 500, 400, 100, 90, 50, 40, 10, 9, 5, 4, 1};
        string reps[] = {"M", "CM", "D", "CD", "C", "XC", "L", "XL", "X", "IX", "V", "IV", "I"};
        string res;
        for (int i = 0; i < 13; i ++ )
            while(num >= values[i])
            {
                num -= values[i];
                res += reps[i];
            }
        return res;
    }

    //013 Roman to int
    int startWith(string s, string sub){
        return s.find(sub) == 0? 1: 0;
    }

    int romanToIntSelf(string s) {
        int values[] = {1000, 900, 500, 400, 100, 90, 50, 40, 10, 9, 5, 4, 1};
        string reps[] = {"M", "CM", "D", "CD", "C", "XC", "L", "XL", "X", "IX", "V", "IV", "I"};
        int res = 0;
        for (int i = 0; i < 13; i++)
            while (true)
            {
                if (startWith(s, reps[i]))
                {
                    res += values[i];
                    s = s.substr(reps[i].size());
                }
                else
                    break;
            }
        return res;
    }

    //Hash-map way
    int romanToInt(string s) {
        int result=0;
        map<char,int> luomab;
        luomab.insert(map<char,int>::value_type('I',1));
        luomab.insert(map<char,int>::value_type('V',5));
        luomab.insert(map<char,int>::value_type('X',10));
        luomab.insert(map<char,int>::value_type('L',50));
        luomab.insert(map<char,int>::value_type('C',100));
        luomab.insert(map<char,int>::value_type('D',500));
        luomab.insert(map<char,int>::value_type('M',1000));
        for(int i = 0; i < s.length(); i++)
        {
            if(luomab[s[i]] < luomab[s[i+1]])
                result -= luomab[s[i]];  //当前位置的值比后一位位置值小，说明为负
            else
                result += luomab[s[i]];
        }
        return result;
    }

    //014 Find the longest common prefix
    string longestCommonPrefix(vector<string>& strs) {
        if (strs.size() == 0) return string("");
        string ministStr = strs.at(0);
        for (int i = 1; i < strs.size(); i++)
            if (ministStr.size() > strs.at(i).size())
                ministStr = strs.at(i);
        while(ministStr.size())
        {
            bool allPassed = true;
            for (int i = 0; i < strs.size(); i++)
                if (!startWith(strs.at(i), ministStr))
                {
                    allPassed = false;
                    break;
                }
            if (allPassed) break;
            ministStr = ministStr.substr(0, ministStr.size() - 1);
        }
        return ministStr;
    }

    //015 Three int sum to zero
    vector<vector<int>> threeSumSelf(vector<int>& nums){  //Two much time...
        vector<vector<int>> res;
        if (nums.size() < 3) return res;

        sort(nums.begin(), nums.end());
        int zeroPos = 0;
        while (zeroPos < nums.size())
            if (nums.at(zeroPos++) > 0)
                break;
        unsigned int i = 0, j, k, size = nums.size();
        for (; i < zeroPos; i++)
        {
            j = size - 1; k = i + 1;
            while (k < j)
            {
                auto sRes = nums.at(i) + nums.at(k) + nums.at(j);
                if (sRes == 0)
                {
                    auto sGrp = vector<int>{nums.at(i), nums.at(k), nums.at(j)};
                    if (!(i > 0 && (nums.at(i) == nums.at(i - 1))) && !(j < size - 1 && (nums.at(j) == nums.at(j + 1))))
                        res.push_back(sGrp);
                    ++k; --j;
                }
                else if (sRes < 0)
                    ++k;
                else
                    --j;
            }
        }
        return res;
    }

    vector<vector<int> > threeSum(vector<int>& nums) { //核心是如何排除重复数字？
        vector<vector<int> > ret;
        int len = nums.size();
        sort(nums.begin(),nums.end());//sort the input
        for(int i=0;i<len-2;i++){
            //find the tripe for each nums[i]
            // j1 and j2 log the index of the other two numbers
            if(i ==0 ||(i>0 && nums[i] != nums[i-1])){
                int p1 = i+1, p2 = len-1; // set two pointers
                while(p1 < p2){
                    if(nums[p1] + nums[p2] < -nums[i]){
                        p1++;
                    }else if(nums[p1] + nums[p2] == -nums[i]){
                        if(p1 == i+1){
                            vector<int > vtemp{nums[i], nums[p1], nums[p2]};
                            ret.push_back(vtemp);
                            vtemp.clear();
                        }else if(nums[p1] != nums[p1-1]){
                            vector<int > vtemp{nums[i], nums[p1], nums[p2]};
                            ret.push_back(vtemp);
                            vtemp.clear();
                        }
                        p1++,p2--;
                    }else{
                        p2--;
                    }
                }
            }
        }
        return ret;
    }

    //016 Find the most three numbers closest to target
    int threeSumClosest(vector<int>& nums, int target) {
        sort(nums.begin(), nums.end());
        int ans = nums[0] + nums[1] + nums[2];
        for(int i = 0; i < nums.size(); i++) {
            int st = i + 1, ed = nums.size() - 1;
            while( st < ed) {
                int sum = nums[st] + nums[ed] + nums[i];
                if(abs(target - sum) < abs(target - ans)) ans = sum;
                if(sum == target) return ans;
                else if(sum < target) st++;
                else ed--;
            }
        }
        return ans;
    }

    //017 List all possible letter combinations
    void getTotalString(string cStr, vector<string> &cRes, const map<int, string> &table){
        if (!cStr.size()) return;

        char cChar = cStr.at(0);
        int tNum = cChar - '0';
        string possibleStr = table.at(tNum);
        vector<string> tRes;
        if (cRes.size() == 0)
        {
            for (int j = 0; j < possibleStr.size(); j++)
                tRes.push_back(possibleStr.substr(j, 1));
        }
        else
        {
            for (int i = 0; i < cRes.size(); i++)
            {
                for (int j = 0; j < possibleStr.size(); j++)
                {
                    string tNew(cRes.at(i));
                    tNew.push_back(possibleStr.at(j));
                    tRes.push_back(tNew);
                }
            }
        }
        cRes.swap(tRes);
        if (cStr.size())
            getTotalString(cStr.substr(1), cRes, table);
    }

    vector<string> letterCombinations(string digits) {
        map<int, string> buttons;
        buttons[2] = "abc"; buttons[3] = "def"; buttons[4] = "ghi";
        buttons[5] = "jkl"; buttons[6] = "mno"; buttons[7] = "pqrs";
        buttons[8] = "tuv"; buttons[9] = "wxyz";
        vector<string> res;
        getTotalString(digits, res, buttons);
        return res;
    }

    //018 Four nums sum to zero
    vector<vector<int>> fourSum1(vector<int>& nums, int target) {
        sort(nums.begin(), nums.end());
        vector<vector<int> > res;
        if(nums.size()<4)
            return res;
        int a, b, c, d, _size = nums.size();
        for(a = 0; a <= _size-4; a++){
            if (nums[a] * 4 > target) break;
            if (a > 0 && nums[a] == nums[a-1]) continue;               //确保nums[a] 改变了
            for (b = a + 1; b <=_size - 3; b++){
                if (b > a + 1 && nums[b] == nums[b - 1]) continue;     //确保nums[b] 改变了
                c = b + 1,d = _size - 1;
                while(c < d){
                    if(nums[a] + nums[b] + nums[c] + nums[d] < target)
                        c++;
                    else if(nums[a] + nums[b] + nums[c] + nums[d] > target)
                        d--;
                    else{
                        res.push_back({nums[a], nums[b], nums[c] ,nums[d]});
                        while(c < d && nums[c + 1] == nums[c])                //确保nums[c] 改变了
                            c++;
                        while(c < d&& nums[d - 1] == nums[d])                //确保nums[d] 改变了
                            d--;
                        c++;
                        d--;
                    }
                }
            }
        }
        return res;
    }

    vector<vector<int>> fourSum2(vector<int>& nums, int target) {            //High-speed way，本质其实就是多一些比较
        if (nums.size() < 4) return {};
        sort(nums.begin(), nums.end());
        int N = nums.size();
        int maxSum3 = nums[N - 3] + nums[N - 2] + nums[N - 1];
        int maxSum2 = nums[N - 2] + nums[N - 1];
        vector<vector<int> > res;
        for (int i = 0; i < N - 3; ++i) {
            if (4 * nums[i] > target) break;
            if (i > 0 && nums[i] == nums[i - 1]) continue;
            if (nums[i] + maxSum3 < target) continue;
            for (int j = i + 1;j < N - 2; ++j) {
                if (2 * (nums[i] + nums[j]) > target) break;
                if (j > i + 1 && nums[j - 1] == nums[j]) continue;
                if (nums[i] + nums[j] + maxSum2 < target) continue;
                int t = target - nums[i] - nums[j];
                int l = j + 1;
                int r = N - 1;
                while (l < r) {
                    if (nums[l] + nums[r] > t) {
                        --r;
                    } else if (nums[l] + nums[r] < t) {
                        ++l;
                    } else {
                        res.push_back({nums[i], nums[j], nums[l], nums[r]});
                        while (l < r && nums[l] == nums[++l]);                 //这个写法非常有意思
                        while (l < r && nums[r] == nums[--r]);
                    }
                }
            }
        }
        return res;
    }

    ListNode* removeNth(ListNode* head, int n) {
        if (!n)
        {
            auto first = head->next;
            delete head;
            return first;
        }

        auto t = head;
        while (--n && t->next)
            t = t->next;
        if (n) return NULL;  //Over-range
        auto target = t->next;
        t->next = target->next;
        target->next = NULL;
        delete target;
        return head;
    }

    //019 remove node and return head
    ListNode* removeNthFromEndSelf(ListNode* head, int n) {
        int size = 1;
        auto cNode = head;
        while (cNode = cNode->next)
            size++;
        if (n > size) return NULL;
        if (n == size)
        {
            auto tHead = head->next;
            head->next = NULL;
            delete head;
            return tHead;
        }

        int wCount = size - n;
        auto tNode = head;
        while (--wCount)
            tNode = tNode->next;
        auto targetNode = tNode->next;
        tNode->next = targetNode->next;
        targetNode->next = NULL;
        delete targetNode;
        return head;
    }

    //Recursive
    ListNode* removeNthFromEnd(ListNode* head, int n) {
        ListNode *start = new ListNode(0);                                //这个方法也很值得借鉴，在开头附加一个额外节点，这样不用考虑特殊情况，非常巧妙
        start->next = head;
        foo(start, n);
        return start->next;
    }

    int foo (ListNode* l, int n){
        if(l->next == NULL) return 1;
        int f = foo(l->next, n);                                           //这两步之前其实进行了回溯，从最底层向上回溯，这样避免了单纯的去计算size
        if(f == n)
            l->next = l->next->next;
        return f + 1;
    }

    //020 Judege the brakets valid
    bool isValid(string s) {
        stack<char> st;
        map<char, char> bracketsTable;
        bracketsTable[')'] = '(';
        bracketsTable[']'] = '[';
        bracketsTable['}'] = '{';
        auto sIt = s.cbegin();
        while (sIt != s.cend())
        {
            if (bracketsTable.end() == bracketsTable.find(*sIt))
                st.push(*sIt);
            else
            {
                if (!st.size() || st.top() != bracketsTable[*sIt])
                    return false;
                else
                    st.pop();
            }
            ++sIt;
        }

        if (st.size())
            return false;
        return true;
    }

    //021 Merge two increasing node list
    ListNode* mergeTwoLists(ListNode* l1, ListNode* l2) {
        auto vSt = new ListNode(0);
        ListNode *l1C, *l2C, *cNode = vSt;
        l1C = l1; l2C = l2;
        while (!(l1C == NULL && l2C == NULL))
        {
            if (l1C == NULL)
            {
                cNode->next = l2C;
                l2C = l2C->next;
            }
            else if (l2C == NULL)
            {
                cNode->next = l1C;
                l1C = l1C->next;
            }
            else
            {
                if (l1C->val > l2C->val)
                {
                    cNode->next = l2C;
                    l2C = l2C->next;
                }
                else
                {
                    cNode->next = l1C;
                    l1C = l1C->next;
                }
            }
            cNode = cNode->next;
        }
        return vSt->next;
    }

    //022 Brackets generation
    void fillThesis(int leftCount, int rightCount, vector<string> &res){
        if (!leftCount && !rightCount) return;

        auto it = res.begin();
        if (leftCount == rightCount)
        {
            if (!res.size())
                res.push_back("(");
            else
                while (res.end() != it)
                    (*it++).push_back('(');
            fillThesis(--leftCount, rightCount, res);
        }
        else
        {
            if (!leftCount)
            {
                while (res.end() != it)
                    (*it++).push_back(')');
                fillThesis(leftCount, --rightCount, res);
            }
            else
            {
                vector<string> tNewLeft = res, tNewRight = res;
                for (auto itLeft = tNewLeft.begin(); itLeft != tNewLeft.end(); itLeft++)
                    (*itLeft).push_back('(');
                fillThesis(leftCount - 1, rightCount, tNewLeft);

                for (auto itRight = tNewRight.begin(); itRight != tNewRight.end(); itRight++)
                    (*itRight).push_back(')');
                fillThesis(leftCount, rightCount - 1, tNewRight);

                tNewLeft.insert(tNewLeft.end(), tNewRight.begin(), tNewRight.end());
                res.swap(tNewLeft);
            }
        }
    }

    vector<string> generateParenthesis(int n) {                                //A little bit slow
        vector<string> res;
        fillThesis(n, n, res);
        return res;
    }

    //023 Merge multiple lists
    ListNode* mergeKLists(vector<ListNode*>& lists) {
        priority_queue<pair<int,ListNode*>,vector<pair<int,ListNode*>>,greater<pair<int,ListNode*>>> m;   //利用priority_queue来保证
        for(auto x:lists){
            if(x){
                m.push(make_pair(x->val,x));
            }
        }
        ListNode* root  = new ListNode(-1);
        auto q = root;
        while(!m.empty()){
            auto p = m.top();
            m.pop();
            q->next = p.second;
            q = q->next;
            if(p.second->next){
                m.push(make_pair(p.second->next->val, p.second->next));
            }
        }
        return root->next;
    }

    //030 (what a search engine usually could encounter) sliding window way to solve
    vector<int> findSubstring(string s, vector<string>& words)
    {
        if (words.size() == 0) return {};
        unordered_map<string, int> wordcnt;
        for (auto& w : words)
        {
            wordcnt[w]++; //Record the possible combination
        }
        int len = words[0].size();

        vector<int> ans;
        for (int i = 0; i < len; i++)        //核心：比较开始每固定位数是否在两个HashMap中都含有，同时计数
        {
            int left = i;
            int right = left;
            int cnt = 0;

            unordered_map<string, int> window;
            while (left + words.size() * len <= s.size())
            {
                string temp = "";
                while (cnt < words.size())
                {
                    temp = s.substr(right, len);
                    if (wordcnt.find(temp) == wordcnt.end() || window[temp] >= wordcnt[temp])
                        break;
                    window[temp]++;
                    cnt++;
                    right += len;
                }

                if (window == wordcnt)
                {
                    ans.push_back(left);
                }

                if (wordcnt.find(temp) != wordcnt.end())
                {
                    window[s.substr(left, len)]--;
                    cnt--;
                    left += len;
                }
                else
                {
                    right += len;
                    left = right;
                    cnt = 0;
                    window.clear();
                }
            }
        }
        return ans;
    }

    //031 Use the next permutation
    void nextPermutation(vector<int>& nums) {
        int pos = nums.size() - 1;
        while (pos > 0 && nums[pos] <= nums[pos - 1])
            pos--;
        std::reverse(nums.begin() + pos, nums.end());  //逆序
        if (pos > 0){
            int start = pos;
            for (; start < nums.size(); start++){ //寻找第一个大于nums[pos - 1]的数
                if (nums[start] > nums[pos - 1]){
                    swap(nums[start], nums[pos - 1]); //交换
                    break;
                }
            }
        }
    }

    void nextPermutationSelf(vector<int> &nums) {                        //标准字典序排法
        int oriPos = nums.size() - 1;
        while (oriPos > 0 && nums.at(oriPos) <= nums.at(oriPos - 1))
            oriPos--;
        if (!oriPos)
        {
            std::sort(nums.begin(), nums.end());
            return;
        }
        --oriPos;
        int sOriPos = nums.size() - 1;
        while (sOriPos > 0 && nums.at(oriPos) >= nums.at(sOriPos))
            sOriPos--;
        swap(nums[oriPos], nums[sOriPos]);
        std::sort(nums.begin() + oriPos + 1, nums.end());
    }

    //032 O(n^2)way to solve
    bool isVaild(string s) {
        stack<char> a;
        for (auto v : s) {
            if (v == '(') {
                a.push(v);
            } else if (!a.empty() && a.top() == '(') {
                a.pop();
            } else return false;
        }
        return a.empty();
    }

    int longestValidParentheses(string s) {
        int maxLen = 0;
        for (int i = 0; i < s.length(); i++) {
            for (int j = 2; j + i <= s.length(); j += 2) {
                if (isVaild(s.substr(i, j))) {
                    if (j > maxLen) {
                        maxLen = j;
                    }
                }
            }
        }
        return maxLen;
    }

    //Another example to show dynamic programming
    int longestValidParentheses2(string s) {    //O(n) way to solve
        int maxLen = 0;
        vector<int> dp(s.length());                                                                  //存储每一位截止有效的位数
        for (int i = 1; i < s.length(); i++) {
            if (s[i] == ')') {
                if (s[i - 1] == '(') {
                    dp[i] = (i - 2 >= 0 ? dp[i - 2] : 0) + 2;                                        //如果前一位为（，则直接+2
                } else if (i - dp[i - 1] - 1 >= 0 && s[i - dp[i - 1] - 1] == '(') {                  //如果还是），则
                    dp[i] = (i - dp[i - 1] - 2 >= 0 ? dp[i - dp[i - 1] - 2] : 0) + dp[i - 1] + 2;    //其实还是先前全部有效内容叠加，其中dp[i - dp[i-1] - 2]代指前一个(之前全部有效指标
                }
            }
            maxLen = max(dp[i], maxLen);
        }
        return maxLen;
    }

    //033 Search in rotated sort array
    int search(vector<int>& nums, int target) {
        int left = 0;
        int right = nums.size() - 1;
        while (left <= right)
        {
            int mid = left + ((right - left) >> 1);              //快速计算平均值
            if (target == nums[mid])
                return mid;
            if (nums[mid] >= nums[left]) {                       //1. 说明左半部分递增;2. 只有两个数的情况下会有mid=left
                if (target >= nums[left] && target < nums[mid])
                    right = mid - 1;
                else
                    left = mid + 1;
            }
            else                                                 //说明右半部分单调递增
            {
                if (target > nums[mid] && target <= nums[right])
                    left = mid + 1;
                else
                    right = mid - 1;
            }
        }
        return -1;
    }

    int search2(vector<int>& nums, int target) {
        int lo = 0, hi = nums.size() - 1;
        while (lo < hi) {
            int mid = (lo + hi) / 2;
            if ((nums[0] > target) ^ (nums[0] > nums[mid]) ^ (target > nums[mid]))
                lo = mid + 1;
            else
                hi = mid;
        }
        return lo == hi && nums[lo] == target ? lo : -1;
    }

    //034
    vector<int> searchRange(vector<int>& nums, int target) {
        vector<int> count;
        int left = 0, right = nums.size() - 1;
        while (left <= right)
        {
            int mid = left + ((right - left) >> 1);
            if (nums[left] > target || nums[right] < target)
                break;
            if (nums[mid] > target)
                right = mid - 1;
            else if (nums[mid] < target)
                left = mid + 1;
            else
            {
                int st = mid, end = mid;
                while (st >= left && target == nums[st])
                    st--;
                while (end <= right && target == nums[end] )
                    end++;
                count.push_back(++st);
                count.push_back(--end);
                return count;
            }
        }
        return vector<int>{-1, -1};
    }

    //035
    int searchInsert(vector<int>& nums, int target) {
        int left = 0, right = nums.size() - 1;
        while (left <= right)
        {
            if (target < nums[left])
                return left;
            if (target > nums[right])
                return right + 1;
            int mid = left + ((right - left) >> 1);
            if (nums[mid] > target)
                right = mid - 1;
            else if (nums[mid] < target)
                left = mid + 1;
            else
                return mid;
        }
        return left;
    }

    //036
    bool checkValid(const map<char, int> &tMap) {
        for (auto t : tMap)
            if (t.first != '.' && t.second > 1)
                return false;
        return true;
    }

    bool isValidSudoku(vector<vector<char>>& board) {   //One-time traversal way
        vector<map<char, int> > colVec(9), rowVec(9), gridVec(9);
        for (int i = 0; i < 9; i++)
        {
            for (int j = 0; j < 9; j++)
            {
                auto cVal = board[i][j];
                ++((colVec[i])[cVal]);
                ++((rowVec[j])[cVal]);
                int indexOfGrid = i / 3 * 3 + j / 3;
                ++((gridVec[indexOfGrid])[cVal]);
            }
        }

        for (auto s: colVec)
            if (!checkValid(s))
                return false;
        for (auto s: rowVec)
            if (!checkValid(s))
                return false;
        for (auto s: gridVec)
            if (!checkValid(s))
                return false;

        return true;
    }

    bool isValidSudoku2(vector<vector<char>>& board) {
        int xsp[9][9] = {0};  //row means which number, col means which row/col/grid, each one means the count of which number
        int ysp[9][9] = {0};
        int lsp[9][9] = {0};
        for(int i = 0; i < 9; i++)
        {
            for(int j = 0; j < 9; j++)
            {
                if(board[i][j] != '.')
                {
                    if(( ++xsp[board[i][j]-'1'][i] > 1) ||
                       ( ++ysp[board[i][j]-'1'][j] >1) ||
                       ( ++lsp[board[i][j]-'1'][(i / 3) * 3 + j / 3] >1))   //Core way to save time——add and judge at same time
                        return false;
                }
            }
        }
        return true;
    }

    //037
    void solveSudoku(vector<vector<char>>& board) {
        backtrack(board,0,0);
    }

    bool backtrack(vector<vector<char>>& board, int row, int col){
        if(col == 9)
            return backtrack(board, row + 1, 0);

        if(row == 9)
            return true;

        for(int i = row; i < 9; i++){
            for(int j = col; j < 9; j++){
                if(board[i][j] != '.'){
                    return backtrack(board, i, j + 1);
                }
                for(char ch = '1'; ch <= '9'; ch++){
                    if(!isValid(board, i, j, ch))
                        continue;
                    board[i][j] = ch;
                    if(backtrack(board, i, j + 1))
                        return true;
                    board[i][j] = '.';
                }
                return false;

            }
        }
        return false;
    }

    bool isValid(vector<vector<char>>& board, int row, int col, char ch){
        for(int i = 0; i < 9; i++){
            if(board[row][i] == ch) return false;
            if(board[i][col] == ch) return false;
            if(board[(row/3)*3 + i/3][(col/3)*3 + i%3] == ch) return false;
        }
        return true;
    }

    //024 Swap pairs
    ListNode* swapPairs(ListNode* head) {
        if (!head || head->next == NULL) return head;
        ListNode *st = head, *a1 = head->next, *tBefore = NULL;
        ListNode *pHead;
        while (st)
        {
            if (!a1) break;
            auto t1 = a1->next;
            a1->next = st;
            st->next = t1;
            if (!tBefore)
                pHead = a1;
            else
                tBefore->next = a1;

            if (!(st->next))
                break;
            tBefore = st;
            st = st->next;
            a1 = st->next;
        }
        return pHead;
    }

    //025
    void reverseKGroupPart(ListNode* tHead, ListNode* tEnd){
        stack<ListNode*> nodes;
        auto sHead = tHead, sEnd = tEnd;
        while (sHead != sEnd)
        {
            nodes.push(sHead);
            sHead = sHead->next;
        }
        nodes.push(sEnd);

        ListNode *fHead = nodes.top(); nodes.pop();
        while (nodes.size())
        {
            fHead->next = nodes.top();
            fHead = nodes.top();
            nodes.pop();
        }
    }

    ListNode* reverseKGroup(ListNode* head, int k) {
        ListNode *cNode = head, *grpBefore = NULL, *grpAfter, *pHead;
        while (cNode)
        {
            int wK = k;
            auto stNode = cNode;
            while (--wK && cNode->next)
                cNode = cNode->next;
            auto cNNode = cNode->next;
            if (!wK)
                reverseKGroupPart(stNode, cNode);

            if (!grpBefore)
                pHead = cNode;
            else
                grpBefore->next = (!wK) ? cNode : stNode;

            grpBefore = (!wK) ? stNode : cNode;
            if (!(cNode = cNNode))
            {
                grpBefore->next = NULL;
                break;
            }
        }
        return pHead;
    }

    //026
    int removeDuplicates(vector<int>& nums) {
        if (!nums.size()) return 0;
        auto it = nums.begin(), itb = it;
        int tVal;
        while (it != nums.end())
        {
            itb = it;
            tVal = *it;
            while ((it + 1) != nums.end() && *(it + 1) == tVal)
                ++it;
            if (it != itb)
                it = nums.erase(itb++, it);
            ++it;
        }
        return nums.size();
    }

    //027
    int removeElement(vector<int>& nums, int val) {
        int size = nums.size(), lastp = size - 1, stp = 0, count = 0;
        while (stp <= lastp)
        {
            if (nums.at(stp) == val)
            {
                int tSt = stp + 1;
                while (tSt <= lastp)
                {
                    nums[tSt - 1] = nums[tSt];
                    ++tSt;
                }
                --lastp;
            }
            else
                ++stp;
        }
        return (lastp + 1);
    }

    //028
    int strStr(string haystack, string needle) {
        if (haystack.size() < needle.size()) return -1;
        for (int st = 0; st < haystack.size() - needle.size() + 1; st++)
            if (haystack.substr(st, needle.size()) == needle)
                return st;
        return -1;
    }

    //029 no divide and multiply and mod to do division
    int divideSelf(int dividend, int divisor) {
        if (!dividend) return 0;
        int d1sgn = dividend > 0 ? 1 : -1;
        int d2sgn = divisor > 0 ? 1 : -1;
        int count = 0;
        if (divisor == 1) return dividend;
        if (divisor == -1)
            if (INT_MIN == dividend)
                return INT_MAX;
            else
                return -dividend;
        if (d1sgn > 0)
            if (d2sgn > 0)
                while ((dividend -= divisor) >= 0)
                    count++;
            else
                while ((dividend += divisor) >= 0)
                    count--;
        else
            if (d2sgn > 0)
                while ((dividend += divisor) <= 0)
                    count--;
            else
                while ((dividend -= divisor) <= 0)
                    count++;
        return count;
    }

    int divide(int dividend, int divisor) {
        if(dividend == 0) return 0;
        if(divisor == 1) return dividend;
        if(divisor == -1){
            if (dividend > INT_MIN) return -dividend;                  // 只要不是最小的那个整数，都是直接返回相反数就好啦
            return INT_MAX;                                            // 是最小的那个，那就返回最大的整数啦
        }
        long a = dividend;
        long b = divisor;
        int sign = 1;
        if((a > 0 && b < 0) || (a < 0 && b > 0)){
            sign = -1;
        }
        a = a > 0 ? a: -a;
        b = b > 0 ? b: -b;
        long res = div(a,b);
        if (sign > 0)
            return res > INT_MAX ? INT_MAX : res;
        return -res;
    }

    int div(long a, long b){                                            // (经典)似乎精髓和难点就在于下面这几句
        if (a < b) return 0;
        long count = 1;
        long tb = b;                                                    // 在后面的代码中不更新b
        while((tb + tb) <= a){
            count = count + count;                                      // 最小解翻倍
            tb = tb + tb;                                               // 当前测试的值也翻倍
        }
        return count + div(a - tb, b);
    }


};



int main(int argc, char *argv[])
{
    Solution solution;
    std::cout << "The answer of 006 is:" << solution.convert("FEARCODETIME", 3) << endl;
    std::cout << "The answer of 007 is:" << solution.reverse(1534236469) << endl;
    std::cout << "The answer of 008 is:" << solution.myAtoi("2147483646") << endl;
    std::cout << "The answer of 009 is:" << solution.isPalindrome(122322) << endl;
    std::cout << "The answer of 010 is:" << solution.isMatch("Tooth", "Toot.") << endl;
    std::cout << "The answer of 011 is:" << solution.maxArea(vector<int>{4, 8, 1, 8, 3, 2}) << endl;
    solution.combination("abc");
    std::cout << "The answer of 012 is:" << solution.intToRoman2(1994) << endl;
    std::cout << "The answer of 013 is:" << solution.romanToInt("MCMXCIV") << endl;
    std::cout << "The answer of 014 is:" << solution.longestCommonPrefix(vector<string>{"flower", "flow", "flight"}) << endl;
    auto res = solution.threeSumSelf(vector<int>{-1, 0, 1, 2, -1, -4});
    solution.letterCombinations("234");
    solution.fourSum1(vector<int>{1, 0, -1, 0, 2, -2}, 0);
    auto ln1 = new ListNode(1), ln2 = new ListNode(2), ln3 = new ListNode(3), ln4 = new ListNode(4), ln5 = new ListNode(5);
    ln1->next = ln2; ln2->next = ln3; ln3->next = ln4; ln4->next = ln5;
    solution.removeNthFromEnd(ln1, 5);
    std::cout << "The answer of 020 is:" << solution.isValid("]") << endl;

    auto lnn1 = new ListNode(1), lnn2 = new ListNode(2), lnn3 = new ListNode(4), lnn4 = new ListNode(1), lnn5 = new ListNode(3), lnn6 = new ListNode(4);
    lnn1->next = lnn2; lnn2->next = lnn3; lnn4->next = lnn5; lnn5->next = lnn6;
    solution.mergeTwoLists(lnn1, lnn4);

    solution.generateParenthesis(2);

    auto la1 = new ListNode(1); auto la2 = new ListNode(4); auto la3 = new ListNode(5); la1->next = la2; la2->next = la3;
    auto lb1 = new ListNode(1); auto lb2 = new ListNode(3); auto lb3 = new ListNode(4); lb1->next = lb2; lb2->next = lb3;
    auto lc1 = new ListNode(2); auto lc2 = new ListNode(6); lc1->next = lc2;
    vector<ListNode*> KGrp{la1, lb1, lc1};
    solution.mergeKLists(KGrp);

    auto ld1 = new ListNode(1), ld2 = new ListNode(2), ld3 = new ListNode(3), ld4 = new ListNode(4); ld1->next = ld2; ld2->next = ld3; ld3->next = ld4;
    solution.swapPairs(ld1);

    auto le1 = new ListNode(1), le2 = new ListNode(2), le3 = new ListNode(3), le4 = new ListNode(4), le5 = new ListNode(5); le1->next = le2; le2->next = le3; le3->next = le4; le4->next = le5;
    solution.reverseKGroup(le1, 2);

    std::cout << "The answer of 026 is:" << solution.removeDuplicates(vector<int>{0,0,1,1,1,2,2,3,3,4,4}) << endl;
    std::cout << "The answer of 027 is:" << solution.removeElement(vector<int>{0,1,2,2,3,0,4,2}, 2) << endl;
    std::cout << "The answer of 029 is:" << solution.divide(100000, -3) << endl;
    vector<string> trump{"foo","bar"};
    solution.findSubstring("barfoothefoobarman", trump);
    solution.nextPermutationSelf(vector<int>{3, 2, 1});
    std::cout << "The answer of 032 is:" << solution.longestValidParentheses("(())())") << endl;
    std::cout << "The answer of 033 is:" << solution.search(vector<int>{ 5, 6, 1, 2, 3, 4}, 1) << endl;
    solution.searchRange(vector<int>{ 1}, 1);
    std::cout << "The answer of 035 is:" << solution.searchInsert(vector<int>{ 1, 3, 4, 5, 7, 9}, 4) << endl;
    vector<vector<char> > suduku{
        {'5','3','.','.','7','.','.','.','.'},
        {'6','.','.','1','9','5','.','.','.'},
        {'.','9','8','.','.','.','.','6','.'},
        {'8','.','.','.','6','.','.','.','3'},
        {'4','.','.','8','.','3','.','.','1'},
        {'7','.','.','.','2','.','.','.','6'},
        {'.','6','.','.','.','.','2','8','.'},
        {'.','.','.','4','1','9','.','.','5'},
        {'.','.','.','.','8','.','.','7','9'}};
    std::cout << "The answer of 036 is:" << solution.isValidSudoku(suduku) << endl;
    solution.solveSudoku(suduku);

    return 0;
}
