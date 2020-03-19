#include <string>
#include <iostream>
#include <vector>
#include <stack>
#include <algorithm>
#include <map>

using namespace std;

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

    //010 Expression match
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

    //015
    vector<vector<int>> threeSum(vector<int>& nums){

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

    return 0;
}
