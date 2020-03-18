#include <string>
#include <iostream>
#include <vector>
#include <stack>

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


};

int main(int argc, char *argv[])
{
    Solution solution;
    std::cout << "The answer of 006 is:" << solution.convert("FEARCODETIME", 3) << endl;
    std::cout << "The answer of 007 is:" << solution.reverse(1534236469) << endl;
    std::cout << "The answer of 008 is:" << solution.myAtoi("2147483646") << endl;
    std::cout << "The answer of 009 is:" << solution.isPalindrome(122322) << endl;
    std::cout << "The answer of 010 is:" << solution.isMatch("Tooth", "Toot.") << endl;

    return 0;
}
