#include <vector>
#include <unordered_map>
#include <iostream>
#include <algorithm>

//给定一个字符串，请你找出其中不含有重复字符的最长子串的长度。
//示例 1:
//输入 : "abcabcbb"
//输出 : 3
// 解释 : 因为无重复字符的最长子串是 "abc"，所以其长度为 3。
//示例 2 :
//  输入 : "bbbbb"
//   输出 : 1
//	解释 : 因为无重复字符的最长子串是 "b"，所以其长度为 1。
//示例 3 :
//	 输入: "pwwkew"
//   输出: 3
//解释: 因为无重复字符的最长子串是 "wke"，所以其长度为 3。
//请注意，你的答案必须是子串的长度，"pwke" 是一个子序列，不是子串。

using namespace std;

class Solution {
public:
	int lengthOfLongestSubstring(string s){
		int res = 0, left = -1, n = s.size();
		unordered_map<int, int> m;
		for (int i = 0; i < n; ++i){
			if (m.count(s[i]) && m[s[i]] > left){ //建立map，寻找每次窗格最左侧和最右侧的内容，而每次记录最左侧非重复字母的最大index为left
				left = m[s[i]];
			}
			m[s[i]] = i;
			res = max(res, i - left);
		}
		return res;
	}
};

void main()
{
	Solution solution;
	cout << solution.lengthOfLongestSubstring("missipi");
}