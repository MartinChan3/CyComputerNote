#include <vector>
#include <unordered_map>
#include <iostream>
#include <algorithm>

//����һ���ַ����������ҳ����в������ظ��ַ�����Ӵ��ĳ��ȡ�
//ʾ�� 1:
//���� : "abcabcbb"
//��� : 3
// ���� : ��Ϊ���ظ��ַ�����Ӵ��� "abc"�������䳤��Ϊ 3��
//ʾ�� 2 :
//  ���� : "bbbbb"
//   ��� : 1
//	���� : ��Ϊ���ظ��ַ�����Ӵ��� "b"�������䳤��Ϊ 1��
//ʾ�� 3 :
//	 ����: "pwwkew"
//   ���: 3
//����: ��Ϊ���ظ��ַ�����Ӵ��� "wke"�������䳤��Ϊ 3��
//��ע�⣬��Ĵ𰸱������Ӵ��ĳ��ȣ�"pwke" ��һ�������У������Ӵ���

using namespace std;

class Solution {
public:
	int lengthOfLongestSubstring(string s){
		int res = 0, left = -1, n = s.size();
		unordered_map<int, int> m;
		for (int i = 0; i < n; ++i){
			if (m.count(s[i]) && m[s[i]] > left){ //����map��Ѱ��ÿ�δ������������Ҳ�����ݣ���ÿ�μ�¼�������ظ���ĸ�����indexΪleft
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