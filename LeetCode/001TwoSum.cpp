#include <vector>
#include <unordered_map>
#include <iostream>

using namespace std;

class Solution {
public:
	vector<int> twoSum(vector<int>& nums, int target) {
		unordered_map<int, int> m;   //ʹ��map��Ҫԭ��Ϊ����ʱ��ΪO(n)
		vector<int> res;
		for (int i = 0; i < nums.size(); ++i) {
			m[nums[i]] = i;          //��ֵ-ֵ�ķ�ʽ����
		}
		for (int i = 0; i < nums.size(); ++i) {
			int t = target - nums[i];
			if (m.count(t) && m[t] != i) {  //map�����е�count()��ʽΪѰ�Ҷ�Ӧ�ļ�ֵ
				res.push_back(i);
				res.push_back(m[t]);
				break;
			}
		}
		return res;
	}

	vector<int> twoSum2(vector<int>& nums, int target) {  //�ϲ�Ϊһ��ѭ��
		unordered_map<int, int> m;
		for (int i = 0; i < nums.size(); ++i) {
			if (m.count(target - nums[i])) {    //�Ľ�֮����������һ�����ģ�һ��������д��
				return{ i, m[target - nums[i]] };
			}
			m[nums[i]] = i;
		}
		return{};
	}
};

void main()
{
	vector<int> nums;
	nums.push_back(2);
	nums.push_back(7);
	nums.push_back(11);
	nums.push_back(13);
	int	target = 9;
	Solution solution;
	solution.twoSum2(nums, target);
}