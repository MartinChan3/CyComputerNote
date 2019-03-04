#include <vector>
#include <unordered_map>
#include <iostream>

using namespace std;

class Solution {
public:
	vector<int> twoSum(vector<int>& nums, int target) {
		unordered_map<int, int> m;   //使用map主要原因为搜索时间为O(n)
		vector<int> res;
		for (int i = 0; i < nums.size(); ++i) {
			m[nums[i]] = i;          //键值-值的方式索引
		}
		for (int i = 0; i < nums.size(); ++i) {
			int t = target - nums[i];
			if (m.count(t) && m[t] != i) {  //map容器中的count()方式为寻找对应的键值
				res.push_back(i);
				res.push_back(m[t]);
				break;
			}
		}
		return res;
	}

	vector<int> twoSum2(vector<int>& nums, int target) {  //合并为一个循环
		unordered_map<int, int> m;
		for (int i = 0; i < nums.size(); ++i) {
			if (m.count(target - nums[i])) {    //改进之处核心在于一边审阅，一边向数组写入
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