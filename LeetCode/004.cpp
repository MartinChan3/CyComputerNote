#include <vector>
#include <unordered_map>
#include <iostream>
#include <algorithm>

//给定两个大小为 m 和 n 的有序数组 nums1 和 nums2。
//请你找出这两个有序数组的中位数，并且要求算法的时间复杂度为 O(log(m + n))。
//你可以假设 nums1 和 nums2 不会同时为空。
//示例 1:
//nums1 = [1, 3]
//nums2 = [2]
//则中位数是 2.0
//示例 2:
//nums1 = [1, 2]
//nums2 = [3, 4]
//则中位数是(2 + 3) / 2 = 2.5

using namespace std;

class Solution {
public:
	double findMedianSortedArrays(vector<int>& nums1, vector<int>& nums2) {
		int m = nums1.size(), n = nums2.size(), left = (m + n + 1) / 2, right = (m + n + 2) / 2;
		return findKth(nums1, 0, nums2, 0, left) + findKth(nums1, 0, nums2, 0, right) / 2.0;
	}

	int findKth(vector<int>& nums1, int i, vector<int>& nums2, int j , int k){
		if (i >= nums1.size()) return nums2[j + k - 1];
		if (j >= nums2.size()) return nums1[i + k - 1];
		if (k == 1) return min(nums1[i], nums2[j]);
		int midVal1 = (i + k / 2 - 1 < nums1.size()) ? nums1[i + k / 2 - 1] : INT_MAX;
		int midVal2 = (j + k / 2 - 1 < nums2.size()) ? nums2[j + k / 2 - 1] : INT_MAX;
		if (midVal1 < midVal2)
			return findKth(nums1, i + k / 2, nums2, j, k - k / 2);
		else
			return findKth(nums1, i , nums2, j + k / 2, k - k / 2);
	};
};

void main()
{
	vector<int> nums1, nums2;
	nums1.push_back(1);
	nums2.push_back(2);
	nums1.push_back(2);
	nums2.push_back(3);
	Solution solution;
	cout << solution.findMedianSortedArrays(nums1, nums2);	
}