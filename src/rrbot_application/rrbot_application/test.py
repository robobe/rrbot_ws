
from typing import List

class Solution:
    def twoSum(self, nums: List[int], target: int) -> List[int]:
        for i in range(len(nums)-1):
            j = i+1
            if nums[i] + nums[j] == target:
                return [i,j]
                
nums=[2,7,11,15]
target = 9

s = Solution()
print(s.twoSum(nums, target))

nums=[3,2,4]
target = 6

print(s.twoSum(nums, target))