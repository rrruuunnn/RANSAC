#ifndef POST_TREATMENT_H
#define POST_TREATMENT_H
#include<iostream>
#include"DataType.h"
#include"Math_Calculate.h"
#include<pcl/point_cloud.h>
#include<pcl/point_types.h>
#include<pcl/features/normal_3d.h>
#include<pcl/kdtree/kdtree_flann.h>
using namespace std;
//将未提取的体素归到最近的面中
void DealRemainVoxel(std::vector<SuperVoxel> *ptrDivideToHouses, std::vector<std::vector<int>> SortVoxelIndex, std::vector<int> UnsortVoxelIndex);
#endif // !POST_TREATMENT_H
#pragma once
