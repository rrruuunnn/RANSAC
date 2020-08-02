#ifndef INIT_DATA_H
#define INIT_DATA
#include"DataType.h"
#include"Math_Calculate.h"
#include<iostream>
#include<Windows.h>
#include<pcl/point_cloud.h>
#include<pcl/point_types.h>
#include<pcl/kdtree/kdtree_flann.h>
#include<pcl/features/normal_3d.h>
#include<string>
#include"RANSAC_Process.h"
using namespace std;
//读文件，计算各点法向量
vector<Point> ReadTXT(const char* FileName);
//将数据初始化，获取每个体素的所有点、计算体素中心点、法向量、点数
vector<SuperVoxel> DataInit(vector<Point> points);
//各体素重新计算中心点，法向量等等。
void VoxelsInit(vector<SuperVoxel> &SuperVoxels);
//将各个房屋分离出来,输入体素簇，输出多个体素簇(房子)
vector<vector<SuperVoxel>> DivideToHouse(vector<SuperVoxel> allVoxels);
//体素评估（运用体素标准差进行评估，目前定义的阈值为0.3m）
void SuperVoxelsEstimate(vector<SuperVoxel> &allVoxels);
#endif // !INIT_DATA_H
#pragma once
