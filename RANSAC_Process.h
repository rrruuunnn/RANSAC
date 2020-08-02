#ifndef RANSAC_PROCESS_H
#define RANSAC_PROCESS_H
#include<iostream>
#include"DataType.h"
#include"Math_Calculate.h"
#include<pcl/point_types.h>
#include<pcl/point_cloud.h>
#include<pcl/features/normal_3d.h>
#include<pcl/surface/gp3.h>
#include<Windows.h>
#include<math.h>
using namespace std;
//找到3个最佳初始点构建初始假设平面
vector<Point> InitPlane(vector<Point> allPoints);
//计算平面法向量
Normal CalNormal_ThreePoints(vector<Point> Plane);
//通过PCL库函数对点云数据进行三角剖分
std::vector<GraphType1>  TrianglePouFen(vector<SuperVoxel> AHouse);
//自适应阈值计算程序
double CalThreshold(vector<SuperVoxel> CurrentVoxels, double InitRatio, Normal N, double d, double Threshold);
//运用RANSAC算法实现从给定体素簇中寻找一个最佳的面并提取与面一致性较好的体素。
vector<int> FindBestPlane(vector<SuperVoxel> allVoxel, float &angle_Threshold, float &distance_Threshold);
//自适应阈值计算
double FindBestThreshold(vector<SuperVoxel> allVoxel, float &angle_Threshold, float &distance_Threshold);
//利用找到的各个角度＞30°的三角形平面和各个超体素进行投票
float VoteForPlane(vector<Point> Plane, vector<SuperVoxel> allVoxel, int32_t pointsCount, int& r, float& d, float angle_Threshold, float distance_Threshold);
//自适应阈值计算程序1
double CalThreshold1(std::vector<SuperVoxel> CurrentVoxels, double InitRatio, Normal N, double d, double Threshold);
//DealGraph_DealIndex函数的辅助函数
void DealGraph_DealIndexAssist(GraphType1* OneGraph, vector<int>*Selected, int *SelectedCount, vector<int>*tempResult, vector<int> VoxelID);
//建立各体素间的邻接关系，利用各体素的邻接关系和已提取面的体素ID将提取面分为多个在数学上共面但实际不共面的屋顶面
vector<vector<int>> DealGraph_DealIndex(vector<GraphType1>* ptrGraph, vector<int>VoxelID);
//dealOnePartVoxel1的辅助函数
R1 dealOnePartVoxel1Add(vector<SuperVoxel> *ptrPartVoxels, vector<GraphType1> *ptrGraph, vector<int> ptrDealingIndex,vector<int> *NotSort, float angle_Threshold, float distance_Threshold);
//通过建筑物体素，邻接关系，即将处理的体素ID，得到提取面、下一次将处理的体素ID以及未被提取的体素ID
void dealOnePartVoxel1(vector<SuperVoxel> *ptrPartVoxels, vector<GraphType1> *ptrGraph, vector<int> ptrDealingIndex, vector<vector<int>> *facesIDs,vector<int> *NotSort);

#endif // !RANSAC_PROCESS_H
#pragma once
