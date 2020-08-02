#ifndef DATATYPE_H
#define DATATYPE

#include<iostream>
#include<vector>
using namespace std;

//定义法向量
struct Normal
{
	float NX, NY, NZ;
};

//定义点
struct Point
{
	float X, Y, Z;//点的坐标
	int label;//点的类别
	int ID;
	Normal pNormal;//点的法向量
};

//定义体素
struct SuperVoxel
{
	std::vector<Point> AllPoints;//体素包含所有点
	Point CenterPoint;//体素中心点	
	Normal normal;//体素整体法向量	
	float Weight;//体素权值
	float PointCount;//体素点数
	int ID;
};

//建立图结构
struct GraphType1
{
	int ID;//节点的索引
	std::vector<GraphType1*> AllNeibors;//节点所有邻居的地址
};

struct R1 
{
	vector<vector<int>> UsedIndex;//记录本轮迭代中已经归类的体素ID
	vector<vector<int>> DealingIndexs;//记录本轮迭代后未归类的体素ID，这些体素将再次进行迭代
};

struct R2 {
	vector<int> DealingIndexSingle;//以数组形式记录即将运用RANSAC算法计算的体素ID
};

#endif // !DATATYPE_H
#pragma once
