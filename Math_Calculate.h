#ifndef MATH_CALCULATE_H
#define MATH_CALCULATE_H
#define M_PI 3.1415926535897932
#include"DataType.h"
#include<iostream>
#include<random>
#include<array>
#include<numeric>
#include<pcl/point_cloud.h>
#include<pcl/point_types.h>
#include<pcl/kdtree/kdtree_flann.h>
#include<pcl/features/normal_3d.h>
using namespace std;

//计算两向量夹角
float CalAngle_TwoVector(Point p1,Point p2);
//计算三个点形成三角形的角度
vector<float> CalAngle_ThreePoints(Point p1, Point p2, Point p3);
//获取随机点
void getRand(long Max, long *backNum);
//计算法向量夹角
float CalACos(float x1, float y1, float z1, float x2, float y2, float z2);
//数组求交集
bool vectors_intersection(vector<int> v1, vector<int> v2);
//集合减法，通过算法来得到下一次迭代时剩余的ID值
vector<int> VectorDec(vector<int> A, vector<int> B);
//找到某元素在数组中索引
int FindIndexInVector(int Number, vector<int> Vector);
//计算点face2到平面face1之间的距离
float CalDis_Faces(Point face1, Point face2);
//寻找两个vector共有的元素
std::vector<int> findSame(std::vector<int>&nLeft, std::vector<int>&nRight);
//求各个面的整体法向量
Point CalNormal(pcl::PointCloud<pcl::PointXYZ>::Ptr CenterPoint, pcl::PointCloud<pcl::PointXYZ>::Ptr ReferencePoints);
//求标准差
float CalSD(std::vector<float> alldeviation);
//通过原始数据求标准差
double CalSD1(std::vector<double> allNums,std::vector<int>Weight);
//通过原始数据求标准差
double CalSD2(std::vector<double> allNums);


#endif // !MATH_CALCULATE_H
#pragma once
