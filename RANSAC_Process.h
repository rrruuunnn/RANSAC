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
//�ҵ�3����ѳ�ʼ�㹹����ʼ����ƽ��
vector<Point> InitPlane(vector<Point> allPoints);
//����ƽ�淨����
Normal CalNormal_ThreePoints(vector<Point> Plane);
//ͨ��PCL�⺯���Ե������ݽ��������ʷ�
std::vector<GraphType1>  TrianglePouFen(vector<SuperVoxel> AHouse);
//����Ӧ��ֵ�������
double CalThreshold(vector<SuperVoxel> CurrentVoxels, double InitRatio, Normal N, double d, double Threshold);
//����RANSAC�㷨ʵ�ִӸ������ش���Ѱ��һ����ѵ��沢��ȡ����һ���ԽϺõ����ء�
vector<int> FindBestPlane(vector<SuperVoxel> allVoxel, float &angle_Threshold, float &distance_Threshold);
//����Ӧ��ֵ����
double FindBestThreshold(vector<SuperVoxel> allVoxel, float &angle_Threshold, float &distance_Threshold);
//�����ҵ��ĸ����Ƕȣ�30���������ƽ��͸��������ؽ���ͶƱ
float VoteForPlane(vector<Point> Plane, vector<SuperVoxel> allVoxel, int32_t pointsCount, int& r, float& d, float angle_Threshold, float distance_Threshold);
//����Ӧ��ֵ�������1
double CalThreshold1(std::vector<SuperVoxel> CurrentVoxels, double InitRatio, Normal N, double d, double Threshold);
//DealGraph_DealIndex�����ĸ�������
void DealGraph_DealIndexAssist(GraphType1* OneGraph, vector<int>*Selected, int *SelectedCount, vector<int>*tempResult, vector<int> VoxelID);
//���������ؼ���ڽӹ�ϵ�����ø����ص��ڽӹ�ϵ������ȡ�������ID����ȡ���Ϊ�������ѧ�Ϲ��浫ʵ�ʲ�������ݶ���
vector<vector<int>> DealGraph_DealIndex(vector<GraphType1>* ptrGraph, vector<int>VoxelID);
//dealOnePartVoxel1�ĸ�������
R1 dealOnePartVoxel1Add(vector<SuperVoxel> *ptrPartVoxels, vector<GraphType1> *ptrGraph, vector<int> ptrDealingIndex,vector<int> *NotSort, float angle_Threshold, float distance_Threshold);
//ͨ�����������أ��ڽӹ�ϵ���������������ID���õ���ȡ�桢��һ�ν����������ID�Լ�δ����ȡ������ID
void dealOnePartVoxel1(vector<SuperVoxel> *ptrPartVoxels, vector<GraphType1> *ptrGraph, vector<int> ptrDealingIndex, vector<vector<int>> *facesIDs,vector<int> *NotSort);

#endif // !RANSAC_PROCESS_H
#pragma once
