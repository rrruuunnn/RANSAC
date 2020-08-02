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

//�����������н�
float CalAngle_TwoVector(Point p1,Point p2);
//�����������γ������εĽǶ�
vector<float> CalAngle_ThreePoints(Point p1, Point p2, Point p3);
//��ȡ�����
void getRand(long Max, long *backNum);
//���㷨�����н�
float CalACos(float x1, float y1, float z1, float x2, float y2, float z2);
//�����󽻼�
bool vectors_intersection(vector<int> v1, vector<int> v2);
//���ϼ�����ͨ���㷨���õ���һ�ε���ʱʣ���IDֵ
vector<int> VectorDec(vector<int> A, vector<int> B);
//�ҵ�ĳԪ��������������
int FindIndexInVector(int Number, vector<int> Vector);
//�����face2��ƽ��face1֮��ľ���
float CalDis_Faces(Point face1, Point face2);
//Ѱ������vector���е�Ԫ��
std::vector<int> findSame(std::vector<int>&nLeft, std::vector<int>&nRight);
//�����������巨����
Point CalNormal(pcl::PointCloud<pcl::PointXYZ>::Ptr CenterPoint, pcl::PointCloud<pcl::PointXYZ>::Ptr ReferencePoints);
//���׼��
float CalSD(std::vector<float> alldeviation);
//ͨ��ԭʼ�������׼��
double CalSD1(std::vector<double> allNums,std::vector<int>Weight);
//ͨ��ԭʼ�������׼��
double CalSD2(std::vector<double> allNums);


#endif // !MATH_CALCULATE_H
#pragma once
