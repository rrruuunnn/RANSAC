#ifndef DATATYPE_H
#define DATATYPE

#include<iostream>
#include<vector>
using namespace std;

//���巨����
struct Normal
{
	float NX, NY, NZ;
};

//�����
struct Point
{
	float X, Y, Z;//�������
	int label;//������
	int ID;
	Normal pNormal;//��ķ�����
};

//��������
struct SuperVoxel
{
	std::vector<Point> AllPoints;//���ذ������е�
	Point CenterPoint;//�������ĵ�	
	Normal normal;//�������巨����	
	float Weight;//����Ȩֵ
	float PointCount;//���ص���
	int ID;
};

//����ͼ�ṹ
struct GraphType1
{
	int ID;//�ڵ������
	std::vector<GraphType1*> AllNeibors;//�ڵ������ھӵĵ�ַ
};

struct R1 
{
	vector<vector<int>> UsedIndex;//��¼���ֵ������Ѿ����������ID
	vector<vector<int>> DealingIndexs;//��¼���ֵ�����δ���������ID����Щ���ؽ��ٴν��е���
};

struct R2 {
	vector<int> DealingIndexSingle;//��������ʽ��¼��������RANSAC�㷨���������ID
};

#endif // !DATATYPE_H
#pragma once
