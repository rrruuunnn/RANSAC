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
//���ļ���������㷨����
vector<Point> ReadTXT(const char* FileName);
//�����ݳ�ʼ������ȡÿ�����ص����е㡢�����������ĵ㡢������������
vector<SuperVoxel> DataInit(vector<Point> points);
//���������¼������ĵ㣬�������ȵȡ�
void VoxelsInit(vector<SuperVoxel> &SuperVoxels);
//���������ݷ������,�������شأ����������ش�(����)
vector<vector<SuperVoxel>> DivideToHouse(vector<SuperVoxel> allVoxels);
//�����������������ر�׼�����������Ŀǰ�������ֵΪ0.3m��
void SuperVoxelsEstimate(vector<SuperVoxel> &allVoxels);
#endif // !INIT_DATA_H
#pragma once
