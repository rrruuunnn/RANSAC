#include<iostream>
#include"DataType.h"
#include"Init_Data.h"
#include"RANSAC_Process.h"
#include"Post_treatment.h"

/*
	�����Ϊ�����ݽṹ���塢���ݶ�ȡ����ʼ����RANSAC����������������6����
*/
int main()
{
	
	//���ݶ�ȡ���ļ���ÿ�и�ʽΪ X Y Z R G B��
	std::vector<Point> points;
	const char* s = "F:\\cluster\\ExpResult\\a1\\out2.xyz";
	points = ReadTXT(s);

	//��ʼ�������ط��ࣺ���ļ���RGBֵ��ͬ�ĵ��Ϊһ�����أ�
	std::vector<SuperVoxel> allVoxels;
	allVoxels = DataInit(points);

	VoxelsInit(allVoxels);

	int a = 0;
	for (int i = 0; i < allVoxels.size(); i++)
	{
		for (int j = 0; j < allVoxels[i].AllPoints.size(); j++)
		{
			a++;
		}
	}

	FILE *fpn1 = fopen("Normal1.txt", "w+");
	for (int i = 0; i < allVoxels.size(); i++)
	{
		fprintf(fpn1, "%f %f %f %f %f %f\n", allVoxels[i].CenterPoint.X, allVoxels[i].CenterPoint.Y, allVoxels[i].CenterPoint.Z, allVoxels[i].normal.NX, allVoxels[i].normal.NY, allVoxels[i].normal.NZ);
	}
	fclose(fpn1);
		
	SuperVoxelsEstimate(allVoxels);	
	a = 0;
	for (int i = 0; i < allVoxels.size(); i++)
	{
		for (int j = 0; j < allVoxels[i].AllPoints.size(); j++)
		{
			a++;
		}
	}

	VoxelsInit(allVoxels);
	a = 0;
	for (int i = 0; i < allVoxels.size(); i++)
	{
		for (int j = 0; j < allVoxels[i].AllPoints.size(); j++)
		{
			a++;
		}
	}

	FILE *fpn = fopen("Normal.txt", "w+");
	FILE *fpn2 = fopen("Normal2.txt", "w+");
	for (int i = 0; i < allVoxels.size(); i++)
	{
		for (int j = 0; j < allVoxels[i].AllPoints.size(); j++)
		{
			fprintf(fpn2, "%f %f %f %f %f %f %d\n", allVoxels[i].AllPoints[j].X, allVoxels[i].AllPoints[j].Y, allVoxels[i].AllPoints[j].Z, allVoxels[i].AllPoints[j].pNormal.NX, allVoxels[i].AllPoints[j].pNormal.NY, allVoxels[i].AllPoints[j].pNormal.NZ, i);
		}
		fprintf(fpn, "%f %f %f %f %f %f %d\n", allVoxels[i].CenterPoint.X, allVoxels[i].CenterPoint.Y, allVoxels[i].CenterPoint.Z, allVoxels[i].normal.NX, allVoxels[i].normal.NY, allVoxels[i].normal.NZ, i);
	}
	fclose(fpn2);
	fclose(fpn);

	//��ʼ����������ִأ����������ذ���DBSCAN�㷨�ۼ�Ϊһ�����Ľ����
	std::vector<std::vector<SuperVoxel>> DivideToHouses = DivideToHouse(allVoxels);

	a = 0;
	for (int i = 0; i < 1; i++)
	{
		for (int j = 0; j < DivideToHouses[i].size(); j++)
		{			
			a += DivideToHouses[i][j].AllPoints.size();
		}
	}

	//��ӡ������
	FILE *TXTDivideToHouses = fopen("TXTDivideToHouses.txt", "w+");
	for (int i = 0; i < DivideToHouses.size(); i++)
		for (int j = 0; j < DivideToHouses[i].size(); j++)
			for (int k = 0; k < DivideToHouses[i][j].AllPoints.size(); k++)
				fprintf(TXTDivideToHouses, "%f %f %f %d\n", DivideToHouses[i][j].AllPoints[k].X, DivideToHouses[i][j].AllPoints[k].Y, DivideToHouses[i][j].AllPoints[k].Z, i * 10);
	fclose(TXTDivideToHouses);

	//��һ��������Ϊ���崦��ý�������������أ�����ȡ�ݶ���
	FILE *fp_sum = fopen("sum.txt", "w+"); fclose(fp_sum);//���յ��ݶ���ָ����洢��sum.txt����
	FILE *fpsum = fopen("sum1.txt", "w+"); fclose(fpsum);
	for (int i = 0; i < DivideToHouses.size(); i++)
	{
		if (DivideToHouses[i].size() > 2)
		{
			for (int j = 0; j < DivideToHouses[i].size(); j++)
			{
				DivideToHouses[i][j].ID = j;
				for (int k = 0; k < DivideToHouses[i][j].AllPoints.size(); k++)
				{
					DivideToHouses[i][j].AllPoints[k].ID = j;
					DivideToHouses[i][j].ID = j;
				}
			}
			std::vector<int> *NotSort = new std::vector<int>();//��¼δ��������أ�Ϊ������׼��
			vector<vector<int>> *FacesIDs = new vector<vector<int>>;//�����洢ÿ���������������ID��
			std::vector<GraphType1> Graph1 = TrianglePouFen(DivideToHouses[i]);//��¼�������ڽӹ�ϵ������������ѧ�ϵĹ������


			//��ʼ��ָ��
			vector<GraphType1> *ptrGraph1 = new vector<GraphType1>;//ͼ�ṹ
			ptrGraph1 = &Graph1;
			vector<SuperVoxel> *ptrDivideToHouses = new vector<SuperVoxel>;
			ptrDivideToHouses = &DivideToHouses[i];
			vector<int> DealingIndex;//��¼��һ����Ҫ���������ID
			for (int j = 0; j < DivideToHouses[i].size(); j++)
			{
				DealingIndex.push_back(DivideToHouses[i][j].ID);
			}			
			dealOnePartVoxel1(ptrDivideToHouses, ptrGraph1, DealingIndex, FacesIDs, NotSort);//���õ����㷨��������������

			DealRemainVoxel(ptrDivideToHouses, *FacesIDs, *NotSort);//���ݶ�����ȡ�����󣬶�δ��������ؽ��к���������ൽ������棩
		}
	}
	
	std::cout << "���н���" << std::endl;	
	
	system("pause");
	return 0;
}