#include<iostream>
#include"DataType.h"
#include"Init_Data.h"
#include"RANSAC_Process.h"
#include"Post_treatment.h"

/*
	程序分为了数据结构定义、数据读取、初始化、RANSAC、迭代处理、后处理，共6部分
*/
int main()
{
	
	//数据读取（文件中每行格式为 X Y Z R G B）
	std::vector<Point> points;
	const char* s = "F:\\cluster\\ExpResult\\a1\\out2.xyz";
	points = ReadTXT(s);

	//初始化（体素分类：将文件中RGB值相同的点归为一个体素）
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

	//初始化（建筑物分簇：将各个体素按照DBSCAN算法聚集为一个个的建筑物）
	std::vector<std::vector<SuperVoxel>> DivideToHouses = DivideToHouse(allVoxels);

	a = 0;
	for (int i = 0; i < 1; i++)
	{
		for (int j = 0; j < DivideToHouses[i].size(); j++)
		{			
			a += DivideToHouses[i][j].AllPoints.size();
		}
	}

	//打印各个簇
	FILE *TXTDivideToHouses = fopen("TXTDivideToHouses.txt", "w+");
	for (int i = 0; i < DivideToHouses.size(); i++)
		for (int j = 0; j < DivideToHouses[i].size(); j++)
			for (int k = 0; k < DivideToHouses[i][j].AllPoints.size(); k++)
				fprintf(TXTDivideToHouses, "%f %f %f %d\n", DivideToHouses[i][j].AllPoints[k].X, DivideToHouses[i][j].AllPoints[k].Y, DivideToHouses[i][j].AllPoints[k].Z, i * 10);
	fclose(TXTDivideToHouses);

	//以一个建筑物为整体处理该建筑物的所有体素，并提取屋顶面
	FILE *fp_sum = fopen("sum.txt", "w+"); fclose(fp_sum);//最终的屋顶面分割结果存储在sum.txt里面
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
			std::vector<int> *NotSort = new std::vector<int>();//记录未归类的体素，为后处理做准备
			vector<vector<int>> *FacesIDs = new vector<vector<int>>;//用来存储每个面的索引（若干ID）
			std::vector<GraphType1> Graph1 = TrianglePouFen(DivideToHouses[i]);//记录各体素邻接关系，用来避免数学上的共面情况


			//开始用指针
			vector<GraphType1> *ptrGraph1 = new vector<GraphType1>;//图结构
			ptrGraph1 = &Graph1;
			vector<SuperVoxel> *ptrDivideToHouses = new vector<SuperVoxel>;
			ptrDivideToHouses = &DivideToHouses[i];
			vector<int> DealingIndex;//记录下一次需要处理的体素ID
			for (int j = 0; j < DivideToHouses[i].size(); j++)
			{
				DealingIndex.push_back(DivideToHouses[i][j].ID);
			}			
			dealOnePartVoxel1(ptrDivideToHouses, ptrGraph1, DealingIndex, FacesIDs, NotSort);//运用迭代算法处理整个建筑物

			DealRemainVoxel(ptrDivideToHouses, *FacesIDs, *NotSort);//在屋顶面提取结束后，对未归类的体素进行后处理（将其归类到最近的面）
		}
	}
	
	std::cout << "运行结束" << std::endl;	
	
	system("pause");
	return 0;
}