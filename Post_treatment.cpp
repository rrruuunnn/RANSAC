#include"Post_treatment.h"
void DealRemainVoxel(std::vector<SuperVoxel> *ptrDivideToHouses, std::vector<std::vector<int>> SortVoxelIndex, std::vector<int> UnsortVoxelIndex)
{
	int SortCount = 0;
	int UnsortCount = 0;
	for (int i = 0; i < SortVoxelIndex.size(); i++)
		for (int j = 0; j < SortVoxelIndex[i].size(); j++)
			SortCount += (*ptrDivideToHouses)[SortVoxelIndex[i][j]].AllPoints.size();
	for (int i = 0; i < UnsortVoxelIndex.size(); i++)
		UnsortCount += (*ptrDivideToHouses)[UnsortVoxelIndex[i]].AllPoints.size();
	if (SortCount > UnsortCount)
	{

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
		pcl::PointXYZ tempp;
		std::vector<int> lab;
		std::vector<int> Labed(UnsortVoxelIndex.size());
		for (int i = 0; i < SortVoxelIndex.size(); i++)
		{
			for (int j = 0; j < SortVoxelIndex[i].size(); j++)
			{
				tempp.x = (*ptrDivideToHouses)[SortVoxelIndex[i][j]].CenterPoint.X;
				tempp.y = (*ptrDivideToHouses)[SortVoxelIndex[i][j]].CenterPoint.Y;
				tempp.z = (*ptrDivideToHouses)[SortVoxelIndex[i][j]].CenterPoint.Z;
				(*ptrDivideToHouses)[SortVoxelIndex[i][j]].ID = i + 1;
				cloud->points.push_back(tempp);
				lab.push_back(i + 1);
			}
		}

		pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
		kdtree.setInputCloud(cloud);
		vector<int> nearPointIndex;//邻域点的id
		vector<float> nearPointDistance;//邻域点的距离
		for (int i = 0; i < UnsortVoxelIndex.size(); i++)
		{
			tempp.x = (*ptrDivideToHouses)[UnsortVoxelIndex[i]].CenterPoint.X;
			tempp.y = (*ptrDivideToHouses)[UnsortVoxelIndex[i]].CenterPoint.Y;
			tempp.z = (*ptrDivideToHouses)[UnsortVoxelIndex[i]].CenterPoint.Z;
			if (kdtree.nearestKSearch(tempp, 2, nearPointIndex, nearPointDistance) > 0)
			{
				Labed[i] = lab[nearPointIndex[0]];
				(*ptrDivideToHouses)[UnsortVoxelIndex[i]].ID = lab[nearPointIndex[0]];
			}
		}
		long RandColor[3];
		getRand(255, RandColor);
		int R = 0; int G = 0; int B = 0;
		FILE *fp_sum = fopen("sum.txt", "a");
		for (int i = 0; i < SortVoxelIndex.size(); i++)
		{
			R = ((*ptrDivideToHouses)[SortVoxelIndex[i][0]].ID*RandColor[0]) % 255;
			G = ((*ptrDivideToHouses)[SortVoxelIndex[i][0]].ID*RandColor[1]) % 255;
			B = ((*ptrDivideToHouses)[SortVoxelIndex[i][0]].ID*RandColor[2]) % 255;
			for (int j = 0; j < SortVoxelIndex[i].size(); j++)
			{
				for (int k = 0; k < (*ptrDivideToHouses)[SortVoxelIndex[i][j]].AllPoints.size(); k++)
				{
					fprintf(fp_sum, "%f %f %f %d\n", (*ptrDivideToHouses)[SortVoxelIndex[i][j]].AllPoints[k].X, (*ptrDivideToHouses)[SortVoxelIndex[i][j]].AllPoints[k].Y, (*ptrDivideToHouses)[SortVoxelIndex[i][j]].AllPoints[k].Z, R+G+B);
				}
			}
		}
		for (int i = 0; i < UnsortVoxelIndex.size(); i++)
		{
			R = ((*ptrDivideToHouses)[UnsortVoxelIndex[i]].ID*RandColor[0]) % 255;
			G = ((*ptrDivideToHouses)[UnsortVoxelIndex[i]].ID*RandColor[1]) % 255;
			B = ((*ptrDivideToHouses)[UnsortVoxelIndex[i]].ID*RandColor[2]) % 255;
			for (int k = 0; k < (*ptrDivideToHouses)[UnsortVoxelIndex[i]].AllPoints.size(); k++)
			{
				fprintf(fp_sum, "%f %f %f %d %d %d\n", (*ptrDivideToHouses)[UnsortVoxelIndex[i]].AllPoints[k].X, (*ptrDivideToHouses)[UnsortVoxelIndex[i]].AllPoints[k].Y, (*ptrDivideToHouses)[UnsortVoxelIndex[i]].AllPoints[k].Z, R, G, B);
			}
		}
		fclose(fp_sum);
	}
}