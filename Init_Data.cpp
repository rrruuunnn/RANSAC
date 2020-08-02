#include"Init_Data.h"
vector<Point> ReadTXT(const char* a)
{
	Point point = { 0,0,0 ,0,0,0 };//一个点
	vector<Point> points;//所有点
	//读取txt中点的XYZRGB值
	FILE *fp_txt;	
	fp_txt = fopen(a, "r");
	if (fp_txt)
	{
		while (fscanf(fp_txt, "%f %f %f %d", &point.X, &point.Y, &point.Z, &point.label) != EOF)
		{
			/*if (point.X >= 500000)
			{*/
				//point.X += 300;
				//point.Y += 400;
				//point.Z -= ;
			/*}*/
			points.push_back(point);
		}
	}
	fclose(fp_txt);	

	//计算法向量
	pcl::PointXYZ pt;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	for (int i = 0; i < points.size(); i++)
	{
		pt.x = points[i].X + 150;
		pt.y = points[i].Y;
		pt.z = points[i].Z;
		cloud->push_back(pt);
	}
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud(cloud);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	ne.setSearchMethod(tree);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
	ne.setKSearch(15);
	ne.compute(*cloud_normals);	

	for (int i = 0; i < points.size(); i++)
	{
		if (cloud_normals->points[i].normal_z > 0)
		{
			points[i].pNormal.NX = cloud_normals->points[i].normal_x;
			points[i].pNormal.NY = cloud_normals->points[i].normal_y;
			points[i].pNormal.NZ = cloud_normals->points[i].normal_z;
		}
		else
		{
			points[i].pNormal.NX = cloud_normals->points[i].normal_x*(-1);
			points[i].pNormal.NY = cloud_normals->points[i].normal_y*(-1);
			points[i].pNormal.NZ = cloud_normals->points[i].normal_z*(-1);
		}
	}
	return points;
}

vector<SuperVoxel> DataInit(vector<Point> points)//通过文件名称获取信息，还应添加超体素个数小于4的情况
{
	//将各个点进行归类
	//改进的数据读取方式
	std::vector<int32_t> labels(points.size());
	for (int i = 0; i < points.size(); i++)
	{
		labels[i] = points[i].label;
	}
	std::int32_t MaxLabel = *std::max_element(labels.begin(), labels.end());
	std::int32_t MinLabel = *std::min_element(labels.begin(), labels.end());
	
	vector<SuperVoxel> allVoxel(MaxLabel + 1);
	for (int i = 0; i < points.size(); i++)
	{
		allVoxel[labels[i]].AllPoints.push_back(points[i]);
	}
	for (int i = allVoxel.size() - 1; i >= 0; i--)
	{
		if (allVoxel[i].AllPoints.size() == 0)
		{
			allVoxel.erase(allVoxel.begin() + i);
		}
	}
	////计算中心点、法向量、点数

	//for (int i = 0; i < allVoxel.size(); i++)
	//{
	//	if (allVoxel[i].AllPoints.size() < 4)
	//	{
	//		continue;
	//	}
	//	//计算中心点
	//	float sumX, sumY, sumZ;
	//	sumX = 0; sumY = 0; sumZ = 0;
	//	for (int j = 0; j < allVoxel[i].AllPoints.size(); j++)
	//	{
	//		sumX += allVoxel[i].AllPoints[j].X;
	//		sumY += allVoxel[i].AllPoints[j].Y;
	//		sumZ += allVoxel[i].AllPoints[j].Z;
	//	}
	//	float avgX, avgY, avgZ;
	//	avgX = avgY = avgZ = 0;
	//	avgX = sumX / allVoxel[i].AllPoints.size();
	//	avgY = sumY / allVoxel[i].AllPoints.size();
	//	avgZ = sumZ / allVoxel[i].AllPoints.size();
	//	allVoxel[i].CenterPoint.X = avgX;
	//	allVoxel[i].CenterPoint.Y = avgY;
	//	allVoxel[i].CenterPoint.Z = avgZ;
	//	allVoxel[i].CenterPoint.label = allVoxel[i].AllPoints[0].label;
	//	//计算点数
	//	allVoxel[i].PointCount = allVoxel[i].AllPoints.size();
	//	allVoxel[i].Weight = allVoxel[i].AllPoints.size();
	//	allVoxel[i].ID = i;

	//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);
	//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud3(new pcl::PointCloud<pcl::PointXYZ>);
	//	pcl::PointXYZ temppxyz;
	//	
	//	temppxyz.x = allVoxel[i].CenterPoint.X;
	//	temppxyz.y = allVoxel[i].CenterPoint.Y;
	//	temppxyz.z = allVoxel[i].CenterPoint.Z;
	//	cloud2->points.push_back(temppxyz);
	//	
	//	for (int j = 0; j < allVoxel[i].AllPoints.size(); j++)
	//	{
	//		temppxyz.x = allVoxel[i].AllPoints[j].X;
	//		temppxyz.y = allVoxel[i].AllPoints[j].Y;
	//		temppxyz.z = allVoxel[i].AllPoints[j].Z;
	//		cloud3->points.push_back(temppxyz);
	//	}

	//	Point p1 = CalNormal(cloud2, cloud3);
	//	if (p1.pNormal.NZ < 0)
	//	{
	//		allVoxel[i].normal.NX = allVoxel[i].CenterPoint.pNormal.NX = (-1)*p1.pNormal.NX;
	//		allVoxel[i].normal.NY = allVoxel[i].CenterPoint.pNormal.NY = (-1)*p1.pNormal.NY;
	//		allVoxel[i].normal.NZ = allVoxel[i].CenterPoint.pNormal.NZ = (-1)*p1.pNormal.NZ;
	//	}
	//	else
	//	{
	//		allVoxel[i].normal.NX = allVoxel[i].CenterPoint.pNormal.NX = p1.pNormal.NX;
	//		allVoxel[i].normal.NY = allVoxel[i].CenterPoint.pNormal.NY = p1.pNormal.NY;
	//		allVoxel[i].normal.NZ = allVoxel[i].CenterPoint.pNormal.NZ = p1.pNormal.NZ;
	//	}
	//}
	//

	return allVoxel;
}

void VoxelsInit(vector<SuperVoxel> &allVoxel)
{
	int OriginCount = allVoxel.size();
	vector<SuperVoxel> AddVoxel;
	for (int i = OriginCount-1; i >=0; i--)
	{
		if (allVoxel[i].AllPoints.size() < 3)
		{			
			AddVoxel.push_back(allVoxel[i]);
			allVoxel.erase(allVoxel.begin() + i);
			continue;
		}
		//计算中心点
		float sumX, sumY, sumZ;
		sumX = 0; sumY = 0; sumZ = 0;
		for (int j = 0; j < allVoxel[i].AllPoints.size(); j++)
		{
			sumX += allVoxel[i].AllPoints[j].X;
			sumY += allVoxel[i].AllPoints[j].Y;
			sumZ += allVoxel[i].AllPoints[j].Z;
		}
		float avgX, avgY, avgZ;
		avgX = avgY = avgZ = 0;
		avgX = sumX / allVoxel[i].AllPoints.size();
		avgY = sumY / allVoxel[i].AllPoints.size();
		avgZ = sumZ / allVoxel[i].AllPoints.size();
		allVoxel[i].CenterPoint.X = avgX;
		allVoxel[i].CenterPoint.Y = avgY;
		allVoxel[i].CenterPoint.Z = avgZ;
		allVoxel[i].CenterPoint.label = allVoxel[i].AllPoints[0].label;
		//计算点数
		allVoxel[i].PointCount = allVoxel[i].AllPoints.size();
		allVoxel[i].Weight = allVoxel[i].AllPoints.size();
		allVoxel[i].ID = i;

		//最大似然法计算法向量
		std::vector<float> Def(allVoxel[i].AllPoints.size());
		for (int m = 0; m < allVoxel[i].AllPoints.size(); m++)
		{
			for (int n = 0; n < allVoxel[i].AllPoints.size(); n++)
			{
				Def[m] += CalAngle_TwoVector(allVoxel[i].AllPoints[m], allVoxel[i].AllPoints[n]);
			}
		}
		int MinAngleIndex = 0;
		float MinAngle = 9999.0f;
		for (int m = 0; m < Def.size(); m++)
		{
			if (Def[m] < MinAngle)
			{
				MinAngleIndex = m;
			}
		}
		if (allVoxel[i].AllPoints[MinAngleIndex].pNormal.NZ < 0)
		{
			allVoxel[i].CenterPoint.pNormal.NX = allVoxel[i].normal.NX = allVoxel[i].AllPoints[MinAngleIndex].pNormal.NX*(-1);
			allVoxel[i].CenterPoint.pNormal.NY = allVoxel[i].normal.NY = allVoxel[i].AllPoints[MinAngleIndex].pNormal.NY*(-1);
			allVoxel[i].CenterPoint.pNormal.NZ = allVoxel[i].normal.NZ = allVoxel[i].AllPoints[MinAngleIndex].pNormal.NZ*(-1);
		}
		else
		{
			allVoxel[i].CenterPoint.pNormal.NX = allVoxel[i].normal.NX = allVoxel[i].AllPoints[MinAngleIndex].pNormal.NX;
			allVoxel[i].CenterPoint.pNormal.NY = allVoxel[i].normal.NY = allVoxel[i].AllPoints[MinAngleIndex].pNormal.NY;
			allVoxel[i].CenterPoint.pNormal.NZ = allVoxel[i].normal.NZ = allVoxel[i].AllPoints[MinAngleIndex].pNormal.NZ;
		}

		//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>());		
		//pcl::PointXYZ tempp1;
		//tempp1.x = allVoxel[i].CenterPoint.X; tempp1.y = allVoxel[i].CenterPoint.Y; tempp1.z = allVoxel[i].CenterPoint.Z;
		//cloud1->points.push_back(tempp1);
		//for (int j = 0; j < allVoxel[i].AllPoints.size(); j++)
		//{
		//	tempp1.x = allVoxel[i].AllPoints[j].X; tempp1.y = allVoxel[i].AllPoints[j].Y; tempp1.z = allVoxel[i].AllPoints[j].Z;
		//	cloud1->points.push_back(tempp1);
		//}
		////估计法向量
		//pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
		//ne.setInputCloud(cloud1);
		//pcl::search::KdTree<pcl::PointXYZ>::Ptr tree1(new pcl::search::KdTree<pcl::PointXYZ>());
		//ne.setSearchMethod(tree1);
		//pcl::PointCloud<pcl::Normal>::Ptr cloud_normals1(new pcl::PointCloud<pcl::Normal>);
		//ne.setKSearch(allVoxel[i].AllPoints.size());
		//ne.compute(*cloud_normals1);
		//if (cloud_normals1->points[0].normal_z > 0)
		//{
		//	allVoxel[i].normal.NX=allVoxel[i].CenterPoint.pNormal.NX = cloud_normals1->points[0].normal_x;
		//	allVoxel[i].normal.NY =allVoxel[i].CenterPoint.pNormal.NY= cloud_normals1->points[0].normal_y;
		//	allVoxel[i].normal.NZ =allVoxel[i].CenterPoint.pNormal.NZ= cloud_normals1->points[0].normal_z;
		//}
		//else
		//{
		//	allVoxel[i].normal.NX = allVoxel[i].CenterPoint.pNormal.NX = cloud_normals1->points[0].normal_x * (-1);
		//	allVoxel[i].normal.NY = allVoxel[i].CenterPoint.pNormal.NY = cloud_normals1->points[0].normal_y * (-1);
		//	allVoxel[i].normal.NZ = allVoxel[i].CenterPoint.pNormal.NZ = cloud_normals1->points[0].normal_z * (-1);
		//}
	}
	SuperVoxel s1;
	s1.ID = 0;
	s1.PointCount = 1;
	s1.Weight = 1;
	for (int i = 0; i < AddVoxel.size(); i++)
	{
		for (int j = 0; j < AddVoxel[i].AllPoints.size(); j++)
		{
			s1.AllPoints.push_back(AddVoxel[i].AllPoints[j]);
			s1.CenterPoint = AddVoxel[i].AllPoints[j];
			s1.normal = AddVoxel[i].AllPoints[j].pNormal;
			allVoxel.push_back(s1);
			s1.AllPoints.clear();
		}
	}
}

void SuperVoxelsEstimate(vector<SuperVoxel> &allVoxels)
{
	int voxelCount = allVoxels.size();
	std::vector<SuperVoxel> DaSan;
	for (int i = voxelCount - 1; i >= 0; i--)
	{
		if (allVoxels[i].AllPoints.size() < 4)
		{
			DaSan.push_back(allVoxels[i]);
			allVoxels.erase(allVoxels.begin() + i);
			continue;
		}
		int UnFitCount = 0;
		int AllCount = allVoxels[i].AllPoints.size();
		std::vector<float> allDis;
		float a, b, c, d;
		a = allVoxels[i].normal.NX; b = allVoxels[i].normal.NY; c = allVoxels[i].normal.NZ;
		d = (-1)*(a*allVoxels[i].CenterPoint.X + b * allVoxels[i].CenterPoint.Y + c * allVoxels[i].CenterPoint.Z);
		float dis = 0;
		for (int L = 0; L < allVoxels[i].AllPoints.size(); L++)
		{
			dis = abs(a * allVoxels[i].AllPoints[L].X + b * allVoxels[i].AllPoints[L].Y + c * allVoxels[i].AllPoints[L].Z + d);
			allDis.push_back(dis);
		}
		float SD = CalSD(allDis);//计算标准差
		
		//将超体素分离，得到多个点(超体素)
		int allDisSize = allDis.size();

		if (allDisSize >= 12)
		{
			int aa = 0;
		}

		SuperVoxel ss1;
		ss1.ID = 0; ss1.PointCount = ss1.Weight = 1;

		for (int j = allDisSize - 1; j >= 0; j--)
		{
			if (fabsf( allDis[j] )> (SD * 1.5))
			{

				ss1.AllPoints.push_back(allVoxels[i].AllPoints[j]);
				ss1.CenterPoint.X = ss1.AllPoints[0].X; ss1.CenterPoint.Y = ss1.AllPoints[0].Y; ss1.CenterPoint.Z = ss1.AllPoints[0].Z;
				ss1.normal = ss1.AllPoints[0].pNormal;
				DaSan.push_back(ss1);
				ss1.AllPoints.clear();

				allVoxels[i].AllPoints.erase(allVoxels[i].AllPoints.begin() + j);
				UnFitCount++;
			}
		}
		float UnFitRatio = 1.0f*UnFitCount / AllCount;
		if ((allVoxels[i].AllPoints.size() - UnFitCount) < 4 || UnFitRatio > 0.2)
		{
			DaSan.push_back(allVoxels[i]);
			allVoxels.erase(allVoxels.begin() + i);
		}
	}

	SuperVoxel s1;
	s1.ID = 0;
	s1.PointCount = s1.Weight = 1;
	for (int i = 0; i < DaSan.size(); i++)
	{
		for (int j = 0; j < DaSan[i].AllPoints.size(); j++)
		{			
			s1.AllPoints.push_back(DaSan[i].AllPoints[j]);
			s1.CenterPoint = DaSan[i].AllPoints[j];			
			s1.normal = DaSan[i].AllPoints[j].pNormal;			
			allVoxels.push_back(s1);
			s1.AllPoints.clear();
		}
	}
}

vector<vector<SuperVoxel>> DivideToHouse(vector<SuperVoxel> allVoxels)
{
	vector<int> pLab;
	pcl::PointXYZ tempp;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
	for (int i = 0; i < allVoxels.size(); i++)
	{
		for (int j = 0; j < allVoxels[i].AllPoints.size(); j++)
		{
			pLab.push_back(i);
			tempp.x = allVoxels[i].AllPoints[j].X;
			tempp.y = allVoxels[i].AllPoints[j].Y;
			tempp.z = allVoxels[i].AllPoints[j].Z;
			cloud->points.push_back(tempp);
		}
	}
	//建立二维数组
	//vector<vector<int>> VV(allVoxels.size(), vector<int>(allVoxels.size()));
	//建立无向图结构
	std::vector<GraphType1> Nei_Graph(allVoxels.size());
	std::vector<int> lab(allVoxels.size());
	for (int i = 0; i < Nei_Graph.size(); i++)
	{
		Nei_Graph[i].ID = i;
		lab[i] = i;
	}
	std::vector<int> Nei_Index;
	std::vector<int>::iterator ite;
	//建立kd树进行邻域搜索
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(cloud);//传入点云
	vector<int> nearPointIndex;//邻域点的id
	vector<float> nearPointDistance;//邻域点的距离
	int Neighbor1 = 0;
	int Neighbor2 = 0;
	for (int i = 0; i < cloud->points.size(); i++)
	{
		tempp = cloud->points[i];
		if (kdtree.nearestKSearch(tempp, 7, nearPointIndex, nearPointDistance) > 0)
		{
			for (int j = 1; j < nearPointIndex.size(); j++)
			{
				Neighbor1 = pLab[i];
				Neighbor2 = pLab[nearPointIndex[j]];
				if (Neighbor1 != Neighbor2)
				{
					//VV[Neighbor1][Neighbor2] = 1;
					//无向图
					for (int k = 0; k < Nei_Graph[Neighbor1].AllNeibors.size(); k++)
					{
						Nei_Index.push_back(Nei_Graph[Neighbor1].AllNeibors[k]->ID);
					}
					ite = std::find(Nei_Index.begin(), Nei_Index.end(), Neighbor2);
					if (ite == Nei_Index.end())
					{
						Nei_Graph[Neighbor1].AllNeibors.push_back(&Nei_Graph[Neighbor2]);
					}
					Nei_Index.clear();
				}
			}
		}
	}

	//将得到的邻近数组处理得到各个房屋板块
	//vector<int> selected(VV.size());
	//int VoxelCount = 0;
	//vector<vector<int>> allIndex;
	//while (VoxelCount != VV.size())
	//{
	//	for (int i = 0; i < selected.size(); i++)
	//	{
	//		if (selected[i] == 0)
	//		{
	//			vector<int> partIndex;
	//			vector<int> waitingIndex;
	//			int StartIndex = i;
	//			waitingIndex.push_back(StartIndex);
	//			selected[StartIndex] = 1;
	//			while (waitingIndex.size() > 0)
	//			{
	//				int CurrentIndex = waitingIndex[0];
	//				for (int j = 0; j < VV.size(); j++)
	//				{
	//					if (selected[j] == 0 && VV[CurrentIndex][j] == 1)
	//					{
	//						waitingIndex.push_back(j);
	//						selected[j] = 1;
	//					}
	//				}
	//				partIndex.push_back(CurrentIndex);
	//				waitingIndex.erase(waitingIndex.begin());
	//			}
	//			allIndex.push_back(partIndex);
	//			VoxelCount += partIndex.size();
	//		}
	//	}
	//}

	//无向图分割
	std::vector<std::vector<int>> Result = DealGraph_DealIndex(&Nei_Graph, lab);

	vector<vector<SuperVoxel>> DivideToHouses;
	vector<SuperVoxel> AHouse;
	for (int i = 0; i < Result.size(); i++)
	{
		for (int j = 0; j < Result[i].size(); j++)
		{
			AHouse.push_back(allVoxels[Result[i][j]]);
		}
		DivideToHouses.push_back(AHouse);
		AHouse.clear();
	}

	return DivideToHouses;
}