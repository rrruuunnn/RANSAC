#include"RANSAC_Process.h"
vector<Point> InitPlane(vector<Point> allPoints)
{
	int Docount = 0;
	vector<Point> ThreePoints;

	//�޶�������ʹ������֮����С�нǴ���30��
	vector<float> Angles(3);
	int allPointsCount = allPoints.size();
	if (allPointsCount < 3)
	{
		cout << "�������С��3" << endl;
		//system("pause");
	}
	Point tempp1 = { 0,0,0,0,0,0 }; Point tempp2 = { 0,0,0,0,0,0 }; Point tempp3 = { 0,0,0,0,0,0 };
	while (Angles[0] < 30 || Angles[1] < 30 || Angles[2] < 30)
	{
		long RandN[3];
		getRand(allPoints.size(), RandN);
		int randNumber1 = RandN[0];
		int randNumber2 = RandN[1];
		int randNumber3 = RandN[2];
		tempp1 = allPoints[randNumber1];
		tempp2 = allPoints[randNumber2];
		tempp3 = allPoints[randNumber3];
		Angles = CalAngle_ThreePoints(tempp1, tempp2, tempp3);
		Docount++;
		if (Docount > 100)break;
	}
	ThreePoints.push_back(tempp1);
	ThreePoints.push_back(tempp2);
	ThreePoints.push_back(tempp3);
	return ThreePoints;
}

//����ƽ�淨����
Normal CalNormal_ThreePoints(vector<Point> Plane)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
	for (int i = 0; i < Plane.size(); i++)
	{
		pcl::PointXYZ temp;
		temp.x = Plane[i].X;
		temp.y = Plane[i].Y;
		temp.z = Plane[i].Z;
		cloud->points.push_back(temp);
	}
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud(cloud);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	ne.setSearchMethod(tree);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud < pcl::Normal >);
	ne.setRadiusSearch(100.0);
	ne.compute(*cloud_normals);//���Ƿ���������������

	float X1 = Plane[0].X; float Y1 = Plane[0].Y; float Z1 = Plane[0].Z;
	float X2 = Plane[1].X; float Y2 = Plane[1].Y; float Z2 = Plane[1].Z;
	float X3 = Plane[2].X; float Y3 = Plane[2].Y; float Z3 = Plane[2].Z;
	float deltaX1 = X2 - X1; float deltaY1 = Y2 - Y1; float deltaZ1 = Z2 - Z1;
	float deltaX2 = X3 - X1; float deltaY2 = Y3 - Y1; float deltaZ2 = Z3 - Z1;
	float X = deltaY1 * deltaZ2 - deltaZ1 * deltaY2;
	float Y = deltaZ1 * deltaX2 - deltaX1 * deltaZ2;
	float Z = deltaX1 * deltaY2 - deltaY1 * deltaX2;
	float SUMXYZ = sqrtf(X*X + Y * Y + Z * Z);
	float NX1 = X / SUMXYZ;
	float NY1 = Y / SUMXYZ;
	float NZ1 = Z / SUMXYZ;

	float NX = cloud_normals->points[0].normal_x;
	float NY = cloud_normals->points[0].normal_y;
	float NZ = cloud_normals->points[0].normal_z;
	if (NZ < 0)
	{
		NX *= (-1);
		NY *= (-1);
		NZ *= (-1);
	}
	Normal normal;
	normal.NX = NX;
	normal.NY = NY;
	normal.NZ = NZ;
	return normal;
}

std::vector<GraphType1>  TrianglePouFen(vector<SuperVoxel> AHouse)
{
	// ��һ��XYZ�����͵�PCD�ļ��򿪲��洢��������
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointXYZ tempp;
	for (int i = 0; i < AHouse.size(); i++)
	{
		/*tempp.x = AHouse[i].CenterPoint.X;
		tempp.y = AHouse[i].CenterPoint.Y;
		tempp.z = AHouse[i].CenterPoint.Z;
		cloud->points.push_back(tempp);*/
		for (int j = 0; j < AHouse[i].AllPoints.size(); j++)
		{
			tempp.x = AHouse[i].AllPoints[j].X;
			tempp.y = AHouse[i].AllPoints[j].Y;
			tempp.z = AHouse[i].AllPoints[j].Z;
			cloud->points.push_back(tempp);
		}
	}
	//* the data should be available in cloud

	// Normal estimation*
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;      //���߹��ƶ���
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);   //�洢���Ƶķ���
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);  //����kd��ָ��
	tree->setInputCloud(cloud);   ///��cloud����tree����
	n.setInputCloud(cloud);
	n.setSearchMethod(tree);
	n.setKSearch(15);
	n.compute(*normals);       ////���Ʒ��ߴ洢������
	//* normals should not contain the point normals + surface curvatures

	// Concatenate the XYZ and normal fields*
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);    //�����ֶ�
	//* cloud_with_normals = cloud + normals

	//��������������
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
	tree2->setInputCloud(cloud_with_normals);   //���ƹ���������

	// Initialize objects
	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;   //�������ǻ�����
	pcl::PolygonMesh triangles;                //�洢�������ǻ�������ģ��

	// Set the maximum distance between connected points (maximum edge length)
	gp3.setSearchRadius(3);  //�������ӵ�֮��������룬���������������߳���

	// ���ø�����ֵ
	gp3.setMu(10);  //���ñ���������������ڵ����Զ����Ϊ2.5��Ϊ��ʹ�õ����ܶȵı仯
	gp3.setMaximumNearestNeighbors(20);    //������������������������
	gp3.setMaximumSurfaceAngle(M_PI / 4); // ����ĳ�㷨�߷���ƫ�������㷨�ߵ����Ƕ�45
	gp3.setMinimumAngle(M_PI / 18); // �������ǻ���õ����������ڽǵ���С�ĽǶ�Ϊ10
	gp3.setMaximumAngle(2 * M_PI / 3); // �������ǻ���õ����������ڽǵ����Ƕ�Ϊ120
	gp3.setNormalConsistency(false);  //���øò�����֤���߳���һ��

	// Get result
	gp3.setInputCloud(cloud_with_normals);     //�����������Ϊ�������
	gp3.setSearchMethod(tree2);   //����������ʽ
	gp3.reconstruct(triangles);  //�ؽ���ȡ���ǻ�		

	//������֮����ڽӹ�ϵ
	vector<vector<int>> NeiAdj(1, vector<int>(0));
	vector<int> pLab;
	for (int i = 0; i < AHouse.size(); i++)
	{
		for (int j = 0; j < AHouse[i].AllPoints.size(); j++)
		{
			pLab.push_back(i);
		}
	}
	//���ö����ͼ�ṹ
	std::vector<GraphType1> AllGraph(AHouse.size());
	for (int i = 0; i < AHouse.size(); i++)
	{
		AllGraph[i].ID = i;
	}
	int Tri1, Tri2, Tri3;//����������
	Tri1 = Tri2 = Tri3 = 0;
	int AHouseIndex1, AHouseIndex2, AHouseIndex3;//���ݸ�����������
	AHouseIndex1 = AHouseIndex2 = AHouseIndex3 = 0;
	vector<int>::iterator ret;
	vector<int> AllNeiBors1;
	vector<int> AllNeiBors2;
	for (int i = 0; i < triangles.polygons.size(); i++)
	{
		Tri1 = triangles.polygons[i].vertices[0];
		Tri2 = triangles.polygons[i].vertices[1];
		Tri3 = triangles.polygons[i].vertices[2];
		AHouseIndex1 = pLab[Tri1];
		AHouseIndex2 = pLab[Tri2];
		AHouseIndex3 = pLab[Tri3];
		if (AHouseIndex1 != AHouseIndex2)
		{
			for (int j = 0; j < AllGraph[AHouseIndex1].AllNeibors.size(); j++)
			{
				AllNeiBors1.push_back(AllGraph[AHouseIndex1].AllNeibors[j]->ID);
			}
			ret = std::find(AllNeiBors1.begin(), AllNeiBors1.end(), AHouseIndex2);
			if (ret == AllNeiBors1.end())
			{
				AllGraph[AHouseIndex1].AllNeibors.push_back(&AllGraph[AHouseIndex2]);
			}
			AllNeiBors1.clear();
			for (int j = 0; j < AllGraph[AHouseIndex2].AllNeibors.size(); j++)
			{
				AllNeiBors2.push_back(AllGraph[AHouseIndex2].AllNeibors[j]->ID);
			}
			ret = std::find(AllNeiBors2.begin(), AllNeiBors2.end(), AHouseIndex1);
			if (ret == AllNeiBors2.end())
			{
				AllGraph[AHouseIndex2].AllNeibors.push_back(&AllGraph[AHouseIndex1]);
			}
			AllNeiBors2.clear();
		}
		if (AHouseIndex1 != AHouseIndex3)
		{
			for (int j = 0; j < AllGraph[AHouseIndex1].AllNeibors.size(); j++)
			{
				AllNeiBors1.push_back(AllGraph[AHouseIndex1].AllNeibors[j]->ID);
			}
			ret = std::find(AllNeiBors1.begin(), AllNeiBors1.end(), AHouseIndex3);
			if (ret == AllNeiBors1.end())
			{
				AllGraph[AHouseIndex1].AllNeibors.push_back(&AllGraph[AHouseIndex3]);
			}
			AllNeiBors1.clear();
			for (int j = 0; j < AllGraph[AHouseIndex3].AllNeibors.size(); j++)
			{
				AllNeiBors2.push_back(AllGraph[AHouseIndex3].AllNeibors[j]->ID);
			}
			ret = std::find(AllNeiBors2.begin(), AllNeiBors2.end(), AHouseIndex1);
			if (ret == AllNeiBors2.end())
			{
				AllGraph[AHouseIndex3].AllNeibors.push_back(&AllGraph[AHouseIndex1]);
			}
			AllNeiBors2.clear();
		}
		if (AHouseIndex2 != AHouseIndex3)
		{
			for (int j = 0; j < AllGraph[AHouseIndex2].AllNeibors.size(); j++)
			{
				AllNeiBors1.push_back(AllGraph[AHouseIndex2].AllNeibors[j]->ID);
			}
			ret = std::find(AllNeiBors1.begin(), AllNeiBors1.end(), AHouseIndex3);
			if (ret == AllNeiBors1.end())
			{
				AllGraph[AHouseIndex2].AllNeibors.push_back(&AllGraph[AHouseIndex3]);
			}
			AllNeiBors1.clear();
			for (int j = 0; j < AllGraph[AHouseIndex3].AllNeibors.size(); j++)
			{
				AllNeiBors2.push_back(AllGraph[AHouseIndex3].AllNeibors[j]->ID);
			}
			ret = std::find(AllNeiBors2.begin(), AllNeiBors2.end(), AHouseIndex2);
			if (ret == AllNeiBors2.end())
			{
				AllGraph[AHouseIndex3].AllNeibors.push_back(&AllGraph[AHouseIndex2]);
			}
			AllNeiBors2.clear();
		}
	}
	return AllGraph;
}


double CalThreshold1(std::vector<SuperVoxel> CurrentVoxels, double InitRatio, Normal N, double d, double Threshold)
{
	std::vector<double> Error;
	double error = 0;
	for (int i = 0; i < CurrentVoxels.size(); i++)
	{
		for (int j = 0; j < CurrentVoxels[i].AllPoints.size(); j++)
		{
			error = abs(N.NX*CurrentVoxels[i].AllPoints[j].X + N.NY*CurrentVoxels[i].AllPoints[j].Y + N.NZ*CurrentVoxels[i].AllPoints[j].Z + d) / sqrt(N.NX*N.NX + N.NY*N.NY + N.NZ*N.NZ);
			Error.push_back(error);
		}
	}

	sort(Error.begin(), Error.end());

	float maxValue = *max_element(Error.begin(), Error.end());
	cout << "maxValue:" << maxValue << " " << N.NX << " " << N.NY << " " << N.NZ << " " << d << endl;

	double D = 1;
	double ratio0 = InitRatio; double ratio1 = ratio0;

	std::vector<double> Error1(Error.begin(), Error.begin() + (int)(ratio0*Error.size()));
	double SD0 = CalSD2(Error1);
	double SD1 = SD0;

	int LocalRunTime = 0;
	double V = maxValue;

	while (true)
	{
		std::vector<double> Zi;
		for (int i = 0; i < Error.size(); i++)
		{
			Zi.push_back(
				(ratio0 * pow((1 / sqrt(2 * M_PI) / SD0), D) / exp(pow(Error[i], 2) / 2 / SD0 / SD0))
				/
				(ratio0 * pow((1 / sqrt(2 * M_PI) / SD0), D) / exp(pow(Error[i], 2) / 2 / SD0 / SD0) + (1 - ratio0) / (2 * V))
			);
		}
		ratio0 = std::accumulate(Zi.begin(), Zi.end(), 0.0) / Zi.size();
		double ek1Up = 0;
		for (int i = 0; i < Error.size(); i++)
		{
			ek1Up += Zi[i] * pow(Error[i], 2);
		}
		SD0 = sqrt(ek1Up / std::accumulate(std::begin(Zi), std::end(Zi), 0.0));
		if (abs(ratio1 - ratio0) < 0.0001&&abs(SD1 - SD0) < 0.0001)
		{
			ratio1 = ratio0;
			SD1 = SD0;
			break;
		}
		else 
		{
			ratio1 = ratio0;
			SD1 = SD0;
		}
		if (LocalRunTime > 100)
		{
			cout << "���������" << endl;
			break;
		}
	}
	int FitIndex1 = 0;
	for (int i =0; i <Error.size(); i++)
	{
		if (Error[i] > 0.392)
		{
			FitIndex1 = i;
			break;
		}
	}
	int FitIndex2 = 0;
	for (int i = 0; i < Error.size(); i++)
	{
		if (Error[i] > (SD1*1.96))
		{
			FitIndex2 = i;
			break;
		}
	}
	//cout << "���۵������" << FitIndex1 << " ʵ�ʵ�����" << FitIndex2 << endl;
	return SD1;
}

double CalThreshold(vector<SuperVoxel> CurrentVoxels,double InitRatio, Normal N,double d, double Threshold)
{
	std::vector<double> Error;
	double error = 0;
	double error1 = 0;
	for (int i = 0; i < CurrentVoxels.size(); i++)
	{
		error = abs(CurrentVoxels[i].CenterPoint.X*N.NX + N.NY*CurrentVoxels[i].CenterPoint.Y + N.NZ*CurrentVoxels[i].CenterPoint.Z + d) / sqrt(N.NX*N.NX + N.NY*N.NY + N.NZ*N.NZ);
		
		std::vector<double> sumdis;
		for (int j = 0; j < CurrentVoxels[i].AllPoints.size(); j++)
		{
			sumdis.push_back(abs(CurrentVoxels[i].AllPoints[j].X*N.NX + N.NY*CurrentVoxels[i].AllPoints[j].Y + N.NZ*CurrentVoxels[i].AllPoints[j].Z + d) / sqrt(N.NX*N.NX + N.NY*N.NY + N.NZ*N.NZ));
		}
		//if ((std::accumulate(sumdis.begin(),sumdis.end(),0.0) / CurrentVoxels[i].AllPoints.size() - error) > 0.01)
		//{
		//	cout << "����ʧ��" << endl;
		//}
		error1 = std::accumulate(std::begin(sumdis), std::end(sumdis), 0.0) / sumdis.size();
		Error.push_back(error1);
	}

	//�ѽ����ӡ����
	FILE *ErrorTXT = fopen("ErrorTXT.txt", "w+");
	for (int i = 0; i < Error.size(); i++)
	{
		for (int j = 0; j < CurrentVoxels[i].AllPoints.size(); j++)
		{
			fprintf(ErrorTXT, "%f %f %f\n", CurrentVoxels[i].AllPoints[j].X, CurrentVoxels[i].AllPoints[j].Y, CurrentVoxels[i].AllPoints[j].Z);
		}
	}
	fclose(ErrorTXT);

	std::vector<int> Index;
	int pointsCount = 0;
	for (int i = 0; i < CurrentVoxels.size(); i++)
	{
		Index.push_back(CurrentVoxels[i].AllPoints.size());
		pointsCount += CurrentVoxels[i].AllPoints.size();
	}
	double tempE = 0;
	int tempI = 0;
	for (int i = 0; i < Error.size(); i++)
	{
		for (int j = i + 1; j < Error.size(); j++)
		{
			if (Error[i] > Error[j])
			{
				tempE = Error[i]; Error[i] = Error[j]; Error[j] = tempE;
				tempI = Index[i]; Index[i] = Index[j]; Index[j] = tempI;
			}
		}
	}
	
	//std::sort(Error.begin(), Error.end());

	double D = 1;
	double ratio0 = InitRatio; double ratio1 = ratio0;

	std::vector<double>Error1(Error.begin(), Error.begin() + (int)(ratio0*CurrentVoxels.size()));
	std::vector<int>Index1(Index.begin(), Index.begin() + (int)(ratio0*CurrentVoxels.size()));

	double temp = CalSD1(Error1,Index1);
	double SD0 = temp;
	double SD1 = SD0;

	int LocalRunTime = 0;

	double V = 3;

	while (true)
	{
		std::vector<double> Zi;
		for (int m = 0; m < CurrentVoxels.size(); m++)
		{
			Zi.push_back(
				(ratio0 * pow((1 / sqrt(2 * M_PI) / SD0), D) / exp(pow(Error[m], 2) / 2 / SD0 / SD0))
				/
				(ratio0 * pow((1 / sqrt(2 * M_PI) / SD0), D) / exp(pow(Error[m], 2) / 2 / SD0 / SD0) + (1 - ratio0) / (2 * V))
			);
		}

		//ratio0 = std::accumulate(std::begin(Zi), std::end(Zi), 0.0) / Zi.size();
		for (int i = 0; i < Zi.size(); i++)
		{
			ratio0 += Zi[i] * Index[i];
		}
		ratio0 /= pointsCount;

		double ek1Up = 0;
		double ek1Down = 0;
		for (int i = 0; i < CurrentVoxels.size(); i++)
		{
			ek1Up +=( Zi[i]*Index[i]) * pow(Error[i], 2);
			ek1Down += Zi[i] * Index[i];
		}

		SD0 = sqrt(ek1Up / ek1Down);

		if (abs(ratio0 - ratio1) < 0.00001&&abs(SD1 - SD0) < 0.00001)
		{
			ratio1 = ratio0;
			SD1 = SD0;
			break;
		}
		else
		{
			ratio1 = ratio0;
			SD1 = SD0;
		}

		LocalRunTime++;

		if (LocalRunTime > 100)
		{
			std::cout << "���������" << endl;
			break;
		}		
	}

	int FitIndex = 0;
	for (int i = 0; i < CurrentVoxels.size(); i++)
	{
		if (Error[i] > (SD1*1.96))
		{
			FitIndex = i;
			break;
		}
	}

	int FitIndex2 = 0;
	for (int i = 0; i < CurrentVoxels.size(); i++)
	{
		if (Error[i] > SD1)
		{
			FitIndex2 = i;
			break;
		}
	}

	int FitIndex1 = 0;
	for (int i = 0; i < CurrentVoxels.size(); i++)
	{
		if (Error[i] > 0.392)
		{
			FitIndex1 = i;
			break;
		}
	}
	//cout << "�����������ϵĸ�����" << FitIndex1 << " ʵ���ϵĸ�����" << FitIndex << endl;
	return SD1;
}

float VoteForPlane(vector<Point> Plane, vector<SuperVoxel> allVoxel, int32_t pointsCount, int& r, float& d, float angle_Threshold, float distance_Threshold)
{	
	r = 0;//r������¼���������;���ĳ����ظ���
	int PointCount = 0;//��¼ͶƱ���ܵ���
	float cosr = 0;
	float distancer = 0;
	float Score = 0;//�÷�
	Normal n_plane;
	n_plane = CalNormal_ThreePoints(Plane);//����ֻ��һ��ƽ���a,b,c������û��d����	
	d = (-1)*(n_plane.NX*Plane[0].X + n_plane.NY*Plane[0].Y + n_plane.NZ*Plane[0].Z);//�������������dֵ
	//allVoxel�еı���(ȫ�ֱ���)
	Point tempCenterPoint = { 0,0,0,0,0,0 };//���ĵ�
	Normal tempNormal = { 0,0,0 };//������
	//�����������
	float m = allVoxel.size();//�����ظ���

	for (int i = 0; i < m; i++)
	{
		tempCenterPoint = allVoxel[i].CenterPoint;
		tempNormal = allVoxel[i].normal;
		//�������ƽ��������֮�������ֵ
		float cos1 = (n_plane.NX*tempNormal.NX + n_plane.NY*tempNormal.NY + n_plane.NZ*tempNormal.NZ) / (sqrtf(n_plane.NX*n_plane.NX + n_plane.NY*n_plane.NY + n_plane.NZ*n_plane.NZ)*sqrtf(tempNormal.NX*tempNormal.NX + tempNormal.NY*tempNormal.NY + tempNormal.NZ*tempNormal.NZ));
		cos1 = fabsf(cos1);

		cosr = cos1;

		if (cos1 < angle_Threshold)
		{
			cos1 = angle_Threshold;
		}
		//�����������ĵ㵽����ƽ��ľ���
		float distance = fabsf(n_plane.NX*tempCenterPoint.X + n_plane.NY*tempCenterPoint.Y + n_plane.NZ*tempCenterPoint.Z + d) / sqrtf(n_plane.NX*n_plane.NX + n_plane.NY*n_plane.NY + n_plane.NZ*n_plane.NZ);

		distancer = distance;

		if (distance >= distance_Threshold)
		{
			distance = distance_Threshold;
		}
		Score += (1/(1-angle_Threshold) * fabsf(cos1) - (1/(1-angle_Threshold)-1))*( - (1/distance_Threshold)*distance + 1) * allVoxel[i].AllPoints.size();//�������н�0.95->1��0->1��;����0-0.5m��1->0��
		PointCount += allVoxel[i].AllPoints.size();

		if (cosr > angle_Threshold &&distancer < distance_Threshold)
		{
			r++;
		}
	}
	Score /= PointCount;
	return Score;
}

vector<int> FindBestPlane(vector<SuperVoxel> allVoxel, float &angle_Threshold, float &distance_Threshold)
{
	vector<Point> allPoints;
	for (int i = 0; i < allVoxel.size(); i++)
	{
		for (int j = 0; j < allVoxel[i].AllPoints.size(); j++)
		{
			allPoints.push_back(allVoxel[i].AllPoints[j]);
		}
	}

	float MaxScore = 0;
	float Score = 0;
	float m = allVoxel.size();//�����ظ���
	Normal MaxNormal;
	MaxNormal.NX = MaxNormal.NY = MaxNormal.NZ = 0;
	vector<Point> Plane(3);
	vector<Point> PlaneBest(3);//��ĿǰΪֹ��õ�ƽ��
	float d = 0;
	float rightd = 0;
	int DoCount = 0;
	//������Сѭ������
	int Fit = 0;
	float r = 0;
	int Intera = 9999;
	int CurrentIntera = 9999;
	float MultiCount = 0;
	float SingleCount = 0;
	for (int i = 0; i < allVoxel.size(); i++)
	{
		if (allVoxel[i].AllPoints.size() > 1)
		{
			MultiCount++;
		}
		else
		{
			SingleCount++;
		}
	}
	float AvgCount = MultiCount + SingleCount / 6;
	while (DoCount < Intera)
	{
		if (allPoints.size() < 10)
		{
			DoCount++;
			continue;
		}
		Plane = InitPlane(allPoints);
		Score = VoteForPlane(Plane, allVoxel, allPoints.size(), ref(Fit), ref(d), angle_Threshold, distance_Threshold);
		//����ѭ������
		r = 1.0f* Fit / AvgCount;//�ڵ����
		if (r >= 1)
		{
			r = 0.99;
		}
		if (r < 0.05)
		{
			CurrentIntera = 9999;
		}
		else
		{
			CurrentIntera = log(100) / log(1.0f / (1 - r * r*r));//�����ڵ���ʼ�����С��������
		}
		if (MaxScore < Score)
		{
			MaxScore = Score;
			MaxNormal = CalNormal_ThreePoints(Plane);
			for (int i = 0; i < 3; i++)
			{
				PlaneBest[i].X = Plane[i].X;
				PlaneBest[i].Y = Plane[i].Y;
				PlaneBest[i].Z = Plane[i].Z;
			}
		}
		if (Intera > CurrentIntera)
		{
			Intera = CurrentIntera;
		}
		if (Intera < 100)//�涨�������ٵ���100��
			Intera = 100;
		DoCount++;
	}
	if (MaxNormal.NX == 0 || PlaneBest[0].X == 0)//����Ҳ������ʵļ���ƽ���������ڸ����ش�����ȡ��
	{
		vector<int> res;
		//cout << "û�ҵ����ʵ���" << endl;
		return res;
	}
	float d2 = (-1)*(MaxNormal.NX*PlaneBest[0].X + MaxNormal.NY*PlaneBest[0].Y + MaxNormal.NZ*PlaneBest[0].Z);

	//��������Ӧ����
	

	Point tempCenterPoint = { 0,0,0,0,0,0 };
	Normal tempNormal = { 0,0,0 };
	
	//�����Ѿ�����ĳ�����
	vector<int> erase;
	int GetFace = 0;

	for (int i = 0; i < m; i++)
	{
		tempCenterPoint = allVoxel[i].CenterPoint;
		tempNormal = allVoxel[i].normal;
		float cos = (MaxNormal.NX*tempNormal.NX + MaxNormal.NY*tempNormal.NY + MaxNormal.NZ*tempNormal.NZ) / (sqrtf(MaxNormal.NX*MaxNormal.NX + MaxNormal.NY*MaxNormal.NY + MaxNormal.NZ*MaxNormal.NZ)*sqrtf(tempNormal.NX*tempNormal.NX + tempNormal.NY*tempNormal.NY + tempNormal.NZ*tempNormal.NZ));
		float distance = fabsf(MaxNormal.NX*tempCenterPoint.X + MaxNormal.NY*tempCenterPoint.Y + MaxNormal.NZ*tempCenterPoint.Z + d2) / sqrtf(MaxNormal.NX*MaxNormal.NX + MaxNormal.NY*MaxNormal.NY + MaxNormal.NZ*MaxNormal.NZ);
		if (fabsf(cos) >= angle_Threshold&&fabsf(distance) <= distance_Threshold)
		{
			GetFace++;
			erase.push_back(allVoxel[i].ID);
		}
		
	}	
	if (erase.size() == 0)
		cout << "���ҵ��ˣ������ز��ϸ�" << endl;
	//cout << "��" << erase.size() << "���һ����" << endl;	
	return erase;//���ݷ�����������ж������ر���ȡ������
}

double FindBestThreshold(vector<SuperVoxel> allVoxel, float &angle_Threshold, float &distance_Threshold)
{
	vector<Point> allPoints;
	for (int i = 0; i < allVoxel.size(); i++)
	{
		for (int j = 0; j < allVoxel[i].AllPoints.size(); j++)
		{
			allPoints.push_back(allVoxel[i].AllPoints[j]);
		}
	}

	float MaxScore = 0;
	float Score = 0;
	float m = allVoxel.size();//�����ظ���
	Normal MaxNormal;
	MaxNormal.NX = MaxNormal.NY = MaxNormal.NZ = 0;
	vector<Point> Plane(3);
	vector<Point> PlaneBest(3);//��ĿǰΪֹ��õ�ƽ��
	float d = 0;
	float rightd = 0;
	int DoCount = 0;
	//������Сѭ������
	int Fit = 0;
	float r = 0;
	int Intera = 9999;
	int CurrentIntera = 9999;
	float MultiCount = 0;
	float SingleCount = 0;
	for (int i = 0; i < allVoxel.size(); i++)
	{
		if (allVoxel[i].AllPoints.size() > 1)
		{
			MultiCount++;
		}
		else
		{
			SingleCount++;
		}
	}
	float AvgCount = MultiCount + SingleCount / 6;
	while (DoCount < Intera)
	{
		if (allPoints.size() < 10)
		{
			DoCount++;
			continue;
		}
		Plane = InitPlane(allPoints);
		Score = VoteForPlane(Plane, allVoxel, allPoints.size(), ref(Fit), ref(d), angle_Threshold, distance_Threshold);
		//����ѭ������
		r = 1.0f* Fit / AvgCount;//�ڵ����
		if (r >= 1)
		{
			r = 0.99;
		}
		if (r < 0.05)
		{
			CurrentIntera = 9999;
		}
		else
		{
			CurrentIntera = log(100) / log(1.0f / (1 - r * r*r));//�����ڵ���ʼ�����С��������
		}
		if (MaxScore < Score)
		{
			MaxScore = Score;
			MaxNormal = CalNormal_ThreePoints(Plane);
			for (int i = 0; i < 3; i++)
			{
				PlaneBest[i].X = Plane[i].X;
				PlaneBest[i].Y = Plane[i].Y;
				PlaneBest[i].Z = Plane[i].Z;
			}
		}
		if (Intera > CurrentIntera)
		{
			Intera = CurrentIntera;
		}
		if (Intera < 100)//�涨�������ٵ���100��
			Intera = 100;
		DoCount++;
	}
	if (MaxNormal.NX == 0 || PlaneBest[0].X == 0)//����Ҳ������ʵļ���ƽ���������ڸ����ش�����ȡ��
	{
		cout << "����Ӧ��ֵ������û�ҵ����ʵ���" << endl;
		return 0;
	}
	float d2 = (-1)*(MaxNormal.NX*PlaneBest[0].X + MaxNormal.NY*PlaneBest[0].Y + MaxNormal.NZ*PlaneBest[0].Z);

	//��������Ӧ����


	Point tempCenterPoint = { 0,0,0,0,0,0 };
	Normal tempNormal = { 0,0,0 };

	//�����Ѿ�����ĳ�����
	vector<int> erase;
	int GetFace = 0;

	for (int i = 0; i < m; i++)
	{
		tempCenterPoint = allVoxel[i].CenterPoint;
		tempNormal = allVoxel[i].normal;
		float cos = (MaxNormal.NX*tempNormal.NX + MaxNormal.NY*tempNormal.NY + MaxNormal.NZ*tempNormal.NZ) / (sqrtf(MaxNormal.NX*MaxNormal.NX + MaxNormal.NY*MaxNormal.NY + MaxNormal.NZ*MaxNormal.NZ)*sqrtf(tempNormal.NX*tempNormal.NX + tempNormal.NY*tempNormal.NY + tempNormal.NZ*tempNormal.NZ));
		float distance = fabsf(MaxNormal.NX*tempCenterPoint.X + MaxNormal.NY*tempCenterPoint.Y + MaxNormal.NZ*tempCenterPoint.Z + d2) / sqrtf(MaxNormal.NX*MaxNormal.NX + MaxNormal.NY*MaxNormal.NY + MaxNormal.NZ*MaxNormal.NZ);
		if (fabsf(cos) >= angle_Threshold && fabsf(distance) <= distance_Threshold)
		{
			GetFace++;
			erase.push_back(allVoxel[i].ID);
		}

	}
	if (erase.size() == 0)
		cout << "���ҵ��ˣ������ز��ϸ�" << endl;
	//cout << "��" << erase.size() << "���һ����" << endl;

	//��������Ӧ��ֵ�������
	int FitCount = 0;
	float sumX1, sumY1, sumZ1;
	sumX1 = sumY1 = sumZ1 = 0;
	pcl::PointCloud<pcl::PointXYZ>::Ptr ps(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointXYZ p1;

	for (int m = 0; m < erase.size(); m++)
	{
		FitCount += allVoxel[erase[m]].AllPoints.size();

		p1.x = allVoxel[erase[m]].CenterPoint.X;
		p1.y = allVoxel[erase[m]].CenterPoint.Y;
		p1.z = allVoxel[erase[m]].CenterPoint.Z;
		ps->points.push_back(p1);

		sumX1 += allVoxel[erase[m]].CenterPoint.X;
		sumY1 += allVoxel[erase[m]].CenterPoint.Y;
		sumZ1 += allVoxel[erase[m]].CenterPoint.Z;
	}
	
	float avgX1 = sumX1 / erase.size();
	float avgY1 = sumY1 / erase.size();
	float avgZ1 = sumZ1 / erase.size();
	pcl::PointCloud<pcl::PointXYZ>::Ptr c1(new pcl::PointCloud<pcl::PointXYZ>());
	p1.x = avgX1; p1.y = avgY1; p1.z = avgZ1;
	c1->points.push_back(p1);


	Point avgNormal = CalNormal(c1, ps);
	float avgD = (-1)*(avgNormal.pNormal.NX*avgX1 + avgNormal.pNormal.NY*avgY1 + avgNormal.pNormal.NZ*avgZ1);

	int pointcount = 0;
	for (int i = 0; i < allVoxel.size(); i++)
	{
		pointcount += allVoxel[i].AllPoints.size();
	}

	Normal n1;
	n1 = avgNormal.pNormal;
	//n1.NX = n1.NY = 0; n1.NZ = 1; avgD = 0.0001;
	//cout << n1.NX << " " << n1.NY << " " << n1.NZ << " " << avgD << endl;
	//cout << 1.0*erase.size() / allVoxel.size() << endl;

	FILE *fp1 = fopen("PointWeight.txt", "w+");
	for (int i = 0; i < allVoxel.size(); i++)
	{
		fprintf(fp1, "%f %f %f %d\n", allVoxel[i].CenterPoint.X, allVoxel[i].CenterPoint.Y, allVoxel[i].CenterPoint.Z, allVoxel[i].AllPoints.size());
	}
	fclose(fp1);
	double Threshold1 = CalThreshold1(allVoxel, 1.0*FitCount / pointcount, n1/*MaxNormal*/, avgD, 0.5);
	double Threshold = CalThreshold(allVoxel, (1.0*erase.size() / allVoxel.size()), n1/*MaxNormal*/, avgD, 0.5);

	distance_Threshold = Threshold1 * 1.96;
	if (erase.size() < 3)
		Threshold1 = 0;

	//cout << "T1:" << Threshold1 * 1.96 << endl;
	//cout << "T2:" << Threshold * 1.96 << endl;
	//system("pause");

	return (Threshold1*1.96);//���ݷ�����������ж������ر���ȡ������
}

vector<vector<int>> DealGraph_DealIndex(vector<GraphType1>* ptrGraph, vector<int>VoxelID)
{
	vector<GraphType1> PartGraphs(VoxelID.size());
	for (int i = 0; i < PartGraphs.size(); i++)
	{
		PartGraphs[i].ID = VoxelID[i];
	}
	//������������ͼ���ڽӹ�ϵ
	for (int i = 0; i < PartGraphs.size(); i++)
	{
		int ID = PartGraphs[i].ID;
		vector<int> AllNeiBors;
		for (int j = 0; j < (*ptrGraph)[ID].AllNeibors.size(); j++)
		{
			AllNeiBors.push_back((*ptrGraph)[ID].AllNeibors[j]->ID);
		}
		vector<int> Res = findSame(AllNeiBors, VoxelID);
		//��������
		vector<int>::iterator ite;
		int Index = 0;
		for (int j = 0; j < Res.size(); j++)
		{
			ite = std::find(std::begin(VoxelID), std::end(VoxelID), Res[j]);
			Index = std::distance(VoxelID.begin(), ite);
			PartGraphs[i].AllNeibors.push_back(&PartGraphs[Index]);
		}
	}

	vector<vector<int>> Result;
	int VoxelIDSize = VoxelID.size();//�����С
	int SelectedCount = 0;//�Ѿ�ѡ���˶��ٸ�����
	int CurrentSelectIndex = 0;
	vector<int> Selected(VoxelID.size());
	while (SelectedCount != VoxelIDSize)
	{
		if (SelectedCount > VoxelIDSize)
		{
			cout << "�����߼�����!" << endl;
			system("pause");
			break;
		}
		else
		{
			while (Selected[CurrentSelectIndex] == 1)
			{
				CurrentSelectIndex++;
			}
			vector<int>* tempResult = new vector<int>;
			DealGraph_DealIndexAssist(&(PartGraphs[CurrentSelectIndex]), &Selected, &SelectedCount, tempResult, VoxelID);
			vector<int> TempResult;
			for (int i = 0; i < (*tempResult).size(); i++)
			{
				TempResult.push_back((*tempResult)[i]);
			}
			Result.push_back(TempResult);
			delete(tempResult);
			TempResult.clear();
		}
	}
	return Result;
}

void DealGraph_DealIndexAssist(GraphType1* OneGraph, vector<int>*Selected, int *SelectedCount, vector<int>*tempResult, vector<int> VoxelID)
{
	tempResult->push_back(OneGraph->ID);
	//cout << "���ظ���:" << tempResult->size() << endl;
	int Index = 0;
	vector<int>::iterator ite;
	ite = std::find(VoxelID.begin(), VoxelID.end(), OneGraph->ID);
	Index = std::distance(VoxelID.begin(), ite);
	(*Selected)[Index] = 1;
	(*SelectedCount)++;
	if (OneGraph->AllNeibors.size() > 0)
	{
		for (int i = 0; i < OneGraph->AllNeibors.size(); i++)
		{
			if ((*Selected)[FindIndexInVector(OneGraph->AllNeibors[i]->ID, VoxelID)] != 1)
			{
				DealGraph_DealIndexAssist(OneGraph->AllNeibors[i], Selected, SelectedCount, tempResult, VoxelID);
			}
		}
	}
}

R1 dealOnePartVoxel1Add(vector<SuperVoxel> *ptrPartVoxels, vector<GraphType1> *ptrGraph, vector<int> ptrDealingIndex,vector<int> *NotSort, float angle_Threshold, float distance_Threshold)
{	
	vector<SuperVoxel> WaitingVoxel;
	for (int i = 0; i < ptrDealingIndex.size(); i++)
		WaitingVoxel.push_back((*ptrPartVoxels)[ptrDealingIndex[i]]);//���г�������ͨ��ID�����Ҫ���������

	vector<int> VoxelID = FindBestPlane(WaitingVoxel, angle_Threshold, distance_Threshold);//���ط��ϵĳ����ص�ID����������Ҫ������ID�����ط�Ϊ�����
	
	vector<vector<int>> facesID;
	//��ptrGraph�е��ڽӹ�ϵ����VoxelID��Ϊ�����	
	facesID = DealGraph_DealIndex(ptrGraph, VoxelID);
	vector<int> Rest = VectorDec(ptrDealingIndex, VoxelID);

	vector<vector<int>> facesUnID;
	facesUnID = DealGraph_DealIndex(ptrGraph, Rest);
	if (VoxelID.size() == 0)
	{
		facesUnID.clear();		
		for (int s = 0; s < ptrDealingIndex.size(); s++)
		{
			(*NotSort).push_back(ptrDealingIndex[s]);
		}
	}
	R1 r1;
	for (int i = 0; i < facesID.size(); i++)
	{
		r1.UsedIndex.push_back(facesID[i]);
	}
	r1.DealingIndexs = facesUnID;
	return r1;

}

void dealOnePartVoxel1(vector<SuperVoxel> *ptrPartVoxels, vector<GraphType1> *ptrGraph, vector<int> ptrDealingIndex, vector<vector<int>> *facesIDs, vector<int> *NotSort)
{
	//ֱ���������Ӧ��ֵ
	float Init_AngThreshold = 0.95;
	float Init_DisThreshold = 0.3;
	FindBestThreshold((*ptrPartVoxels), Init_AngThreshold, Init_DisThreshold);
	cout << "�����شؾ�����ֵ��" << Init_DisThreshold << endl;

	Init_DisThreshold = 0.5;
	FindBestThreshold((*ptrPartVoxels), Init_AngThreshold, Init_DisThreshold);
	cout << "�����شؾ�����ֵ��" << Init_DisThreshold << endl;

	Init_DisThreshold = 0.7;
	FindBestThreshold((*ptrPartVoxels), Init_AngThreshold, Init_DisThreshold);
	cout << "�����شؾ�����ֵ��" << Init_DisThreshold << endl;
	//Init_DisThreshold += 0.2;
	vector<R2> R2S;
	R1 r1;
	r1 = dealOnePartVoxel1Add(ptrPartVoxels, ptrGraph, ptrDealingIndex, NotSort, Init_AngThreshold, Init_DisThreshold);
	//facesIDs->push_back(r1.UsedIndex);//����
	for (int i = 0; i < r1.UsedIndex.size(); i++)
	{
		facesIDs->push_back(r1.UsedIndex[i]);
	}

	//��ӡ
	FILE *fpsum = fopen("sum1.txt", "a");
	for (int i = 0; i < r1.UsedIndex.size(); i++)
	{
		long s[3];
		getRand(255, s);
		for (int j = 0; j < r1.UsedIndex[i].size(); j++)
		{
			for (int k = 0; k < (*ptrPartVoxels)[r1.UsedIndex[i][j]].AllPoints.size(); k++)
			{
				fprintf(fpsum, "%f %f %f %d %d %d\n", (*ptrPartVoxels)[r1.UsedIndex[i][j]].AllPoints[k].X, (*ptrPartVoxels)[r1.UsedIndex[i][j]].AllPoints[k].Y, (*ptrPartVoxels)[r1.UsedIndex[i][j]].AllPoints[k].Z, s[0], s[1], s[2]);
			}
		}
	}


	for (int i = 0; i < r1.DealingIndexs.size(); i++)
	{
		R2 tempR2;
		tempR2.DealingIndexSingle = r1.DealingIndexs[i];
		R2S.push_back(tempR2);
	}
	while (R2S.size() > 0)
	{
		if (R2S[0].DealingIndexSingle.size() > 3)
		{
			R1 tempR1;
			tempR1 = dealOnePartVoxel1Add(ptrPartVoxels, ptrGraph, R2S[0].DealingIndexSingle, NotSort, Init_AngThreshold, Init_DisThreshold);
			//facesIDs->push_back(tempR1.UsedIndex);
			for (int i = 0; i < tempR1.UsedIndex.size(); i++)
			{
				facesIDs->push_back(tempR1.UsedIndex[i]);
			}

			//��ӡ
			for (int i = 0; i < tempR1.UsedIndex.size(); i++)
			{
				long s[3];
				getRand(255, s);
				for (int j = 0; j < tempR1.UsedIndex[i].size(); j++)
				{
					for (int k = 0; k < (*ptrPartVoxels)[tempR1.UsedIndex[i][j]].AllPoints.size(); k++)
					{
						fprintf(fpsum, "%f %f %f %d %d %d\n", (*ptrPartVoxels)[tempR1.UsedIndex[i][j]].AllPoints[k].X, (*ptrPartVoxels)[tempR1.UsedIndex[i][j]].AllPoints[k].Y, (*ptrPartVoxels)[tempR1.UsedIndex[i][j]].AllPoints[k].Z, s[0], s[1], s[2]);
					}
				}
			}


			for (int i = 0; i < tempR1.DealingIndexs.size(); i++)
			{
				R2 tempR2;
				tempR2.DealingIndexSingle = tempR1.DealingIndexs[i];
				R2S.push_back(tempR2);
			}
			//cout << "����ȡ" << facesIDs->size() << "����" << endl;
		}
		R2S.erase(R2S.begin());
	}
	fclose(fpsum);
}