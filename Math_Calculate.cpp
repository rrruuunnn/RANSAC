#include"Math_Calculate.h"

float CalAngle_TwoVector(Point p1, Point p2)
{
	float COS = (p1.X*p2.X + p1.Y*p2.Y + p1.Z*p2.Z) / (sqrtf(p1.X*p1.X + p1.Y*p1.Y + p1.Z*p1.Z)*sqrtf(p2.X*p2.X + p2.Y*p2.Y + p2.Z*p2.Z));
	float Angle = acosf(COS)*180.0f / M_PI;
	return fabsf(Angle);
}
vector<float> CalAngle_ThreePoints(Point p1, Point p2, Point p3)
{
	Point vec1, vec2, vec3;
	vec1.X = p2.X - p1.X; vec1.Y = p2.Y - p1.Y; vec1.Z = p2.Z - p1.Z;
	vec2.X = p3.X - p1.X; vec2.Y = p3.Y - p1.Y; vec2.Z = p3.Z - p1.Z;
	vec3.X = p3.X - p2.X; vec3.Y = p3.Y - p2.Y; vec3.Z = p3.Z - p2.Z;
	float Angle1 = 0; float Angle2 = 0; float Angle3 = 0;
	Angle1 = CalAngle_TwoVector(vec1, vec2);
	Angle2 = 180.0 - CalAngle_TwoVector(vec1, vec3);
	Angle3 = CalAngle_TwoVector(vec2, vec3);
	vector<float> Angles;
	Angles.push_back(Angle1);
	Angles.push_back(Angle2);
	Angles.push_back(Angle3);
	return Angles;
}
void getRand(long Max, long *backNum)
{
	std::array<int, 3> seed_data;
	std::random_device r;
	std::generate_n(seed_data.data(), seed_data.size(), std::ref(r));
	std::seed_seq seq(std::begin(seed_data), std::end(seed_data));

	std::mt19937 eng(seq);

	backNum[0] = abs(seed_data.at(0)) % Max;
	backNum[1] = abs(seed_data.at(1)) % Max;
	backNum[2] = abs(seed_data.at(2)) % Max;

	if (backNum[0] == backNum[1] || backNum[1] == backNum[2] || backNum[0] == backNum[2])
	{
		getRand(Max, backNum);
	}
}

float CalACos(float x1, float y1, float z1, float x2, float y2, float z2)
{
	float cos1 = (x1 * x2 + y1 * y2 + z1 * z2) / sqrtf(x1 * x1 + y1 * y1 + z1 * z1) / sqrtf(x2 * x2 + y2 * y2 + z2 * z2);
	float Angle = acosf(cos1)*180.0f / 3.1415926f;
	return Angle;
}

bool vectors_intersection(vector<int> v1, vector<int> v2)
{
	vector<int> v;
	sort(v1.begin(), v1.end());
	sort(v2.begin(), v2.end());
	set_intersection(v1.begin(), v1.end(), v2.begin(), v2.end(), back_inserter(v));
	if (v.size() > 0)
	{
		return true;
	}
	else
	{
		return false;
	}
}

float CalDis_Faces(Point face1, Point face2)
{
	float d = (-1)*(face1.pNormal.NX*face1.X + face1.pNormal.NY*face1.Y + face1.pNormal.NZ*face1.Z);
	float Dis = (face1.pNormal.NX*face2.X + face1.pNormal.NY*face2.Y + face1.pNormal.NZ*face2.Z + d);
	return Dis;
}

std::vector<int> findSame(std::vector<int>&nLeft, std::vector<int>&nRight)
{
	std::vector<int> nResult;
	for (std::vector<int>::iterator nIterator = nLeft.begin(); nIterator != nLeft.end(); nIterator++)
	{
		if (std::find(nRight.begin(), nRight.end(), *nIterator) != nRight.end())
		{
			nResult.push_back(*nIterator);
		}
	}
	return nResult;
}

Point CalNormal(pcl::PointCloud<pcl::PointXYZ>::Ptr CenterPoint, pcl::PointCloud<pcl::PointXYZ>::Ptr ReferencePoints)
{
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud(CenterPoint);
	ne.setSearchSurface(ReferencePoints);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	ne.setSearchMethod(tree);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
	ne.setKSearch(ReferencePoints->points.size());
	ne.compute(*cloud_normals);
	Point resultPoint;
	resultPoint.X = CenterPoint->points[0].x;
	resultPoint.Y = CenterPoint->points[0].y;
	resultPoint.Z = CenterPoint->points[0].z;
	resultPoint.pNormal.NX = cloud_normals->points[0].normal_x;
	resultPoint.pNormal.NY = cloud_normals->points[0].normal_y;
	resultPoint.pNormal.NZ = cloud_normals->points[0].normal_z;

	if (resultPoint.pNormal.NZ < 0)
	{
		resultPoint.pNormal.NX *= (-1);
		resultPoint.pNormal.NY *= (-1);
		resultPoint.pNormal.NZ *= (-1);
	}

	return resultPoint;
}

vector<int> VectorDec(vector<int> A, vector<int> B)
{
	std::sort(A.begin(), A.end());
	std::sort(B.begin(), B.end());
	std::vector<int> v_intersection;
	std::set_intersection(A.begin(), A.end(), B.begin(), B.end(), std::back_inserter(v_intersection));
	std::vector<int> v_difference;
	std::set_difference(A.begin(), A.end(), v_intersection.begin(), v_intersection.end(), std::inserter(v_difference, v_difference.begin()));
	return v_difference;
}

int FindIndexInVector(int Number, vector<int> Vector)
{
	std::vector<int>::iterator ite = std::find(Vector.begin(), Vector.end(), Number);
	int Res = std::distance(Vector.begin(), ite);
	return Res;
}

float CalSD(std::vector<float> alldeviation)
{
	double sum = std::accumulate(std::begin(alldeviation), std::end(alldeviation), 0.0);
	double mean = sum / alldeviation.size();

	double accum = 0.0;
	std::for_each(std::begin(alldeviation), std::end(alldeviation), [&](const double d) {accum += (d - mean)*(d - mean); });

	double stdev = sqrt(accum / (alldeviation.size() - 1));
	return stdev;
}
double CalSD1(std::vector<double> allNums,std::vector<int>Weight)
{
	//double sum = std::accumulate(std::begin(allNums), std::end(allNums), 0.0);
	//double mean = sum / allNums.size();

	//double accum = 0.0;
	//std::for_each(std::begin(allNums), std::end(allNums), [&](const double d) {
	//	accum += (d - mean)*(d - mean);
	//});

	//double stdev = sqrt(accum / (allNums.size() - 1));
	//return stdev;

	int sumWeight = 0;
	for (int i = 0; i < Weight.size(); i++)
	{
		sumWeight += Weight[i];
	}
	double sumValue = 0;
	for (int i = 0; i < Weight.size(); i++)
	{
		sumValue += allNums[i] * Weight[i];
	}
	double avgValue = sumValue / sumWeight;//计算平均值

	double ceshi = std::accumulate(allNums.begin(), allNums.end(), 0.0);

	double sumSD = 0;
	for (int i = 0; i < Weight.size(); i++)
	{
		sumSD += pow(allNums[i] - avgValue, 2)*Weight[i];
	}
	double avgSD = sumSD / sumWeight;

	return sqrt(avgSD);

}
double CalSD2(std::vector<double> alldeviation)
{
	double sum = std::accumulate(std::begin(alldeviation), std::end(alldeviation), 0.0);
	double mean = sum / alldeviation.size();

	double accum = 0.0;
	std::for_each(std::begin(alldeviation), std::end(alldeviation), [&](const double d) {accum += (d - mean)*(d - mean); });

	double stdev = sqrt(accum / (alldeviation.size() - 1));
	return stdev;
}