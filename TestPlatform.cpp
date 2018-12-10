// TestPlatform.cpp : 定义控制台应用程序的入口点。
//

#include "stdafx.h"
#include <iostream>

using namespace std;

int main()
{
	LARGE_INTEGER nfreq, nst, nend;//Timer parameters.
	QueryPerformanceFrequency(&nfreq);
	TyrePointCloud my_cloud;
	PointCloud<PointXYZI>::Ptr pins;
	PointCloud<PointXYZ>::Ptr cloud(::new PointCloud<PointXYZ>);

	cout << "Please input the path of point cloud:" << endl;
	string filename = "";
	cin >> filename;
	cout << "Start loading point clouds, please wait..." << endl;
	QueryPerformanceCounter(&nst);
	my_cloud.LoadTyrePC(filename);
	QueryPerformanceCounter(&nend);
	cout << "Loading costs:" << (nend.QuadPart - nst.QuadPart) / nfreq.QuadPart << " seconds." << endl;
	cloud = my_cloud.GetOriginalPC();

	cout << "Start searching pins, please wait..." << endl;
	QueryPerformanceCounter(&nst);
	my_cloud.FindPinsBySegmentation(cloud, pins);
	QueryPerformanceCounter(&nend);
	cout << "Searching costs:" << (nend.QuadPart - nst.QuadPart) / nfreq.QuadPart << " seconds." << endl;
	for (size_t ii = 0; ii < pins->points.size(); ++ii)
	{
		cout << "Point" << ii + 1 << endl
			<< "x=" << pins->points[ii].x << "\t"
			<< "y=" << pins->points[ii].y << "\t"
			<< "z=" << pins->points[ii].z << "\t"
			<< "length=" << pins->points[ii].intensity << endl;
		if (ii >= 5)
		{
			break;
		}
	}
	system("pause");
    return 0;
}