// stdafx.h : ��׼ϵͳ�����ļ��İ����ļ���
// ���Ǿ���ʹ�õ��������ĵ�
// �ض�����Ŀ�İ����ļ�
//

#pragma once

#include "targetver.h"

#include <stdio.h>
#include <tchar.h>



// TODO:  �ڴ˴����ó�����Ҫ������ͷ�ļ�
#include "TyrePointCloud.h"

#include <iostream>

#include <pcl\io\pcd_io.h>
#include <pcl\io\ply_io.h>
#include <pcl\io\io.h>

#include <pcl\common\common.h>

#include <pcl\point_types.h>
#include <pcl\point_cloud.h>
#include <pcl\features\normal_3d.h>
#include <pcl\features\normal_3d_omp.h>
#include <pcl\common\transforms.h>

using namespace pcl;
using namespace Eigen;
using namespace std;