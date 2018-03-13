#include "registration.h"
#include <time.h>
#include <algorithm>
#include <windows.h>
#include <cstdlib>
#include <time.h>
#include <iostream>
#include <string>

using pcl::visualization::PointCloudColorHandlerGenericField;
using pcl::visualization::PointCloudColorHandlerCustom;

template<class PointT>
DWORD WINAPI readFLS(LPVOID lpParameter) {
	CoInitialize(NULL);
		regis::threadParam<PointT>* t = (regis::threadParam<PointT>*)lpParameter;

		// 以下liscence需要整段输入，key放入liscence key
		BSTR licenseCode =
			L"FARO Open Runtime License\n"
			L"Key:W2CW4PNRTCTXXJ6T6KXYSRUPL\n" // License Key
			L"\n"
			L"The software is the registered property of "
			L"FARO Scanner Production GmbH, Stuttgart, Germany.\n"
			L"All rights reserved.\n"
			L"This software may only be used with written permission "
			L"of FARO Scanner Production GmbH, Stuttgart, Germany.";
 		IiQLicensedInterfaceIfPtr liPtr(__uuidof(iQLibIf));
		liPtr->License = licenseCode;
		IiQLibIfPtr libRef = static_cast<IiQLibIfPtr>(liPtr);	//点云数据IO
		libRef->load(t->filepath.c_str());						//加载数据
		IiQObjectIfPtr libObj = libRef->getScanObject(0);
		IiQScanObjIfPtr scanRef = libObj->getScanObjSpecificIf();//法如扫描属性IO
		scanRef->load();//加载属性
		double x, y, z, angle;
		int refl;
		int rows = libRef->getScanNumRows(0);
		int cols = libRef->getScanNumCols(0);

		// 获取变换矩阵的参数
		libRef->getScanOrientation(0, &x, &y, &z, &angle);
		// 由文档公式得
		double ca = cos(-angle);
		double sa = sin(-angle);
		// 变换矩阵
		double R[3][3];
		R[0][0] = x * x * (1 - ca) + ca;
		R[0][1] = y * x * (1 - ca) - z * sa;
		R[0][2] = z * x * (1 - ca) + y * sa;
		R[1][0] = x * y * (1 - ca) + z * sa;
		R[1][1] = y * y * (1 - ca) + ca;
		R[1][2] = z * y * (1 - ca) - x * sa;
		R[2][0] = x * z * (1 - ca) - y * sa;
		R[2][1] = y * z * (1 - ca) + x * sa;
		R[2][2] = z * z * (1 - ca) + ca;

		std::vector<PointT> temp;
		for (int col = 0; col < cols; col = col + t->scale)
		{
			for (int row = 0; row < rows; row = row + t->scale) {
				libRef->getScanPoint(0, row, col, &x, &y, &z, &refl);     // 读取数据,x,y,z 点坐标， refl为反射值
				if (x == 0 && y == 0 && z == 0)
				{
					continue;
				}
				//temp.push_back(regis::Point(x*R[0][0] + y*R[1][0] + z*R[2][0], x*R[0][1] + y*R[1][1] + z*R[2][1], x*R[0][2] + y*R[1][2] + z*R[2][2]));
				temp.push_back(PointT(x*R[0][0] + y*R[1][0] + z*R[2][0] + t->offset.x, x*R[0][1] + y*R[1][1] + z*R[2][1] + t->offset.y, x*R[0][2] + y*R[1][2] + z*R[2][2]));
			}
		}

		WaitForSingleObject(t->_mutex, INFINITE);
		std::cout << t->filepath << "got mutex" << std::endl;
		for (int i = 0; i < temp.size(); i++) {
			t->ptCloud->push_back(temp[i]);
		}

		std::cout << "initialize octree for " << t->filepath << std::endl;
		t->octree->_ocTree.initialize(*(t->ptCloud));
		std::cout << "octree completed - " << t->filepath << std::endl;
		ReleaseMutex(t->_mutex);

		libRef->unloadScan(1);
		libRef = NULL;
		liPtr = NULL;

		CoUninitialize();
		return 0;
	}

DWORD WINAPI Thr2ed_findNearest(LPVOID lpParameter) {
	regis::findNearestParam* t = (regis::findNearestParam*)lpParameter;
	*(t->threadcount)++;
	std::cout << "creating thread for finding Nearest Point. " << "now count:" << *(t->threadcount) << std::endl;
	for (int i = 0; i < t->ref.size(); i++)
	{
		double tdis = t->a.L2dis(t->ref[i]);
		if (tdis < *(t->dis))
		{
			*(t->dis) = tdis;
			t->result = t->ref[i];
		}
	}
	*(t->threadcount)--;
	return 0L;
}

template<class PointT>
void registration::extractData(std::vector<PointT> pCloud,std::vector<std::string> filename, std::vector<regis::Vec> _vec, int scale)
{
	CoInitialize(NULL);
	std::vector<HANDLE> pth;
	pCloud.resize(filename.size());
	octree.resize(filename.size());
	std::vector<regis::threadParam<PointT>> _param;

	for (int i = 0; i < filename.size(); i++)
	{
		regis::threadParam<PointT> tParam;
		tParam.scale = scale;
		tParam.filepath = filename[i];
		tParam.ptCloud = &(pCloud[i]);
		tParam.octree = &(octree[i]);
		tParam._mutex = &_mutex;
		tParam.offset = _vec[i];
		_param.push_back(tParam);
	}
	Sleep(1000);
	for (int i = 0; i < filename.size(); i++)
	{
		HANDLE tThrd = CreateThread(NULL, 0, &readFLS, &(_param[i]), 0, NULL);
		pth.push_back(tThrd);
	}

 	for (int i = 0; i < filename.size(); i++)
	{
		std::cout << "waiting" << i << std::endl;
		WaitForSingleObject(pth[i], INFINITE);
		CloseHandle(pth[i]);
		std::cout << i << "done" << std::endl;

	}

	CoUninitialize();
}

void registration::extractFLSData(std::vector<std::string> filename, std::vector<regis::Vec> _vec, int scale)
{
	data.resize(filename.size());
	octree.resize(filename.size());
	boundingBox.resize(filename.size());
	fileName = filename;
	for (int32_t i = 0; i < filename.size(); i++)
	{
		CoInitialize(NULL);
		// 以下liscence需要整段输入，key放入liscence key
		BSTR licenseCode =
			L"FARO Open Runtime License\n"
			L"Key:W2CW4PNRTCTXXJ6T6KXYSRUPL\n" // License Key
			L"\n"
			L"The software is the registered property of "
			L"FARO Scanner Production GmbH, Stuttgart, Germany.\n"
			L"All rights reserved.\n"
			L"This software may only be used with written permission "
			L"of FARO Scanner Production GmbH, Stuttgart, Germany.";

		IiQLicensedInterfaceIfPtr liPtr(__uuidof(iQLibIf));
		liPtr->License = licenseCode;
		IiQLibIfPtr libRef = static_cast<IiQLibIfPtr>(liPtr);	//点云数据IO
		libRef->load(filename[i].c_str());						//加载数据
		IiQObjectIfPtr libObj = libRef->getScanObject(0);
		IiQScanObjIfPtr scanRef = libObj->getScanObjSpecificIf();//法如扫描属性IO
		scanRef->load();//加载属性
		double x, y, z, angle;
		int refl;
		int rows = libRef->getScanNumRows(0);
		int cols = libRef->getScanNumCols(0);

		// 获取变换矩阵的参数
		libRef->getScanOrientation(0, &x, &y, &z, &angle);
		// 由文档公式得
		double ca = cos(-angle);
		double sa = sin(-angle);
		// 变换矩阵
		double R[3][3];
		R[0][0] = x * x * (1 - ca) + ca;
		R[0][1] = y * x * (1 - ca) - z * sa;
		R[0][2] = z * x * (1 - ca) + y * sa;
		R[1][0] = x * y * (1 - ca) + z * sa; 
		R[1][1] = y * y * (1 - ca) + ca;
		R[1][2] = z * y * (1 - ca) - x * sa;
		R[2][0] = x * z * (1 - ca) - y * sa;
		R[2][1] = y * z * (1 - ca) + x * sa;
		R[2][2] = z * z * (1 - ca) + ca;

		double minx=0, miny=0, minz=0, maxx=0, maxy=0, maxz=0;
		for (int col = 0; col < cols; col = col + scale)
		{
			for (int row = 0; row < rows; row = row + scale) {
				libRef->getScanPoint(0, row, col, &x, &y, &z, &refl);     // 读取数据,x,y,z 点坐标， refl为反射值
				if (x == 0 && y == 0 && z == 0)
				{
					continue;
				}
				
				double xx, yy, zz;
				xx = x*R[0][0] + y*R[1][0] + z*R[2][0];
				yy = x*R[0][1] + y*R[1][1] + z*R[2][1];
				zz = x*R[0][2] + y*R[1][2] + z*R[2][2];

				if (xx*xx+yy*yy+zz*zz>100)
				{
					continue;
				}

				xx += _vec[i].x;
				yy += _vec[i].y;

				//筛选处理
				if (xx < minx) {
					minx = xx;
				}
				if(xx > maxx){
					maxx = xx;
				}
				if (yy < miny)
				{
					miny = yy; 
				}
				if (yy>maxy)
				{
					maxy =yy;
				}
				if (zz<minz)
				{
					minz = zz;
				}
				if (zz>maxz)
				{
					maxz = zz;
				}
				//*****************

				//temp.push_back(regis::Point(x*R[0][0] + y*R[1][0] + z*R[2][0], x*R[0][1] + y*R[1][1] + z*R[2][1], x*R[0][2] + y*R[1][2] + z*R[2][2]));
				//加偏移量
				//data[i].push_back(regis::Point(x*R[0][0] + y*R[1][0] + z*R[2][0] + _vec[i].x, x*R[0][1] + y*R[1][1] + z*R[2][1] +_vec[i].y, x*R[0][2] + y*R[1][2] + z*R[2][2]));
				//不加偏移量
				//data[i].push_back(regis::Point(xx,yy,zz));
				//加偏移量
				data[i].push_back(regis::Point(xx, yy, zz));
			}
		}
		boundingBox[i]=regis::Box(minx, miny, minz, maxx, maxy, maxz);

		std::cout << "initialize octree for " << filename[i] << std::endl;
		octree[i]._ocTree.initialize(data[i]);
		std::cout << "octree completed - " << filename[i] << std::endl;
		libRef = NULL;
		liPtr = NULL;
		CoUninitialize();
	}

	
}

void registration::extractFLS2PCD(std::vector<std::string> filename, std::vector<regis::Vec> _vec, int scale)
{
	pcddata.resize(filename.size());
	for (int i=0;i<pcddata.size();i++)
	{
		PointCloud::Ptr a(new PointCloud);
		pcddata[i] = a;
	}
	boundingBox.resize(filename.size());

	for (int32_t i = 0; i < filename.size(); i++)
	{
		CoInitialize(NULL);
		// 以下liscence需要整段输入，key放入liscence key
		BSTR licenseCode =
			L"FARO Open Runtime License\n"
			L"Key:W2CW4PNRTCTXXJ6T6KXYSRUPL\n" // License Key
			L"\n"
			L"The software is the registered property of "
			L"FARO Scanner Production GmbH, Stuttgart, Germany.\n"
			L"All rights reserved.\n"
			L"This software may only be used with written permission "
			L"of FARO Scanner Production GmbH, Stuttgart, Germany.";

		IiQLicensedInterfaceIfPtr liPtr(__uuidof(iQLibIf));
		liPtr->License = licenseCode;
		IiQLibIfPtr libRef = static_cast<IiQLibIfPtr>(liPtr);	//点云数据IO
		libRef->load(filename[i].c_str());						//加载数据
		IiQObjectIfPtr libObj = libRef->getScanObject(0);
		IiQScanObjIfPtr scanRef = libObj->getScanObjSpecificIf();//法如扫描属性IO
		scanRef->load();//加载属性
		double x, y, z, angle;
		int refl;
		int rows = libRef->getScanNumRows(0);
		int cols = libRef->getScanNumCols(0);

		// 获取变换矩阵的参数
		libRef->getScanOrientation(0, &x, &y, &z, &angle);
		// 由文档公式得
		double ca = cos(-angle);
		double sa = sin(-angle);
		// 变换矩阵
		double R[3][3];
		R[0][0] = x * x * (1 - ca) + ca;
		R[0][1] = y * x * (1 - ca) - z * sa;
		R[0][2] = z * x * (1 - ca) + y * sa;
		R[1][0] = x * y * (1 - ca) + z * sa;
		R[1][1] = y * y * (1 - ca) + ca;
		R[1][2] = z * y * (1 - ca) - x * sa;
		R[2][0] = x * z * (1 - ca) - y * sa;
		R[2][1] = y * z * (1 - ca) + x * sa;
		R[2][2] = z * z * (1 - ca) + ca;

		double minx = 0, miny = 0, minz = 0, maxx = 0, maxy = 0, maxz = 0,device_high=0;
		int highcount = 0;
		for (int col = 0; col < cols; col = col + scale)
		{
			for (int row = 0; row < rows; row = row + scale) {
				libRef->getScanPoint(0, row, col, &x, &y, &z, &refl);     // 读取数据,x,y,z 点坐标， refl为反射值
				if (x == 0 && y == 0 && z == 0)
				{
					continue;
				}

				double xx, yy, zz;
				xx = x*R[0][0] + y*R[1][0] + z*R[2][0];
				yy = x*R[0][1] + y*R[1][1] + z*R[2][1];
				zz = x*R[0][2] + y*R[1][2] + z*R[2][2];

				if (xx*xx + yy*yy + zz*zz>225)
				{
					continue;
				}

				//计算仪器高
				if (row == rows - 1)
				{
					highcount++;
					device_high += std::pow(xx*xx + yy*yy, 0.5)*std::tan(M_PI/3.0);
					std::cout << std::pow(xx*xx + yy*yy, 0.5)*std::tan(M_PI / 3.0) << std::endl;
				}

				//平移处理
				xx += _vec[i].x;
				yy += _vec[i].y;

				//筛选处理
				if (xx < minx) {
					minx = xx;
				}
				if (xx > maxx) {
					maxx = xx;
				}
				if (yy < miny)
				{
					miny = yy;
				}
				if (yy>maxy)
				{
					maxy = yy;
				}
				if (zz<minz)
				{
					minz = zz;
				}
				if (zz>maxz)
				{
					maxz = zz;
				}
				//*****************

				

				//temp.push_back(regis::Point(x*R[0][0] + y*R[1][0] + z*R[2][0], x*R[0][1] + y*R[1][1] + z*R[2][1], x*R[0][2] + y*R[1][2] + z*R[2][2]));
				//加偏移量
				//data[i].push_back(regis::Point(x*R[0][0] + y*R[1][0] + z*R[2][0] + _vec[i].x, x*R[0][1] + y*R[1][1] + z*R[2][1] +_vec[i].y, x*R[0][2] + y*R[1][2] + z*R[2][2]));
				//不加偏移量
				//data[i].push_back(regis::Point(xx,yy,zz));
				//加偏移量
				(*pcddata[i]).push_back(PointT_pcl(xx, yy, zz));
			}
		}
		
		faro_altitude.push_back(device_high / highcount);

		//remove NAN points from the cloud
		std::vector<int> indices;
		pcl::removeNaNFromPointCloud(*pcddata[i],*pcddata[i], indices);
		
		boundingBox[i] = regis::Box(minx, miny, minz, maxx, maxy, maxz);
		libRef = NULL;
		liPtr = NULL;
		CoUninitialize();
	}

	
	//showRotateCloudLeft(pcddata);

}

void registration::extractCsvData(std::vector<std::string> filename, std::vector<regis::Vec> _vec)
{
	//设置读入数据
	data.resize(filename.size());
	//初始化八叉树大小
	octree.resize(filename.size());
	//初始化包围盒大小
	boundingBox.resize(filename.size());


	for (int i = 0; i < filename.size(); i++)
	{
		FILE* file;
		fopen_s(&file, filename[i].c_str(), "r");
		regis::Point temp;
		std::cout << "reading file:" << filename[i] << std::endl;
		double minx = 9999, miny = 9999, minz = 9999, maxx = -9999, maxy = -9999, maxz = -9999;
		while (!std::feof(file))
		{
			fscanf_s(file, "%lf,%lf,%lf,%*lf,%*lf,%*lf\n", &temp.x, &temp.y, &temp.z);
			if (temp.x*temp.x + temp.y * temp.y + temp.z * temp.z > 225)
			{
				continue;
			}
			temp.x += _vec[i].x;
			temp.y += _vec[i].y;
			data[i].push_back(temp);

			//筛选处理
			if (temp.x < minx) {
				minx = temp.x;
			}
			if (temp.x > maxx) {
				maxx = temp.x;
			}
			if (temp.y < miny)
			{
				miny = temp.y;
			}
			if (temp.y > maxy)
			{
				maxy = temp.y;
			}
			if (temp.z < minz)
			{
				minz = temp.z;
			}
			if (temp.z > maxz)
			{
				maxz = temp.z;
			}
			//*****************
		}
		boundingBox[i] = regis::Box(minx, miny, minz, maxx, maxy, maxz);

		std::cout << filename[i] << " filesize:" << data[i].size() << std::endl;

		std::cout << "initialize octree for " << filename[i] << std::endl;

		octree[i]._ocTree.initialize(data[i]);

		std::cout << "octree completed - " << filename[i] << std::endl;

	}
}

regis::Point registration::rotatePoint(regis::Point p, regis::Point rotateCenter, double angle)
{
		double x = (p.x - rotateCenter.x)*std::cos((angle / 180)*PI) - (p.y - rotateCenter.y)*std::sin((angle / 180)*PI) + rotateCenter.x;
		double y = (p.x - rotateCenter.x)*std::sin((angle / 180)*PI) + (p.y - rotateCenter.y)*std::cos((angle / 180)*PI) + rotateCenter.y;
		return regis::Point(x, y, p.z);
}

PointT_pcl registration::rotatePoint(PointT_pcl p, regis::Point rotateCenter, double angle)
{
	double x = (p.x - rotateCenter.x)*std::cos((angle / 180)*PI) - (p.y - rotateCenter.y)*std::sin((angle / 180)*PI) + rotateCenter.x;
	double y = (p.x - rotateCenter.x)*std::sin((angle / 180)*PI) + (p.y - rotateCenter.y)*std::cos((angle / 180)*PI) + rotateCenter.y;
	return PointT_pcl(x, y, p.z);
}

double registration::getICPerror(std::vector<regis::Point> moveStation, std::vector<regis::Point> refStation, regis::Vec moveS, regis::Vec refS, int sample)
{
	double result = 0;


	if (moveStation.size() < sample)
	{
		sample = moveStation.size();
	}


	std::srand((unsigned)time(NULL));


	int count = 0;
	int maxthread = 12;
	int threads = 0;
	double *dis = new double[sample];


	//最多运行1.5sample的循环
	// 		for (int i=0;i<1.5*sample;i++)
	// 		{
	// 			//当sample的数量达标时退出循环
	// 			if (count==sample)
	// 				break;
	// 
	// 			int id = rand() % moveStation.size();
	// 			//多线程版本
	// // 			if (moveStation[id].x>min(moveS.x, refS.x) & moveStation[id].x<max(moveS.x, refS.x) &moveStation[id].y>min(moveS.y, refS.y) &moveStation[id].y<max(moveS.y, refS.y))
	// // 			{
	// // 				count++;
	// // 				regis::findNearestParam Pa;
	// // 				Pa.dis = ++dis;
	// // 				Pa.threadcount = &threads;
	// // 				while (threads>=maxthread)
	// // 				{
	// // 					Sleep(100);
	// // 				}
	// // 				HANDLE tThrd = CreateThread(NULL, 0, &regis::Thr2ed_findNearest, &Pa, 0, NULL);
	// // 				CloseHandle(tThrd);
	// // 			}
	// 
	// 			//普通版本
	// 			if (moveStation[id].x > min(moveS.x, refS.x) & moveStation[id].x <max(moveS.x, refS.x) & moveStation[id].y>min(moveS.y, refS.y) & moveStation[id].y< max(moveS.y, refS.y)) {
	// 				count++;
	// 				result += findNearestPoint(moveStation[id], refStation, 0);
	// 			}
	// 		}


	double xlenth = std::max(moveS.x, refS.x) - std::min(moveS.x, refS.x);
	double ylenth = std::max(moveS.y, refS.y) - std::min(moveS.y, refS.y);


	//最多运行1.5sample的循环,version 2
	for (int i = 0; i < moveStation.size(); i++)
	{
		//当sample的数量达标时退出循环
		if (count == sample)
			break;

		//普通版本
		if (moveStation[i].x > std::min(moveS.x, refS.x) - xlenth & moveStation[i].x <std::max(moveS.x, refS.x) + xlenth & moveStation[i].y>std::min(moveS.y, refS.y) - ylenth & moveStation[i].y < std::max(moveS.y, refS.y) + ylenth) {
			count++;
			result += findNearestPoint(moveStation[i], refStation, 0);
		}
	}

	std::cout << count << "pairs of common points are selected." << std::endl;
	return result / count;
}

double registration::getICPerror_OC(std::vector<regis::Point> moveStation, std::vector<regis::Point> refStation, regis::Vec moveS, regis::Vec refS, int sample,const regis::ocTree* moveOC, const regis::ocTree* refOC)
{
	double result = 0;

	//采样大小
	if (moveStation.size() < sample) 
	{
		sample = moveStation.size();
	}
	int count = 0;
	//获取站点中心包围盒大小
	double dis = std::pow(std::pow(moveS.x - refS.x, 2) + std::pow(moveS.y - refS.y, 2), 0.5);

	//最多运行1.5sample的循环,version 2
	for (int i = 0; i < moveStation.size(); i++)
	{
		//当sample的数量达标时退出循环
		if (count == sample)
			break;

		//普通版本
		if (moveStation[i].XYdis(regis::Point(moveS.x,moveS.y,0))<dis || moveStation[i].XYdis(regis::Point(refS.x, refS.y, 0))<dis) {
			double tr = findNearestPoint_OC(moveStation[i], refStation, 0, refOC);
			if (tr != -1) {
				count++;
				result +=tr;
			}

		}
	}

	std::cout << count << "pairs of common points are selected." << std::endl;
	return result / count;
}

double registration::getICPerror_KD(const PointCloud::Ptr moveStation,const PointCloud::Ptr refStation, regis::Vec moveS, regis::Vec refS, int sample, const pcl::KdTreeFLANN<PointT_pcl> refKD)
{
	double result = 0;

	//采样大小
	if (moveStation->size() < sample)
	{
		sample = moveStation->size();
	}
	int count = 0;
	//获取站点中心包围盒大小  
	double dis = std::pow(std::pow(moveS.x - refS.x, 2) + std::pow(moveS.y - refS.y, 2), 0.5);

	//最多运行1.5sample的循环,version 2
	for (int i = 0; i < moveStation->size(); i++)
	{
		//当sample的数量达标时退出循环
		if (count == sample)
			break;

		//普通版本
		double d1= std::pow(std::pow(moveStation->points[i].x - moveS.x, 2) +\
			std::pow(moveStation->points[i].y - moveS.y, 2), 0.5);
		double d2 = std::pow(std::pow(moveStation->points[i].x - refS.x, 2) + \
			std::pow(moveStation->points[i].y - refS.y, 2), 0.5);
		
		std::vector<int> pointIdxNKNSearch(1);
		std::vector<float> pointNKNSquaredDistance(1);

		if (d1 < dis || d2 < dis) {
			int tr=refKD.nearestKSearch((*moveStation)[i], 1, pointIdxNKNSearch, pointNKNSquaredDistance);
			if (tr ==1 && pointNKNSquaredDistance[0]<2) {
				count++;
				result += pointNKNSquaredDistance[0];
			}
		}
	}

	std::cout << count << "pairs of common points are selected." << std::endl;
	return result / count;
}

double registration::findNearestPoint(regis::Point a, std::vector<regis::Point> ref, double tttt)
{
	double dis = 9999; 
	regis::Point res;
	for (size_t i = 0; i < ref.size(); i++)
	{
		double tdis = a.L2dis(ref[i]);
		if (tdis < dis)
		{
			dis = tdis;
			res = ref[i];
		}
	}
	return dis;
}

double registration::findNearestPoint_OC(regis::Point a, std::vector<regis::Point> ref, double tttt,const regis::ocTree* rOC)
{
	double dis = 9999;
	float step = 0.05;
	regis::Point res;
	std::vector<uint32_t> sss;
	
	rOC->_ocTree.radiusNeighbors<unibn::L2Distance<regis::Point>>(a, 2.0f, sss);
	if (sss.size() == 0) {
		return -1;
	}

	rOC->_ocTree.radiusNeighbors<unibn::L2Distance<regis::Point>>(a, step, sss);

	while (sss.size()==0)
	{
		step += 0.05;
		rOC->_ocTree.radiusNeighbors<unibn::L2Distance<regis::Point>>(a, step, sss);
	}

	for (size_t i = 0; i < sss.size(); i++)
	{
		double tdis = a.L2dis(ref[sss[i]]);
		if (tdis < dis)
		{
			dis = tdis;
		}
	}
	return dis;
}

regis::Point registration::findNearestPoint(regis::Point a, std::vector<regis::Point> ref)
{
	double dis = 9999;
	regis::Point res;
	for (size_t i = 0; i < ref.size(); i++)
	{
		double tdis = a.L2dis(ref[i]);
		if (tdis < dis)
		{
			dis = tdis;
			res = ref[i];
		}
	}
	return res;
}

void registration::getRotation(std::vector<regis::Vec> _vec, double step)
{
	//初始迭代步长
	int iterator_times = 360 / step;
	double min_err_angle = 0;    //最小误差角度
	double min_err = 999999;    //最小误差值
	for (size_t i = 0; i < iterator_times; i++)
	{
		std::cout << "iterator: " << i << "begin." << std::endl;

		double res = 0;

		std::cout << "rotating point cloud...." << std::endl;

		//旋转点云
		for (size_t j = 0; j < data.size(); j++)
		{
			for (size_t k = 0; k < data[j].size(); k++)
			{
				data[j][k] = rotatePoint(data[j][k], regis::Point(_vec[j].x, _vec[j].y, 0), step);
			}
		}

		std::cout << "calculate ICP error " << std::endl;
		//计算最近点误差
		res += getICPerror(data[0], data[1], _vec[0], _vec[1], 1000);
		std::cout << "Station 0-1 " << "ICP error:" << res << std::endl;
		res += getICPerror(data[1], data[2], _vec[1], _vec[2], 1000);
		std::cout << "Station 1-2 " << "ICP error:" << res << std::endl;
		res += getICPerror(data[2], data[3], _vec[2], _vec[3], 1000);
		std::cout << "Station 2-3 " << "ICP error:" << res << std::endl;
		res += getICPerror(data[3], data[4], _vec[3], _vec[4], 1000);
		std::cout << "Station 3-4 " << "ICP error:" << res << std::endl;

		std::cout << "iterator: " << i << " ,ICP error:" << res << std::endl;

		if (res < min_err)
		{
			min_err = res;
			min_err_angle = i * 10;
			std::cout << "best angle: " << min_err_angle << std::endl;
			std::cout << "ICP error:" << res << std::endl;
		}
	}
}

void registration::getRotation2(std::vector<regis::Vec> _vec, double step)
{
	//获取特征数据
	std::vector<std::vector<regis::Point>> target_data = getFeaturedata(0.9, 0);
	std::cout << "特征数据获取完毕. " << std::endl;

	//建立临时八叉树
	std::vector<regis::ocTree> target_octree;
	target_octree.resize(target_data.size());

	//初始迭代步长
	int iterator_times = 360 / step;
	double min_err_angle = 0;    //最小误差角度
	double min_err = 999999;    //最小误差值

	for (size_t i = 0; i < iterator_times; i++)
	{
		std::cout << "iterator: " << i << "begin." << std::endl;

		double res = 0;

		std::cout << "rotating point cloud...." << std::endl;

		//旋转点云
		for (size_t j = 0; j < target_data.size(); j++)
		{
			for (size_t k = 0; k < target_data[j].size(); k++)
			{
				target_data[j][k] = rotatePoint(target_data[j][k], regis::Point(_vec[j].x, _vec[j].y, 0), step);
			}
		}
		std::cout << "旋转点云完毕." << std::endl;
		
		//建立临时八叉树
		target_octree.resize(target_data.size());
		for (int i = 0; i < target_data.size(); i++)
		{
			target_octree[i]._ocTree.initialize(target_data[i]);
		}
		std::cout << "特征数据八叉树建立完毕." << std::endl;


		std::cout << "calculate ICP error " << std::endl;
		//计算最近点误差
		res += getICPerror_OC(target_data[0], target_data[1], _vec[0], _vec[1], 1000,&target_octree[0],&target_octree[1]);
		std::cout << "Station 0-1 " << "ICP error:" << res << std::endl;
		res += getICPerror_OC(target_data[1], target_data[2], _vec[1], _vec[2], 1000, &target_octree[1], &target_octree[2]);
		std::cout << "Station 1-2 " << "ICP error:" << res << std::endl;
 		res += getICPerror_OC(target_data[2], target_data[3], _vec[2], _vec[3], 1000, &target_octree[2], &target_octree[3]);
 		std::cout << "Station 2-3 " << "ICP error:" << res << std::endl;
 		res += getICPerror_OC(target_data[3], target_data[4], _vec[3], _vec[4], 1000, &target_octree[3], &target_octree[4]);
 		std::cout << "Station 3-4 " << "ICP error:" << res << std::endl;

		std::cout << "iterator: " << i << " ,ICP error:" << res << std::endl;

		if (res < min_err)
		{
			min_err = res;
			min_err_angle = i * step;
		}

		target_octree.swap(std::vector<regis::ocTree>());
	}
	std::cout << "best angle: " << min_err_angle << std::endl;
	std::cout << "ICP error:" << min_err << std::endl;
}

double registration::getRotation_onRender(std::vector<regis::Vec> _vec, double step)
{
	//获取特征数据
	std::vector<PointCloud::Ptr> target_data;
	for (int i=0;i<data.size();i++)
	{
		target_data.push_back(PointCloud::Ptr(new PointCloud));
	}
	getFeaturedata(target_data, 0.9, 0);
	std::cout << "特征数据获取完毕. " << std::endl;

	//建立临时kd树
	std::vector<pcl::KdTreeFLANN<PointT_pcl>> target_octree;
	target_octree.resize(target_data.size());

	//初始迭代步长
	int iterator_times = 360 / step;
	double min_err_angle = 0;    //最小误差角度
	double min_err = 999999;    //最小误差值

	for (size_t i = 0; i < iterator_times; i++)
	{
		std::cout << "iterator: " << i << "begin." << std::endl;

		double res = 0;

		std::cout << "rotating point cloud...." << std::endl;

		//旋转点云
		for (size_t j = 0; j < target_data.size(); j++)
		{
			for (size_t k = 0; k < target_data[j]->points.size(); k++)
			{
				target_data[j]->points[k] = rotatePoint(target_data[j]->points[k], regis::Point(_vec[j].x, _vec[j].y, 0), step);
			}
		}
		std::cout << "旋转点云完毕." << std::endl;

		

		//建立临时kd树
		target_octree.resize(target_data.size());
		for (int i = 0; i < target_data.size(); i++)
		{
			target_octree[i].setInputCloud(target_data[i]);
		}
		std::cout << "特征数据八叉树建立完毕." << std::endl;


		std::cout << "calculate ICP error " << std::endl;  
		//计算最近点误差
		//办公室版本
// 		res += getICPerror_KD(target_data[0], target_data[1], _vec[0], _vec[1], 1000, target_octree[1]);
// 		std::cout << "Station 0-1 " << "ICP error:" << res << std::endl;
// 		res += getICPerror_KD(target_data[1], target_data[2], _vec[1], _vec[2], 1000,target_octree[2]);
// 		std::cout << "Station 1-2 " << "ICP error:" << res << std::endl;
// 		res += getICPerror_KD(target_data[2], target_data[3], _vec[2], _vec[3], 1000,target_octree[3]);
// 		std::cout << "Station 2-3 " << "ICP error:" << res << std::endl;
// 		res += getICPerror_KD(target_data[3], target_data[4], _vec[3], _vec[4], 1000,target_octree[4]);
// 		std::cout << "Station 3-4 " << "ICP error:" << res << std::endl;

		//宝岗大道版本
		res += getICPerror_KD(target_data[0], target_data[1], _vec[0], _vec[1], 1000, target_octree[1]);
		std::cout << "Station 0-1 " << "ICP error:" << res << std::endl;
		res += getICPerror_KD(target_data[1], target_data[2], _vec[1], _vec[2], 1000, target_octree[2]);
		std::cout << "Station 1-2 " << "ICP error:" << res << std::endl;
		res += getICPerror_KD(target_data[1], target_data[3], _vec[1], _vec[3], 1000, target_octree[3]);
		std::cout << "Station 2-3 " << "ICP error:" << res << std::endl;
		res += getICPerror_KD(target_data[3], target_data[4], _vec[3], _vec[4], 1000, target_octree[4]);
		std::cout << "Station 3-4 " << "ICP error:" << res << std::endl;
		res += getICPerror_KD(target_data[4], target_data[5], _vec[4], _vec[5], 1000, target_octree[5]);
		std::cout << "Station 4-5 " << "ICP error:" << res << std::endl;
		res += getICPerror_KD(target_data[5], target_data[6], _vec[5], _vec[6], 1000, target_octree[6]);
		std::cout << "Station 5-6 " << "ICP error:" << res << std::endl;
		res += getICPerror_KD(target_data[5], target_data[11], _vec[5], _vec[11], 1000, target_octree[11]);
		std::cout << "Station 5-11 " << "ICP error:" << res << std::endl;
		res += getICPerror_KD(target_data[6], target_data[7], _vec[6], _vec[7], 1000, target_octree[7]);
		std::cout << "Station 6-7 " << "ICP error:" << res << std::endl;
		res += getICPerror_KD(target_data[6], target_data[8], _vec[6], _vec[8], 1000, target_octree[8]);
		std::cout << "Station 6-8 " << "ICP error:" << res << std::endl;
		res += getICPerror_KD(target_data[8], target_data[9], _vec[8], _vec[9], 1000, target_octree[9]);
		std::cout << "Station 8-9 " << "ICP error:" << res << std::endl;
		res += getICPerror_KD(target_data[9], target_data[10], _vec[9], _vec[10], 1000, target_octree[10]);
		std::cout << "Station 9-10 " << "ICP error:" << res << std::endl;

		std::cout << "iterator: " << i << " ,ICP error:" << res << std::endl;

		if (res < min_err)
		{
			min_err = res;
			min_err_angle = i * step;
			showRotateCloudRight(target_data);
		}

		target_octree.swap(std::vector<pcl::KdTreeFLANN<PointT_pcl>>());
	}
	std::cout << "best angle: " << min_err_angle << std::endl;
	std::cout << "ICP error:" << min_err << std::endl;

	return min_err_angle;
}

void registration::getVisableArea()
{

}

void registration::getdatasize()
{
	std::cout << "files:" << data.size();
	std::cout << "files 1:" << data[0].size();
	std::cout << "files 2:" << data[1].size();
}

void registration::subsample(double subDis)
{	
	subdata_ind.clear();
	for (int i=0;i<data.size();i++)
	{
		std::vector<uint32_t> ind;
		std::vector<uint32_t> res;
		ind.resize(data[i].size());

		for (size_t j=0;j<ind.size();j++)
		{
			if (ind[j]==0){
				res.push_back(j);
				std::vector<uint32_t> t;
				octree[i]._ocTree.radiusNeighbors<unibn::L2Distance<regis::Point>>(data[i][j], subDis, t);
				for (size_t k = 0; k < t.size(); k++)
				{
					ind[t[k]] = 1;
				}
			}
		}
		subdata_ind.push_back(res);

		std::cout << "subsample data" << i << "size:" << res.size() << std::endl;
	}
}

void registration::CalculateFeature(double ocDis, bool x_bool, bool y_bool, bool z_bool)
{
	//判断是否已有数据文件
	if (data.size() == 0)
	{
		std::cout << "not data exist." << std::endl;
		return;
	}
	else {
		if (octree.size() != data.size())
		{
			//进行建树
		}

		pointFeature.resize(data.size());

		//计算特征
		//遍历所有点云文件
		clock_t begin;
		begin = clock();
		std::cout << "start calculate feature" << std::endl;
		for (int32_t i = 0; i < data.size(); i++)
		{
			pointFeature[i].resize(data[i].size());

			int colnum = (boundingBox[i].maxx - boundingBox[i].minx)/ocDis;
			int rownum = (boundingBox[i].maxy - boundingBox[i].miny)/ocDis;
			int highnum= (boundingBox[i].maxz - boundingBox[i].minz) / ocDis;
			std::cout << "size:" <<colnum<<","<<rownum<< std::endl;

			for (int32_t col=0;col<colnum;col++)
			{
				for (int32_t row=0;row<rownum;row++)
				{
					
					regis::Point t(boundingBox[i].minx + ocDis / 2.0 + ocDis*col, boundingBox[i].miny + ocDis / 2.0 + ocDis*row, boundingBox[i].minz);

					std::vector<uint32_t> zres;
					std::vector<uint32_t> accumulate_res;
					//计算当前块区特征值
					int zCount=0,zmax=0,zmin=-1;
					for (int32_t high=0;high<highnum;high++)
					{
						t.z = boundingBox[i].minz + ocDis / 2.0 + ocDis*high;
						octree[i]._ocTree.radiusNeighbors<unibn::MaxDistance<regis::Point>>(t,ocDis/2.0, zres);
						if (zres.size()!=0)
						{
							accumulate_res.insert(accumulate_res.end(), zres.begin(), zres.end());
							if (zmin ==-1)
								zmin = high;
							zmax =high;
							zCount++;
						}
					}

					float this_fea, this_length;
					if (zCount != 0) {
						this_fea = zCount / (zmax - zmin+1.0);
						this_length = ocDis*(zmax - zmin+1.0);
					}
					else {
						this_fea = 0;
						this_length = 0;
					}
					
					//进行特征值淘汰
					for (int32_t temp=0;temp<accumulate_res.size();temp++)
					{
						if (pointFeature[i][accumulate_res[temp]].zChannel<this_fea)
						{
							pointFeature[i][accumulate_res[temp]].zChannel = this_fea;
							pointFeature[i][accumulate_res[temp]].zlength = this_length;
						}
					}
				}
			}
		}
		std::cout << "time cost:" <<(clock()-begin)/CLOCKS_PER_SEC<< std::endl;
	}
}

void registration::CalculateNormals(std::vector<regis::Vec> _vec)
{
	std::vector < pcl::NormalEstimation<PointT_pcl, pcl::Normal>> ne;
	std::vector<pcl::PointCloud<pcl::Normal>::Ptr> nor;
	
	ne.resize(pcddata.size());
	nor.resize(pcddata.size());

	for (int i =0;i<pcddata.size();i++)
	{
		pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
		nor[i]=cloud_normals;

		ne[i].setInputCloud(pcddata[i]);
		pcl::search::KdTree<PointT_pcl>::Ptr tree(new pcl::search::KdTree<PointT_pcl>());
		ne[i].setSearchMethod(tree);
		// Output datasets
		ne[i].setViewPoint(_vec[i].x, _vec[i].y, faro_altitude[i]);
		std::cout << "faro_altitude: " << _vec[i].x<<","<< _vec[i].y << "," << faro_altitude[i] << std::endl;
		ne[i].setKSearch(24);
		ne[i].compute(*nor[i]);
	}

 	p->removePointCloud("l1");
// 	//p->removePointCloud("l2");
// 	
// 
 	PointCloudColorHandlerCustom<PointT_pcl> tgt_h(pcddata[0], 255, 0, 0);
// 	//PointCloudColorHandlerCustom<PointT_pcl> src_h(pcddata[1], 255, 0, 0);
 	p->addPointCloud(pcddata[0], tgt_h, "l1", vp_1);
// 	//p->addPointCloud(pcddata[1], src_h, "l2", vp_1);
	 
	//p->addPointCloudNormals < PointT_pcl, pcl::Normal> (constPCD, constNormal,10,0.02,"l2",vp_1);
	p->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "l1", vp_1);
 	p->addPointCloudNormals<PointT_pcl,pcl::Normal>(pcddata[0],nor[0],10,0.2,"normals",vp_1);
// 
	
 	p->spin();
 }

void registration::octreeTest(){
// 	std::cout << "Example 1: Searching radius neighbors with default access by public x,y,z variables." << std::endl;
// 	std::cout << "// radiusNeighbors returns indexes to neighboring points." << std::endl;
// 	std::vector<uint32_t> results;
// 	int64_t begin, end;
// 	regis::Point q(-0.0103871, 0.034859, 1.57938);
// 	regis::Point q2 = data[0][0];



// 	begin = clock();
// 	results.clear();
// 	octree[0]._ocTree.radiusNeighbors<unibn::XYDistance<regis::Point>>(q2, 0.05f, results);
// 	std::cout << results.size() << " readius neighborsrd( r = 20m) found for(" << q2.x << "," << q2.y << "," << q2.z << ")"
// 		<< std::endl << "cost:" << ((double)(clock() - begin) / CLOCKS_PER_SEC) << std::endl;
// 
// 	begin = clock();
// 	results.clear();
// 	octree[0]._ocTree.radiusNeighbors<unibn::XYDistance<regis::Point>>(q2, 0.05f, results);
// 	std::cout << results.size() << " readius neighborsrd( r = 20m) found for(" << q2.x << "," << q2.y << "," << q2.z << ")"
// 		<< std::endl << "cost:" << ((double)(clock() - begin) / CLOCKS_PER_SEC) << std::endl;
// 	begin = clock();
// 
// 	results.clear();
// 	octree[0]._ocTree.radiusNeighbors<unibn::XYDistance<regis::Point>>(data[0][5], 0.025f, results);
// 	std::cout << results.size() << " readius neighborsrd( r = 20m) found for(" << q2.x << "," << q2.y << "," << q2.z << ")"
// 		<< std::endl << "cost:" << ((double)(clock() - begin) / CLOCKS_PER_SEC) << std::endl;
}

void registration::writefileTest()
{
	
	for (int tt = 0; tt < data.size(); tt++) {
		std::string t = std::to_string(tt);
		FILE *fp = fopen((t.append(".csv").c_str()), "w");
		for (int32_t i = 0; i < data[tt].size(); i++)
		{
			//fprintf(fp, "%lf,%lf,%lf\n", data[0][i].x, data[0][i].y, data[0][i].z);

			fprintf(fp, "%lf,%lf,%lf,%lf\n", data[0][i].x, data[0][i].y, data[0][i].z, pointFeature[0][i].zChannel);
		}
		fclose(fp);
	}
	
}

void registration::AlignClouds(double ang, std::vector<regis::Vec> _vec)
{
	//最终变换矩阵，输出用
	std::vector < Eigen::Matrix4f> final_matrix;
	final_matrix.resize(data.size());
	//旋转矩阵
	std::vector < Eigen::Matrix4f> rot_matrix;
	rot_matrix.resize(data.size());

	std::cout << "第一次拼接" << std::endl;
	//预处理，抽稀
	subsample(0.04);
	//获取特征数据
	std::vector<PointCloud::Ptr> target_data;
	for (int i = 0; i < data.size(); i++)
	{
		target_data.push_back(PointCloud::Ptr(new PointCloud));
	}
	getFeaturedata(target_data, 0.9, 1);
	std::cout << "特征数据获取完毕. " << std::endl;
	//旋转点云,并初次记录旋转矩阵
	for (size_t j = 0; j < target_data.size(); j++)
	{
		rot_matrix[j] = rot_mat(Eigen::Vector3f(_vec[j].x, _vec[j].y, 0), Eigen::Vector3f(0, 0, 1), ang / 180 * M_PI);
		pcl::transformPointCloud(*target_data[j], *target_data[j], rot_matrix[j]);
	}
	std::cout << "旋转点云完毕." << std::endl;
	std::cout << "进行初次迭代." << std::endl;
	PointCloud::Ptr result(new PointCloud);
	Eigen::Matrix4f transMat;
	for (size_t i = 0; i < target_data.size() - 1; i++)
	{
		int ref,movef;
		switch (i)
		{
		case 0:
			ref = 0,movef = 1;
		case 1:
			ref = 1, movef = 2;
		case 2:
			ref = 1,movef = 3;
		case 3:
			ref = 3, movef = 4;
		case 4:
			ref = 4, movef = 5;
		case 5:
			ref = 5, movef = 11;
		case 6:
			ref = 5, movef = 6;
		case 7:
			ref = 6, movef = 8;
		case 8:
			ref = 6, movef =7;
		case 9:
			ref = 8, movef = 9;
		case 10:
			ref = 9, movef = 10;
		default:
			break;
		}

		pairAlign(target_data[ref], target_data[movef], result, transMat,1.5, true,0.1);
		pcl::transformPointCloud(*target_data[movef], *target_data[movef], transMat);
		final_matrix[movef] = transMat;
	}
	std::cout << "初次迭代结束，按q继续精拼." << std::endl;
	showRotateCloudLeft(target_data);

	std::cout << "进行第二次迭代." << std::endl;
	//预处理，抽稀
	subsample(0.03);
	//获取特征数据
	target_data.clear();
	for (int i = 0; i < data.size(); i++)
	{
		target_data.push_back(PointCloud::Ptr(new PointCloud));
	}
	getFeaturedata(target_data, 0.95,1);

	std::cout << "特征数据获取完毕. " << std::endl;
	//旋转点云，并加入第一次粗拼结果
	for (size_t j = 0; j < target_data.size(); j++)
	{
		pcl::transformPointCloud(*target_data[j], *target_data[j], rot_matrix[j]);
		if (j!=0)
		{
			pcl::transformPointCloud(*target_data[j], *target_data[j],final_matrix[j]);
		}
	}
	std::cout << "旋转点云&第一次结果还原完毕." << std::endl;
	std::cout << "进行第二次迭代." << std::endl;
	for (size_t i = 0; i < target_data.size()-1; i++)
	{
		int ref, movef;
		switch (i)
		{
		case 0:
			ref = 0, movef = 1;
		case 1:
			ref = 1, movef = 2;
		case 2:
			ref = 1, movef = 3;
		case 3:
			ref = 3, movef = 4;
		case 4:
			ref = 4, movef = 5;
		case 5:
			ref = 5, movef = 11;
		case 6:
			ref = 5, movef = 6;
		case 7:
			ref = 6, movef = 8;
		case 8:
			ref = 6, movef = 7;
		case 9:
			ref = 8, movef = 9;
		case 10:
			ref = 9, movef = 10;
		default:
			break;
		}

		pairAlign(target_data[ref], target_data[movef], result, transMat,0.1,false);
		pcl::transformPointCloud(*target_data[movef], *target_data[movef], transMat);
		final_matrix[movef] = transMat * final_matrix[movef];
	}
	std::cout << "第二次迭代完毕." << std::endl;
	//加入平移矩阵
	std::vector<Eigen::Matrix4f> shift_matrix;
	shift_matrix.resize(final_matrix.size());
	for (int i = 0; i < shift_matrix.size(); i++)
	{
		shift_matrix[i] << 1, 0, 0, _vec[i].x,
			0, 1, 0, _vec[i].y,
			0, 0, 1, 0,
			0, 0, 0, 1;
		if (i==0){
			final_matrix[i] = rot_matrix[i] * shift_matrix[i];
		}
		else {
			final_matrix[i] = final_matrix[i] * rot_matrix[i]*shift_matrix[i];
		}
		
	}

	exportpose(final_matrix, fileName);
	showRotateCloudLeft(target_data);
}

void registration::pairAlign(const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr output, Eigen::Matrix4f & final_transform,double co_dis, bool downsample,double leaf_size)
{
	PointCloud::Ptr src(new PointCloud);
	PointCloud::Ptr tgt(new PointCloud);

	pcl::VoxelGrid<PointT_pcl> grid;
	if (downsample)
	{
		grid.setLeafSize(leaf_size, leaf_size, leaf_size);
		grid.setInputCloud(cloud_src);
		grid.filter(*src);

		//tgt = cloud_tgt;
		grid.setInputCloud(cloud_tgt);
		grid.filter(*tgt);
	}
	else
	{
		src = cloud_src;
		tgt = cloud_tgt;
	}
	pcl::GeneralizedIterativeClosestPoint<PointT_pcl, PointT_pcl> reg;
	reg.setTransformationEpsilon(1e-4);

	// Set the maximum distance between two correspondences (src<->tgt) to 10cm
	// Note: adjust this based on the size of your datasets
	reg.setMaxCorrespondenceDistance(co_dis);
	//reg.setEuclideanFitnessEpsilon(1e-10);
	//reg.setRANSACIterations(20);
	//reg.setRANSACOutlierRejectionThreshold(1);

	reg.setInputSource(src);
	reg.setInputTarget(tgt);

	//
	// Run the same optimization in a loop and visualize the results
	Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity(), prev, targetToSource;
	PointCloud::Ptr reg_result = src;
	reg.setMaximumIterations(30);
	for (int i = 0; i < 30; ++i)
	{
		PCL_INFO("Iteration Nr. %d.\n", i);
		//cout << reg.getFinalTransformation() << endl;

		// save cloud for visualization purpose
		src = reg_result;

		// Estimate
		reg.setInputSource(src);
		reg.align(*reg_result);

		//accumulate transformation between each Iteration
		Ti = reg.getFinalTransformation() * Ti;
		cout << Ti << endl;
		cout << "icp registration error" << reg.getFitnessScore() << std::endl;
		//if the difference between this transformation and the previous one
		//is smaller than the threshold, refine the process by reducing
		//the maximal correspondence distance
		if (fabs((reg.getLastIncrementalTransformation() - prev).sum()) < reg.getTransformationEpsilon())
			reg.setMaxCorrespondenceDistance(reg.getMaxCorrespondenceDistance() - 0.001);

		prev = reg.getLastIncrementalTransformation();

		// visualize current state
		//showCloudsRight(points_with_normals_tgt, points_with_normals_src);
	}

	// Get the transformation from target to source
	targetToSource = Ti.inverse();

	//
	// Transform target back in source frame
	pcl::transformPointCloud(*cloud_tgt, *output, targetToSource);

	//add the source to the transformed target
	*output += *cloud_src;

	final_transform = targetToSource;
}

void registration::pairAlign_ICP(const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr output, Eigen::Matrix4f & final_transform, double co_dis, bool downsample, double leaf_size)
{
	PointCloud::Ptr src(new PointCloud);
	PointCloud::Ptr tgt(new PointCloud);

	pcl::VoxelGrid<PointT_pcl> grid;
	if (downsample)
	{
		grid.setLeafSize(leaf_size, leaf_size, leaf_size);
		grid.setInputCloud(cloud_src);
		grid.filter(*src);

		//tgt = cloud_tgt;
		grid.setInputCloud(cloud_tgt);
		grid.filter(*tgt);
	}
	else
	{
		src = cloud_src;
		tgt = cloud_tgt;
	}

	// Compute surface normals and curvature
	// 	PointCloudWithNormals::Ptr points_with_normals_src(new PointCloudWithNormals);
	// 	PointCloudWithNormals::Ptr points_with_normals_tgt(new PointCloudWithNormals);
	// 
	// 	pcl::NormalEstimation<PointT_pcl, PointNormalT> norm_est;
	// 	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	// 	norm_est.setSearchMethod(tree);
	// 	norm_est.setKSearch(15);
	// 
	// 	norm_est.setInputCloud(src);
	// 	norm_est.setViewPoint(27.419594, 6.191127, 0.0);
	// 	norm_est.compute(*points_with_normals_src);
	// 	pcl::copyPointCloud(*src, *points_with_normals_src);
	// 
	// 	norm_est.setInputCloud(tgt);
	// 	norm_est.setViewPoint(20.608019, 6.209724, 0.0);
	// 	norm_est.compute(*points_with_normals_tgt);
	// 	pcl::copyPointCloud(*tgt, *points_with_normals_tgt);

	//
	// Instantiate our custom point representation (defined above) ...
	MyPointRepresentation point_representation;
	// ... and weight the 'curvature' dimension so that it is balanced against x, y, and z
	float alpha[4] = { 1.0, 1.0, 1.0, 1.0 };
	point_representation.setRescaleValues(alpha);

	// Align
	//pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT> reg;
	//pcl::IterativeClosestPointWithNormals<PointNormalT, PointNormalT> reg;
	pcl::GeneralizedIterativeClosestPoint<PointT_pcl, PointT_pcl> reg;
	//pcl::IterativeClosestPoint<PointNormalT, PointNormalT> icp;
	//reg.setTransformationEpsilon(1e-6);

	// Set the maximum distance between two correspondences (src<->tgt) to 10cm
	// Note: adjust this based on the size of your datasets
	reg.setMaxCorrespondenceDistance(co_dis);
	//reg.setEuclideanFitnessEpsilon(1e-10);
	//reg.setRANSACIterations(20);
	//reg.setRANSACOutlierRejectionThreshold(1);

	// Set the point representation
	//reg.setPointRepresentation(boost::make_shared<const MyPointRepresentation>(point_representation));

	//reg.setInputSource(points_with_normals_src);
	//reg.setInputTarget(points_with_normals_tgt);
	reg.setInputSource(src);
	reg.setInputTarget(tgt);



	//
	// Run the same optimization in a loop and visualize the results
	Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity(), prev, targetToSource;
	PointCloud::Ptr reg_result = src;
	//PointCloudWithNormals::Ptr reg_result = points_with_normals_src;
	reg.setMaximumIterations(30);
	for (int i = 0; i < 30; ++i)
	{
		PCL_INFO("Iteration Nr. %d.\n", i);
		//cout << reg.getFinalTransformation() << endl;

		// save cloud for visualization purpose
		src = reg_result;

		// Estimate
		reg.setInputSource(src);
		reg.align(*reg_result);

		//accumulate transformation between each Iteration
		Ti = reg.getFinalTransformation() * Ti;
		cout << Ti << endl;
		cout << "icp registration error" << reg.getFitnessScore() << std::endl;
		//if the difference between this transformation and the previous one
		//is smaller than the threshold, refine the process by reducing
		//the maximal correspondence distance
		if (fabs((reg.getLastIncrementalTransformation() - prev).sum()) < reg.getTransformationEpsilon())
			reg.setMaxCorrespondenceDistance(reg.getMaxCorrespondenceDistance() - 0.001);

		prev = reg.getLastIncrementalTransformation();

		// visualize current state
		//showCloudsRight(points_with_normals_tgt, points_with_normals_src);
	}

	//
	// Get the transformation from target to source
	targetToSource = Ti.inverse();

	//
	// Transform target back in source frame
	pcl::transformPointCloud(*cloud_tgt, *output, targetToSource);

	// 	p->removePointCloud("source");
	// 	p->removePointCloud("target");

	// 	PointCloudColorHandlerCustom<PointT_pcl> cloud_tgt_h(output, 0, 255, 0);
	// 	PointCloudColorHandlerCustom<PointT_pcl> cloud_src_h(cloud_src, 255, 0, 0);
	// 	p->addPointCloud(output, cloud_tgt_h, "target", vp_2);
	// 	p->addPointCloud(cloud_src, cloud_src_h, "source", vp_2);

	//	PCL_INFO("Press q to continue the registration.\n");
	// 	p->spin();
	// 
	// 	p->removePointCloud("source");
	// 	p->removePointCloud("target");

	//add the source to the transformed target
	*output += *cloud_src;

	final_transform = targetToSource;
}

void registration::showCloudsLeft(const PointCloud::Ptr cloud_target, const PointCloud::Ptr cloud_source)
{
	p->removePointCloud("vp1_target");
	p->removePointCloud("vp1_source");

	PointCloudColorHandlerCustom<PointT_pcl> tgt_h(cloud_target, 0, 255, 0);
	PointCloudColorHandlerCustom<PointT_pcl> src_h(cloud_source, 255, 0, 0);
	p->addPointCloud(cloud_target, tgt_h, "vp1_target", vp_1);
	p->addPointCloud(cloud_source, src_h, "vp1_source", vp_1);

	//PCL_INFO("Press q to begin the registration.\n");
	p->spin();
}

void registration::showRotateCloudLeft(const std::vector<PointCloud::Ptr> clouds)
{
	for (int i=0;i<clouds.size();i++)
	{
		p->removePointCloud(std::string("l").append(std::to_string(i)));
		PointCloudColorHandlerCustom<PointT_pcl> tgt_h(clouds[i], int(rand()*255), int(rand() * 255), int(rand() * 255));
		p->addPointCloud(clouds[i], tgt_h, std::string("l").append(std::to_string(i)), vp_1);
	}
	p->spin();
}

void registration::showCloudsRight(const PointCloudWithNormals::Ptr cloud_target, const PointCloudWithNormals::Ptr cloud_source)
{
	p->removePointCloud("source");
	p->removePointCloud("target");


	PointCloudColorHandlerGenericField<PointNormalT> tgt_color_handler(cloud_target, "curvature");
	if (!tgt_color_handler.isCapable())
		PCL_WARN("Cannot create curvature color handler!");

	PointCloudColorHandlerGenericField<PointNormalT> src_color_handler(cloud_source, "curvature");
	if (!src_color_handler.isCapable())
		PCL_WARN("Cannot create curvature color handler!");


	p->addPointCloud(cloud_target, tgt_color_handler, "target", vp_2);
	p->addPointCloud(cloud_source, src_color_handler, "source", vp_2);

	p->spin();
}

void registration::showRotateCloudRight(const std::vector<PointCloud::Ptr> clouds)
{
	for (int i = 0; i < clouds.size(); i++)
	{
		p->removePointCloud(std::string("r").append(std::to_string(i)));
		PointCloudColorHandlerCustom<PointT_pcl> tgt_h(clouds[i], int(rand() * 255), int(rand() * 255), int(rand() * 255));
		p->addPointCloud(clouds[i], tgt_h, std::string("r").append(std::to_string(i)), vp_2);
	}
	p->spin();
}

std::vector<std::vector<regis::Point>> registration::getFeaturedata(double fea,double fea_len)
{
	//获取指定Feature抽稀数据
	std::vector<std::vector<regis::Point>> temp;
 	temp.resize(data.size());

	for (int i=0;i<temp.size();i++)
	{
		for (int j=0;j<subdata_ind[i].size();j++)
		{
			if (pointFeature[i][subdata_ind[i][j]].zChannel>fea )
			{
				temp[i].push_back(data[i][subdata_ind[i][j]]);
			}
		}
		std::cout << "特征数据 " << i << "大小为:" << temp[i].size() << std::endl;
	}
	return temp;
}

void registration::getFeaturedata(std::vector<PointCloud::Ptr> tPtr, double fea, double fea_len)
{
	//获取指定Feature抽稀数据
	for (int i = 0; i < data.size(); i++)
	{
		for (int j = 0; j < subdata_ind[i].size(); j++)
		{
			if (pointFeature[i][subdata_ind[i][j]].zChannel > fea && pointFeature[i][subdata_ind[i][j]].zlength > fea_len)
			{
				tPtr[i]->push_back(PointT_pcl(data[i][subdata_ind[i][j]].x, \
					data[i][subdata_ind[i][j]].y, data[i][subdata_ind[i][j]].z));
			}
		}
		std::cout << "特征数据 " << i << "大小为:" << tPtr[i]->size() << std::endl;
	}
	showRotateCloudLeft(tPtr);
}

void registration::exportpose(std::vector<Eigen::Matrix4f> mat, std::vector<std::string> name)
{
	for (int i=0;i<name.size();i++)
	{
		FILE* file = fopen((name[i].append(".pose")).c_str(), "w");
		fprintf(file, "<?xml version='1.0' encoding='utf-8'?>\n");
		fprintf(file, "<Pose Version=\"3.2.1.584\">\n");
		fprintf(file, "    <Matrix4x4 RowOrder=\"%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf\"\n",
			mat[i](0, 0), mat[i](0, 1), mat[i](0, 2), mat[i](0, 3),
			mat[i](1, 0), mat[i](1, 1), mat[i](1, 2), mat[i](1, 3),
			mat[i](2, 0), mat[i](2, 1), mat[i](2, 2), mat[i](2, 3),
			mat[i](3, 0), mat[i](3, 1), mat[i](3, 2), mat[i](3, 3));
		fprintf(file, "    <Offset X=\"0\" Y=\"0\" Z=\"0\"/>\n");
		fprintf(file, "</Pose>");
		fclose(file);
	}
}



 