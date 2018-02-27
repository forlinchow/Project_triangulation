#include "registration.h"
#include <time.h>
#include <algorithm>
#include <windows.h>
#include <cstdlib>
#include <time.h>
#include <iostream>
#include <string.h>

DWORD WINAPI readFLS(LPVOID lpParameter) {
	CoInitialize(NULL);
		regis::threadParam* t = (regis::threadParam*)lpParameter;

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

		std::vector<regis::Point> temp;
		for (int col = 0; col < cols; col = col + t->scale)
		{
			for (int row = 0; row < rows; row = row + t->scale) {
				libRef->getScanPoint(0, row, col, &x, &y, &z, &refl);     // 读取数据,x,y,z 点坐标， refl为反射值
				if (x == 0 && y == 0 && z == 0)
				{
					continue;
				}
				//temp.push_back(regis::Point(x*R[0][0] + y*R[1][0] + z*R[2][0], x*R[0][1] + y*R[1][1] + z*R[2][1], x*R[0][2] + y*R[1][2] + z*R[2][2]));
				temp.push_back(regis::Point(x*R[0][0] + y*R[1][0] + z*R[2][0] + t->offset.x, x*R[0][1] + y*R[1][1] + z*R[2][1] + t->offset.y, x*R[0][2] + y*R[1][2] + z*R[2][2]));
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

void registration::extractData(std::vector<std::string> filename, std::vector<regis::Vec> _vec, int scale)
{
	CoInitialize(NULL);
	std::vector<HANDLE> pth;
	data.resize(filename.size());
	octree.resize(filename.size());
	std::vector<regis::threadParam> _param;

	for (int i = 0; i < filename.size(); i++)
	{
		regis::threadParam tParam;
		tParam.scale = scale;
		tParam.filepath = filename[i];
		tParam.ptCloud = &(data[i]);
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

				if (xx*xx+yy*yy+zz*zz>225)
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
			min_err_angle = i * 10;
			std::cout << "best angle: " << min_err_angle << std::endl;
			std::cout << "ICP error:" << res << std::endl;
		}

		target_octree.swap(std::vector<regis::ocTree>());
	}
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
	long int deal_size = 0;
	std::vector <std::vector<regis::Point>> temp = data;

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

std::vector<std::vector<regis::Point>> registration::getFeaturedata(double fea,double fea_len)
{
	std::vector<std::vector<regis::Point>> temp;
 	temp.resize(pointFeature.size());

	for (int i=0;i<temp.size();i++)
	{
		
		for (int j=0;j<pointFeature[i].size();j++)
		{
			if (pointFeature[i][j].zChannel>fea )
			{
				temp[i].push_back(data[i][j]);
			}
		}
		std::cout << "特征数据 " << i << "大小为:" << temp[i].size() << std::endl;
	}

	return temp;
}



 