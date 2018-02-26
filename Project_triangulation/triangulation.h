#pragma once
// fangyusoftware
// design by forlin at 09.26.2017
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <math.h>
#include <algorithm>
#include <stdio.h>
#include "Octree.hpp"
#include "tinyply.h"
#import "./iQOpen.dll" no_namespace
//#include "iqopen.tlh"
namespace Tri {
	struct Point
	{
		float x;
		float y;
		float z;
		Point() :x(0), y(0), z(0) {}
		Point(float a, float b, float c) :x(a), y(b), z(c) {}
		void get2float() {
			x = float(x);
			y = float(y);
			z = float(z);
		}
	};

	struct Normal
	{
		float nx;
		float ny;
		float nz;
		Normal() :nx(0), ny(0), nz(0) {}
		Normal(float a, float b, float c) :nx(a), ny(b), nz(c) {
			ones();
		}
		void ones() {
			float dis = pow(pow(nx, 2) + pow(ny, 2) + pow(nz, 2), 0.5);
			if (dis != 0)
			{
				nx /= dis;
				ny /= dis;
				nz /= dis;
			}
		}
		float norm() {
			return pow(pow(nx, 2) + pow(ny, 2) + pow(nz, 2), 0.5);
		}
		void reverse() {
			nx = -nx;
			ny = -ny;
			nz = -nz;
		}
	};

	struct nPoint
	{
		float x;
		float y;
		float z;
		float nx;
		float ny;
		float nz;
		nPoint() :x(0), y(0), z(0) {
			nx = 0; ny = 0; nz = 0;
		}
		nPoint(float a, float b, float c, float d, float e, float f) :x(a), y(b), z(c) {
			nx = 0; ny = 0; nz = 0;
		}
		nPoint(Point a, Normal na) {
			x = a.x; y = a.y; z = a.z;
			nx = na.nx; ny = na.ny; nz = na.nz;
		}
	};

	struct Param
	{
		int searchRadiu4normals;
		float a;

		Param() :searchRadiu4normals(3), a(1.0) {}
	};

	struct nearPoint
	{
		Tri::Point point;
		float dis;
		nearPoint() :point(), dis(0) {}
		nearPoint(Tri::Point p, float d) :point(p), dis(d) {}
	};

	bool compDisSmall2Big(const nearPoint &a, const nearPoint &b) {
		return a.dis < b.dis;
	}
}

float operator*(const Tri::Normal& a, const Tri::Normal& b) {
	return (a.nx*b.nx + a.ny*b.ny + a.nz*b.nz);
}

class Triangulation
{
public:
	Triangulation() { }
	~Triangulation() { }

	void set_Mutex(HANDLE* mu) {
		mutex = *mu;
	}

	Tri::Point* findNearest3Points(IiQLibIfPtr libRef, int row, int col, int rowSize, int colSize, Tri::Point center, Tri::Point scanPosition) {
		Tri::Point *res = new Tri::Point[3];

		std::vector<Tri::nearPoint>temp;

		double x, y, z; int refl;

		//获取起算子左上角坐标
		int orign_row = row - (triParam.searchRadiu4normals - 1) / 2;
		int orign_col = col - (triParam.searchRadiu4normals - 1) / 2;

		//循环获取所有距离
		for (int i = 0; i < triParam.searchRadiu4normals; i++)
		{
			for (int j = 0; j < triParam.searchRadiu4normals; j++) {
				int r, c;
				r = orign_row + i;
				c = orign_col + j;
				if (r < 0)
				{
					r = rowSize + r;
				}
				if (r >= rowSize)
				{
					r = r - rowSize;
				}
				if (c < 0)
				{
					c = colSize + c;
				}
				if (c >= colSize)
				{
					c = c - colSize;
				}
				libRef->getScanPoint(0, r, c, &x, &y, &z, &refl);
				float t_dis = pow(x - center.x, 2) + pow(y - center.y, 2) + pow(z - center.z, 2);
				Tri::Point t_Point = Tri::Point(x, y, z);
				temp.push_back(Tri::nearPoint(t_Point, t_dis));
			}
		}
		sort(temp.begin(), temp.end(), Tri::compDisSmall2Big);

		res[0] = temp[1].point;
		res[1] = temp[2].point;
		res[2] = temp[3].point;
		return res;
	}

	Tri::Normal calculatNormals(IiQLibIfPtr libRef, int row, int col, int rowSize, int colSize, Tri::Point center, Tri::Point scanPosition) {
		Tri::Point* res = findNearest3Points(libRef, row, col, rowSize, colSize, center, scanPosition);

		//计算平面法向量
		float x1, y1, z1, x2, y2, z2, x3, y3, z3;
		x1 = res[0].x; x2 = res[1].x; x3 = res[2].x;
		y1 = res[0].y; y2 = res[1].y; y3 = res[2].y;
		z1 = res[0].z; z2 = res[1].z; z3 = res[2].z;
		Tri::Normal nor1 = Tri::Normal((y3 - y1)*(z3 - z1) - (z2 - z1)*(y3 - y1), (x3 - x1)*(z2 - z1) - (x2 - x1)*(z3 - z1), (x2 - x1)*(y3 - y1) - (x3 - x1)*(y2 - y1));
		Tri::Normal nor2 = Tri::Normal(scanPosition.x - center.x, scanPosition.y - center.y, scanPosition.z - center.z);

		if (nor1*nor2 / (nor1.norm()*nor2.norm()) < 0)
		{
			nor1.reverse();
		}
		nor1.ones();

		delete res;
		return nor1;
	}

	void exportNPointdata(std::string folderName) {
		folderName.replace(folderName.end() - 3, folderName.end(), "csv");

		WaitForSingleObject(mutex, INFINITE);
		std::cout << "export data -" << folderName << std::endl;
		ReleaseMutex(mutex);

		FILE *out;
		errno_t err;
		if ((err = fopen_s(&out, folderName.c_str(), "w")) != 0)    //不同点2
		{
			std::cout << "无法打开此文件\n";            //如果打不开，就输出打不开
			exit(0);                               //终止程序
		}

		for (int i = 0; i < nPoints.size(); i++)
		{
			fprintf(out, "%f,%f,%f,%f,%f,%f\n", nPoints[i].x, nPoints[i].y, nPoints[i].z, nPoints[i].nx, nPoints[i].ny, nPoints[i].nz);
		}
		fclose(out);
	}

	int run(std::string folderName) {
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
		IiQLibIfPtr libRef = static_cast<IiQLibIfPtr>(liPtr);//点云数据IO

		libRef->load(folderName.c_str());//加载数据
		IiQObjectIfPtr libObj = libRef->getScanObject(0);
		IiQScanObjIfPtr scanRef = libObj->getScanObjSpecificIf();//法如扫描属性IO
		scanRef->load();//加载属性

		double x, y, z, angle;

		int rows = libRef->getScanNumRows(0);
		int cols = libRef->getScanNumCols(0);
		for (int col = 0; col < cols; col++)
		{
			for (int row = 0; row < rows; row++) {
				double x, y, z;
				int refl;
				libRef->getScanPoint(0, row, col, &x, &y, &z, &refl);     // 读取数据,x,y,z 点坐标， refl为反射值
				//points1.push_back(Tri::Point(x, y, z));

				//WaitForSingleObject(mutex, INFINITE);
				//std::cout <<x<< y <<z<< std::endl;

				//获取起算点坐标
// 				double x, y, z; int refl;
// 				Tri::Point scanPosition;
// 				libRef->getScanPosition(0, &x, &y, &z);
// 				scanPosition.x = x; scanPosition.y = y; scanPosition.z = z;
				Tri::Point scanPosition = Tri::Point(0, 0, 0);
				Tri::Point center = Tri::Point(x, y, z);
				nPoints.push_back(Tri::nPoint(center, calculatNormals(libRef, row, col, rows, cols, center, scanPosition)));
				//ReleaseMutex(mutex);

			}
		}

		// 		WaitForSingleObject(mutex, INFINITE);
		// 		std::cout << "creating octree for "<< folderName<<std::endl;
		// 		//unibn::OctreeParams params;
		// 		//octree.initialize(points1);
		// 		std::cout << "octree for " << folderName << "done"<<std::endl;
		// 		ReleaseMutex(mutex);

		exportNPointdata(folderName);
		return 0;
	};

	int calNormal(std::string folderName,int scale=1) {
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
		IiQLibIfPtr libRef = static_cast<IiQLibIfPtr>(liPtr);//点云数据IO

		libRef->load(folderName.c_str());//加载数据
		IiQObjectIfPtr libObj = libRef->getScanObject(0);
		IiQScanObjIfPtr scanRef = libObj->getScanObjSpecificIf();//法如扫描属性IO
		scanRef->load();//加载属性

		double x, y, z, angle;
		nPoints.clear();

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

		int rows = libRef->getScanNumRows(0);
		int cols = libRef->getScanNumCols(0);
		Tri::nPoint temp;
		for (int col = 0; col < cols; col=col+scale)
		{
			for (int row = 0; row < rows; row=row+scale) {
				double x, y, z;
				int refl;
				libRef->getScanPoint(0, row, col, &x, &y, &z, &refl);     // 读取数据,x,y,z 点坐标， refl为反射值

				if (x==0 & y==0 & z==0)
				{
					continue;
				}

				Tri::Point scanPosition = Tri::Point(0, 0, 0);
				Tri::Point center = Tri::Point(x,y,z);
				Tri::Normal tN = calculatNormals(libRef, row, col, rows, cols, center, scanPosition);

				temp.x = x*R[0][0] + y*R[1][0] + z*R[2][0];
				temp.y = x*R[0][1] + y*R[1][1] + z*R[2][1];
				temp.z = x*R[0][2] + y*R[1][2] + z*R[2][2];
				temp.nx = tN.nx*R[0][0] + tN.ny*R[1][0] + tN.nz*R[2][0];
				temp.ny = tN.nx*R[0][1] + tN.ny*R[1][1] + tN.nz*R[2][1];
				temp.nz = tN.nx*R[0][2] + tN.ny*R[1][2] + tN.nz*R[2][2];

				nPoints.push_back(temp);
			}
		}

		exportNPointdata(folderName);
		return 0;
	};

	int mesh(std::string folderName) {  
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
		IiQLibIfPtr libRef = static_cast<IiQLibIfPtr>(liPtr);//点云数据IO

		libRef->load(folderName.c_str());//加载数据
		IiQObjectIfPtr libObj = libRef->getScanObject(0);
		IiQScanObjIfPtr scanRef = libObj->getScanObjSpecificIf();//法如扫描属性IO
		scanRef->load();//加载属性

		int rows = libRef->getScanNumRows(0);
		int cols = libRef->getScanNumCols(0);

		//ply属性
		std::vector<float> verts;
		std::vector<float> norms;
		std::vector<uint8_t> colors;

		std::vector<int32_t> vertexIndicies;
		std::vector<float> faceTexcoords;
		Tri::Normal t_norm;

		for (int col = 0; col < cols; col++)
		{
			for (int row = 0; row < rows; row++) {
				double x, y, z;int refl;
				libRef->getScanPoint(0, row, col, &x, &y, &z, &refl);     // 读取数据,x,y,z 点坐标， refl为反射值
				Tri::Point scanPosition = Tri::Point(0, 0, 0);
				Tri::Point center = Tri::Point(x, y, z);
				t_norm = calculatNormals(libRef, row, col, rows, cols, center, scanPosition);

				verts.push_back(x);
				verts.push_back(y);
				verts.push_back(z);
				
				norms.push_back(t_norm.nx);
				norms.push_back(t_norm.ny);
				norms.push_back(t_norm.nz);

				/*colors.push_back(refl * 255);
				colors.push_back(refl * 255);
				colors.push_back(refl * 255);
				colors.push_back(255);*/


				
				if (x == 0 && y == 0 && z == 0)
				{
					continue;
				}
				if (row==rows-1 || col==cols-1)
				{
					continue;
				}

				if (calculatEdge(libRef, row, col, rows, cols))
				{
					vertexIndicies.push_back(upDateNum(row, col, rows, cols));
					vertexIndicies.push_back(upDateNum(row + 1, col, rows, cols));
					vertexIndicies.push_back(upDateNum(row, col + 1, rows, cols));
					vertexIndicies.push_back(upDateNum(row + 1, col + 1, rows, cols));
					vertexIndicies.push_back(upDateNum(row + 1, col, rows, cols));
					vertexIndicies.push_back(upDateNum(row, col + 1, rows, cols));
				}
				/*faceTexcoords.push_back(refl);
				faceTexcoords.push_back(refl);
				faceTexcoords.push_back(refl);
				faceTexcoords.push_back(refl);
				faceTexcoords.push_back(refl);
				faceTexcoords.push_back(refl);*/
			}
		}
			
	
		tinyply::PlyFile myFile;
		myFile.add_properties_to_element("vertex", { "x", "y", "z" }, verts);
		myFile.add_properties_to_element("vertex", { "nx", "ny", "nz" }, norms);
		//myFile.add_properties_to_element("vertex", { "red", "green", "blue", "alpha" }, colors);

		// List property types must also be created with a count and type of the list (data property type
		// is automatically inferred from the type of the vector argument). 
		myFile.add_properties_to_element("face", { "vertex_indices" }, vertexIndicies, 3, tinyply::PlyProperty::Type::UINT8);
		//myFile.add_properties_to_element("face", { "texcoord" }, faceTexcoords, 6, tinyply::PlyProperty::Type::UINT8);
		myFile.comments.push_back("generated by FYtech");

		folderName.replace(folderName.end() - 3, folderName.end(), "ply");
		std::filebuf fb;
		fb.open(folderName, std::ios::out | std::ios::binary);
		std::ostream outputStream(&fb);
		myFile.write(outputStream, true);

		fb.close();
		libRef = NULL;
		libObj = NULL;
		scanRef = NULL;
		liPtr = NULL;
		CoUninitialize();
		return 0;
	}

	bool calculatEdge(IiQLibIfPtr libRef, int row, int col, int rowSize, int colSize) {
		Tri::Point a, b, c, d;
		int refl;
		double x,y,z;
		libRef->getScanPoint(0, row, col, &x, &y, &z, &refl);
		a = Tri::Point(x, y, z);


		libRef->getScanPoint(0, row + 1, col, &x, &y, &z, &refl);
		b = Tri::Point(x, y, z);
		libRef->getScanPoint(0, row, col + 1, &x, &y, &z, &refl);
		c = Tri::Point(x, y, z);
		libRef->getScanPoint(0, row + 1, col + 1, &x, &y, &z, &refl);
		d = Tri::Point(x, y, z);



// 		libRef->getScanPoint(0, row, col, (double*)&a.x, (double*)&a.y, (double*)&a.z, &refl);
// 		libRef->getScanPoint(0, row+1, col, (double*)&b.x, (double*)&b.y, (double*)&b.z, &refl);
// 		libRef->getScanPoint(0, row, col+1, (double*)&c.x, (double*)&c.y, (double*)&c.z, &refl);
// 		libRef->getScanPoint(0, row+1, col+1, (double*)&d.x, (double*)&d.y, (double*)&d.z, &refl);

	//	a.get2float(); b.get2float();c.get2float();

		if (dist2(a,b)>0.5 || dist2(a,c)>0.5 || dist2(c,b)>0.5 || dist2(b,d)>0.5 || dist2(c,d)>0.5 )
		{
			return false;
		}
		else
		{
			return true;
		}
				//边长超过阈值则结束
	}

	double dist2(Tri::Point a, Tri::Point b) {
		return std::pow(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2) + std::pow(a.z - b.z, 2), 0.5);
	}


	int upDateNum(int row, int col, int rows, int cols) {
		int tr, tc;
		if (col < cols) {
			tc = col;
		}
		else {
			tc = col - cols;
		}

		if (row < rows) {
			tr = row;
		}
		else {
			tr = row;
			tc = (cols-1)-tc;
		}

		return (tc*rows + tr);
	}
protected:

private:
	HANDLE mutex;
	Tri::Param triParam;
	unibn::Octree<Tri::Point> octree;
	std::vector<Tri::Point> points1;
	std::vector<Tri::nPoint> nPoints;

	//std::vector<std::vector<Tri::Point>> points;

};

// void FLSDataExtractor::extract(const std::string & folderName)
// {
// 	CoInitialize(NULL);
// 	// 以下liscence需要整段输入，key放入liscence key
// 	BSTR licenseCode =
// 		L"FARO Open Runtime License\n"
// 		L"Key:W2CW4PNRTCTXXJ6T6KXYSRUPL\n" // License Key
// 		L"\n"
// 		L"The software is the registered property of "
// 		L"FARO Scanner Production GmbH, Stuttgart, Germany.\n"
// 		L"All rights reserved.\n"
// 		L"This software may only be used with written permission "
// 		L"of FARO Scanner Production GmbH, Stuttgart, Germany.";
// 	IiQLicensedInterfaceIfPtr liPtr(__uuidof(iQLibIf));
// 	liPtr->License = licenseCode;
// 	IiQLibIfPtr libRef = static_cast<IiQLibIfPtr>(liPtr);//点云数据IO
// 
// 	libRef->load(folderName.c_str());//加载数据
// 	IiQObjectIfPtr libObj = libRef->getScanObject(0);
// 	IiQScanObjIfPtr scanRef = libObj->getScanObjSpecificIf();//法如扫描属性IO
// 	scanRef->load();//加载属性
// 
// 	double x, y, z, angle;
// 
// 	// 扫描原点
// 	libRef->getScanPosition(0, &x, &y, &z);
// 	flsdata.ScanPosition.x = x;
// 	flsdata.ScanPosition.y = y;
// 	flsdata.ScanPosition.z = z;
// 
// 	// 获取变换矩阵的参数
// 	libRef->getScanOrientation(0, &x, &y, &z, &angle);
// 	// 由文档公式得
// 	double ca = cos(-angle);
// 	double sa = sin(-angle);
// 	// 变换矩阵
// 	double R[3][3];
// 	R[0][0] = x * x * (1 - ca) + ca;
// 	R[0][1] = y * x * (1 - ca) - z * sa;
// 	R[0][2] = z * x * (1 - ca) + y * sa;
// 	R[1][0] = x * y * (1 - ca) + z * sa;
// 	R[1][1] = y * y * (1 - ca) + ca;
// 	R[1][2] = z * y * (1 - ca) - x * sa;
// 	R[2][0] = x * z * (1 - ca) - y * sa;
// 	R[2][1] = y * z * (1 - ca) + x * sa;
// 	R[2][2] = z * z * (1 - ca) + ca;
// 
// 	// 获取方向角
// 	scanRef->getCompassAxis(&x, &y, &z);
// 	double yaw;
// 	yaw = (180 / M_PI) * atan2(y, x);
// 	if (yaw < 0)
// 		yaw = -(yaw + 90);
// 	else
// 		yaw = 270 - yaw;
// 	flsdata.yaw = yaw;
// 
// 	// 获取仰角
// 	scanRef->getInclinometerAxis(&x, &y, &z);
// 	double xy2 = hypot(x, y);
// 	double pitch = (180 / M_PI) * atan2(xy2, z);
// 	flsdata.pitch = pitch;
// 
// 	int rows = libRef->getScanNumRows(0);
// 	int cols = libRef->getScanNumCols(0);
// 
// 
// 	//access all points points by point
// 	for (int col = 0; col < cols; col++)
// 	{
// 		for (int row = 0; row < rows; row++) {
// 			double x, y, z;
// 			int refl;
// 			int result;
// 			result = libRef->getScanPoint(0, row, col, &x, &y, &z, &refl);     // 读取数据,x,y,z 点坐标， refl为反射值
// 			Point tmp;
// 			tmp.x = x*R[0][0] + y*R[1][0] + z*R[2][0] + flsdata.ScanPosition.x;
// 			tmp.y = x*R[0][1] + y*R[1][1] + z*R[2][1] + flsdata.ScanPosition.y;
// 			tmp.z = x*R[0][2] + y*R[1][2] + z*R[2][2] + flsdata.ScanPosition.z;
// 			tmp.reflection = refl;
// 			flsdata.data.push_back(tmp);
// 		}
// 	}
// 
// 	// free
// 	libRef = NULL;
// 	libObj = NULL;
// 	scanRef = NULL;
// 	liPtr = NULL;
// 	CoUninitialize();
// }