#include "registration.h"


void main(){
// 	HANDLE main_Mutex = CreateMutex(NULL,false,NULL);
// 	
// 	HANDLE hThrd1;
// 	HANDLE hThrd2;
// 	DWORD thrd1Id;
// 	DWORD thrd2Id;
// 	std::string a1 = "GZ_lsjz_20170908_026.fls";
// 	std::string a2 = "Gz_lsjz0914_002.fls";
// 	Triangulation t1;
// 	Triangulation t2;
// 
// 	TriParam tp1;
// 	tp1.a = &t1;
// 	tp1.folderpath = a1;
// 	tp1.main_Mutex = &main_Mutex;
// 
// 	TriParam tp2;
// 	tp2.a = &t2;
// 	tp2.folderpath = a2;
// 	tp2.main_Mutex = &main_Mutex;
// 
// 
// 	std::cout << "creating thread 1\n";
// 	hThrd1 = CreateThread(NULL, 0, Thr2eadProc, &tp1, 0, &thrd1Id);
// 	SetThreadPriority(hThrd1, 31);
// 	std::cout << "creating thread 2\n";
// 	hThrd2 = CreateThread(NULL, 0, Thr2eadProc, &tp2, 0, &thrd2Id);
// 	SetThreadPriority(hThrd2, 31);
// 	WaitForSingleObject(hThrd1, INFINITE);
// 	WaitForSingleObject(hThrd2, INFINITE);
// 
// 	CloseHandle(hThrd1);
// 	CloseHandle(hThrd2);

//自动拼接部分代码测试
	std::vector<std::string> filepath;
// 	filepath.push_back(std::string("Gd_zh_171022_043 - Cloud.csv"));
// 	filepath.push_back(std::string("Gd_zh_171022_044 - Cloud.csv"));
// 	filepath.push_back(std::string("Gd_zh_171022_045 - Cloud.csv"));
// 	filepath.push_back(std::string("Gd_zh_171022_046 - Cloud.csv"));
// 	filepath.push_back(std::string("Gd_zh_171022_047 - Cloud.csv"));
	filepath.push_back(std::string("Gd_zh_171022_043.fls"));
 	filepath.push_back(std::string("Gd_zh_171022_044.fls"));
 	filepath.push_back(std::string("Gd_zh_171022_045.fls"));
// 	filepath.push_back(std::string("Gd_zh_171022_046.fls"));
//  	filepath.push_back(std::string("Gd_zh_171022_047.fls"));

	std::vector < regis::Vec > _vec;
	_vec.push_back(regis::Vec(27.39,6.21));
	_vec.push_back(regis::Vec(20.57,6.18));
	_vec.push_back(regis::Vec(11.98,5.62));
//  	_vec.push_back(regis::Vec(9.97,2.27));
//  	_vec.push_back(regis::Vec(3.48,2.39));

// 	043，27.39，6.21
// 		044，20.57.6.18，
// 		 045，11.98，5.62，
// 		046，9.97，2.27
// 		047，3.48.2.39

	registration t;
	t.extractFLSData(filepath,_vec, 1);
	t.CalculateFeature(0.05,1,0,0);
// 	
	//t.extractCsvData(filepath, _vec);
// // 	t.getdatasize();
  	t.getRotation2(_vec,5);
 	//t.octreeTest();
// // 	Triangulation t;
// 	for (size_t i=0;i<filepath.size();i++) 
// 	{
// 		std::cout << "cal " << filepath[i] << std::endl;
// 		t.calNormal(filepath[i],2);
// 	}
	//t.writefileTest();
	system("pause");
}
