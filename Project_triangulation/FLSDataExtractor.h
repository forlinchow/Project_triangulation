#pragma once

#include "Header Files/FileIO/iqopen.tlh"
#include "Header Files/FileIO/DataExtractor.h"
#include <math.h>
#include <string>
#include <vector>
#include <stdio.h>


struct Point {
	float x;
	float y;
	float z;
	float reflection = 0 ; // scalar
};

struct FLSData {
	std::vector<Point> data;
	Point ScanPosition;
	int rows;
	int cols;
	double yaw;
	double pitch;
};

class FLSDataExtractor:public DataExtractor
{
public:
	FLSDataExtractor() {}
	~FLSDataExtractor() {}
	FLSData getData() { return flsdata;}
	void extract(const std::string& folderName);
	
	//fls doesn't exist write_api
	void write_original_file() {}

	void writeTXT(FILE *);//Ð´ÎÄ¼þ¼Ð

private:
	FLSData flsdata;
};