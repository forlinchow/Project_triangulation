#pragma once
#include <string>
#include "Octree.hpp"
#import "iQOpen.dll" no_namespace
#define PI 3.14159265f

namespace regis {


	struct Point
	{
		double x;
		double y;
		double z;
		Point() :x(0), y(0), z(0) {}
		Point(double a, double b, double c) :x(a), y(b), z(c) {}

		double L2dis(Point b) {
			return std::pow(std::pow(x - b.x, 2) + std::pow(y - b.y, 2) + std::pow(z - b.z, 2), 0.5);
		}
		double XYdis(Point b) {
			return std::pow(std::pow(x - b.x, 2) + std::pow(y - b.y, 2), 0.5);
		}
	};

	struct Box
	{
		double minx;
		double miny;
		double minz;
		double maxx;
		double maxy;
		double maxz;

		Box() :minx(0), miny(0), minz(0), maxx(0), maxy(0), maxz(0) {}
		Box(double a, double b, double c, double d, double e, double f) : minx(a), miny(b), minz(c), maxx(d), maxy(e), maxz(f) {}

		Point center() {
			return Point((maxx + minx) / 2.0, (maxy + miny) / 2.0, (maxz + minz) / 2.0);
		}
	};

	struct Feature
	{
		float xChannel;
		float xlength;
		float yChannel;
		float ylength;
		float zChannel;
		float zlength;
		Feature() : xChannel(0), yChannel(0), zChannel(0), xlength(0), ylength(0), zlength(0) {}
		Feature(float a, float b, float c,float d, float e, float f) :xChannel(a), yChannel(b), zChannel(c),xlength(d), ylength(e), zlength(f) {}

	};

	struct ocTree
	{
		unibn::Octree<regis::Point> _ocTree;
		unibn::OctreeParams params;

		ocTree(){};
	};

	struct Vec 
	{
		double x;
		double y;
		Vec():x(0),y(0){}
		Vec(double a,double b):x(a),y(b){}
	};

	struct threadParam
	{
		std::vector<regis::Point>* ptCloud;
		ocTree* octree;
		std::string filepath;
		regis::Vec offset;
		HANDLE* _mutex;
		int scale;
		threadParam() :scale(10) { 
			octree = NULL; 
		}
	};

	struct findNearestParam 
	{
		Point a;
		std::vector<Point> ref;
		Point result;
		double* dis;
		int* threadcount;
	};
}

class registration
{
public:
	registration() {
	};
	~registration() {
		data.clear();
	}
	;

	void extractData(std::vector<std::string> filename, std::vector<regis::Vec> _vec, int scale = 1);    //多线程版本

	void extractFLSData(std::vector<std::string> filename, std::vector<regis::Vec> _vec, int scale = 1 );
	
	void extractCsvData(std::vector<std::string> filename, std::vector<regis::Vec> _vec);

	regis::Point rotatePoint(regis::Point p, regis::Point rotateCenter, double angle);

	double getICPerror(std::vector<regis::Point> moveStation, std::vector<regis::Point> refStation, regis::Vec moveS, regis::Vec refS, int sample);

	double getICPerror_OC(std::vector<regis::Point> moveStation, std::vector<regis::Point> refStation, regis::Vec moveS, regis::Vec refS, int sample, const regis::ocTree* moveOC, const regis::ocTree* refOC);

	double findNearestPoint(regis::Point a, std::vector<regis::Point> ref, double tttt);

	double findNearestPoint_OC(regis::Point a, std::vector<regis::Point> ref, double tttt,const regis::ocTree* refOC);

	regis::Point findNearestPoint(regis::Point a, std::vector<regis::Point> ref);

	void getRotation(std::vector < regis::Vec > _vec, double step = 10);

	void getRotation2(std::vector < regis::Vec > _vec, double step = 10);  //特征点版本

	void getVisableArea();

	void getdatasize();

	void subsample(double subDis);

	void CalculateFeature(double ocDis,bool x_bool=1 ,bool y_bool = 1,bool z_bool = 1);

	void octreeTest();

	void writefileTest();

	std::vector <std::vector<regis::Point>> getFeaturedata(double fea = 0.9, double fea_len = 0);

private: 
	std::vector <std::vector<regis::Point>> data;
	std::vector <std::vector<regis::Feature>> pointFeature;
	std::vector<regis::ocTree> octree;
	std::vector<regis::Box> boundingBox;

	std::vector <std::vector<regis::Point>> subdata;
	std::vector<regis::ocTree> suboctree;

	std::vector<std::vector<uint32_t>>subdata_ind;

	HANDLE _mutex;
};
