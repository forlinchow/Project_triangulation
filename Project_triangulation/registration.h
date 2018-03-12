#pragma once
#include <string>
#include "Octree.hpp"
#include <pcl/point_cloud.h>
#include <boost/make_shared.hpp>
#include <boost/thread/thread.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_representation.h>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/gicp.h>
//#include <pcl/registration/gicp.h>
//#include <pcl/registration/gicp.h>
//#include <pcl/registration>
#include <pcl/registration/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <vtkAutoInit.h>
VTK_MODULE_INIT(vtkRenderingOpenGL);

#import "iQOpen.dll" no_namespace
#define PI 3.14159265f

typedef pcl::PointXYZ PointT_pcl;
typedef pcl::PointCloud<PointT_pcl> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

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

	template<class PointT>
	struct threadParam
	{
		std::vector<PointT>* ptCloud;
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
		p = new pcl::visualization::PCLVisualizer("window");
		p->createViewPort(0.0, 0, 0.5, 1.0, vp_1);
		p->createViewPort(0.5, 0, 1.0, 1.0, vp_2);
		p->spinOnce();
	};
	~registration() {
		data.clear();
	}
	;
	
	template<class PointT>
	void extractData(std::vector<PointT> pCloud, std::vector<std::string> filename, std::vector<regis::Vec> _vec, int scale = 1);    //多线程版本

	//void extractFLS2PCD_parellel(std::vector<std::string> filename, std::vector<regis::Vec> _vec, int scale = 1);   //多线程版本

	void extractFLSData(std::vector<std::string> filename, std::vector<regis::Vec> _vec, int scale = 1 );

	void extractFLS2PCD(std::vector<std::string> filename, std::vector<regis::Vec> _vec, int scale = 1);
	
	void extractCsvData(std::vector<std::string> filename, std::vector<regis::Vec> _vec);

	regis::Point rotatePoint(regis::Point p, regis::Point rotateCenter, double angle);

	PointT_pcl rotatePoint(PointT_pcl p, regis::Point rotateCenter, double angle);

	double getICPerror(std::vector<regis::Point> moveStation, std::vector<regis::Point> refStation, regis::Vec moveS, regis::Vec refS, int sample);

	double getICPerror_OC(std::vector<regis::Point> moveStation, std::vector<regis::Point> refStation, regis::Vec moveS, regis::Vec refS, int sample, const regis::ocTree* moveOC, const regis::ocTree* refOC);

	double getICPerror_KD(const PointCloud::Ptr moveStation, const PointCloud::Ptr refStation, regis::Vec moveS, regis::Vec refS,\
		int sample, const pcl::KdTreeFLANN<PointT_pcl> refKD);

	double findNearestPoint(regis::Point a, std::vector<regis::Point> ref, double tttt);

	double findNearestPoint_OC(regis::Point a, std::vector<regis::Point> ref, double tttt,const regis::ocTree* refOC);

	regis::Point findNearestPoint(regis::Point a, std::vector<regis::Point> ref);

	void getRotation(std::vector < regis::Vec > _vec, double step = 10);

	void getRotation2(std::vector < regis::Vec > _vec, double step = 10);  //特征点版本
	
	double getRotation_onRender(std::vector < regis::Vec > _vec, double step = 10);

	void getVisableArea();

	void getdatasize();

	void subsample(double subDis);

	void CalculateFeature(double ocDis,bool x_bool=1 ,bool y_bool = 1,bool z_bool = 1);

	void CalculateNormals(std::vector<regis::Vec> _vec);

	void octreeTest(); 

	void writefileTest();

	void AlignClouds(double ang, std::vector<regis::Vec> _vec);

	void pairAlign(const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr output, Eigen::Matrix4f &final_transform,double co_dis=0.1, bool downsample = false, double leaf_size = 0.1);

	void showCloudsLeft(const PointCloud::Ptr cloud_target, const PointCloud::Ptr cloud_source);

	void showRotateCloudLeft(const std::vector<PointCloud::Ptr> clouds);

	void showCloudsRight(const PointCloudWithNormals::Ptr cloud_target, const PointCloudWithNormals::Ptr cloud_source);

	void showRotateCloudRight(const std::vector<PointCloud::Ptr> clouds);

	std::vector <std::vector<regis::Point>> getFeaturedata(double fea = 0.9, double fea_len = 0);

	void getFeaturedata(std::vector<PointCloud::Ptr> clouds,double fea = 0.9, double fea_len = 0);

	// Returns the rotation matrix around a vector  placed at a point , rotate by angle t  
	Eigen::Matrix4f rot_mat(const Eigen::Vector3f& point, const Eigen::Vector3f& vector, const float t)
	{
		float u = vector(0);
		float v = vector(1);
		float w = vector(2);
		float a = point(0);
		float b = point(1);
		float c = point(2);

		Eigen::Matrix4f matrix1,matrix2,matrix3;
		matrix1 << 1, 0, 0, -a,
			0, 1, 0, -b,
			0, 0, 1, -c,
			0, 0, 0, 1;

		matrix2(0, 0) = u*u*(1 - cos(t)) + cos(t); matrix2(1, 0) = u*v*(1 - cos(t)) + w*sin(t); matrix2(2, 0) = u*w*(1 - cos(t)) - v*sin(t); matrix2(3, 0) = 0;
		matrix2(0, 1) = u*v*(1 - cos(t)) - w*sin(t); matrix2(1, 1) = v*v*(1 - cos(t)) + cos(t); matrix2(2, 1) = v*w*(1 - cos(t)) + u*sin(t); matrix2(3, 1) = 0;
		matrix2(0, 2) = u*w*(1 - cos(t)) + v*sin(t); matrix2(1, 2) = v*w*(1 - cos(t)) - u*sin(t); matrix2(2, 2) = w*w*(1 - cos(t)) + cos(t); matrix2(3, 2) = 0;
		matrix2(0, 3) = 0; matrix2(1, 3) = 0; matrix2(2, 3) = 0;matrix2(3,3)=1;
		
		matrix3 << 1, 0, 0, a,
			0, 1, 0, b,
			0, 0, 1, c,
			0, 0, 0, 1;

		return matrix3*matrix2*matrix1;
	}

	void exportpose(std::vector<Eigen::Matrix4f> mat,std::vector<std::string> name);
private: 
	std::vector <std::string> fileName;
	std::vector <std::vector<regis::Point>> data;
	std::vector <std::vector<regis::Feature>> pointFeature;
	std::vector<float> faro_altitude;
	std::vector<regis::ocTree> octree;
	std::vector<regis::Box> boundingBox;

	std::vector <std::vector<regis::Point>> subdata;
	std::vector<regis::ocTree> suboctree;

	std::vector<std::vector<uint32_t>>subdata_ind;
	std::vector<PointCloud::Ptr> pcddata;
	pcl::visualization::PCLVisualizer *p;
	int vp_1, vp_2;
	HANDLE _mutex;
};

// Define a new point representation for < x, y, z, curvature >
class MyPointRepresentation : public pcl::PointRepresentation <PointNormalT>
{
	using pcl::PointRepresentation<PointNormalT>::nr_dimensions_;
public:
	MyPointRepresentation()
	{
		// Define the number of dimensions
		nr_dimensions_ = 4;
	}

	// Override the copyToFloatArray method to define our feature vector
	virtual void copyToFloatArray(const PointNormalT &p, float * out) const
	{
		// < x, y, z, curvature >
		out[0] = p.x;
		out[1] = p.y;
		out[2] = p.z;
		out[3] = p.curvature;
	}
};
