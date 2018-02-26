#pragma once
#ifndef POINT_CLOUD_DATA_EXTRACTOR_H
#define POINT_CLOUD_DATA_EXTRACTOR_H

#include <iostream>
#include <string>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <algorithm>
#include <QDialog>

#define MAX_FILE_ROW_LENGTH 1024 * 4

namespace FYData {
	struct Scalar {
		std::string name;
		float value;

		Scalar(std::string n, float v) :name(n), value(v) {}
	};

	struct Point {
		float x;
		float y;
		float z;
		std::vector<Scalar> scalar;

		void Add_scalar(std::string n, float v) {
			scalar.push_back(Scalar(n, v));
		}

		float Get_scalar(std::string n) {
			for (int i = 0; i < scalar.size(); i++) {
				if (scalar[i].name == n) {
					return scalar[i].value;
				}
			}
		}
	};
}

class DataExtractor : public QDialog {

public:
	
	DataExtractor(QWidget *parent = 0);

	virtual ~DataExtractor();

	
	//获取数据集
	std::vector<FYData::Point> get_data() {
		return data_vector;
	};

	//获取数据集大小
	int get_data_size() {
		return data_vector.size();
	}

	//写文件函数,未实现
	void write_original_data(const std::string &);

	//读取文件函数
	void read_data(const std::string &file_name);


	FYData::Point get_min_point() {
		return min_Point;
	};

	FYData::Point get_max_point() {
		return max_Point;
	}

protected:
	// 子类必须实现的数据读取函数，不同格式的文件实现不同
	virtual void extract(FILE *input_data) {};

	//子类必须实现的原始数据写入函数，不同格式的文件实现不同
	virtual void write_original_file(FILE *output) {};

	bool format_check(const std::string &file_name) const;
	 
protected:

	// 数据文件格式
	std::string File_Format;
	
	// 子类必须包含至少一个存储读取出的数据的私有成员变量
	std::vector<FYData::Point> data_vector;

	FYData::Point min_Point;
	FYData::Point max_Point;
	bool fill_data = false;
};

#endif
