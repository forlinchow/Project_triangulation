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

	
	//��ȡ���ݼ�
	std::vector<FYData::Point> get_data() {
		return data_vector;
	};

	//��ȡ���ݼ���С
	int get_data_size() {
		return data_vector.size();
	}

	//д�ļ�����,δʵ��
	void write_original_data(const std::string &);

	//��ȡ�ļ�����
	void read_data(const std::string &file_name);


	FYData::Point get_min_point() {
		return min_Point;
	};

	FYData::Point get_max_point() {
		return max_Point;
	}

protected:
	// �������ʵ�ֵ����ݶ�ȡ��������ͬ��ʽ���ļ�ʵ�ֲ�ͬ
	virtual void extract(FILE *input_data) {};

	//�������ʵ�ֵ�ԭʼ����д�뺯������ͬ��ʽ���ļ�ʵ�ֲ�ͬ
	virtual void write_original_file(FILE *output) {};

	bool format_check(const std::string &file_name) const;
	 
protected:

	// �����ļ���ʽ
	std::string File_Format;
	
	// ��������������һ���洢��ȡ�������ݵ�˽�г�Ա����
	std::vector<FYData::Point> data_vector;

	FYData::Point min_Point;
	FYData::Point max_Point;
	bool fill_data = false;
};

#endif
