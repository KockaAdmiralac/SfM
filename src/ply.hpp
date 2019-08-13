#ifndef _PLY_H
#define _PLY_H
#include <libplyxx.h>
#include <opencv2/core.hpp>

struct Mesh {
	Mesh(const std::vector<cv::Point3d> &vertices) : vertices(vertices) {};
	Mesh(std::vector<cv::Point3d> &&vertices) : vertices(std::move(vertices)) {};
	std::vector<cv::Point3d> vertices;
};

void readply(PATH_STRING filename, std::vector<cv::Point3d> &vertices);
void writeply(PATH_STRING filename, std::vector<cv::Point3d> &vertices);
#endif
