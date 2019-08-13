#include <ply.hpp>

bool areClose(double a, double b) {
	const double EPSILON = 1.0e-1;
	return abs(a - b) < EPSILON;
}

bool operator ==(const cv::Point3d &a, const cv::Point3d &b) {
    return areClose(a.x, b.x) && areClose(a.y, b.y) && areClose(a.z, b.z);
}

void readply(PATH_STRING filename, std::vector<cv::Point3d> &vertices) {
	libply::File file(filename);
	const libply::ElementsDefinition& definitions = file.definitions();

	const size_t vertexCount = definitions.at(0).size;
	vertices.reserve(vertexCount);
	libply::ElementReadCallback vertexCallback = [&vertices](libply::ElementBuffer& e) {
		vertices.emplace_back(e[0], e[1], e[2]);
	};

	file.setElementReadCallback("vertex", vertexCallback);
	file.read();
}

void writeply(PATH_STRING filename, std::vector<cv::Point3d> &vertices) {
	libply::FileOut file(filename, libply::File::Format::BINARY_LITTLE_ENDIAN);
	file.setElementsDefinition({
		libply::Element("vertex", vertices.size(), {
			libply::Property("x", libply::Type::FLOAT, false),
			libply::Property("y", libply::Type::FLOAT, false),
			libply::Property("z", libply::Type::FLOAT, false)
		})
	});

	libply::ElementWriteCallback vertexCallback = [&vertices](libply::ElementBuffer& e, size_t index) {
		const cv::Point3d &v = vertices[index];
		e[0] = v.x;
		e[1] = v.y;
		e[2] = v.z;
	};
	file.setElementWriteCallback("vertex", vertexCallback);
	file.write();
}
