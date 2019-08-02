#include <libplyxx.h>

bool areClose(double a, double b) {
	const double EPSILON = 1.0e-1;
	return abs(a - b) < EPSILON;
}

struct Vertex {
	Vertex(double x, double y, double z) : x(x), y(y), z(z) {};
	bool operator==(const Vertex& other) const {
		return areClose(x, other.x) && areClose(y, other.y) && areClose(z, other.z);
	}
	double x, y, z;
};

struct Mesh {
	typedef unsigned int VertexIndex;
	typedef std::vector<Vertex> VertexList;

	Mesh(const VertexList& vertices) : vertices(vertices) {};
	Mesh(VertexList&& vertices) : vertices(std::move(vertices)) {};

	VertexList vertices;
};

void readply(PATH_STRING filename, Mesh::VertexList& vertices) {
	libply::File file(filename);
	const auto& definitions = file.definitions();

	const auto vertexDefinition = definitions.at(0);
	const size_t vertexCount = vertexDefinition.size;
	vertices.reserve(vertexCount);
	libply::ElementReadCallback vertexCallback = [&vertices](libply::ElementBuffer& e) {
		vertices.emplace_back(e[0], e[1], e[2]);
	};

	file.setElementReadCallback("vertex", vertexCallback);
	file.read();
}
