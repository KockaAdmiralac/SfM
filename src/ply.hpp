#include <libplyxx.h>

struct Vertex {
	Vertex(double x, double y, double z) : x(x), y(y), z(z) {};
	bool operator==(const Vertex& other) const;
	double x, y, z;
};

typedef std::vector<Vertex> VertexList;

struct Mesh {
	Mesh(const VertexList& vertices) : vertices(vertices) {};
	Mesh(VertexList&& vertices) : vertices(std::move(vertices)) {};
	VertexList vertices;
};

void readply(PATH_STRING filename, VertexList& vertices);
void writeply(PATH_STRING filename, VertexList& vertices);
