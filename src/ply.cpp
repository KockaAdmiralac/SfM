#include <ply.hpp>

bool areClose(double a, double b) {
	const double EPSILON = 1.0e-1;
	return abs(a - b) < EPSILON;
}

bool operator ==(const Vertex& a, const Vertex& b) {
    return areClose(a.x, b.x) && areClose(a.y, b.y) && areClose(a.z, b.z);
}

void readply(PATH_STRING filename, VertexList& vertices) {
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
