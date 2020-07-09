#include "ply.h"

#include <climits>
#include <fstream>
#include <sstream>

// https://stackoverflow.com/questions/105252/how-do-i-convert-between-big-endian-and-little-endian-values-in-c
template <typename T>
T swap_endian(T u)
{
	static_assert(CHAR_BIT == 8, "CHAR_BIT != 8");

	union {
		T u;
		unsigned char u8[sizeof(T)];
	} source, dest;

	source.u = u;

	for (size_t k = 0; k < sizeof(T); k++)
		dest.u8[k] = source.u8[sizeof(T) - k - 1];

	return dest.u;
}

namespace ply {

struct Header {
	int VertexCount		  = 0;
	int FaceCount		  = 0;
	int XElem			  = -1;
	int YElem			  = -1;
	int ZElem			  = -1;
	int NXElem			  = -1;
	int NYElem			  = -1;
	int NZElem			  = -1;
	int UElem			  = -1;
	int VElem			  = -1;
	int VertexPropCount   = 0;
	int IndElem			  = -1;
	bool SwitchEndianness = false;

	inline bool hasVertices() const { return XElem >= 0 && YElem >= 0 && ZElem >= 0; }
	inline bool hasNormals() const { return NXElem >= 0 && NYElem >= 0 && NZElem >= 0; }
	inline bool hasUVs() const { return UElem >= 0 && VElem >= 0; }
	inline bool hasIndices() const { return IndElem >= 0; }
};

static mesh::TriMesh read(std::istream& stream, const Header& header, bool ascii)
{
	const auto readFloat = [&]() {
		float val;
		stream.read(reinterpret_cast<char*>(&val), sizeof(val));
		if (header.SwitchEndianness)
			val = swap_endian<float>(val);
		return val;
	};
	
	const auto readIdx = [&]() {
		uint32_t val;
		stream.read(reinterpret_cast<char*>(&val), sizeof(val));
		if (header.SwitchEndianness)
			val = swap_endian<uint32_t>(val);
		return val;
	};

	mesh::TriMesh trimesh;
	trimesh.vertices.reserve(header.VertexCount);
	if (header.hasNormals())
		trimesh.normals.reserve(header.VertexCount);
	if (header.hasUVs())
		trimesh.texcoords.reserve(header.VertexCount);

	for (int i = 0; i < header.VertexCount; ++i) {

		float x = 0, y = 0, z = 0;
		float nx = 0, ny = 0, nz = 0;
		float u = 0, v = 0;

		if (ascii) {
			std::string line;
			if (!std::getline(stream, line)) {
				error("Not enough vertices given");
				return mesh::TriMesh(); // Failed
			}

			std::stringstream sstream(line);
			int elem = 0;
			while (sstream) {
				float val;
				sstream >> val;

				if (header.XElem == elem)
					x = val;
				else if (header.YElem == elem)
					y = val;
				else if (header.ZElem == elem)
					z = val;
				else if (header.NXElem == elem)
					nx = val;
				else if (header.NYElem == elem)
					ny = val;
				else if (header.NZElem == elem)
					nz = val;
				else if (header.UElem == elem)
					u = val;
				else if (header.VElem == elem)
					v = val;
				++elem;
			}
		} else {
			for (int elem = 0; elem < header.VertexPropCount; ++elem) {
				float val = readFloat();
				if (header.XElem == elem)
					x = val;
				else if (header.YElem == elem)
					y = val;
				else if (header.ZElem == elem)
					z = val;
				else if (header.NXElem == elem)
					nx = val;
				else if (header.NYElem == elem)
					ny = val;
				else if (header.NZElem == elem)
					nz = val;
				else if (header.UElem == elem)
					u = val;
				else if (header.VElem == elem)
					v = val;
			}
		}

		trimesh.vertices.emplace_back(x,y,z);

		if (header.hasNormals()) {
			float norm = sqrt(nx * nx + ny * ny + nz * nz);
			if (norm == 0.0f)
				norm = 1.0f;
			trimesh.normals.emplace_back(nx / norm, ny / norm, nz / norm);
		}

		if (header.hasUVs()) {
			trimesh.texcoords.emplace_back(u, v);
		}
	}

	if(trimesh.vertices.empty())  {
		error("No vertices found in ply file");
		return mesh::TriMesh(); // Failed
	}

	trimesh.indices.reserve(header.FaceCount * 4);
	
	if (ascii) {
		for (int i = 0; i < header.FaceCount; ++i) {
			std::string line;
			if (!std::getline(stream, line)) {
				error("Not enough indices given");
				return mesh::TriMesh(); // Failed
			}

			std::stringstream sstream(line);

			uint32_t elems;
			sstream >> elems;

			uint32_t i0, i1, i2;
			sstream >> i0 >> i1 >> i2;

			trimesh.indices.insert(trimesh.indices.end(), {i0, i1, i2, 0});

			if(elems == 4) {
				uint32_t i3;
				sstream >> i3;
				trimesh.indices.insert(trimesh.indices.end(), {i0, i2, i3, 0});
			}

			if (elems != 3 && elems != 4) {
				error("Only triangle or quads allowed in ply files");
				return mesh::TriMesh();
			}
		}
	} else {
		for (int i = 0; i < header.FaceCount; ++i) {
			uint8_t elems;
			stream.read(reinterpret_cast<char*>(&elems), sizeof(elems));

			uint32_t i0, i1, i2;
			i0 = readIdx();
			i1 = readIdx();
			i2 = readIdx();
			trimesh.indices.insert(trimesh.indices.end(), {i0, i1, i2, 0});

			if(elems == 4) {
				uint32_t i3 = readIdx();
				trimesh.indices.insert(trimesh.indices.end(), {i0, i2, i3, 0});
			}

			if (elems != 3 && elems != 4) {
				error("Only triangle or quads allowed in ply files");
				return mesh::TriMesh();
			}
		}
	}

	return trimesh;
}

static inline bool isAllowedVertIndType(const std::string& str)
{
	return str == "uchar"
		   || str == "int"
		   || str == "uint8_t"
		   || str == "uint";
}

mesh::TriMesh load_mesh(const std::string& file)
{
	std::fstream stream(file, std::ios::in | std::ios::binary);
	if (!stream) {
		error("Given file '", file, "' can not be opened.");
		return mesh::TriMesh();
	}

	// Header
	std::string magic;
	stream >> magic;
	if (magic != "ply") {
		error("Given file '", file, "' is not a ply file.");
		return mesh::TriMesh();
	}

	std::string method;
	Header header;

	int facePropCounter = 0;
	for (std::string line; std::getline(stream, line);) {
		std::stringstream sstream(line);

		std::string action;
		sstream >> action;
		if (action == "comment")
			continue;
		else if (action == "format") {
			sstream >> method;
		} else if (action == "element") {
			std::string type;
			sstream >> type;
			if (type == "vertex")
				sstream >> header.VertexCount;
			else if (type == "face")
				sstream >> header.FaceCount;
		} else if (action == "property") {
			std::string type;
			sstream >> type;
			if (type == "float") {
				std::string name;
				sstream >> name;
				if (name == "x")
					header.XElem = header.VertexPropCount;
				else if (name == "y")
					header.YElem = header.VertexPropCount;
				else if (name == "z")
					header.ZElem = header.VertexPropCount;
				else if (name == "nx")
					header.NXElem = header.VertexPropCount;
				else if (name == "ny")
					header.NYElem = header.VertexPropCount;
				else if (name == "nz")
					header.NZElem = header.VertexPropCount;
				else if (name == "u")
					header.UElem = header.VertexPropCount;
				else if (name == "v")
					header.VElem = header.VertexPropCount;
				++header.VertexPropCount;
			} else if (type == "list") {
				++facePropCounter;

				std::string countType;
				sstream >> countType;

				std::string indType;
				sstream >> indType;

				std::string name;
				sstream >> name;
				if (!isAllowedVertIndType(countType)) {
					warn("Only 'property list uchar int' is supported");
					continue;
				}

				if (name == "vertex_indices")
					header.IndElem = facePropCounter - 1;
			} else {
				warn("Only float or list properties allowed. Ignoring...");
				++header.VertexPropCount;
			}
		} else if (action == "end_header")
			break;
	}

	// Content
	if (!header.hasVertices() || !header.hasIndices() || header.VertexCount <= 0 || header.FaceCount <= 0) {
		warn("Ply file '", file, "' does not contain valid mesh data");
		return mesh::TriMesh();
	}

	header.SwitchEndianness = (method == "binary_big_endian");
	mesh::TriMesh trimesh = read(stream, header, (method == "ascii"));
	if(trimesh.vertices.empty())
		return trimesh;

	trimesh.face_normals.resize(trimesh.indices.size() / 4);
	trimesh.face_area.resize(trimesh.indices.size() / 4);
	mesh::compute_face_normals(trimesh.indices, trimesh.vertices, trimesh.face_normals, trimesh.face_area, 0);

	if(trimesh.normals.empty()) {
		warn("No normals are present, computing smooth approximation.");
		trimesh.normals.resize(trimesh.vertices.size());
		mesh::compute_vertex_normals(trimesh.indices, trimesh.face_normals, trimesh.normals, 0);
	} else {
		mesh::fix_normals(trimesh);
	}

	if(trimesh.texcoords.empty()) {
		trimesh.texcoords.resize(trimesh.vertices.size());
		warn("No texture coordinates are present, using default value.");
		std::fill(trimesh.texcoords.begin(), trimesh.texcoords.end(), float2(0.0f));
	}

	return trimesh;
}
} // namespace PR
