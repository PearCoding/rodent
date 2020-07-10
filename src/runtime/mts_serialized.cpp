#include "mts_serialized.h"

#include <climits>
#include <fstream>
#include <sstream>

#include <zlib.h>

namespace mts {
	constexpr size_t BUFFER_SIZE = 32768;
	class CompressedStream {
	public:
		inline CompressedStream(std::istream& in, size_t size) : mIn(in), mSize(size), mPos(0) {
			mStream.zalloc   = Z_NULL;
			mStream.zfree    = Z_NULL;
			mStream.opaque   = Z_NULL;
			mStream.avail_in = 0;
			mStream.next_in  = Z_NULL;

			int retval = inflateInit2(&mStream, 15);
			if (retval != Z_OK)
				error("Could not initialize ZLIB: ", retval);
		}

		template<typename T>
		inline void read(T *ptr){
			size_t size = sizeof(T);
			uint8_t *targetPtr = (uint8_t *) ptr;
			while (size > 0) {
				if (mStream.avail_in == 0) {
					size_t remaining = mSize - mPos;
					mStream.next_in = mBuffer;
					mStream.avail_in = (uInt) std::min(remaining, sizeof(mBuffer));
					if (mStream.avail_in == 0)
						error("Read less data than expected (", size, " more bytes required)");
					mIn.read(reinterpret_cast<char*>(mBuffer), mStream.avail_in);

					if(!mIn.good())
						error("Could not read ", mStream.avail_in, " bytes");

					mPos += mStream.avail_in;
				}

				mStream.avail_out = (uInt) size;
				mStream.next_out = targetPtr;

				int retval = inflate(&mStream, Z_NO_FLUSH);
				switch (retval) {
					case Z_STREAM_ERROR:
						error("inflate(): stream error!");
					case Z_NEED_DICT:
						error("inflate(): need dictionary!");
					case Z_DATA_ERROR:
						error("inflate(): data error!");
					case Z_MEM_ERROR:
						error("inflate(): memory error!");
				};

				size_t outputSize = size - (size_t) mStream.avail_out;
				targetPtr += outputSize;
				size -= outputSize;

				if (size > 0 && retval == Z_STREAM_END)
					error("inflate(): attempting to read past the end of the stream!");
			}
		}

	private:
		std::istream& mIn;
		size_t mSize;
		size_t mPos;
		z_stream mStream;
		uint8_t mBuffer[BUFFER_SIZE];
	};

	enum MeshFlags {
		MF_VERTEXNORMALS = 0x0001,
		MF_TEXCOORDS 	 = 0x0002,
		MF_VERTEXCOLORS  = 0x0008,
		MF_FACENORMALS   = 0x0010,
		MF_FLOAT         = 0x1000,
		MF_DOUBLE        = 0x2000,
	};

	template<typename T>
	void extractMeshVertices(mesh::TriMesh& trimesh, CompressedStream& cin, uint32_t flags) {
		// Vertex Positions
		for(size_t i =0; i < trimesh.vertices.size(); ++i) {
			T x, y, z;
			cin.read(&x);
			cin.read(&y);
			cin.read(&z);
			trimesh.vertices[i] = float3(x,y,z);
		}

		// Normals
		if(flags & MF_VERTEXNORMALS) {
			for(size_t i =0; i < trimesh.normals.size(); ++i) {
				T x, y, z;
				cin.read(&x);
				cin.read(&y);
				cin.read(&z);
				trimesh.normals[i] = float3(x,y,z);
			}
		}

		// UV
		if(flags & MF_TEXCOORDS) {
			for(size_t i =0; i < trimesh.texcoords.size(); ++i) {
				T x, y;
				cin.read(&x);
				cin.read(&y);
				trimesh.texcoords[i] = float2(x,y);
			}
		}

		// Vertex Color (ignored)
		if(flags & MF_VERTEXCOLORS) {
			for(size_t i =0; i < trimesh.vertices.size()*3; ++i) {
				T _ignore;
				cin.read(&_ignore);
			}
		}
	}

	template<typename T>
	void extractMeshIndices(mesh::TriMesh& trimesh, CompressedStream& cin) {
		size_t tricount = trimesh.indices.size()/4;
		// Indices
		for(size_t i =0; i < tricount; ++i) {
			T x, y, z;
			cin.read(&x);
			cin.read(&y);
			cin.read(&z);
			trimesh.indices[i*4 + 0] = x;
			trimesh.indices[i*4 + 1] = y;
			trimesh.indices[i*4 + 2] = z;
			trimesh.indices[i*4 + 3] = 0;
		}
	}

    mesh::TriMesh load_mesh(const std::string& file, size_t shapeIndex) {
		std::fstream stream(file, std::ios::in | std::ios::binary);
		if (!stream) {
			error("Given file '", file, "' can not be opened.");
			return mesh::TriMesh();
		}

		// Check header
		uint16_t fileIdent;
		uint16_t fileVersion;
		stream.read(reinterpret_cast<char*>(&fileIdent), sizeof(fileIdent));

		if(fileIdent != 0x041C) {
			error("Given file '", file, "' is not a valid Mitsuba serialized file.");
			return mesh::TriMesh();
		}
		stream.read(reinterpret_cast<char*>(&fileVersion), sizeof(fileVersion));
		if(fileVersion < 3) {
			error("Given file '", file, "' has an insufficient version number ", fileVersion, " < 3.");
			return mesh::TriMesh();
		}

		// Extract amount of shapes inside the file
		uint32_t shapeCount;
		stream.seekg(-std::streamoff(sizeof(shapeCount)), std::ios::end);
		stream.read(reinterpret_cast<char*>(&shapeCount), sizeof(shapeCount));

		if(!stream.good()) {
			error("Given file '", file, "' can not access end of file dictionary.");
			return mesh::TriMesh();
		}

		if(shapeIndex >= shapeCount) {
			error("Given file '", file, "' can not access shape index ", shapeIndex, " as it only contains ", shapeCount, " shapes.");
			return mesh::TriMesh();
		}

		// Extract mesh file start position
		uint64_t shapeFileStart;
		uint64_t shapeFileEnd;

		if(fileVersion >= 4) {
			stream.seekg(-std::streamoff(sizeof(shapeCount) + sizeof(shapeFileStart)*(shapeCount - shapeIndex)), std::ios::end);
			stream.read(reinterpret_cast<char*>(&shapeFileStart), sizeof(shapeFileStart));

			if(!stream.good()) {
				error("Given file '", file, "' could not extract shape file offset.");
				return mesh::TriMesh();
			}

			// Extract mesh file end position
			if(shapeIndex == shapeCount - 1) {
				stream.seekg(-std::streamoff(sizeof(shapeCount)), std::ios::end);
				shapeFileEnd = stream.tellg();
			} else  {
				stream.seekg(-std::streamoff(sizeof(shapeCount) + sizeof(shapeFileEnd)*(shapeCount - shapeIndex + 1)), std::ios::end);
				stream.read(reinterpret_cast<char*>(&shapeFileEnd), sizeof(shapeFileEnd));
			}
		} else { /* Version 3 uses uint32_t instead of uint64_t */
			uint32_t _shapeFileStart;
			uint32_t _shapeFileEnd;

			stream.seekg(-std::streamoff(sizeof(shapeCount) + sizeof(_shapeFileStart)*(shapeCount - shapeIndex)), std::ios::end);
			stream.read(reinterpret_cast<char*>(&_shapeFileStart), sizeof(_shapeFileStart));

			if(!stream.good()) {
				error("Given file '", file, "' could not extract shape file offset.");
				return mesh::TriMesh();
			}

			// Extract mesh file end position
			if(shapeIndex == shapeCount - 1) {
				stream.seekg(-std::streamoff(sizeof(shapeCount)), std::ios::end);
				_shapeFileEnd = stream.tellg();
			} else  {
				stream.seekg(-std::streamoff(sizeof(shapeCount) + sizeof(_shapeFileEnd)*(shapeCount - shapeIndex + 1)), std::ios::end);
				stream.read(reinterpret_cast<char*>(&_shapeFileEnd), sizeof(_shapeFileEnd));
			}

			shapeFileStart = _shapeFileStart;
			shapeFileEnd = _shapeFileEnd;
		}

		if(!stream.good()) {
			error("Given file '", file, "' could not extract shape file offset.");
			return mesh::TriMesh();
		}

		const size_t maxContentSize = shapeFileEnd - shapeFileStart - sizeof(uint16_t)*2;

		// Go to start position of mesh
		stream.seekg(sizeof(uint16_t)*2/*Header*/ + shapeFileStart, std::ios::beg);

		// Inflate with zlib
		CompressedStream cin(stream, maxContentSize);

		uint32_t mesh_flags;
		cin.read(&mesh_flags);

		if(fileVersion >= 4) {
			uint8_t utf8Char;
			do {
				cin.read(&utf8Char);
			} while(utf8Char != 0); // Ignore shape name
		}

		uint64_t vertexCount;
		uint64_t triCount;
		cin.read(&vertexCount);
		cin.read(&triCount);

		if(vertexCount == 0 || triCount == 0) {
			error("Given file '", file, "' has no valid mesh.");
			return mesh::TriMesh();
		}

		mesh::TriMesh trimesh;
		trimesh.vertices.resize(vertexCount);
		trimesh.normals.resize(vertexCount);
		trimesh.texcoords.resize(vertexCount);
		trimesh.face_normals.resize(triCount);
		trimesh.face_area.resize(triCount);
		trimesh.indices.resize(triCount*4);

		if (mesh_flags & MF_DOUBLE)
			extractMeshVertices<double>(trimesh, cin, mesh_flags);
		else 
			extractMeshVertices<float>(trimesh, cin, mesh_flags);
		
		if(vertexCount > 0xFFFFFFFF)
			extractMeshIndices<uint64_t>(trimesh, cin);
		else
			extractMeshIndices<uint32_t>(trimesh, cin);

		mesh::compute_face_normals(trimesh.indices, trimesh.vertices, trimesh.face_normals, trimesh.face_area, 0);

		if(!(mesh_flags & MF_VERTEXNORMALS)) {
			warn("No normals are present, computing smooth approximation.");
			mesh::compute_vertex_normals(trimesh.indices, trimesh.face_normals, trimesh.normals, 0);
		} else
			mesh::fix_normals(trimesh);

		if(!(mesh_flags & MF_TEXCOORDS)) {
			warn("No texture coordinates are present, using default value.");
			std::fill(trimesh.texcoords.begin(), trimesh.texcoords.end(), float2(0.0f));
		}

		return trimesh;
	}
} // namespace PR