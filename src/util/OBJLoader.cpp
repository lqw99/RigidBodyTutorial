#include "util/OBJLoader.h"
#include "tiny_obj_loader.h"
#include <cassert>
#include <cstddef>
#include <iostream>
#include <string>
#include <vector>

namespace {
// 2D/3D point data structures
struct Point3D {
  Point3D() : x(0), y(0), z(0) {}

  float x, y, z;
};

struct TriangleInds {
  TriangleInds() : i(0), j(0), k(0) {}

  int i, j, k;
};

// Extract path from a string
std::string extractPath(const std::string &filepathname) {
  std::size_t pos = filepathname.find_last_of("/\\");

  if (pos == std::string::npos)
    return std::string(".");

  return filepathname.substr(0, pos);
}
} // namespace

bool OBJLoader::load_obj(const std::string &filename, Eigen::MatrixXf &meshV,
                         Eigen::MatrixXi &meshF) {
  tinyobj::ObjReaderConfig reader_config;
  reader_config.triangulate = true;
  tinyobj::ObjReader reader;

  if (!reader.ParseFromFile(filename, reader_config)) {
    if (!reader.Error().empty()) {
      std::cerr << "TinyObjReader: " << reader.Error();
    }
    return false;
  }
  auto &attrib = reader.GetAttrib();
  auto &shapes = reader.GetShapes();
  int num_vert = attrib.vertices.size() / 3;
  meshV.resize(num_vert, 3);
  for (unsigned int i = 0; i < num_vert; ++i) {
    for (int j = 0; j < 3; j++) {
      meshV(i, j) = attrib.vertices[3 * i + j];
    }
  }

  std::vector<Eigen::MatrixXi> meshFs(shapes.size());
  for (size_t s = 0; s < shapes.size(); s++) {
    int num_face_s = shapes[s].mesh.num_face_vertices.size();
    auto &face_elems = shapes[s].mesh.indices;
    meshFs[s].resize(num_face_s, 3);
    for (size_t f = 0; f < num_face_s; f++) {
      for (int i = 0; i < 3; i++) {
        meshFs[s](f, i) = face_elems[3 * f + i].vertex_index;
      }
    }
  }

  Eigen::Index total_rows = 0;
  for (const auto &F : meshFs) {
    total_rows += F.rows();
  }

  meshF.resize(total_rows, 3);

  Eigen::Index offset = 0;
  for (const auto &F : meshFs) {
    meshF.middleRows(offset, F.rows()) = F;
    offset += F.rows();
  }
  return true;

  // // Loop over shapes (all shapes belong to a body)
  // for (size_t s = 0; s < shapes.size(); s++) {
  //   auto &face_elems = shapes[s].mesh.indices;
  //   // Loop over faces(polygon)
  //   size_t index_offset = 0;

  //   for (size_t f = 0; f < shapes[s].mesh.num_face_vertices.size(); f++) {
  //     size_t fv = size_t(shapes[s].mesh.num_face_vertices[f]);

  //     // Loop over vertices in the face.
  //     for (size_t v = 0; v < fv; v++) {
  //       // access to vertex
  //       tinyobj::index_t idx = shapes[s].mesh.indices[index_offset + v];
  //       tinyobj::real_t vx = attrib.vertices[3 * size_t(idx.vertex_index) +
  //       0]; tinyobj::real_t vy = attrib.vertices[3 * size_t(idx.vertex_index)
  //       + 1]; tinyobj::real_t vz = attrib.vertices[3 *
  //       size_t(idx.vertex_index) + 2];

  //       // Check if `normal_index` is zero or positive. negative = no normal
  //       // data
  //       if (idx.normal_index >= 0) {
  //         tinyobj::real_t nx = attrib.normals[3 * size_t(idx.normal_index) +
  //         0]; tinyobj::real_t ny = attrib.normals[3 *
  //         size_t(idx.normal_index) + 1]; tinyobj::real_t nz =
  //         attrib.normals[3 * size_t(idx.normal_index) + 2];
  //       }

  //       // Check if `texcoord_index` is zero or positive. negative = no
  //       // texcoord data
  //       if (idx.texcoord_index >= 0) {
  //         tinyobj::real_t tx =
  //             attrib.texcoords[2 * size_t(idx.texcoord_index) + 0];
  //         tinyobj::real_t ty =
  //             attrib.texcoords[2 * size_t(idx.texcoord_index) + 1];
  //       }

  //       // Optional: vertex colors
  //       // tinyobj::real_t red   =
  //       // attrib.colors[3*size_t(idx.vertex_index)+0]; tinyobj::real_t green
  //       // = attrib.colors[3*size_t(idx.vertex_index)+1]; tinyobj::real_t
  //       blue
  //       // = attrib.colors[3*size_t(idx.vertex_index)+2];
  //     }
  //     index_offset += fv;
  //   }
  // }
}
