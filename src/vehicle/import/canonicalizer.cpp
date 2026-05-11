// Copyright (C) 2026 Abdul, Hashim.
//
// This file is part of CARLA Studio.
// Licensed under the GNU Affero General Public License v3 or later -
// see <LICENSE> at the project root or
// <https://www.gnu.org/licenses/agpl-3.0.html> for the full text.
// SPDX-License-Identifier: AGPL-3.0-or-later

#include "vehicle/import/canonicalizer.h"

#include <QFileInfo>

#ifdef CARLA_STUDIO_WITH_ASSIMP
#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>
#endif

namespace carla_studio::vehicle_import {

bool assimp_available() {
#ifdef CARLA_STUDIO_WITH_ASSIMP
  return true;
#else
  return false;
#endif
}

MeshGeometry load_mesh_geometry_any(const QString &path) {
  const QString ext = QFileInfo(path).suffix().toLower();
  if (ext == "obj") return load_mesh_geometry_obj(path);

#ifdef CARLA_STUDIO_WITH_ASSIMP
  MeshGeometry g;
  Assimp::Importer imp;
  const aiScene *scene = imp.ReadFile(
      path.toStdString().c_str(),
      aiProcess_Triangulate | aiProcess_JoinIdenticalVertices |
      aiProcess_PreTransformVertices);
  if (!scene || !scene->HasMeshes()) return g;
  int vBase = 0;
  for (unsigned mi = 0; mi < scene->m_num_meshes; ++mi) {
    const aiMesh *m = scene->m_meshes[mi];
    for (unsigned vi = 0; vi < m->m_num_vertices; ++vi) {
      g.verts.push_back(static_cast<float>(m->m_vertices[vi].x));
      g.verts.push_back(static_cast<float>(m->m_vertices[vi].y));
      g.verts.push_back(static_cast<float>(m->m_vertices[vi].z));
    }
    for (unsigned fi = 0; fi < m->m_num_faces; ++fi) {
      const aiFace &f = m->m_faces[fi];
      if (f.m_num_indices != 3) { ++g.malformed_faces; continue; }
      g.faces.push_back(vBase + static_cast<int>(f.m_indices[0]));
      g.faces.push_back(vBase + static_cast<int>(f.m_indices[1]));
      g.faces.push_back(vBase + static_cast<int>(f.m_indices[2]));
    }
    vBase += static_cast<int>(m->m_num_vertices);
  }
  g.valid = !g.verts.empty() && !g.faces.empty();
  return g;
#else
  return MeshGeometry{};
#endif
}

CanonicalizeResult canonicalize_mesh(const QString & ,
                                    const QString &output_path) {
  CanonicalizeResult r;
  r.output_path = output_path;
  r.detail = QStringLiteral(
      "Canonicalizer is staged but not yet emitting a UE-canonical FBX. "
      "Today, Lite-mode kits include the source mesh untouched and rely on the "
      "auto-generated README to walk users through the canonical setup. "
      "Full FBX rewrite (axis remap, scale-to-cm, origin recentre, smoothing "
      "groups, canonical bone names, ASCII->binary) lands once Assimp is built "
      "in (-DCARLA_STUDIO_FETCH_ASSIMP=ON) and the export side of Slice 3 "
      "is implemented.");
  return r;
}

}
