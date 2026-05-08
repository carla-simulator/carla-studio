// Copyright (C) 2026 Abdul, Hashim.
//
// This file is part of CARLA Studio.
// Licensed under the GNU Affero General Public License v3 or later —
// see <LICENSE> at the project root or
// <https://www.gnu.org/licenses/agpl-3.0.html> for the full text.
// SPDX-License-Identifier: AGPL-3.0-or-later

#include "vehicle_import/Canonicalizer.h"

#include <QFileInfo>

#ifdef CARLA_STUDIO_WITH_ASSIMP
#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>
#endif

namespace carla_studio::vehicle_import {

bool assimpAvailable() {
#ifdef CARLA_STUDIO_WITH_ASSIMP
  return true;
#else
  return false;
#endif
}

MeshGeometry loadMeshGeometryAny(const QString &path) {
  const QString ext = QFileInfo(path).suffix().toLower();
  if (ext == "obj") return loadMeshGeometryOBJ(path);

#ifdef CARLA_STUDIO_WITH_ASSIMP
  MeshGeometry g;
  Assimp::Importer imp;
  const aiScene *scene = imp.ReadFile(
      path.toStdString().c_str(),
      aiProcess_Triangulate | aiProcess_JoinIdenticalVertices |
      aiProcess_PreTransformVertices);
  if (!scene || !scene->HasMeshes()) return g;
  int vBase = 0;
  for (unsigned mi = 0; mi < scene->mNumMeshes; ++mi) {
    const aiMesh *m = scene->mMeshes[mi];
    for (unsigned vi = 0; vi < m->mNumVertices; ++vi) {
      g.verts.push_back(static_cast<float>(m->mVertices[vi].x));
      g.verts.push_back(static_cast<float>(m->mVertices[vi].y));
      g.verts.push_back(static_cast<float>(m->mVertices[vi].z));
    }
    for (unsigned fi = 0; fi < m->mNumFaces; ++fi) {
      const aiFace &f = m->mFaces[fi];
      if (f.mNumIndices != 3) { ++g.malformedFaces; continue; }
      g.faces.push_back(vBase + static_cast<int>(f.mIndices[0]));
      g.faces.push_back(vBase + static_cast<int>(f.mIndices[1]));
      g.faces.push_back(vBase + static_cast<int>(f.mIndices[2]));
    }
    vBase += static_cast<int>(m->mNumVertices);
  }
  g.valid = !g.verts.empty() && !g.faces.empty();
  return g;
#else
  return MeshGeometry{};
#endif
}

CanonicalizeResult canonicalizeMesh(const QString & /*inputPath*/,
                                    const QString &outputPath) {
  CanonicalizeResult r;
  r.outputPath = outputPath;
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

}  // namespace carla_studio::vehicle_import
