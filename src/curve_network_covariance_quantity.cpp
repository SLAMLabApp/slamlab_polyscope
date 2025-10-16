// Covariance ellipsoid visualization for curve networks
// Implementation

#include "polyscope/curve_network_covariance_quantity.h"
#include "polyscope/curve_network.h"
#include "polyscope/point_cloud.h"
#include "polyscope/polyscope.h"
#include "polyscope/simple_triangle_mesh.h"

#include <cmath>
#include <iostream>

using std::cout;
using std::endl;

namespace polyscope {

CurveNetworkNodeCovarianceQuantity::CurveNetworkNodeCovarianceQuantity(
    std::string name, CurveNetwork& network_, const std::vector<glm::mat3>& positionCovariances_,
    const std::vector<glm::mat3>& rotationCovariances_)
    : CurveNetworkQuantity(name, network_, false), positionCovariances(positionCovariances_),
      rotationCovariances(rotationCovariances_),
      showPositionEllipsoids(uniquePrefix() + "#showPositionEllipsoids", true),
      showRotationEllipsoids(uniquePrefix() + "#showRotationEllipsoids", true),
      showOnlyLast(uniquePrefix() + "#showOnlyLast", false), ellipsoidScale(uniquePrefix() + "#ellipsoidScale", 1.0f),
      positionEllipsoidColor(uniquePrefix() + "#positionEllipsoidColor", glm::vec3{0.6f, 0.2f, 0.8f}), // Purple
      rotationEllipsoidColor(uniquePrefix() + "#rotationEllipsoidColor", glm::vec3{1.0f, 0.9f, 0.2f}), // Yellow
      transparency(uniquePrefix() + "#transparency", 0.5f) {

  // Create sphere geometry template
  createSphereGeometry();
}

std::string CurveNetworkNodeCovarianceQuantity::niceName() { return name + " (covariance)"; }

void CurveNetworkNodeCovarianceQuantity::draw() {
  if (!isEnabled()) return;

  drawEllipsoids();
}

void CurveNetworkNodeCovarianceQuantity::buildCustomUI() {
  ImGui::SameLine();

  // == Options popup
  if (ImGui::Button("Options")) {
    ImGui::OpenPopup("OptionsPopup");
  }
  if (ImGui::BeginPopup("OptionsPopup")) {

    if (ImGui::Checkbox("Show Position Ellipsoids", &showPositionEllipsoids.get())) {
      setShowPositionEllipsoids(getShowPositionEllipsoids());
    }

    if (ImGui::Checkbox("Show Rotation Ellipsoids", &showRotationEllipsoids.get())) {
      setShowRotationEllipsoids(getShowRotationEllipsoids());
    }

    if (ImGui::Checkbox("Show Only Last", &showOnlyLast.get())) {
      setShowOnlyLast(getShowOnlyLast());
    }

    if (ImGui::SliderFloat("Ellipsoid Scale", &ellipsoidScale.get(), 0.1f, 10.0f)) {
      setEllipsoidScale(getEllipsoidScale());
    }

    if (ImGui::ColorEdit3("Position Color", &positionEllipsoidColor.get()[0])) {
      setPositionEllipsoidColor(getPositionEllipsoidColor());
    }

    if (ImGui::ColorEdit3("Rotation Color", &rotationEllipsoidColor.get()[0])) {
      setRotationEllipsoidColor(getRotationEllipsoidColor());
    }

    if (ImGui::SliderFloat("Transparency", &transparency.get(), 0.0f, 1.0f)) {
      setTransparency(getTransparency());
    }

    ImGui::EndPopup();
  }
}

void CurveNetworkNodeCovarianceQuantity::refresh() {
  ellipsoidProgram.reset();
  prepareEllipsoidGeometry();
}

void CurveNetworkNodeCovarianceQuantity::buildNodeInfoGUI(size_t vInd) {
  ImGui::TextUnformatted(name.c_str());
  ImGui::NextColumn();

  if (vInd < positionCovariances.size()) {
    ImGui::Text("Position Cov:");
    const glm::mat3& cov = positionCovariances[vInd];
    ImGui::Text("  [%.3f %.3f %.3f]", cov[0][0], cov[0][1], cov[0][2]);
    ImGui::Text("  [%.3f %.3f %.3f]", cov[1][0], cov[1][1], cov[1][2]);
    ImGui::Text("  [%.3f %.3f %.3f]", cov[2][0], cov[2][1], cov[2][2]);
  }

  if (vInd < rotationCovariances.size()) {
    ImGui::Text("Rotation Cov:");
    const glm::mat3& cov = rotationCovariances[vInd];
    ImGui::Text("  [%.3f %.3f %.3f]", cov[0][0], cov[0][1], cov[0][2]);
    ImGui::Text("  [%.3f %.3f %.3f]", cov[1][0], cov[1][1], cov[1][2]);
    ImGui::Text("  [%.3f %.3f %.3f]", cov[2][0], cov[2][1], cov[2][2]);
  }

  ImGui::NextColumn();
}

// === Setters and getters

CurveNetworkNodeCovarianceQuantity* CurveNetworkNodeCovarianceQuantity::setShowPositionEllipsoids(bool val) {
  showPositionEllipsoids = val;
  requestRedraw();
  return this;
}
bool CurveNetworkNodeCovarianceQuantity::getShowPositionEllipsoids() { return showPositionEllipsoids.get(); }

CurveNetworkNodeCovarianceQuantity* CurveNetworkNodeCovarianceQuantity::setShowRotationEllipsoids(bool val) {
  showRotationEllipsoids = val;
  requestRedraw();
  return this;
}
bool CurveNetworkNodeCovarianceQuantity::getShowRotationEllipsoids() { return showRotationEllipsoids.get(); }

CurveNetworkNodeCovarianceQuantity* CurveNetworkNodeCovarianceQuantity::setShowOnlyLast(bool val) {
  showOnlyLast = val;
  requestRedraw();
  return this;
}
bool CurveNetworkNodeCovarianceQuantity::getShowOnlyLast() { return showOnlyLast.get(); }

CurveNetworkNodeCovarianceQuantity* CurveNetworkNodeCovarianceQuantity::setEllipsoidScale(float val) {
  ellipsoidScale = val;
  requestRedraw();
  return this;
}
float CurveNetworkNodeCovarianceQuantity::getEllipsoidScale() { return ellipsoidScale.get(); }

CurveNetworkNodeCovarianceQuantity* CurveNetworkNodeCovarianceQuantity::setPositionEllipsoidColor(glm::vec3 val) {
  positionEllipsoidColor = val;
  requestRedraw();
  return this;
}
glm::vec3 CurveNetworkNodeCovarianceQuantity::getPositionEllipsoidColor() { return positionEllipsoidColor.get(); }

CurveNetworkNodeCovarianceQuantity* CurveNetworkNodeCovarianceQuantity::setRotationEllipsoidColor(glm::vec3 val) {
  rotationEllipsoidColor = val;
  requestRedraw();
  return this;
}
glm::vec3 CurveNetworkNodeCovarianceQuantity::getRotationEllipsoidColor() { return rotationEllipsoidColor.get(); }

CurveNetworkNodeCovarianceQuantity* CurveNetworkNodeCovarianceQuantity::setTransparency(float val) {
  transparency = glm::clamp(val, 0.0f, 1.0f);
  requestRedraw();
  return this;
}
float CurveNetworkNodeCovarianceQuantity::getTransparency() { return transparency.get(); }

// === Helper functions

void CurveNetworkNodeCovarianceQuantity::createSphereGeometry(int subdivisions) {
  // Create an icosphere (subdivided icosahedron) for smooth ellipsoids
  // This is a simple implementation - can be optimized

  // Clear existing data
  sphereVertices.clear();
  sphereNormals.clear();
  sphereIndices.clear();

  // Golden ratio
  const float t = (1.0f + std::sqrt(5.0f)) / 2.0f;
  const float len = std::sqrt(1.0f + t * t);

  // Create icosahedron vertices (normalized)
  std::vector<glm::vec3> baseVertices = {
      glm::vec3(-1, t, 0) / len, glm::vec3(1, t, 0) / len, glm::vec3(-1, -t, 0) / len, glm::vec3(1, -t, 0) / len,
      glm::vec3(0, -1, t) / len, glm::vec3(0, 1, t) / len, glm::vec3(0, -1, -t) / len, glm::vec3(0, 1, -t) / len,
      glm::vec3(t, 0, -1) / len, glm::vec3(t, 0, 1) / len, glm::vec3(-t, 0, -1) / len, glm::vec3(-t, 0, 1) / len};

  // Icosahedron faces
  std::vector<glm::uvec3> baseFaces = {{0, 11, 5}, {0, 5, 1},  {0, 1, 7},   {0, 7, 10}, {0, 10, 11},
                                       {1, 5, 9},  {5, 11, 4}, {11, 10, 2}, {10, 7, 6}, {7, 1, 8},
                                       {3, 9, 4},  {3, 4, 2},  {3, 2, 6},   {3, 6, 8},  {3, 8, 9},
                                       {4, 9, 5},  {2, 4, 11}, {6, 2, 10},  {8, 6, 7},  {9, 8, 1}};

  // Subdivide for smoother sphere (default is 2: 320 faces)
  std::vector<glm::vec3> currentVertices = baseVertices;
  std::vector<glm::uvec3> currentFaces = baseFaces;

  for (int sub = 0; sub < subdivisions; sub++) {
    std::vector<glm::uvec3> newFaces;
    std::map<std::pair<uint32_t, uint32_t>, uint32_t> midpointCache;

    auto getMidpoint = [&](uint32_t i1, uint32_t i2) -> uint32_t {
      auto key = std::make_pair(std::min(i1, i2), std::max(i1, i2));
      auto it = midpointCache.find(key);
      if (it != midpointCache.end()) {
        return it->second;
      }

      glm::vec3 mid = glm::normalize(currentVertices[i1] + currentVertices[i2]);
      uint32_t newIdx = currentVertices.size();
      currentVertices.push_back(mid);
      midpointCache[key] = newIdx;
      return newIdx;
    };

    for (const auto& face : currentFaces) {
      uint32_t a = getMidpoint(face.x, face.y);
      uint32_t b = getMidpoint(face.y, face.z);
      uint32_t c = getMidpoint(face.z, face.x);

      newFaces.push_back({face.x, a, c});
      newFaces.push_back({face.y, b, a});
      newFaces.push_back({face.z, c, b});
      newFaces.push_back({a, b, c});
    }

    currentFaces = newFaces;
  }

  // Store final subdivided sphere
  for (const auto& v : currentVertices) {
    sphereVertices.push_back(v);
    sphereNormals.push_back(glm::normalize(v)); // For unit sphere, normal = position
  }

  for (const auto& f : currentFaces) {
    sphereIndices.push_back(f.x);
    sphereIndices.push_back(f.y);
    sphereIndices.push_back(f.z);
    sphereFaces.push_back(f); // Also store as triangles for mesh rendering
  }
}

CurveNetworkNodeCovarianceQuantity::EllipsoidParams
CurveNetworkNodeCovarianceQuantity::computeEllipsoidParams(const glm::mat3& covariance, float scale) const {
  EllipsoidParams params;

  // For visualization purposes, we'll use a simplified eigenvalue decomposition
  // In practice, you'd use a proper SVD or eigen decomposition library

  // For now, just use the diagonal elements as axes and identity rotation
  // This is a placeholder - proper implementation needs eigendecomposition
  params.axes = glm::vec3(std::sqrt(std::max(covariance[0][0], 0.001f)), std::sqrt(std::max(covariance[1][1], 0.001f)),
                          std::sqrt(std::max(covariance[2][2], 0.001f))) *
                scale;

  params.rotation = glm::mat3(1.0f); // Identity for now

  return params;
}

void CurveNetworkNodeCovarianceQuantity::prepareEllipsoidGeometry() {
  // Ensure sphere geometry exists
  if (sphereVertices.empty()) {
    createSphereGeometry();
  }

  // Note: We'll use Polyscope's SimpleTriangleMesh for rendering ellipsoids
  // Each ellipsoid will be registered as a separate mesh with appropriate transforms
}

void CurveNetworkNodeCovarianceQuantity::drawEllipsoids() {
  // Get the curve network parent structure
  CurveNetwork& network = dynamic_cast<CurveNetwork&>(parent);

  // Determine which nodes to draw
  size_t startIdx = 0;
  size_t endIdx = network.nNodes();

  if (showOnlyLast.get() && endIdx > 0) {
    startIdx = endIdx - 1;
  }

  // For each node, draw position and/or rotation ellipsoids as meshes
  float scale = ellipsoidScale.get();
  float alpha = transparency.get();

  // Render each ellipsoid as a transformed sphere mesh
  for (size_t i = 0; i < network.nNodes(); i++) {
    glm::vec3 nodePos = network.nodePositionsData[i];
    bool shouldDrawThisNode = (i >= startIdx); // Respects "show only last" filter

    // Handle position ellipsoid
    if (i < positionCovariances.size()) {
      std::string meshName = parent.name + "_" + name + "_pos_" + std::to_string(i);

      if (showPositionEllipsoids.get() && shouldDrawThisNode) {
        EllipsoidParams params = computeEllipsoidParams(positionCovariances[i], scale);

        // Transform sphere vertices to create ellipsoid
        std::vector<glm::vec3> ellipsoidVerts;
        ellipsoidVerts.reserve(sphereVertices.size());

        for (const auto& v : sphereVertices) {
          // Scale by eigenvalues (axes lengths) and rotate by eigenvectors
          glm::vec3 scaled = glm::vec3(v.x * params.axes.x, v.y * params.axes.y, v.z * params.axes.z);
          glm::vec3 rotated = params.rotation * scaled;
          ellipsoidVerts.push_back(nodePos + rotated);
        }

        // Register/update mesh
        auto* mesh = polyscope::registerSimpleTriangleMesh(meshName, ellipsoidVerts, sphereFaces);
        mesh->setEnabled(true);
        mesh->setSurfaceColor(positionEllipsoidColor.get());
        mesh->setTransparency(alpha);
        mesh->setMaterial("wax"); // Smooth material that shows curvature well
      } else {
        // Disable the mesh if it exists but shouldn't be shown
        auto* mesh = polyscope::getSimpleTriangleMesh(meshName);
        if (mesh != nullptr) {
          mesh->setEnabled(false);
        }
      }
    }

    // Handle rotation ellipsoids (flattened ellipsoids/discs)
    if (i < rotationCovariances.size()) {
      // Get position ellipsoid size to offset rotation ellipsoids outside
      float maxPosRadius = 0.0f;
      if (i < positionCovariances.size()) {
        EllipsoidParams posParams = computeEllipsoidParams(positionCovariances[i], scale);
        maxPosRadius = std::max({posParams.axes.x, posParams.axes.y, posParams.axes.z});
      }

      for (int axis = 0; axis < 3; axis++) {
        std::string discName = parent.name + "_" + name + "_rot_" + std::to_string(i) + "_" + std::to_string(axis);

        if (showRotationEllipsoids.get() && shouldDrawThisNode) {
          EllipsoidParams params = computeEllipsoidParams(rotationCovariances[i], scale);

          // Compute offset direction (perpendicular to the flattened axis)
          glm::vec3 offsetDir(0.0f);
          offsetDir[axis] = 1.0f;                                                 // Direction along the flattened axis
          glm::vec3 offset = params.rotation * offsetDir * (maxPosRadius * 1.3f); // 30% outside position ellipsoid

          // Create flattened ellipsoids for rotation uncertainty
          std::vector<glm::vec3> discVerts;
          discVerts.reserve(sphereVertices.size());

          for (const auto& v : sphereVertices) {
            // Flatten along one axis to create a disc
            glm::vec3 scaled = v;
            scaled[axis] *= 0.1f; // Flatten
            scaled = glm::vec3(scaled.x * params.axes.x, scaled.y * params.axes.y, scaled.z * params.axes.z);
            glm::vec3 rotated = params.rotation * scaled;
            discVerts.push_back(nodePos + offset + rotated); // Add offset here
          }

          auto* disc = polyscope::registerSimpleTriangleMesh(discName, discVerts, sphereFaces);
          disc->setEnabled(true);
          disc->setSurfaceColor(rotationEllipsoidColor.get());
          disc->setTransparency(alpha);
          disc->setMaterial("wax");
        } else {
          // Disable the disc if it exists but shouldn't be shown
          auto* disc = polyscope::getSimpleTriangleMesh(discName);
          if (disc != nullptr) {
            disc->setEnabled(false);
          }
        }
      }
    }
  }
}

} // namespace polyscope
