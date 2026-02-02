// Template implementation for covariance ellipsoid visualization base class

#pragma once

#include "polyscope/polyscope.h"
#include "polyscope/simple_triangle_mesh.h"

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <iostream>

namespace polyscope {

// === Constructor ===

template <typename DerivedQuantity, typename ParentStructure>
CovarianceQuantityBase<DerivedQuantity, ParentStructure>::CovarianceQuantityBase(
    const std::string& uniquePrefix, const std::vector<glm::mat3>& positionCovariances_,
    const std::vector<glm::mat3>& rotationCovariances_, const std::vector<glm::mat3>& poseRotations_)
    : positionCovariances(positionCovariances_), rotationCovariances(rotationCovariances_),
      poseRotations(poseRotations_), showPositionEllipsoids(uniquePrefix + "#showPositionEllipsoids", true),
      showRotationEllipsoids(uniquePrefix + "#showRotationEllipsoids", true),
      showOnlyLast(uniquePrefix + "#showOnlyLast", true), showEveryNth(uniquePrefix + "#showEveryNth", 1),
      positionScale(uniquePrefix + "#positionScale", 1.0f), rotationScale(uniquePrefix + "#rotationScale", 1.0f),
      positionEllipsoidColor(uniquePrefix + "#positionEllipsoidColor", glm::vec3{0.6f, 0.2f, 0.8f}),
      rotationEllipsoidColor(uniquePrefix + "#rotationEllipsoidColor", glm::vec3{1.0f, 0.9f, 0.2f}),
      transparency(uniquePrefix + "#transparency", 0.5f) {
  createSphereGeometry();
}

// === Static helper methods for 6x6 covariance extraction ===

template <typename DerivedQuantity, typename ParentStructure>
template <typename Mat6x6>
glm::mat3 CovarianceQuantityBase<DerivedQuantity, ParentStructure>::extractPositionCovariance(const Mat6x6& cov6x6) {
  // Extract position covariance (bottom-right 3x3 of 6x6)
  // Convention: indices 3-5 are x, y, z position
  // GLM uses column-major: glm::mat3[col][row]
  // Assumes Eigen-style operator() access: (row, col)
  glm::mat3 posCov{0.0f};
  for (int row{}; row < 3; ++row) {
    for (int col{}; col < 3; ++col) {
      posCov[col][row] = cov6x6(row + 3, col + 3);
    }
  }
  return posCov;
}

template <typename DerivedQuantity, typename ParentStructure>
template <typename Mat6x6>
glm::mat3 CovarianceQuantityBase<DerivedQuantity, ParentStructure>::extractRotationCovariance(const Mat6x6& cov6x6) {
  // Extract rotation covariance (top-left 3x3 of 6x6)
  // Convention: indices 0-2 are roll, pitch, yaw rotations
  // Assumes Eigen-style operator() access: (row, col)
  glm::mat3 rotCov{0.0f};
  for (int row{}; row < 3; ++row) {
    for (int col{}; col < 3; ++col) {
      rotCov[col][row] = cov6x6(row, col);
    }
  }
  return rotCov;
}

template <typename DerivedQuantity, typename ParentStructure>
template <typename Transform>
glm::mat3 CovarianceQuantityBase<DerivedQuantity, ParentStructure>::extractPoseRotation(const Transform& pose) {
  // Extract rotation matrix from Eigen::Transform types
  // Assumes the transform has a .rotation() method returning a 3x3 rotation matrix
  auto eigenRot = pose.rotation();
  glm::mat3 rotation{};
  for (int row{}; row < 3; ++row) {
    for (int col{}; col < 3; ++col) {
      rotation[col][row] = eigenRot(row, col);
    }
  }
  return rotation;
}

// === Sphere geometry generation ===

template <typename DerivedQuantity, typename ParentStructure>
void CovarianceQuantityBase<DerivedQuantity, ParentStructure>::createSphereGeometry(int subdivisions) {
  // Create an icosphere (subdivided icosahedron) for smooth ellipsoids
  sphereVertices.clear();
  sphereNormals.clear();
  sphereIndices.clear();
  sphereFaces.clear();

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

  // Subdivide for smoother sphere
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
    sphereFaces.push_back(f);
  }
}

// === Ellipsoid parameter computation ===

template <typename DerivedQuantity, typename ParentStructure>
typename CovarianceQuantityBase<DerivedQuantity, ParentStructure>::EllipsoidParams
CovarianceQuantityBase<DerivedQuantity, ParentStructure>::computeEllipsoidParams(const glm::mat3& covariance,
                                                                                 float scale) const {
  EllipsoidParams params;

  // Convert GLM matrix to Eigen for stable eigendecomposition
  // GLM is column-major: mat[col][row]
  Eigen::Matrix3f cov_eigen{};
  for (int i{}; i < 3; ++i)
    for (int j{}; j < 3; ++j) cov_eigen(i, j) = 0.5f * (covariance[j][i] + covariance[i][j]); // Ensure symmetry

  // Perform eigendecomposition using Eigen's SelfAdjointEigenSolver
  // This is specifically designed for symmetric matrices and is very stable
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigensolver(cov_eigen);

  if (eigensolver.info() != Eigen::Success) {
    // Fallback to identity if eigendecomposition failed
    params.axes = glm::vec3(0.01f) * scale;
    params.rotation = glm::mat3(1.0f);
    return params;
  }

  // Get eigenvalues (already sorted in ascending order by SelfAdjointEigenSolver)
  Eigen::Vector3f eigenvalues{eigensolver.eigenvalues()};
  Eigen::Matrix3f eigenvectors{eigensolver.eigenvectors()};

  // Ensure eigenvalues are positive
  for (int i{}; i < 3; ++i) eigenvalues(i) = std::max(eigenvalues(i), 1e-8f);

  // Check for invalid/infinite eigenvalues
  for (int i{}; i < 3; ++i) {
    if (!std::isfinite(eigenvalues(i)) || eigenvalues(i) > 1e6f) {
      params.axes = glm::vec3(0.01f) * scale;
      params.rotation = glm::mat3(1.0f);
      return params;
    }
  }

  // Sort in descending order (largest first) for consistent visualization
  // Swap indices 0 and 2 since SelfAdjointEigenSolver returns ascending order
  std::swap(eigenvalues(0), eigenvalues(2));
  eigenvectors.col(0).swap(eigenvectors.col(2));

  // Ensure consistent eigenvector orientation by enforcing that the component
  // with largest absolute value is always positive
  for (int i{}; i < 3; ++i) {
    // Find component with largest absolute value
    int maxIdx{};
    float maxComp{std::abs(eigenvectors(0, i))};
    for (int j{1}; j < 3; ++j) {
      if (std::abs(eigenvectors(j, i)) > maxComp) {
        maxComp = std::abs(eigenvectors(j, i));
        maxIdx = j;
      }
    }
    // Flip if largest component is negative
    if (eigenvectors(maxIdx, i) < 0.0f) eigenvectors.col(i) = -eigenvectors.col(i);
  }

  // Ensure right-handed coordinate system (det = +1)
  float det{eigenvectors.determinant()};
  if (!std::isfinite(det) || std::abs(det) < 0.01f) {
    // Degenerate matrix, use identity
    params.axes = glm::vec3(0.01f) * scale;
    params.rotation = glm::mat3(1.0f);
    return params;
  }
  if (det < 0.0f) eigenvectors.col(2) = -eigenvectors.col(2); // Flip third eigenvector

  // Semi-axes are sqrt of eigenvalues (standard deviations), scaled by user parameter
  glm::vec3 axes{std::sqrt(eigenvalues(0)) * scale, std::sqrt(eigenvalues(1)) * scale,
                 std::sqrt(eigenvalues(2)) * scale};

  // Clamp axes to reasonable bounds
  const float maxAxisLength{1000.0f};
  for (int i{}; i < 3; ++i) {
    if (!std::isfinite(axes[i]))
      axes[i] = 0.01f;
    else
      axes[i] = glm::clamp(axes[i], 1e-6f, maxAxisLength);
  }

  // Convert Eigen rotation matrix back to GLM (column-major)
  glm::mat3 rotation{};
  for (int i{}; i < 3; ++i)
    for (int j{}; j < 3; ++j)
      rotation[i][j] = eigenvectors(j, i); // Transpose: GLM is column-major, Eigen is row-major access

  params.axes = axes;
  params.rotation = rotation;

  return params;
}


// === Preparation ===

template <typename DerivedQuantity, typename ParentStructure>
void CovarianceQuantityBase<DerivedQuantity, ParentStructure>::prepareEllipsoidGeometry() {
  // Ensure sphere geometry exists
  if (sphereVertices.empty()) {
    createSphereGeometry();
  }
}

// === Drawing ===

template <typename DerivedQuantity, typename ParentStructure>
void CovarianceQuantityBase<DerivedQuantity, ParentStructure>::drawEllipsoids(
    const std::string& parentName, const std::string& quantityName, size_t numElements,
    const std::function<glm::vec3(size_t)>& getPosition) {

  // Determine which elements to draw
  size_t startIdx = 0;
  size_t endIdx = numElements;

  if (showOnlyLast.get() && endIdx > 0) {
    startIdx = endIdx - 1;
  }

  // Get visualization parameters
  float posScale = positionScale.get();
  float rotScale = rotationScale.get();
  float alpha = transparency.get();
  int everyNth = showEveryNth.get();

  // Batch all ellipsoids into single meshes to avoid polluting structure list
  std::vector<glm::vec3> posEllipsoidVerts;
  std::vector<glm::uvec3> posEllipsoidFaces;
  std::vector<glm::vec3> rotEllipsoidVerts;
  std::vector<glm::uvec3> rotEllipsoidFaces;

  uint32_t posVertexOffset = 0;
  uint32_t rotVertexOffset = 0;

  // Build batched geometry - only process elements we'll actually draw
  for (size_t i = startIdx; i < numElements; i++) {
    glm::vec3 elementPos = getPosition(i);

    // Apply "every Nth" filter, but ALWAYS show the last element
    bool isLastElement = (i == numElements - 1);
    bool passesNthFilter = (i % everyNth == 0) || isLastElement;

    // Handle position ellipsoid
    if (i < positionCovariances.size() && showPositionEllipsoids.get() && passesNthFilter) {
      EllipsoidParams params = computeEllipsoidParams(positionCovariances[i], posScale);

      // Get pose rotation if available
      glm::mat3 poseRotation = glm::mat3(1.0f); // Identity by default
      if (i < poseRotations.size()) {
        poseRotation = poseRotations[i];
      }

      // Transform and add sphere vertices to batched geometry
      for (const auto& v : sphereVertices) {
        // Scale by eigenvalues (axes lengths) and rotate by eigenvectors
        glm::vec3 scaled = glm::vec3(v.x * params.axes.x, v.y * params.axes.y, v.z * params.axes.z);
        glm::vec3 rotated = params.rotation * scaled;
        // Apply pose rotation to orient ellipsoid with the pose
        glm::vec3 orientedRotated = poseRotation * rotated;
        posEllipsoidVerts.push_back(elementPos + orientedRotated);
      }

      // Add faces with proper vertex offset
      for (const auto& face : sphereFaces) {
        posEllipsoidFaces.push_back(
            glm::uvec3(face.x + posVertexOffset, face.y + posVertexOffset, face.z + posVertexOffset));
      }
      posVertexOffset += sphereVertices.size();
    }

    // Handle rotation ellipsoids (2D discs perpendicular to each axis, RViz-style)
    if (i < rotationCovariances.size() && showRotationEllipsoids.get() && passesNthFilter) {
      // Get position ellipsoid parameters to determine offset distance per direction
      EllipsoidParams posParams{};
      posParams.axes = glm::vec3(0.0f);
      posParams.rotation = glm::mat3(1.0f); // Identity if no position covariance
      if (i < positionCovariances.size()) {
        posParams = computeEllipsoidParams(positionCovariances[i], posScale);
      }

      // Extract diagonal rotation uncertainties (already converted to metric scale in curve_network.ipp)
      const glm::mat3& rotCov = rotationCovariances[i];
      glm::vec3 rotUncertainties{rotCov[0][0], rotCov[1][1], rotCov[2][2]};

      // Create 3 discs, one perpendicular to each world axis (X, Y, Z)
      // Each disc uses the OTHER two axis uncertainties for its radii
      // Discs remain fixed in world frame at pose position, aligned with world axes
      for (int axis = 0; axis < 3; axis++) {
        // Compute offset direction along the world axis (no rotation applied)
        glm::vec3 offsetDir(0.0f);
        offsetDir[axis] = 1.0f;

        // Calculate ellipsoid radius in this world direction
        // Transform direction to ellipsoid local frame
        glm::vec3 localDir = glm::transpose(posParams.rotation) * offsetDir;

        // Calculate radius along this direction using the ellipsoid equation
        // r = sqrt((localDir.x * a)^2 + (localDir.y * b)^2 + (localDir.z * c)^2)
        float ellipsoidRadius = std::sqrt(localDir.x * localDir.x * posParams.axes.x * posParams.axes.x +
                                          localDir.y * localDir.y * posParams.axes.y * posParams.axes.y +
                                          localDir.z * localDir.z * posParams.axes.z * posParams.axes.z);

        // Position disc at ellipsoid surface + fixed offset multiplier (like RViz)
        float offsetDistance = std::max(ellipsoidRadius * 1.3f, 0.15f);
        glm::vec3 offset = offsetDir * offsetDistance;

        // Determine disc radius - each disc shows uncertainty of rotation AROUND its perpendicular axis
        // Disc perpendicular to X shows roll uncertainty (rotation around X)
        // Disc perpendicular to Y shows pitch uncertainty (rotation around Y)
        // Disc perpendicular to Z shows yaw uncertainty (rotation around Z)
        glm::vec3 discRadii{1.0f, 1.0f, 1.0f};
        const float thinness{0.001f}; // Make discs very thin for proper 2D appearance

        if (axis == 0) { // Disc perpendicular to X-axis (in YZ plane) - shows roll uncertainty
          float radius = rotUncertainties[0] * rotScale; // roll uncertainty (rotation around X)
          discRadii.x = thinness;
          discRadii.y = radius;
          discRadii.z = radius;
        } else if (axis == 1) { // Disc perpendicular to Y-axis (in XZ plane) - shows pitch uncertainty
          float radius = rotUncertainties[1] * rotScale; // pitch uncertainty (rotation around Y)
          discRadii.x = radius;
          discRadii.y = thinness;
          discRadii.z = radius;
        } else { // Disc perpendicular to Z-axis (in XY plane) - shows yaw uncertainty
          float radius = rotUncertainties[2] * rotScale; // yaw uncertainty (rotation around Z)
          discRadii.x = radius;
          discRadii.y = radius;
          discRadii.z = thinness;
        }

        // Create the 2D disc vertices - keep in world frame alignment
        for (const auto& v : sphereVertices) {
          // Scale sphere vertices to create a thin disc (aligned with world axes)
          glm::vec3 scaled{v.x * discRadii.x, v.y * discRadii.y, v.z * discRadii.z};
          // Position disc at offset from pose position (no rotation applied to disc)
          rotEllipsoidVerts.push_back(elementPos + offset + scaled);
        }

        // Add faces with proper vertex offset
        for (const auto& face : sphereFaces) {
          rotEllipsoidFaces.push_back(
              glm::uvec3(face.x + rotVertexOffset, face.y + rotVertexOffset, face.z + rotVertexOffset));
        }
        rotVertexOffset += sphereVertices.size();
      }
    }
  }

  // Register or remove position ellipsoids mesh
  std::string posMeshName = parentName + "_" + quantityName + "_position_ellipsoids";

  // Always check if an existing mesh exists and ensure it's handled properly
  bool meshExists = polyscope::hasSimpleTriangleMesh(posMeshName);
  bool shouldShow = showPositionEllipsoids.get() && !posEllipsoidVerts.empty() && !posEllipsoidFaces.empty();

  if (shouldShow) {
    // Create or update mesh with valid data
    if (meshExists) {
      auto* mesh = polyscope::getSimpleTriangleMesh(posMeshName);
      if (mesh != nullptr) {
        mesh->update(posEllipsoidVerts, posEllipsoidFaces);
        mesh->setEnabled(true);
        mesh->setSurfaceColor(positionEllipsoidColor.get());
        mesh->setTransparency(alpha);
        mesh->setMaterial("wax");
      }
    } else {
      // Create new mesh
      auto* mesh = polyscope::registerSimpleTriangleMesh(posMeshName, posEllipsoidVerts, posEllipsoidFaces);
      if (mesh != nullptr) {
        mesh->setEnabled(true);
        mesh->setSurfaceColor(positionEllipsoidColor.get());
        mesh->setTransparency(alpha);
        mesh->setMaterial("wax");
      }
    }
  } else {
    // Hide and remove any existing mesh
    if (meshExists) {
      auto* mesh = polyscope::getSimpleTriangleMesh(posMeshName);
      if (mesh != nullptr) {
        // Disable to immediately hide from UI
        mesh->setEnabled(false);
      }
      // Remove completely
      polyscope::removeSimpleTriangleMesh(posMeshName, false);
    }
  }

  // Register or remove rotation ellipsoids mesh
  std::string rotMeshName = parentName + "_" + quantityName + "_rotation_ellipsoids";

  // Always check if an existing mesh exists and ensure it's handled properly
  meshExists = polyscope::hasSimpleTriangleMesh(rotMeshName);
  shouldShow = showRotationEllipsoids.get() && !rotEllipsoidVerts.empty() && !rotEllipsoidFaces.empty();

  if (shouldShow) {
    // Create or update mesh with valid data
    if (meshExists) {
      auto* mesh = polyscope::getSimpleTriangleMesh(rotMeshName);
      if (mesh != nullptr) {
        mesh->update(rotEllipsoidVerts, rotEllipsoidFaces);
        mesh->setEnabled(true);
        mesh->setSurfaceColor(rotationEllipsoidColor.get());
        mesh->setTransparency(alpha);
        mesh->setMaterial("wax");
      }
    } else {
      // Create new mesh
      auto* mesh = polyscope::registerSimpleTriangleMesh(rotMeshName, rotEllipsoidVerts, rotEllipsoidFaces);
      if (mesh != nullptr) {
        mesh->setEnabled(true);
        mesh->setSurfaceColor(rotationEllipsoidColor.get());
        mesh->setTransparency(alpha);
        mesh->setMaterial("wax");
      }
    }
  } else {
    // Hide and remove any existing mesh
    if (meshExists) {
      auto* mesh = polyscope::getSimpleTriangleMesh(rotMeshName);
      if (mesh != nullptr) {
        // Disable to immediately hide from UI
        mesh->setEnabled(false);
      }
      // Remove completely
      polyscope::removeSimpleTriangleMesh(rotMeshName, false);
    }
  }
}

// === UI ===

template <typename DerivedQuantity, typename ParentStructure>
void CovarianceQuantityBase<DerivedQuantity, ParentStructure>::buildCovarianceUI() {
  // General display options
  if (ImGui::Checkbox("Show Only Last", &showOnlyLast.get())) {
    setShowOnlyLast(getShowOnlyLast());
  }

  // Slider for showing every Nth ellipsoid
  if (ImGui::SliderInt("Show Every Nth", &showEveryNth.get(), 1, 100, "%d")) {
    setShowEveryNth(getShowEveryNth());
  }
  if (ImGui::IsItemHovered()) {
    ImGui::SetTooltip("Show every Nth ellipsoid (1=all, 10=every 10th, etc.). Always shows the last one.");
  }

  if (ImGui::SliderFloat("Transparency", &transparency.get(), 0.0f, 1.0f)) {
    setTransparency(getTransparency());
  }

  ImGui::Separator();

  // Position covariance settings
  if (ImGui::TreeNode("Position Covariance")) {
    if (ImGui::Checkbox("Show Position Ellipsoids", &showPositionEllipsoids.get())) {
      setShowPositionEllipsoids(getShowPositionEllipsoids());
    }

    if (ImGui::SliderFloat("Position Scale", &positionScale.get(), 0.1f, 1000.0f)) {
      setPositionScale(getPositionScale());
    }
    if (ImGui::IsItemHovered()) {
      ImGui::SetTooltip("Scales position ellipsoids and rotation disc offsets");
    }

    if (ImGui::ColorEdit3("Position Color", &positionEllipsoidColor.get()[0])) {
      setPositionEllipsoidColor(getPositionEllipsoidColor());
    }

    ImGui::TreePop();
  }

  ImGui::Separator();

  // Rotation covariance settings
  if (ImGui::TreeNode("Rotation Covariance")) {
    if (ImGui::Checkbox("Show Rotation Ellipsoids", &showRotationEllipsoids.get())) {
      setShowRotationEllipsoids(getShowRotationEllipsoids());
    }

    if (ImGui::SliderFloat("Rotation Scale", &rotationScale.get(), 0.1f, 1000.0f)) {
      setRotationScale(getRotationScale());
    }
    if (ImGui::IsItemHovered()) {
      ImGui::SetTooltip("Scales rotation disc sizes only");
    }

    if (ImGui::ColorEdit3("Rotation Color", &rotationEllipsoidColor.get()[0])) {
      setRotationEllipsoidColor(getRotationEllipsoidColor());
    }

    ImGui::TreePop();
  }
}

template <typename DerivedQuantity, typename ParentStructure>
void CovarianceQuantityBase<DerivedQuantity, ParentStructure>::buildCovarianceInfoGUI(size_t elementInd) {
  if (elementInd < positionCovariances.size()) {
    ImGui::Text("Position Cov:");
    const glm::mat3& cov = positionCovariances[elementInd];
    ImGui::Text("  [%.3f %.3f %.3f]", cov[0][0], cov[0][1], cov[0][2]);
    ImGui::Text("  [%.3f %.3f %.3f]", cov[1][0], cov[1][1], cov[1][2]);
    ImGui::Text("  [%.3f %.3f %.3f]", cov[2][0], cov[2][1], cov[2][2]);
  }

  if (elementInd < rotationCovariances.size()) {
    ImGui::Text("Rotation Cov:");
    const glm::mat3& cov = rotationCovariances[elementInd];
    ImGui::Text("  [%.3f %.3f %.3f]", cov[0][0], cov[0][1], cov[0][2]);
    ImGui::Text("  [%.3f %.3f %.3f]", cov[1][0], cov[1][1], cov[1][2]);
    ImGui::Text("  [%.3f %.3f %.3f]", cov[2][0], cov[2][1], cov[2][2]);
  }
}

// === Setters and getters ===

template <typename DerivedQuantity, typename ParentStructure>
DerivedQuantity* CovarianceQuantityBase<DerivedQuantity, ParentStructure>::setShowPositionEllipsoids(bool val) {
  showPositionEllipsoids = val;
  // Force immediate mesh update by calling draw() directly
  // This ensures meshes are created/removed immediately when toggling
  derived()->draw();
  polyscope::requestRedraw();
  return derived();
}

template <typename DerivedQuantity, typename ParentStructure>
bool CovarianceQuantityBase<DerivedQuantity, ParentStructure>::getShowPositionEllipsoids() {
  return showPositionEllipsoids.get();
}

template <typename DerivedQuantity, typename ParentStructure>
DerivedQuantity* CovarianceQuantityBase<DerivedQuantity, ParentStructure>::setShowRotationEllipsoids(bool val) {
  showRotationEllipsoids = val;
  // Force immediate mesh update by calling draw() directly
  // This ensures meshes are created/removed immediately when toggling
  derived()->draw();
  polyscope::requestRedraw();
  return derived();
}

template <typename DerivedQuantity, typename ParentStructure>
bool CovarianceQuantityBase<DerivedQuantity, ParentStructure>::getShowRotationEllipsoids() {
  return showRotationEllipsoids.get();
}

template <typename DerivedQuantity, typename ParentStructure>
DerivedQuantity* CovarianceQuantityBase<DerivedQuantity, ParentStructure>::setShowOnlyLast(bool val) {
  showOnlyLast = val;
  polyscope::requestRedraw();
  return derived();
}

template <typename DerivedQuantity, typename ParentStructure>
bool CovarianceQuantityBase<DerivedQuantity, ParentStructure>::getShowOnlyLast() {
  return showOnlyLast.get();
}

template <typename DerivedQuantity, typename ParentStructure>
DerivedQuantity* CovarianceQuantityBase<DerivedQuantity, ParentStructure>::setShowEveryNth(int val) {
  showEveryNth = std::max(1, val); // Ensure at least 1
  polyscope::requestRedraw();
  return derived();
}

template <typename DerivedQuantity, typename ParentStructure>
int CovarianceQuantityBase<DerivedQuantity, ParentStructure>::getShowEveryNth() {
  return showEveryNth.get();
}

template <typename DerivedQuantity, typename ParentStructure>
DerivedQuantity* CovarianceQuantityBase<DerivedQuantity, ParentStructure>::setPositionScale(float val) {
  positionScale = val;
  polyscope::requestRedraw();
  return derived();
}

template <typename DerivedQuantity, typename ParentStructure>
float CovarianceQuantityBase<DerivedQuantity, ParentStructure>::getPositionScale() {
  return positionScale.get();
}

template <typename DerivedQuantity, typename ParentStructure>
DerivedQuantity* CovarianceQuantityBase<DerivedQuantity, ParentStructure>::setRotationScale(float val) {
  rotationScale = val;
  polyscope::requestRedraw();
  return derived();
}

template <typename DerivedQuantity, typename ParentStructure>
float CovarianceQuantityBase<DerivedQuantity, ParentStructure>::getRotationScale() {
  return rotationScale.get();
}

template <typename DerivedQuantity, typename ParentStructure>
DerivedQuantity* CovarianceQuantityBase<DerivedQuantity, ParentStructure>::setPositionEllipsoidColor(glm::vec3 val) {
  positionEllipsoidColor = val;
  polyscope::requestRedraw();
  return derived();
}

template <typename DerivedQuantity, typename ParentStructure>
glm::vec3 CovarianceQuantityBase<DerivedQuantity, ParentStructure>::getPositionEllipsoidColor() {
  return positionEllipsoidColor.get();
}

template <typename DerivedQuantity, typename ParentStructure>
DerivedQuantity* CovarianceQuantityBase<DerivedQuantity, ParentStructure>::setRotationEllipsoidColor(glm::vec3 val) {
  rotationEllipsoidColor = val;
  polyscope::requestRedraw();
  return derived();
}

template <typename DerivedQuantity, typename ParentStructure>
glm::vec3 CovarianceQuantityBase<DerivedQuantity, ParentStructure>::getRotationEllipsoidColor() {
  return rotationEllipsoidColor.get();
}

template <typename DerivedQuantity, typename ParentStructure>
DerivedQuantity* CovarianceQuantityBase<DerivedQuantity, ParentStructure>::setTransparency(float val) {
  transparency = glm::clamp(val, 0.0f, 1.0f);
  polyscope::requestRedraw();
  return derived();
}

template <typename DerivedQuantity, typename ParentStructure>
float CovarianceQuantityBase<DerivedQuantity, ParentStructure>::getTransparency() {
  return transparency.get();
}

} // namespace polyscope
