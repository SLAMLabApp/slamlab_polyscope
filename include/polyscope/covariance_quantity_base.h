// Base class for covariance ellipsoid visualization
// Shared between point clouds and curve networks using CRTP pattern

#pragma once

#include "polyscope/persistent_value.h"
#include "polyscope/render/engine.h"

#include <functional>
#include <glm/glm.hpp>
#include <map>
#include <vector>

namespace polyscope {

// CRTP base class for covariance visualization
// DerivedQuantity: the derived quantity type (e.g., CurveNetworkNodeCovarianceQuantity)
// ParentStructure: the structure type (e.g., CurveNetwork, PointCloud)
template <typename DerivedQuantity, typename ParentStructure>
class CovarianceQuantityBase {
protected:
  // === Covariance data ===
  std::vector<glm::mat3> positionCovariances; // 3x3 position covariance for each element
  std::vector<glm::mat3> rotationCovariances; // 3x3 rotation covariance for each element
  std::vector<glm::mat3> poseRotations;       // 3x3 rotation matrix for each element's pose orientation

  // === Visualization parameters ===
  PersistentValue<bool> showPositionEllipsoids;
  PersistentValue<bool> showRotationEllipsoids;
  PersistentValue<bool> showOnlyLast;
  PersistentValue<int> showEveryNth;
  PersistentValue<float> positionScale;
  PersistentValue<float> rotationScale;
  PersistentValue<glm::vec3> positionEllipsoidColor;
  PersistentValue<glm::vec3> rotationEllipsoidColor;
  PersistentValue<float> transparency;

  // === Sphere geometry template ===
  std::vector<glm::vec3> sphereVertices;
  std::vector<glm::vec3> sphereNormals;
  std::vector<uint32_t> sphereIndices;
  std::vector<glm::uvec3> sphereFaces;

  // === Helper structures ===
  struct EllipsoidParams {
    glm::vec3 axes;     // Semi-axes lengths
    glm::mat3 rotation; // Rotation matrix from eigenvectors
  };

public:
  // Constructor
  CovarianceQuantityBase(const std::string& uniquePrefix, const std::vector<glm::mat3>& positionCovariances_,
                         const std::vector<glm::mat3>& rotationCovariances_,
                         const std::vector<glm::mat3>& poseRotations_);

  // === Static helper methods for 6x6 covariance matrices ===
  // These methods extract the position and rotation 3x3 blocks from 6x6 covariance matrices
  // Convention: 6x6 matrix = [rotation 3x3, cross 3x3; cross 3x3, position 3x3]
  //            indices 0-2: rotation (roll, pitch, yaw)
  //            indices 3-5: position (x, y, z)

  // Extract position covariance (bottom-right 3x3 block, indices [3:5, 3:5])
  template <typename Mat6x6>
  static glm::mat3 extractPositionCovariance(const Mat6x6& cov6x6);

  // Extract rotation covariance (top-left 3x3 block, indices [0:2, 0:2])
  template <typename Mat6x6>
  static glm::mat3 extractRotationCovariance(const Mat6x6& cov6x6);

  // Extract rotation matrix from a transform/pose
  template <typename Transform>
  static glm::mat3 extractPoseRotation(const Transform& pose);

  // === Core functionality ===
  void createSphereGeometry(int subdivisions = 2);
  EllipsoidParams computeEllipsoidParams(const glm::mat3& covariance, float scale) const;
  void prepareEllipsoidGeometry();

  // Draw ellipsoids - requires position getter function from derived class
  void drawEllipsoids(const std::string& parentName, const std::string& quantityName, size_t numElements,
                      const std::function<glm::vec3(size_t)>& getPosition);

  // === UI ===
  void buildCovarianceUI();
  void buildCovarianceInfoGUI(size_t elementInd);

  // === Setters and getters ===
  DerivedQuantity* setShowPositionEllipsoids(bool val);
  bool getShowPositionEllipsoids();

  DerivedQuantity* setShowRotationEllipsoids(bool val);
  bool getShowRotationEllipsoids();

  DerivedQuantity* setShowOnlyLast(bool val);
  bool getShowOnlyLast();

  DerivedQuantity* setShowEveryNth(int val);
  int getShowEveryNth();

  DerivedQuantity* setPositionScale(float val);
  float getPositionScale();

  DerivedQuantity* setRotationScale(float val);
  float getRotationScale();

  DerivedQuantity* setPositionEllipsoidColor(glm::vec3 val);
  glm::vec3 getPositionEllipsoidColor();

  DerivedQuantity* setRotationEllipsoidColor(glm::vec3 val);
  glm::vec3 getRotationEllipsoidColor();

  DerivedQuantity* setTransparency(float val);
  float getTransparency();

private:
  // Helper to cast to derived type (CRTP pattern)
  DerivedQuantity* derived() { return static_cast<DerivedQuantity*>(this); }
  const DerivedQuantity* derived() const { return static_cast<const DerivedQuantity*>(this); }
};

} // namespace polyscope

#include "polyscope/covariance_quantity_base.ipp"
