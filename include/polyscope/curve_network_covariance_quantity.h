// Covariance ellipsoid visualization for curve networks
// Visualizes uncertainty as 3D ellipsoids for position and 2D ellipsoids for rotation

#pragma once

#include "polyscope/curve_network_quantity.h"
#include "polyscope/render/engine.h"

#include <glm/glm.hpp>
#include <vector>

namespace polyscope {

// Forward declare
class CurveNetwork;

// Represents covariance ellipsoids at curve network nodes
// Each node can have a 6x6 covariance matrix (3 rotation + 3 position)
class CurveNetworkNodeCovarianceQuantity : public CurveNetworkQuantity {
public:
  CurveNetworkNodeCovarianceQuantity(std::string name, CurveNetwork& network_,
                                     const std::vector<glm::mat3>& positionCovariances_,
                                     const std::vector<glm::mat3>& rotationCovariances_);

  virtual void draw() override;
  virtual void buildCustomUI() override;
  virtual std::string niceName() override;
  virtual void refresh() override;
  virtual void buildNodeInfoGUI(size_t vInd) override;

  // === Setters and getters

  // Set visualization options
  CurveNetworkNodeCovarianceQuantity* setShowPositionEllipsoids(bool val);
  bool getShowPositionEllipsoids();

  CurveNetworkNodeCovarianceQuantity* setShowRotationEllipsoids(bool val);
  bool getShowRotationEllipsoids();

  CurveNetworkNodeCovarianceQuantity* setShowOnlyLast(bool val);
  bool getShowOnlyLast();

  CurveNetworkNodeCovarianceQuantity* setEllipsoidScale(float val);
  float getEllipsoidScale();

  CurveNetworkNodeCovarianceQuantity* setPositionEllipsoidColor(glm::vec3 val);
  glm::vec3 getPositionEllipsoidColor();

  CurveNetworkNodeCovarianceQuantity* setRotationEllipsoidColor(glm::vec3 val);
  glm::vec3 getRotationEllipsoidColor();

  CurveNetworkNodeCovarianceQuantity* setTransparency(float val);
  float getTransparency();

private:
  // === Visualization parameters

  PersistentValue<bool> showPositionEllipsoids;
  PersistentValue<bool> showRotationEllipsoids;
  PersistentValue<bool> showOnlyLast;
  PersistentValue<float> ellipsoidScale;
  PersistentValue<glm::vec3> positionEllipsoidColor;
  PersistentValue<glm::vec3> rotationEllipsoidColor;
  PersistentValue<float> transparency;

  // === Covariance data

  std::vector<glm::mat3> positionCovariances; // 3x3 position covariance for each node
  std::vector<glm::mat3> rotationCovariances; // 3x3 rotation covariance for each node

  // === Rendering data

  std::shared_ptr<render::ShaderProgram> ellipsoidProgram;

  // Ellipsoid template geometry (unit sphere)
  std::vector<glm::vec3> sphereVertices;
  std::vector<glm::vec3> sphereNormals;
  std::vector<uint32_t> sphereIndices;
  std::vector<glm::uvec3> sphereFaces; // Triangle faces for mesh rendering

  // Helper functions
  void createSphereGeometry(int subdivisions = 3);
  void prepareEllipsoidGeometry();
  void drawEllipsoids();

  // Compute eigenvectors and eigenvalues for scaling and orientation
  struct EllipsoidParams {
    glm::vec3 center;
    glm::vec3 axes;     // eigenvalues (semi-axes lengths)
    glm::mat3 rotation; // eigenvectors (orientation)
  };

  EllipsoidParams computeEllipsoidParams(const glm::mat3& covariance, float scale) const;
};

} // namespace polyscope
