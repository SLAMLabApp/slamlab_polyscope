// Covariance ellipsoid visualization for point clouds
// Implementation

#include "polyscope/point_cloud_covariance_quantity.h"
#include "polyscope/point_cloud.h"

namespace polyscope {

PointCloudCovarianceQuantity::PointCloudCovarianceQuantity(std::string name, PointCloud& cloud_,
                                                           const std::vector<glm::mat3>& positionCovariances_,
                                                           const std::vector<glm::mat3>& rotationCovariances_,
                                                           const std::vector<glm::mat3>& poseRotations_)
    : PointCloudQuantity(name, cloud_, false),
      CovarianceQuantityBase<PointCloudCovarianceQuantity, PointCloud>(uniquePrefix(), positionCovariances_,
                                                                       rotationCovariances_, poseRotations_) {}

std::string PointCloudCovarianceQuantity::niceName() { return name + " (covariance)"; }

void PointCloudCovarianceQuantity::draw() {
  // Always clean up meshes first, regardless of enabled state
  std::string posMeshName = parent.name + "_" + name + "_position_ellipsoids";
  std::string rotMeshName = parent.name + "_" + name + "_rotation_ellipsoids";

  // If not enabled, just ensure meshes are cleaned up and return
  if (!isEnabled()) {
    polyscope::removeSimpleTriangleMesh(posMeshName, false);
    polyscope::removeSimpleTriangleMesh(rotMeshName, false);
    return;
  }

  // Get the point cloud parent structure
  PointCloud& cloud = dynamic_cast<PointCloud&>(parent);

  // Use base class drawing with lambda to get point positions
  drawEllipsoids(parent.name, name, cloud.nPoints(), [&](size_t i) { return cloud.getPointPosition(i); });
}

void PointCloudCovarianceQuantity::buildCustomUI() {
  // Use base class UI implementation
  buildCovarianceUI();
}

void PointCloudCovarianceQuantity::refresh() { prepareEllipsoidGeometry(); }

void PointCloudCovarianceQuantity::buildInfoGUI(size_t pointInd) {
  ImGui::TextUnformatted(name.c_str());
  ImGui::NextColumn();

  // Use base class info GUI implementation
  buildCovarianceInfoGUI(pointInd);

  ImGui::NextColumn();
}

} // namespace polyscope
