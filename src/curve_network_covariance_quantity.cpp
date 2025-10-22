// Covariance ellipsoid visualization for curve networks
// Implementation

#include "polyscope/curve_network_covariance_quantity.h"
#include "polyscope/curve_network.h"

namespace polyscope {

CurveNetworkNodeCovarianceQuantity::CurveNetworkNodeCovarianceQuantity(
    std::string name, CurveNetwork& network_, const std::vector<glm::mat3>& positionCovariances_,
    const std::vector<glm::mat3>& rotationCovariances_, const std::vector<glm::mat3>& poseRotations_)
    : CurveNetworkQuantity(name, network_, false),
      CovarianceQuantityBase<CurveNetworkNodeCovarianceQuantity, CurveNetwork>(uniquePrefix(), positionCovariances_,
                                                                               rotationCovariances_, poseRotations_) {}

std::string CurveNetworkNodeCovarianceQuantity::niceName() { return name + " (covariance)"; }

void CurveNetworkNodeCovarianceQuantity::draw() {
  // Always clean up meshes first, regardless of enabled state
  std::string posMeshName = parent.name + "_" + name + "_position_ellipsoids";
  std::string rotMeshName = parent.name + "_" + name + "_rotation_ellipsoids";

  // If not enabled, just ensure meshes are cleaned up and return
  if (!isEnabled()) {
    polyscope::removeSimpleTriangleMesh(posMeshName, false);
    polyscope::removeSimpleTriangleMesh(rotMeshName, false);
    return;
  }

  // Get the curve network parent structure
  CurveNetwork& network = dynamic_cast<CurveNetwork&>(parent);

  // Use base class drawing with lambda to get node positions
  drawEllipsoids(parent.name, name, network.nNodes(), [&](size_t i) { return network.nodePositionsData[i]; });
}

void CurveNetworkNodeCovarianceQuantity::buildCustomUI() {
  // Use base class UI implementation
  buildCovarianceUI();
}

void CurveNetworkNodeCovarianceQuantity::refresh() { prepareEllipsoidGeometry(); }

void CurveNetworkNodeCovarianceQuantity::buildNodeInfoGUI(size_t vInd) {
  ImGui::TextUnformatted(name.c_str());
  ImGui::NextColumn();

  // Use base class info GUI implementation
  buildCovarianceInfoGUI(vInd);

  ImGui::NextColumn();
}

} // namespace polyscope
