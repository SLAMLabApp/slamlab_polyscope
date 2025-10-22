// Covariance ellipsoid visualization for curve networks
// Visualizes uncertainty as 3D ellipsoids for position and 2D ellipsoids for rotation

#pragma once

#include "polyscope/covariance_quantity_base.h"
#include "polyscope/curve_network_quantity.h"

#include <glm/glm.hpp>
#include <vector>

namespace polyscope {

// Forward declare
class CurveNetwork;

// Represents covariance ellipsoids at curve network nodes
// Each node can have a 6x6 covariance matrix (3 rotation + 3 position)
// Uses the shared CovarianceQuantityBase for all common functionality
class CurveNetworkNodeCovarianceQuantity
    : public CurveNetworkQuantity,
      public CovarianceQuantityBase<CurveNetworkNodeCovarianceQuantity, CurveNetwork> {
public:
  // Make the base class a friend so it can access parent
  friend class CovarianceQuantityBase<CurveNetworkNodeCovarianceQuantity, CurveNetwork>;

  // Constructor with split 3x3 matrices (for internal use or advanced usage)
  CurveNetworkNodeCovarianceQuantity(std::string name, CurveNetwork& network_,
                                     const std::vector<glm::mat3>& positionCovariances_,
                                     const std::vector<glm::mat3>& rotationCovariances_,
                                     const std::vector<glm::mat3>& poseRotations_ = {});

  virtual void draw() override;
  virtual void buildCustomUI() override;
  virtual std::string niceName() override;
  virtual void refresh() override;
  virtual void buildNodeInfoGUI(size_t vInd) override;

  // === Setters and getters (use base class implementations via using declarations)
  using CovarianceQuantityBase<CurveNetworkNodeCovarianceQuantity, CurveNetwork>::setShowPositionEllipsoids;
  using CovarianceQuantityBase<CurveNetworkNodeCovarianceQuantity, CurveNetwork>::getShowPositionEllipsoids;
  using CovarianceQuantityBase<CurveNetworkNodeCovarianceQuantity, CurveNetwork>::setShowRotationEllipsoids;
  using CovarianceQuantityBase<CurveNetworkNodeCovarianceQuantity, CurveNetwork>::getShowRotationEllipsoids;
  using CovarianceQuantityBase<CurveNetworkNodeCovarianceQuantity, CurveNetwork>::setShowOnlyLast;
  using CovarianceQuantityBase<CurveNetworkNodeCovarianceQuantity, CurveNetwork>::getShowOnlyLast;
  using CovarianceQuantityBase<CurveNetworkNodeCovarianceQuantity, CurveNetwork>::setShowEveryNth;
  using CovarianceQuantityBase<CurveNetworkNodeCovarianceQuantity, CurveNetwork>::getShowEveryNth;
  using CovarianceQuantityBase<CurveNetworkNodeCovarianceQuantity, CurveNetwork>::setEllipsoidScale;
  using CovarianceQuantityBase<CurveNetworkNodeCovarianceQuantity, CurveNetwork>::getEllipsoidScale;
  using CovarianceQuantityBase<CurveNetworkNodeCovarianceQuantity, CurveNetwork>::setPositionEllipsoidColor;
  using CovarianceQuantityBase<CurveNetworkNodeCovarianceQuantity, CurveNetwork>::getPositionEllipsoidColor;
  using CovarianceQuantityBase<CurveNetworkNodeCovarianceQuantity, CurveNetwork>::setRotationEllipsoidColor;
  using CovarianceQuantityBase<CurveNetworkNodeCovarianceQuantity, CurveNetwork>::getRotationEllipsoidColor;
  using CovarianceQuantityBase<CurveNetworkNodeCovarianceQuantity, CurveNetwork>::setTransparency;
  using CovarianceQuantityBase<CurveNetworkNodeCovarianceQuantity, CurveNetwork>::getTransparency;
};

} // namespace polyscope
