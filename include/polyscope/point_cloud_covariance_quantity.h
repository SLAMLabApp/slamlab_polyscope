// Covariance ellipsoid visualization for point clouds
// Visualizes uncertainty as 3D ellipsoids for position and 2D ellipsoids for rotation

#pragma once

#include "polyscope/covariance_quantity_base.h"
#include "polyscope/point_cloud_quantity.h"

#include <glm/glm.hpp>
#include <vector>

namespace polyscope {

// Forward declare
class PointCloud;

// Represents covariance ellipsoids at point cloud points
// Each point can have a 6x6 covariance matrix (3 rotation + 3 position)
// Uses the shared CovarianceQuantityBase for all common functionality
class PointCloudCovarianceQuantity : public PointCloudQuantity,
                                     public CovarianceQuantityBase<PointCloudCovarianceQuantity, PointCloud> {
public:
  // Make the base class a friend so it can access parent
  friend class CovarianceQuantityBase<PointCloudCovarianceQuantity, PointCloud>;

  // Constructor with split 3x3 matrices (for internal use or advanced usage)
  PointCloudCovarianceQuantity(std::string name, PointCloud& cloud_, const std::vector<glm::mat3>& positionCovariances_,
                               const std::vector<glm::mat3>& rotationCovariances_,
                               const std::vector<glm::mat3>& poseRotations_ = {});

  virtual void draw() override;
  virtual void buildCustomUI() override;
  virtual std::string niceName() override;
  virtual void refresh() override;
  virtual void buildInfoGUI(size_t pointInd) override;

  // === Setters and getters (use base class implementations via using declarations)
  using CovarianceQuantityBase<PointCloudCovarianceQuantity, PointCloud>::setShowPositionEllipsoids;
  using CovarianceQuantityBase<PointCloudCovarianceQuantity, PointCloud>::getShowPositionEllipsoids;
  using CovarianceQuantityBase<PointCloudCovarianceQuantity, PointCloud>::setShowRotationEllipsoids;
  using CovarianceQuantityBase<PointCloudCovarianceQuantity, PointCloud>::getShowRotationEllipsoids;
  using CovarianceQuantityBase<PointCloudCovarianceQuantity, PointCloud>::setShowOnlyLast;
  using CovarianceQuantityBase<PointCloudCovarianceQuantity, PointCloud>::getShowOnlyLast;
  using CovarianceQuantityBase<PointCloudCovarianceQuantity, PointCloud>::setShowEveryNth;
  using CovarianceQuantityBase<PointCloudCovarianceQuantity, PointCloud>::getShowEveryNth;
  using CovarianceQuantityBase<PointCloudCovarianceQuantity, PointCloud>::setEllipsoidScale;
  using CovarianceQuantityBase<PointCloudCovarianceQuantity, PointCloud>::getEllipsoidScale;
  using CovarianceQuantityBase<PointCloudCovarianceQuantity, PointCloud>::setPositionEllipsoidColor;
  using CovarianceQuantityBase<PointCloudCovarianceQuantity, PointCloud>::getPositionEllipsoidColor;
  using CovarianceQuantityBase<PointCloudCovarianceQuantity, PointCloud>::setRotationEllipsoidColor;
  using CovarianceQuantityBase<PointCloudCovarianceQuantity, PointCloud>::getRotationEllipsoidColor;
  using CovarianceQuantityBase<PointCloudCovarianceQuantity, PointCloud>::setTransparency;
  using CovarianceQuantityBase<PointCloudCovarianceQuantity, PointCloud>::getTransparency;
};

} // namespace polyscope
