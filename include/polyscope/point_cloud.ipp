// Copyright 2017-2023, Nicholas Sharp and the Polyscope contributors. https://polyscope.run

#pragma once

namespace polyscope {


// Shorthand to add a point cloud to polyscope
template <class T>
PointCloud* registerPointCloud(std::string name, const T& points) {
  checkInitialized();

  PointCloud* s = new PointCloud(name, standardizeVectorArray<glm::vec3, 3>(points));
  bool success = registerStructure(s);
  if (!success) {
    safeDelete(s);
  }
  return s;
}
template <class T>
PointCloud* registerPointCloud2D(std::string name, const T& points) {
  checkInitialized();

  std::vector<glm::vec3> points3D(standardizeVectorArray<glm::vec3, 2>(points));
  for (auto& v : points3D) {
    v.z = 0.;
  }
  PointCloud* s = new PointCloud(name, points3D);
  bool success = registerStructure(s);
  if (!success) {
    safeDelete(s);
  }
  return s;
}

template <class V>
void PointCloud::updatePointPositions(const V& newPositions) {
  validateSize(newPositions, nPoints(), "point cloud updated positions " + name);
  points.data = standardizeVectorArray<glm::vec3, 3>(newPositions);
  points.markHostBufferUpdated();
}

template <class V>
void PointCloud::updatePointPositions2D(const V& newPositions2D) {
  validateSize(newPositions2D, nPoints(), "point cloud updated positions " + name);
  std::vector<glm::vec3> positions3D = standardizeVectorArray<glm::vec3, 2>(newPositions2D);
  for (glm::vec3& v : positions3D) {
    v.z = 0.;
  }

  // Call the main version
  updatePointPositions(positions3D);
}


// Shorthand to get a point cloud from polyscope
inline PointCloud* getPointCloud(std::string name) {
  return dynamic_cast<PointCloud*>(getStructure(PointCloud::structureTypeName, name));
}
inline bool hasPointCloud(std::string name) { return hasStructure(PointCloud::structureTypeName, name); }
inline void removePointCloud(std::string name, bool errorIfAbsent) {
  removeStructure(PointCloud::structureTypeName, name, errorIfAbsent);
}


// =====================================================
// ============== Quantities
// =====================================================


template <class T>
PointCloudColorQuantity* PointCloud::addColorQuantity(std::string name, const T& colors) {
  validateSize(colors, nPoints(), "point cloud color quantity " + name);
  return addColorQuantityImpl(name, standardizeVectorArray<glm::vec3, 3>(colors));
}

template <class T>
PointCloudScalarQuantity* PointCloud::addScalarQuantity(std::string name, const T& data, DataType type) {
  validateSize(data, nPoints(), "point cloud scalar quantity " + name);
  return addScalarQuantityImpl(name, standardizeArray<float, T>(data), type);
}


template <class T>
PointCloudParameterizationQuantity* PointCloud::addParameterizationQuantity(std::string name, const T& param,
                                                                            ParamCoordsType type) {
  validateSize(param, nPoints(), "point cloud parameterization quantity " + name);
  return addParameterizationQuantityImpl(name, standardizeVectorArray<glm::vec2, 2>(param), type);
}

template <class T>
PointCloudParameterizationQuantity* PointCloud::addLocalParameterizationQuantity(std::string name, const T& param,
                                                                                 ParamCoordsType type) {
  validateSize(param, nPoints(), "point cloud parameterization quantity " + name);
  return addLocalParameterizationQuantityImpl(name, standardizeVectorArray<glm::vec2, 2>(param), type);
}

template <class T>
PointCloudVectorQuantity* PointCloud::addVectorQuantity(std::string name, const T& vectors, VectorType vectorType) {
  validateSize(vectors, nPoints(), "point cloud vector quantity " + name);
  return addVectorQuantityImpl(name, standardizeVectorArray<glm::vec3, 3>(vectors), vectorType);
}
template <class T>
PointCloudVectorQuantity* PointCloud::addVectorQuantity2D(std::string name, const T& vectors, VectorType vectorType) {
  validateSize(vectors, nPoints(), "point cloud vector quantity " + name);

  std::vector<glm::vec3> vectors3D(standardizeVectorArray<glm::vec3, 2>(vectors));
  for (auto& v : vectors3D) {
    v.z = 0.;
  }

  return addVectorQuantityImpl(name, vectors3D, vectorType);
}

// === Covariance 6x6 API (RECOMMENDED) ===

template <class T>
PointCloudCovarianceQuantity* PointCloud::addCovariance6x6Quantity(std::string name, const T& covariances6x6) {
  validateSize(covariances6x6, nPoints(), "point cloud covariance6x6 quantity " + name);

  // Extract 3x3 blocks from 6x6 matrices
  std::vector<glm::mat3> posCovs;
  std::vector<glm::mat3> rotCovs;
  std::vector<glm::mat3> poseRots;

  posCovs.reserve(covariances6x6.size());
  rotCovs.reserve(covariances6x6.size());
  poseRots.reserve(covariances6x6.size());

  for (const auto& cov6x6 : covariances6x6) {
    posCovs.push_back(PointCloudCovarianceQuantity::extractPositionCovariance(cov6x6));
    rotCovs.push_back(PointCloudCovarianceQuantity::extractRotationCovariance(cov6x6));
    poseRots.push_back(glm::mat3{1.0f}); // Identity - point clouds have no orientation
  }

  PointCloudCovarianceQuantity* q = new PointCloudCovarianceQuantity(name, *this, posCovs, rotCovs, poseRots);
  addQuantity(q);
  return q;
}

template <class TCov, class TPose>
PointCloudCovarianceQuantity* PointCloud::addCovariance6x6Quantity(std::string name, const TCov& covariances6x6,
                                                                   const TPose& poseRotations) {
  validateSize(covariances6x6, nPoints(), "point cloud covariance6x6 quantity " + name);
  validateSize(poseRotations, nPoints(), "point cloud covariance6x6 quantity poses " + name);

  // Extract 3x3 blocks from 6x6 matrices and pose rotations
  std::vector<glm::mat3> posCovs;
  std::vector<glm::mat3> rotCovs;
  std::vector<glm::mat3> poseRots;

  posCovs.reserve(covariances6x6.size());
  rotCovs.reserve(covariances6x6.size());
  poseRots.reserve(poseRotations.size());

  for (const auto& cov6x6 : covariances6x6) {
    posCovs.push_back(PointCloudCovarianceQuantity::extractPositionCovariance(cov6x6));
    rotCovs.push_back(PointCloudCovarianceQuantity::extractRotationCovariance(cov6x6));
  }

  for (const auto& rot : poseRotations) {
    poseRots.push_back(PointCloudCovarianceQuantity::extractPoseRotation(rot));
  }

  PointCloudCovarianceQuantity* q = new PointCloudCovarianceQuantity(name, *this, posCovs, rotCovs, poseRots);
  addQuantity(q);
  return q;
}

// === Advanced covariance API (3x3 blocks) ===

template <class T1, class T2>
PointCloudCovarianceQuantity* PointCloud::addCovarianceQuantity(std::string name, const T1& positionCovariances,
                                                                const T2& rotationCovariances) {
  validateSize(positionCovariances, nPoints(), "point cloud covariance quantity " + name);
  validateSize(rotationCovariances, nPoints(), "point cloud covariance quantity " + name);

  // Convert input types to std::vector<glm::mat3>
  std::vector<glm::mat3> posCovs;
  std::vector<glm::mat3> rotCovs;
  posCovs.reserve(positionCovariances.size());
  rotCovs.reserve(rotationCovariances.size());

  for (const auto& cov : positionCovariances) {
    posCovs.push_back(cov);
  }
  for (const auto& cov : rotationCovariances) {
    rotCovs.push_back(cov);
  }

  PointCloudCovarianceQuantity* q = new PointCloudCovarianceQuantity(name, *this, posCovs, rotCovs);
  addQuantity(q);
  return q;
}

template <class T1, class T2, class T3>
PointCloudCovarianceQuantity* PointCloud::addCovarianceQuantity(std::string name, const T1& positionCovariances,
                                                                const T2& rotationCovariances,
                                                                const T3& poseRotations) {
  validateSize(positionCovariances, nPoints(), "point cloud covariance quantity " + name);
  validateSize(rotationCovariances, nPoints(), "point cloud covariance quantity " + name);
  validateSize(poseRotations, nPoints(), "point cloud covariance quantity " + name);

  // Convert input types to std::vector<glm::mat3>
  std::vector<glm::mat3> posCovs;
  std::vector<glm::mat3> rotCovs;
  std::vector<glm::mat3> poseRots;
  posCovs.reserve(positionCovariances.size());
  rotCovs.reserve(rotationCovariances.size());
  poseRots.reserve(poseRotations.size());

  for (const auto& cov : positionCovariances) {
    posCovs.push_back(cov);
  }
  for (const auto& cov : rotationCovariances) {
    rotCovs.push_back(cov);
  }
  for (const auto& rot : poseRotations) {
    poseRots.push_back(rot);
  }

  PointCloudCovarianceQuantity* q = new PointCloudCovarianceQuantity(name, *this, posCovs, rotCovs, poseRots);
  addQuantity(q);
  return q;
}


} // namespace polyscope
