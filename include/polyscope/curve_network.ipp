// Copyright 2017-2023, Nicholas Sharp and the Polyscope contributors. https://polyscope.run

#pragma once

#include "polyscope/curve_network_covariance_quantity.h"

namespace polyscope {


// Shorthand to add a curve network to polyscope
template <class P, class E>
CurveNetwork* registerCurveNetwork(std::string name, const P& nodes, const E& edges) {
  checkInitialized();

  CurveNetwork* s = new CurveNetwork(name, standardizeVectorArray<glm::vec3, 3>(nodes),
                                     standardizeVectorArray<std::array<size_t, 2>, 2>(edges));
  bool success = registerStructure(s);
  if (!success) {
    safeDelete(s);
  }
  return s;
}
template <class P, class E>
CurveNetwork* registerCurveNetwork2D(std::string name, const P& nodes, const E& edges) {
  checkInitialized();

  std::vector<glm::vec3> points3D(standardizeVectorArray<glm::vec3, 2>(nodes));
  for (auto& v : points3D) {
    v.z = 0.;
  }
  CurveNetwork* s = new CurveNetwork(name, points3D, standardizeVectorArray<std::array<size_t, 2>, 2>(edges));
  bool success = registerStructure(s);
  if (!success) {
    safeDelete(s);
  }
  return s;
}


// Shorthand to add curve from a line of points
template <class P>
CurveNetwork* registerCurveNetworkLine(std::string name, const P& nodes) {
  checkInitialized();

  std::vector<std::array<size_t, 2>> edges;
  size_t N = adaptorF_size(nodes);
  for (size_t iE = 1; iE < N; iE++) {
    edges.push_back({iE - 1, iE});
  }

  CurveNetwork* s = new CurveNetwork(name, standardizeVectorArray<glm::vec3, 3>(nodes), edges);
  bool success = registerStructure(s);
  if (!success) {
    safeDelete(s);
  }
  return s;
}
template <class P>
CurveNetwork* registerCurveNetworkLine2D(std::string name, const P& nodes) {
  checkInitialized();

  std::vector<std::array<size_t, 2>> edges;
  size_t N = adaptorF_size(nodes);
  for (size_t iE = 1; iE < N; iE++) {
    edges.push_back({iE - 1, iE});
  }
  std::vector<glm::vec3> points3D(standardizeVectorArray<glm::vec3, 2>(nodes));
  for (auto& v : points3D) {
    v.z = 0.;
  }

  CurveNetwork* s = new CurveNetwork(name, points3D, edges);
  bool success = registerStructure(s);
  if (!success) {
    safeDelete(s);
  }
  return s;
}


template <class P>
CurveNetwork* registerCurveNetworkSegments(std::string name, const P& nodes) {
  checkInitialized();

  std::vector<std::array<size_t, 2>> edges;
  size_t N = adaptorF_size(nodes);

  if (N % 2 != 0) {
    exception("registerCurveNetworkSegments should have an even number of nodes");
  }

  for (size_t iE = 0; iE < N; iE += 2) {
    edges.push_back({iE, iE + 1});
  }

  CurveNetwork* s = new CurveNetwork(name, standardizeVectorArray<glm::vec3, 3>(nodes), edges);
  bool success = registerStructure(s);
  if (!success) {
    safeDelete(s);
  }
  return s;
}
template <class P>
CurveNetwork* registerCurveNetworkSegments2D(std::string name, const P& nodes) {
  checkInitialized();

  std::vector<std::array<size_t, 2>> edges;
  size_t N = adaptorF_size(nodes);

  if (N % 2 != 0) {
    exception("registerCurveNetworkSegments2D should have an even number of nodes");
  }

  for (size_t iE = 0; iE < N; iE += 2) {
    edges.push_back({iE, iE + 1});
  }

  std::vector<glm::vec3> points3D(standardizeVectorArray<glm::vec3, 2>(nodes));
  for (auto& v : points3D) {
    v.z = 0.;
  }

  CurveNetwork* s = new CurveNetwork(name, points3D, edges);
  bool success = registerStructure(s);
  if (!success) {
    safeDelete(s);
  }
  return s;
}

// Shorthand to add curve from a loop of points
template <class P>
CurveNetwork* registerCurveNetworkLoop(std::string name, const P& nodes) {
  checkInitialized();

  std::vector<std::array<size_t, 2>> edges;
  size_t N = adaptorF_size(nodes);
  for (size_t iE = 0; iE < N; iE++) {
    edges.push_back({iE, (iE + 1) % N});
  }

  CurveNetwork* s = new CurveNetwork(name, standardizeVectorArray<glm::vec3, 3>(nodes), edges);
  bool success = registerStructure(s);
  if (!success) {
    safeDelete(s);
  }
  return s;
}
template <class P>
CurveNetwork* registerCurveNetworkLoop2D(std::string name, const P& nodes) {
  checkInitialized();

  std::vector<std::array<size_t, 2>> edges;
  size_t N = adaptorF_size(nodes);
  for (size_t iE = 0; iE < N; iE++) {
    edges.push_back({iE, (iE + 1) % N});
  }
  std::vector<glm::vec3> points3D(standardizeVectorArray<glm::vec3, 2>(nodes));
  for (auto& v : points3D) {
    v.z = 0.;
  }

  CurveNetwork* s = new CurveNetwork(name, points3D, edges);
  bool success = registerStructure(s);
  if (!success) {
    safeDelete(s);
  }
  return s;
}


template <class V>
void CurveNetwork::updateNodePositions(const V& newPositions) {
  validateSize(newPositions, nNodes(), "newPositions");
  nodePositions.data = standardizeVectorArray<glm::vec3, 3>(newPositions);
  nodePositions.markHostBufferUpdated();
  recomputeGeometryIfPopulated();
}


template <class V>
void CurveNetwork::updateNodePositions2D(const V& newPositions2D) {
  validateSize(newPositions2D, nNodes(), "newPositions2D");
  std::vector<glm::vec3> positions3D = standardizeVectorArray<glm::vec3, 2>(newPositions2D);
  for (glm::vec3& v : positions3D) {
    v.z = 0.;
  }

  // Call the main version
  updateNodePositions(positions3D);
}

// Shorthand to get a curve network from polyscope
inline CurveNetwork* getCurveNetwork(std::string name) {
  return dynamic_cast<CurveNetwork*>(getStructure(CurveNetwork::structureTypeName, name));
}
inline bool hasCurveNetwork(std::string name) { return hasStructure(CurveNetwork::structureTypeName, name); }
inline void removeCurveNetwork(std::string name, bool errorIfAbsent) {
  removeStructure(CurveNetwork::structureTypeName, name, errorIfAbsent);
}


// =====================================================
// ============== Quantities
// =====================================================

template <class T>
CurveNetworkNodeColorQuantity* CurveNetwork::addNodeColorQuantity(std::string name, const T& colors) {
  validateSize(colors, nNodes(), "curve network node color quantity " + name);
  return addNodeColorQuantityImpl(name, standardizeVectorArray<glm::vec3, 3>(colors));
}

template <class T>
CurveNetworkEdgeColorQuantity* CurveNetwork::addEdgeColorQuantity(std::string name, const T& colors) {
  validateSize(colors, nEdges(), "curve network edge color quantity " + name);
  return addEdgeColorQuantityImpl(name, standardizeVectorArray<glm::vec3, 3>(colors));
}


template <class T>
CurveNetworkNodeScalarQuantity* CurveNetwork::addNodeScalarQuantity(std::string name, const T& data, DataType type) {
  validateSize(data, nNodes(), "curve network node scalar quantity " + name);
  return addNodeScalarQuantityImpl(name, standardizeArray<float, T>(data), type);
}

template <class T>
CurveNetworkEdgeScalarQuantity* CurveNetwork::addEdgeScalarQuantity(std::string name, const T& data, DataType type) {
  validateSize(data, nEdges(), "curve network edge scalar quantity " + name);
  return addEdgeScalarQuantityImpl(name, standardizeArray<float, T>(data), type);
}


template <class T>
CurveNetworkNodeVectorQuantity* CurveNetwork::addNodeVectorQuantity(std::string name, const T& vectors,
                                                                    VectorType vectorType) {
  validateSize(vectors, nNodes(), "curve network node vector quantity " + name);
  return addNodeVectorQuantityImpl(name, standardizeVectorArray<glm::vec3, 3>(vectors), vectorType);
}

template <class T>
CurveNetworkNodeVectorQuantity* CurveNetwork::addNodeVectorQuantity2D(std::string name, const T& vectors,
                                                                      VectorType vectorType) {
  validateSize(vectors, nNodes(), "curve network node vector quantity " + name);

  std::vector<glm::vec3> vectors3D(standardizeVectorArray<glm::vec3, 2>(vectors));
  for (auto& v : vectors3D) {
    v.z = 0.;
  }
  return addNodeVectorQuantityImpl(name, vectors3D, vectorType);
}


template <class T>
CurveNetworkEdgeVectorQuantity* CurveNetwork::addEdgeVectorQuantity(std::string name, const T& vectors,
                                                                    VectorType vectorType) {
  validateSize(vectors, nEdges(), "curve network edge vector quantity " + name);
  return addEdgeVectorQuantityImpl(name, standardizeVectorArray<glm::vec3, 3>(vectors), vectorType);
}

template <class T>
CurveNetworkEdgeVectorQuantity* CurveNetwork::addEdgeVectorQuantity2D(std::string name, const T& vectors,
                                                                      VectorType vectorType) {
  validateSize(vectors, nEdges(), "curve network edge vector quantity " + name);

  std::vector<glm::vec3> vectors3D(standardizeVectorArray<glm::vec3, 2>(vectors));
  for (auto& v : vectors3D) {
    v.z = 0.;
  }
  return addEdgeVectorQuantityImpl(name, vectors3D, vectorType);
}

// === Covariance 6x6 API (RECOMMENDED) ===

template <class T>
CurveNetworkNodeCovarianceQuantity* CurveNetwork::addNodeCovariance6x6Quantity(std::string name,
                                                                               const T& covariances6x6) {
  validateSize(covariances6x6, nNodes(), "curve network node covariance6x6 quantity " + name);

  // Extract 3x3 blocks from 6x6 matrices
  std::vector<glm::mat3> posCovs;
  std::vector<glm::mat3> rotCovs;
  std::vector<glm::mat3> poseRots;

  posCovs.reserve(covariances6x6.size());
  rotCovs.reserve(covariances6x6.size());
  poseRots.reserve(covariances6x6.size());

  for (const auto& cov6x6 : covariances6x6) {
    posCovs.push_back(CurveNetworkNodeCovarianceQuantity::extractPositionCovariance(cov6x6));
    rotCovs.push_back(CurveNetworkNodeCovarianceQuantity::extractRotationCovariance(cov6x6));
    poseRots.push_back(glm::mat3{1.0f}); // Identity - covariances assumed in world frame
  }

  CurveNetworkNodeCovarianceQuantity* q =
      new CurveNetworkNodeCovarianceQuantity(name, *this, posCovs, rotCovs, poseRots);
  addQuantity(q);
  return q;
}

template <class TCov, class TPose>
CurveNetworkNodeCovarianceQuantity*
CurveNetwork::addNodeCovariance6x6Quantity(std::string name, const TCov& covariances6x6, const TPose& poses) {
  validateSize(covariances6x6, nNodes(), "curve network node covariance6x6 quantity " + name);
  validateSize(poses, nNodes(), "curve network node covariance6x6 quantity poses " + name);

  // Extract 3x3 blocks from 6x6 matrices and pose rotations
  std::vector<glm::mat3> posCovs;
  std::vector<glm::mat3> rotCovs;
  std::vector<glm::mat3> poseRots;

  posCovs.reserve(covariances6x6.size());
  rotCovs.reserve(covariances6x6.size());
  poseRots.reserve(poses.size());

  for (const auto& cov6x6 : covariances6x6) {
    posCovs.push_back(CurveNetworkNodeCovarianceQuantity::extractPositionCovariance(cov6x6));
    rotCovs.push_back(CurveNetworkNodeCovarianceQuantity::extractRotationCovariance(cov6x6));
  }

  for (const auto& pose : poses) {
    poseRots.push_back(CurveNetworkNodeCovarianceQuantity::extractPoseRotation(pose));
  }

  CurveNetworkNodeCovarianceQuantity* q =
      new CurveNetworkNodeCovarianceQuantity(name, *this, posCovs, rotCovs, poseRots);
  addQuantity(q);
  return q;
}

// === Advanced covariance API (3x3 blocks) ===

template <class T1, class T2>
CurveNetworkNodeCovarianceQuantity* CurveNetwork::addNodeCovarianceQuantity(std::string name,
                                                                            const T1& positionCovariances,
                                                                            const T2& rotationCovariances) {
  validateSize(positionCovariances, nNodes(), "curve network node covariance quantity " + name);
  validateSize(rotationCovariances, nNodes(), "curve network node covariance quantity " + name);

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

  CurveNetworkNodeCovarianceQuantity* q = new CurveNetworkNodeCovarianceQuantity(name, *this, posCovs, rotCovs);
  addQuantity(q);
  return q;
}

template <class T1, class T2, class T3>
CurveNetworkNodeCovarianceQuantity*
CurveNetwork::addNodeCovarianceQuantity(std::string name, const T1& positionCovariances, const T2& rotationCovariances,
                                        const T3& poseRotations) {
  validateSize(positionCovariances, nNodes(), "curve network node covariance quantity " + name);
  validateSize(rotationCovariances, nNodes(), "curve network node covariance quantity " + name);

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

  CurveNetworkNodeCovarianceQuantity* q =
      new CurveNetworkNodeCovarianceQuantity(name, *this, posCovs, rotCovs, poseRots);
  addQuantity(q);
  return q;
}


} // namespace polyscope
