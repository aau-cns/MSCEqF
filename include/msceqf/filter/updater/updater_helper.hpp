// Copyright (C) 2023 Alessandro Fornasier.
// Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the authors at <alessandro.fornasier@ieee.org>

#ifndef UPDATER_HELPER_HPP
#define UPDATER_HELPER_HPP

#include <opencv2/opencv.hpp>
#include <functional>
#include <boost/math/distributions/chi_squared.hpp>

#include "types/fptypes.hpp"
#include "msceqf/state/state.hpp"
#include "utils/tools.hpp"

namespace msceqf
{
using MatrixXBlockRowRef = Eigen::Ref<MatrixX::RowsBlockXpr>;  //!< Block row reference for a dynamic matrix
using VectorXBlockRowRef = Eigen::Ref<VectorX::RowsBlockXpr>;  //!< Block row reference for a dynamic vector

using ColsMap = utils::InsertionOrderedMap<MSCEqFState::MSCEqFKey, size_t>;  //!< Map of indices for C and delta

/**
 * @brief FeatHelper struct.
 * This struct implements a helper structure holding all the information related to a single feature measurement to
 * be used in the computation of the C matrix, Cf matrix and residual delta, for the MSCEqF update.
 */
struct FeatHelper
{
  FeatHelper(
      const Vector3& A_f, const Vector2& uv, const Vector2& uvn, const fp& anchor_timestamp, const fp& clone_timestamp)
      : A_f_(A_f), uv_(uv), uvn_(uvn), anchor_timestamp_(anchor_timestamp), clone_timestamp_(clone_timestamp){};

  const Vector3& A_f_;          //!< Triangulated feature in anchor frame
  const Vector2& uv_;           //!< (measured) feature coordinates
  const Vector2& uvn_;          //!< Normalized (measured) feature coordinates
  const fp& anchor_timestamp_;  //!< Timestamp of the anchor
  const fp& clone_timestamp_;   //!< Timestamp of the feature measurement
};

/**
 * @brief ProjectionHelper interface.
 * This class provides an interface to the implementation of the projection function as well as its differential.
 *
 */
class ProjectionHelper
{
 public:
  virtual ~ProjectionHelper() = default;

  /**
   * @brief Projection function. This function projects a 3D point.
   *
   * @param f
   * @return R3 vector representing a 3D point
   */
  [[nodiscard]] virtual Vector3 pi(const Vector3& f) = 0;

  /**
   * @brief Projection differential function. This function computes the differential of the projection function.
   *
   * @param f
   * @return Differential of the projection function
   */
  [[nodiscard]] virtual MatrixX dpi(const Vector3& f) = 0;

  /**
   * @brief Computes a block row of the C matrix and a block of the residual, corresponding to the given feature
   * This method compute the Ct matrix, the Cf matrix and the residual for a given feature.
   *
   * @param X MSCEqF state
   * @param feat Feature helper
   * @param C_block_row Block row of the C matrix
   * @param delta_block_row Block of the residual delta
   * @param Cf_block_row Block of the Cf matrix
   * @param cols_map Map of indices for the C matrix and the residual delta
   */
  virtual void residualJacobianBlock(const MSCEqFState& X,
                                     const SystemState& xi0,
                                     const FeatHelper& feat,
                                     MatrixXBlockRowRef C_block_row,
                                     VectorXBlockRowRef delta_block_row,
                                     MatrixXBlockRowRef Cf_block_row,
                                     const ColsMap& cols_map) = 0;

  /**
   * @brief Get the number of rows of a C matrix block and a residual block
   *
   * @return rows of a single block of the C matrix and the residual
   */
  [[nodiscard]] const size_t& block_rows() const { return block_rows_; }

  /**
   * @brief Get the dimension lost due to nullspace projection
   *
   * @return dimension lost due to nullspace projection
   */
  [[nodiscard]] const size_t& dim_loss() const { return dim_loss_; }

 protected:
  /// Rule of 5
  ProjectionHelper(const FeatureRepresentation& feature_representation);
  ProjectionHelper(const ProjectionHelper&) = default;
  ProjectionHelper(ProjectionHelper&&) = default;
  ProjectionHelper& operator=(const ProjectionHelper&) = default;
  ProjectionHelper& operator=(ProjectionHelper&&) = default;

  FeatureRepresentation feature_representation_;  //!< Feature representation

  size_t block_rows_;  //!< Number of rows of a C matrix block and a residual block
  size_t dim_loss_;    //!< Dimension lost due to nullspace projection
};

/**
 * @brief ProjectionHelperS2 class.
 * This class provides an implementation of the projection on the unit sphere as well as its differential.
 *
 */
class ProjectionHelperS2 : public ProjectionHelper
{
 public:
  ProjectionHelperS2(const FeatureRepresentation& feature_representation) : ProjectionHelper(feature_representation)
  {
    block_rows_ = 3;
  }

  /**
   * @brief Projection function. This function projects a 3D point on the unit sphere.
   *
   * @param f
   * @return R3 vector representing a 3D point on the unit sphere
   */
  [[nodiscard]] Vector3 pi(const Vector3& f) override;

  /**
   * @brief Projection differential function. This function computes the differential of the projection function.
   *
   * @param f
   * @return Differential of the S2 projection function
   */
  [[nodiscard]] MatrixX dpi(const Vector3& f) override;

  /**
   * @brief Computes a block row of the C matrix and a block of the residual, corresponding to the given feature
   * This method compute the Ct matrix, the Cf matrix and the residual for a given feature.
   *
   * @param X MSCEqF state
   * @param feat Feature helper
   * @param C_block_row Block row of the C matrix
   * @param delta_block_row Block of the residual delta
   * @param Cf_block_row Block of the Cf matrix
   * @param cols_map Map of indices for the C matrix and the residual delta
   */
  void residualJacobianBlock(const MSCEqFState& X,
                             const SystemState& xi0,
                             const FeatHelper& feat,
                             MatrixXBlockRowRef C_block_row,
                             VectorXBlockRowRef delta_block_row,
                             MatrixXBlockRowRef Cf_block_row,
                             const ColsMap& cols_map) override;
};

/**
 * @brief ProjectionHelperZ1 class.
 * This class provides an implementation of the projection on the unit plane as well as its differential.
 *
 */
class ProjectionHelperZ1 : public ProjectionHelper
{
 public:
  ProjectionHelperZ1(const FeatureRepresentation& feature_representation) : ProjectionHelper(feature_representation)
  {
    block_rows_ = 2;
  }

  /**
   * @brief Projection function. This function projects a 3D point on the unit plane.
   *
   * @param f
   * @return R2 vector representing a 3D point on the unit plane
   */
  [[nodiscard]] Vector3 pi(const Vector3& f) override;

  /**
   * @brief Projection differential function. This function computes the differential of the projection function.
   *
   * @param f
   * @return Differential of the Z1 projection function
   */
  [[nodiscard]] MatrixX dpi(const Vector3& f) override;

  /**
   * @brief Computes a block row of the C matrix and a block of the residual, corresponding to the given feature
   * This method compute the Ct matrix, the Cf matrix and the residual for a given feature.
   *
   * @param X MSCEqF state
   * @param feat Feature helper
   * @param C_block_row Block row of the C matrix
   * @param delta_block_row Block of the residual delta
   * @param Cf_block_row Block of the Cf matrix
   * @param cols_map Map of indices for the C matrix and the residual delta
   */
  void residualJacobianBlock(const MSCEqFState& X,
                             const SystemState& xi0,
                             const FeatHelper& feat,
                             MatrixXBlockRowRef C_block_row,
                             VectorXBlockRowRef delta_block_row,
                             MatrixXBlockRowRef Cf_block_row,
                             const ColsMap& cols_map) override;
};

using ProjectionHelperSharedPtr = std::shared_ptr<ProjectionHelper>;
using ProjectionHelperUniquePtr = std::unique_ptr<ProjectionHelper>;
using ProjectionHelperZ1SharedPtr = std::shared_ptr<ProjectionHelperZ1>;
using ProjectionHelperZ1UniquePtr = std::unique_ptr<ProjectionHelperZ1>;
using ProjectionHelperS2SharedPtr = std::shared_ptr<ProjectionHelperS2>;
using ProjectionHelperS2UniquePtr = std::unique_ptr<ProjectionHelperS2>;

/**
 * @brief Factory method for ProjectionHelper
 *
 * @tparam T
 * @return ProjectionHelperUniquePtr
 */
template <typename T>
[[nodiscard]] static ProjectionHelperUniquePtr createProjectionHelper(
    const FeatureRepresentation& feature_representation)
{
  if constexpr (std::is_base_of_v<ProjectionHelper, T>)
  {
    return std::make_unique<T>(feature_representation);
  }
  else
  {
    return nullptr;
  }
}

/**
 * @brief Updater helper struct.
 * This structs implements common helper methods for MSCEqF update.
 *
 */
struct UpdaterHelper
{
  /**
   * @brief Xi operator R^3 -> R^2x4
   *
   * @param f
   * @return Matrix representing the Xi operator
   */
  [[nodiscard]] static Eigen::Matrix<fp, 2, 4> Xi(const Vector3& f);

  /**
   * @brief Compute the Jacobian for inverse depth parametrization, used in the Cf matrix
   *
   * @param A_f Given feature in the anchor frame
   * @return Jacobian matrix for inverse depth parametrization
   */
  [[nodiscard]] static Matrix3 inverseDepthJacobian(const Vector3& A_f);

  /**
   * @brief Perform in-place nullspace projection of the Cf matrix on the Ct matrix and the residual using QR
   * decomposition
   *
   * @param Ct C matrix
   * @param delta Residual
   * @param Cf Cf matrix
   */
  static void nullspaceProjection(Eigen::Ref<MatrixX> Cf, MatrixXBlockRowRef Ct, VectorXBlockRowRef delta);

  /**
   * @brief Perform in-place compression of the C matrix and the residual using QR decomposition
   *
   * @param C C matrix
   * @param delta Residual
   * @param R R matrix
   */
  static void updateQRCompression(MatrixX& C, VectorX& delta, MatrixX& R);

  /**
   * @brief Perform chi2 test (based on precomputed table) on the given block of the residual
   *
   * @param chi2 Chi2 value
   * @param dof Degrees of freedom
   * @param chi2_table Precomputed chi2 table
   * @return true if test passed, false otherwise
   */
  [[nodiscard]] static bool chi2Test(const fp& chi2, const size_t& dof, const std::map<uint, fp>& chi2_table);
};

}  // namespace msceqf

#endif  // UPDATER_HELPER_HPP