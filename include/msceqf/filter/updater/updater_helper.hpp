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
#include "msceqf/system/system.hpp"

namespace msceqf
{

using MatrixXBlockRowRef = Eigen::Ref<MatrixX::RowsBlockXpr>;  //!< Block row reference for a dynamic matrix
using VectorXBlockRowRef = Eigen::Ref<VectorX::RowsBlockXpr>;  //!< Block row reference for a dynamic vector

/**
 * @brief FeatHelper struct.
 * This struct implements a helper structure holding all the information related to a single feature measurement to be
 * used in the MSCEqF update.
 *
 * @note The clone pointer is passed by reference because there is no need to share ownership of the clone.
 */
struct FeatHelper
{
  FeatHelper(const Vector3& A_f,
             const Vector2& uv,
             const Vector2& uvn,
             const MSCEqFSE3StateSharedPtr& anchor,
             const MSCEqFSE3StateSharedPtr& clone,
             const fp& timestamp)
      : A_f_(A_f), uv_(uv), uvn_(uvn), anchor_(anchor), clone_(clone), timetamp_(timestamp){};

  const Vector3& A_f_;                     //!< Triangulated feature in anchor frame
  const Vector2& uv_;                      //!< (measured) feature coordinates
  const Vector2& uvn_;                     //!< Normalized (measured) feature coordinates
  const MSCEqFSE3StateSharedPtr& anchor_;  //!< Anchor
  const MSCEqFSE3StateSharedPtr& clone_;   //!< Clone of the state at the time of the feature measurement
  const fp& timetamp_;                     //!< Timestamp of the feature measurement
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
   * @return Vector3
   */
  [[nodiscard]] virtual Vector3 pi(const Vector3& f) = 0;

  /**
   * @brief Projection differential function. This function computes the differential of the projection function.
   *
   * @param f
   * @return MatrixX
   */
  [[nodiscard]] virtual MatrixX dpi(const Vector3& f) = 0;

  /**
   * @brief Computes a block row of the C matrix and a block of the residual, corresponding to the given feature
   * This method compute the Ct matrix, the Cf matrix and the residual for a given feature. Perform the nullspace
   * projection and assign the result to the given block of C matrix, and the residual, after performing a chi2
   * rejection test.
   *
   * @param X MSCEqF state
   * @param feat Feature helper
   * @param C_block_row Block row of the C matrix
   * @param delta_block_row Block of the residual delta
   * @param Cf_block_row Block of the Cf matrix
   */
  virtual void innovationBlock(const MSCEqFState& X,
                               const SystemState& xi0,
                               const FeatHelper& feat,
                               MatrixXBlockRowRef C_block_row,
                               VectorXBlockRowRef delta_block_row,
                               MatrixXBlockRowRef Cf_block_row) = 0;

  size_t block_rows_;  //!< Number of rows of a C matrix block and a residual block

 protected:
  /// Rule of 5
  ProjectionHelper(const FeatureRepresentation& feature_representation)
      : feature_representation_(feature_representation)
  {
  }
  ProjectionHelper(const ProjectionHelper&) = default;
  ProjectionHelper(ProjectionHelper&&) = default;
  ProjectionHelper& operator=(const ProjectionHelper&) = default;
  ProjectionHelper& operator=(ProjectionHelper&&) = default;

  FeatureRepresentation feature_representation_;  //!< Feature representation
};

/**
 * @brief ProjectionHelperS2 class.
 * This class provides an implementation of the projection on hte unit sphere as well as its differential.
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
   * @return Vector3
   */
  [[nodiscard]] Vector3 pi(const Vector3& f) override;

  /**
   * @brief Projection differential function. This function computes the differential of the projection function.
   *
   * @param f
   * @return MatrixX
   */
  [[nodiscard]] MatrixX dpi(const Vector3& f) override;

  /**
   * @brief Computes a block row of the C matrix and a block of the residual, corresponding to the given feature
   * This method compute the Ct matrix, the Cf matrix and the residual for a given feature. Perform the nullspace
   * projection and assign the result to the given block of C matrix, and the residual, after performing a chi2
   * rejection test.
   *
   * @param X MSCEqF state
   * @param feat Feature helper
   * @param C_block_row Block row of the C matrix
   * @param delta_block_row Block of the residual delta
   * @param Cf_block_row Block of the Cf matrix
   */
  void innovationBlock(const MSCEqFState& X,
                       const SystemState& xi0,
                       const FeatHelper& feat,
                       MatrixXBlockRowRef C_block_row,
                       VectorXBlockRowRef delta_block_row,
                       MatrixXBlockRowRef Cf_block_row) override;
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
   * @return Vector3
   */
  [[nodiscard]] Vector3 pi(const Vector3& f) override;

  /**
   * @brief Projection differential function. This function computes the differential of the projection function.
   *
   * @param f
   * @return MatrixX
   */
  [[nodiscard]] MatrixX dpi(const Vector3& f) override;

  /**
   * @brief Computes a block row of the C matrix and a block of the residual, corresponding to the given feature
   * This method compute the Ct matrix, the Cf matrix and the residual for a given feature. Perform the nullspace
   * projection and assign the result to the given block of C matrix, and the residual, after performing a chi2
   * rejection test.
   *
   * @param X MSCEqF state
   * @param feat Feature helper
   * @param C_block_row Block row of the C matrix
   * @param delta_block_row Block of the residual delta
   * @param Cf_block_row Block of the Cf matrix
   */
  void innovationBlock(const MSCEqFState& X,
                       const SystemState& xi0,
                       const FeatHelper& feat,
                       MatrixXBlockRowRef C_block_row,
                       VectorXBlockRowRef delta_block_row,
                       MatrixXBlockRowRef Cf_block_row) override;
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
   * @return Eigen::Matrix<fp, 2, 4>
   */
  [[nodiscard]] static Eigen::Matrix<fp, 2, 4> Xi(const Vector3& f);

  /**
   * @brief Compute the Jacobian for inverse depth parametrization, used in the Cf matrix
   *
   * @param A_f Given feature in the anchor frame
   * @return Matrix3
   */
  [[nodiscard]] static Matrix3 inverseDepthJacobian(const Vector3& A_f);

  /**
   * @brief Perform in-place nullspace projection of the Cf matrix on the Ct matrix and the residual using QR
   * decomposition
   *
   * @param Ct C matrix
   * @param delta residual
   * @param Cf Cf matrix
   */
  static void nullspaceProjection(Eigen::Ref<MatrixX> Cf, MatrixXBlockRowRef Ct, VectorXBlockRowRef delta);

  /**
   * @brief Perform in-place compression of the C matrix and the residual using QR decomposition
   *
   * @param C C matrix
   * @param delta residual
   * @param R R matrix
   */
  static void updateQRCompression(MatrixX& C, VectorX& delta, MatrixX& R);

  /**
   * @brief Perform chi2 test (based on precomputed table) on the given block of the residual
   *
   * @param X MSCEqF state
   * @param Ct_block Block of the Ct matrix
   * @param delta_block Block of the residual delta
   * @param pixel_std Standard deviation of the pixel noise
   * @param chi2_table Chi2 table (precomputed)
   * @return true if test passed, false otherwise
   */
  [[nodiscard]] static bool chi2Test(const MSCEqFState& X,
                                     const MatrixXBlockRowRef Ct_block,
                                     const VectorXBlockRowRef delta_block,
                                     const fp& pixel_std,
                                     const std::map<uint, fp>& chi2_table);
};

}  // namespace msceqf

#endif  // UPDATER_HELPER_HPP