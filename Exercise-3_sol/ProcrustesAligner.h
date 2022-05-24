#pragma once
#include "SimpleMesh.h"

class ProcrustesAligner {
public:
	Matrix4f estimatePose(const std::vector<Vector3f>& sourcePoints, const std::vector<Vector3f>& targetPoints) {
		ASSERT(sourcePoints.size() == targetPoints.size() && "The number of source and target points should be the same, since every source point is matched with corresponding target point.");

		// We estimate the pose between source and target points using Procrustes algorithm.
		// Our shapes have the same scale, therefore we don't estimate scale. We estimated rotation and translation
		// from source points to target points.

		auto sourceMean = computeMean(sourcePoints);
		auto targetMean = computeMean(targetPoints);
		
		Matrix3f rotation = estimateRotation(sourcePoints, sourceMean, targetPoints, targetMean);
		Vector3f translation = computeTranslation(sourceMean, targetMean, rotation);

		// TODO: Compute the transformation matrix by using the computed rotation and translation.
		// You can access parts of the matrix with .block(start_row, start_col, num_rows, num_cols) = elements
		Matrix4f estimatedPose = Matrix4f::Identity();
		estimatedPose.block(0, 0, 3, 3) = rotation;
		estimatedPose.block(0, 3, 3, 1) = translation;
		return estimatedPose;
	}

private:
	Vector3f computeMean(const std::vector<Vector3f>& points) {
		// TODO: Compute the mean of input points.
		// Hint: You can use the .size() method to get the length of a vector.
		Vector3f mean = Vector3f::Zero();
		auto numPoints = points.size();
		for (int i = 0; i < numPoints; i++)
		{
			mean += points[i];
		}
		return mean / numPoints;
	}

	Matrix3f estimateRotation(const std::vector<Vector3f>& sourcePoints, const Vector3f& sourceMean, const std::vector<Vector3f>& targetPoints, const Vector3f& targetMean) {
		// TODO: Estimate the rotation from source to target points, following the Procrustes algorithm.
		// To compute the singular value decomposition you can use JacobiSVD() from Eigen.
		// Hint: You can initialize an Eigen matrix with "MatrixXf m(num_rows,num_cols);" and access/modify parts of it using the .block() method (see above).
		auto nPoints = sourcePoints.size();
		MatrixXf sourceMatrix(nPoints, 3);
		MatrixXf targetMatrix(nPoints, 3);
		for (int i = 0; i < nPoints; ++i) 
		{
			sourceMatrix.block(i, 0, 1, 3) = (sourcePoints[i] - sourceMean).transpose();
			targetMatrix.block(i, 0, 1, 3) = (targetPoints[i] - targetMean).transpose();
		}
		JacobiSVD<Matrix3f> svd(targetMatrix.transpose() * sourceMatrix, ComputeFullV | ComputeFullU);
		Matrix3f U = svd.matrixU();
		Matrix3f V = svd.matrixV();
		float d = (U * V.transpose()).determinant();
		Matrix3f D = Matrix3f::Identity();
		D(2, 2) = d;
		return U * D * V.transpose();
	}

	Vector3f computeTranslation(const Vector3f& sourceMean, const Vector3f& targetMean, const Matrix3f& rotation) {
		// TODO: Compute the translation vector from source to target points.
		return targetMean - rotation * sourceMean;
	}
};
