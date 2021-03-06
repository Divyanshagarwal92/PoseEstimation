#include <iostream>
#include <fstream>
#include <string>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues> 


using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

// Pretty much regulates the logging.
bool DEBUG_FLAG = false;


class PoseSolver {
  public:

PoseSolver() {
  K = MatrixXd::Zero(3,3);
}

/* @brief load data from the txt file. This data includes 3D-2D correspondences and
 *   camera matrix
 * @param[in]: fname: Filename containing data
 */
int loadData(string fname) {
  ifstream myfile(fname);
  int numPoints;
  if (myfile.is_open()) {
    string tmp;
    myfile >> tmp >> K(0,0) >> K(1,1) >> K(0,2) >> K(1,2);
    myfile >> tmp >> numPoints;
    world_points.resize(numPoints, 3);
    image_points.resize(numPoints, 2);
    for (int i = 0; i < numPoints; i++) {
      myfile >> world_points(i,0) >> world_points(i,1) >> world_points(i,2) 
             >> image_points(i,0) >> image_points(i,1);
    }
    myfile.close();
  }
  K(2,2) = 1;
  if (world_points.rows() != image_points.rows())
    numPoints = -1;
  return numPoints;

}

/* @brief: Show Camera Intrinsics
 */
void showIntrinsics() {
  cout << "Intrinsics:" << endl;
  cout << K << endl;
}

/* @brief: Show 3D-2D correspondeces
 */
void show3Dto2DCorrespondence() {
  cout << "World Pts:" << endl;
  cout << world_points << endl;
  cout << "Image Pts:" << endl;
  cout << image_points << endl;
}


/* @brief: Given 2D-3D correspondece, estimate the projection matrix.
 * @param[out]: projextion_mat: Estimated projection matrix
 */
void getProjectionMatrix(MatrixXd& projection_mat) {
  // Rearange the terms to form the 2n x 12 matrix.
  int numPoints = world_points.rows();
  MatrixXd data(2*numPoints,12);
  for (int i = 0; i < numPoints; i++) {
    data.row(2*i) << world_points(i,0), world_points(i,1), world_points(i,2), 1, 0, 0, 0, 0,
                     -image_points(i,0) * world_points(i,0), -image_points(i,0) * world_points(i,1),
                     -image_points(i,0) * world_points(i,2), -image_points(i,0);
    data.row(2*i + 1) << 0, 0, 0, 0, world_points(i,0), world_points(i,1), world_points(i,2), 1,
                         -image_points(i,1) * world_points(i,0), -image_points(i,1) * world_points(i,1),
                         -image_points(i,1) * world_points(i,2), -image_points(i,1);
  }

  // Solved the constrained LSE. Done by doing an eigen analysis on
  // data.transpose * data. The eigen vector corresponding to the smallest
  // eigen value is the projection matrix.
  Eigen::EigenSolver<MatrixXd> es(data.transpose() * data);

  // Find the eigen vector correspond to smallest eigen value.
  std::ptrdiff_t index;
  auto lambda = es.eigenvalues().real().minCoeff(&index);
  auto min_vec = es.eigenvectors().col(index).real();
  VectorXd sc_min_vec = lambda * min_vec;

  if (DEBUG_FLAG) {
    cout << "Lambda:" << lambda << " location: " << index << endl;
    cout << "lambda * min_vec:\n" << sc_min_vec << endl;
  }

  projection_mat = Eigen::Map<MatrixXd>(sc_min_vec.data(), 4, 3).transpose();
}

/* @brief: Reconstruct the rotation and translation matrix from the projection matrix,
 * @param[in] proj_mat: Projection Matrix (3x4)
 * @param[out] rotation: Rotation Matrix of the estimated 3D pose (3x3)
 * @param[out] translation: ranslation Matrix of the estimated 3D pose (3x3)
 */
void getPose(const MatrixXd& proj_mat,
             MatrixXd& rotation, MatrixXd& translation) {
  MatrixXd first_chunk = proj_mat.block(0,0,3,3);
  MatrixXd second_chunk = proj_mat.block(0,3,3,1);

  rotation = K.inverse()*first_chunk;
  translation = K.inverse()*second_chunk;
}
 private:
  MatrixXd K;
  MatrixXd world_points;
  MatrixXd image_points;
};

int main(int argc, char** argv) {

  if (argc != 2) {
    cout << "Invalid input.\n Usage: <pose_estimation> <data_file.txt>";
  }
  PoseSolver handle;
  string fname = argv[1];

  // Load data from the file, say n points
  int numPoints = handle.loadData(fname);

  if (numPoints < 6 || numPoints == -1) {
    cout << "Pose can not be estimated\n";
    return 0;
  }

  if (DEBUG_FLAG) {
  // Print the loaded data for sanity.
    handle.showIntrinsics();
    handle.show3Dto2DCorrespondence();
  }

  MatrixXd proj_mat;
  handle.getProjectionMatrix(proj_mat);
  if (DEBUG_FLAG) {
    cout << "Projection Matrix:\n" << proj_mat << endl;
  }

  // P is K[R|t], t = K-inv(last col of P), R = K-inv(first 3 cols of P)
  MatrixXd rotation, translation;
  handle.getPose(proj_mat, rotation, translation);
  cout << "Rotation matrix:\n" << rotation << endl;
  cout << "Translation matrix:\n" << translation << endl;

  return 1;
}
