#pragma once

#include <cassert>
#include <vector>

#include <tf/transform_broadcaster.h>

static const double EYE4[]={1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1};
static const unsigned int SIZE_H=16;

static const double EYE3[]={1,0,0, 0,1,0, 0,0,1};
static const unsigned int SIZE_R=9;

void mulHMatrixVector(std::vector<double>& res, const std::vector<double>& matrix, const std::vector<double>& vector)
{
  assert(res.size() == 4 && matrix.size() == 16 && (vector.size() == 3 || vector.size() == 4));
  res[0] = matrix[0]*vector[0] + matrix[1]*vector[1] + matrix[2]*vector[2] + matrix[3];
  res[1] = matrix[4]*vector[0] + matrix[5]*vector[1] + matrix[6]*vector[2] + matrix[7];
  res[2] = matrix[8]*vector[0] + matrix[9]*vector[1] + matrix[10]*vector[2] + matrix[11];
  res[3] = 1;
}

void mulMatrixVector(double* res, const std::vector<double>& matrix, const std::vector<double>& vector, const double columns, const double rows)
{
  assert(matrix.size() == (columns*rows) && vector.size() == columns);
  for(unsigned int i = 0; i < rows; ++i)
  {
    double sum = 0.0;
    for(unsigned int j = 0; j < columns; ++j)
    {
      sum += matrix[i*columns + j] * vector[j];
    }
    res[i] = sum;
  }
}



//@todo Remove ROS stuff
void makeTransform(std::vector<double>& hmatrix, geometry_msgs::TransformStamped::_transform_type& trans)
{
  assert(hmatrix.size() == 16);
  trans.translation.x = hmatrix[3];
  trans.translation.y = hmatrix[7];
  trans.translation.z = hmatrix[11];

  btMatrix3x3 rotMatrix(hmatrix[0], hmatrix[1], hmatrix[2], hmatrix[4], hmatrix[5], hmatrix[6], hmatrix[8],
      hmatrix[9], hmatrix[10]);
  tf::Quaternion quat;
  rotMatrix.getRotation(quat);

  trans.rotation.w = quat.getW();
  trans.rotation.x = quat.getX();
  trans.rotation.y = quat.getY();
  trans.rotation.z = quat.getZ();
}


//@todo Remove ROS stuff
void makeTransform(std::vector<double>& hmatrix, tf::Transform& trans)
{
  assert(hmatrix.size() == 16);
  trans.setOrigin(tf::Vector3(hmatrix[3], hmatrix[7], hmatrix[11]));
  btMatrix3x3 rotMatrix(hmatrix[0], hmatrix[1], hmatrix[2], hmatrix[4], hmatrix[5], hmatrix[6], hmatrix[8],
      hmatrix[9], hmatrix[10]);
  tf::Quaternion quat;
  rotMatrix.getRotation(quat);
  trans.setRotation(quat);
}

enum ROT_STYLE {ROT_X = 0, ROT_Y = 1, ROT_Z = 2, EYE = 3};

template<int TYPE>
void HomogeneousMatrix(std::vector<double>& HMatrix, const double rot_angle, const std::vector<double>& p);

// Note: Flat matrices can be shown in Orocos directly.
template<>
void HomogeneousMatrix<ROT_X>(std::vector<double>& hmatrix, const double rot_angle, const std::vector<double>& p)
{
  assert(hmatrix.size() == 16 && (p.size() == 3 || p.size() == 4));
  hmatrix[0]  = 1; hmatrix[1] = 0;              hmatrix[2]  = 0;               hmatrix[3]  = p[0];
  hmatrix[4]  = 0; hmatrix[5] = cos(rot_angle); hmatrix[6]  = -sin(rot_angle); hmatrix[7]  = p[1];
  hmatrix[8]  = 0; hmatrix[9] = sin(rot_angle); hmatrix[10] = cos(rot_angle);  hmatrix[11] = p[2];
  hmatrix[12] = 0; hmatrix[13] = 0;             hmatrix[14] = 0;               hmatrix[15] = 1;

}

template<>
void HomogeneousMatrix<ROT_Y>(std::vector<double>& hmatrix, const double rot_angle, const std::vector<double>& p)
{
  assert(hmatrix.size() == 16 && (p.size() == 3 || p.size() == 4));
  hmatrix[0]  = cos(rot_angle);  hmatrix[1]  = 0; hmatrix[2]  = sin(rot_angle); hmatrix[3]  = p[0];
  hmatrix[4]  = 0;               hmatrix[5]  = 1; hmatrix[6]  = 0;              hmatrix[7]  = p[1];
  hmatrix[8]  = -sin(rot_angle); hmatrix[9]  = 0; hmatrix[10] = cos(rot_angle); hmatrix[11] = p[2];
  hmatrix[12] = 0;               hmatrix[13] = 0; hmatrix[14] = 0;              hmatrix[15] = 1;
}


template<>
void HomogeneousMatrix<ROT_Z>(std::vector<double>& hmatrix, const double rot_angle, const std::vector<double>& p)
{
  assert(hmatrix.size() == 16 && (p.size() == 3 || p.size() == 4));
  hmatrix[0]  = cos(rot_angle); hmatrix[1]  = -sin(rot_angle); hmatrix[2]   = 0; hmatrix[3]  = p[0];
  hmatrix[4]  = sin(rot_angle); hmatrix[5]  = cos(rot_angle);  hmatrix[6]   = 0; hmatrix[7]  = p[1];
  hmatrix[8]  = 0;              hmatrix[9]  = 0;               hmatrix[10]  = 1; hmatrix[11] = p[2];
  hmatrix[12] = 0;              hmatrix[13] = 0;               hmatrix[14]  = 0; hmatrix[15] = 1;
}

template<>
void HomogeneousMatrix<EYE>(std::vector<double>& hmatrix, const double rot_angle, const std::vector<double>& p)
{
  assert(hmatrix.size() == 16 && (p.size() == 3 || p.size() == 4));
  hmatrix[0]  = 1; hmatrix[1]  = 0; hmatrix[2]   = 0; hmatrix[3]  = p[0];
  hmatrix[4]  = 0; hmatrix[5]  = 1; hmatrix[6]   = 0; hmatrix[7]  = p[1];
  hmatrix[8]  = 0; hmatrix[9]  = 0; hmatrix[10]  = 1; hmatrix[11] = p[2];
  hmatrix[12] = 0; hmatrix[13] = 0; hmatrix[14]  = 0; hmatrix[15] = 1;
}

void mulMatrixMatrixSquare(std::vector<double>& res, const std::vector<double>& m1, const std::vector<double>& m2, const double dimension)
{
  assert(res.size() == m1.size() && m1.size() == m2.size() && m2.size() == (dimension*dimension));

  for (int i = 0; i < dimension; ++i)
  {
      for (int j = 0; j < dimension; ++j)
      {
          double sum = 0;
          for (int k = 0; k < dimension; ++k)
          {
              sum += m1[i*dimension + k] * m2[k*dimension + j];
          }
          res[i*dimension + j] = sum;
      }
  }
}
inline std::vector<double> operator+	(const std::vector<double>& lhs, const std::vector<double>& rhs){
	assert(lhs.size() == rhs.size() );
	std::vector<double> result(lhs.size(), 0.0);
	for(unsigned int i =0; i < lhs.size(); ++i)
		result[i] =lhs[i]+rhs[i];
	return result;
}

inline std::vector<double> operator-	(const std::vector<double>& lhs, const std::vector<double>& rhs){
	assert(lhs.size() == rhs.size() );
	std::vector<double> result(lhs.size(), 0.0);
	for(unsigned int i = 0; i < lhs.size(); ++i)
		result[i] =lhs[i]-rhs[i];
	return result;
}

inline std::vector<double> operator*	(const std::vector<double>& lhs, const double& rhs){
  std::vector<double> result(lhs.size(), 0.0);
	for(unsigned int i =0; i < lhs.size(); ++i)
		result[i] =lhs[i]*rhs;
	return result;
}

inline double operator*	(const std::vector<double>& lhs, const std::vector<double>& rhs){
	assert(lhs.size() == rhs.size() );
	double result;
	for(unsigned int i = 0; i<lhs.size(); ++i)
		result+=lhs[i]*rhs[i];
	return result;
}

void matrixTranspose (std::vector<double>& res, std::vector<double>& source, int rows,int columns )
{
	for(int i = 0; i < rows; i++)
		for( int j = 0; j < columns; j++)
			res[i * columns + j] = source[ j * rows + i];
}

void makeTilde(std::vector<double>& tilde, std::vector<double>& p)
{
	 assert(tilde.size() == 16 && p.size() == 6);

	 tilde[0] = 0;		tilde[1] = -p[2];	tilde[2] = p[1];	tilde[3] = p[3];
	 tilde[4] = p[2];	tilde[5] = 0;		tilde[6] = -p[0];	tilde[7] = p[4];
	 tilde[8] = -p[1];  tilde[9] = p[0];	tilde[10] = 0;		tilde[11] = p[5];
	 tilde[12] = 0; 	tilde[13] = 0;		tilde[14] = 0;		tilde[15] = 0;
}

void makeSkew(std::vector<double>& skew, std::vector<double>& p)
{
  assert(skew.size() == 9 && p.size() == 3);

  skew[0] = 0; skew[1] = -p[2]; skew[2] = p[1];
  skew[3] = p[2]; skew[4] = 0; skew[5] = -p[0];
  skew[6] = -p[1]; skew[7] = p[0]; skew[8] = 0;
}

void AdjointH(std::vector<double>& adj, std::vector<double>& hmatrix)
{
  assert(adj.size() == 36 && hmatrix.size() == 16);
  std::vector<double> rot(9, 0.0);

  // Nullify
  adj[3] = 0;  adj[4] = 0;  adj[5] = 0;
  adj[9] = 0;  adj[10] = 0; adj[11] = 0;
  adj[15] = 0; adj[16] = 0; adj[17] = 0;

  // Copy rotation matrices
  rot[0] = adj[0] = adj[21] = hmatrix[0];
  rot[1] = adj[1] = adj[22] = hmatrix[1];
  rot[2] = adj[2] = adj[23] = hmatrix[2];

  rot[3] = adj[6] = adj[27] = hmatrix[4];
  rot[4] = adj[7] = adj[28] = hmatrix[5];
  rot[5] = adj[8] = adj[29] = hmatrix[6];

  rot[6] = adj[12] = adj[33] = hmatrix[8];
  rot[7] = adj[13] = adj[34] = hmatrix[9];
  rot[8] = adj[14] = adj[35] = hmatrix[10];

  // skew(p) * R
  std::vector<double> res(9, 0.0);
  std::vector<double> skew(9, 0.0);
  std::vector<double> p(3, 0.0);
  p[0] = hmatrix[3];
  p[1] = hmatrix[7];
  p[2] = hmatrix[11];

  makeSkew(skew, p);

  mulMatrixMatrixSquare(res, skew, rot, 3);
  adj[18] = res[0];
  adj[19] = res[1];
  adj[20] = res[2];

  adj[24] = res[3];
  adj[25] = res[4];
  adj[26] = res[5];

  adj[30] = res[6];
  adj[31] = res[7];
  adj[32] = res[8];
}

void computeFiniteTwist(std::vector<double>& homogenous, std::vector<double>& twist, double sampletime)
{
  using namespace std;
  assert(homogenous.size() == 16 && twist.size() == 6);

  // timed_twist = T*sampletime //input velocity from any where
  vector<double> timed_twist(6, 0.0);
  for (unsigned int i=0; i<twist.size(); ++i)
    timed_twist[i] = twist[i]*sampletime;

  // omega=timed_twist[1:3];
  vector<double> omega(3, 0.0);
  omega.assign(timed_twist.begin(), timed_twist.begin()+3);

  // v=timed_twist[4:6]
  vector<double> v(3, 0.0);
  v.assign(timed_twist.begin()+3, timed_twist.begin()+6);

  // omega_norm=norm(omega);
  double omega_norm = sqrt(pow(omega[0],2)+pow(omega[1],2)+pow(omega[2],2));

  // if (omega_norm==0) then
  if (omega_norm==0)
  {
    //H=tilde(timed_twist)+eye(4);
    vector<double> tilde_timed_twist(16, 0.0);
    makeTilde(tilde_timed_twist,timed_twist);

    vector<double> eye4(SIZE_H, 0.0);
    eye4.assign(EYE4, EYE4 + SIZE_H);
    homogenous=eye4+tilde_timed_twist;
  }
  else
  {
    vector<double> eye3(SIZE_R, 0.0);
    eye3.assign(EYE3, EYE3+SIZE_R);

    //omega_temp=omega/omega_norm;
    vector<double> omega_temp(3, 0.0 );
    for (unsigned int i=0; i < omega_temp.size(); ++i)
      omega_temp[i]=omega[i]/omega_norm;

    /* R=eye(3)+skew(omega_temp)*sin(omega_norm)+skew(omega_temp)*skew(omega_temp)*(1-cos(omega_norm));*/
    vector<double> skew_omega_temp(SIZE_R, 0.0 );
    makeSkew(skew_omega_temp,omega_temp);

    vector<double> R(9, 0.0 );
    vector<double> skew_omega_temp2(9, 0.0);
    mulMatrixMatrixSquare(skew_omega_temp2,skew_omega_temp,skew_omega_temp,3);

    R=eye3+skew_omega_temp*sin(omega_norm)+skew_omega_temp2*(1-cos(omega_norm));

    /*P = ((eye(3)-R)*(skew(omega)*v)+transpose(omega)*v*omega)/omega_norm^2;*/
    vector<double> skew_omega(SIZE_R, 0.0);
    makeSkew(skew_omega,omega);

    vector<double> skew_omega_v(3, 0.0 );
    mulMatrixVector(&skew_omega_v[0], skew_omega, v, 3, 3);

    vector<double> sum(3, 0.0 );
    vector<double> eye3_R = (eye3-R);
    mulMatrixVector(&sum[0], eye3_R, skew_omega_v, 3, 3);

    vector<double> trans_omega(3, 0.0);
    matrixTranspose(trans_omega,omega,3,1);

    sum=sum + omega*(trans_omega*v);
    sum=sum*(1.0/pow(omega_norm,2));

    // Write result
    homogenous[0]=R[0];homogenous[1]=R[1];homogenous[2]=R[2];homogenous[3]=sum[0];
    homogenous[4]=R[3];homogenous[5]=R[4];homogenous[6]=R[5];homogenous[7]=sum[1];
    homogenous[8]=R[6];homogenous[9]=R[7];homogenous[10]=R[8];homogenous[11]=sum[2];
    homogenous[12]=0;homogenous[13]=0;homogenous[14]=0;homogenous[15]=1;
  }


}
