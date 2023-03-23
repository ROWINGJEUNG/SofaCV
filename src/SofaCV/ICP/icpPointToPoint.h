/*
Copyright 2011. All rights reserved.
Institute of Measurement and Control Systems
Karlsruhe Institute of Technology, Germany

Authors: Andreas Geiger

libicp is free software; you can redistribute it and/or modify it under the
terms of the GNU General Public License as published by the Free Software
Foundation; either version 2 of the License, or any later version.

libicp is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
libicp; if not, write to the Free Software Foundation, Inc., 51 Franklin
Street, Fifth Floor, Boston, MA 02110-1301, USA 
*/

#ifndef ICP_POINT_TO_POINT_H
#define ICP_POINT_TO_POINT_H

#include "icp.h"

class IcpPointToPoint : public Icp {

public:

  IcpPointToPoint (double *M,const int32_t M_num,const int32_t dim) : Icp(M,M_num,dim) {}
  virtual ~IcpPointToPoint () {}

  // ICP�� �� �ܰ踦 �����ϱ� ���� �Լ�
  double fitStep (double *T,const int32_t T_num,Matrix &R,Matrix &t,const std::vector<int32_t> &active);

  // ICP ����� error�� �����ϱ� ���� �Լ� (only for 3D points)
  double getICPError(double* T, const int32_t T_num, Matrix& R, Matrix& t, const double indist);

  // target�� ���� �����Ÿ� ���Ͽ� �ִ� source ������ list�� ��ȯ
  std::vector<int32_t> getInliers (double *T,const int32_t T_num,const Matrix &R,const Matrix &t,const double indist);

  // target�� ���� �������� �ȿ� �ִ� source ���� list�� ��ȯ (3������ ���ؼ��� ������)
  std::vector<int32_t> getRanges(double* T, const int32_t T_num, const Matrix& R, const Matrix& t, const double min, const double max);

  // Source�� �� ������ ���� ����� Target ���� index�� return�Ѵ�
  std::vector<int32_t> getNearestIdxs(double* T, const int32_t T_num, const Matrix& R, const Matrix& t);
};

#endif // ICP_POINT_TO_POINT_H
