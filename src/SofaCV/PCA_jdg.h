#pragma once
#include <Eigen/Dense>
#include <numeric>

// row of [_m] : variable.
// col of [_m] : observation.
// The meanining of row and col are different from those of Matlab.

bool pcapc12(const std::vector<std::vector<double> >&_m, std::vector<double>*const w1, std::vector<double>*const w2, std::vector<double>* const w3,
			double &evalue1, double& evalue2, double& evalue3)
{
	int M = (int)(_m.size());
	const int N = (int)(_m[0].size());
	Eigen::MatrixXd centered(N, M);
	int m = 0; int n = 0;
	
	for (m = 0; m < M; m++) 
	{
		double sum = std::accumulate(_m[m].begin(), _m[m].end(), 0.0);
		double mean = sum / N;
		
		for (n = 0; n < N; n++) 
		{
			double value = _m[m][n];
			centered(n, m) = (value - mean);
		}
	}
	
	Eigen::MatrixXd cov = centered.adjoint() * centered;
	Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig(cov);
	
	Eigen::MatrixXd evecs = eig.eigenvectors();
	Eigen::VectorXd evalues = eig.eigenvalues();
	Eigen::MatrixXd pcaTransform = evecs.rightCols(3);
	evalue1 = evalues(2);
	evalue2 = evalues(1);
	evalue3 = evalues(0);

	for (m = 0; m < M; m++) 
	{
		if (w1 != NULL) w1->push_back(pcaTransform(m, 2));
		if (w2 != NULL) w2->push_back(pcaTransform(m, 1));
		if (w3 != NULL) w3->push_back(pcaTransform(m, 0));
	}
	return true;
}