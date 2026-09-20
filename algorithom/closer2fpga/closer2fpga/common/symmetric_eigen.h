#pragma once
#include <vector>

namespace linalg {
// Stateful Jacobi sweeps over a symmetric matrix. Eigenvalues are ascending;
// eigenvectors are columns. The input matrix is passed by value as workspace.
// Preconditions: n >= 2; matrix contains n*n finite symmetric entries.
bool eigen_symmetric(std::vector<double> matrix, int n, std::vector<double>& eigenvalues,
                     std::vector<double>& eigenvectors);
// Read/modify/write accumulation: A += row * row^T, not a stateless operation.
// Precondition: matrix has row.size()*row.size() entries.
void accumulate_outer(std::vector<double>& matrix, const std::vector<double>& row);
} // namespace linalg
