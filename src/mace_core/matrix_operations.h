#ifndef MATRIXOPERATIONS_H
#define MATRIXOPERATIONS_H

#include "Eigen/Core"

#include <vector>

//!
//! \brief Structure defining a modication to a single cell of a matrix
//!
template <typename T>
struct MatrixCellData
{
    int i;
    int j;
    T data;
};


static void OperateOnMatrixd(Eigen::MatrixXd &mat, const std::vector<MatrixCellData<double>> &operations)
{
    for(MatrixCellData<double> cell : operations)
    {
        mat(cell.i, cell.j) = cell.data;
    }
}

#endif // MATRIXOPERATIONS_H
