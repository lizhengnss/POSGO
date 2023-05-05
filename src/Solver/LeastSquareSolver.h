/*--------------------------------------------------------------------------------
 * POSGO: a GNSS POSition system based on Graph Optimization
 *
 * Copyright (C) 2022 Satellite POD and Navigation Augmentation Group,
 *                    Wuhan University
 *
 *     Author : Zhen Li
 *    Contact : sdkjlizhen@foxmail.com
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 * reference :
 *
 * Created by lizhen on 2021/8/21.
 *-------------------------------------------------------------------------------*/

#ifndef LEASTSQUARESOLVER_H
#define LEASTSQUARESOLVER_H

#include "Solver.h"

class LeastSquareSolver: public Solver{
public:
    LeastSquareSolver()  = default;
    ~LeastSquareSolver() = default;
    bool Solving(VectorXd &dx, MatrixXd &Qdx, vector<Equation>& eq);
    bool SolutionFeedback(Solution& sol, int mode);
private:
    int mRow = 0;                            /* row of observation */
    int mCol = 0;                            /* col of H mat       */
    Matrix<double, Dynamic, Dynamic> mH;     /* design matrix      */
    Matrix<double, Dynamic, Dynamic> mV;     /* measurements       */
    Matrix<double, Dynamic, Dynamic> mR;     /* weight             */
private:
    bool InitSolving(const vector<Equation>& eq) override;
};


#endif //LEASTSQUARESOLVER_H
