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
 * Created by lizhen on 2021/9/16.
 *-------------------------------------------------------------------------------*/

#ifndef KALMANFILTERSOLVER_H
#define KALMANFILTERSOLVER_H

#include "Solver.h"

class KalmanFilterSolver: public Solver{
public:
    KalmanFilterSolver()  = default;
    ~KalmanFilterSolver() = default;

    /* construct equation and filter */
    bool Solving(const vector<Equation>& eq, Solution& solution, MatrixXd& Xf, MatrixXd& Pf, MatrixXd& R); /* with equation construct */

    bool InitGnssFilter(const Solution& solution, MatrixXd& Xf, MatrixXd& Pf);   /* initialize filter state for first epoch */
    bool InitVelFilter(const Solution& solution, MatrixXd& Xf, MatrixXd& Pf);    /* same as above, but this for velocity    */

    void initx(double x, double var, int id, MatrixXd& Xf, MatrixXd& Pf);        /* init id as x and var, in Xf and Pf      */

private:
    int mRow;                                /* row of observation       */
    int mCol;                                /* col of full H mat        */
    int mPartCol;                            /* col of part matrix       */
    vector<int> mValueId;                    /* list of non-zero states  */
    Matrix<double, Dynamic, Dynamic> mX_;    /* part state matrix matrix */
    Matrix<double, Dynamic, Dynamic> mP_;    /* part variance matrix     */
    Matrix<double, Dynamic, Dynamic> mH;     /* full design matrix       */
    Matrix<double, Dynamic, Dynamic> mH_;    /* design matrix            */
    Matrix<double, Dynamic, Dynamic> mV;     /* measurements             */
    Matrix<double, Dynamic, Dynamic> mK;     /*  */
    Matrix<double, Dynamic, Dynamic> mG;     /* design matrix without weight for pdop calculation */
    Matrix<double, Dynamic, Dynamic> mI;     /*  */

private:
    bool InitSolving(const vector<Equation>& eq) override;                                      /* initialize filter state for first epoch    */
    bool FormFilterMatrix(const vector<Equation>& eq, const MatrixXd& Xf, const MatrixXd& Pf);  /* remove the part with 0 in Xf, Pf and H mat */
    double MeasutementUpdate(MatrixXd& Xf, MatrixXd& Pf, const MatrixXd& R);                    /* calculate measurement update               */

};


#endif //KALMANFILTERSOLVER_H
