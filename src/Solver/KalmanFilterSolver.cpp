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

#include "KalmanFilterSolver.h"


bool KalmanFilterSolver::InitGnssFilter(const Solution& solution, MatrixXd& Xf, MatrixXd& Pf){
    for (int i = 0; i < 3; i++)
    {
        this->initx(solution.pos[i], VAR_POS, CState::XiP()  + i, Xf, Pf);
        this->initx(solution.vel[i], VAR_VEL, CState::XiV()  + i, Xf, Pf);
        this->initx(1E-6, VAR_ACC, CState::XiAc() + i, Xf, Pf);
    }
    if (config.VelocityMode > VELOPT_OFF) this->initx(1e-6, VAR_CDR, CState::XiCdr(), Xf, Pf);
    return true;
}


bool KalmanFilterSolver::InitVelFilter(const Solution& solution, MatrixXd& Xf, MatrixXd& Pf){
    for (int i = 0; i < 3; i++)
    {
        this->initx(solution.vel[i], VAR_VEL, 0 + i, Xf, Pf);
        this->initx(1E-6,            VAR_ACC, 3 + i, Xf, Pf);
    }
    this->initx(1e-6, VAR_CDR, 6, Xf, Pf);
    return true;
}


bool KalmanFilterSolver::Solving(const vector<Equation>& eq, Solution& solution, MatrixXd& Xf, MatrixXd& Pf, MatrixXd& R){
    if (!this->InitSolving(eq))       return false;
    if (!this->FormFilterMatrix(eq, Xf, Pf))  return false;
    if (!(solution.pdop = this->MeasutementUpdate(Xf, Pf, R)) && eq.size() < 3) return false;
    return true;
}


bool KalmanFilterSolver::InitSolving(const vector<Equation> &eq) {
    if (eq.empty())
    {
        Log.Trace(TWARNING, "KalmanFilterSolver, Empty dimension");
        return false;
    }

    mRow = int(eq.size());
    mCol = int(eq[0].H.size());

    if (mRow == 0 || mCol == 0)
    {
        Log.Trace(TERROR, "KalmanFilterSolver, Equation dimension error, row= "
                           + to_string(mRow) + " col= " + to_string(mCol));
        return false;
    }

    mH = MatrixXd::Zero(mRow, mCol);
    mV = MatrixXd::Zero(mRow, 1);

    return true;
}


bool KalmanFilterSolver::FormFilterMatrix(const vector<Equation>& eq, const MatrixXd& Xf, const MatrixXd& Pf) {
    int i, j;
    int counter = 0;
    mValueId.clear();

    for (auto iter = eq.begin(); iter != eq.end() ; iter++, counter++)
    {
        if (iter->H.size() != mCol)
        {
            Log.Trace(TERROR, "FormFilterMatrix, wrong equation dimension for col = " + to_string(iter->H.size()));
            return false;
        }
        for (i = 0; i < iter->H.size(); ++i)
        {
            mH(counter, i) = iter->H[i];
        }
        mV(counter, 0) = iter->omc;
    }

    /* create list of non-zero states */
    for (i = 0, mPartCol = 0; i < Xf.size(); i++)
    {
        if (Xf(i, 0) != 0.0 && Pf(i, i) > 0.0)
        {
            mValueId.push_back(i);
            mPartCol++;
        }
    }

    mX_ = MatrixXd::Zero(mPartCol, 1);
    mP_ = MatrixXd::Zero(mPartCol, mPartCol);
    mH_ = MatrixXd::Zero(mRow, mPartCol);
    mI = MatrixXd::Identity(mPartCol, mPartCol);

    /* compress array by removing zero elements to save computation time */
    for (i = 0; i < mPartCol; i++)
    {
        mX_(i, 0) = Xf(mValueId[i], 0);
        for (j = 0; j < mPartCol; j++) mP_(i, j) = Pf(mValueId[i], mValueId[j]);
        for (j = 0; j < mRow; j++) mH_(j, i) = mH (j, mValueId[i]);
    }

    return true;
}


double KalmanFilterSolver::MeasutementUpdate(MatrixXd& Xf, MatrixXd& Pf, const MatrixXd& R) {
    Log.TraceMat(TEXPORT, "X", mX_);
    Log.TraceMat(TEXPORT, "P", mP_);

    Log.TraceMat(TEXPORT, "H", mH_);
    Log.TraceMat(TEXPORT, "V", mV);
    Log.TraceMat(TEXPORT, "R", R);

    mK = (mH_ * mP_ * mH_.transpose() + R).inverse();
    mK = mP_ * mH_.transpose() * mK;

    mX_ = mX_ + mK * mV;
    mP_ = (mI - mK * mH_) * mP_ * (mI - mK * mH_).transpose() + mK * R * mK.transpose();

    Log.TraceMat(TEXPORT, "Xpost", mX_);
    Log.TraceMat(TEXPORT, "Ppost", mP_);

    /* copy values from compressed arrays back to full arrays */
    for (int i = 0; i < mPartCol; i++)
    {
        Xf(mValueId[i], 0) = mX_(i, 0);
        for (int j = 0; j < mPartCol; j++) Pf(mValueId[i], mValueId[j]) = mP_(i, j);
    }

    mG = (mH_.transpose() * mH_).block<3,3>(0, 0).inverse();   // for pdop
    return sqrt(mG(0, 0) + mG(1, 1) + mG(2, 2));
}


void KalmanFilterSolver::initx(double x, double var, int id, MatrixXd& Xf, MatrixXd& Pf) {
    Xf(id, 0) = x;
    for (int i = 0; i < Pf.cols(); i++)
    {
        Pf(i, id) = 0.0;
    }
    for (int j = 0; j < Pf.rows(); j++)
    {
        Pf(id, j) = 0.0;
    }
    Pf(id, id) = var;
}

