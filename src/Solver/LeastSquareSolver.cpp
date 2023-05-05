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

#include "LeastSquareSolver.h"


bool LeastSquareSolver::Solving(VectorXd &dx, MatrixXd &Qdx, vector<Equation>& eq){
    int counter = 0;

    if(!this->InitSolving(eq)) return false;
    for (auto iter = eq.begin(); iter != eq.end() ; iter++, counter++)
    {
        if (iter->H.size() != mCol)
        {
            Log.Trace(TERROR, "Solving: incorrectly equation dimension for col = " + to_string(iter->H.size()));
            return false;
        }
        for (int i = 0; i < iter->H.size(); ++i)
        {
            mH(counter, i) = iter->H[i];
        }
        mR(counter, counter) = 1.0 / iter->R;
        mV(counter, 0) = iter->omc;
    }

    Qdx = (mH.transpose() * mR * mH).inverse();

    dx = Qdx * mH.transpose() * mR * mV;

    Log.TraceMat(TEXPORT, "H", mH);
    Log.TraceMat(TEXPORT, "V", mV);
    Log.TraceMat(TEXPORT, "R", mR);
    Log.TraceVec(TEXPORT, "X", dx);

    return true;
}

bool LeastSquareSolver::SolutionFeedback(Solution& sol, int mode){

    if (mode == LSQ_SOLVE_POS)
    {
        for (int i = 0; i < 3; i++)
        {
            sol.pos(i) += sol.pdx(i, 0);
        }
        for (int i = 3; i < sol.pdx.rows(); i++)
        {
            sol.drck[i-3] += sol.pdx(i, 0);
        }
    }
    else if(mode == LSQ_SOLVE_VEL)
    {
        for (int i = 0; i < 3; i++)
        {
            sol.vel(i) += sol.vdx(i, 0);
            sol.veli(i) = sol.vel(i);
        }
        sol.drcd += sol.vdx(3, 0);
    }

    return true;
}

bool LeastSquareSolver::InitSolving(const vector<Equation>& eq){
    if (eq.empty())
    {
        Log.Trace(TWARNING,"LeastSquareSolver, Empty dimension");
        return false;
    }

    mRow = int(eq.size());
    mCol = int(eq[0].H.size());
    if (mRow == 0 || mCol == 0)
    {
        Log.Trace(TERROR, "LeastSquareSolver, Equation dimension error, row= "
                          + to_string(mRow) + " col= " + to_string(mCol));
        return false;
    }

    mH = MatrixXd::Zero(mRow, mCol);
    mR = MatrixXd::Zero(mRow, mRow);
    mV = MatrixXd::Zero(mRow, 1);

    return true;
}
