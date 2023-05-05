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
 * Created by lizhen on 2021/9/1.
 *-------------------------------------------------------------------------------*/

#include "GraphOptimizeSolver.h"


 GraphOptimizeSolver::~GraphOptimizeSolver() {
    for (int i = 0; i < mStateArray.size(); i++)
    {
        delete[] mStateArray[i];
    }
 }


bool GraphOptimizeSolver::SetupSolverOptions(ceres::Solver::Options& options){
    options.minimizer_progress_to_stdout = false;  // output process
    options.use_nonmonotonic_steps = true;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;   // solve type
    options.trust_region_strategy_type = ceres::TrustRegionStrategyType::DOGLEG;  // or LEVENBERG_MARQUARDT
    options.dogleg_type = ceres::DoglegType::SUBSPACE_DOGLEG;
    options.num_threads = 2;
    options.max_num_iterations = 64;  // max number of iterations
    return true;
}


bool GraphOptimizeSolver::SetupLossFunction(){
    if      (config.LossFunction == FGLOSSFUN_HUBER)   mLossFunction = new ceres::HuberLoss(config.LossFunctionValue);
    else if (config.LossFunction == FGLOSSFUN_CAUCHY)  mLossFunction = new ceres::CauchyLoss(config.LossFunctionValue);
    else if (config.LossFunction == FGLOSSFUN_ARCTAN)  mLossFunction = new ceres::ArctanLoss(config.LossFunctionValue);
    else if (config.LossFunction == FGLOSSFUN_TUKEY)   mLossFunction = new ceres::TukeyLoss(config.LossFunctionValue);
    else if (config.LossFunction == FGLOSSFUN_SOFTONE) mLossFunction = new ceres::SoftLOneLoss(config.LossFunctionValue);
    else mLossFunction = nullptr;
    return true;
}


bool GraphOptimizeSolver::CheckSlideWindow(){
    if (mSlideWindow.empty()) return true;   // empty slide window

    if ((CTimes::timediff(mObs.time, mSlideWindow[mSlideWindow.size() - 1].time) > MAX_WINDOW_INERVAL) || !mIsVelocity)
    {
        mSlideWindow.clear();
        int StateSize = int(mStateArray.size());
        for (int i = 0; i < StateSize; i++)
        {
            delete[] mStateArray[0];
            mStateArray.erase(mStateArray.begin());
        }

    }
    return true;
}


bool GraphOptimizeSolver::SolveGraph(ceres::Problem& problem, ceres::Solver::Options& options, ceres::Solver::Summary summary) {
    ceres::Solve(options, &problem, &summary);
 //   cout<<summary.BriefReport()<<"\n";
 //   cout<<summary.FullReport()<<"\n";
    return true;
}


bool GraphOptimizeSolver::SolveCovariance(ceres::Problem &problem, int state_num) {
    ceres::Covariance::Options cov_options;
    ceres::Covariance covariance(cov_options);

    int index = mSlideWindowSize - 1;
    vector<pair<const double*, const double*>> covariance_blocks;

    mStateVar = MatrixXd::Identity(state_num, state_num);

    covariance_blocks.emplace_back(make_pair(mStateArray[index], mStateArray[index]));

    if (covariance.Compute(covariance_blocks, &problem))
    {
        covariance.GetCovarianceBlock(mStateArray[index], mStateArray[index], mStateVar.data());
        return true;
    }
    else
    {
        mStateVar *= SQR(999.9);
        return false;
    }

}


bool GraphOptimizeSolver::SaveResult(Solution& solution){
    int id = mSlideWindowSize - 1;

    if (config.ProcessMode == PMODE_SINGLE)
    {   // check for spp optimize
        double vv = 0;
        for (int i = 0; i < 3; i++) vv += fabs(mPreviousPos[i] - mStateArray[id][i]);
        if (vv > 300 && mSlideWindowSize > 1)
        {
            for (int i = 0; i < 3; i++)
            {   // position
                mStateArray[id][i] = mPreviousPos[i] + mSlideWindow[id-1].vel[i];
            }
            for (int i = 0; i < 5; i++)
            {
                mStateArray[id][3 + i] = mPreviousClk[i];
            }
            return false;
        }
    }
    else if (config.ProcessMode == PMODE_RTK)
    {   // check for rtk optimize
        double vv1 = 0, vv2 = 0;
        for (int i = 0; i < 3; i++) vv1 += fabs(mPrioriPos[i] - mStateArray[id][i]);
        for (int i = 0; i < 3; i++) vv2 += fabs(solution.pos[i] - mStateArray[id][i]);
        if ((vv1 > 15 && vv2 > 300) || mStateVar(0,0)>100)
        {
            for (int i = 0; i < 3; i++)
            {   // position
                mStateArray[id][i] = mPrioriPos[i];
            }
            return false;
        }
    }

    for (int i = 0; i < 3; i++)
    {   // position
        solution.pos[i] = mStateArray[id][i];
    }

    if (config.ProcessMode == PMODE_SINGLE)
    {
        for (int i = 0; i < 5; i++)
        {   // clock bias
            solution.drck[i] = mStateArray[id][3 + i];
        }
    }

    return true;
}


bool GraphOptimizeSolver::SaveCovariance(Solution &solution) {
    solution.Qpos = mStateVar;
    return true;
}


void GraphOptimizeSolver::AdjustSlideWindow() {
    if (mSlideWindowSize < config.SlideWindowSize) return;
    mSlideWindow.erase(mSlideWindow.begin());
    delete[] mStateArray[0];
    mStateArray.erase(mStateArray.begin());
}
