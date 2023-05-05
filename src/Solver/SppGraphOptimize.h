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
 * Created by lizhen on 2021/11/9.
 *-------------------------------------------------------------------------------*/

#ifndef SPPGRAPHOPTIMIZE_H
#define SPPGRAPHOPTIMIZE_H

#include "GraphOptimizeSolver.h"
#include "../SensorErrorModel/GnssErrorModel.h"

#define SppStateSize 8

class SppGraphOptimize:public GraphOptimizeSolver {
public:
    SppGraphOptimize();
    ~SppGraphOptimize() = default;
    bool Solving(Solution& solution, const GnssObsEpoch& obs) override;

private:
    bool InitSolving(const GnssObsEpoch& obs, const Solution& solution, ceres::Solver::Options& options);
    bool InitNewlyAddedGraph(const Solution& solution);                            /* initialize the newly added states       */

    bool AddParameterBlocksToGraph(ceres::Problem& problem) override;              /* add parameter blocks as graph node      */
    bool AddPseudorangeFactors(ceres::Problem& problem) override;                  /* add pseudorange FACTORS to factor graph */
    bool AddDopplerFactors(ceres::Problem& problem) override;                      /* add Doppler FACTORS to factor graph     */
    bool AddConstrainFactors(ceres::Problem& problem, double* x0);                 /* add Constrain FACTORS to factor graph   */

private:
    struct PseudorangeFactor{
        PseudorangeFactor(string prn, Vector3d spos, double P, double var ):
                prn(prn), spos(spos), P(P), var(var){}

        template <typename T>
        bool operator()(const T* const state, T* residuals) const{
            T est_pseudorange;
            T delta_x = pow((state[0] - spos.x()),2);
            T delta_y = pow((state[1] - spos.y()),2);
            T delta_z = pow((state[2] - spos.z()),2);

            est_pseudorange = sqrt(delta_x + delta_y + delta_z);
            est_pseudorange = est_pseudorange + WGS84_WIE * (spos.x() * state[1] - spos.y() * state[0]) / CLIGHT;

            est_pseudorange += state[3];  // clock
            int system = CString::satsys(prn);
            if (config.BdsOption == BDSOPT_BD2_3)
            {
                if(system == SYS_BDS && CString::str2num(prn, 1, 2) > 18) system = SYS_BD3;
            }

            switch (system)
            {
                case SYS_BDS: est_pseudorange += state[2 + SYS_BDS]; break;
                case SYS_GLO: est_pseudorange += state[2 + SYS_GLO]; break;
                case SYS_GAL: est_pseudorange += state[2 + SYS_GAL]; break;
                case SYS_BD3: est_pseudorange += state[2 + SYS_BD3]; break;
                default:                                    break;
            }
            residuals[0] = (est_pseudorange - T(P)) / T(var);
            return true;
        }

        string prn;
        Vector3d spos;
        double P;
        double var;
    };


    struct ConstrainFactor{
        ConstrainFactor(int sys, double P, double var): sys(sys), P(P), var(var){}
        template <typename T>
        bool operator()(const T* const state, T* residuals) const{
            residuals[0] = (P - state[3 + sys]) / T(var);
            return true;
        }
        int sys;
        double P;
        double var;
    };

};


#endif //SPPGRAPHOPTIMIZE_H
