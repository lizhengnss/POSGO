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

#ifndef RTKGRAPHOPTIMIZE_H
#define RTKGRAPHOPTIMIZE_H

#include "GraphOptimizeSolver.h"
#include "../SensorErrorModel/GnssErrorModel.h"

#define RtkStateSize 3

class RtkGraphOptimize: public GraphOptimizeSolver{
public:
    RtkGraphOptimize()  {
        mBasePose << config.BasePos[0], config.BasePos[1], config.BasePos[2];
    };
    ~RtkGraphOptimize() = default;
    bool Solving(Solution& solution, const GnssObsEpoch& obs, const GnssObsEpoch& obs_base, GraphInfo& FgInfo) override;

private:
    int mPsrNum;
    Vector3d mBasePose;       /* position of base station         */
    GnssObsEpoch mObsBase;    /* base obs of GNSS for newly added */

    bool InitSolving(const GnssObsEpoch& obs, const GnssObsEpoch& obs_base, const Solution& solution, ceres::Solver::Options& options);
    bool InitNewlyAddedGraph(Solution& solution, GraphInfo& FgInfo);
    bool AddParameterBlocksToGraph(ceres::Problem& problem) override;
    bool AddDopplerFactors(ceres::Problem& problem) override;  /* add Doppler factors to factor graph     */
    bool AddDDPseudorangeFactors(ceres::Problem& problem);

private:
    struct DDObservation{
        int Amb_Ref;
        int Amb_Sat;
        double lamref;
        double lamsat;
        double DD_Sat;           /* DD observation for rover, sat i          */
        double DD_Ref;           /* DD observation for rover, sat ref        */
        double DD_Sat_Base;      /* DD observation for base, sat i           */
        double DD_Ref_Base;      /* DD observation for base, sat ref         */
        double DDVar;            /* error variance of double-differenced obs */
        Vector3d Sat_Pos;        /* satellite position of rover, sat i       */
        Vector3d Ref_Pos;        /* satellite position of rover, sat ref     */
        Vector3d Sat_Base_Pos;   /* satellite position of base, sat i        */
        Vector3d Ref_Base_Pos;   /* satellite position of base, sat ref      */
    };

private:
    struct DDPseudorangeFactor{
        DDPseudorangeFactor(DDObservation DDPseudorange, Vector3d base_pos)
                    :DDPseudorange(DDPseudorange), base_pos(base_pos){}

        template <typename T>
        bool operator()(const T* const state, T* residuals) const
        {
            Vector3d base = base_pos;

            T r_Ref_Base = T (CMaths::distance(base, DDPseudorange.Ref_Base_Pos));
            r_Ref_Base += WGS84_WIE * (DDPseudorange.Ref_Base_Pos(0) * base(1) - DDPseudorange.Ref_Base_Pos(1) * base(0)) / CLIGHT;

            T r_Sat_Base = T (CMaths::distance(base, DDPseudorange.Sat_Base_Pos));
            r_Sat_Base += WGS84_WIE * (DDPseudorange.Sat_Base_Pos(0) * base(1) - DDPseudorange.Sat_Base_Pos(1) * base(0)) / CLIGHT;

            T dx1 = pow(state[0] - DDPseudorange.Sat_Pos(0), 2);
            T dy1 = pow(state[1] - DDPseudorange.Sat_Pos(1), 2);
            T dz1 = pow(state[2] - DDPseudorange.Sat_Pos(2), 2);
            T r_Sat_Rover = sqrt(dx1 + dy1 + dz1);
            r_Sat_Rover += WGS84_WIE * (DDPseudorange.Sat_Pos(0) * state[1] - DDPseudorange.Sat_Pos(1) * state[0]) / CLIGHT;

            T dx2 = pow(state[0] - DDPseudorange.Ref_Pos(0), 2);
            T dy2 = pow(state[1] - DDPseudorange.Ref_Pos(1), 2);
            T dz2 = pow(state[2] - DDPseudorange.Ref_Pos(2), 2);
            T r_Ref_Rover = sqrt(dx2 + dy2 + dz2);
            r_Ref_Rover += WGS84_WIE * (DDPseudorange.Ref_Pos(0) * state[1] - DDPseudorange.Ref_Pos(1) * state[0]) / CLIGHT;

            /* expected and observation of DD measurments */
            T DD_r   = T ((r_Sat_Rover - r_Sat_Base) - (r_Ref_Rover - r_Ref_Base));
            T DD_obs = T ((DDPseudorange.DD_Sat - DDPseudorange.DD_Sat_Base) - (DDPseudorange.DD_Ref - DDPseudorange.DD_Ref_Base));

            residuals[0] = (DD_obs - DD_r) / DDPseudorange.DDVar;

            return true;
        }

        DDObservation DDPseudorange;
        Vector3d base_pos;
    };

};


#endif //RTKGRAPHOPTIMIZE_H
