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
 * Created by lizhen on 2021/9/22.
 *-------------------------------------------------------------------------------*/

#ifndef RTKPOSITION_H
#define RTKPOSITION_H

#include "SppPosition.h"
#include "../ResultExport/WriteResultFile.h"
#include "../SensorErrorModel/GnssErrorModel.h"
#include "../Solver/RtkGraphOptimize.h"

#define DTAGE       0.999    /* tolerance of time difference age (s) */

class RtkPosition: public MultiPosition{

public:
    RtkPosition();
    ~RtkPosition();
    bool Processing() override;

private:
    bool Initialization() override;
    bool FilterPosition() override;        /* solve position using EKF method            */
    bool GraphPosition();                  /* solve position using Graph method          */
    bool DataPrepared() override;          /* including GNSS and IMU (TC) data matching  */
    bool UpdateStates() override;          /* include pos, vel, acc, ion, trp and amb    */

    bool FormPositionDesignMatrix(Equation& eq_t, const Vector3d& e, int ambR, int ambB,
                                  int frq, double lami, double lamj) override;
    void FormVarMatrix(const vector<Equation> &eq) override;

private:
    bool RtkInitialize();                  /* init rtk slove process                                          */
    bool CheckBaseCoord();                 /* check whether the coordinates of the base station are incorrect */
    bool MatchBaseRoverData();             /* match base station and rover observation according to time      */
    int SelectCommonSatellite();           /* select common satellite of base and rover(will judge base el)   */
    bool SelectRefSatellite(int sys);      /* select reference satellite of each system according to el       */
    bool VaildSatellite(int id, int frq);  /* judge whether it's a vaild satellite according zero difference  */

private:
    struct omcinfo{
        double L[NFREQ] = {0.0};
        double P[NFREQ] = {0.0};
    };

private:       /* function for generate undifferenced(y=mObs-r) and double difference(omc=(yr-ybr)-(y-yb)) equation------------------------*/
    /* undifferenced phase/code residuals calculate [observed pseudorange - range] */
    bool CalZeroDiffResiduals(GnssObsEpoch &ob, const Vector3d& pos, map<int, omcinfo>& omc, const ReceiverInfo& head); /* cal undifferenced residuals */
    int CalDoubleDiffResiduals();                                                           /*cal double-differenced residuals and partial derivatives */

    void CalGeometricRange(GnssObsEpoch &ob, const Vector3d& pos, const ReceiverInfo& head); /* cal distance between sat and rover(r), include azel    */
    void CalUndifferResidual(GnssObsEpoch &ob, map<int, omcinfo> &omc);                      /* cal difference between observation and r               */

    void CalCodeResiduals(int sys);      /* calculate double differences of code residuals for each sat     */

private:
    int mObsNum = 0;                    /* number of vaild common obs between base and rover                 */
    int mBaseId = 0;                    /* ID of base observation is obss_base, for backward                 */
    int mRefSat[NUMSYS][NFREQ] = {{0}}; /* Reference satellite of each system and each frequency             */
    bool mIsInitialize = false;         /* remark for initialization of rtk process                          */
    double mBaseline = 0;               /* distance from base to rover                                       */

    int mGroup = 0;                     /* number of groups (2 for each system, phase and code), for rtk use */
    int mPair[NUMSYS*NFREQ*2] = {0};    /* number of sat pairs in group, mGroup                              */

    map<int, omcinfo> mOmc ;                     /* omc of carrier-phase(m) and pseudorange (m) for rover    */
    map<int, omcinfo> mOmcBase;                  /* ditto, but this is for base station's omc                */

    ReceiverInfo mBaseInf;                       /* informaton of base station                               */
    GnssObsEpoch mObsBase;                       /* observation of base for this epoch                       */
    GnssObsEpoch mObsBase_t;                     /* observation of GNSS for this epoch, temp use             */
    GraphOptimizeSolver::GraphInfo mFgInfo;      /* information saved for graph optimize used                */
    vector<GnssObsEpoch> mObssBase;              /* observation of base, for backward use, all data          */

    std::shared_ptr <ReadRinexObs> pGnssObsBase; /* Child pointer of read gnss observation of base station   */
    std::shared_ptr <SppPosition> pSpp;          /* pointer of spp position, using for init position for rtk */

};


#endif //RTKPOSITION_H
