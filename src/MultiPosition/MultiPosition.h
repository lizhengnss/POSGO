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
 * Created by lizhen on 2021/7/21.
 *-------------------------------------------------------------------------------*/

#ifndef MULTIPOSITION_H
#define MULTIPOSITION_H

#include "../Global.h"
#include "../DataManagement/ReadGnssData.h"
#include "../DataManagement/ReadGnssProduct.h"
#include "../OrbitClock/NavOrbitClock.h"
#include "../SensorErrorModel/GnssErrorModel.h"
#include "../Solver/LeastSquareSolver.h"
#include "../Solver/KalmanFilterSolver.h"
#include "../Solver/GraphOptimizeSolver.h"
#include "../ResultExport/WriteResultFile.h"
#include "../SystemSetting/DebugInfo.h"

#define MAX_ITR     300         /* max number of iteration for point pos        */
#define MAX_VAR_EPH SQR(300.0)  /* max variance eph to reject satellite (m^2)   */

class MultiPosition{

public:
    MultiPosition() = default;
    ~MultiPosition() = default;

    int mSolveType = MODE_FORWARD;          /* solve mode              */

    virtual bool Processing() = 0;
    void GetSolution(Solution& solution);   /**/

protected:
    struct sQuality{
        int index = 0;
        double fact = 0.0;
    };

protected:
    std::shared_ptr <ReadRinexObs>        pGnssObs;   /* pointer of read gnss observation file            */
    std::shared_ptr <ReadRinexNav>        pGnssNav;   /* pointer of read gnss navigation file             */
    std::shared_ptr <ReadIonTecGrid>      pGnssIon;   /* pointer of read gnss ionosphere file             */

    std::shared_ptr <OrbitClock>          pOrbclk;    /* pointer of satellite orbit and clock             */
    std::shared_ptr <GnssErrorModel>      pEm;        /* pointer of error model                           */
    std::shared_ptr <ResultExport>        pEx;        /* pointer of result export                         */

    std::shared_ptr <LeastSquareSolver>   pLsq;       /* pointer of slover for least square               */
    std::shared_ptr <GraphOptimizeSolver> pGo;        /* pointer of slover for graph optimization         */
    std::shared_ptr <KalmanFilterSolver>  pEkf;       /* pointer of slover for EKF                        */
    std::shared_ptr <KalmanFilterSolver>  pEkfVel;    /* pointer of slover for velocity EKF               */

    std::shared_ptr <DebugRobust>         pRobLog;    /* pointer of export robust debug information       */
    std::shared_ptr <DebugResidual>       pPosResLog; /* pointer of export pos residual debug information */
    std::shared_ptr <DebugResidual>       pVelResLog; /* pointer of export vel residual debug information */

protected:
    int mEpoch = 0;                 /* Arc segment                                    */
    int mRoverId = 0;               /* ID of rover observation is obss, for backward  */
    bool mIsUseVelocity = false;
    bool mVsat[MAXSAT][NFREQ];      /* valid satellite information sign               */
    STimes mTime;                   /* time of observation                            */
    Solution mSol;                  /* result information of this epoch               */
    vector<Equation> mPosEq;        /* information needed to form position equations  */
    vector<Equation> mPosEqPri;     /* priori information position equations          */
    vector<GnssObsEpoch> mObss;     /* observation of all epoch, for backward use     */

    Vector3d mPrePos;
    Vector3d mPostPos;

    GnssNavData mNavs;              /* all navigation information read form file      */
    GnssObsEpoch mObs;              /* observation of GNSS for this epoch             */
    ReceiverInfo mRoverInf;         /* informaton of rover                            */

    Matrix<double, Dynamic, Dynamic> mXf;       /* all state matrix matrix            */
    Matrix<double, Dynamic, Dynamic> mPf;       /* all variance matrix                */
    Matrix<double, Dynamic, Dynamic> mR;        /* error matrix                       */

private:
    virtual bool Initialization() = 0;
    virtual bool DataPrepared() = 0;

    virtual bool UpdateStates(){           /* time update of states, include pos, vel, acc, ion, trp, ambiguity and so on */
        Log.Trace(TERROR, "*UpdateStates function in MultiPosition class overload error, please check!");
        return false;
    }

    virtual bool FilterPosition(){         /* solve position using EKF method        */
        Log.Trace(TERROR, "*FilterPosition function in MultiPosition class overload error, please check!");
        return false;
    }

    virtual bool GraphPosition(){           /* solve position using Graph method      */
        Log.Trace(TERROR, "*GraphPosition function in MultiPosition class overload error, please check!");
        return false;
    }

    virtual bool FormPositionDesignMatrix(Equation& eq_t, const Vector3d& e, int ambR, int ambB,
                                            int frq, double lami, double lamj){             /* construction of design mat H for pos   */
        Log.Trace(TERROR, "*FormPositionDesignMatrix function in MultiPosition class overload error, please check!");
        return false;
    }

    virtual void FormVarMatrix(const vector<Equation> &eq) {
        Log.Trace(TERROR, "*FormVarMatrix function in MultiPosition class overload error, please check!");
    }

protected:
    bool UpdatePosition();            /* time update, include position, velocity and acceleration */
    bool UpdataReceiverClock();       /* init receiver clock bias and drift as white noise        */

    void GetAllSatPosVelClock(GnssObsEpoch &ob);                                  /* get all satellite position, velocity and clock bias    */
    bool GetSatellitePosVelClock(STimes tTime, SatInfo &sat, const double *P);    /* get one satellite position, velocity and clock bias    */
    bool SatelliteExclude(SatInfo &sat);                                 /* exclude satellite according to config and satellite health      */
    double GeoDistance(SatInfo &sat, const Vector3d& rr, Vector3d& e);   /* calculate distance between sat.pos and rr, and line-of-sight(e) */
    bool SatAzimuthElevation(const Vector3d& pos, const Vector3d& e, GnssObsData& obs); /* calculate satellite azimuth/elevation angle      */
    double SingleDifferObs(double pi, double pj);                                       /* single-differenced of observation                */

    bool ReadAllObservation(std::shared_ptr <ReadRinexObs> p, vector<GnssObsEpoch>& obss);     /* read all data and save to obss, mainly for backward use */
    void SaveLastSolution();                                                      /* save last epoch solution to prevent crash caused by solution failure */
    void LoadLastSolution();                                                      /* load last epoch solution when solution failure of this epoch         */
    void SaveSolutionState();                                                     /* save fixed solution and other information to related variable        */

    bool CheckMaxResidual(int id, int frq, double res, bool code, const MatrixXd& Pf); /* check whether the max residual exceeded the threshold           */
    int SolutionQualityCheck(const Solution& solution, const vector<Equation>& eq);

protected:  /* for quality control use ---------------------------------------------------------*/
    bool mQcMask = false;            /* mask for qc according to max norm residual    */
    int mIterQc = 0;                 /* iteration number of robust filter execution   */
    double mV0 = 0, mV1 = 0;         /* max norm v(v0), and this norm v(v0)           */
    double mK0 = config.RobThres[1];
    double mK1 = config.RobThres[2]; /* threshold for robust kernel function          */
    vector<sQuality> mQc;            /* all index and variance expansion factor of eq */
    Matrix<double, Dynamic, Dynamic> mXt;   /* temporary mat of state for qc use      */
    Matrix<double, Dynamic, Dynamic> mPt;   /* temporary mat of variance for qc use   */

    bool isReduced(int id);          /* judge whether the observation been expand     */
    int FindMaxNormResidual(const vector<Equation>& eq); /* max norm residual eq.omc  */

    bool Robust(vector<Equation>& eq);         /* robust function                     */
    bool Robust_IGG3(vector<Equation>& eq);    /* robust using IGG3 kernel function   */
    bool Robust_Huber(vector<Equation>& eq);   /* robust using Huber kernel function  */

protected:  /* for gnss velocity use -----------------------------------------------------------*/
    Matrix<double, Dynamic, Dynamic> mXvf;          /* vel state matrix matrix        */
    Matrix<double, Dynamic, Dynamic> mPvf;          /* vel variance matrix            */
    vector<Equation> mVelEq;        /* information needed to form velocity equations  */
    vector<Equation> mVelEqPri;     /* priori information velocity equations          */

    bool VelocityProcess();                    /* solve velocity                           */
    bool SolveVelocity();                      /* solve velocity using least square method */
    bool FilterVelocity();                     /* solve velocity using EKF method          */
    bool UpdateVelState();                     /* time update, include vel, acc and rcd    */
    bool FormVelocity(const Vector3d &vel, const double drcd);   /* construction of velocity error equation */
    void FormVelVarMatrix(const vector<Equation> &eq);           /* construction of variance mat R for vel  */
    bool FormVelDesignMatrix(Equation& eq_t, const Vector3d& e); /* construction of design mat H for vel    */
    bool RobustVelocity(vector<Equation>& eq);                   /* quality control for solve velocity      */
    int VelocityQualityCheck(const vector<Equation>& eq);        /* velocity result check                   */

private:
    const double sChisqr[150]={      /* chi-sqr(n) (alpha=0.001) */
            10.8,13.8,16.3,18.5,20.5,22.5,24.3,26.1,27.9,29.6,
            31.3,32.9,34.5,36.1,37.7,39.3,40.8,42.3,43.8,45.3,
            46.8,48.3,49.7,51.2,52.6,54.1,55.5,56.9,58.3,59.7,
            61.1,62.5,63.9,65.2,66.6,68.0,69.3,70.7,72.1,73.4,
            74.7,76.0,77.3,78.6,80.0,81.3,82.6,84.0,85.4,86.7,
            88.0,89.3,90.6,91.9,93.3,94.7,96.0,97.4,98.7,100 ,
            101 ,102 ,103 ,104 ,105 ,107 ,108 ,109 ,110 ,112 ,
            113 ,114 ,115 ,116 ,118 ,119 ,120 ,122 ,123 ,125 ,
            126 ,127 ,128 ,129 ,131 ,132 ,133 ,134 ,135 ,137 ,
            138 ,139 ,140 ,142 ,143 ,144 ,145 ,147 ,148 ,149 ,
            150 ,151 ,152 ,153 ,154 ,155 ,156 ,157 ,158 ,159 ,
            160 ,161 ,162 ,163 ,164 ,165 ,166 ,167 ,168 ,169 ,
            170 ,171 ,172 ,173 ,174 ,175 ,176 ,177 ,178 ,179 ,
            180 ,181 ,182 ,183 ,184 ,185 ,186 ,187 ,188 ,189 ,
            190 ,191 ,192 ,193 ,194 ,195 ,196 ,197 ,198 ,199
    };

};



class SmoothPosition {
public:
    SmoothPosition();
    ~SmoothPosition();
    bool Smooth();                             /* smooth solution using forward and backward solution */

private:
    bool Initialization();                                               /* read forward and backward solution and init */
    bool Smoothing(const SmoothSolution& fs, const SmoothSolution& bs);  /* smoothing epoch by epoch                    */
    void SmoothCalculate(const double xf, const double Qf, const double xb, const double Qb, double& xs, double& Qs); /* smooth calculate */

private:
    std::shared_ptr <ReadGnssResultBin> pForward;   /* pointer of read GNSS result forward temp file  */
    std::shared_ptr <ReadGnssResultBin> pBackward;  /* pointer of read GNSS result backward temp file */
    std::shared_ptr <ResultExport>      pEx;        /* Parent pointer of result export                */

    Solution mSol;                                  /* result information of this epoch               */
    map<double, SmoothSolution> mForwardSol;        /* forward solution                               */
    map<double, SmoothSolution> mBackwardSol;       /* backward solution                              */
};


#endif //MULTIPOSITION_H
