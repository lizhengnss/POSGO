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
 * Created by lizhen on 2021/7/13.
 *-------------------------------------------------------------------------------*/

#ifndef GLOBAL_H
#define GLOBAL_H


#include <vector>
#include <memory>
#include <Eigen/Eigen>
#include "SystemSetting/DebugLog.h"
#include "CommonFunction/Types.h"
#include "CommonFunction/CTimes.h"
#include "CommonFunction/CMaths.h"
#include "CommonFunction/CString.h"
#include "CommonFunction/CEarth.h"


using namespace std;
using namespace Eigen;

#define SQR(x)      ((x)*(x))
#define MAX(x,y)    ((x)>=(y)?(x):(y))
#define MIN(x,y)    ((x)>=(y)?(y):(x))


struct GNSSConf{
    int ProcessMode;          /* positioning mode, include spp, rtk and ppp  */
    int SolverMode;           /* lsq, ekf and go                             */
    int SolveType;            /* forward, backward amd smooth                */

    int NumFreq;                            /* number of frequencies        */
    int eph_sel[8] = { 0,0,0,0,0,0,0,0};    /* ephemeris selections N G C R E J I S */
    bool System[5] = {false};               // N G C R E

    STimes StartTime;
    STimes EndTime;

    vector<int> GPS_FreqID;
    vector<int> BD2_FreqID;
    vector<int> BD3_FreqID;
    vector<int> GLO_FreqID;
    vector<int> GAL_FreqID;

    vector<double> GPS_Freq;
    vector<double> BD2_Freq;
    vector<double> BD3_Freq;
    vector<double> GLO_Freq;
    vector<double> GAL_Freq;

    double SampleRate;
    int Ionospheric;        /* ionosphere option (IONOOPT_???)  */
    int Tropospheric;       /* troposphere option (TROPOPT_???) */
    int TropMapFunction;
    int CodeBias;
    double ErrRatio[3] = {100, 100, 100};   /* ratio of pseudo-range and carrier     */
    double Error[5] = {                     /* measurement error factor              */
                        0.003, 0.003, 0,    /* [1-3]:error factor a/b/c of phase (m) */
                          1.0,              /* [4]:doppler frequency (hz)            */
                         20.0 };            /* [5]: snr max value (dB.Hz)            */

    double Std[3] = {30, 0.03, 0.3};             /* initial-state std [0]bias,[1]iono [2]trop, (m)                     */
    double Process[5] = {0.0, 4e-2, 1e-4, 1, 3}; /* process-noise std [0]bias,[1]iono [2]trop [3]accbl [4]acch         */
    double SnrMask[9] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; /* SNR threshold of each elevation, [0-10] e.g. */
    double RobThres[3] = {5, 1.8, 4.0};          /* parameter of robust process [0]Number of iterations, [1]k0, [2]k1  */


    int WeightMode;
    int RobustMode;
    int VelocityMode;
    int BdsOption;
    double Elevation;     /* elevation mask angle (rad)         */
    double MaxResidual;   /* reject threshold of innovation (m) */

    bool AdjustObservation;
    string ExcludeSatellite;

    int PseudorangeSmooth;
    int PseudorangeSmoothWindow;
    double PseudorangeSmoothThreshold;

    int LossFunction;
    int SlideWindowSize;
    double LossFunctionValue;

    /* base station setting */
    Vector3d BasePos;
    Vector3d BaseBLH;

    /* File name settings */
    string GnssObs_File;
    string GnssObs_Base_File;
    string Nav_File;
    string Nav_Sys_File[4];  /* 0 for GPS, 1 for BDS, 2 for GLO, 3 for GAL*/
    string Ion_File;
    string Result_File;

    string Leapsec_File;
    string Temp_File;

    /* Result output settings */
    string sep;
    bool ExportHead;
    int ExportTimeFormat;
    int ExportTimeSystem;
    bool ExportVelocity;
    bool ExportAcceleration;
    bool ExportDop;
    bool ExportClock;
    bool ExportClockDrift;
    int ExportCoordinate;
    int ExportHeightDatum;
    int ExportBLHFormat;

    /* Debug settings */
    int DebugLevel;
    string DebugFile;
    bool DebugRobust;
    bool DebugResPos;
    bool DebugResVel;
};


extern GNSSConf config;
extern DebugLog Log;


#endif //GLOBAL_H
