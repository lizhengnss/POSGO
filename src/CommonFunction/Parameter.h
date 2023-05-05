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

#ifndef PARAMETER_H
#define PARAMETER_H


enum SOFTWARE_VERSION{
    MAJOR_VERSION = 1,
    MINOR_VERSION = 0,
};


/* Trace debug export level--------------------------------------------*/
enum DEBUG_LEVEL{
    TERROR = 1,
    TINFO,
    TWARNING,
    TDEBUG,
    TEXPORT,
};


/* Colour setting for output ------------------------------------------*/
#define RED     "\033[31m"      /* Red    */
#define GREEN   "\033[32m"      /* Green  */
#define YELLOW  "\033[33m"      /* Yellow */


/* Mathematical constants parameter -----------------------------------*/
#define PI        3.1415926535897932    /* pi                    */
#define D2R       (PI/180.0)            /* deg to rad            */
#define R2D       (180.0/PI)            /* rad to deg            */
#define CLIGHT    299792458.0           /* speed of light (m/s)  */


#define NFREQ             7         /* number of carrier frequencies          */
#define NEXOBS            7         /* number of extended mObs codes          */
#define MAXFREQ           7         /* max NFREQ                              */
#define NUMSYS            5         /* number of systems                      */
#define MAXCODESTRING    70         /* max number of observation code strings */
#define MAXOBSTYPE       64         /* max number of obs type in RINEX        */
#define SNR_UNIT      0.001         /* SNR unit (dBHz)                        */
#define MAX_PDOP       20.0         /* threshold of dop in quality check      */


#define VAR_POS     SQR(30.0)       /* initial variance of receiver pos (m^2)      */
#define VAR_VEL     SQR(10.0)       /* initial variance of receiver vel ((m/s)^2)  */
#define VAR_ACC     SQR(10.0)       /* initial variance of receiver acc ((m/ss)^2) */
#define VAR_CLK     SQR(30.0)       /* init variance receiver clock (m^2)          */
#define VAR_CDR     SQR(10.0)       /* init variance of receiver click drift       */

/* Frequency of satellite carrier phase---------------------------------------------*/
#define FREQ1       1.57542E9           /* L1/E1/B1C  frequency (Hz)        */
#define FREQ2       1.22760E9           /* L2         frequency (Hz)        */
#define FREQ5       1.17645E9           /* L5/E5a/B2a frequency (Hz)        */
#define FREQ6       1.27875E9           /* E6/L6  frequency (Hz)            */
#define FREQ7       1.20714E9           /* E5b    frequency (Hz)            */
#define FREQ8       1.191795E9          /* E5a+b/B2a+b  frequency (Hz)      */
#define FREQ9       2.492028E9          /* S      frequency (Hz)            */
#define FREQ1_GLO   1.60200E9           /* GLONASS G1 base frequency (Hz)   */
#define DFRQ1_GLO   0.56250E6           /* GLONASS G1 bias frequency (Hz/n) */
#define FREQ2_GLO   1.24600E9           /* GLONASS G2 base frequency (Hz)   */
#define DFRQ2_GLO   0.43750E6           /* GLONASS G2 bias frequency (Hz/n) */
#define FREQ3_GLO   1.202025E9          /* GLONASS G3 frequency (Hz)        */
#define FREQ1a_GLO  1.600995E9          /* GLONASS G1a frequency (Hz)       */
#define FREQ2a_GLO  1.248060E9          /* GLONASS G2a frequency (Hz)       */
#define FREQ1_CMP   1.561098E9          /* BDS B1I     frequency (Hz)       */
#define FREQ2_CMP   1.20714E9           /* BDS B2I/B2b frequency (Hz)       */
#define FREQ3_CMP   1.26852E9           /* BDS B3      frequency (Hz)       */

/* Satellite number of each system ----------------------------------------------------*/
#define MINPRNGPS   1                   /* min satellite PRN number of GPS      */
#define MAXPRNGPS   32                  /* max satellite PRN number of GPS      */
#define NSATGPS     (MAXPRNGPS-MINPRNGPS+1) /* number of GPS satellites         */

#define MINPRNBDS   1                   /* min satellite sat number of BeiDou   */
#define MAXPRNBDS   60                  /* max satellite sat number of BeiDou   */
#define NSATBDS     (MAXPRNBDS-MINPRNBDS+1) /* number of BeiDou satellites      */

#define MINPRNGLO   1                   /* min satellite slot number of GLONASS */
#define MAXPRNGLO   27                  /* max satellite slot number of GLONASS */
#define NSATGLO     (MAXPRNGLO-MINPRNGLO+1) /* number of GLONASS satellites     */

#define MINPRNGAL   1                   /* min satellite PRN number of Galileo  */
#define MAXPRNGAL   36                  /* max satellite PRN number of Galileo  */
#define NSATGAL    (MAXPRNGAL-MINPRNGAL+1) /* number of Galileo satellites      */

#define MAXSAT (NSATGPS + NSATBDS + NSATGLO + NSATGAL)


/* Index of each GNSS system--------------------------------------------------------*/
enum SYSTEM_INDEX{
    SYS_NONE = 0,              /* navigation system: none    */
    SYS_GPS,                   /* navigation system: GPS     */
    SYS_BDS,                   /* navigation system: BeiDou2 */
    SYS_GLO,                   /* navigation system: GLONASS */
    SYS_GAL,                   /* navigation system: Galileo */
    SYS_BD3,                   /* navigation system: BeiDou3 */
    SYS_QZS,                   /* navigation system: QZSS    */
    SYS_ALL,                   /* navigation system: all     */
};


enum OBSTYPE_INDEX{
    CARRIERPHASE = 0,
    PSEUDORANGE1,
    PSEUDORANGE2,
    PSEUDORANGE3,
    DOPPLER,
    TDCP,
};


/* Solution state parameter --------------------------------------------------------*/
enum SOLUTION_STATE{
    SOLQ_NONE = 0,           /* solution status: no solution, 0       */
    SOLQ_FIX,                /* solution status: fix, 1               */
    SOLQ_FLOAT,              /* solution status: float, 2             */
    SOLQ_FIXWL,              /* solution status: fix (e)wide-lane, 3  */
    SOLQ_FIXFLOAT,           /* solution status: fix-float, smooth, 4 */
    SOLQ_SINGLE,             /* solution status: single, 5            */
};


/* Velocity state parameter --------------------------------------------------------*/
enum VELOCITY_STATE{
    VELQ_NONE = 0,           /* velocity status: no solution   */
    VELQ_DOPPLER,            /* solution status: doppler       */
    VELQ_TDCP,               /* solution status: TDCP          */
};


/* quality check state ------------------------------------------------------------*/
enum QUALITY_STATE{
    QC_GOOD = 0,    /* solution quality is good             */
    QC_CHISQR,      /* failed to pass Chi-square validation */
    QC_LACK,        /* lack satellite                       */
    QC_PDOP,        /* soulution with large PDOP            */
    QC_ROBUST,      /* robust solution failed               */
    QC_BADVEL,      /* velocity bad                         */
};


/* Type of least squares solver ----------------------------------------------------*/
enum LSQSOLVE_TYPE{
    LSQ_SOLVE_POS = 0,
    LSQ_SOLVE_VEL,
};


enum ROBUST_TYPE{
    ROBUST_REJECT = 0,
    ROBUST_REDUCE,
    ROBUST_CONRED,  // continuous redeced
};


/* Configure parameter options ----------------------------------------------------*/
/* Switch ----------------------------------------------------------------*/
enum SWITCH_TYPE{
    SWITCH_OFF = 0,
    SWITCH_ON,
};

/* GNSS solve mode--------------------------------------------------------*/
enum GNSS_SOLVE_MODE{
    PMODE_SINGLE = 0,       /* positioning mode: single */
    PMODE_RTK,              /* positioning mode: rtk    */
    PMODE_PPP,              /* positioning mode: ppp    */
};

/* Process mode-----------------------------------------------------------*/
enum PROCESS_MODE{
    MODE_FORWARD = 0,       /* forward  */
    MODE_BACKWARD,          /* backward */
    MODE_SMOOTH,            /* RTS      */
};

/* Estimator option-------------------------------------------------------*/
enum ESTIMATE_MODE{
    SOLVER_LSQ  = 0,        /* least square estimator           */
    SOLVER_EKF,             /* extended Kalman filter estimator */
    SOLVER_GO,              /* graph optimization estimator     */
};

/* GNSS velocity option---------------------------------------------------*/
enum VELOCITY_MODE{
    VELOPT_OFF = 0,         /* no GNSS velocity mObs    */
    VELOPT_DOPPLER,         /* doppler observation      */
    VELOPT_TDCP,            /* time difference of phase */
};

/* BDS option-------------------------------------------------------------*/
enum BDS_OPTION{
    BDSOPT_BD2 = 0,         /* only BDS2        */
    BDSOPT_BD3,             /* only BDS3        */
    BDSOPT_BD23,            /* BD23 as a system */
    BDSOPT_BD2_3,           /* BD23 as 2 system */
};

/* Code bias correction option--------------------------------------------*/
enum CODEBIAS_OPTION{
    CODEBIAS_OFF = 0,       /* correction off */
    CODEBIAS_TGD,           /* broadcast TGD  */
    CODEBIAS_DCB,           /* precise DCB    */
};

/* Ionosphere option------------------------------------------------------*/
enum IONOSPHERE_OPTION{
    IONOOPT_OFF = 0,       /* correction off     */
    IONOOPT_BRDC,          /* broadcast model    */
    IONOOPT_IFLC,          /* L1/L2 iono-free LC */
    IONOOPT_TEC,           /* IONEX TEC model    */
};

/* Troposphere mode option------------------------------------------------*/
enum TROPOSPHERE_OPTION{
    TROPOPT_OFF = 0,       /* correction off     */
    TROPOPT_SAAS,          /* Saastamoinen model */
};

/* Tropospheric mapping function option-----------------------------------*/
enum TROPO_MAPFUN_OPTION{
    TROPMAP_NMF = 0,
    TROPMAP_GMF,
    TROPMAP_VMF,
};

/* Weighting option-------------------------------------------------------*/
enum WEIGHT_OPTION{
    WEIGHTOPT_ELEVATION = 0,    /* elevation */
    WEIGHTOPT_SNR,              /* snr       */
};

/* Robust option----------------------------------------------------------*/
enum ROBUST_OPTION{
    ROBUST_OFF = 0,       /* robust off            */
    ROBUST_IGG3,          /* IGG3 kernel function  */
    ROBUST_HUBER,         /* Huber kernel function */
};

/* Pseudorange Smooth---------------------------------------------------- */
enum PSESMOOTH_OPTION{
    SMOOTH_OFF = 0,       /* smooth off            */
    SMOOTH_DOPPLER,       /* smooth using doppler  */
    SMOOTH_PHASE,         /* smooth using phase    */
};

/* FG lossfuncion option--------------------------------------------------*/
enum LOSSFUN_OPTION{
    FGLOSSFUN_NULL = 0,
    FGLOSSFUN_HUBER,
    FGLOSSFUN_CAUCHY,
    FGLOSSFUN_ARCTAN,
    FGLOSSFUN_TUKEY,
    FGLOSSFUN_SOFTONE,
};


/* Solution format option-------------------------------------------------*/
enum SOLUTION_FORMAT{
    SOLF_XYZ = 0,          /* x/y/z-ecef     */
    SOLF_BLH,              /* lat/lon/height */
};

/* Format of longitude and latitude---------------------------------------*/
enum BLF_FORMAT{
    BLHF_DEG = 0,         /* degree.              */
    BLHF_DMS,             /* degree-minute-second */
};

/* Datum option-----------------------------------------------------------*/
enum DATUM_OPTION{
    HEIGHT_ELLIPS = 0,    /* Ellipsoid height */
    HEIGHT_GEODETIC,      /* geodetic height  */
};

/* time system option-----------------------------------------------------*/
enum TIME_OPTION{
    TIMES_GPST = 0,       /* gps time           */
    TIMES_UTC,            /* utc                */
    TIMES_BST,            /* beijing local time */
};

/* time format option-----------------------------------------------------*/
enum TIME_FORMAT{
    TIMEF_TOW = 0,        /* wwww [sep] ssssss.sss   */
    TIMEF_HMS,            /* yyyy/mm/dd hh:mm:ss.sss */
};


#endif //PARAMETER_H
