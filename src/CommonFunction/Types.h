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
 * Created by lizhen on 2022/4/19.
 *-------------------------------------------------------------------------------*/

#ifndef TYPES_H
#define TYPES_H

#include <map>
#include <string>
#include <time.h>
#include <Eigen/Eigen>
#include "Parameter.h"

using std::map;
using std::string;
using std::vector;
using namespace Eigen;


struct STimes{
    time_t time = 0.0;         /* time (s) expressed by standard time_t */
    double sec = 0.0;          /* fraction of second under 1 s          */
    STimes(){}
    STimes(time_t _time, double _sec){
        time = _time;
        sec  = _sec;
    }
};


struct GnssNavHead{
    string type;
    double version;      /* version of rinex file                                  */
    double utc_gps[8];   /* GPS delta-UTC parameters {A0,A1,T,W}                   */
    double utc_bds[8];   /* BeiDou UTC parameters                                  */
    double utc_glo[8];   /* GLONASS UTC GPS time parameters                        */
    double utc_gal[8];   /* Galileo UTC GPS time parameters                        */
    double utc_qzs[8];   /* QZS UTC GPS time parameters                            */
    double ion_gps[8];   /* GPS iono model parameters {a0,a1,a2,a3,b0,b1,b2,b3}    */
    double ion_bds[8];   /* BeiDou iono model parameters {a0,a1,a2,a3,b0,b1,b2,b3} */
    double ion_gal[4];   /* Galileo iono model parameters {ai0,ai1,ai2,0}          */
    double ion_qzs[8];   /* QZSS iono model parameters {a0,a1,a2,a3,b0,b1,b2,b3}   */
};


struct Ephemeris{               /* GPS/QZS/GAL broadcast ephemeris type                   */
    int sat;                    /* satellite number                                       */
    int iode,iodc;              /* IODE,IODC                                              */
    int sva;                    /* SV accuracy (URA index)                                */
    int svh;                    /* SV health (0:ok)                                       */
    int week;                   /* GPS/QZS: gps week, GAL: galileo week                   */
    int code;                   /* GPS/QZS: code on L2                                    */
                                /* GAL: data source defined as rinex 3.03                 */
                                /* BDS: data source (0:unknown,1:B1I,2:B1Q,3:B2I,
                                                     4:B2Q,5:B3I,6:B3Q)                   */
    int flag;                   /* GPS/QZS: L2 P data flag                                */
                                /* BDS: nav type (0:unknown,1:IGSO/MEO,2:GEO)             */
    STimes toe,toc,ttr;         /* Toe,Toc,T_trans                                        */
                                /* SV orbit parameters                                    */
    double A,e,i0,OMG0,omg,M0,deln,OMGd,idot;
    double crc,crs,cuc,cus,cic,cis;
    double toes;                /* Toe (s) in week                                        */
    double fit;                 /* fit interval (h)                                       */
    double f0,f1,f2;            /* SV clock parameters (af0,af1,af2)                      */
    double tgd[6]={0};          /* group delay parameters                                 */
                                /* GPS/QZS:tgd[0]=TGD                                     */
                                /* GAL:tgd[0]=BGD_E1E5a,tgd[1]=BGD_E1E5b                  */
                                /* CMP:tgd[0]=TGD_B1I ,tgd[1]=TGD_B2I/B2b,tgd[2]=TGD_B1Cp */
                                /*     tgd[3]=TGD_B2ap,tgd[4]=ISC_B1Cd   ,tgd[5]=ISC_B2ad */
    double Adot,ndot;           /* Adot,ndot for CNAV                                     */
};


struct GEphemeris{              /* GLONASS broadcast ephemeris type             */
    int sat;                    /* satellite number                             */
    int iode;                   /* IODE (0-6 bit of tb field)                   */
    int frq;                    /* satellite frequency number                   */
    int svh,sva,age;            /* satellite health, accuracy, age of operation */
    STimes toe;                 /* epoch of epherides (gpst)                    */
    STimes tof;                 /* message frame time (gpst)                    */
    double pos[3]={0};          /* satellite position (ecef) (m)                */
    double vel[3]={0};          /* satellite velocity (ecef) (m/s)              */
    double acc[3]={0};          /* satellite acceleration (ecef) (m/s^2)        */
    double taun,gamn;           /* SV clock bias (s)/relative freq bias         */
    double dtaun;               /* delay between L1 and L2 (s)                  */
};


struct GnssNavData{
    int n,nmax;                                /* number of broadcast ephemeris            */
    int ng,ngmax;                              /* number of glonass ephemeris              */
    map<int, map<time_t,GEphemeris>> gloephs;  /** GPS/BDS/GAL ephemeris map after unique **/
    map<int, map<time_t,Ephemeris>> ephs;      /** GPS/BDS/GAL ephemeris map after unique **/
    vector<Ephemeris> eph;                     /* GPS/BDS/GAL ephemeris temp vector        */
    vector<GEphemeris> geph;                   /* GLONASS ephemeris temp vector            */
    GnssNavData(){
        n = ng = nmax = ngmax = 0;
    }
};


struct SatInfo{
    string prn;             /* PRN of satellite                                       */
    Vector3d pos;           /* satellite position                                     */
    Vector3d vel;           /* satellite velocity, obtained by position difference    */
    double dts[2] = {0.0};  /* clock and clock drift */
    double var = 0;
    double tgd[6] = {0.0};  /* GPS/QZS:tgd[0]=TGD                                     */
                            /* GAL:tgd[0]=BGD_E1E5a,tgd[1]=BGD_E1E5b                  */
                            /* CMP:tgd[0]=TGD_B1I ,tgd[1]=TGD_B2I/B2b,tgd[2]=TGD_B1Cp */
                            /*     tgd[3]=TGD_B2ap,tgd[4]=ISC_B1Cd   ,tgd[5]=ISC_B2ad */
    int svh = 0;            /* sign of satellite health                               */
    int glofcn = 0;         /* channel number of GLONASS                              */
    bool vaild = false;
};


struct IonTecGrid{      /* TEC grid type                       */
    STimes time;        /* epoch time (GPST)                   */
    int ndata[3];       /* TEC grid data size {nlat,nlon,nhgt} */
    double rb;          /* earth radius (km)                   */
    double lats[3];     /* latitude start/interval (deg)       */
    double lons[3];     /* longitude start/interval (deg)      */
    double hgts[3];     /* heights start/interval (km)         */
    double *data;       /* TEC grid data (tecu)                */
    float *rms;         /* RMS values (tecu)                   */
};


struct ReceiverInfo{          /* receiver information form obs rinex file    */
    string name = "";         /* marker name                                 */
    string marker = "";       /* marker number                               */
    string antdes = "";       /* antenna descriptor                          */
    string antsno = "";       /* antenna serial number                       */
    string rectype = "";      /* receiver type descriptor                    */
    string recver = "";       /* receiver firmware version                   */
    string recsno = "";       /* receiver serial number                      */
    int deltype = 0;          /* antenna delta type (0:enu,1:xyz)            */
    double version = 0;       /* version of rinex file                       */
    Vector3d pos;             /* station position (ecef) (m)                 */
    Vector3d del;             /* antenna position delta (e/n/u or x/y/z) (m) */
    GnssNavHead nav_head;
    int nIonTec;              /* number of tec grid data                     */
    IonTecGrid *IonTec = nullptr; /* tec grid data                           */
};


struct GnssObsData{
    string prn;                      /* satellite/receiver number               */
    int lock[NFREQ] = {0};           /* lock of continuity observation          */
    uint8_t freq;                    /* GLONASS frequency channel (0-13)        */
    uint8_t LLI [NFREQ+NEXOBS]={0};  /* loss of lock indicator                  */
    uint8_t sst [NFREQ+NEXOBS]={0};  /* Signal strength or Shadow matching sign */
    uint8_t code[NFREQ+NEXOBS]={0};  /* code indicator (CODE_???)               */
    uint16_t SNR [NFREQ+NEXOBS]={0}; /* signal strength (0.25 dBHz)             */
    double L[NFREQ+NEXOBS]={0};      /* observation data carrier-phase (cycle)  */
    double P[NFREQ+NEXOBS]={0};      /* observation data pseudorange (m)        */
    double D[NFREQ+NEXOBS]={0};      /* observation data doppler frequency (Hz) */
    double DL[NFREQ+NEXOBS]={0};     /* time difference of carrier-phase        */
    double r = 0;                    /* distance between the sat and rover      */
    double azimuth = 0;              /* azimuth angle between sat and rover     */
    double elevation = 0;            /* elevation angle between sat and rover   */
    double ion = 0.0;                /* ionospheric delay (L1) (m)              */
    double ion_var = 0.0;            /* ionospheric delay (L1) variance (m^2)   */
    double trp = 0.0;                /* tropospheric delay (m)                  */
    double trp_var = 0.0;            /* tropospheric delay variance (m^2)       */
    bool isGo = false;               /* vaild sat used for SPP GO solver        */
    Vector3d los;                    /* line of sight unit vectors to sats      */
    SatInfo sat;
};


struct GnssObsEpoch{              /* all observation in this epoch             */
    int num = 0;                  /* number of vaild satellite of this epoch   */
    int satnum = 0;               /* number of tracked satellite of this epoch */
    STimes time;                  /* receiver sampling time (GPST)             */
    map<int, GnssObsData> obsp;   /* observation of this epoch                 */
    map<int, GnssObsData> lobsp;  /* observation of last epoch                 */
};



struct SmoothSolution{    /* using for save forward or backward solution for smooth */
    int stat = 0;                         /* solution status of position            */
    int nsat = 0;                         /* number of each systen: ALL G R E C2 C3 */
    Vector3d pos  = Vector3d::Zero();     /* position in ecef coordinate system     */
    Vector3d vel  = Vector3d::Zero();     /* velicity in ecef coordinate system     */
    Vector3d Qpos = Vector3d::Zero();     /* variance of position in ecef           */
    Vector3d Qvel = Vector3d::Zero();     /* variance of velicity in ecef           */
};


struct Equation{
    int system;
    int freq;
    string prn;
    vector<double> H;
    double omc = 0;
    double R = 0;
    double Rb = 0;
    double el = 0;
    double snr = 0;
};


struct Solution{
    STimes time;                         /* time of soulution                  */
    int stat = 0;                        /* solution status of position        */
    int vstat = 0;                       /* solution status of velicity        */
    int Qc = 0;                          /* solution quality mark              */
    int nqc = 0;                         /* iteration of quality check number  */
    int epoch = 0;                       /* Total epoch                        */
    int nsat[7] = {0};                   /* number of each systen: ALL G R E C2 C3 Doppler */
    double drck[5] = {0};                /* GPS,BD2,GLO,GAL,BD3 receiver clock bias */
    double ldrck[5] = {0};               /* receiver clock of last epoch       */
    double dt = 0.0;                     /* time between this and last epoch   */
    double age = 0.0;                    /* age of differential (s)            */
    double drcd = 0.0;                   /* receiver clock drift               */
    double pdop = 0.0;                   /* PDOP value                         */

    Vector3d pos  = Vector3d::Zero();    /* position in ecef coordinate system */
    Vector3d blh  = Vector3d::Zero();    /* geodetic position                  */
    Vector3d vel  = Vector3d::Zero();    /* velicity in ecef coordinate system */
    Vector3d veln = Vector3d::Zero();    /* geodetic velicity                  */
    Vector3d veli = Vector3d::Zero();    /* instantaneous velicity in ecef     */
    Vector3d acc  = Vector3d::Zero();    /* acceleration in ecef sysytem       */

    Vector3d lpos  = Vector3d::Zero();   /* position in ecef of last epoch     */
    Vector3d lblh  = Vector3d::Zero();   /* geodetic position of last epoch    */
    Vector3d lvel  = Vector3d::Zero();   /* velicity in ecef of last epoch     */
    Vector3d lveln = Vector3d::Zero();   /* geodetic velicity of last epoch    */

    Matrix<double, Dynamic, Dynamic> Qpos;  /* variance of position            */
    Matrix<double, Dynamic, Dynamic> Qvel;  /* variance of velicity            */
    Matrix<double, Dynamic, Dynamic> Qveli; /* variance of acceleration        */

    VectorXd pdx;                           /* for lsq solver use */
    VectorXd vdx;                           /* for lsq solver use */
};

#endif //TYPES_H
