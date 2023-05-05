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
 * Created by lizhen on 2021/7/22.
 *-------------------------------------------------------------------------------*/

#ifndef READGNSSPRODUCT_H
#define READGNSSPRODUCT_H

#include "ReadFile.h"

#define MINFREQ_GLO -7                /* min frequency number GLONASS */
#define MAXFREQ_GLO 13                /* max frequency number GLONASS */

class ReadRinexNav:public ReadFile{
public:
    ReadRinexNav()  = default;
    ~ReadRinexNav() = default;
    bool ReadHead() override;    /* read nav file(brdc) head           */
    bool Read() override;        /* read all satellite's nav ephemeris */
    GnssNavData GetNavData();    /* Interface of get nav data          */
    GnssNavHead GetNavHead();    /* Interface of get nav head          */
    void UniqueNav();            /* delete duplicated ephemeris        */

private:
    using sateph  = map<time_t,Ephemeris>;
    using gsateph = map<time_t,GEphemeris>;

    GnssNavData mNavs;          /** ephemeris from nav, for next step of calculation **/
    GnssNavHead mHead;          /** head of navigation file                          **/
    bool mEndOfFile;            /* whether read to the end of the file                */
    bool mIsVaild;              /* whether is a vaild ephemeris data                  */

    STimes mToc;                /* time for each ephemeris(in the head of each data)  */
    int mSatId;                 /* satellite id,G(1-32),C(33-82),R(83-109),E(110-145) */
    int mEphSystem;             /* system of each ephemeris                           */
    vector<double> mData;       /* temporary data when read nav file                  */
    Ephemeris mEph_t;           /* temporary data using to save GCE ephemeris         */
    GEphemeris mGeph_t;         /* temporary data using to save R ephemeris           */

    bool ReadEachEph();              /* read ephemeris and save to [data]             */
    bool AddEachEph();               /* save ephemeris to nav, include GCE and R      */

    bool Decode_Eph();               /* decode ephemeris form [data], for GPS/BDS/GAL */
    void AddEph();                   /* save GPS/BDS/GAL ephemeris to nav.eph         */
    void UniqueEph();                /* sort and unique GPS/BDS/GAL ephemeris         */

    bool Decode_Eph_GLO();           /* decode ephemeris form [data], for GLO         */
    void AddGLOEph();                /* save GLO ephemeris to nav.geph                */
    void UniqueGeph();               /* sort and unique glonass ephemeris             */

    int uraindex(double value);      /* URA value (m) to URA index                    */
    int sisa_index(double value);    /* Galileo SISA value (m) to SISA index          */
};


class ReadIonTecGrid:public ReadFile{
public:
    ReadIonTecGrid()  = default;
    ~ReadIonTecGrid() = default;

    int GetIonTecDataNumber();
    IonTecGrid* GetIonTecData();                      /* Interface of get GIM data          */

private:
    bool ReadHead() override;                         /* read ionex header */
    bool Read() override;                             /* read ionex body   */

    int mNt, mNtmax;                                  /* number of tec grid data            */
    double mVersion;                                  /* version of ionex file              */
    double mRb;                                       /* Average earth radius               */
    double mLats[3]={0}, mLons[3]={0}, mHgts[3]={0};  /* begin, end and step length         */
    double mNexp = -1.0;                              /* unit exponential, -1 mean 0.1 TECU */
    IonTecGrid* mIonTec = nullptr;                    /* tec grid data                      */

    IonTecGrid* AddTec();
    void CombineTec();

private:
    static int Nitem(const double *range);                       /* get number of items            */
    static int GetIndex(double value, const double *range);      /* get index                      */
    static int DataIndex(int i, int j, int k, const int *ndata); /* data index (i:lat,j:lon,k:hgt) */

};


#endif //READGNSSPRODUCT_H
