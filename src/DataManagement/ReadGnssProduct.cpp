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
 *   [1]. IS-GPS-200D, Navstar GPS Space Segment/Navigation User Interfaces,
 *            7 March, 2006
 * Created by lizhen on 2021/7/22.
 *-------------------------------------------------------------------------------*/

#include "ReadGnssProduct.h"


bool ReadRinexNav::ReadHead(){
    int i, j;
    while (true)
    {
        getline(mInfile, mBuff);
        if (mBuff.find("END OF HEADER") != string::npos) break;
        if (EndFile()) return false;
        mTempdata =  CString::split(mBuff, " ");

        if (mBuff.find("RINEX VERSION / TYPE"          ) != string::npos)
        {
            mHead.version = CString::str2num(mBuff, 0, 9);
            mHead.type = mBuff.substr(40, 1);
        }
        else if (mBuff.find("ION ALPHA"              ) != string::npos)
        {   /* opt ver.2 */
            for (i = 0, j = 2; i < 4; i++, j += 12) mHead.ion_gps[i] = CString::str2num(mBuff, j, 12);
        }
        else if (mBuff.find("ION BETA"               ) != string::npos)
        {   /* opt ver.2 */
            for (i=0,j=2;i<4;i++,j+=12) mHead.ion_gps[i + 4] = CString::str2num(mBuff, j, 12);
        }
        else if (mBuff.find("DELTA-UTC: A0,A1,T,W"   ) != string::npos)
        {   /* opt ver.2 */
            for (i = 0, j = 3; i < 2; i++, j += 19) mHead.utc_gps[i] = CString::str2num(mBuff, j, 19);
            for (; i < 4; i++, j += 9) mHead.utc_gps[i] = CString::str2num(mBuff, j, 9);
        }
        else if (mBuff.find("IONOSPHERIC CORR"       ) != string::npos)
        {   /* opt ver.3 */
            if (mBuff.find("GPSA") != string::npos)
            {
                for (i = 0, j = 5; i < 4; i++, j += 12) mHead.ion_gps[i    ] = CString::str2num(mBuff, j, 12);
            }
            else if (mBuff.find("GPSB") != string::npos)
            {
                for (i = 0, j = 5; i < 4; i++, j += 12) mHead.ion_gps[i + 4] = CString::str2num(mBuff, j, 12);
            }
            else if (mBuff.find("BDSA") != string::npos)
            {   /* v.3.02 */
                for (i = 0, j = 5; i < 4; i++, j += 12) mHead.ion_bds[i    ] = CString::str2num(mBuff, j, 12);
            }
            else if (mBuff.find("BDSB") != string::npos)
            {   /* v.3.02 */
                for (i = 0, j = 5; i < 4; i++, j += 12) mHead.ion_bds[i + 4] = CString::str2num(mBuff, j, 12);
            }
            else if (mBuff.find("GAL" ) != string::npos)
            {
                for (i = 0, j = 5; i < 4; i++, j += 12) mHead.ion_gal[i    ] = CString::str2num(mBuff, j, 12);
            }
            else if (mBuff.find("QZSA") != string::npos)
            {   /* v.3.02 */
                for (i = 0, j = 5; i < 4; i++, j += 12) mHead.ion_qzs[i    ] = CString::str2num(mBuff, j, 12);
            }
            else if (mBuff.find("QZSB") != string::npos)
            {   /* v.3.02 */
                for (i = 0, j = 5; i < 4; i++, j += 12) mHead.ion_qzs[i + 4] = CString::str2num(mBuff, j, 12);
            }
        }
        else if (mBuff.find("TIME SYSTEM CORR"       ) != string::npos)
        {   /* opt ver.3 */
            if (mBuff.find("GPUT") != string::npos)
            {
                mHead.utc_gps[0] = CString::str2num(mBuff, 5, 17);
                mHead.utc_gps[1] = CString::str2num(mBuff, 22, 16);
                mHead.utc_gps[2] = CString::str2num(mBuff, 38, 7);
                mHead.utc_gps[3] = CString::str2num(mBuff, 45, 5);
            }
            else if (mBuff.find("BDUT") != string::npos)
            {   /* v.3.02 */
                mHead.utc_bds[0] = CString::str2num(mBuff, 5, 17);
                mHead.utc_bds[1] = CString::str2num(mBuff, 22, 16);
                mHead.utc_bds[2] = CString::str2num(mBuff, 38, 7);
                mHead.utc_bds[3] = CString::str2num(mBuff, 45, 5);
            }
            else if (mBuff.find("GLUT") != string::npos)
            {
                mHead.utc_glo[0] = -CString::str2num(mBuff, 5, 17); /* tau_C */
            }
            else if (mBuff.find("GLGP") != string::npos)
            {
                mHead.utc_glo[1] = CString::str2num(mBuff, 5, 17); /* tau_GPS */
            }
            else if (mBuff.find("GAUT") != string::npos)
            {   /* v.3.02 */
                mHead.utc_gal[0] = CString::str2num(mBuff, 5, 17);
                mHead.utc_gal[1] = CString::str2num(mBuff, 22, 16);
                mHead.utc_gal[2] = CString::str2num(mBuff, 38, 7);
                mHead.utc_gal[3] = CString::str2num(mBuff, 45, 5);
            }
            else if (mBuff.find("QZUT") != string::npos)
            {   /* v.3.02 */
                mHead.utc_qzs[0] = CString::str2num(mBuff, 5, 17);
                mHead.utc_qzs[1] = CString::str2num(mBuff, 22, 16);
                mHead.utc_qzs[2] = CString::str2num(mBuff, 38, 7);
                mHead.utc_qzs[3] = CString::str2num(mBuff, 45, 5);
            }
        }
        else if (mBuff.find("LEAP SECONDS"           ) != string::npos)
        {   /* opt */
            mHead.utc_gps[4]=CString::str2num(mBuff, 0, 6);
            mHead.utc_gps[7]=CString::str2num(mBuff, 6, 6);
            mHead.utc_gps[5]=CString::str2num(mBuff, 12, 6);
            mHead.utc_gps[6]=CString::str2num(mBuff, 18, 6);
        }

    }
    return true;
}

bool ReadRinexNav::Read() {
    while (true)
    {
        mEndOfFile = this->ReadEachEph();
        if (!mEndOfFile) break;
        if (mEphSystem == SYS_GLO)
        {
            mIsVaild = this->Decode_Eph_GLO();
        }
        else
        {
            mIsVaild = this->Decode_Eph();
        }
        if (mIsVaild) this->AddEachEph();
    }

    if (mNavs.n <= 0 && mNavs.ng <= 0)
    {
        Log.Trace(TERROR, "*ReadNav: no nav data");
        return false;
    }

    return true;
}

GnssNavData ReadRinexNav::GetNavData(){
    return this->mNavs;
}

GnssNavHead ReadRinexNav::GetNavHead(){
    this->ReadHead();
    return this->mHead;
}


bool ReadRinexNav::ReadEachEph(){
    int i = 0, sp = 3, j, p;
    mData.clear();
    while (true)
    {
        getline(mInfile, mBuff);
        if (EndFile()) return false;
        if (i == 0)
        {   // first line of each eph
            /* decode satellite field */
            if (mHead.version >= 3.0 || mHead.type == "E" || mHead.type == "J")
            {   /* ver.3 or GAL/QZS */
                mSatId = CString::satprn2id(mBuff.substr(0, 3));
                sp = 4;
                if (mHead.version >= 3.0)
                {
                    mEphSystem = CString::satsys(mBuff.substr(0, 3));
                    if (!mEphSystem)
                    {
                        // todo sys=(id[0]=='S')?SYS_SBS:((id[0]=='R')?SYS_GLO:SYS_GPS);
                    }
                }
            }
            else
            {
                // todo v2.x
            }

            /* decode Toc field */
            mToc = CTimes::str2time(mBuff.substr(sp, 19));
            /* decode data fields */
            for (j = 0, p = sp + 19; j < 3; j++, i++, p += 19)
            {
                mData.push_back(CString::str2num(mBuff, p, 19));
            }
        }
        else
        {   // leftover line of each eph
            /* decode data fields */
            for (j=0, p = sp; j < 4; j++, i++, p += 19)
            {
                if (p > mBuff.length()) continue;
                mData.push_back(CString::str2num(mBuff, p, 19));
            }
            /* decode ephemeris */
            if ((mEphSystem == SYS_GLO && i >= 15) || i >= 31)
            {   // GLONASS eph
                return true;
            }
        }

    }
}

void ReadRinexNav::UniqueNav(){
    /* unique ephemeris */
    this->UniqueEph ();
    this->UniqueGeph();
}

bool ReadRinexNav::AddEachEph(){
    switch (mEphSystem)
    {
        case SYS_GLO: this->AddGLOEph(); break;
        case SYS_GPS:
        case SYS_BDS:
        case SYS_GAL: this->AddEph(); break;
        case SYS_QZS: break; // todo
        default: Log.Trace(TWARNING, "AddEachEph: wrong system flag " + to_string(mEphSystem));
            return false;
    }
    return true;
}

bool ReadRinexNav::Decode_Eph(){

    if (!(mEphSystem & (SYS_GPS | SYS_GAL | SYS_QZS | SYS_BDS)))
    {
        Log.Trace(TWARNING, "Decode_Eph: ephemeris error, invalid satellite sat=" + to_string(mSatId));
        return false;
    }

    mEph_t.sat = mSatId;
    mEph_t.toc = mToc;

    mEph_t.f0 = mData[0];
    mEph_t.f1 = mData[1];
    mEph_t.f2 = mData[2];

    mEph_t.A    = SQR(mData[10]); mEph_t.e   = mData[ 8]; mEph_t.i0   = mData[15]; mEph_t.OMG0 = mData[13];
    mEph_t.omg  = mData[17]; mEph_t.M0  = mData[ 6]; mEph_t.deln = mData[ 5]; mEph_t.OMGd = mData[18];
    mEph_t.idot = mData[19]; mEph_t.crc = mData[16]; mEph_t.crs  = mData[ 4]; mEph_t.cuc  = mData[ 7];
    mEph_t.cus  = mData[ 9]; mEph_t.cic = mData[12]; mEph_t.cis  = mData[14];

    if (mEphSystem == SYS_GPS || mEphSystem == SYS_QZS)
    {
        mEph_t.iode = (int)mData[ 3];      /* IODE */
        mEph_t.iodc = (int)mData[26];      /* IODC */
        mEph_t.toes =      mData[11];      /* Toe (s) in GPS week */
        mEph_t.week = (int)mData[21];      /* GPS week */
        mEph_t.toe  = CTimes::adjweek(CTimes::gpst2time(mEph_t.week, mData[11]), mToc);
        mEph_t.ttr  = CTimes::adjweek(CTimes::gpst2time(mEph_t.week, mData[27]), mToc);

        mEph_t.code = (int)mData[20];      /* GPS: codes on L2 ch */
        mEph_t.svh  = (int)mData[24];      /* SV health */
        mEph_t.sva  = uraindex(mData[23]); /* URA index (m->index) */
        mEph_t.flag = (int)mData[22];      /* GPS: L2 P data flag */

        mEph_t.tgd[0] =   mData[25];       /* TGD */
        if (mEphSystem == SYS_GPS)
        {
            mEph_t.fit = mData[28];        /* fit interval (h) */
        }
        else
        {
            mEph_t.fit = mData[28] == 0.0 ? 1.0 : 2.0; /* fit interval (0:1h,1:>2h) */
        }
    }
    else if (mEphSystem == SYS_GAL)
    {   /* GAL ver.3 */
        mEph_t.iode = (int)mData[ 3];      /* IODnav */
        mEph_t.toes =      mData[11];       /* Toe (s) in Galileo week */
        mEph_t.week = (int)mData[21];      /* Galileo week = GPS week */
        mEph_t.toe  = CTimes::adjweek(CTimes::gpst2time(mEph_t.week, mData[11]), mToc);
        mEph_t.ttr  = CTimes::adjweek(CTimes::gpst2time(mEph_t. week, mData[27]), mToc);

        mEph_t.code = (int)mData[20];    /* data sources */
        /* bit 0 set: I/NAV E1-B */
        /* bit 1 set: F/NAV E5a-I */
        /* bit 2 set: F/NAV E5b-I */
        /* bit 8 set: af0-af2 mToc are for E5a.E1 */
        /* bit 9 set: af0-af2 mToc are for E5b.E1 */
        mEph_t.svh =(int)mData[24];      /* sv health */
        /* bit     0: E1B DVS */
        /* bit   1-2: E1B HS */
        /* bit     3: E5a DVS */
        /* bit   4-5: E5a HS */
        /* bit     6: E5b DVS */
        /* bit   7-8: E5b HS */
        mEph_t.sva = sisa_index(mData[23]); /* sisa (m->index) */

        mEph_t.tgd[0] =   mData[25];      /* BGD E5a/E1 */
        mEph_t.tgd[1] =   mData[26];      /* BGD E5b/E1 */
    }
    else if (mEphSystem == SYS_BDS)
    {   /* BeiDou v.3.02 */
        mEph_t.toc  = CTimes::bdt2gpst(mEph_t.toc);  /* bdt -> gpst */
        mEph_t.iode = (int)mData[ 3];      /* AODE */
        mEph_t.iodc = (int)mData[28];      /* AODC */
        mEph_t.toes =     mData[11];       /* Toe (s) in BDT week */
        mEph_t.week = (int)mData[21];      /* bdt week */
        mEph_t.toe  = CTimes::bdt2gpst(CTimes::bdt2time(mEph_t.week, mData[11])); /* BDT -> GPST */
        mEph_t.ttr  = CTimes::bdt2gpst(CTimes::bdt2time(mEph_t.week, mData[27])); /* BDT -> GPST */
        mEph_t.toe  = CTimes::adjweek(mEph_t.toe, mToc);
        mEph_t.ttr  = CTimes::adjweek(mEph_t.ttr, mToc);

        mEph_t.svh = (int)mData[24];      /* satH1 */
        mEph_t.sva = uraindex(mData[23]);  /* URA index (m->index) */

        mEph_t.tgd[0] =   mData[25];      /* TGD1 B1/B3 */
        mEph_t.tgd[1] =   mData[26];      /* TGD2 B2/B3 */
    }

    if (mEph_t.iode < 0 || 1023 < mEph_t.iode)
    {
        Log.Trace(TWARNING, "Decode_Eph: rinex nav invalid: sat=" + to_string(mSatId) + ", iode=" + to_string(mEph_t.iode));
        return false;
    }

    if (mEph_t.iodc < 0 || 1023 < mEph_t.iodc)
    {
        Log.Trace(TWARNING, "Decode_Eph: rinex nav invalid: sat=" + to_string(mSatId) + ", iodc=" + to_string(mEph_t.iodc));
        return false;
    }
    return true;
}

void ReadRinexNav::UniqueEph(){
    int n = 0;

    if (mNavs.n <= 0) return;
    for(auto& eph0 : mNavs.eph)
    {   // unique ephs
        if(mNavs.ephs.find(eph0.sat) == mNavs.ephs.end()) mNavs.ephs[eph0.sat] = sateph();
        sateph& ephs = mNavs.ephs[eph0.sat];
        if (eph0.sat > NSATGPS + NSATBDS + NSATGLO)
        {   // GAL
            if (config.eph_sel[SYS_GAL] == 0 &&! (eph0.code & (1<<9))) continue; /* I/NAV */
            if (config.eph_sel[SYS_GAL] == 1 &&! (eph0.code & (1<<8))) continue; /* F/NAV */
            if (ephs.find(eph0.toe.time) == ephs.end()) ephs[eph0.toe.time] = eph0;
        }
        else
        {
            if(ephs.find(eph0.toe.time) == ephs.end()) ephs[eph0.toe.time] = eph0;
        }

    }

    for (auto iter = mNavs.ephs.begin(); iter != mNavs.ephs.end(); iter++)
    {   // count for eph number
        n += iter->second.size();
    }

    mNavs.eph.clear();
    mNavs.n = n;
    mNavs.nmax = mNavs.n;
}

void ReadRinexNav::AddEph(){
    mNavs.eph.push_back(mEph_t);
    mNavs.n += 1;
}

bool ReadRinexNav::Decode_Eph_GLO(){
    int week,dow;
    double tow,tod;
    STimes tof;

    mGeph_t.sat = mSatId;

    /* Toc rounded by 15 min in utc */
    tow = CTimes::time2gpst(mToc, &week);
    //  mToc = gpst2time(week, floor((tow + 450.0) / 900.0) * 900); no use
    dow = (int)floor(tow / 86400.0);

    /* time of frame in UTC */
    tod = mHead.version <= 2.99 ? mData[2] : fmod(mData[2], 86400.0); /* Tod (v.2), Tow (v.3) in UTC */
    tof = CTimes::gpst2time(week,tod + dow * 86400.0);
    tof = CTimes::adjday(tof, mToc);

    mGeph_t.toe = CTimes::utc2gpst(mToc);  /* Toc (GPST) */
    mGeph_t.tof = CTimes::utc2gpst(tof);   /* Tof (GPST) */

    /* IODE = Tb (7bit), Tb =index of UTC+3H within current day */
    mGeph_t.iode = (int)(fmod(tow + 10800.0, 86400.0) / 900.0 + 0.5);

    mGeph_t.taun = -mData[0];       /* -taun */
    mGeph_t.gamn =  mData[1];       /* +gamman */

    mGeph_t.pos[0] = mData[3] * 1E3; mGeph_t.pos[1] = mData[7] * 1E3; mGeph_t.pos[2] = mData[11] * 1E3;
    mGeph_t.vel[0] = mData[4] * 1E3; mGeph_t.vel[1] = mData[8] * 1E3; mGeph_t.vel[2] = mData[12] * 1E3;
    mGeph_t.acc[0] = mData[5] * 1E3; mGeph_t.acc[1] = mData[9] * 1E3; mGeph_t.acc[2] = mData[13] * 1E3;

    mGeph_t.svh = (int)mData[ 6];
    mGeph_t.frq = (int)mData[10];
    mGeph_t.age = (int)mData[14];

    /* some receiver output >128 for minus frequency number */
    if (mGeph_t.frq > 128) mGeph_t.frq-=256;

    if (mGeph_t.frq < MINFREQ_GLO || MAXFREQ_GLO < mGeph_t.frq)
    {
        Log.Trace(TWARNING, "Decode_Eph_GLO: rinex glonav invalid freq: sat=" + to_string(mSatId)
                     + ", fn=" + to_string(mGeph_t.frq));
        return false;
    }
    return true;
}

void ReadRinexNav::UniqueGeph(){
    int n = 0;
    map<int, map<time_t,GEphemeris>>:: iterator iter;

    if (mNavs.ng <= 0) return;

    for(auto& eph0 : mNavs.geph)
    {   // unique ephs
        if(mNavs.gloephs.find(eph0.sat) == mNavs.gloephs.end()) mNavs.gloephs[eph0.sat] = gsateph();
        gsateph& ephs = mNavs.gloephs[eph0.sat];
        if(ephs.find(eph0.toe.time) == ephs.end()) ephs[eph0.toe.time] = eph0;
    }

    for (iter = mNavs.gloephs.begin(); iter != mNavs.gloephs.end(); iter++)
    {
        n += iter->second.size();
    }  // count for eph number

    mNavs.geph.clear();
    mNavs.ng = n;
    mNavs.ngmax = mNavs.ng;
}

void ReadRinexNav::AddGLOEph(){
    mNavs.geph.push_back(mGeph_t);
    mNavs.ng += 1;
}

int ReadRinexNav::uraindex(double value){
    int i;
    const double ura_eph[]={         /* RAa values (ref [1] 20.3.3.3.1.1) */
            2.4,3.4,4.85,6.85,9.65,13.65,24.0,48.0,96.0,192.0,384.0,768.0,1536.0,
            3072.0,6144.0,0.0
    };
    for (i = 0; i < 15; i++) if (ura_eph[i] >= value) break;
    return i;
}

int ReadRinexNav::sisa_index(double value){
    if (value < 0.0 || value > 6.0) return 255; /* unknown or NAPA */
    else if (value <= 0.5) return (int)(value / 0.01);
    else if (value <= 1.0) return (int)((value - 0.5) / 0.02) + 50;
    else if (value <= 2.0) return (int)((value - 1.0) / 0.04) + 75;
    return ((int)(value - 2.0) / 0.16) + 100;
}


IonTecGrid *ReadIonTecGrid::GetIonTecData(){
    /* read ionex header */
    this->ReadHead();

    /* read ionex body */
    this->Read();

    /* combine tec grid data */
    if (mNt > 0) this->CombineTec();

    /* todo P1-P2 dcb */
    return this->mIonTec;
}

int ReadIonTecGrid::GetIonTecDataNumber(){
    return this->mNt;
}


bool ReadIonTecGrid::ReadHead(){
    while (true)
    {
        getline(mInfile, mBuff);
        if (mBuff.find("END OF HEADER") != string::npos) break;
        if (EndFile()) return false;
        mTempdata =  CString::split(mBuff, " ");

        if      (mBuff.find("IONEX VERSION / TYPE" ) != string::npos)
        {
            mVersion = CString::str2num(mBuff, 0, 8);
        }
        else if (mBuff.find("BASE RADIUS"          ) != string::npos)
        {
            mRb = CString::str2num(mBuff, 0, 8);
        }
        else if (mBuff.find("HGT1 / HGT2 / DHGT"   ) != string::npos)
        {
            mHgts[0] = CString::str2num(mBuff,  2, 6);
            mHgts[1] = CString::str2num(mBuff,  8, 6);
            mHgts[2] = CString::str2num(mBuff, 14, 6);
        }
        else if (mBuff.find("LAT1 / LAT2 / DLAT"   ) != string::npos)
        {
            mLats[0] = CString::str2num(mBuff,  2, 6);
            mLats[1] = CString::str2num(mBuff,  8, 6);
            mLats[2] = CString::str2num(mBuff, 14, 6);
        }
        else if (mBuff.find("LON1 / LON2 / DLON"   ) != string::npos)
        {
            mLons[0] = CString::str2num(mBuff,  2, 6);
            mLons[1] = CString::str2num(mBuff,  8, 6);
            mLons[2] = CString::str2num(mBuff, 14, 6);
        }
        else if (mBuff.find("EXPONENT"             ) != string::npos)
        {
            mNexp = CString::str2num(mBuff, 0, 6);
        }
        else if (mBuff.find("START OF AUX DATA") != string::npos && mBuff.find("DIFFERENTIAL CODE BIASES") != string::npos)
        {
            // todo read dcb
        }

    }
    return true;
}


bool ReadIonTecGrid::Read(){
    int type = 0;
    int i, j, k, m, n, index;
    double lat, lon[3], hgt, x;
    STimes time = {0, 0.0};
    IonTecGrid *Pointer = nullptr;

    while (true)
    {
        getline(mInfile, mBuff);
        if (EndFile()) break;

        if (mBuff.length() < 60) continue;

        if      (mBuff.find("START OF TEC MAP"      ) != string::npos)
        {
            if ((Pointer = this->AddTec())) type = 1;  // create new ion data pointer 
        }
        else if (mBuff.find("END OF TEC MAP"        ) != string::npos)
        {
            type = 0;
            Pointer = nullptr;
        }
        else if (mBuff.find("START OF RMS MAP"      ) != string::npos)
        {
            type = 2;
            Pointer = nullptr;
        }
        else if (mBuff.find("END OF RMS MAP"        ) != string::npos)
        {
            type = 0;
            Pointer = nullptr;
        }
        else if (mBuff.find("EPOCH OF CURRENT MAP"  ) != string::npos)
        {
            time = CTimes::str2time(mBuff);
            if (type == 2)
            {
                for (i = mNt-1; i>=0; i--)
                {
                    if (fabs(CTimes::timediff(time, mIonTec[i].time)) >= 1.0) continue;
                    Pointer = mIonTec + i;
                    break;
                }
            }
            else if (Pointer) Pointer->time = time;
        }
        else if (mBuff.find("LAT/LON1/LON2/DLON/H"  ) != string::npos && Pointer)
        {
            lat    = CString::str2num(mBuff, 2,6);
            lon[0] = CString::str2num(mBuff, 8,6);
            lon[1] = CString::str2num(mBuff,14,6);
            lon[2] = CString::str2num(mBuff,20,6);
            hgt    = CString::str2num(mBuff,26,6);

            i = GetIndex(lat, Pointer->lats);
            k = GetIndex(hgt, Pointer->hgts);
            n = Nitem(lon);

            for (m=0; m<n; m++)
            {
                if (m%16==0 && !getline(mInfile, mBuff)) break;
                j = GetIndex(lon[0] + lon[2] * m, Pointer->lons);
                if ((index = DataIndex(i, j, k, Pointer->ndata)) < 0) continue;
                if (m%16*5+5 > mBuff.length()) continue;
                if ((x = CString::str2num(mBuff, m%16*5, 5)) == 9999.0) continue;

                if (type==1) Pointer->data[index] =         x * pow(10.0, mNexp);
                else         Pointer->rms [index] = (float)(x * pow(10.0, mNexp));
            }
        }

    }
    return true;
}


IonTecGrid *ReadIonTecGrid::AddTec(){
    int n;
    int ndata[3];
    IonTecGrid *nav_tec = nullptr, *p = nullptr;

    ndata[0] = Nitem(mLats);
    ndata[1] = Nitem(mLons);
    ndata[2] = Nitem(mHgts);
    if (ndata[0]<=1 || ndata[1]<=1 || ndata[2]<=0) return nullptr;

    if (mNt >= mNtmax)
    {
        mNtmax += 64;
        if (!(nav_tec = (IonTecGrid *)realloc(mIonTec, sizeof(IonTecGrid)*mNtmax))) 
        {
            Log.Trace(TERROR, "*AddTec malloc error, ntmax=" + to_string(mNtmax));
            mIonTec = nullptr;
            mNt = mNtmax = 0;
            free(mIonTec);
            return nullptr;
        }
        mIonTec = nav_tec;
    }

    p = mIonTec + mNt;
    p->time = {0,0};
    p->rb = mRb;
    for (int i = 0; i < 3; i++)
    {
        p->ndata[i] = ndata[i];
        p->lats[i]  = mLats[i];
        p->lons[i]  = mLons[i];
        p->hgts[i]  = mHgts[i];
    }
    n = ndata[0]*ndata[1]*ndata[2];

    if (!(p->data = (double *)malloc(sizeof(double)*n))||
        !(p->rms  = (float  *)malloc(sizeof(float )*n)))
    {
        return nullptr;
    }

    for (int i = 0; i < n; i++)
    {
        p->data[i] = 0.0;
        p->rms [i] = 0.0f;
    }

    mNt++;
    return p;
}


void ReadIonTecGrid::CombineTec(){
    int i, j, n = 0;
    IonTecGrid tmp;

    for (i = 0; i < mNt-1; i++) {
        for (j = i+1; j < mNt; j++)
        {
            if (CTimes::timediff(mIonTec[j].time, mIonTec[i].time) < 0.0)
            {
                tmp        = mIonTec[i];
                mIonTec[i] = mIonTec[j];
                mIonTec[j] = tmp;
            }
        }
    }

    for (i = 0; i < mNt; i++)
    {
        if (i>0 && CTimes::timediff(mIonTec[i].time,mIonTec[n-1].time)==0.0)
        {
            free(mIonTec[n-1].data);
            free(mIonTec[n-1].rms );
            mIonTec[n-1] = mIonTec[i];
            continue;
        }
        mIonTec[n++] = mIonTec[i];
    }
    mNt = n;
}


/* get number of items -------------------------------------------------------*/
int ReadIonTecGrid::Nitem(const double *range){
    return GetIndex(range[1], range) + 1;
}


int ReadIonTecGrid::GetIndex(double value, const double *range){
    if (range[2] == 0.0) return 0;
    if (range[1] > 0.0 && (value<range[0] || range[1]<value)) return -1;
    if (range[1] < 0.0 && (value<range[1] || range[0]<value)) return -1;
    return (int)floor((value - range[0]) / range[2] + 0.5);
}


/* data index (i:lat,j:lon,k:hgt) --------------------------------------------*/
int ReadIonTecGrid::DataIndex(int i, int j, int k, const int *ndata){
    if (i<0 || ndata[0]<=i || j<0 || ndata[1]<=j || k<0 || ndata[2]<=k) return -1;
    return i + ndata[0] * (j + ndata[1] * k);
}
