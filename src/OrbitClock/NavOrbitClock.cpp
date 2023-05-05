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
 *   [1]. BeiDou navigation satellite system signal in space interface control
 *            document open service signal B1I (version 3.0), China Satellite
 *            Navigation office, February, 2019
 *   [2]. Global Navigation Satellite System GLONASS, Interface Control Document
 *            Navigational radiosignal In bands L1, L2, (Version 5.1), 2008
 *   [3]. European GNSS (Galileo) Open Service Signal In Space Interface Control
 *            Document, Issue 1.3, December, 2016
 *   [4]. IS-GPS-200K, Navstar GPS Space Segment/Navigation User Interfaces,
 *            May 6, 2019
 * Created by lizhen on 2021/7/22.
 *-------------------------------------------------------------------------------*/

#include "NavOrbitClock.h"

NavOrbClk::NavOrbClk(GnssNavData& nav_t){
    this->mNavs = nav_t;
}


double NavOrbClk::CalSatClock(STimes tTime, int sys, int id){
    mSystem = sys; mSatId = id;
    if (mSystem == SYS_GPS || mSystem == SYS_BDS || mSystem == SYS_GAL || mSystem == SYS_QZS)
    {
        if (!this->SelectEph(tTime)) return 1e9;
        return this->eph2clk(tTime);
    }
    else if (mSystem == SYS_GLO)
    {
        if (!this->SelectGEph(tTime)) return 1e9;
        return this->geph2clk(tTime);
    }
    else return 1e9;
}


bool NavOrbClk::CalSatPosVel(STimes tTime, SatInfo &sat){
    double tt = 1E-3;
    STimes tCorrTime;
    SatInfo sat2;
    mSystem = CString::satsys(sat.prn);
    mSatId  = CString::satprn2id(sat.prn);

    sat.svh = -1;     // sat health flag (-1:correction not available)
    if (mSystem == SYS_GPS || mSystem == SYS_BDS || mSystem == SYS_GAL || mSystem == SYS_QZS)
    {
        if (!this->SelectEph(tTime)) return false;
        this->eph2pos(tTime, sat);
        tCorrTime = CTimes::timeadd(tTime,tt);
        this->eph2pos(tCorrTime, sat2);
        sat.svh = mEph_t.svh;
        sat.tgd[0] = mEph_t.tgd[0];
        sat.tgd[1] = mEph_t.tgd[1];
    }
    else if (mSystem == SYS_GLO)
    {
        if (!this->SelectGEph(tTime)) return false;
        this->geph2pos(tTime, sat);
        tCorrTime = CTimes::timeadd(tTime,tt);
        this->geph2pos(tCorrTime, sat2);
        sat.svh = mGeph_t.svh;
        sat.glofcn = mGeph_t.frq;
        sat.tgd[0] = mGeph_t.dtaun;
    }
    else return false;

    /* satellite velocity and clock drift by differential approx */
    sat.vel = (sat2.pos - sat.pos) / tt;
    sat.dts[1] = (sat2.dts[0] - sat.dts[0]) / tt;

    return true;
}


bool NavOrbClk::SelectEph(STimes tTime){
    int sel, iode = -1;
    double t, tmax, tmin;
    mEph_t = {};
    switch (mSystem)
    {
        case SYS_GPS: tmax = MAXDTOE     + 1.0; sel = config.eph_sel[SYS_GPS]; break;
        case SYS_BDS: tmax = MAXDTOE_BDS + 1.0; sel = config.eph_sel[SYS_BDS]; break;
        case SYS_GAL: tmax = MAXDTOE_GAL      ; sel = config.eph_sel[SYS_GAL]; break;
        default:      tmax = MAXDTOE     + 1.0; break;
    }

    tmin = tmax + 1.0;
    if (mNavs.ephs.find(mSatId) != mNavs.ephs.end())
    {
        auto ephss = mNavs.ephs[mSatId];
        for (auto iter = ephss.begin(); iter != ephss.end() ; iter++)
        {
            if (iode >= 0 && iter->second.iode != iode) continue;
            if (mSystem == SYS_GAL)
            {
                if (sel == 0 &&! (iter->second.code & (1<<9))) continue;       /* I/NAV  */
                if (sel == 1 &&! (iter->second.code & (1<<8))) continue;       /* F/NAV  */
                if (CTimes::timediff(iter->second.toe,tTime) >= 0.0) continue; /* AOD<=0 */
            }
            if ((t = fabs(CTimes::timediff(iter->second.toe, tTime))) > tmax) continue;
            if (iode >= 0) { mEph_t = iter->second; break;   }
            if (t <= tmin) { mEph_t = iter->second; tmin = t;} /* toe closest to time */
        }
    }
    else
    { // no broadcast ephemeris
        return false;
    }

    if (!mEph_t.sat) return false;

    return true;
}


/* broadcast ephemeris to satellite clock bias ---------------------------------
* compute satellite clock bias with broadcast ephemeris (gps, galileo, qzss)
* args   : STimes    time     I   time by satellite clock (gpst)
* return : satellite clock bias
* notes  : satellite clock does not include relativity correction and tgd
*-----------------------------------------------------------------------------*/
double NavOrbClk::eph2clk(STimes tTime){
    double t,ts;

    t = ts = CTimes::timediff(tTime, mEph_t.toc);
    for (int i = 0; i < 2; i++)
    {
        t = ts - (mEph_t.f0 + mEph_t.f1 * t + mEph_t.f2 * t * t);
    }

    return mEph_t.f0 + mEph_t.f1 * t + mEph_t.f2 * t * t;
}


bool NavOrbClk::SelectGEph(STimes tTime){
    int iode = -1;
    double t,tmax = MAXDTOE_GLO, tmin = tmax + 1.0;
    mGeph_t = {};
    if (mNavs.gloephs.find(mSatId) != mNavs.gloephs.end())
    {
        auto ephss = mNavs.gloephs[mSatId];
        for (auto iter = ephss.begin(); iter != ephss.end() ; iter++)
        {
            if (iode >= 0 && iter->second.iode != iode) continue;
            if ((t = fabs(CTimes::timediff(iter->second.toe,tTime))) > tmax) continue;
            if (iode >= 0) { mGeph_t = iter->second; break;   }
            if (t <= tmin) { mGeph_t = iter->second; tmin = t;} /* toe closest to time */
        }
    }
    else
    {
        Log.Trace(TWARNING, "no glonass ephemeris, sat= " + to_string(mSatId));
        return false;
    }

    if (!mGeph_t.sat)
    {
        Log.Trace(TWARNING, "no glonass ephemeris, sat= " + to_string(mSatId));
        return false;
    }
    return true;
}


/* glonass ephemeris to satellite clock bias -----------------------------------
* compute satellite clock bias with glonass ephemeris
* args   : STimes    time     I   time by satellite clock (gpst)
* return : satellite clock bias (s)
*-----------------------------------------------------------------------------*/
double NavOrbClk::geph2clk(STimes tTime){
    double t,ts;

    t = ts = CTimes::timediff(tTime, mGeph_t.toe);
    for (int i = 0; i < 2; i++)
    {
        t = ts - (-mGeph_t.taun + mGeph_t.gamn * t);
    }
    return -mGeph_t.taun + mGeph_t.gamn * t;
}


/* broadcast ephemeris to satellite position and clock bias --------------------
* compute satellite position and clock bias with broadcast ephemeris (gps,
* galileo, qzss)
* args   : STimes   time     I   time (gpst)
*          Vector3d sat.pos  O   satellite position (ecef) {x,y,z} (m)
*          double   sat.dts  O   satellite clock bias (s)
*          double   sat.var  O   satellite position and clock variance (m^2)
* return : none
* notes  : satellite clock includes relativity correction without code bias
*          (tgd or bgd)
*-----------------------------------------------------------------------------*/
void NavOrbClk::eph2pos(STimes tTime, SatInfo& sat){
    int n;
    double tk, M, E, Ek, sinE, cosE, u, r, i, O, sin2u, cos2u, x, y, sinO, cosO, cosi, mu, omge;
    double xg,yg,zg,sino,coso;

    if (mEph_t.A <= 0.0)
    {
        sat.pos[0] = sat.pos[1] = sat.pos[2] = sat.dts[0] = sat.var = 0.0;
        return;
    }

    tk = CTimes::timediff(tTime, mEph_t.toe);

    switch (mSystem)
    {
        case SYS_BDS: mu = MU_CMP; omge = OMGE_CMP;  break;
        case SYS_GAL: mu = MU_GAL; omge = OMGE_GAL;  break;
        default:      mu = MU_GPS; omge = WGS84_WIE; break;
    }

    M = mEph_t.M0 + (sqrt(mu / (mEph_t.A * mEph_t.A * mEph_t.A)) + mEph_t.deln) * tk;
    for (n = 0, E = M, Ek = 0.0; fabs(E - Ek) > RTOL_KEPLER && n < MAX_ITER_KEPLER; n++) 
    {
        Ek = E; E -= (E - mEph_t.e * sin(E) - M) / (1.0 - mEph_t.e * cos(E));
    }

    if (n >= MAX_ITER_KEPLER)
    {
        Log.Trace(TINFO, "eph2pos: kepler iteration overflow sat= " + sat.prn);
        return;
    }

    sinE = sin(E);
    cosE = cos(E);
    u = atan2(sqrt(1.0 - mEph_t.e * mEph_t.e) * sinE, cosE - mEph_t.e) + mEph_t.omg;
    r = mEph_t.A * (1.0 - mEph_t.e * cosE);
    i = mEph_t.i0 + mEph_t.idot * tk;

    sin2u = sin(2.0 * u);
    cos2u = cos(2.0 * u);
    u += mEph_t.cus * sin2u + mEph_t.cuc * cos2u;
    r += mEph_t.crs * sin2u + mEph_t.crc * cos2u;
    i += mEph_t.cis * sin2u + mEph_t.cic * cos2u;
    x  = r * cos(u);
    y  = r * sin(u);
    cosi = cos(i);

    /* beidou geo satellite */
    int bdprn = mSatId - NSATGPS;
    if (mSystem == SYS_BDS && (bdprn <= 5 || bdprn >= 59))
    {   /* ref [1] table 4-1 */
        O = mEph_t.OMG0 + mEph_t.OMGd * tk - omge * mEph_t.toes;
        sinO = sin(O); cosO = cos(O);
        xg = x * cosO - y * cosi * sinO;
        yg = x * sinO + y * cosi * cosO;
        zg = y * sin(i);
        sino = sin(omge * tk); coso = cos(omge * tk);
        sat.pos[0] =  xg * coso + yg * sino * COS_5 + zg * sino * SIN_5;
        sat.pos[1] = -xg * sino + yg * coso * COS_5 + zg * coso * SIN_5;
        sat.pos[2] = -yg * SIN_5 + zg * COS_5;
    }
    else
    {
        O = mEph_t.OMG0 + (mEph_t.OMGd - omge) * tk - omge * mEph_t.toes;
        sinO = sin(O); cosO = cos(O);
        sat.pos[0] = x * cosO - y * cosi * sinO;
        sat.pos[1] = x * sinO + y * cosi * cosO;
        sat.pos[2] = y * sin(i);
    }

    tk = CTimes::timediff(tTime, mEph_t.toc);
    sat.dts[0] = mEph_t.f0 + mEph_t.f1 * tk + mEph_t.f2 * tk * tk;

    /* relativity correction */
    sat.dts[0] -= 2.0 * sqrt(mu * mEph_t.A) * mEph_t.e * sinE / SQR(CLIGHT);

    /* position and clock error variance */
    sat.var = this->var_uraeph(mSystem);
}


/* glonass ephemeris to satellite position and clock bias ----------------------
* compute satellite position and clock bias with glonass ephemeris
* args   : STimes   time     I   time (gpst)
*          Vector3d sat.pos  O   satellite position (ecef) {x,y,z} (m)
*          double   sat.dts  O   satellite clock bias (s)
*          double   sat.var  O   satellite position and clock variance (m^2)
* return : none
* notes  : see ref [2]
*-----------------------------------------------------------------------------*/
void NavOrbClk::geph2pos(STimes tTime, SatInfo& sat){
    double t, tt, x[6];

    t = CTimes::timediff(tTime, mGeph_t.toe);
    sat.dts[0] = -mGeph_t.taun + mGeph_t.gamn * t;

    for (int i = 0; i < 3; i++)
     {
        x[i    ] = mGeph_t.pos[i];
        x[i + 3] = mGeph_t.vel[i];
    }

    for (tt = t < 0.0? -TSTEP: TSTEP; fabs(t) > 1E-9; t -= tt)
    {
        if (fabs(t) < TSTEP) tt = t;
        glorbit(tt,x);
    }

    for (int i = 0; i < 3; i++) sat.pos[i] = x[i];

    sat.var = SQR(ERREPH_GLO);
}


/* glonass orbit differential equations --------------------------------------*/
void NavOrbClk::deq(const double *x, double *xdot, const double *acc){
    double a, b, c, omg2 = SQR(OMGE_GLO);
    double r2 = x[0] * x[0] + x[1] * x[1] + x[2] * x[2];
    double r3 = r2 * sqrt(r2);

    if (r2 <= 0.0)
    {
        xdot[0] = xdot[1] = xdot[2] = xdot[3] = xdot[4] = xdot[5] = 0.0;
        return;
    }

    /* ref [2] A.3.1.2 with bug fix for xdot[4],xdot[5] */
    a = 1.5 * WGS84_J2 * MU_GLO * SQR(RE_GLO) / r2 / r3; /* 3/2*J2*mu*Ae^2/r^5 */
    b = 5.0 * x[2] * x[2] / r2;                          /* 5*z^2/r^2          */
    c = -MU_GLO / r3 - a * (1.0 - b);                    /* -mu/r^3-a(1-b)     */
    xdot[0] = x[3]; xdot[1] = x[4]; xdot[2] = x[5];
    xdot[3] = (c + omg2) * x[0] + 2.0 * OMGE_GLO * x[4] + acc[0];
    xdot[4] = (c + omg2) * x[1] - 2.0 * OMGE_GLO * x[3] + acc[1];
    xdot[5] = (c - 2.0 * a) * x[2] + acc[2];
}


void NavOrbClk::glorbit(double t, double *x){
    double k1[6], k2[6], k3[6], k4[6], w[6];

    deq(x, k1, mGeph_t.acc); for (int i = 0; i < 6; i++) w[i] = x[i] + k1[i] * t / 2.0;
    deq(w, k2, mGeph_t.acc); for (int i = 0; i < 6; i++) w[i] = x[i] + k2[i] * t / 2.0;
    deq(w, k3, mGeph_t.acc); for (int i = 0; i < 6; i++) w[i] = x[i] + k3[i] * t;
    deq(w, k4, mGeph_t.acc);
    for (int i = 0; i < 6; i++) x[i] += (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]) * t / 6.0;
}


double NavOrbClk::var_uraeph(int system){
    int ura = mEph_t.sva;
    double var;
    const double ura_value[]={
            2.4,3.4,4.85,6.85,9.65,13.65,24.0,48.0,96.0,192.0,384.0,768.0,1536.0,
            3072.0,6144.0
    };

    if (system == SYS_GAL)
    {   /* galileo sisa (ref [3] 5.1.11) */
        if (ura<= 49) return SQR(ura*0.01);
        if (ura<= 74) return SQR(0.5+(ura- 50)*0.02);
        if (ura<= 99) return SQR(1.0+(ura- 75)*0.04);
        if (ura<=125) return SQR(2.0+(ura-100)*0.16);
        var = SQR(STD_GAL_NAPA);
    }
    else
    {   /* gps ura (ref [4] 20.3.3.3.1.1) */
        var = ura < 0 || 14 < ura? SQR(6144.0): SQR(ura_value[ura]);
    }

    return var;
}
