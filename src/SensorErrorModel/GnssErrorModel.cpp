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
 *   [1]. RTCA/DO-229C, Minimum operational performance standards for global
 *            positioning system/wide area augmentation system airborne equipment,
 *            November 28, 2001
 *   [2]. A.E.Niell, Global mapping functions for the atmosphere delay at radio
 *            wavelengths, Journal of geophysical research, 1996
 *   [3] J.Boehm, A.Niell, P.Tregoning and H.Shuh, Global Mapping Function
 *            (GMF): A new empirical mapping function base on numerical weather
 *            model data, Geophysical Research Letters, 33, L07304, 2006
 *
 * Created by lizhen on 2021/7/22.
 *-------------------------------------------------------------------------------*/

#include "GnssErrorModel.h"


/* set selected satellite ephemeris --------------------------------------------
* Set selected satellite ephemeris for multiple ones like LNAV - CNAV, I/NAV -
* F/NAV. Call it before calling satpos to use unselected one.
* args   : int    sys       I   satellite system (SYS_???)
*          int    sel       I   selection of ephemeris
*                                 GPS,QZS : 0:LNAV ,1:CNAV  (default: LNAV)
*                                 GAL     : 0:I/NAV,1:F/NAV (default: I/NAV)
* notes  : default ephemeris selection for galileo is any.
*-----------------------------------------------------------------------------*/
void GnssErrorModel::SetEphemerisSel(){
    if (config.ProcessMode == PMODE_SINGLE)
    {
        if (config.GAL_FreqID.size() > 1 & config.GAL_FreqID[1] == 2)
        {
            config.eph_sel[SYS_GAL] = 1; //E5a
        }
    }

}


bool GnssErrorModel::CodeBiasCorrect(GnssObsData& obs, const SatInfo& sat){
    if      (config.CodeBias == CODEBIAS_OFF) return true;
    else if (config.CodeBias == CODEBIAS_TGD) return CodeBiasTGD(obs,sat);
    else if (config.CodeBias == CODEBIAS_DCB)
    {
        return false;
        // todo dcb
    }
    else
    {
        Log.Trace(TERROR, "CodeBias correct mode set error, please check!");
        return false;
    }

}


bool GnssErrorModel::CheckSnrMask(double el, double snr){
    int i;
    double minsnr,a;

    a = (el * R2D + 5.0) / 10.0;
    i = (int)floor(a);
    a -= i;
    if      (i < 1) minsnr = config.SnrMask[0];
    else if (i > 8) minsnr = config.SnrMask[8];
    else            minsnr = (1.0 - a) * config.SnrMask[i-1] + a * config.SnrMask[i];
    return snr < minsnr;
}


/* ionosphere model ------------------------------------------------------------
* compute ionospheric delay by broadcast ionosphere model (klobuchar model)
* args   :
* return : ionospheric delay (L1) (m)
*-----------------------------------------------------------------------------*/
double GnssErrorModel::klobuchar(){
    int week;
    const double *ion = mNavHead.ion_gps;
    double tt, f, psi, phi, lam, amp, per, x;
    double nion = sqrt(SQR(ion[4]) + SQR(ion[5]) + SQR(ion[6]) + SQR(ion[7]));  // judge according 4 at end

    if (mBLH[2] < -1E3 || mElevation <= 0) return 0.0;
    if (nion <= 1e-2) ion = ion_default;

    /* earth centered angle (semi-circle) */
    psi = 0.0137 / (mElevation / PI + 0.11) - 0.022;

    /* subionospheric latitude/longitude (semi-circle) */
    phi = mBLH[0] / PI + psi * cos(mAzimuth);
    if      (phi >  0.416) phi = 0.416;
    else if (phi < -0.416) phi = -0.416;
    lam = mBLH[1] / PI + psi * sin(mAzimuth) / cos(phi * PI);

    /* geomagnetic latitude (semi-circle) */
    phi += 0.064 * cos((lam - 1.617) * PI);

    /* local time (s) */
    tt  = 43200.0 * lam + CTimes::time2gpst(mTime, &week);
    tt -= floor(tt / 86400.0) * 86400.0; /* 0<=tt<86400 */

    /* slant factor */
    f = 1.0 + 16.0 * pow(0.53 - mElevation / PI,3.0);

    /* ionospheric delay */
    amp = ion[0] + phi * (ion[1] + phi * (ion[2] + phi * ion[3]));
    per = ion[4] + phi * (ion[5] + phi * (ion[6] + phi * ion[7]));
    amp = amp <     0.0?     0.0: amp;
    per = per < 72000.0? 72000.0: per;
    x   = 2.0 * PI * (tt - 50400.0) / per;

    return CLIGHT * f * (fabs(x) < 1.57? 5E-9 + amp * (1.0 + x * x * (-0.5 + x * x /24.0)): 5E-9);
}


double GnssErrorModel::klobuchar_BDS(){
    const double *ion = mNavHead.ion_bds;
    double re = 6378.0, hion = 375.0;
    double f, x, sow, dt, phi, amp, per;
    Vector3d blhp = Vector3d::Zero(3);

    double nion = sqrt(SQR(ion[4]) + SQR(ion[5]) + SQR(ion[6]) + SQR(ion[7]));  // judge according 4 at end

    if (mBLH[2] < -1E3 || mElevation <= 0) return 0.0;
    if (nion <= 1e-3) return klobuchar();

    f = ionppp(re, hion, blhp);

    sow = CTimes::time2gpst(mTime, nullptr);
    dt  = 43200.0 * blhp[1] / PI + sow;
    dt -= floor(dt / 86400.0) * 86400.0;

    phi = blhp[0] / PI;
    amp = ion[0] + phi * (ion[1] + phi * (ion[2] + phi * ion[3]));
    per = ion[4] + phi * (ion[5] + phi * (ion[6] + phi * ion[7]));
    amp = amp < 0.0? 0.0: amp;
    per = per < 72000.0? 72000.0: per;
    per = per >= 172800.0? 172800.0: per;
    x = dt - 50400.0;
    return CLIGHT * f * (fabs(x) < per / 4.0? 5E-9 + amp * cos(2.0 * PI * x / per): 5E-9);
}


/* ionosphere model by tec grid data -------------------------------------------
* compute ionospheric delay by tec grid data
* args   : double *delay    O   ionospheric delay (L1) (m)
*          double *var      O   ionospheric dealy (L1) variance (m^2)
* return : status (1:ok,0:error)
* notes  : before calling the function, read tec grid data by calling readtec()
*          return ok with delay=0 and var=VAR_NOTEC if el<MIN_EL or h<MIN_HGT
*-----------------------------------------------------------------------------*/
bool GnssErrorModel::IonTecGridDelay(double& delay, double& var){
    int i, stat[2];
    double a, tt;
    double dels[2], vars[2];

    if ( mElevation < MIN_EL || mBLH[2] < MIN_HGT)
    {
        delay = 0.0;
        var   = VAR_NOTEC;
        return true;
    }

    for (i = 0; i < mIonTecNum; i++)
    {
        if (CTimes::timediff(mIonTec[i].time, mTime)>0.0) break;
    }

    if (i==0 || i >= mIonTecNum)
    {
        Log.Trace(TINFO, CTimes::time2str(mTime) + ":tec grid out of period");
        return false;
    }

    if ((tt = CTimes::timediff(mIonTec[i].time, mIonTec[i-1].time)) == 0.0)
    {
        Log.Trace(TINFO, "tec grid time interval error");
        return false;
    }

    /* ionospheric delay by tec grid data */
    stat[0] = iondelay(mIonTec+i-1, dels[0], vars[0]);
    stat[1] = iondelay(mIonTec+i  , dels[1], vars[1]);

    if (!stat[0] && !stat[1])
    {
        Log.Trace(TINFO, CTimes::time2str(mTime) + ": tec grid out of area pos=" +
                         to_string(mBLH[0]*R2D)  + " " + to_string(mBLH[1]*R2D) + " azel=" +
                         to_string(mAzimuth*R2D) + " " + to_string(mElevation*R2D));
        return false;
    }

    if (stat[0] && stat[1]) /* linear interpolation by time */
    {
        a     = CTimes::timediff(mTime, mIonTec[i-1].time) / tt;
        delay = dels[0] * (1.0 - a) + dels[1] * a;
        var   = vars[0] * (1.0 - a) + vars[1] * a;
    }
    else if (stat[0])
    {   /* nearest-neighbour extrapolation by time */
        delay = dels[0];
        var   = vars[0];
    }
    else
    {
        delay = dels[1];
        var   = vars[1];
    }
    return true;
}


/* ionospheric pierce point position -------------------------------------------
* compute ionospheric pierce point (ipp) position and slant factor
* args   : double   re        I   earth radius (km)
*          double   hion      I   altitude of ionosphere (km)
*          Vector3d posp      O   pierce point position {lat,lon,h} (rad,m)
* return : slant factor
* notes  : see ref [1], only valid on the earth surface
*          fixing bug on ref [1] A.4.4.10.1 A-22,23
*-----------------------------------------------------------------------------*/
double GnssErrorModel::ionppp(double re, double hion, Vector3d& posp){
    double cosaz, rp, ap, sinap, tanap;

    rp = re / (re + hion) * cos(mElevation);
    ap = PI / 2.0 - mElevation - asin(rp);
    sinap = sin(ap);
    tanap = tan(ap);
    cosaz = cos(mAzimuth);
    posp[0] = asin(sin(mBLH[0]) * cos(ap) + cos(mBLH[0]) * sinap * cosaz);

    if ((mBLH[0] > 70.0 * D2R && tanap * cosaz > tan(PI / 2.0 - mBLH[0])) ||
        (mBLH[0] < -70.0 * D2R && -tanap * cosaz > tan(PI / 2.0 + mBLH[0]))) 
    {
        posp[1] = mBLH[1] + PI - asin(sinap * sin(mAzimuth) / cos(posp[0]));
    }
    else
    {
        posp[1] = mBLH[1] + asin(sinap * sin(mAzimuth) / cos(posp[0]));
    }
    return 1.0 / sqrt(1.0 - rp * rp);
}


/* troposphere model -----------------------------------------------------------
* compute tropospheric delay by standard atmosphere and saastamoinen model
* args   : double humi      I   relative humidity
* return : tropospheric delay (m)
*-----------------------------------------------------------------------------*/
double GnssErrorModel::saastamoinen(double humi){
    const double temp0 = 15.0; /* temparature at sea level */
    double hgt, pres, temp, e, z, trph, trpw;

    if (mBLH[2] < -100.0 || 1E4 < mBLH[2] || mElevation <= 0) return 0.0;

    /* standard atmosphere */
    hgt = mBLH[2] < 0.0 ? 0.0 : mBLH[2];

    pres = 1013.25 * pow(1.0 - 2.2557E-5 * hgt, 5.2568);
    temp = temp0 - 6.5E-3 * hgt + 273.16;
    e = 6.108 * humi * exp((17.15 * temp - 4684.0) / (temp - 38.45));

    /* saastamoninen model */
    z = PI / 2.0 - mElevation;
    trph = 0.0022768 * pres / (1.0 - 0.00266 * cos(2.0 * mBLH[0]) - 0.00028 * hgt / 1E3) / cos(z);
    trpw = 0.002277 * (1255.0 / temp + 0.05) * e / cos(z);
    return trph + trpw;
}


/* troposphere mapping function ------------------------------------------------
* compute tropospheric mapping function by NMF, GMF and VMF1
* args   : double *mapfw    IO  wet mapping function (NULL: not output)
* return : dry mapping function
* note   : see ref [2] (NMF) and [3] (GMF)
*          original JGR paper of [2] has bugs in eq.(4) and (5). the corrected
*          paper is obtained from:
*          ftp://web.haystack.edu/pub/aen/nmf/NMF_JGR.pdf
*-----------------------------------------------------------------------------*/
double GnssErrorModel::tropmapf(double& mapfw){
    Log.Trace(TEXPORT, "tropmapf: pos= " + to_string(mBLH[0] * R2D) + " " + to_string(mBLH[1] * R2D) + " "
                       + to_string(mBLH[2]) +
                       "  azel= " + to_string(mAzimuth*R2D) + " " + to_string(mElevation * R2D));

    if (mBLH[2] < -1000.0 || mBLH[2] > 20000.0)
    {
        if (mapfw > 0.0) mapfw=0.0;
        return 0.0;
    }

    if      (config.TropMapFunction == TROPMAP_NMF) return this->trpmap_nmf(mapfw);
    else if (config.TropMapFunction == TROPMAP_GMF) return this->tropmap_gmf(mapfw);
    else if (config.TropMapFunction == TROPMAP_VMF) return this->tropmap_vmf1(0.0, 0.0, mapfw); // todo vmf ad and aw
    else    return 0.0;
}


double GnssErrorModel::gettgd(const SatInfo& sat, int type){
    int system = CString::satsys(sat.prn);

    if (system == SYS_GLO)
    {
        return -sat.tgd[0] * CLIGHT;  /* dtaun, delay between L1 and L2 (s) */
    }
    else
    {
        return sat.tgd[type] * CLIGHT;
    }
}


bool GnssErrorModel::CodeBiasTGD(GnssObsData& obs, const SatInfo& sat){
    int system = CString::satsys(obs.prn);
    int bdprn, galbrd;
    double alpha, beta;

    switch (system)
    {
        case SYS_GPS:  // L1 L2
            alpha =  SQR(FREQ1) / SQR(FREQ2);
            if (obs.P[0] > 1e-3) obs.P[0] -= gettgd(sat, 0);         // P1
            if (obs.P[1] > 1e-3) obs.P[1] -= gettgd(sat, 0) * alpha; // P2
            return true;
        case SYS_BDS:  // B2:B1I, B2I, B3:B1I, B2b todo isc
            bdprn = int(CString::str2num(sat.prn, 1, 2));
            if(bdprn <= 18)
            {
                if (obs.P[0] > 1e-3) obs.P[0] -= gettgd(sat, 0); // B1I
                if (obs.P[1] > 1e-3) obs.P[1] -= gettgd(sat, 1); // B2I
            }
            else
            {
                if (obs.P[0] > 1e-3) obs.P[0] -= gettgd(sat, 0); // B1I
            //    if (obs.P[1] > 1e-3) obs.P[1] -= gettgd(sat, 1); // B2b
            }
            return true;
        case SYS_GLO:  // G1 G2
            alpha = SQR(FREQ2_GLO) / (SQR(FREQ1_GLO) - SQR(FREQ2_GLO));
            beta  = SQR(FREQ1_GLO) / (SQR(FREQ1_GLO) - SQR(FREQ2_GLO));
            if (obs.P[0] > 1e-3) obs.P[0] -= gettgd(sat, 0) * alpha;
            if (obs.P[1] > 1e-3) obs.P[1] -= gettgd(sat, 0) * beta;
            return true;
        case SYS_GAL:
            galbrd = config.eph_sel[SYS_GAL];
            if(galbrd == 0)
            {   /*BGD_E1E5b*/  // note different brdc
                alpha =  SQR(FREQ1) / SQR(FREQ7);
                if (obs.P[0] > 1e-3) obs.P[0] -= gettgd(sat, 0);
                if (obs.P[1] > 1e-3) obs.P[1] -= gettgd(sat, 0) * alpha;
            }
            else if(galbrd == 1)
            {   /*BGD_E1E5a*/
                alpha =  SQR(FREQ1) / SQR(FREQ5);
                if (obs.P[0] > 1e-3) obs.P[0] -= gettgd(sat, 0);
                if (obs.P[2] > 1e-3) obs.P[2] -= gettgd(sat, 1) * alpha;
            }
        default: return false;
    }
}


double GnssErrorModel::trpmap_nmf(double& mapw){
    double y, cosy, ah[3], aw[3], dm;
    double el = mElevation, lat = mBLH[0] * R2D, hgt = mBLH[2];

    if (el <= 0.0)
    {
        mapw = 0.0;
        return 0.0;
    }

    /* year from doy 28, added half a year for southern latitudes */
    y = (CTimes::time2doy(mTime) - 28.0) / 365.25 + (lat < 0.0? 0.5: 0.0);

    cosy = cos(2.0 * PI * y);
    lat = fabs(lat);

    for (int i  =0; i < 3; i++)
    {
        ah[i] = interpc(sNmfCoef[i    ], lat) - interpc(sNmfCoef[i + 3], lat) * cosy;
        aw[i] = interpc(sNmfCoef[i + 6], lat);
    }

    /* ellipsoidal height is used instead of height above sea level */
    dm = (1.0 / sin(el) - mapf(el, sAht[0], sAht[1], sAht[2])) * hgt / 1E3;

    mapw = mapf(el, aw[0], aw[1], aw[2]);

    return mapf(el, ah[0], ah[1], ah[2]) + dm;
}


double GnssErrorModel::tropmap_gmf(double& mapw){
    int i, j, k = 0, nmax = 9;
    double x, y, z, doy, mjd;
    double ah, sine, beta, gamma, topcon, ht_corr_coef, ht_corr, ch, aw;
    double maph, phh, c11h, c10h;
    double ahm = 0.0, aha = 0.0, bh = 0.0029, c0h = 0.062;
    double a_ht = 2.53e-5, b_ht = 5.49e-3, c_ht = 1.14e-3, hs_km = mBLH[2] / 1000.0;
    double bw = 0.00146, cw = 0.04391, awm = 0.0, awa = 0.0;
    double v[20][20] = {{0.0}};
    double w[20][20] = {{0.0}};
    double zenith = PI / 2.0 - mElevation;

    mjd = CTimes::time2mjday(mTime);
    doy = mjd - 44239.0 + 1.0 - 28;

    x = cos(mBLH[0]) * cos(mBLH[1]);
    y = cos(mBLH[0]) * sin(mBLH[1]);
    z = sin(mBLH[0]);

    v[0][0] = 1.0;
    w[0][0] = 0.0;
    v[1][0] = z*v[0][0];
    w[1][0] = 0.0;

    for (i = 2; i <= nmax; ++i)
    {
        v[i][0] = ((2 * i - 1) * z * v[i - 1][0] - (i - 1) * v[i - 2][0]) / i;
        w[i][0] = 0.0;
    }

    for (j = 1; j <= nmax; ++j)
    {
        v[j][j] = (2 * j - 1) * (x * v[j - 1][j - 1] - y * w[j - 1][j - 1]);
        w[j][j] = (2 * j - 1) * (x * w[j - 1][j - 1] + y * v[j - 1][j - 1]);
        if (j < nmax)
        {
            v[j + 1][j] = (2 * j + 1) * z * v[j][j];
            w[j + 1][j] = (2 * j + 1) * z * w[j][j];
        }

        for (i = j + 2; i <= nmax; ++i)
        {
            v[i][j] = ((2 * i - 1) * z * v[i - 1][j] - (i + j - 1) * v[i - 2][j]) / (i - j);
            w[i][j] = ((2 * i - 1) * z * w[i - 1][j] - (i + j - 1) * w[i - 2][j]) / (i - j);
        }
    }

    if (mBLH[0] < 0)
    {
        phh  = PI;
        c11h = 0.007;
        c10h = 0.002;
    }
    else
    {
        phh  = 0;
        c11h = 0.005;
        c10h = 0.001;
    }

    ch = c0h + ((cos(doy / 365.250 * 2 * PI + phh) + 1) * c11h / 2 + c10h) * (1 - cos(mBLH[0]));

    for (i = 0; i <= nmax; ++i)
    {
        for (j = 0; j <= i; ++j)
        {
            k = k + 1;
            ahm = ahm + (sAhMean[k - 1] * v[i][j] + sBhMean[k - 1] * w[i][j]);
            aha = aha + (sAhAmp[k - 1] * v[i][j] + sBhAmp[k - 1] * w[i][j]);
        }
    }

    ah = (ahm + aha * cos(doy / 365.25 * 2.0 * PI)) * 1e-5;
    sine = sin(PI / 2 - zenith);
    beta = bh / (sine + ch);
    gamma = ah / (sine + beta);
    topcon = (1.0 + ah / (1.0 + bh / (1.0 + ch)));
    maph = topcon / (sine + gamma);

    //height correction for hydrostatic mapping function from Niell (1996)
    beta = b_ht / (sine + c_ht);
    gamma = a_ht / (sine + beta);
    topcon = (1.0 + a_ht / (1.0 + b_ht / (1.0 + c_ht)));
    ht_corr_coef = 1 / sine - topcon / (sine + gamma);
    ht_corr = ht_corr_coef*hs_km;

    maph += ht_corr;

    k = 0;
    for (i = 0; i <= nmax; ++i)
    {
        for (j = 0; j <= i; ++j)
        {
            k = k + 1;
            awm = awm + (sAwMean[k - 1] * v[i][j] + sBwMean[k - 1] * w[i][j]);
            awa = awa + (sAwAmp[k - 1] * v[i][j] + sBwAmp[k - 1] * w[i][j]);
        }
    }

    aw = (awm + awa*cos(doy / 365.25 * 2 * PI))*1e-5;
    beta = bw / (sine + cw);
    gamma = aw / (sine + beta);
    topcon = (1.0 + aw / (1.0 + bw / (1.0 + cw)));

    mapw = topcon / (sine + gamma);

    return maph;
}


double GnssErrorModel::tropmap_vmf1(double ad, double aw, double& mapw){
    double ch, sine, beta, gamma, topcon;
    double maph, phh, c11h, c10h, ht_corr_coef, ht_corr;
    double bh = 0.0029, c0h = 0.062;
    double a_ht=2.53e-5, b_ht=5.49e-3, c_ht=1.14e-3, bw=0.00146, cw=0.04391;
    double doy = CTimes::time2doy(mTime);
    double dlat = mBLH[0], hgt=mBLH[2], hs_km = hgt / 1000.e0;


    if (dlat < 0.0)
    {   /* southern hemisphere*/
        phh  = PI;
        c11h = 0.007;
        c10h = 0.002;
    }
    else
    {   /* northern hemisphere*/
        phh  = 0.0;
        c11h = 0.005;
        c10h = 0.001;
    }

    ch = c0h + ((cos(doy / 365.25 * (2.0 * PI) + phh) + 1.0) * c11h / 2.0 + c10h) * (1.0 - cos(dlat));
    sine = sin(mElevation);
    beta = bh / (sine + ch);
    gamma = ad / (sine + beta);
    topcon = 1.0 + ad / (1.0 + bh / (1.0 + ch));
    maph = topcon / (sine + gamma);

    beta = b_ht / (sine + c_ht);
    gamma = a_ht / (sine + beta);
    topcon = 1.0 + a_ht / (1.0 + b_ht / (1.0 + c_ht));
    ht_corr_coef = 1.0 / sine - topcon / (sine + gamma);
    ht_corr = ht_corr_coef * hs_km;
    maph += ht_corr;

    beta = bw / (sine + cw);
    gamma = aw / (sine + beta);
    topcon = 1.0 + aw / (1.0 + bw / (1.0 + cw));
    mapw = topcon / (sine + gamma);

    return maph;
}


double GnssErrorModel::interpc(const double coef[], double lat){
    int i = (int)(lat/15.0);
    if      (i<1) return coef[0];
    else if (i>4) return coef[4];
    return coef[i-1] * (1.0-lat/15.0+i) + coef[i] * (lat/15.0-i);
}


double GnssErrorModel::mapf(double el, double a, double b, double c){
    double sinel = sin(el);
    return (1.0 + a / (1.0 + b / (1.0 + c))) / (sinel + (a / (sinel + b / (sinel + c))));
}


/* ionosphere delay by tec grid data ------------------------------------------
 *  opt    model option
 *         bit0: 0:earth-fixed,1:sun-fixed
 *         bit1: 0:single-layer,1:modified single-layer
 *-----------------------------------------------------------------------------*/
bool GnssErrorModel::iondelay(const IonTecGrid *tec, double&delay, double& var){
    int opt = 1;
    double fs, hion, rp;
    double vtec, rms;
    Vector3d posp = Vector3d::Zero(3);
    const double fact = 40.30E16 / FREQ1 / FREQ1; /* tecu->L1 iono (m) */

    delay = var = 0.0;

    for (int i = 0; i < tec->ndata[2]; i++) /* for a layer */
    {
        hion = tec->hgts[0] + tec->hgts[2] * i;

        /* ionospheric pierce point position */
        fs = ionppp(tec->rb, hion, posp);

        if (opt&2)
        {   /* modified single layer mapping function (M-SLM) ref [2] */
            rp = tec->rb / (tec->rb + hion) * sin(0.9782 * (PI / 2.0 - mElevation));
            fs = 1.0 / sqrt(1.0 - rp * rp);
        }
        if (opt&1)
        {   /* earth rotation correction (sun-fixed coordinate) */
            posp[1] += 2.0 * PI * CTimes::timediff(mTime, tec->time) / 86400.0;
        }

        /* interpolate tec grid data */
        if (!interptec(tec, i, posp, vtec, rms)) return false;
        delay += fact * fs * vtec;
        var   += fact * fact * fs * fs * rms * rms;
    }
    return true;
}


/* interpolate tec grid data -------------------------------------------------*/
bool GnssErrorModel::interptec(const IonTecGrid *tec, int k, const Vector3d& posp, double& value, double& rms){
    int i, j, n, index;
    double a, b, dlat, dlon;
    double d[4] = {0}, r[4] = {0};

    value = rms = 0.0;
    if (tec->lats[2]==0.0 || tec->lons[2]==0.0) return false;

    dlat = posp[0] * R2D - tec->lats[0];
    dlon = posp[1] * R2D - tec->lons[0];
    if (tec->lons[2]>0.0) dlon -= floor( dlon/360) * 360.0; /*  0<=dlon<360 */
    else                  dlon += floor(-dlon/360) * 360.0; /* -360<dlon<=0 */

    a = dlat/tec->lats[2];
    b = dlon/tec->lons[2];
    i = (int)floor(a); a-=i;
    j = (int)floor(b); b-=j;

    /* get gridded tec data */
    for (n = 0; n < 4; n++)
    {
        if ((index = DataIndex(i+(n%2), j+(n<2?0:1), k, tec->ndata))<0) continue;
        d[n] = tec->data[index];
        r[n] = tec->rms [index];
    }

    if (d[0]>0.0 && d[1]>0.0 && d[2]>0.0 && d[3]>0.0)
    {   /* bilinear interpolation (inside of grid) */
        value = (1.0-a) * (1.0-b) * d[0] + a * (1.0-b) * d[1] + (1.0-a) * b * d[2] + a * b * d[3];
        rms   = (1.0-a) * (1.0-b) * r[0] + a * (1.0-b) * r[1] + (1.0-a) * b * r[2] + a * b * r[3];
    }
    /* nearest-neighbour extrapolation (outside of grid) */
    else if (a<=0.5&&b<=0.5&&d[0]>0.0) {value=d[0]; rms=r[0];}
    else if (a> 0.5&&b<=0.5&&d[1]>0.0) {value=d[1]; rms=r[1];}
    else if (a<=0.5&&b> 0.5&&d[2]>0.0) {value=d[2]; rms=r[2];}
    else if (a> 0.5&&b> 0.5&&d[3]>0.0) {value=d[3]; rms=r[3];}
    else
    {
        i = 0;
        for (n=0;n<4;n++) if (d[n]>0.0) {i++; value += d[n];  rms += r[n];}
        if (i == 0) return false;
        value /= i; rms /= i;
    }
    return true;
}


/* data index (i:lat,j:lon,k:hgt) --------------------------------------------*/
int GnssErrorModel::DataIndex(int i, int j, int k, const int *ndata){
    if (i<0 || ndata[0]<=i || j<0 || ndata[1]<=j || k<0 || ndata[2]<=k) return -1;
    return i+ndata[0]*(j+ndata[1]*k);
}



bool SppErrorModel::IonosphericCorrect(const STimes tTime, GnssObsData& obs, const Vector3d& blh, const ReceiverInfo& RecInfo){
    double freq;
    mTime = tTime;
    mBLH = blh;
    mNavHead = RecInfo.nav_head;
    mAzimuth = obs.azimuth;
    mElevation = obs.elevation;
    mIonTec = RecInfo.IonTec;
    mIonTecNum = RecInfo.nIonTec;

    if (config.Ionospheric == IONOOPT_OFF || config.Ionospheric == IONOOPT_IFLC) return true;
    int sys = CString::satsys(obs.prn);

    /* broadcast model */
    if (config.Ionospheric == IONOOPT_BRDC)
    {
        if (sys == SYS_BDS) obs.ion = klobuchar_BDS();
        else                obs.ion = klobuchar();

        obs.ion_var = SQR(obs.ion * ERR_BRDCI);
        return true;
    }

    /* IONEX TEC model */
    if (config.Ionospheric == IONOOPT_TEC)
    {
       return this->IonTecGridDelay(obs.ion, obs.ion_var);
    }

    obs.ion = 0.0;
    // *var = ionoopt==IONOOPT_OFF?SQR(ERR_ION):0.0;

    if ((freq = CState::GetFrequency(obs.prn, 0, obs.sat.glofcn)) == 0.0) return false;
    obs.ion *= SQR(FREQ1 / freq);
    obs.ion_var *= SQR(FREQ1 / freq);
    return true;
}


/* tropospheric correction -----------------------------------------------------
* compute tropospheric correction
* args   : STimes time           I   time
*          double obs.trp        O   tropospheric delay (m)
*          double obs.trp_var    O   tropospheric delay variance (m^2)
* return : status(1:ok,0:error)
*-----------------------------------------------------------------------------*/
bool SppErrorModel::TroposphericCorrect(const STimes tTime, GnssObsData& obs, const Vector3d& blh){
    mTime = tTime;
    mBLH = blh;
    mElevation = obs.elevation;

    if (config.Tropospheric == TROPOPT_OFF) return true;

    /* Saastamoinen model */
    if (config.Tropospheric == TROPOPT_SAAS)
    {
        obs.trp = saastamoinen(REL_HUMI);
        obs.trp_var = SQR(ERR_SAAS / (sin(mElevation) + 0.1));
        return true;
    }

    /* SBAS (MOPS) troposphere model  todo*/

    return true;
}


double SppErrorModel::ObsCombination(const GnssObsData& obs){
    int f1, f2;
    double P;
    double alpha, beta;
    double freq1, freq2;

    if        (config.Ionospheric == IONOOPT_IFLC)
    {
        f1 = CState::GetFrequencyIndex(obs.prn,0);
        f2 = CState::GetFrequencyIndex(obs.prn,1);
        if (this->CheckSnrMask(obs.elevation, obs.SNR[f2] * SNR_UNIT)) return 0.0;
        if (obs.P[f1] < 1e-3 || obs.P[f2] < 1e-3) return 0.0;
        freq1 =  CState::GetFrequency(obs.prn, 0, 0);  // mObs.sat.glofcn ??
        freq2 =  CState::GetFrequency(obs.prn, 1, 0);
        alpha =  SQR(freq1) / (SQR(freq1) - SQR(freq2));
        beta  = -SQR(freq2) / (SQR(freq1) - SQR(freq2));
        P     =  alpha * obs.P[f1] + beta * obs.P[f2];
    }
    return P;

}


/* measurement error variance ------------------------------------------------*/
double SppErrorModel::ErrorVariance(int system, int type, GnssObsData& obs, double bl, uint16_t snr) {
    double fact;
    double el = obs.elevation<MIN_EL? MIN_EL: obs.elevation;

    switch (type)
    {
        case PSEUDORANGE1: fact = config.ErrRatio[0]; break;
        case PSEUDORANGE2: fact = config.ErrRatio[1]; break;
        case PSEUDORANGE3: fact = config.ErrRatio[2]; break;
        case DOPPLER     : fact = config.Error[3];    break;
        case TDCP        : fact = config.Error[3] / sqrt(config.ErrRatio[0]); break;
        default          : return 0;
    }

    switch (system)
    {
        case SYS_GPS:
        case SYS_BDS:
        case SYS_GAL: fact *= EFACT_GPS; break;
        case SYS_GLO: fact *= EFACT_GLO; break;
        default:      fact *= EFACT_GPS; break;
    }

    if (type == DOPPLER || type == TDCP) return SQR(fact) / sin(el); // doppler

    double a = fact * config.Error[0];
    double b = fact * config.Error[1];
    double snr_max  = config.Error[4];

    /* note: SQR(3.0) is approximated scale factor for error variance
       in the case of IF combination */
    fact = (config.Ionospheric == IONOOPT_IFLC) ? SQR(3.0) : 1.0; /* iono-free */
    switch (config.WeightMode)
     {
        case WEIGHTOPT_ELEVATION: return fact * ( SQR(a) + SQR(b) / sin(el));
        case WEIGHTOPT_SNR      : return fact * SQR(a) * pow(10, 0.1 * MAX(snr_max - snr, 0));
        default: return 0;
    }

}



bool RtkErrorModel::TroposphericCorrect(const STimes tTime, GnssObsData& obs, const Vector3d& blh){
    mTime = tTime;
    mBLH = blh;

    double mapfw = 0.0;
    if (config.Tropospheric == TROPOPT_OFF) return true;

    /* Saastamoinen model */
    if (config.Tropospheric == TROPOPT_SAAS)
    {
        mElevation = 90.0 * D2R;
        obs.trp = saastamoinen(0);
        mElevation = obs.elevation;
        obs.trp *= this->tropmapf(mapfw); //dry mapping function
        obs.trp_var = SQR(ERR_SAAS / (sin(mElevation) + 0.1));
        return true;
    }

    return true;
}


double RtkErrorModel::ErrorVariance(int system, int type, GnssObsData& obs, double bl, uint16_t snr){
    double a, b, c = config.Error[2] * bl / 1E4;
    double snr_max = config.Error[4];
    double fact;   // 0:Pseudorange, 1:phase L1, 2:phase L2, 3:phase L5 4:doppler
    double sinel = sin(obs.elevation);

    /* normal error model */
    switch (type)
    {  /* use err ratio only */
        case CARRIERPHASE: fact = 1.0;                break;
        case PSEUDORANGE1: fact = config.ErrRatio[0]; break;
        case PSEUDORANGE2: fact = config.ErrRatio[1]; break;
        case PSEUDORANGE3: fact = config.ErrRatio[2]; break;
        case DOPPLER     : fact = config.Error[3];    break;
        case TDCP        : fact = config.Error[3] / sqrt(config.ErrRatio[0]); break;
        default          : return 0;
    }

    switch (system)
    {
        case SYS_GPS: fact *= EFACT_GPS; break;
        case SYS_BDS: fact *= EFACT_BDS; break;
        case SYS_GLO: fact *= EFACT_GLO; break;
        case SYS_GAL: fact *= EFACT_GAL; break;
        case SYS_BD3: fact *= EFACT_BDS; break;
        default:      fact *= EFACT_GPS; break;
    }

    if (type == DOPPLER || type == TDCP)    return SQR(fact) / sinel; // doppler

    a = fact * config.Error[0];
    b = fact * config.Error[1];

    fact = 1.0;

    switch (config.WeightMode)
    {
        case WEIGHTOPT_ELEVATION: return fact * 2.0 * ( SQR(a) + SQR(b / sinel) + SQR(c) );
        case WEIGHTOPT_SNR      : return fact * SQR(a) * pow(10, 0.1 * MAX(snr_max - snr, 0));
        default: return 0;
    }
}



