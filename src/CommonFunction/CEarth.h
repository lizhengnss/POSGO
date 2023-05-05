/*--------------------------------------------------------------------------------
 * POSGO: An Optimization-Based GNSS Navigation System
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
 *   [1]. P.D.Groves, Principles of GNSS, Intertial, and Multisensor Integrated
 *            Navigation System (Second Edition), 2013
 *   [2]. Global Navigation Satellite System GLONASS, Interface Control Document
 *            Navigational radiosignal In bands L1, L2, (Version 5.1), 2008
 *   [3]. IS-GPS-200K, Navstar GPS Space Segment/Navigation User Interfaces,
 *            May 6, 2019
 *   [4]. European GNSS (Galileo) Open Service Signal In Space Interface Control
 *            Document, Issue 1.3, December, 2016
 *   [5]. BeiDou navigation satellite system signal in space interface control
 *            document open service signal B1I (version 3.0), China Satellite
 *            Navigation office, February, 2019
 * Created by lizhen on 2022/4/19.
 *-------------------------------------------------------------------------------*/

#ifndef CEARTH_H
#define CEARTH_H


#include "Types.h"
#include <iostream>

#define SQR(x)      ((x)*(x))

static constexpr double WGS84_WIE = 7.2921151467E-5;       // 地球自转角速度
static constexpr double WGS84_FE  = 0.0033528106647474805; // 扁率 (1.0/298.257223563)
static constexpr double WGS84_RA  = 6378137.0000000000;    // 长半轴a
static constexpr double WGS84_RB  = 6356752.3142451793;    // 短半轴b
static constexpr double WGS84_GM0 = 398600441800000.00;    // 地球引力常数
static constexpr double WGS84_E   = 0.08181919084255230;   // 第一偏心率
static constexpr double WGS84_E1  = 0.0066943799901413156; // 第一偏心率平方
static constexpr double WGS84_E2  = 0.0067394967422764341; // 第二偏心率平方
static constexpr double WGS84_J2  = 1.082627E-3;           // 地球第二引力常数

/* Geophysical constants parameters ----------------------------------------------*/
static constexpr double RE_GLO  = 6378136.0;        /* radius of earth (m)            ref [2]  */
static constexpr double MU_GPS  = 3.9860050E14;     /* gravitational constant         ref [3]  */
static constexpr double MU_GLO  = 3.9860044E14;     /* gravitational constant         ref [2]  */
static constexpr double MU_GAL  = 3.986004418E14;   /* earth gravitational constant   ref [4]  */
static constexpr double MU_CMP  = 3.986004418E14;   /* earth gravitational constant   ref [5]  */

static constexpr double OMGE_GLO = 7.292115E-5;      /* earth angular velocity (rad/s) ref [2]  */
static constexpr double OMGE_GAL = 7.2921151467E-5;  /* earth angular velocity (rad/s) ref [4]  */
static constexpr double OMGE_CMP = 7.292115E-5;      /* earth angular velocity (rad/s) ref [5]  */


class CEarth {

public:
    /* transform geodetic to ecef position -----------------------------------------
     * transform geodetic position to ecef position
     * args   : Vector3d blh      I   geodetic position {lat,lon,h} (rad,m)
     * return : Vector3d pos      O   ecef position {x,y,z} (m)
     * notes  : WGS84, ellipsoidal height
     *-----------------------------------------------------------------------------*/
    static Vector3d blh2ecef(const Vector3d& blh) {
        double sinp = sin(blh[0]), cosp = cos(blh[0]), sinl = sin(blh[1]), cosl = cos(blh[1]);
        double e2 = WGS84_FE * (2.0 - WGS84_FE), v = WGS84_RA / sqrt(1.0 - e2 * sinp * sinp);
        double x, y, z;

        x = (v + blh[2]) * cosp * cosl;
        y = (v + blh[2]) * cosp * sinl;
        z = (v * (1.0 - e2) + blh[2]) * sinp;

        return {x, y, z};
    }

    /* transform ecef to geodetic postion ------------------------------------------
     * args   : Vector3d pos      I   ecef position {x,y,z} (m)
     * return : Vector3d blh      O   geodetic position {lat,lon,h} (rad,m)
     * notes  : WGS84, ellipsoidal height
     *-----------------------------------------------------------------------------*/
    static Vector3d ecef2blh(const Vector3d& pos) {
        double e2 = WGS84_FE * (2.0 - WGS84_FE), r2 = pos.dot(pos)-pos[2]*pos[2];
        double z, zk, v = WGS84_RA, sinp;
        double lat, lon, h;

        for (z = pos[2], zk = 0.0; fabs(z - zk) >= 1E-4;)
        {
            zk = z;
            sinp = z / sqrt(r2 + z * z);
            v = WGS84_RA / sqrt(1.0 - e2 * sinp * sinp);
            z = pos[2] + v * e2 * sinp;
        }
        lat = r2>1E-12? atan(z / sqrt(r2)): (pos[2] > 0.0? PI / 2.0: -PI / 2.0);
        lon = r2>1E-12? atan2(pos[1], pos[0]): 0.0;
        h = sqrt(r2 + z * z) - v;
        return {lat, lon, h};
    }

    /* compute ecef to local coordinate transfromation matrix-----------------------
    * args   : Vector3d pos      I   geodetic position {lat,lon} (rad)
    * return : Matrix3d E        O   ecef to local coord transformation Matrix3d
    *-----------------------------------------------------------------------------*/
    static Matrix3d xyz2enu(const Vector3d& pos){
        Matrix3d E =  Matrix3d::Zero(3, 3);
        double sinp = sin(pos[0]), cosp = cos(pos[0]);
        double sinl = sin(pos[1]), cosl = cos(pos[1]);

        E(0, 0) = -sinl     ;  E(0, 1) =  cosl     ;  E(0, 2) =  0.0;
        E(1, 0) = -sinp*cosl;  E(1, 1) = -sinp*sinl;  E(1, 2) = cosp;
        E(2, 0) =  cosp*cosl;  E(2, 1) =  cosp*sinl;  E(2, 2) = sinp;
        return E;
    }


    /* transform ecef vector to local tangental coordinate -------------------------
    * args   : Vector3d pos      I   geodetic position {lat,lon} (rad)
    *          Vector3d r        I   vector in ecef coordinate {x,y,z}
    * return : Vector3d e        O   vector in local tangental coordinate {e,n,u}
    *-----------------------------------------------------------------------------*/
    static Vector3d ecef2enu(const Vector3d& pos, const Vector3d r){
        Vector3d e  = Vector3d::Zero(3);
        Matrix3d E  = Matrix3d::Zero(3, 3);
        MatrixXd Mr = MatrixXd::Zero(3, 1);

        Mr << r[0], r[1], r[2];
        E = xyz2enu(pos);
        e = E * Mr;
        return e;
    }


    /* geoid height ----------------------------------------------------------------
    * get geoid height from geoid model
    * args   : Vector3d pos      I   geodetic position {lat,lon} (rad)
    * return : double   h        O   geoid height (m) (0.0:error)
    * notes  : to use external geoid model, call function opengeoid() to open
    *          geoid model before calling the function. If the external geoid model
    *          is not open, the function uses embedded geoid model.
    *-----------------------------------------------------------------------------*/
    static double geoidh(const Vector3d& pos) {
        double posd[2],h;
        posd[0] = pos[0] * R2D;
        posd[1] = pos[1] * R2D;
        if (posd[1] < 0.0) posd[1] += 360.0;
        if (posd[1] < 0.0 || 360.0 - 1E-12 < posd[1] || posd[0] < -90.0 || 90.0 < posd[0])
        {
            std::cout<< GREEN << "geoidh: out of range for geoid model, lat=" << posd[0]
                              << " lon=" << posd[1];
            return 0.0;
        }

        h = geoidh_emb (posd); // todo other datum

        if (fabs(h)>200.0)
        {
            std::cout << GREEN << "geoidh: invalid geoid model, lat=" << posd[0] << " lon=" << posd[1]; 
            return 0.0;
        }
        return h;
    }

private:
    /* embedded geoid model ------------------------------------------------------*/
    static double geoidh_emb(const double *pos){
        int i1,i2,j1,j2;
        double a,b,y[4];
        const double dlon = 1.0, dlat = 1.0;
        const double range[] = {0.00, 360.00, -90.00, 90.00};
        if (pos[1] < range[0] || range[1] < pos[1] || pos[0] < range[2] || range[3] < pos[0])
        {
            std::cout<< GREEN << "geoidh_emb: out of geoid model range, lat=" << pos[0]
                              << " lon=" << pos[1];
            return 0.0;
        }
        a = (pos[1] - range[0]) / dlon;
        b = (pos[0] - range[2]) / dlat;
        i1 = (int)a; a -= i1; i2 = i1<360? i1+1: i1;
        j1 = (int)b; b -= j1; j2 = j1<180? j1+1: j1;

        return interpb(y,a,b);
    }


    /* bilinear interpolation ----------------------------------------------------*/
    static double interpb(const double *y, double a, double b){
        return y[0] * (1.0-a) * (1.0-b) + y[1] * a * (1.0-b) + y[2] * (1.0-a) * b + y[3] * a * b;
    }

};


#endif //CEARTH_H
