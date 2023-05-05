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

#ifndef NAVORBITCLOCK_H
#define NAVORBITCLOCK_H

#include "OrbitClock.h"

#define MAXDTOE     7200.0        /* max time difference to GPS Toe (s)     */
#define MAXDTOE_GAL 14400.0       /* max time difference to Galileo Toe (s) */
#define MAXDTOE_BDS 21600.0       /* max time difference to BeiDou Toe (s)  */
#define MAXDTOE_GLO 1800.0        /* max time difference to GLONASS Toe (s) */

#define SIN_5 -0.0871557427476582 /* sin(-5.0 deg) */
#define COS_5  0.9961946980917456 /* cos(-5.0 deg) */

#define TSTEP           60.0      /* integration step glonass ephemeris (s) */
#define RTOL_KEPLER     1E-13     /* relative tolerance for Kepler equation */
#define MAX_ITER_KEPLER 30        /* max number of iteration of Kelpler     */

#define STD_GAL_NAPA 500.0        /* error of galileo ephemeris for NAPA (m) */
#define ERREPH_GLO   5.0          /* error of glonass ephemeris (m)          */

class NavOrbClk:public OrbitClock{
public:
    NavOrbClk()  = default;
    ~NavOrbClk() = default;
    NavOrbClk(GnssNavData& nav_t);

    double CalSatClock(STimes tTime, int sys, int id) override;
    bool CalSatPosVel(STimes tTime, SatInfo& sat) override;

private:
    GnssNavData mNavs;                          /* all navigation data                  */
    Ephemeris mEph_t;                           /* ephemeris of GEC  temporary variable */
    GEphemeris mGeph_t;                         /* glonass ephemeris temporary variable */

    bool SelectEph(STimes tTime);               /* select ephememeris according to closest time             */
    bool SelectGEph(STimes tTime);              /* select glonass ephememeris according to closest time     */
    double eph2clk(STimes tTime);               /* broadcast ephemeris to satellite clock bias              */
    double geph2clk(STimes tTime);              /* glonass ephemeris to satellite clock bias                */

    void eph2pos(STimes tTime, SatInfo& sat);   /* broadcast ephemeris to satellite position and clock bias */
    void geph2pos(STimes tTime, SatInfo& sat);  /* glonass ephemeris to satellite position and clock bias   */
    void glorbit(double t, double *x);          /* glonass position and velocity by numerical integration   */
    void deq(const double *x, double *xdot, const double *acc);   /* glonass orbit differential equations   */

    double var_uraeph(int system);              /* variance by ura ephemeris           */
};


#endif //NAVORBITCLOCK_H
