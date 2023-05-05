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
 * Created by lizhen on 2022/4/24.
 *-------------------------------------------------------------------------------*/

#ifndef CMATHS_H
#define CMATHS_H

#include "Types.h"


class CMaths {
public:

    /* convert degree to degree-minute-second---------------------------------------
    * args   : double deg       I   degree
    *          int    ndec      I   number of decimals of second
    * return : vector           O   degree, minute, sec
    *-----------------------------------------------------------------------------*/
    static vector<double> deg2dms(double deg, int ndec){
        vector<double> time;
        double sign = deg < 0.0? -1.0: 1.0, a = fabs(deg);
        double unit = pow(0.1, ndec);

        time.push_back(floor(a)); a=(a - time[0]) * 60.0;
        time.push_back(floor(a)); a=(a - time[1]) * 60.0;
        time.push_back(floor(a / unit + 0.5) * unit);

        if (time[2] >= 60.0)
        {
            time[2] = 0.0;
            time[1] += 1.0;
            if (time[1] >= 60.0)
            {
                time[1] = 0.0;
                time[0] += 1.0;
            }
        }

        time[0] *= sign;
        return time;
    }

    static double normalPDF(double d){
        const double A1 = 0.31938153;
        const double A2 = -0.356563782;
        const double A3 = 1.781477937;
        const double A4 = -1.821255978;
        const double A5 = 1.330274429;
        const double RSQRT2PI = 0.39894228040143267793994605993438;
        double k = 1.0 / (1.0 + 0.2316419 * fabs(d));
        double cnd = RSQRT2PI * exp(- 0.5 * d * d) * (k * (A1 + k * (A2 + k * (A3 + k * (A4 + k * A5)))));
        if (d > 0)
        {
            cnd = 1.0 - cnd;
        }
        return cnd;
    }

    static double distance(const Vector3d& a, const Vector3d& b){
        Vector3d dr;
        for (int i = 0; i < 3; i++) dr(i) = a(i) - b(i);
        return dr.norm();
    }


};


#endif //CMATHS_H
