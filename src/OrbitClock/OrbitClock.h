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

#ifndef ORBITCLOCK_H
#define ORBITCLOCK_H

#include "../Global.h"

class OrbitClock {
public:
    OrbitClock()  = default;
    ~OrbitClock() = default;

    virtual double CalSatClock(STimes tTime, int sys, int id) {
        Log.Trace(TERROR, "*CalSatClock function in OrbitClock class overload error, please check!");
        return 1e9;
    };
    virtual bool CalSatPosVel(STimes tTime, SatInfo& sat) {
        Log.Trace(TERROR, "CalSatPosVel function in OrbitClock class overload error, please check!");
        return false;
    };

protected:
    int mSystem = 0;       /* system of processing satellite                     */
    int mSatId = 0;        /* satellite id,G(1-32),C(33-82),R(83-109),E(110-145) */
};



#endif //ORBITCLOCK_H
