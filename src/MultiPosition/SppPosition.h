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
 * Created by lizhen on 2021/8/20.
 *-------------------------------------------------------------------------------*/


#ifndef SPPPOSITION_H
#define SPPPOSITION_H

#include "MultiPosition.h"
#include "../SensorErrorModel/GnssErrorModel.h"
#include "../Solver/SppGraphOptimize.h"
#include "../ResultExport/WriteResultFile.h"


class SppPosition:public MultiPosition{
public:
    SppPosition();
    ~SppPosition() = default;
    bool Processing() override;

    bool SimplePosition(GnssObsEpoch ob, GnssNavData& nav);
    void SetRoverInformation(ReceiverInfo info);

private:
    bool Initialization() override;
    bool DataPrepared() override;
    void FormVarMatrix(const vector<Equation> &eq) override; /* construction of variance R for pos   */

    bool SolvePosition();                              /* solve positon using least square method    */
    bool FormEquation();                               /* construction of positioning error equation */
    bool FormVelocityConstrain();
    bool SppPositionDesignMatrix(Equation& eq_t, const Vector3d& e);     /* lsq design mat construct */

private:
    bool RecClockBiasCorrect(Equation& eq_t);    /* time system offset and receiver bias correction  */
    bool FormConstrainDesignMatrix();            /* constraint to avoid rank-deficient               */

    bool MinNumSatellite();  /* check minimum satellites need for position 3+sys    */
    void CheckFrequency();   /* set as default frquency for spp                     */

    int mIter = 0;           /* number of iteration for least square algorithm      */
    int mSysMask[5] = {0};   /* Whether to use GPS, BD2, GLO, GAL, BD3. 0:no, 1:yes */

};


#endif //SPPPOSITION_H
