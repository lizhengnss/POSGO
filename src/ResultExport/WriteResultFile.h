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
 * Created by lizhen on 2021/8/24.
 *-------------------------------------------------------------------------------*/

#ifndef WRITERESULTFILE_H
#define WRITERESULTFILE_H

#include "ResultExport.h"

class WriteResultFile: public ResultExport{
public:
    WriteResultFile();
    ~WriteResultFile() = default;
    bool Export(Solution& sol) override;
    void Close() override;               /* close result file  */
private:
    void WriteHead();                    /* export necessary information to head of file   */
    void WriteSystemHead();              /* export system used to head of result file      */
    void WriteFrequencyHead();           /* export frequency of each system used to head   */
    void WriteResultHead();              /* export result index to head of result file     */

    void WriteTimeTOW();                 /* export time of GPS week and sec of week format */
    void WriteTimeHMS();                 /* export time of yyyy/mm/dd hh:mm:ss.ssss format */
    void WritePositionECEF();            /* export position in ECEF coordinate system      */
    void WritePositionBLH();             /* export position in geodetic coordinate system  */
    void WriteVelocityECEF();            /* export velocity in ECEF coordinate system      */
    void WriteVelocityBLH();             /* export velocity in geodetic coordinate system  */
    void WriteAccelerationECEF();        /* export acceleration in ECEF coordinate system  */
    void WriteAccelerationBLH();         /* export acceleration in geodetic coordinate     */
    void WriteClock();                   /* export clock bias of each system, unit:m       */
    void WriteClockDrift();              /* export clock drift of receiver                 */

private:
    Solution* mSol = nullptr;
    string mSep;
    ofstream mOutfile;                   /* iostream of output file                        */
    string sFreqStr[5][MAXFREQ]={
            {"",    "",     "",   "",    "",     "",    ""},
            {"L1",  "L2",  "L5",  "",    "",     "",    ""},       /*GPS*/
            {"B1I", "B2I", "B3I", "B1C", "B2a",  "B2ab", "B2b"},   /*BDS*/
            {"G1",  "G2",  "G3",  "G1a", "G2a",  "",    ""},       /*GLO*/  // todo G1a and G2b
            {"E1",  "E5b", "E5a", "E6",  "E5ab", "",    ""},       /*GAL*/
    };

};


class WriteTempResult: public ResultExport{
public:
    WriteTempResult();
    ~WriteTempResult() = default;
    bool Export(Solution& sol) override;
    void Close() override;                /* close result file                             */

private:
    Solution* mSol = nullptr;
    string mSep;
    ofstream mOutfile;                    /* iostream of output file                       */
    void WriteStates();                   /* write state to temp result file               */
    void WriteVariance();                 /* write corresponding variance to temp file     */

};


#endif //WRITERESULTFILE_H
