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
 * Created by lizhen on 2022/8/18.
 *-------------------------------------------------------------------------------*/

#include "DebugLog.h"

void DebugLog::TraceLevel(int level) {
    mLevel = level;
}


bool DebugLog::TraceOpen(string filename){
    mStartTime = clock();
    mFile.open(filename, ios::out);
    if (!mFile.is_open())
    {
        return false;
    }
    return true;
}


void DebugLog::TraceClose(){
    if (!mFile.is_open()) mFile.close();
    mEndTime = clock();
    mTotalTime = (double)(mEndTime - mStartTime) / CLOCKS_PER_SEC;
    Trace(1, "Total program running time: " + to_string(mTotalTime) + " s");
}


void DebugLog::Trace(int level, string info){
    if (level > mLevel) return;
    time(&mTime);
    strftime(mSTime, sizeof(mSTime), "%H:%M:%S: ", localtime(&mTime));
    switch (level)
    {
        case 1 : cout << RED << mSTime << info << endl; break;
        case 2 : cout << GREEN << mSTime << info << endl; break;
    //    case 3 : cout << YELLOW << STime << info << endl; break;
        default:                                 break;
    }
    if (mFile.is_open()) mFile << info << endl;
}


 void DebugLog::Trace(int level, STimes tTime, string info){
    string timehms = CTimes::time2str(tTime) + " ";
    if (level > mLevel) return;
    time(&mTime);
    strftime(mSTime, sizeof(mSTime), "%H:%M:%S: ", localtime(&mTime));
    switch (level)
    {
        case 1 : cout << RED << mSTime << timehms << info << endl; break;
        case 2 : cout << GREEN << mSTime << timehms << info << endl; break;
    //    case 3 : cout << YELLOW << STime << info << endl; break;
        default:                                 break;
    }
    if (mFile.is_open()) mFile << timehms << info << endl;
 }


void DebugLog::TraceMat(int level, string info, const MatrixXd& mat) {
    if (level > mLevel) return;
    switch (level)
    {
        case 1 : cout << RED << mSTime << info << endl; break;
        case 2 : cout << GREEN << mSTime << info << endl; break;
    //    case 3 : cout << YELLOW << STime << info << endl; break;
        default:                                          break;
    }
    if (mFile.is_open())
    {
        mFile << info << ": " << endl;
        mFile << fixed << mat << endl;
    }
}


void DebugLog::TraceVec(int level, string info, const VectorXd& vec) {
    if (level > mLevel) return;
    switch (level)
    {
        case 1 : cout << RED << mSTime << info << endl; break;
        case 2 : cout << GREEN << mSTime << info << endl; break;
    //    case 3 : cout << GREEN  << STime << info << endl; break;
        default:                                          break;
    }
    if (mFile.is_open())
    {
        mFile << info << ": " << endl;
        mFile << fixed << vec << endl;
    }
}


void DebugLog::ShowState(string msg, int stat, int epoch){
    cerr << RED << "Processing:"
                << "(" << epoch << ")"
                << msg
                <<" Q=" << stat << "\r";
}

