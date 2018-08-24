/*
 * This is heavily based on
 * https://github.com/osqzss/gps-sdr-sim
 *
 * Copyright © 2015-2018 Takuji Ebinuma
 * Distributed under the MIT License.
 */

#include "gpsgen.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <clocale>

#define MULTITHREADED 1

#if MULTITHREADED
#include <QtConcurrent/QtConcurrent>
#include <QVector>
#endif

#include <QDebug>
#include <QFile>

#define SECONDS_IN_WEEK 604800.0
#define SECONDS_IN_HALF_WEEK 302400.0
#define SECONDS_IN_DAY 86400.0
#define SECONDS_IN_HOUR 3600.0
#define SECONDS_IN_MINUTE 60.0

#define POW2_M5  0.03125
#define POW2_M19 1.907348632812500e-6
#define POW2_M29 1.862645149230957e-9
#define POW2_M31 4.656612873077393e-10
#define POW2_M33 1.164153218269348e-10
#define POW2_M43 1.136868377216160e-13
#define POW2_M55 2.775557561562891e-17

#define POW2_M50 8.881784197001252e-016
#define POW2_M30 9.313225746154785e-010
#define POW2_M27 7.450580596923828e-009
#define POW2_M24 5.960464477539063e-008

// Conventional values employed in GPS ephemeris model (ICD-GPS-200)
#define GM_EARTH 3.986005e14
#define OMEGA_EARTH 7.2921151467e-5

#define WGS84_RADIUS	6378137.0
#define WGS84_ECCENTRICITY 0.0818191908426

#define R2D (180.0 / M_PI)

#define SPEED_OF_LIGHT 2.99792458e8
#define LAMBDA_L1 0.190293672798365

/*! \brief GPS L1 Carrier frequency */
#define CARR_FREQ (1575.42e6)
/*! \brief C/A code frequency */
#define CODE_FREQ (1.023e6)
#define CARR_TO_CODE (1.0/1540.0)

namespace
{
int sinTable512[] = {
    2,   5,   8,  11,  14,  17,  20,  23,  26,  29,  32,  35,  38,  41,  44,  47,
    50,  53,  56,  59,  62,  65,  68,  71,  74,  77,  80,  83,  86,  89,  91,  94,
    97, 100, 103, 105, 108, 111, 114, 116, 119, 122, 125, 127, 130, 132, 135, 138,
    140, 143, 145, 148, 150, 153, 155, 157, 160, 162, 164, 167, 169, 171, 173, 176,
    178, 180, 182, 184, 186, 188, 190, 192, 194, 196, 198, 200, 202, 204, 205, 207,
    209, 210, 212, 214, 215, 217, 218, 220, 221, 223, 224, 225, 227, 228, 229, 230,
    232, 233, 234, 235, 236, 237, 238, 239, 240, 241, 241, 242, 243, 244, 244, 245,
    245, 246, 247, 247, 248, 248, 248, 249, 249, 249, 249, 250, 250, 250, 250, 250,
    250, 250, 250, 250, 250, 249, 249, 249, 249, 248, 248, 248, 247, 247, 246, 245,
    245, 244, 244, 243, 242, 241, 241, 240, 239, 238, 237, 236, 235, 234, 233, 232,
    230, 229, 228, 227, 225, 224, 223, 221, 220, 218, 217, 215, 214, 212, 210, 209,
    207, 205, 204, 202, 200, 198, 196, 194, 192, 190, 188, 186, 184, 182, 180, 178,
    176, 173, 171, 169, 167, 164, 162, 160, 157, 155, 153, 150, 148, 145, 143, 140,
    138, 135, 132, 130, 127, 125, 122, 119, 116, 114, 111, 108, 105, 103, 100,  97,
    94,  91,  89,  86,  83,  80,  77,  74,  71,  68,  65,  62,  59,  56,  53,  50,
    47,  44,  41,  38,  35,  32,  29,  26,  23,  20,  17,  14,  11,   8,   5,   2,
    -2,  -5,  -8, -11, -14, -17, -20, -23, -26, -29, -32, -35, -38, -41, -44, -47,
    -50, -53, -56, -59, -62, -65, -68, -71, -74, -77, -80, -83, -86, -89, -91, -94,
    -97,-100,-103,-105,-108,-111,-114,-116,-119,-122,-125,-127,-130,-132,-135,-138,
    -140,-143,-145,-148,-150,-153,-155,-157,-160,-162,-164,-167,-169,-171,-173,-176,
    -178,-180,-182,-184,-186,-188,-190,-192,-194,-196,-198,-200,-202,-204,-205,-207,
    -209,-210,-212,-214,-215,-217,-218,-220,-221,-223,-224,-225,-227,-228,-229,-230,
    -232,-233,-234,-235,-236,-237,-238,-239,-240,-241,-241,-242,-243,-244,-244,-245,
    -245,-246,-247,-247,-248,-248,-248,-249,-249,-249,-249,-250,-250,-250,-250,-250,
    -250,-250,-250,-250,-250,-249,-249,-249,-249,-248,-248,-248,-247,-247,-246,-245,
    -245,-244,-244,-243,-242,-241,-241,-240,-239,-238,-237,-236,-235,-234,-233,-232,
    -230,-229,-228,-227,-225,-224,-223,-221,-220,-218,-217,-215,-214,-212,-210,-209,
    -207,-205,-204,-202,-200,-198,-196,-194,-192,-190,-188,-186,-184,-182,-180,-178,
    -176,-173,-171,-169,-167,-164,-162,-160,-157,-155,-153,-150,-148,-145,-143,-140,
    -138,-135,-132,-130,-127,-125,-122,-119,-116,-114,-111,-108,-105,-103,-100, -97,
    -94, -91, -89, -86, -83, -80, -77, -74, -71, -68, -65, -62, -59, -56, -53, -50,
    -47, -44, -41, -38, -35, -32, -29, -26, -23, -20, -17, -14, -11,  -8,  -5,  -2
};

int cosTable512[] = {
    250, 250, 250, 250, 250, 249, 249, 249, 249, 248, 248, 248, 247, 247, 246, 245,
    245, 244, 244, 243, 242, 241, 241, 240, 239, 238, 237, 236, 235, 234, 233, 232,
    230, 229, 228, 227, 225, 224, 223, 221, 220, 218, 217, 215, 214, 212, 210, 209,
    207, 205, 204, 202, 200, 198, 196, 194, 192, 190, 188, 186, 184, 182, 180, 178,
    176, 173, 171, 169, 167, 164, 162, 160, 157, 155, 153, 150, 148, 145, 143, 140,
    138, 135, 132, 130, 127, 125, 122, 119, 116, 114, 111, 108, 105, 103, 100,  97,
    94,  91,  89,  86,  83,  80,  77,  74,  71,  68,  65,  62,  59,  56,  53,  50,
    47,  44,  41,  38,  35,  32,  29,  26,  23,  20,  17,  14,  11,   8,   5,   2,
    -2,  -5,  -8, -11, -14, -17, -20, -23, -26, -29, -32, -35, -38, -41, -44, -47,
    -50, -53, -56, -59, -62, -65, -68, -71, -74, -77, -80, -83, -86, -89, -91, -94,
    -97,-100,-103,-105,-108,-111,-114,-116,-119,-122,-125,-127,-130,-132,-135,-138,
    -140,-143,-145,-148,-150,-153,-155,-157,-160,-162,-164,-167,-169,-171,-173,-176,
    -178,-180,-182,-184,-186,-188,-190,-192,-194,-196,-198,-200,-202,-204,-205,-207,
    -209,-210,-212,-214,-215,-217,-218,-220,-221,-223,-224,-225,-227,-228,-229,-230,
    -232,-233,-234,-235,-236,-237,-238,-239,-240,-241,-241,-242,-243,-244,-244,-245,
    -245,-246,-247,-247,-248,-248,-248,-249,-249,-249,-249,-250,-250,-250,-250,-250,
    -250,-250,-250,-250,-250,-249,-249,-249,-249,-248,-248,-248,-247,-247,-246,-245,
    -245,-244,-244,-243,-242,-241,-241,-240,-239,-238,-237,-236,-235,-234,-233,-232,
    -230,-229,-228,-227,-225,-224,-223,-221,-220,-218,-217,-215,-214,-212,-210,-209,
    -207,-205,-204,-202,-200,-198,-196,-194,-192,-190,-188,-186,-184,-182,-180,-178,
    -176,-173,-171,-169,-167,-164,-162,-160,-157,-155,-153,-150,-148,-145,-143,-140,
    -138,-135,-132,-130,-127,-125,-122,-119,-116,-114,-111,-108,-105,-103,-100, -97,
    -94, -91, -89, -86, -83, -80, -77, -74, -71, -68, -65, -62, -59, -56, -53, -50,
    -47, -44, -41, -38, -35, -32, -29, -26, -23, -20, -17, -14, -11,  -8,  -5,  -2,
    2,   5,   8,  11,  14,  17,  20,  23,  26,  29,  32,  35,  38,  41,  44,  47,
    50,  53,  56,  59,  62,  65,  68,  71,  74,  77,  80,  83,  86,  89,  91,  94,
    97, 100, 103, 105, 108, 111, 114, 116, 119, 122, 125, 127, 130, 132, 135, 138,
    140, 143, 145, 148, 150, 153, 155, 157, 160, 162, 164, 167, 169, 171, 173, 176,
    178, 180, 182, 184, 186, 188, 190, 192, 194, 196, 198, 200, 202, 204, 205, 207,
    209, 210, 212, 214, 215, 217, 218, 220, 221, 223, 224, 225, 227, 228, 229, 230,
    232, 233, 234, 235, 236, 237, 238, 239, 240, 241, 241, 242, 243, 244, 244, 245,
    245, 246, 247, 247, 248, 248, 248, 249, 249, 249, 249, 250, 250, 250, 250, 250
};

// Receiver antenna attenuation in dB for boresight angle = 0:5:180 [deg]
double ant_pat_db[37] = {
    0.00,  0.00,  0.22,  0.44,  0.67,  1.11,  1.56,  2.00,  2.44,  2.89,  3.56,  4.22,
    4.89,  5.56,  6.22,  6.89,  7.56,  8.22,  8.89,  9.78, 10.67, 11.56, 12.44, 13.33,
    14.44, 15.56, 16.67, 17.78, 18.89, 20.00, 21.33, 22.67, 24.00, 25.56, 27.33, 29.33,
    31.56
};
}

GpsGen::GpsGen()
{
    for (int i = 0;i < mMaxChan;i++) {
        mIqBufChan[i] = new short[1000000];
    }
}

GpsGen::~GpsGen()
{
    for (int i = 0;i < mMaxChan;i++) {
        delete mIqBufChan[i];
    }
}

/**
 * @brief GpsSim::allocateBuffers
 * Allocate enough space in the buffers for a given sample rate. The default buffers have enough
 * space for up to 5 MSPS.
 *
 * @param sampleRate
 * The samplerate in Hz
 */
void GpsGen::allocateBuffers(double sampleRate)
{
    for (int i = 0;i < mMaxChan;i++) {
        delete mIqBufChan[i];
    }

    int bufSize = ceil(sampleRate) * 2 * 0.01;

    for (int i = 0;i < mMaxChan;i++) {
        mIqBufChan[i] = new short[bufSize];
    }

    (void)sampleRate;
}

void GpsGen::resetState()
{
    memset(mAllocatedSat, 0, sizeof(mAllocatedSat));
    memset(mXyz, 0, sizeof(mXyz));
    memset(mXyzPrev, 0, sizeof(mXyzPrev));
    memset(mEph, 0, sizeof(mEph));
    memset(mChan, 0, sizeof(mChan));

    mEphSet = false;
    mElvmask = 0.0;
    mSpeedNow = 0.0;
}

bool GpsGen::readNavigationFile(const char *filename)
{
    mNeph = readRinexNavAll(mEph, &mIonoutc, filename);

    if (mNeph==0) {
        qWarning() << "ERROR: No ephemeris available";
        return false;
    }

    for (int sv = 0;sv < mMaxSat;sv++) {
        if (mEph[0][sv].vflg==1) {
            mGmin = mEph[0][sv].toc;
            mTmin = mEph[0][sv].t;
            break;
        }
    }

    mGmax.sec = 0;
    mGmax.week = 0;
    mTmax.sec = 0;
    mTmax.mm = 0;
    mTmax.hh = 0;
    mTmax.d = 0;
    mTmax.m = 0;
    mTmax.y = 0;

    for (int sv = 0;sv < mMaxSat;sv++) {
        if (mEph[mNeph - 1][sv].vflg == 1) {
            mGmax = mEph[mNeph-1][sv].toc;
            mTmax = mEph[mNeph-1][sv].t;
            break;
        }
    }

    mG0 = mGmin;
    mT0 = mTmin;

    // Select the current set of ephemerides
    mIeph = -1;

    for (int i = 0;i < mNeph;i++) {
        for (int sv = 0;sv < mMaxSat;sv++) {
            if (mEph[i][sv].vflg == 1) {
                double dt = subGpsTime(mG0, mEph[i][sv].toc);
                if (dt >= -SECONDS_IN_HOUR && dt < SECONDS_IN_HOUR) {
                    mIeph = i;
                    break;
                }
            }
        }

        if (mIeph >= 0) {
            break;
        }
    }

    if (mIeph == -1) {
        qWarning() << "ERROR: No current set of ephemerides has been found";
        return false;
    }

    // Clear all channels
    for (int i = 0;i < mMaxChan;i++) {
        mChan[i].prn = 0;
    }

    // Clear satellite allocation flag
    for (int sv = 0;sv < mMaxSat;sv++) {
        mAllocatedSat[sv] = -1;
    }

    // Initial reception time
    mGrx = incGpsTime(mG0, 0.0);

//    QDateTime now_qt = QDateTime::currentDateTimeUtc();
//    datetime_t now;
//    now.y = now_qt.date().year();
//    now.d = now_qt.date().day();
//    now.m = now_qt.date().month();
//    now.hh = now_qt.time().hour();
//    now.mm = now_qt.time().minute();
//    now.sec = (double)now_qt.time().msec() / 1000.0;
//    date2gps(&now, &mGrx);

    // Allocate visible satellites
    allocateChannel(mChan, mEph[mIeph], mIonoutc, mGrx, mXyz, mElvmask);

    // Receiver antenna gain pattern
    for (int i = 0;i < 37;i++) {
        mAntPat[i] = pow(10.0, -ant_pat_db[i] / 20.0);
    }

    mEphSet = true;

    return true;
}

void GpsGen::setPos(double *llh, double speed)
{
    llh[0] = llh[0] / R2D;
    llh[1] = llh[1] / R2D;

    llh2xyz(llh, mXyz);
    mSpeedNow = fabs(speed) * 3.6;

    double dp[3];
    subVect(dp, mXyz, mXyzPrev);
    double dd = normVect(dp);

    if (dd > 10.0) {
        memcpy(mXyzPrev, mXyz, sizeof(mXyzPrev));
    }
}

/**
 * @brief GpsSim::generateSamples
 * Generate samples
 *
 * @param buffer
 * Buffer to store the samples. Must be
 * sampFreq / 100 * durationSecD100 * 2
 * in size.
 *
 * @param durationSecD100
 * Duration in 1/100 sec
 *
 * @param sampFreq
 * Sample rate in Hz
 */
void GpsGen::generateSamples(int16_t *buffer, int durationSecD100, double sampFreq)
{
    if (!mEphSet) {
        qWarning() << "ERROR: Navigation file not set";
        return;
    }

    int buffer_index = 0;
    int gain[mMaxChan];

    // Sample buffer
    sampFreq = floor(sampFreq / 100.0);
    int iq_buff_size = (int)sampFreq; // samples per 0.01sec
    sampFreq *= 100.0;

    double delt = 1.0 / sampFreq;

    for (int dur = 0;dur < durationSecD100;dur++) {
        double maxKmh = mSpeedNow + 2.0;
        double step = maxKmh / 3.6 / 100.0;
        stepTowards(&mXyzPrev[0], mXyz[0], step);
        stepTowards(&mXyzPrev[1], mXyz[1], step);
        stepTowards(&mXyzPrev[2], mXyz[2], step);

//        memcpy(mXyzPrev, mXyz, sizeof(mXyz));

        for (int i = 0;i < mMaxChan;i++) {
            if (mChan[i].prn > 0) {
                // Refresh code phase and data bit counters
                range_t rho;
                int sv = mChan[i].prn - 1;

                // Current pseudorange
                computeRange(&rho, mEph[mIeph][sv], &mIonoutc, mGrx, mXyzPrev);

                mChan[i].azel[0] = rho.azel[0];
                mChan[i].azel[1] = rho.azel[1];

                // Update code phase and data bit counters
                computeCodePhase(&mChan[i], rho, 0.01);

                // Path loss
                double path_loss = 20200000.0 / rho.d;

                // Receiver antenna gain
                int ibs = (int)((90.0-rho.azel[1]*R2D)/5.0); // covert elevation to boresight
                double ant_gain = mAntPat[ibs];

                // Signal gain
                gain[i] = (int)(path_loss * ant_gain * 128.0); // scaled by 2^7
            }
        }

#if MULTITHREADED
        QVector<QFuture<void> > futures;
#endif

        bool channelsUpdated[mMaxChan];
        for (int i = 0;i < mMaxChan;i++) {
            channelsUpdated[i] = mChan[i].prn > 0;
            if (channelsUpdated[i]) {
#if MULTITHREADED
                futures.append(QtConcurrent::run(this, &GpsGen::generateChannelSamples, &mChan[i], iq_buff_size, gain[i], delt, mIqBufChan[i]));
#else
                generateChannelSamples(&mChan[i], iq_buff_size, gain[i], delt, mIqBufChan[i]);
#endif
            }
        }

#if MULTITHREADED
        for (int i = 0;i < futures.size();i++) {
            futures[i].waitForFinished();
        }
#endif

        for (int isamp=0;isamp < iq_buff_size;isamp++) {
            int i_acc = 0;
            int q_acc = 0;

            for (int i = 0;i < mMaxChan;i++) {
                if (channelsUpdated[i]) {
                    i_acc += mIqBufChan[i][2 * isamp];
                    q_acc += mIqBufChan[i][2 * isamp + 1];
                }
            }

            // Scaled by 2^7
            i_acc = (i_acc+64)>>7;
            q_acc = (q_acc+64)>>7;

            // Store I/Q samples into buffer
            buffer[buffer_index++] = (short)i_acc;
            buffer[buffer_index++] = (short)q_acc;
        }

        //
        // Update navigation message and channel allocation every 30 seconds
        //

        int igrx = (int)(mGrx.sec*10.0+0.5);

        // Every 30 seconds
        if (igrx % 300 == 0) {
            // Update navigation message
            for (int i = 0;i < mMaxChan;i++) {
                if (mChan[i].prn > 0) {
                    generateNavMsg(mGrx, &mChan[i], 0);
                }
            }

            // Refresh ephemeris and subframes
            // Quick and dirty fix. Need more elegant way.
            for (int sv = 0;sv < mMaxSat;sv++) {
                if (mEph[mIeph+1][sv].vflg == 1) {
                    double dt = subGpsTime(mEph[mIeph+1][sv].toc, mGrx);
                    if (dt < SECONDS_IN_HOUR) {
                        mIeph++;

                        for (int i = 0;i < mMaxChan;i++) {
                            // Generate new subframes if allocated
                            if (mChan[i].prn!=0) {
                                eph2sbf(mEph[mIeph][mChan[i].prn-1], mIonoutc, mChan[i].sbf);
                            }
                        }
                    }

                    break;
                }
            }

            // Update channel allocation
            allocateChannel(mChan, mEph[mIeph], mIonoutc, mGrx, mXyz, mElvmask);
        }

        // Update receiver time
        mGrx = incGpsTime(mGrx, 0.01);
    }
}

void GpsGen::stepTowards(double *value, double goal, double step)
{
    if (*value < goal) {
        if ((*value + step) < goal) {
            *value += step;
        } else {
            *value = goal;
        }
    } else if (*value > goal) {
        if ((*value - step) > goal) {
            *value -= step;
        } else {
            *value = goal;
        }
    }
}

void GpsGen::generateChannelSamples(GpsGen::channel_t *chan, int samples, int gain, double dt, short *buffer)
{
    for (int isamp = 0;isamp < samples;isamp++) {
        int iTable = (int)floor(chan->carr_phase * 512.0);

        buffer[isamp * 2] = chan->dataBit * chan->codeCA * cosTable512[iTable] * gain;
        buffer[isamp * 2 + 1] = chan->dataBit * chan->codeCA * sinTable512[iTable] * gain;

        // Update code phase
        chan->code_phase += chan->f_code * dt;

        if (chan->code_phase >= mCaSeqLen) {
            chan->code_phase -= mCaSeqLen;

            chan->icode++;

            // 20 C/A codes = 1 navigation data bit
            if (chan->icode >= 20) {
                chan->icode = 0;
                chan->ibit++;

                // 30 navigation data bits = 1 word
                if (chan->ibit>=30) {
                    chan->ibit = 0;
                    chan->iword++;
                }

                // Set new navigation data bit
                chan->dataBit = (int)((chan->dwrd[chan->iword] >> (29 - chan->ibit)) & 0x1UL) * 2 - 1;
            }
        }

        // Set currnt code chip
        chan->codeCA = chan->ca[(int)chan->code_phase] * 2 - 1;

        // Update carrier phase
        chan->carr_phase += chan->f_carr * dt;

        while (chan->carr_phase >= 1.0) {
            chan->carr_phase -= 1.0;
        }

        while (chan->carr_phase < 0.0) {
            chan->carr_phase += 1.0;
        }
    }
}

void GpsGen::subVect(double *y, const double *x1, const double *x2)
{
    y[0] = x1[0]-x2[0];
    y[1] = x1[1]-x2[1];
    y[2] = x1[2]-x2[2];
}

double GpsGen::normVect(const double *x)
{
    return(sqrt(x[0]*x[0]+x[1]*x[1]+x[2]*x[2]));
}

double GpsGen::dotProd(const double *x1, const double *x2)
{
    return(x1[0]*x2[0]+x1[1]*x2[1]+x1[2]*x2[2]);
}

/* !\brief generate the C/A code sequence for a given Satellite Vehicle PRN
 *  \param[in] prn PRN nuber of the Satellite Vehicle
 *  \param[out] ca Caller-allocated integer array of 1023 bytes
 */
void GpsGen::codegen(int *ca, int prn)
{
    int delay[] = {
        5,   6,   7,   8,  17,  18, 139, 140, 141, 251,
        252, 254, 255, 256, 257, 258, 469, 470, 471, 472,
        473, 474, 509, 512, 513, 514, 515, 516, 859, 860,
        861, 862};

    int g1[mCaSeqLen], g2[mCaSeqLen];
    int r1[mNDWrdSbf], r2[mNDWrdSbf];
    int c1, c2;
    int i,j;

    if (prn<1 || prn>32) {
        return;
    }

    for (i=0;i < mNDWrdSbf;i++) {
        r1[i] = r2[i] = -1;
    }

    for (i=0;i < mCaSeqLen;i++) {
        g1[i] = r1[9];
        g2[i] = r2[9];
        c1 = r1[2]*r1[9];
        c2 = r2[1]*r2[2]*r2[5]*r2[7]*r2[8]*r2[9];

        for (j = 9;j > 0;j--) {
            r1[j] = r1[j-1];
            r2[j] = r2[j-1];
        }
        r1[0] = c1;
        r2[0] = c2;
    }

    for (i=0,j=mCaSeqLen-delay[prn-1]; i<mCaSeqLen; i++,j++) {
        ca[i] = (1-g1[i]*g2[j%mCaSeqLen])/2;
    }
}

void GpsGen::date2gps(const GpsGen::datetime_t *t, GpsGen::gpstime_t *g)
{
    int doy[12] = {0,31,59,90,120,151,181,212,243,273,304,334};
    int ye;
    int de;
    int lpdays;

    ye = t->y - 1980;

    // Compute the number of leap days since Jan 5/Jan 6, 1980.
    lpdays = ye/4 + 1;
    if ((ye%4)==0 && t->m<=2)
        lpdays--;

    // Compute the number of days elapsed since Jan 5/Jan 6, 1980.
    de = ye*365 + doy[t->m-1] + t->d + lpdays - 6;

    // Convert time to GPS weeks and seconds.
    g->week = de / 7;
    g->sec = (double)(de%7)*SECONDS_IN_DAY + t->hh*SECONDS_IN_HOUR
            + t->mm*SECONDS_IN_MINUTE + t->sec;

    return;
}

void GpsGen::gps2date(const GpsGen::gpstime_t *g, GpsGen::datetime_t *t)
{
    // Convert Julian day number to calendar date
    int c = (int)(7*g->week + floor(g->sec/86400.0)+2444245.0) + 1537;
    int d = (int)((c-122.1)/365.25);
    int e = 365*d + d/4;
    int f = (int)((c-e)/30.6001);

    t->d = c - e - (int)(30.6001*f);
    t->m = f - 1 - 12*(f/14);
    t->y = d - 4715 - ((7 + t->m)/10);

    t->hh = ((int)(g->sec/3600.0))%24;
    t->mm = ((int)(g->sec/60.0))%60;
    t->sec = g->sec - 60.0*floor(g->sec/60.0);

    return;
}

void GpsGen::xyz2llh(const double *xyz, double *llh)
{
    double a,eps,e,e2;
    double x,y,z;
    double rho2,dz,zdz,nh,slat,n,dz_new;

    a = WGS84_RADIUS;
    e = WGS84_ECCENTRICITY;

    eps = 1.0e-3;
    e2 = e*e;

    if (normVect(xyz)<eps) {
        // Invalid ECEF vector
        llh[0] = 0.0;
        llh[1] = 0.0;
        llh[2] = -a;
        return;
    }

    x = xyz[0];
    y = xyz[1];
    z = xyz[2];

    rho2 = x*x + y*y;
    dz = e2*z;

    while (1) {
        zdz = z + dz;
        nh = sqrt(rho2 + zdz*zdz);
        slat = zdz / nh;
        n = a / sqrt(1.0-e2*slat*slat);
        dz_new = n*e2*slat;

        if (fabs(dz-dz_new) < eps) {
            break;
        }

        dz = dz_new;
    }

    llh[0] = atan2(zdz, sqrt(rho2));
    llh[1] = atan2(y, x);
    llh[2] = nh - n;
}

void GpsGen::llh2xyz(const double *llh, double *xyz)
{
    double n;
    double a;
    double e;
    double e2;
    double clat;
    double slat;
    double clon;
    double slon;
    double d,nph;
    double tmp;

    a = WGS84_RADIUS;
    e = WGS84_ECCENTRICITY;
    e2 = e*e;

    clat = cos(llh[0]);
    slat = sin(llh[0]);
    clon = cos(llh[1]);
    slon = sin(llh[1]);
    d = e*slat;

    n = a/sqrt(1.0-d*d);
    nph = n + llh[2];

    tmp = nph*clat;
    xyz[0] = tmp*clon;
    xyz[1] = tmp*slon;
    xyz[2] = ((1.0-e2)*n + llh[2])*slat;
}

void GpsGen::ltcmat(const double *llh, double t[3][3])
{
    double slat, clat;
    double slon, clon;

    slat = sin(llh[0]);
    clat = cos(llh[0]);
    slon = sin(llh[1]);
    clon = cos(llh[1]);

    t[0][0] = -slat*clon;
    t[0][1] = -slat*slon;
    t[0][2] = clat;
    t[1][0] = -slon;
    t[1][1] = clon;
    t[1][2] = 0.0;
    t[2][0] = clat*clon;
    t[2][1] = clat*slon;
    t[2][2] = slat;
}

void GpsGen::ecef2neu(const double *xyz, double t[3][3], double *neu)
{
    neu[0] = t[0][0]*xyz[0] + t[0][1]*xyz[1] + t[0][2]*xyz[2];
    neu[1] = t[1][0]*xyz[0] + t[1][1]*xyz[1] + t[1][2]*xyz[2];
    neu[2] = t[2][0]*xyz[0] + t[2][1]*xyz[1] + t[2][2]*xyz[2];
}

void GpsGen::neu2azel(double *azel, const double *neu)
{
    double ne;

    azel[0] = atan2(neu[1],neu[0]);
    if (azel[0]<0.0) {
        azel[0] += (2.0 * M_PI);
    }

    ne = sqrt(neu[0]*neu[0] + neu[1]*neu[1]);
    azel[1] = atan2(neu[2], ne);
}

/*! \brief Compute Satellite position, velocity and clock at given time
 *  \param[in] eph Ephemeris data of the satellite
 *  \param[in] g GPS time at which position is to be computed
 *  \param[out] pos Computed position (vector)
 *  \param[out] vel Computed velociy (vector)
 *  \param[clk] clk Computed clock
 */
void GpsGen::satpos(GpsGen::ephem_t eph, GpsGen::gpstime_t g, double *pos, double *vel, double *clk)
{
    // Computing Satellite Velocity using the Broadcast Ephemeris
    // http://www.ngs.noaa.gov/gps-toolbox/bc_velo.htm

    double tk;
    double mk;
    double ek;
    double ekold;
    double ekdot;
    double cek,sek;
    double pk;
    double pkdot;
    double c2pk,s2pk;
    double uk;
    double ukdot;
    double cuk,suk;
    double ok;
    double sok,cok;
    double ik;
    double ikdot;
    double sik,cik;
    double rk;
    double rkdot;
    double xpk,ypk;
    double xpkdot,ypkdot;

    double relativistic, OneMinusecosE, tmp;

    tk = g.sec - eph.toe.sec;

    if(tk>SECONDS_IN_HALF_WEEK) {
        tk -= SECONDS_IN_WEEK;
    } else if (tk<-SECONDS_IN_HALF_WEEK) {
        tk += SECONDS_IN_WEEK;
    }

    mk = eph.m0 + eph.n*tk;
    ek = mk;
    ekold = ek + 1.0;

    OneMinusecosE = 0; // Suppress the uninitialized warning.
    while(fabs(ek-ekold)>1.0E-14) {
        ekold = ek;
        OneMinusecosE = 1.0-eph.ecc*cos(ekold);
        ek = ek + (mk-ekold+eph.ecc*sin(ekold))/OneMinusecosE;
    }

    sek = sin(ek);
    cek = cos(ek);

    ekdot = eph.n/OneMinusecosE;

    relativistic = -4.442807633E-10*eph.ecc*eph.sqrta*sek;

    pk = atan2(eph.sq1e2*sek,cek-eph.ecc) + eph.aop;
    pkdot = eph.sq1e2*ekdot/OneMinusecosE;

    s2pk = sin(2.0*pk);
    c2pk = cos(2.0*pk);

    uk = pk + eph.cus*s2pk + eph.cuc*c2pk;
    suk = sin(uk);
    cuk = cos(uk);
    ukdot = pkdot*(1.0 + 2.0*(eph.cus*c2pk - eph.cuc*s2pk));

    rk = eph.A*OneMinusecosE + eph.crc*c2pk + eph.crs*s2pk;
    rkdot = eph.A*eph.ecc*sek*ekdot + 2.0*pkdot*(eph.crs*c2pk - eph.crc*s2pk);

    ik = eph.inc0 + eph.idot*tk + eph.cic*c2pk + eph.cis*s2pk;
    sik = sin(ik);
    cik = cos(ik);
    ikdot = eph.idot + 2.0*pkdot*(eph.cis*c2pk - eph.cic*s2pk);

    xpk = rk*cuk;
    ypk = rk*suk;
    xpkdot = rkdot*cuk - ypk*ukdot;
    ypkdot = rkdot*suk + xpk*ukdot;

    ok = eph.omg0 + tk*eph.omgkdot - OMEGA_EARTH*eph.toe.sec;
    sok = sin(ok);
    cok = cos(ok);

    pos[0] = xpk*cok - ypk*cik*sok;
    pos[1] = xpk*sok + ypk*cik*cok;
    pos[2] = ypk*sik;

    tmp = ypkdot*cik - ypk*sik*ikdot;

    vel[0] = -eph.omgkdot*pos[1] + xpkdot*cok - tmp*sok;
    vel[1] = eph.omgkdot*pos[0] + xpkdot*sok + tmp*cok;
    vel[2] = ypk*cik*ikdot + ypkdot*sik;

    // Satellite clock correction
    tk = g.sec - eph.toc.sec;

    if(tk>SECONDS_IN_HALF_WEEK) {
        tk -= SECONDS_IN_WEEK;
    } else if(tk<-SECONDS_IN_HALF_WEEK) {
        tk += SECONDS_IN_WEEK;
    }

    clk[0] = eph.af0 + tk*(eph.af1 + tk*eph.af2) + relativistic - eph.tgd;
    clk[1] = eph.af1 + 2.0*tk*eph.af2;
}

/*! \brief Compute Subframe from Ephemeris
 *  \param[in] eph Ephemeris of given SV
 *  \param[out] sbf Array of five sub-frames, 10 long words each
 */
void GpsGen::eph2sbf(const GpsGen::ephem_t eph, const GpsGen::ionoutc_t ionoutc, unsigned long sbf[5][mNDWrdSbf])
{
    unsigned long wn;
    unsigned long toe;
    unsigned long toc;
    unsigned long iode;
    unsigned long iodc;
    long deltan;
    long cuc;
    long cus;
    long cic;
    long cis;
    long crc;
    long crs;
    unsigned long ecc;
    unsigned long sqrta;
    long m0;
    long omg0;
    long inc0;
    long aop;
    long omgdot;
    long idot;
    long af0;
    long af1;
    long af2;
    long tgd;
    int svhlth;
    int codeL2;

    unsigned long ura = 0UL;
    unsigned long dataId = 1UL;
    unsigned long sbf4_page25_svId = 63UL;
    unsigned long sbf5_page25_svId = 51UL;

    unsigned long wna;
    unsigned long toa;

    signed long alpha0,alpha1,alpha2,alpha3;
    signed long beta0,beta1,beta2,beta3;
    signed long A0,A1;
    signed long dtls,dtlsf;
    unsigned long tot,wnt,wnlsf,dn;
    unsigned long sbf4_page18_svId = 56UL;

    // FIXED: This has to be the "transmission" week number, not for the ephemeris reference time
    //wn = (unsigned long)(eph.toe.week%1024);
    wn = 0UL;
    toe = (unsigned long)(eph.toe.sec/16.0);
    toc = (unsigned long)(eph.toc.sec/16.0);
    iode = (unsigned long)(eph.iode);
    iodc = (unsigned long)(eph.iodc);
    deltan = (long)(eph.deltan/POW2_M43/M_PI);
    cuc = (long)(eph.cuc/POW2_M29);
    cus = (long)(eph.cus/POW2_M29);
    cic = (long)(eph.cic/POW2_M29);
    cis = (long)(eph.cis/POW2_M29);
    crc = (long)(eph.crc/POW2_M5);
    crs = (long)(eph.crs/POW2_M5);
    ecc = (unsigned long)(eph.ecc/POW2_M33);
    sqrta = (unsigned long)(eph.sqrta/POW2_M19);
    m0 = (long)(eph.m0/POW2_M31/M_PI);
    omg0 = (long)(eph.omg0/POW2_M31/M_PI);
    inc0 = (long)(eph.inc0/POW2_M31/M_PI);
    aop = (long)(eph.aop/POW2_M31/M_PI);
    omgdot = (long)(eph.omgdot/POW2_M43/M_PI);
    idot = (long)(eph.idot/POW2_M43/M_PI);
    af0 = (long)(eph.af0/POW2_M31);
    af1 = (long)(eph.af1/POW2_M43);
    af2 = (long)(eph.af2/POW2_M55);
    tgd = (long)(eph.tgd/POW2_M31);
    svhlth = (unsigned long)(eph.svhlth);
    codeL2 = (unsigned long)(eph.codeL2);

    wna = (unsigned long)(eph.toe.week%256);
    toa = (unsigned long)(eph.toe.sec/4096.0);

    alpha0 = (signed long)round(ionoutc.alpha0/POW2_M30);
    alpha1 = (signed long)round(ionoutc.alpha1/POW2_M27);
    alpha2 = (signed long)round(ionoutc.alpha2/POW2_M24);
    alpha3 = (signed long)round(ionoutc.alpha3/POW2_M24);
    beta0 = (signed long)round(ionoutc.beta0/2048.0);
    beta1 = (signed long)round(ionoutc.beta1/16384.0);
    beta2 = (signed long)round(ionoutc.beta2/65536.0);
    beta3 = (signed long)round(ionoutc.beta3/65536.0);
    A0 = (signed long)round(ionoutc.A0/POW2_M30);
    A1 = (signed long)round(ionoutc.A1/POW2_M50);
    dtls = (signed long)(ionoutc.dtls);
    tot = (unsigned long)(ionoutc.tot/4096);
    wnt = (unsigned long)(ionoutc.wnt%256);
    // TO DO: Specify scheduled leap seconds in command options
    // 2016/12/31 (Sat) -> WNlsf = 1929, DN = 7 (http://navigationservices.agi.com/GNSSWeb/)
    // Days are counted from 1 to 7 (Sunday is 1).
    wnlsf = 1929%256;
    dn = 7;
    dtlsf = 18;

    // Subframe 1
    sbf[0][0] = 0x8B0000UL<<6;
    sbf[0][1] = 0x1UL<<8;
    sbf[0][2] = ((wn&0x3FFUL)<<20) | ((codeL2&0x3UL)<<18) | ((ura&0xFUL)<<14) | ((svhlth&0x3FUL)<<8) | (((iodc>>8)&0x3UL)<<6);
    sbf[0][3] = 0UL;
    sbf[0][4] = 0UL;
    sbf[0][5] = 0UL;
    sbf[0][6] = (tgd&0xFFUL)<<6;
    sbf[0][7] = ((iodc&0xFFUL)<<22) | ((toc&0xFFFFUL)<<6);
    sbf[0][8] = ((af2&0xFFUL)<<22) | ((af1&0xFFFFUL)<<6);
    sbf[0][9] = (af0&0x3FFFFFUL)<<8;

    // Subframe 2
    sbf[1][0] = 0x8B0000UL<<6;
    sbf[1][1] = 0x2UL<<8;
    sbf[1][2] = ((iode&0xFFUL)<<22) | ((crs&0xFFFFUL)<<6);
    sbf[1][3] = ((deltan&0xFFFFUL)<<14) | (((m0>>24)&0xFFUL)<<6);
    sbf[1][4] = (m0&0xFFFFFFUL)<<6;
    sbf[1][5] = ((cuc&0xFFFFUL)<<14) | (((ecc>>24)&0xFFUL)<<6);
    sbf[1][6] = (ecc&0xFFFFFFUL)<<6;
    sbf[1][7] = ((cus&0xFFFFUL)<<14) | (((sqrta>>24)&0xFFUL)<<6);
    sbf[1][8] = (sqrta&0xFFFFFFUL)<<6;
    sbf[1][9] = (toe&0xFFFFUL)<<14;

    // Subframe 3
    sbf[2][0] = 0x8B0000UL<<6;
    sbf[2][1] = 0x3UL<<8;
    sbf[2][2] = ((cic&0xFFFFUL)<<14) | (((omg0>>24)&0xFFUL)<<6);
    sbf[2][3] = (omg0&0xFFFFFFUL)<<6;
    sbf[2][4] = ((cis&0xFFFFUL)<<14) | (((inc0>>24)&0xFFUL)<<6);
    sbf[2][5] = (inc0&0xFFFFFFUL)<<6;
    sbf[2][6] = ((crc&0xFFFFUL)<<14) | (((aop>>24)&0xFFUL)<<6);
    sbf[2][7] = (aop&0xFFFFFFUL)<<6;
    sbf[2][8] = (omgdot&0xFFFFFFUL)<<6;
    sbf[2][9] = ((iode&0xFFUL)<<22) | ((idot&0x3FFFUL)<<8);

    if (ionoutc.vflg==1) {
        // Subframe 4, page 18
        sbf[3][0] = 0x8B0000UL<<6;
        sbf[3][1] = 0x4UL<<8;
        sbf[3][2] = (dataId<<28) | (sbf4_page18_svId<<22) | ((alpha0&0xFFUL)<<14) | ((alpha1&0xFFUL)<<6);
        sbf[3][3] = ((alpha2&0xFFUL)<<22) | ((alpha3&0xFFUL)<<14) | ((beta0&0xFFUL)<<6);
        sbf[3][4] = ((beta1&0xFFUL)<<22) | ((beta2&0xFFUL)<<14) | ((beta3&0xFFUL)<<6);
        sbf[3][5] = (A1&0xFFFFFFUL)<<6;
        sbf[3][6] = ((A0>>8)&0xFFFFFFUL)<<6;
        sbf[3][7] = ((A0&0xFFUL)<<22) | ((tot&0xFFUL)<<14) | ((wnt&0xFFUL)<<6);
        sbf[3][8] = ((dtls&0xFFUL)<<22) | ((wnlsf&0xFFUL)<<14) | ((dn&0xFFUL)<<6);
        sbf[3][9] = (dtlsf&0xFFUL)<<22;

    } else {
        // Subframe 4, page 25
        sbf[3][0] = 0x8B0000UL<<6;
        sbf[3][1] = 0x4UL<<8;
        sbf[3][2] = (dataId<<28) | (sbf4_page25_svId<<22);
        sbf[3][3] = 0UL;
        sbf[3][4] = 0UL;
        sbf[3][5] = 0UL;
        sbf[3][6] = 0UL;
        sbf[3][7] = 0UL;
        sbf[3][8] = 0UL;
        sbf[3][9] = 0UL;
    }

    // Subframe 5, page 25
    sbf[4][0] = 0x8B0000UL<<6;
    sbf[4][1] = 0x5UL<<8;
    sbf[4][2] = (dataId<<28) | (sbf5_page25_svId<<22) | ((toa&0xFFUL)<<14) | ((wna&0xFFUL)<<6);
    sbf[4][3] = 0UL;
    sbf[4][4] = 0UL;
    sbf[4][5] = 0UL;
    sbf[4][6] = 0UL;
    sbf[4][7] = 0UL;
    sbf[4][8] = 0UL;
    sbf[4][9] = 0UL;
}

/*! \brief Count number of bits set to 1
 *  \param[in] v long word in whihc bits are counted
 *  \returns Count of bits set to 1
 */
unsigned long GpsGen::countBits(unsigned long v)
{
    unsigned long c;
    const int S[] = {1, 2, 4, 8, 16};
    const unsigned long B[] = {
        0x55555555, 0x33333333, 0x0F0F0F0F, 0x00FF00FF, 0x0000FFFF};

    c = v;
    c = ((c >> S[0]) & B[0]) + (c & B[0]);
    c = ((c >> S[1]) & B[1]) + (c & B[1]);
    c = ((c >> S[2]) & B[2]) + (c & B[2]);
    c = ((c >> S[3]) & B[3]) + (c & B[3]);
    c = ((c >> S[4]) & B[4]) + (c & B[4]);

    return(c);
}

/*! \brief Compute the Checksum for one given word of a subframe
 *  \param[in] source The input data
 *  \param[in] nib Does this word contain non-information-bearing bits?
 *  \returns Computed Checksum
 */
unsigned long GpsGen::computeChecksum(unsigned long source, int nib)
{
    /*
        Bits 31 to 30 = 2 LSBs of the previous transmitted word, D29* and D30*
        Bits 29 to  6 = Source data bits, d1, d2, ..., d24
        Bits  5 to  0 = Empty parity bits
        */

    /*
        Bits 31 to 30 = 2 LSBs of the previous transmitted word, D29* and D30*
        Bits 29 to  6 = Data bits transmitted by the SV, D1, D2, ..., D24
        Bits  5 to  0 = Computed parity bits, D25, D26, ..., D30
        */

    /*
                          1            2           3
        bit    12 3456 7890 1234 5678 9012 3456 7890
        ---    -------------------------------------
        D25    11 1011 0001 1111 0011 0100 1000 0000
        D26    01 1101 1000 1111 1001 1010 0100 0000
        D27    10 1110 1100 0111 1100 1101 0000 0000
        D28    01 0111 0110 0011 1110 0110 1000 0000
        D29    10 1011 1011 0001 1111 0011 0100 0000
        D30    00 1011 0111 1010 1000 1001 1100 0000
        */

    unsigned long bmask[6] = {
        0x3B1F3480UL, 0x1D8F9A40UL, 0x2EC7CD00UL,
        0x1763E680UL, 0x2BB1F340UL, 0x0B7A89C0UL };

    unsigned long D;
    unsigned long d = source & 0x3FFFFFC0UL;
    unsigned long D29 = (source>>31)&0x1UL;
    unsigned long D30 = (source>>30)&0x1UL;

    if (nib) { // Non-information bearing bits for word 2 and 10
        /*
            Solve bits 23 and 24 to presearve parity check
            with zeros in bits 29 and 30.
            */

        if ((D30 + countBits(bmask[4] & d)) % 2) {
            d ^= (0x1UL<<6);
        }

        if ((D29 + countBits(bmask[5] & d)) % 2) {
            d ^= (0x1UL<<7);
        }
    }

    D = d;
    if (D30) {
        D ^= 0x3FFFFFC0UL;
    }

    D |= ((D29 + countBits(bmask[0] & d)) % 2) << 5;
    D |= ((D30 + countBits(bmask[1] & d)) % 2) << 4;
    D |= ((D29 + countBits(bmask[2] & d)) % 2) << 3;
    D |= ((D30 + countBits(bmask[3] & d)) % 2) << 2;
    D |= ((D30 + countBits(bmask[4] & d)) % 2) << 1;
    D |= ((D29 + countBits(bmask[5] & d)) % 2);

    D &= 0x3FFFFFFFUL;
    //D |= (source & 0xC0000000UL); // Add D29* and D30* from source data bits

    return(D);
}

/*! \brief Replace all 'E' exponential designators to 'D'
 *  \param str String in which all occurrences of 'E' are replaced with *  'D'
 *  \param len Length of input string in bytes
 *  \returns Number of characters replaced
 */
int GpsGen::replaceExpDesignator(char *str, int len)
{
    int i,n=0;

    for (i=0; i<len; i++) {
        if (str[i]=='D') {
            n++;
            str[i] = 'E';
        }
    }

    return(n);
}

double GpsGen::subGpsTime(GpsGen::gpstime_t g1, GpsGen::gpstime_t g0)
{
    double dt;

    dt = g1.sec - g0.sec;
    dt += (double)(g1.week - g0.week) * SECONDS_IN_WEEK;

    return(dt);
}

GpsGen::gpstime_t GpsGen::incGpsTime(GpsGen::gpstime_t g0, double dt)
{
    gpstime_t g1;

    g1.week = g0.week;
    g1.sec = g0.sec + dt;

    g1.sec = round(g1.sec*1000.0)/1000.0; // Avoid rounding error

    while (g1.sec>=SECONDS_IN_WEEK) {
        g1.sec -= SECONDS_IN_WEEK;
        g1.week++;
    }

    while (g1.sec<0.0) {
        g1.sec += SECONDS_IN_WEEK;
        g1.week--;
    }

    return(g1);
}

/*! \brief Read Ephemersi data from the RINEX Navigation file
 *  \param[out] eph Array of Output SV ephemeris data
 *  \param[in] fname File name of the RINEX file
 *  \returns Number of sets of ephemerides in the file
 */
int GpsGen::readRinexNavAll(GpsGen::ephem_t eph[][mMaxSat], GpsGen::ionoutc_t *ionoutc, const char *fname)
{
    const int maxChar = 100;

    int ieph;
    int sv;
    char str[maxChar];
    char tmp[20];

    datetime_t t;
    gpstime_t g;
    gpstime_t g0;
    double dt;

    int flags = 0x0;

    setlocale(LC_NUMERIC, "C");

    // QFile so that the resource system can be used.
    QFile qf(fname);

    if (!qf.open(QIODevice::ReadOnly)) {
        return -1;
    }

    // Clear valid flag
    for (ieph = 0;ieph < mEphemArraySize; ieph++) {
        for (sv = 0;sv < mMaxSat;sv++) {
            eph[ieph][sv].vflg = 0;
        }
    }

    // Read header lines
    while (1) {
        if (qf.readLine(str, maxChar) <= 0) {
            break;
        }

        if (strncmp(str+60, "END OF HEADER", 13)==0) {
            break;
        } else if (strncmp(str+60, "ION ALPHA", 9)==0) {
            strncpy(tmp, str+2, 12);
            tmp[12] = 0;
            replaceExpDesignator(tmp, 12);
            ionoutc->alpha0 = atof(tmp);

            strncpy(tmp, str+14, 12);
            tmp[12] = 0;
            replaceExpDesignator(tmp, 12);
            ionoutc->alpha1 = atof(tmp);

            strncpy(tmp, str+26, 12);
            tmp[12] = 0;
            replaceExpDesignator(tmp, 12);
            ionoutc->alpha2 = atof(tmp);

            strncpy(tmp, str+38, 12);
            tmp[12] = 0;
            replaceExpDesignator(tmp, 12);
            ionoutc->alpha3 = atof(tmp);

            flags |= 0x1;
        } else if (strncmp(str+60, "ION BETA", 8)==0) {
            strncpy(tmp, str+2, 12);
            tmp[12] = 0;
            replaceExpDesignator(tmp, 12);
            ionoutc->beta0 = atof(tmp);

            strncpy(tmp, str+14, 12);
            tmp[12] = 0;
            replaceExpDesignator(tmp, 12);
            ionoutc->beta1 = atof(tmp);

            strncpy(tmp, str+26, 12);
            tmp[12] = 0;
            replaceExpDesignator(tmp, 12);
            ionoutc->beta2 = atof(tmp);

            strncpy(tmp, str+38, 12);
            tmp[12] = 0;
            replaceExpDesignator(tmp, 12);
            ionoutc->beta3 = atof(tmp);

            flags |= 0x1<<1;
        } else if (strncmp(str+60, "DELTA-UTC", 9)==0) {
            strncpy(tmp, str+3, 19);
            tmp[19] = 0;
            replaceExpDesignator(tmp, 19);
            ionoutc->A0 = atof(tmp);

            strncpy(tmp, str+22, 19);
            tmp[19] = 0;
            replaceExpDesignator(tmp, 19);
            ionoutc->A1 = atof(tmp);

            strncpy(tmp, str+41, 9);
            tmp[9] = 0;
            ionoutc->tot = atoi(tmp);

            strncpy(tmp, str+50, 9);
            tmp[9] = 0;
            ionoutc->wnt = atoi(tmp);

            if (ionoutc->tot%4096==0)
                flags |= 0x1<<2;
        } else if (strncmp(str+60, "LEAP SECONDS", 12)==0) {
            strncpy(tmp, str, 6);
            tmp[6] = 0;
            ionoutc->dtls = atoi(tmp);

            flags |= 0x1<<3;
        }
    }

    ionoutc->vflg = 0;
    if (flags==0xF) { // Read all Iono/UTC lines
        ionoutc->vflg = 1;
    }

    // Read ephemeris blocks
    g0.week = -1;
    ieph = 0;

    while (1) {
        if (qf.readLine(str, maxChar) <= 0) {
            break;
        }

        // PRN
        strncpy(tmp, str, 2);
        tmp[2] = 0;
        sv = atoi(tmp)-1;

        // EPOCH
        strncpy(tmp, str+3, 2);
        tmp[2] = 0;
        t.y = atoi(tmp) + 2000;

        strncpy(tmp, str+6, 2);
        tmp[2] = 0;
        t.m = atoi(tmp);

        strncpy(tmp, str+9, 2);
        tmp[2] = 0;
        t.d = atoi(tmp);

        strncpy(tmp, str+12, 2);
        tmp[2] = 0;
        t.hh = atoi(tmp);

        strncpy(tmp, str+15, 2);
        tmp[2] = 0;
        t.mm = atoi(tmp);

        strncpy(tmp, str+18, 4);
        tmp[2] = 0;
        t.sec = atof(tmp);

        date2gps(&t, &g);

        if (g0.week==-1) {
            g0 = g;
        }

        // Check current time of clock
        dt = subGpsTime(g, g0);

        if (dt>SECONDS_IN_HOUR) {
            g0 = g;
            ieph++; // a new set of ephemerides

            if (ieph>=mEphemArraySize) {
                break;
            }
        }

        // Date and time
        eph[ieph][sv].t = t;

        // SV CLK
        eph[ieph][sv].toc = g;

        strncpy(tmp, str+22, 19);
        tmp[19] = 0;
        replaceExpDesignator(tmp, 19); // tmp[15]='E';
        eph[ieph][sv].af0 = atof(tmp);

        strncpy(tmp, str+41, 19);
        tmp[19] = 0;
        replaceExpDesignator(tmp, 19);
        eph[ieph][sv].af1 = atof(tmp);

        strncpy(tmp, str+60, 19);
        tmp[19] = 0;
        replaceExpDesignator(tmp, 19);
        eph[ieph][sv].af2 = atof(tmp);

        // BROADCAST ORBIT - 1
        if (qf.readLine(str, maxChar) <= 0) {
            break;
        }

        strncpy(tmp, str+3, 19);
        tmp[19] = 0;
        replaceExpDesignator(tmp, 19);
        eph[ieph][sv].iode = (int)atof(tmp);

        strncpy(tmp, str+22, 19);
        tmp[19] = 0;
        replaceExpDesignator(tmp, 19);
        eph[ieph][sv].crs = atof(tmp);

        strncpy(tmp, str+41, 19);
        tmp[19] = 0;
        replaceExpDesignator(tmp, 19);
        eph[ieph][sv].deltan = atof(tmp);

        strncpy(tmp, str+60, 19);
        tmp[19] = 0;
        replaceExpDesignator(tmp, 19);
        eph[ieph][sv].m0 = atof(tmp);

        // BROADCAST ORBIT - 2
        if (qf.readLine(str, maxChar) <= 0) {
            break;
        }

        strncpy(tmp, str+3, 19);
        tmp[19] = 0;
        replaceExpDesignator(tmp, 19);
        eph[ieph][sv].cuc = atof(tmp);

        strncpy(tmp, str+22, 19);
        tmp[19] = 0;
        replaceExpDesignator(tmp, 19);
        eph[ieph][sv].ecc = atof(tmp);

        strncpy(tmp, str+41, 19);
        tmp[19] = 0;
        replaceExpDesignator(tmp, 19);
        eph[ieph][sv].cus = atof(tmp);

        strncpy(tmp, str+60, 19);
        tmp[19] = 0;
        replaceExpDesignator(tmp, 19);
        eph[ieph][sv].sqrta = atof(tmp);

        // BROADCAST ORBIT - 3
        if (qf.readLine(str, maxChar) <= 0) {
            break;
        }

        strncpy(tmp, str+3, 19);
        tmp[19] = 0;
        replaceExpDesignator(tmp, 19);
        eph[ieph][sv].toe.sec = atof(tmp);

        strncpy(tmp, str+22, 19);
        tmp[19] = 0;
        replaceExpDesignator(tmp, 19);
        eph[ieph][sv].cic = atof(tmp);

        strncpy(tmp, str+41, 19);
        tmp[19] = 0;
        replaceExpDesignator(tmp, 19);
        eph[ieph][sv].omg0 = atof(tmp);

        strncpy(tmp, str+60, 19);
        tmp[19] = 0;
        replaceExpDesignator(tmp, 19);
        eph[ieph][sv].cis = atof(tmp);

        // BROADCAST ORBIT - 4
        if (qf.readLine(str, maxChar) <= 0) {
            break;
        }

        strncpy(tmp, str+3, 19);
        tmp[19] = 0;
        replaceExpDesignator(tmp, 19);
        eph[ieph][sv].inc0 = atof(tmp);

        strncpy(tmp, str+22, 19);
        tmp[19] = 0;
        replaceExpDesignator(tmp, 19);
        eph[ieph][sv].crc = atof(tmp);

        strncpy(tmp, str+41, 19);
        tmp[19] = 0;
        replaceExpDesignator(tmp, 19);
        eph[ieph][sv].aop = atof(tmp);

        strncpy(tmp, str+60, 19);
        tmp[19] = 0;
        replaceExpDesignator(tmp, 19);
        eph[ieph][sv].omgdot = atof(tmp);

        // BROADCAST ORBIT - 5
        if (qf.readLine(str, maxChar) <= 0) {
            break;
        }

        strncpy(tmp, str+3, 19);
        tmp[19] = 0;
        replaceExpDesignator(tmp, 19);
        eph[ieph][sv].idot = atof(tmp);

        strncpy(tmp, str+22, 19);
        tmp[19] = 0;
        replaceExpDesignator(tmp, 19);
        eph[ieph][sv].codeL2 = (int)atof(tmp);

        strncpy(tmp, str+41, 19);
        tmp[19] = 0;
        replaceExpDesignator(tmp, 19);
        eph[ieph][sv].toe.week = (int)atof(tmp);

        // BROADCAST ORBIT - 6
        if (qf.readLine(str, maxChar) <= 0) {
            break;
        }

        strncpy(tmp, str+22, 19);
        tmp[19] = 0;
        replaceExpDesignator(tmp, 19);
        eph[ieph][sv].svhlth = (int)atof(tmp);
        if ((eph[ieph][sv].svhlth>0) && (eph[ieph][sv].svhlth<32)) {
            eph[ieph][sv].svhlth += 32; // Set MSB to 1
        }

        strncpy(tmp, str+41, 19);
        tmp[19] = 0;
        replaceExpDesignator(tmp, 19);
        eph[ieph][sv].tgd = atof(tmp);

        strncpy(tmp, str+60, 19);
        tmp[19] = 0;
        replaceExpDesignator(tmp, 19);
        eph[ieph][sv].iodc = (int)atof(tmp);

        // BROADCAST ORBIT - 7
        if (qf.readLine(str, maxChar) <= 0) {
            break;
        }

        // Set valid flag
        eph[ieph][sv].vflg = 1;

        // Update the working variables
        eph[ieph][sv].A = eph[ieph][sv].sqrta * eph[ieph][sv].sqrta;
        eph[ieph][sv].n = sqrt(GM_EARTH/(eph[ieph][sv].A*eph[ieph][sv].A*eph[ieph][sv].A)) + eph[ieph][sv].deltan;
        eph[ieph][sv].sq1e2 = sqrt(1.0 - eph[ieph][sv].ecc*eph[ieph][sv].ecc);
        eph[ieph][sv].omgkdot = eph[ieph][sv].omgdot - OMEGA_EARTH;
    }

    qf.close();

    if (g0.week>=0) {
        ieph += 1; // Number of sets of ephemerides
    }

    return(ieph);
}

double GpsGen::ionosphericDelay(const GpsGen::ionoutc_t *ionoutc, GpsGen::gpstime_t g, double *llh, double *azel)
{
    double iono_delay = 0.0;
    double E,phi_u,lam_u,F;

    if (!ionoutc->enable) {
        return (0.0); // No ionospheric delay
    }

    E = azel[1]/M_PI;
    phi_u = llh[0]/M_PI;
    lam_u = llh[1]/M_PI;

    // Obliquity factor
    F = 1.0 + 16.0*pow((0.53 - E),3.0);

    if (!ionoutc->vflg) {
        iono_delay = F*5.0e-9*SPEED_OF_LIGHT;
    } else {
        double t,psi,phi_i,lam_i,phi_m,phi_m2,phi_m3;
        double AMP,PER,X,X2,X4;

        // Earth's central angle between the user position and the earth projection of
        // ionospheric intersection point (semi-circles)
        psi = 0.0137/(E + 0.11) - 0.022;

        // Geodetic latitude of the earth projection of the ionospheric intersection point
        // (semi-circles)
        phi_i = phi_u + psi*cos(azel[0]);
        if(phi_i>0.416) {
            phi_i = 0.416;
        } else if(phi_i<-0.416) {
            phi_i = -0.416;
        }

        // Geodetic longitude of the earth projection of the ionospheric intersection point
        // (semi-circles)
        lam_i = lam_u + psi*sin(azel[0])/cos(phi_i*M_PI);

        // Geomagnetic latitude of the earth projection of the ionospheric intersection
        // point (mean ionospheric height assumed 350 km) (semi-circles)
        phi_m = phi_i + 0.064*cos((lam_i - 1.617)*M_PI);
        phi_m2 = phi_m*phi_m;
        phi_m3 = phi_m2*phi_m;

        AMP = ionoutc->alpha0 + ionoutc->alpha1*phi_m
                + ionoutc->alpha2*phi_m2 + ionoutc->alpha3*phi_m3;

        if (AMP<0.0) {
            AMP = 0.0;
        }

        PER = ionoutc->beta0 + ionoutc->beta1*phi_m
                + ionoutc->beta2*phi_m2 + ionoutc->beta3*phi_m3;

        if (PER<72000.0) {
            PER = 72000.0;
        }

        // Local time (sec)
        t = SECONDS_IN_DAY/2.0*lam_i + g.sec;
        while(t>=SECONDS_IN_DAY) {
            t -= SECONDS_IN_DAY;
        }
        while(t<0) {
            t += SECONDS_IN_DAY;
        }

        // Phase (radians)
        X = 2.0*M_PI*(t - 50400.0)/PER;

        if(fabs(X)<1.57) {
            X2 = X*X;
            X4 = X2*X2;
            iono_delay = F*(5.0e-9 + AMP*(1.0 - X2/2.0 + X4/24.0))*SPEED_OF_LIGHT;
        } else {
            iono_delay = F*5.0e-9*SPEED_OF_LIGHT;
        }
    }

    return (iono_delay);
}

/*! \brief Compute range between a satellite and the receiver
 *  \param[out] rho The computed range
 *  \param[in] eph Ephemeris data of the satellite
 *  \param[in] g GPS time at time of receiving the signal
 *  \param[in] xyz position of the receiver
 */
void GpsGen::computeRange(GpsGen::range_t *rho, GpsGen::ephem_t eph, GpsGen::ionoutc_t *ionoutc, GpsGen::gpstime_t g, double xyz[])
{
    double pos[3],vel[3],clk[2];
    double los[3];
    double tau;
    double range,rate;
    double xrot,yrot;

    double llh[3],neu[3];
    double tmat[3][3];

    // SV position at time of the pseudorange observation.
    satpos(eph, g, pos, vel, clk);

    // Receiver to satellite vector and light-time.
    subVect(los, pos, xyz);
    tau = normVect(los)/SPEED_OF_LIGHT;

    // Extrapolate the satellite position backwards to the transmission time.
    pos[0] -= vel[0]*tau;
    pos[1] -= vel[1]*tau;
    pos[2] -= vel[2]*tau;

    // Earth rotation correction. The change in velocity can be neglected.
    xrot = pos[0] + pos[1]*OMEGA_EARTH*tau;
    yrot = pos[1] - pos[0]*OMEGA_EARTH*tau;
    pos[0] = xrot;
    pos[1] = yrot;

    // New observer to satellite vector and satellite range.
    subVect(los, pos, xyz);
    range = normVect(los);
    rho->d = range;

    // Pseudorange.
    rho->range = range - SPEED_OF_LIGHT*clk[0];

    // Relative velocity of SV and receiver.
    rate = dotProd(vel, los)/range;

    // Pseudorange rate.
    rho->rate = rate; // - SPEED_OF_LIGHT*clk[1];

    // Time of application.
    rho->g = g;

    // Azimuth and elevation angles.
    xyz2llh(xyz, llh);
    ltcmat(llh, tmat);
    ecef2neu(los, tmat, neu);
    neu2azel(rho->azel, neu);

    // Add ionospheric delay
    rho->iono_delay = ionosphericDelay(ionoutc, g, llh, rho->azel);
    rho->range += rho->iono_delay;
}

/*! \brief Compute the code phase for a given channel (satellite)
 *  \param chan Channel on which we operate (is updated)
 *  \param[in] rho1 Current range, after \a dt has expired
 *  \param[in dt delta-t (time difference) in seconds
 */
void GpsGen::computeCodePhase(GpsGen::channel_t *chan, GpsGen::range_t rho1, double dt)
{
    double ms;
    int ims;
    double rhorate;

    // Pseudorange rate.
    rhorate = (rho1.range - chan->rho0.range)/dt;

    // Carrier and code frequency.
    chan->f_carr = -rhorate/LAMBDA_L1;
    chan->f_code = CODE_FREQ + chan->f_carr*CARR_TO_CODE;

    // Initial code phase and data bit counters.
    ms = ((subGpsTime(chan->rho0.g,chan->g0)+6.0) - chan->rho0.range/SPEED_OF_LIGHT)*1000.0;

    ims = (int)ms;
    chan->code_phase = (ms-(double)ims)*mCaSeqLen; // in chip

    chan->iword = ims/600; // 1 word = 30 bits = 600 ms
    ims -= chan->iword*600;

    chan->ibit = ims/20; // 1 bit = 20 code = 20 ms
    ims -= chan->ibit*20;

    chan->icode = ims; // 1 code = 1 ms

    chan->codeCA = chan->ca[(int)chan->code_phase]*2-1;
    chan->dataBit = (int)((chan->dwrd[chan->iword]>>(29-chan->ibit)) & 0x1UL)*2-1;

    // Save current pseudorange
    chan->rho0 = rho1;
}

int GpsGen::generateNavMsg(GpsGen::gpstime_t g, GpsGen::channel_t *chan, int init)
{
    int iwrd,isbf;
    gpstime_t g0;
    unsigned long wn,tow;
    unsigned sbfwrd;
    unsigned long prevwrd;
    int nib;

    g0.week = g.week;
    g0.sec = (double)(((unsigned long)(g.sec+0.5))/30UL) * 30.0; // Align with the full frame length = 30 sec
    chan->g0 = g0; // Data bit reference time

    wn = (unsigned long)(g0.week%1024);
    tow = ((unsigned long)g0.sec)/6UL;

    if (init==1) { // Initialize subframe 5
        prevwrd = 0UL;

        for (iwrd=0; iwrd<mNDWrdSbf; iwrd++) {
            sbfwrd = chan->sbf[4][iwrd];

            // Add TOW-count message into HOW
            if (iwrd==1) {
                sbfwrd |= ((tow&0x1FFFFUL)<<13);
            }

            // Compute checksum
            sbfwrd |= (prevwrd<<30) & 0xC0000000UL; // 2 LSBs of the previous transmitted word
            nib = ((iwrd==1)||(iwrd==9))?1:0; // Non-information bearing bits for word 2 and 10
            chan->dwrd[iwrd] = computeChecksum(sbfwrd, nib);

            prevwrd = chan->dwrd[iwrd];
        }
    } else { // Save subframe 5
        for (iwrd=0; iwrd<mNDWrdSbf; iwrd++) {
            chan->dwrd[iwrd] = chan->dwrd[mNDWrdSbf * mNSbf + iwrd];
            prevwrd = chan->dwrd[iwrd];
        }
    }

    for (isbf=0; isbf<mNSbf; isbf++) {
        tow++;

        for (iwrd=0; iwrd<mNDWrdSbf; iwrd++) {
            sbfwrd = chan->sbf[isbf][iwrd];

            // Add transmission week number to Subframe 1
            if ((isbf==0)&&(iwrd==2)) {
                sbfwrd |= (wn&0x3FFUL)<<20;
            }

            // Add TOW-count message into HOW
            if (iwrd==1) {
                sbfwrd |= ((tow&0x1FFFFUL)<<13);
            }

            // Compute checksum
            sbfwrd |= (prevwrd<<30) & 0xC0000000UL; // 2 LSBs of the previous transmitted word
            nib = ((iwrd==1)||(iwrd==9))?1:0; // Non-information bearing bits for word 2 and 10
            chan->dwrd[(isbf+1)*mNDWrdSbf+iwrd] = computeChecksum(sbfwrd, nib);

            prevwrd = chan->dwrd[(isbf+1)*mNDWrdSbf+iwrd];
        }
    }

    return(1);
}

int GpsGen::checkSatVisibility(GpsGen::ephem_t eph, GpsGen::gpstime_t g, double *xyz, double elvMask, double *azel)
{
    double llh[3],neu[3];
    double pos[3],vel[3],clk[3],los[3];
    double tmat[3][3];

    if (eph.vflg != 1) {
        return (-1); // Invalid
    }

    xyz2llh(xyz,llh);
    ltcmat(llh, tmat);

    satpos(eph, g, pos, vel, clk);
    subVect(los, pos, xyz);
    ecef2neu(los, tmat, neu);
    neu2azel(azel, neu);

    if (azel[1]*R2D > elvMask) {
        return (1); // Visible
    } else {
        return (0); // Invisible
    }
}

int GpsGen::allocateChannel(GpsGen::channel_t *chan, GpsGen::ephem_t *eph, GpsGen::ionoutc_t ionoutc,
                            GpsGen::gpstime_t grx, double *xyz, double elvMask)
{
    (void)elvMask; // TODO?

    int nsat=0;
    int i,sv;
    double azel[2];

    range_t rho;
    double ref[3]={0.0};
    double r_ref,r_xyz;
    double phase_ini;

    for (sv=0;sv < mMaxSat;sv++) {
        if(checkSatVisibility(eph[sv], grx, xyz, 0.0, azel)==1) {
            nsat++; // Number of visible satellites

            if (mAllocatedSat[sv]==-1) { // Visible but not allocated
                // Allocated new satellite
                for (i=0; i<mMaxChan; i++) {
                    if (chan[i].prn==0) {
                        // Initialize channel
                        chan[i].prn = sv+1;
                        chan[i].azel[0] = azel[0];
                        chan[i].azel[1] = azel[1];

                        // C/A code generation
                        codegen(chan[i].ca, chan[i].prn);

                        // Generate subframe
                        eph2sbf(eph[sv], ionoutc, chan[i].sbf);

                        // Generate navigation message
                        generateNavMsg(grx, &chan[i], 1);

                        // Initialize pseudorange
                        computeRange(&rho, eph[sv], &ionoutc, grx, xyz);
                        chan[i].rho0 = rho;

                        // Initialize carrier phase
                        r_xyz = rho.range;

                        computeRange(&rho, eph[sv], &ionoutc, grx, ref);
                        r_ref = rho.range;

                        phase_ini = (2.0*r_ref - r_xyz)/LAMBDA_L1;
                        chan[i].carr_phase = phase_ini - floor(phase_ini);

                        // Done.
                        break;
                    }
                }

                // Set satellite allocation channel
                if (i < mMaxChan) {
                    mAllocatedSat[sv] = i;
                }
            }
        } else if (mAllocatedSat[sv]>=0) { // Not visible but allocated
            // Clear channel
            chan[mAllocatedSat[sv]].prn = 0;

            // Clear satellite allocation flag
            mAllocatedSat[sv] = -1;
        }
    }

    return(nsat);
}
