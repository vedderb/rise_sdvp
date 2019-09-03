/*
    Copyright 2018 Benjamin Vedder	benjamin@vedder.se

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "limesdr.h"
#include <lime/LimeSuite.h>
#include <QFile>

LimeSDR::LimeSDR(QObject *parent) : QThread(parent)
{
    mStop = false;
}

void LimeSDR::setPos(double lat, double lon, double height, double speed)
{
    double llh[3];
    llh[0] = lat;
    llh[1] = lon;
    llh[2] = height;
    mGps.setPos(llh, speed);
}

void LimeSDR::setPosBase(double lat, double lon, double height)
{
    mBaseLat = lat;
    mBaseLon = lon;
    mBaseHeight = height;
}

void LimeSDR::setBaseEnabled(bool en)
{
    mBaseEn = en;
}

void LimeSDR::run()
{
    mStop = false;

    double gain = 1.0;
    double gainBase = 1.0;
    int32_t antenna = LMS_PATH_TX1;
    int32_t antennaBase = LMS_PATH_TX1;
    int32_t channel = 0;
    int32_t channelBase = 1;
    int32_t index = 0;
    double sampleRate = 1.5e6;
    double frequency = 1575.42e6;
    double bandwidth = 100e6;

    mGps.allocateBuffers(sampleRate);
    mGps.resetState();

    double llh[3] = {57.71495867, 12.89134921, 219.0};
    mGps.setPos(llh, 0.0);
    if (!mGps.readNavigationFile("://res/brdc0990.18n")) {
        myPrint("Could not read navigation file\n");
        return;
    }

    if (mBaseEn) {
        mGpsBase.allocateBuffers(sampleRate);
        mGpsBase.resetState();

        double llh2[3] = {mBaseLat, mBaseLon, mBaseHeight};
        mGpsBase.setPos(llh2, 0.0);
        if (!mGpsBase.readNavigationFile("://res/brdc0990.18n")) {
            myPrint("Could not read navigation file for base\n");
            return;
        }
    }

    int device_count = LMS_GetDeviceList(NULL);
    if (device_count < 1) {
        myPrint("No device found\n");
        return;
    }

    lms_info_str_t *device_list = new lms_info_str_t[device_count];
    device_count = LMS_GetDeviceList(device_list);

    for (int i = 0;i < device_count;i++) {
        myPrint("device[%d/%d]=%s\n", i + 1, device_count, device_list[i]);
    }

    lms_device_t *device = NULL;

    if (LMS_Open(&device, device_list[index], NULL)) {
        myPrint("Could not open device.\n");
        delete[] device_list;
        return;
    }

    delete[] device_list;

    int lmsReset = LMS_Reset(device);
    if (lmsReset) {
        myPrint("ERROR lmsReset %d(%s)\n", lmsReset, LMS_GetLastErrorMessage());
    }

    int lmsInit = LMS_Init(device);
    if (lmsInit){
        myPrint("ERROR lmsInit %d(%s)\n", lmsInit, LMS_GetLastErrorMessage());
    }

    int channel_count = LMS_GetNumChannels(device, LMS_CH_TX);

    myPrint("Tx channel count %d\n", channel_count);

    if (channel < 0) {
        channel = 0;
    }

    if (channel >= channel_count) {
        channel = 0;
    }

    if (channel_count < 2 && mBaseEn) {
        myPrint("ERROR: Using a base station requires a LIME SDR with at least 2 TX channels.");
        LMS_Close(device);
        return;
    }

    myPrint("Using channel %d\n", channel);

    int antenna_count = LMS_GetAntennaList(device, LMS_CH_TX, channel, NULL);

    myPrint("TX%d Channel has %d antenna(ae)\n", channel, antenna_count);

    lms_name_t antenna_name[antenna_count];
    if (antenna_count > 0){
        int i = 0;
        lms_range_t antenna_bw[antenna_count];
        LMS_GetAntennaList(device, LMS_CH_TX, channel, antenna_name);
        for (i = 0 ; i < antenna_count ; i++){
            LMS_GetAntennaBW(device, LMS_CH_TX, channel, i, antenna_bw + i);
            myPrint("Channel %d, antenna [%s] has BW [%lf .. %lf] (step %lf)\n",
                    channel, antenna_name[i], antenna_bw[i].min, antenna_bw[i].max, antenna_bw[i].step);
        }
    }

    if (mBaseEn) {
        int antenna_count_base = LMS_GetAntennaList(device, LMS_CH_TX, channelBase, NULL);

        myPrint("TX%d Channel (base) has %d antenna(ae)\n", channelBase, antenna_count_base);

        lms_name_t antenna_name_base[antenna_count_base];
        if (antenna_count_base > 0){
            int i = 0;
            lms_range_t antenna_bw[antenna_count_base];
            LMS_GetAntennaList(device, LMS_CH_TX, channelBase, antenna_name_base);
            for (i = 0 ; i < antenna_count_base ; i++){
                LMS_GetAntennaBW(device, LMS_CH_TX, channelBase, i, antenna_bw + i);
                myPrint("Channel %d, antenna [%s] has BW [%lf .. %lf] (step %lf)\n",
                        channelBase, antenna_name_base[i],
                        antenna_bw[i].min, antenna_bw[i].max, antenna_bw[i].step);
            }
        }
    }

    if (mBaseEn) {
        int enableChannel = LMS_EnableChannel(device, LMS_CH_TX, channelBase, true);
        if (enableChannel){
            myPrint("ERROR base enableChannel(%lf)=%d(%s)\n",
                    channel, enableChannel, LMS_GetLastErrorMessage());
        }
    }

    // Enable our Tx channel
    int enableChannel = LMS_EnableChannel(device, LMS_CH_TX, channel, true);
    if (enableChannel){
        myPrint("ERROR enableChannel(%lf)=%d(%s)\n",
                channel, enableChannel, LMS_GetLastErrorMessage());
    }

    lms_range_t sampleRateRange;
    int getSampleRateRange = LMS_GetSampleRateRange(device, LMS_CH_TX, &sampleRateRange);
    if (getSampleRateRange){
        myPrint("ERROR getSampleRateRange=%d(%s)\n", getSampleRateRange, LMS_GetLastErrorMessage());
    } else{
        myPrint("sampleRateRange [%lf MHz.. %lf MHz] (step=%lf Hz)\n",
                sampleRateRange.min / 1e6, sampleRateRange.max / 1e6, sampleRateRange.step);
    }

    myPrint("Set sample rate to %lf ...\n", sampleRate);

    int setSampleRate = LMS_SetSampleRate(device, sampleRate, 0);
    if (setSampleRate){
        myPrint("ERROR setSampleRate=%d(%s)\n", setSampleRate, LMS_GetLastErrorMessage());
    }

    int setLOFrequency = LMS_SetLOFrequency(device, LMS_CH_TX, channel, frequency);
    if (setLOFrequency){
        myPrint("ERROR setLOFrequency(%lf)=%d(%s)\n",
                frequency, setLOFrequency, LMS_GetLastErrorMessage());
    }

    int setAntenna = LMS_SetAntenna(device, LMS_CH_TX, channel, antenna);
    if (setAntenna){
        myPrint("ERROR setAntenna(%lf)=%d(%s)\n",
                antenna, setLOFrequency, LMS_GetLastErrorMessage());
    }

    int setNormalizedGain = LMS_SetNormalizedGain(device, LMS_CH_TX, channel, gain);
    if (setNormalizedGain){
        myPrint("ERROR setNormalizedGain(%lf)=%d(%s)\n",
                gain, setNormalizedGain, LMS_GetLastErrorMessage());
    }

    if (mBaseEn) {
        setLOFrequency = LMS_SetLOFrequency(device, LMS_CH_TX, channelBase, frequency);
        if (setLOFrequency) {
            myPrint("ERROR base setLOFrequency(%lf)=%d(%s)\n",
                    frequency, setLOFrequency, LMS_GetLastErrorMessage());
        }

        setAntenna = LMS_SetAntenna(device, LMS_CH_TX, channelBase, antennaBase);
        if (setAntenna){
            myPrint("ERROR base setAntenna(%lf)=%d(%s)\n",
                    antennaBase, setLOFrequency, LMS_GetLastErrorMessage());
        }

        setNormalizedGain = LMS_SetNormalizedGain(device, LMS_CH_TX, channelBase, gainBase);
        if (setNormalizedGain){
            myPrint("ERROR base setNormalizedGain(%lf)=%d(%s)\n",
                    gainBase, setNormalizedGain, LMS_GetLastErrorMessage());
        }
    }

    double actualHostSampleRate = 0.0;
    double actualRFSampleRate = 0.0;
    int getSampleRate = LMS_GetSampleRate(device, LMS_CH_TX, channel, &actualHostSampleRate, &actualRFSampleRate);

    if (getSampleRate){
        myPrint("ERROR getSampleRate=%d(%s)\n", getSampleRate, LMS_GetLastErrorMessage());
    } else{
        myPrint("actualRate %lf (Host) / %lf (RF)\n", actualHostSampleRate, actualRFSampleRate);
    }

    myPrint("Calibrating ..." "\n");

    int calibrate = LMS_Calibrate(device, LMS_CH_TX, channel, bandwidth, 0);
    if(calibrate){
        myPrint("ERROR calibrate=%d(%s)\n", calibrate, LMS_GetLastErrorMessage());
    }

    if (mBaseEn) {
        calibrate = LMS_Calibrate(device, LMS_CH_TX, channelBase, bandwidth, 0);
        if(calibrate){
            myPrint("ERROR base calibrate=%d(%s)\n", calibrate, LMS_GetLastErrorMessage());
        }
    }

    myPrint("Setup TX stream ...\n");

    lms_stream_t tx_stream;
    memset(&tx_stream, 0, sizeof(lms_stream_t));

    tx_stream.channel = channel;
    tx_stream.fifoSize = sampleRate;
    tx_stream.throughputVsLatency = 1.0;
    tx_stream.isTx = true;
    tx_stream.dataFmt = lms_stream_t::LMS_FMT_I12;

    int setupStream = LMS_SetupStream(device, &tx_stream);
    if(setupStream) {
        myPrint("ERROR setupStream=%d(%s)\n", setupStream, LMS_GetLastErrorMessage());
    }

    int nSamples = (int)sampleRate / 10;
    short *genSamples = new short[nSamples * 2];
    short *genSamplesBase = new short[nSamples * 2];

    lms_stream_t tx_stream_base;
    memset(&tx_stream_base, 0, sizeof(lms_stream_t));

    if (mBaseEn) {
        tx_stream_base.channel = channelBase;
        tx_stream_base.fifoSize = sampleRate;
        tx_stream_base.throughputVsLatency = 1.0;
        tx_stream_base.isTx = true;
        tx_stream_base.dataFmt = lms_stream_t::LMS_FMT_I12;

        setupStream = LMS_SetupStream(device, &tx_stream_base);
        if (setupStream) {
            myPrint("ERROR setupStream=%d(%s)\n", setupStream, LMS_GetLastErrorMessage());
        }
    }

    LMS_StartStream(&tx_stream);

    if (mBaseEn) {
        LMS_StartStream(&tx_stream_base);
    }

    int loop = 0;
    while(!mStop) {
        mGps.generateSamples(genSamples, 10, sampleRate);

        if (mBaseEn) {
            mGpsBase.generateSamples(genSamplesBase, 10, sampleRate);
        }

        loop++;

        if((loop % 10) == 0) {
            lms_stream_status_t status;
            LMS_GetStreamStatus(&tx_stream, &status);

            myPrint("Time            :%d s\n", loop / 10);
            myPrint("TX rate         :%.2f MB/s\n", status.linkRate / (1024.0 * 1024.0));
            myPrint("fifoFilledCount :%d\n", status.fifoFilledCount);
            myPrint("fifoSize        :%d\n", status.fifoSize);
            myPrint("underrun        :%d\n", status.underrun);
            myPrint("overrun         :%d\n", status.overrun);
            myPrint("droppedPackets  :%d\n\n", status.droppedPackets);
        }

        lms_stream_meta_t tx_metadata; // Use metadata for additional control over sample send function behaviour
        tx_metadata.timestamp = 1e10;
        tx_metadata.flushPartialPacket = false; // Do not discard data remainder when read size differs from packet size
        tx_metadata.waitForTimestamp = false; // Enable synchronization to HW timestamp

        int sendStream = LMS_SendStream(&tx_stream, genSamples, nSamples, &tx_metadata, 1000);

        if (sendStream < 0) {
            myPrint("ERROR sendStream %d(%s)" "\n", sendStream, LMS_GetLastErrorMessage());
        }

        if (mBaseEn) {
            if((loop % 10) == 0) {
                lms_stream_status_t status;
                LMS_GetStreamStatus(&tx_stream_base, &status);

                myPrint("Base Time            :%d s\n", loop / 10);
                myPrint("Base TX rate         :%.2f MB/s\n", status.linkRate / (1024.0 * 1024.0));
                myPrint("Base fifoFilledCount :%d\n", status.fifoFilledCount);
                myPrint("Base fifoSize        :%d\n", status.fifoSize);
                myPrint("Base underrun        :%d\n", status.underrun);
                myPrint("Base overrun         :%d\n", status.overrun);
                myPrint("Base droppedPackets  :%d\n\n", status.droppedPackets);
            }

            tx_metadata.timestamp = 1e10;
            tx_metadata.flushPartialPacket = false; // Do not discard data remainder when read size differs from packet size
            tx_metadata.waitForTimestamp = false; // Enable synchronization to HW timestamp

            sendStream = LMS_SendStream(&tx_stream_base, genSamplesBase, nSamples, &tx_metadata, 1000);

            if (sendStream < 0) {
                myPrint("ERROR base sendStream %d(%s)" "\n", sendStream, LMS_GetLastErrorMessage());
            }
        }
    }

    if (mStop) {
        myPrint("Aborting...\n");
    } else {
        myPrint("Reached end of file\n");
    }

    LMS_StopStream(&tx_stream);
    LMS_DestroyStream(device, &tx_stream);

    delete [] genSamples;
    delete [] genSamplesBase;

    LMS_EnableChannel(device, LMS_CH_TX, channel, false);
    LMS_Close(device);

    myPrint("Stopped");
}

void LimeSDR::stop()
{
    mStop = true;
}

void LimeSDR::myPrint(const char *format, ...)
{
    va_list arg;
    va_start (arg, format);
    int len;

    char *print_buffer = new char[2048];

    len = vsnprintf(print_buffer, 2048, format, arg);
    va_end (arg);

    if(len > 0) {
        QString s(print_buffer);
        emit statusUpdate(s);
    }

    delete print_buffer;
}
