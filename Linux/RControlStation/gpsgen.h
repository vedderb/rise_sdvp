#ifndef GPSGEN_H
#define GPSGEN_H

#include <cstdint>

class GpsGen
{

public:
    GpsGen();
    ~GpsGen();

    void allocateBuffers(double sampleRate);
    void resetState();
    bool readNavigationFile(const char *filename);
    void setPos(double *llh, double speed);
    void generateSamples(int16_t *buffer, int durationSecD100, double sampFreq);

private:
    // Constants
    const static int mMaxSat = 32;
    const static int mMaxChan = 16;
    const static int mCaSeqLen = 1023;
    const static int mNSbf = 5; // Subframes per frame
    const static int mNDWrdSbf = 10; // Number of words per subframe
    const static int mNDWrd = (mNSbf + 1) * mNDWrdSbf; // Number of words
    const static int mEphemArraySize = 13;

    /*! \brief Structure representing GPS time */
    typedef struct {
        int week;	/*!< GPS week number (since January 1980) */
        double sec; /*!< second inside the GPS \a week */
    } gpstime_t;

    /*! \brief Structure repreenting UTC time */
    typedef struct {
        int y; 		/*!< Calendar year */
        int m;		/*!< Calendar month */
        int d;		/*!< Calendar day */
        int hh;		/*!< Calendar hour */
        int mm;		/*!< Calendar minutes */
        double sec;	/*!< Calendar seconds */
    } datetime_t;

    /*! \brief Structure representing ephemeris of a single satellite */
    typedef struct {
        int vflg;	/*!< Valid Flag */
        datetime_t t;
        gpstime_t toc;	/*!< Time of Clock */
        gpstime_t toe;	/*!< Time of Ephemeris */
        int iodc;	/*!< Issue of Data, Clock */
        int iode;	/*!< Isuse of Data, Ephemeris */
        double deltan;	/*!< Delta-N (radians/sec) */
        double cuc;	/*!< Cuc (radians) */
        double cus;	/*!< Cus (radians) */
        double cic;	/*!< Correction to inclination cos (radians) */
        double cis;	/*!< Correction to inclination sin (radians) */
        double crc;	/*!< Correction to radius cos (meters) */
        double crs;	/*!< Correction to radius sin (meters) */
        double ecc;	/*!< e Eccentricity */
        double sqrta;	/*!< sqrt(A) (sqrt(m)) */
        double m0;	/*!< Mean anamoly (radians) */
        double omg0;	/*!< Longitude of the ascending node (radians) */
        double inc0;	/*!< Inclination (radians) */
        double aop;
        double omgdot;	/*!< Omega dot (radians/s) */
        double idot;	/*!< IDOT (radians/s) */
        double af0;	/*!< Clock offset (seconds) */
        double af1;	/*!< rate (sec/sec) */
        double af2;	/*!< acceleration (sec/sec^2) */
        double tgd;	/*!< Group delay L2 bias */
        int svhlth;
        int codeL2;
        // Working variables follow
        double n; 	/*!< Mean motion (Average angular velocity) */
        double sq1e2;	/*!< sqrt(1-e^2) */
        double A;	/*!< Semi-major axis */
        double omgkdot; /*!< OmegaDot-OmegaEdot */
    } ephem_t;

    typedef struct {
        int enable;
        int vflg;
        double alpha0,alpha1,alpha2,alpha3;
        double beta0,beta1,beta2,beta3;
        double A0,A1;
        int dtls,tot,wnt;
        int dtlsf,dn,wnlsf;
    } ionoutc_t;

    typedef struct {
        gpstime_t g;
        double range; // pseudorange
        double rate;
        double d; // geometric distance
        double azel[2];
        double iono_delay;
    } range_t;

    /*! \brief Structure representing a Channel */
    typedef struct {
        int prn;	/*< PRN Number */
        int ca[mCaSeqLen]; /*< C/A Sequence */
        double f_carr;	/*< Carrier frequency */
        double f_code;	/*< Code frequency */
        double carr_phase;
        double code_phase; /*< Code phase */
        gpstime_t g0;	/*!< GPS time at start */
        unsigned long sbf[5][mNDWrdSbf]; /*!< current subframe */
        unsigned long dwrd[mNDWrd]; /*!< Data words of sub-frame */
        int iword;	/*!< initial word */
        int ibit;	/*!< initial bit */
        int icode;	/*!< initial code */
        int dataBit;	/*!< current data bit */
        int codeCA;	/*!< current C/A code */
        double azel[2];
        range_t rho0;
    } channel_t;

    // Private variables
    int mAllocatedSat[mMaxSat];
    double mXyz[3];
    double mXyzPrev[3];
    gpstime_t mGrx;
    channel_t mChan[mMaxChan];
    double mElvmask; // Elevation mask in in degrees
    double mAntPat[37];
    short *mIqBufChan[mMaxChan];
    double mSpeedNow;

    bool mEphSet;
    int mNeph;
    int mIeph;
    ephem_t mEph[mEphemArraySize][mMaxSat];
    ionoutc_t mIonoutc;
    datetime_t mTmin;
    datetime_t mTmax;
    datetime_t mT0;
    gpstime_t mGmin;
    gpstime_t mGmax;
    gpstime_t mG0;

    // Private functions
    void stepTowards(double *value, double goal, double step);
    void generateChannelSamples(channel_t *chan, int samples, int gain, double dt, short *buffer);
    void subVect(double *y, const double *x1, const double *x2);
    double normVect(const double *x);
    double dotProd(const double *x1, const double *x2);
    void codegen(int *ca, int prn);
    void date2gps(const datetime_t *t, gpstime_t *g);
    void gps2date(const gpstime_t *g, datetime_t *t);
    void xyz2llh(const double *xyz, double *llh);
    void llh2xyz(const double *llh, double *xyz);
    void ltcmat(const double *llh, double t[3][3]);
    void ecef2neu(const double *xyz, double t[3][3], double *neu);
    void neu2azel(double *azel, const double *neu);
    void satpos(ephem_t eph, gpstime_t g, double *pos, double *vel, double *clk);
    void eph2sbf(const ephem_t eph, const ionoutc_t ionoutc, unsigned long sbf[5][mNDWrdSbf]);
    unsigned long countBits(unsigned long v);
    unsigned long computeChecksum(unsigned long source, int nib);
    int replaceExpDesignator(char *str, int len);
    double subGpsTime(gpstime_t g1, gpstime_t g0);
    gpstime_t incGpsTime(gpstime_t g0, double dt);
    int readRinexNavAll(ephem_t eph[][mMaxSat], ionoutc_t *ionoutc, const char *fname);
    double ionosphericDelay(const ionoutc_t *ionoutc, gpstime_t g, double *llh, double *azel);
    void computeRange(range_t *rho, ephem_t eph, ionoutc_t *ionoutc, gpstime_t g, double xyz[]);
    void computeCodePhase(channel_t *chan, range_t rho1, double dt);
    int generateNavMsg(gpstime_t g, channel_t *chan, int init);
    int checkSatVisibility(ephem_t eph, gpstime_t g, double *xyz, double elvMask, double *azel);
    int allocateChannel(channel_t *chan, ephem_t *eph, ionoutc_t ionoutc, gpstime_t grx, double *xyz, double elvMask);
};

#endif // GPSGEN_H
