/*
    Copyright 2016 Benjamin Vedder	benjamin@vedder.se

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

#include "ping.h"
#include <QDebug>

#ifdef Q_OS_UNIX
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/param.h>
#include <sys/socket.h>
#include <sys/file.h>
#include <sys/time.h>
#include <netinet/in_systm.h>
#include <netinet/in.h>
#include <netinet/ip.h>
#include <netinet/ip_icmp.h>
#include <netdb.h>
#include <unistd.h>
#include <errno.h>
#endif

Ping::Ping(QObject *parent) : QThread(parent)
{
    mSocket = -1;
#if defined(Q_OS_UNIX) && !defined(Q_OS_ANDROID)
    mPacket = new unsigned char[120000];
    mOutpack = new unsigned char[120000];
#endif
}

Ping::~Ping()
{
    this->wait();

#if defined(Q_OS_UNIX) && !defined(Q_OS_ANDROID)
    if (mSocket >= 0) {
        close(mSocket);
    }

    delete[] mPacket;
    delete[] mOutpack;
#endif
}

bool Ping::pingHost(QString host, int len, QString msg)
{
#if defined(Q_OS_UNIX) && !defined(Q_OS_ANDROID)
    if (this->isRunning()) {
        //emit pingError(msg, "Ping already in progress");
        return false;
    } else {
        if (len < ICMP_MINLEN || len > 65536 - 60) {
            //emit pingError(msg, "Invalid length");
            return false;
        }

        mHost = host;
        mMsg = msg;
        mLen = len;
        this->start();
        return true;
    }
#else
    (void)host;
    (void)len;
    (void)msg;
    emit pingError(mMsg, "Ping support is not implemented for your operating system");
    return false;
#endif
}

// From http://www.linuxforums.org/forum/linux-networking/60389-implementing-ping-c.html#post382967
#define	MAXIPLEN	60
#define	MAXICMPLEN	76
#define	MAXPACKET	(65536 - 60 - ICMP_MINLEN)/* max packet size */

void Ping::run()
{
#if defined(Q_OS_UNIX) && !defined(Q_OS_ANDROID)
    int i, cc, packlen;
    int datalen = mLen - ICMP_MINLEN;
    struct hostent *hp;
    struct sockaddr_in to, from;
    char hnamebuf[MAXHOSTNAMELEN];
    QString hostname;
    struct icmp *icp;
    int ret, fromlen, hlen;
    fd_set rfds;
    struct timeval tv;
    int retval;
    struct timeval start, end;
    int end_t;
    bool cont = true;

    const int errbufl = 256;
    char errbuf[errbufl];

    to.sin_family = AF_INET;

    // try to convert as dotted decimal address, else if that fails assume it's a hostname
    to.sin_addr.s_addr = inet_addr(mHost.toLocal8Bit().data());
    if (to.sin_addr.s_addr != (u_int)-1) {
        hostname = mHost;
    } else {
        hp = gethostbyname(mHost.toLocal8Bit().data());
        if (!hp) {
            emit pingError(mMsg, "unknown host");
            return;
        }

        to.sin_family = hp->h_addrtype;
        bcopy(hp->h_addr, (caddr_t)&to.sin_addr, hp->h_length);
        strncpy(hnamebuf, hp->h_name, sizeof(hnamebuf) - 1);
        hostname = hnamebuf;
    }

    packlen = datalen + MAXIPLEN + MAXICMPLEN;

    if (mSocket < 0) {
        if ( (mSocket = socket(AF_INET, SOCK_RAW, IPPROTO_ICMP)) < 0) {
            char const *res = strerror_r(errno, errbuf, errbufl);
            QString error = "socket: " + QString::fromLocal8Bit(res);
            emit pingError(mMsg, error);
            return;
        }
    }

    icp = (struct icmp *)mOutpack;
    icp->icmp_type = ICMP_ECHO;
    icp->icmp_code = 0;
    icp->icmp_cksum = 0;
    icp->icmp_seq = 12345;	/* seq and id must be reflected */
    icp->icmp_id = getpid();

    cc = datalen + ICMP_MINLEN;
    icp->icmp_cksum = in_cksum((unsigned short *)icp,cc);

    gettimeofday(&start, NULL);

    i = sendto(mSocket, (char *)mOutpack, cc, 0, (struct sockaddr*)&to, (socklen_t)sizeof(struct sockaddr_in));
    if (i < 0 || i != cc) {
        if (i < 0) {
            char const *res = strerror_r(errno, errbuf, errbufl);
            QString error = "sendto error: " + QString::fromLocal8Bit(res);
            emit pingError(mMsg, error);
            //return;
        }
    }

    // Watch stdin (fd 0) to see when it has input.
    FD_ZERO(&rfds);
    FD_SET(mSocket, &rfds);
    // Wait up to one second.
    tv.tv_sec = 1;
    tv.tv_usec = 0;

    while(cont) {
        retval = select(mSocket+1, &rfds, NULL, NULL, &tv);
        if (retval == -1) {
            char const *res = strerror_r(errno, errbuf, errbufl);
            QString error = "select(): " + QString::fromLocal8Bit(res);
            emit pingError(mMsg, error);
            return;
        } else if (retval) {
            fromlen = sizeof(sockaddr_in);
            if ( (ret = recvfrom(mSocket, (char *)mPacket, packlen, 0,(struct sockaddr *)&from, (socklen_t*)&fromlen)) < 0) {
                char const *res = strerror_r(errno, errbuf, errbufl);
                QString error = "recvfrom error: " + QString::fromLocal8Bit(res);
                emit pingError(mMsg, error);
                return;
            }

            // Check the IP header
            hlen = sizeof( struct ip );
            if (ret < (hlen + ICMP_MINLEN)) {
                emit pingError(mMsg, "packet too short (" + QString::number(ret)  +
                                " bytes) from" + hostname);
                return;
            }

            // Now the ICMP part
            icp = (struct icmp *)(mPacket + hlen);
            if (icp->icmp_type == ICMP_ECHOREPLY) {
                if (icp->icmp_seq != 12345) {
                    continue;
                }

                if (icp->icmp_id != getpid()) {
                    continue;
                }

                cont = false;
            } else {
                continue;
            }

            gettimeofday(&end, NULL);
            end_t = 1000000 * (end.tv_sec - start.tv_sec) + (end.tv_usec - start.tv_usec);

            if(end_t < 1) {
                end_t = 1;
            }

            emit pingRx(end_t, mMsg);
            return;
        } else {
            emit pingError(mMsg, "no data within one second");
            return;
        }
    }
#endif
}

uint16_t Ping::in_cksum(uint16_t *addr, unsigned len)
{
    uint16_t answer = 0;
    /*
       * Our algorithm is simple, using a 32 bit accumulator (sum), we add
       * sequential 16 bit words to it, and at the end, fold back all the
       * carry bits from the top 16 bits into the lower 16 bits.
       */
    uint32_t sum = 0;
    while (len > 1)  {
        sum += *addr++;
        len -= 2;
    }

    // mop up an odd byte, if necessary
    if (len == 1) {
        *(unsigned char *)&answer = *(unsigned char *)addr ;
        sum += answer;
    }

    // add back carry outs from top 16 bits to low 16 bits
    sum = (sum >> 16) + (sum & 0xffff); // add high 16 to low 16
    sum += (sum >> 16); // add carry
    answer = ~sum; // truncate to 16 bits
    return answer;
}
