/**
 * EasyProtocol
 *
 * @brief   An easy-to-use protocol providing efficient package constructing and
 *          assembling functions.
 *
 * @version Feb 23, 2016 - Make it Independent from specific structures.
 *          Jul 27, 2016 - Release Version.
 *          Feb 22, 2018 - Regular Update. No functional changes.
 *          Mar 06, 2018 - Change Default Buffer Size.
 *          Jan 12, 2023 - Fix package-size parsing of large data packages.
 *
 * @attention
 *          *****        DO NOT CHANGE THIS FILE           *****
 *          ***** Automatically Generated by IMU Assistant *****
 *
 * @attention
 * <h2><center>&copy; COPYRIGHT(c) 2023 SYD Dynamics ApS</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef EASYPROTOCOL_H
#define EASYPROTOCOL_H


#include "EasyQueue.h"
#include "BasicTypes.h"


//------------------------------------------------------------------------------
// Configure Parameters 1/2
    //#define EP_TURN_ON_STATISTICS_
    //#define EP_TURN_ON_DEBUG_
// Configure Parameters 1/2
//------------------------------------------------------------------------------


#ifdef EP_TURN_ON_DEBUG_
    #include <QDebug>
    #include <QString>
    #include <QObject>
#endif

class EasyProtocol
{
public:
    EasyProtocol();
    virtual ~EasyProtocol();

    virtual int Init(int iDataMaxSize, int oDataMaxSize);

    virtual int CreateOutputPackage(
                  char* payloadData,  int payloadSize,
                  char** packageData, int* packageSize);

    virtual int AssembleInputPackage(
                  char* rawData,      int rawDataLenth,
                  char **payloadData, int *payloadSize);


    void  SetChecksumOption(char option);
    int   GetInDataMaxSize(void);
    int   GetOutDataMaxSize(void);
    int   GetRoundUp(void);

#   ifdef EP_TURN_ON_DEBUG_
    void  DebugPrint(QString title, char* data, int length);
#   endif

#   ifdef EP_TURN_ON_STATISTICS_
    virtual int   TotalBytesOfMemoryUsed();
    int   GetStatistic_Recv_Byte_Total();
    float GetStatistic_Recv_Byte_BadHeadRate();
    int   GetStatistic_Recv_Pkg_Total();
    float GetStatistic_Recv_Pkg_OmitRate();
    float GetStatistic_Recv_Pkg_BadRate();
    float GetStatistic_Recv_Pkg_GoodRate();
    int   GetStatistic_Send_Pkg_Total();
#   endif

protected:
    virtual int UnWrapInData(char* packageData, char**payloadData, int payloadSize);

    /**
     *  Data Size
     */
    int    iDS;
    int    oDS;

private:
    /**
      * Statistics
      */
    int totalMem;
#   ifdef EP_TURN_ON_STATISTICS_
    unsigned int sRecvByte_Total;  // Total Received Bytes
    unsigned int sRecvByte_BadHead;
    unsigned int sRecvByte_GoodHead;
    unsigned int sRecvPkg_Total;
    unsigned int sRecvPkg_Omit;
    unsigned int sRecvPkg_Good;
    unsigned int sRecvPkg_Bad;
    unsigned int sSendPkg;
#   endif

protected:
    /**
     * Data Frame Wrap & Unwrap Data:
     */
    char    HEAD_1_;
    char    HEAD_2_;
    char    HEAD_LENGTH_;
    char    HEAD_LENGTH_MSB_AHEAD_;
    char    SIZE_LENGTH_;
    int     MAX_PAYLOAD_SIZE_;
    char    CHECKSUM_LENGTH_;

    /**
      * Receive data auto Round-Up
      */
    char    ENABLE_ROUND_UP_;
    char    ROUND_UP_NUM_;
    char    roundUpTmp;

    /**
      * CheckSum
      */
    char    CHECKSUM_OPTION_;
    virtual uint16 Checksum_Generate(char* data, int dataLength);
    virtual int    Checksum_Verify(  char* data, int dataLength, uint16 checksumToBeVerified);

private:
    #define EP_PKG_MODIFIER_SIZE_ ( HEAD_LENGTH_ + SIZE_LENGTH_ + CHECKSUM_LENGTH_)
    char *outBuf;
    char *inBuf;
    char* p;
    EasyQueue<char> inQueue;
    char omitStream;
    int  declaredPayloadSize;
};

#endif // EASYPROTOCOL_H
