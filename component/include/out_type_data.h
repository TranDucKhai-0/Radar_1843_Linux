#ifndef OUT_TYPE_DATA_H
#define OUT_TYPE_DATA_H
#include <stdint.h>

/* UART v3, little endian. Header, numPoints * (x,y,z,doppler),
 * then numTargets * outputTargetData_t. No padding on the wire. */
#define RADAR_PACKET_MAGIC "RADARPC2"
#define RADAR_PACKET_VERSION 3U
#define MAX_OUTPUT_POINTS 250U
#define MAX_CUSTOM_TARGETS 20U

typedef struct {
    uint32_t initialized;
    uint32_t inputPoints;
    uint32_t movingPoints;
    uint32_t stepUs;
    uint32_t peakStepUs;
    uint32_t stackUsed;
    uint32_t stackSize;
    int32_t initError;
    float maxSnr;
    float maxAbsDoppler;
} outputGtrackDiagnostics_t;

typedef struct {
    uint8_t magicWord[8];
    uint32_t version;
    uint32_t packetLength;
    uint32_t frameNumber;
    uint32_t numPoints;
    uint32_t numTargets;
    uint32_t numDetectedPoints; /* Before the output/tracker cap of 250. */
    outputGtrackDiagnostics_t tracker;
} outputDataHeader_t;

typedef struct {
    uint32_t tid;
    float posX, posY, posZ;
    float velX, velY, velZ;
    float dimX, dimY;
} outputTargetData_t;

#define MAX_PACKET_SIZE (sizeof(outputDataHeader_t) + MAX_OUTPUT_POINTS * 16U + MAX_CUSTOM_TARGETS * sizeof(outputTargetData_t))
/* Fail the build if either compiler changes the UART ABI. */
typedef char RadarHeaderSizeCheck[(sizeof(outputDataHeader_t) == 72U) ? 1 : -1];
typedef char RadarTargetSizeCheck[(sizeof(outputTargetData_t) == 36U) ? 1 : -1];
#endif
