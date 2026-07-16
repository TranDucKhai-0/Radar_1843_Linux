#ifndef OUT_TYPE_DATA_H
#define OUT_TYPE_DATA_H

/* Biến toàn cục lưu trữ dữ liệu GTrack để đẩy ra ngoài */

typedef struct {
    uint8_t  magicWord[4];  // 4 Bytes: Chuỗi báo hiệu đồng bộ (0x01, 0x02, 0x03, 0x04)
    uint32_t numTargets;    // 4 Bytes: Số lượng đối tượng hợp lệ trong payload (0 - 7)
} outputDataHeader_t;

typedef struct {
    uint32_t tid;           // 4 Bytes: Target ID
    float    posX;          // 4 Bytes: Tọa độ tâm X (m)
    float    posY;          // 4 Bytes: Tọa độ tâm Y (m)
    float    posZ;          // 4 Bytes: Tọa độ tâm Z (m)
    float    velX;          // 4 Bytes: Vận tốc X (m/s)
    float    velY;          // 4 Bytes: Vận tốc Y (m/s)
    float    velZ;          // 4 Bytes: Vận tốc Z (m/s)
    float    dimX;          // 4 Bytes: Kích thước - Độ rộng (m)
    float    dimY;          // 4 Bytes: Kích thước - Độ cao (m)
} outputTargetData_t;

#define MAX_CUSTOM_TARGETS 7
#define FIXED_PACKET_SIZE (sizeof(outputDataHeader_t) + MAX_CUSTOM_TARGETS * sizeof(outputTargetData_t))

#endif

