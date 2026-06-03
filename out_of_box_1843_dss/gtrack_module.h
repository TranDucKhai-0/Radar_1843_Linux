#ifndef GTRACK_MODULE_H
#define GTRACK_MODULE_H

#include <stdint.h>
#include <ti/datapath/dpc/objectdetection/objdethwa/objectdetection.h>

/* Khởi tạo module GTRACK và các Semaphore cần thiết */
void InitGtrackModule(void);

/* Tạo Task độc lập để xử lý GTRACK trên DSP */
void CreateGtrackTask(void);

/* 
 * Hàm dùng để bắn dữ liệu từ luồng DPC của OOB sang GTRACK. 
 * Hàm này sẽ copy dữ liệu và post Semaphore.
 */
void SendPointCloudToGtrack(DPIF_PointCloudCartesian *pPointCloud, uint32_t numPoints);

#endif /* GTRACK_MODULE_H */