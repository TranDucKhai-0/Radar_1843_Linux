#ifndef GTRACK_MODULE_H
#define GTRACK_MODULE_H

#include <stdint.h>
#include <ti/datapath/dpc/objectdetection/objdethwa/objectdetection.h>
#include <ti/alg/gtrack/gtrack.h>
#include <ti/sysbios/knl/Semaphore.h>

/* Khởi tạo module GTRACK và các Semaphore cần thiết */
void InitGtrackModule(void);

/* 
 * Cập nhật vận tốc di chuyển của Radar (Ego-Velocity) 
 * egoVelVec[0] = Vận tốc trục X (sang phải)
 * egoVelVec[1] = Vận tốc trục Y (tiến tới)
 * egoVelVec[2] = Vận tốc trục Z (hướng lên)
 */
void UpdateEgoVelocity(float *egoVelVec);

/* Tạo Task độc lập để xử lý GTRACK trên DSP */
void CreateGtrackTask(void);

/* 
 * Hàm dùng để bắn dữ liệu từ luồng DPC của OOB sang GTRACK. 
 * Hàm này sẽ copy dữ liệu và post Semaphore.
 */
void SendPointCloudToGtrack(DPIF_PointCloudCartesian *pPointCloud, uint32_t numPoints);

/* Lấy danh sách Target đã được track xong */
void GetGtrackTargetList(GTRACK_targetDesc *targetList, uint32_t *numTargets);

/* Semaphore để DSS Main chờ GTRACK xử lý xong */
extern Semaphore_Handle g_gtrackDoneSemHandle;

#endif /* GTRACK_MODULE_H */