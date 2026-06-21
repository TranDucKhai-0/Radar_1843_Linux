#include "gtrack_module.h"
#include <string.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/drivers/osal/DebugP.h>
#include <ti/alg/gtrack/gtrack.h>
#include <ti/alg/gtrack/include/gtrack_int.h>


#define DISTANCE_MIN 2.0f // mét
#define DISTANCE_MAX 80.0f // mét

/*Đây là note fix của Đạt Lê, 18 tháng 6 2026*/
/* Biến toàn cục */
static Semaphore_Handle g_gtrackSemHandle;
Semaphore_Handle g_gtrackDoneSemHandle;
static void* g_pGtrackHandle = NULL;
static uint32_t g_numDetectedPoints = 0;
static uint32_t g_numTargets = 0;


#pragma DATA_SECTION(g_pointCloudBuffer, ".l3ram");
static DPIF_PointCloudCartesian g_pointCloudBuffer[MAX_DETECTED_POINTS];

#pragma DATA_SECTION(g_targetDescs, ".l3ram");
static GTRACK_targetDesc g_targetDescs[MAX_TRACKING_TARGETS];

#pragma DATA_SECTION(g_measurementPoints, ".l3ram");
static GTRACK_measurementPoint g_measurementPoints[MAX_DETECTED_POINTS];

#pragma DATA_SECTION(g_mIndex, ".l3ram");
static uint8_t g_mIndex[MAX_DETECTED_POINTS];

#pragma DATA_SECTION(g_uIndex, ".l3ram");
static uint8_t g_uIndex[MAX_DETECTED_POINTS];

#pragma DATA_SECTION(g_presence, ".l3ram");
static uint8_t g_presence[MAX_TRACKING_TARGETS];

#pragma DATA_SECTION(g_benchmarks, ".l3ram");
static uint32_t g_benchmarks[GTRACK_BENCHMARK_SIZE];
/* ====================================================================== */

/* Lưu vector vận tốc 3D của chính radar (Ego-Velocity) nhận từ MSS qua hệ trục Radar (X phải, Y tiến, Z lên) */
static float g_egoVelocityVec[3] = {0.0f, 0.0f, 0.0f};

/* Hàm Boolean trả lời câu hỏi Đúng/Sai */
static bool IsGtrackInitialized(void)
{
    return (g_pGtrackHandle != NULL);
}

/* Hàm Public để cập nhật vận tốc từ luồng DPM IOCTL truyền xuống */
void UpdateEgoVelocity(float *egoVelVec)
{
    if (egoVelVec != NULL)
    {
        g_egoVelocityVec[0] = egoVelVec[0];
        g_egoVelocityVec[1] = egoVelVec[1];
        g_egoVelocityVec[2] = egoVelVec[2];
    }
}

/* DSS Main gọi hàm này để lấy kết quả đóng gói gửi MSS */
void GetGtrackTargetList(GTRACK_targetDesc *targetList, uint32_t *numTargets)
{
    if (targetList != NULL && numTargets != NULL)
    {
        memcpy(targetList, g_targetDescs, g_numTargets * sizeof(GTRACK_targetDesc));
        *numTargets = g_numTargets;
    }
}

/* Lấy trực tiếp con trỏ đến danh sách Target */
GTRACK_targetDesc* GetGtrackTargetListPointer(uint32_t *numTargets)
{
    if (numTargets != NULL)
    {
        *numTargets = g_numTargets;
    }
    return g_targetDescs;
}

/**
 *  Hàm Private xử lý chính của Task (thêm tiền tố _ + PascalCase)
 */
static void _ProcessGtrackTask(UArg arg0, UArg arg1)
{
    /* Chỉ còn lại các biến cục bộ siêu nhẹ */
    uint16_t numTargets;
    uint32_t i; 

    while (1)
    {
        Semaphore_pend(g_gtrackSemHandle, BIOS_WAIT_FOREVER);

        if (IsGtrackInitialized())
        {
            uint32_t numValidPoints = 0;
            for (i = 0; i < g_numDetectedPoints; i++)
            {
                float cartesianVec[9] = {0.0f};
                cartesianVec[0] = g_pointCloudBuffer[i].x;
                cartesianVec[1] = g_pointCloudBuffer[i].y;
                cartesianVec[2] = g_pointCloudBuffer[i].z;
                
                float sphericalVec[4] = {0.0f}; 
                gtrack_cartesian2spherical(GTRACK_STATE_VECTORS_3DA, cartesianVec, sphericalVec);

                // Lọc bỏ đối tượng nằm ngoài khoảng nhìn
                if (sphericalVec[0] < DISTANCE_MIN || sphericalVec[0] > DISTANCE_MAX)
                    continue;

                /* Dùng trực tiếp mảng toàn cục g_measurementPoints */
                g_measurementPoints[numValidPoints].array[0] = sphericalVec[0]; 
                g_measurementPoints[numValidPoints].array[1] = sphericalVec[1]; 
                g_measurementPoints[numValidPoints].array[2] = sphericalVec[2]; 

                float radialEgoVel = 0.0f;
                float range = sphericalVec[0];
                if (range > 0.01f)
                {
                    radialEgoVel = (g_egoVelocityVec[0] * cartesianVec[0] + 
                                    g_egoVelocityVec[1] * cartesianVec[1] + 
                                    g_egoVelocityVec[2] * cartesianVec[2]) / range;
                }
                g_measurementPoints[numValidPoints].array[3] = g_pointCloudBuffer[i].velocity - radialEgoVel;
                g_measurementPoints[numValidPoints].snr      = 0.0f; 
                numValidPoints++;
            }

            /* Truyền các mảng toàn cục vào thuật toán GTRACK với số điểm hợp lệ thực tế */
            gtrack_step(g_pGtrackHandle, g_measurementPoints, NULL, (uint16_t)numValidPoints, 
                        g_targetDescs, &numTargets, g_mIndex, g_uIndex, g_presence, g_benchmarks);

            /* Gán giá trị đích */
            g_numTargets = numTargets;
            g_numDetectedPoints = 0; 
            
            Semaphore_post(g_gtrackDoneSemHandle);
        }
    }
}

void InitGtrackModule(void)
{
    int32_t errCode;
    GTRACK_moduleConfig config;

    memset(&config, 0, sizeof(GTRACK_moduleConfig));

    /* Cấu hình GTRACK cơ bản dựa trên yêu cầu yeu_cau.txt */
    config.stateVectorType = GTRACK_STATE_VECTORS_3DA;
    config.verbose = GTRACK_VERBOSE_NONE;
    config.deltaT = 0.0333f;                  /* 30 fps */
    config.maxNumPoints = MAX_DETECTED_POINTS;
    config.maxNumTracks = MAX_TRACKING_TARGETS;
    config.maxRadialVelocity = 30.0f;         /* Max 30m/s */
    config.radialVelocityResolution = 0.5f; 
    config.maxAcceleration[0] = 5.0f; 
    config.maxAcceleration[1] = 5.0f; 
    config.maxAcceleration[2] = 5.0f;
    /* Cấu hình giới hạn gia tốc tối đa (5.0 m/s^2) cho cả 3 trục không gian (X, Y, Z) */
    
    g_pGtrackHandle = gtrack_create(&config, &errCode);
    
    /* Khởi tạo Semaphore để đánh thức Task */
    Semaphore_Params semParams;
    Semaphore_Params_init(&semParams);
    semParams.mode = Semaphore_Mode_BINARY;
    g_gtrackSemHandle = Semaphore_create(0, &semParams, NULL);

    g_gtrackDoneSemHandle = Semaphore_create(0, &semParams, NULL);
}

void CreateGtrackTask(void)
{
    Task_Params taskParams;
    Task_Params_init(&taskParams);
    taskParams.priority = GTRACK_TASK_PRIORITY; 
    
    /* GIẢM STACK XUỐNG 2KB VÌ DỮ LIỆU ĐÃ NẰM HẾT Ở L3 RAM */
    taskParams.stackSize = 2 * 1024; 
    
    Task_create(_ProcessGtrackTask, &taskParams, NULL);
}

void SendPointCloudToGtrack(DPIF_PointCloudCartesian *pPointCloud, uint32_t numPoints)
{
    /* Kiểm tra giới hạn để tránh tràn bộ đệm MAX_DETECTED_POINTS */
    if (g_numDetectedPoints + numPoints > MAX_DETECTED_POINTS)
    {
        numPoints = MAX_DETECTED_POINTS - g_numDetectedPoints;
    }

    /* Thực hiện tích lũy nối đuôi dữ liệu */
    if (pPointCloud != NULL && numPoints > 0)
    {
        memcpy((void *)&g_pointCloudBuffer[g_numDetectedPoints], (const void *)pPointCloud, numPoints * sizeof(DPIF_PointCloudCartesian));
        g_numDetectedPoints += numPoints; 
    }
    
    /* Kích hoạt semaphore để Task GTRACK thức dậy xử lý */
    Semaphore_post(g_gtrackSemHandle);
}

/* CÁC HÀM PORTING BẮT BUỘC ĐỂ THƯ VIỆN GTRACK HOẠT ĐỘNG                  */
#include <stdlib.h> 

/* Hàm cấp phát bộ nhớ cho GTRACK khi khởi tạo */
void *gtrack_alloc(uint32_t numElements, uint32_t sizeInBytes)
{
    return malloc(numElements * sizeInBytes);
}

/* Hàm giải phóng bộ nhớ (thường dùng khi xóa GTRACK) */
void gtrack_free(void *pFree, uint32_t sizeInBytes)
{
    if (pFree != NULL)
    {
        free(pFree);
    }
}

/* Hàm in log của GTRACK */
void gtrack_log(GTRACK_VERBOSE_TYPE level, const char *format, ...)
{
    /* Chúng ta để trống hàm này để tiết kiệm tài nguyên xử lý của chip, 
       không cần in log của thư viện ra màn hình */
}
/* ====================================================================== */
