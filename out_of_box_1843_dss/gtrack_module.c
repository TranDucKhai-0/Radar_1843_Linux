#include "gtrack_module.h"
#include <string.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/drivers/osal/DebugP.h>
#include <ti/alg/gtrack/gtrack.h>
#include <ti/alg/gtrack/include/gtrack_int.h>

/* Các hằng số  */
#define GTRACK_TASK_PRIORITY        4
#define MAX_DETECTED_POINTS         1000
#define MAX_TRACKING_TARGETS        20

/* Biến toàn cục */
static Semaphore_Handle g_gtrackSemHandle;
static void* g_pGtrackHandle = NULL;
static DPIF_PointCloudCartesian g_pointCloudBuffer[MAX_DETECTED_POINTS];
static uint32_t g_numDetectedPoints = 0;

static GTRACK_targetDesc g_targetDescs[MAX_TRACKING_TARGETS];
static uint32_t g_numTargets = 0;
Semaphore_Handle g_gtrackDoneSemHandle;

/* Lưu vận tốc của chính radar (Ego-Velocity) nhận từ MSS (m/s) */
// Lưu ý: Đây là vận tốc của radar theo hướng xuyên tâm, phải được MSS xử lý chuyển sang dạng trục y tiên, x phải, z lên
static float g_egoVelocity = 0.0f;

/* Hàm Boolean trả lời câu hỏi Đúng/Sai */
static bool IsGtrackInitialized(void)
{
    return (g_pGtrackHandle != NULL);
}

/* Hàm Public để cập nhật vận tốc từ MSS truyền xuống */
void UpdateEgoVelocity(float currentVelocity)
{
    g_egoVelocity = currentVelocity;
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

/**
 *  Hàm Private xử lý chính của Task (thêm tiền tố _ + PascalCase)
 */
static void _ProcessGtrackTask(UArg arg0, UArg arg1)
{
    GTRACK_measurementPoint measurementPoints[MAX_DETECTED_POINTS];
    GTRACK_targetDesc targetDescs[MAX_TRACKING_TARGETS];
    uint32_t numTargets;
    uint8_t mIndex[MAX_DETECTED_POINTS];
    uint32_t benchmarks[GTRACK_BENCHMARK_SIZE];

    while (1)
    {
        /* Đợi cờ báo hiệu có dữ liệu Point Cloud mới từ OOB */
        Semaphore_pend(g_gtrackSemHandle, BIOS_WAIT_FOREVER);

        /* Luôn chạy GTRACK kể cả khi không có Point Cloud (để duy trì thuật toán theo dõi/dự đoán) */
        if (IsGtrackInitialized())
        {
            /* Chuyển đổi OOB Point Cloud Cartesian -> GTRACK Spherical */
            for (uint32_t i = 0; i < g_numDetectedPoints; i++)
            {
                /* Theo thư viện, array cartesian cần 6 tham số [x, y, z, vx, vy, vz] cho 3DV */
                float cartesianVec[6] = {0.0f};
                cartesianVec[0] = g_pointCloudBuffer[i].x;
                cartesianVec[1] = g_pointCloudBuffer[i].y;
                cartesianVec[2] = g_pointCloudBuffer[i].z;
                /* Để trống vx, vy, vz vì OOB chỉ cung cấp velocity (đã là vận tốc xuyên tâm) */
                
                /* Dùng hàm nội bộ của GTRACK để chuyển đổi Cartesian -> Spherical */
                float sphericalVec[4] = {0.0f}; /* [Range, Azimuth, Elevation, Doppler] */
                gtrack_cartesian2spherical(GTRACK_STATE_VECTORS_3DV, cartesianVec, sphericalVec);

                measurementPoints[i].range = sphericalVec[0];
                measurementPoints[i].azimuth = sphericalVec[1];
                measurementPoints[i].elevation = sphericalVec[2];
                measurementPoints[i].doppler = sphericalVec[3];

                /* Lấy vận tốc xuyên tâm trực tiếp từ OOB, đè lên giá trị doppler tính toán nội bộ */
                /* BÙ TRỪ VẬN TỐC (COMPENSATION): 
                 * Doppler đo được = Doppler thực của vật - Vận tốc radar * cos(Azimuth) * cos(Elevation) */
                float radialEgoVel = g_egoVelocity * cosf(sphericalVec[1]) * cosf(sphericalVec[2]);
                measurementPoints[i].doppler = g_pointCloudBuffer[i].velocity - radialEgoVel;
                measurementPoints[i].snr = 0.0f; /* Nếu cần SNR, phải lấy thêm từ result->objOutSideInfo */
            }

            /* Chạy thuật toán Tracking */
            gtrack_step(g_pGtrackHandle, measurementPoints, NULL, g_numDetectedPoints, 
                        targetDescs, &numTargets, mIndex, benchmarks);

            /* Lưu kết quả vào biến cục bộ để DPM Task lấy */
            memcpy(g_targetDescs, targetDescs, numTargets * sizeof(GTRACK_targetDesc));
            g_numTargets = numTargets;

            /* Báo hiệu cho luồng DSS DPM là đã xử lý xong */
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
    config.stateVectorType = GTRACK_STATE_VECTORS_3DV;
    config.verbose = GTRACK_VERBOSE_NONE;
    config.deltaT = 0.0333f;                  /* 30 fps */
    config.maxNumPoints = MAX_DETECTED_POINTS;
    config.maxNumTracks = MAX_TRACKING_TARGETS;
    config.maxRadialVelocity = 30.0f;         /* Max 30m/s */
    config.radialVelocityResolution = 0.5f; 
    config.maxAcceleration = 5.0f; 

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
    taskParams.priority = GTRACK_TASK_PRIORITY; /* Ưu tiên thấp hơn DPM (5) */
    taskParams.stackSize = 4 * 1024;
    Task_create(_ProcessGtrackTask, &taskParams, NULL);
}

void SendPointCloudToGtrack(DPIF_PointCloudCartesian *pPointCloud, uint32_t numPoints)
{
    if (numPoints > MAX_DETECTED_POINTS)
    {
        numPoints = MAX_DETECTED_POINTS;
    }

    g_numDetectedPoints = numPoints;

    /* Copy dữ liệu vào vùng nhớ tĩnh của module và kích hoạt semaphore */
    if (pPointCloud != NULL && numPoints > 0)
    {
        memcpy(g_pointCloudBuffer, pPointCloud, numPoints * sizeof(DPIF_PointCloudCartesian));
    }
    else
    {
        g_numDetectedPoints = 0;
    }
    
    Semaphore_post(g_gtrackSemHandle);
}