#include <Arduino.h>
#include <AccelStepper.h>
#include <float.h>
#include <limits.h>

#define MOTOR_BLOCK false

// 引脚定义 esp32上6~11引脚不能使用
#define aStepPin 12
#define bStepPin 13
#define cStepPin 14
#define aDirPin 15
#define bDirPin 16
#define cDirPin 17
#define enablePin 25 // 全部电机使能引脚
#define spnEnPin 26  // 电磁阀使能引脚
#define resumePin 27 // Resume引脚, 用于触发中断

// 电机对象
AccelStepper stepperA(AccelStepper::DRIVER, aStepPin, aDirPin); // 主臂
AccelStepper stepperB(AccelStepper::DRIVER, bStepPin, bDirPin); // 副臂
AccelStepper stepperC(AccelStepper::DRIVER, cStepPin, cDirPin); // 丝杠

// 常量定义
const float L1 = 212, L2 = 200, s = 8; // 主臂-L1, 副臂-L2, 丝杠螺距-s
const float zStep = 400.f;             // 200/8*16, z向移动1mm所需步数
const float upperArmStep = 32.0f;      // 200/360*16*(72/20), 主臂转动1度所需步数
const float foreArmStep = 51.77f;      // 200/360*16*(62/20)*(62/33), 副臂转动1度所需步数

// 结构体定义
struct Vec3f
{
public:
    float x, y, z; // 可表示 末端坐标或电机角度
    Vec3f(float X = 0, float Y = 0, float Z = 0) : x(X), y(Y), z(Z){};
    Vec3f operator-(const Vec3f &vec)
    {
        return Vec3f(x - vec.x, y - vec.y, z - vec.z);
    }
};
// 任务项   0-串口打印当前坐标 1-设置当前坐标为(x,y,z) 2-移动机械臂至(x,y,z) 3-启动吸盘 4-关闭吸盘
struct TaskItem
{
    uint32_t taskIndex;
    Vec3f value;
    TaskItem(uint32_t index = 0, Vec3f val = Vec3f()) : taskIndex(index), value(val){};
};

// FreeRTOS 变量
QueueHandle_t taskQueue = xQueueCreate(12, sizeof(TaskItem));
QueueHandle_t msgQueue = xQueueCreate(6, sizeof(char[32]));
SemaphoreHandle_t xMutex = xSemaphoreCreateMutex();
// FreeRTOS 任务句柄
static TaskHandle_t xMainController = NULL;
static TaskHandle_t xMotorRun = NULL;
static TaskHandle_t xMagneticValve = NULL;

// 其他全局对象
static struct Vec3f currentPos = Vec3f(412.f, 0, 0), targetPos, targetAngle;
volatile static bool valveStatus = false;

// 根据目标三维空间坐标，计算电机对应角度
bool InverseKinematics(const Vec3f targetPos)
{
    float x = targetPos.x, y = targetPos.y, z = targetPos.z;
    float theta1, theta2, theta3;
    float alpha = atan2(y, x);
    float cosTheta2 = (x * x + y * y - L1 * L1 - L2 * L2) / (2 * L1 * L2);
    float sinTheta2 = sqrt(1 - cosTheta2 * cosTheta2); // 逆时针为正

    theta1 = alpha - atan2(L2 * sinTheta2, L1 + L2 * cosTheta2);
    theta2 = atan2(sinTheta2, cosTheta2);
    theta3 = 2 * PI * z / s;
    if (isnan(theta1) || isnan(theta2) || isinf(theta1) || isinf(theta2))
        return false;
    targetAngle = Vec3f(theta1, theta2, theta3);
    return true;
}

// 根据当前转角和目标转角，计算电机需要发出的脉冲
Vec3f CalcMoterStep(const Vec3f &deltaAngle)
{
    float aStep = upperArmStep * deltaAngle.x;
    float bStep = (deltaAngle.x * 33 / 62 + deltaAngle.y) * foreArmStep; // 主臂转动θ度时，主臂与副臂之间的夹角会改变(逆时针方向θ*r1/r2)
    float cStep = zStep * deltaAngle.z;
    return Vec3f(aStep, bStep, cStep);
}

// 控制机械臂移动
void Task_MotorRun(void *ptParam)
{
    if (InverseKinematics(targetPos))
    {
        Vec3f step = CalcMoterStep(targetAngle);
        String msg = "steps:";
        msg += "X" + String(step.x) + " Y" + String(step.y) + " Z" + String(step.z);
        xQueueSend(msgQueue, msg.c_str(), portMAX_DELAY);
#if MOTOR_BLOCK
        // 阻塞运行, 电机依次运动
        stepperA.runToNewPosition(step.x);
        stepperB.runToNewPosition(step.y);
        stepperC.runToNewPosition(step.z);
#else
        stepperA.moveTo(step.x);
        stepperB.moveTo(step.y);
        stepperC.moveTo(step.z);
        // int tick = 0;
        while (stepperA.currentPosition() != (int)step.x ||
               stepperB.currentPosition() != (int)step.y ||
               stepperC.currentPosition() != (int)step.z)
        {
            stepperA.run();
            stepperB.run();
            stepperC.run();
            vTaskDelay(1 / portTICK_RATE_MS);
            // if (tick / 1000 == 0)
            //     xQueueSend(msgQueue, String(stepperC.currentPosition()).c_str(), 20);
        }
#endif
        currentPos = targetPos;
        xQueueSend(msgQueue, "Motor reached target!", portMAX_DELAY);
    }
    else
    {
        xQueueSend(msgQueue, "Target pos error!!", portMAX_DELAY);
    }
    xTaskNotifyGive(xMainController);
    // RunToNewPosition
    vTaskDelete(NULL);
}

// 控制电磁阀开关   (spn引脚)
void Task_MagneticValve(void *ptParam)
{
    digitalWrite(spnEnPin, valveStatus);

    if (valveStatus)
        xQueueSend(msgQueue, "Valve already open", portMAX_DELAY);
    else
        xQueueSend(msgQueue, "Valve already close", portMAX_DELAY);
    xTaskNotifyGive(xMainController);
    vTaskDelete(NULL);
}

// 主任务流程管理, 从taskQueue接收指令并执行
void Task_MainController(void *ptParam)
{
    // uint32_t ulNotificationValue;
    TaskItem data;
    xQueueSend(msgQueue, "Create task: MainController", portMAX_DELAY);
    for (;;)
    {
        vTaskDelay(200);
        // ulNotificationValue = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        // xTaskNotifyWait(pdFALSE, ULONG_MAX, &ulNotificationValue, portMAX_DELAY);
        if (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) <= 0)
            continue;
        if (xQueueReceive(taskQueue, &data, portMAX_DELAY) == pdPASS)
        {
            switch (data.taskIndex)
            {
            case 0:
            {
                String msg = "curPos:";
                msg += "X" + String(currentPos.x) + " Y" + String(currentPos.y) + " Z" + String(currentPos.z);
                xQueueSend(msgQueue, msg.c_str(), portMAX_DELAY);
                xTaskNotifyGive(xMainController);
                break;
            }
            case 1:
            {
                currentPos = data.value;
                String msg = "curPos:";
                msg += "X" + String((int)currentPos.x) + " Y" + String((int)currentPos.y) + " Z" + String((int)currentPos.z);
                xQueueSend(msgQueue, msg.c_str(), portMAX_DELAY);
                xTaskNotifyGive(xMainController);
                break;
            }
            case 2:
            {
                targetPos = data.value;
                xTaskCreate(Task_MotorRun, "motor run", 1024 * 8, NULL, 1, NULL);
                break;
            }
            case 3:
                valveStatus = true;
                xTaskCreate(Task_MagneticValve, "Open Valve", 1024, NULL, 1, NULL);
                break;
            case 4:
                valveStatus = false;
                xTaskCreate(Task_MagneticValve, "Close Valve", 1024, NULL, 1, NULL);
                break;
            default:
                xQueueSend(msgQueue, "Command Error", portMAX_DELAY);
                break;
            }
        }
    }
}

// 接收串口数据, 解析并添加至taskQueue
void Task_SerialRecv(void *ptParam)
{
    String recv = "";
    uint32_t operType = 0;
    float x = 0, y = 0, z = 0;
    int x_index = -1, y_index = -1, z_index = -1;
    xTaskNotifyGive(xMainController); // 通知MainController开始处理任务
    xQueueSend(msgQueue, "Create task: SerialRecv", portMAX_DELAY);
    for (;;)
    {
        recv = "";
        vTaskDelay(200);
        while (Serial.available() > 0)
        {
            recv += char(Serial.read());
            delay(2);
        }
        if (recv.length() <= 0)
            continue;
        if (recv.length() <= 0 || !recv.startsWith("M") || recv.indexOf(" ") <= 1)
        {
            xQueueSend(msgQueue, "Error command without M!", 200);
            continue;
        }

        operType = recv.substring(1, recv.indexOf(" ")).toInt();
        switch (operType)
        {
        case 0:
            break;
        case 3:
        case 4:
            break;
        default:
            x_index = recv.indexOf('X');
            y_index = recv.indexOf('Y');
            z_index = recv.indexOf('Z');
            if (x_index < 2 || y_index < 2 || z_index < 2 || y_index <= x_index || z_index <= y_index)
            {
                xQueueSend(msgQueue, "Error X Y Z!", 200);
                continue;
            }
            x = recv.substring(x_index + 1, y_index).toFloat();
            y = recv.substring(y_index + 1, z_index).toFloat();
            z = recv.substring(z_index + 1).toFloat();
            break;
        }
        struct TaskItem item(operType, Vec3f(x, y, z));
        xQueueSend(taskQueue, &item, portMAX_DELAY);
    }
}

// 串口打印, 将msgQueue的内容打印至串口
void Task_SerialPrint(void *ptParam)
{
    char msg[32];
    xQueueSend(msgQueue, "Create task: SerialPrint", portMAX_DELAY);
    for (;;)
    {
        // Serial.println(uxTaskGetStackHighWaterMark(xMainController));
        // xPortGetFreeHeapSize();
        if (xQueueReceive(msgQueue, msg, 200) == pdFALSE)
            continue;
        Serial.println(msg);
        memset(msg, 0, 32);
        vTaskDelay(200);
    }
}

void Task_Test(void *pt)
{
    String recv = "";
    Serial.println("start");
    for (;;)
    {
        while (Serial.available() > 0)
        {
            recv += char(Serial.read());
            delay(2);
        }
        if (recv != "")
            Serial.println(recv);
        // Serial.println(uxTaskGetStackHighWaterMark(nullptr));
        recv = "";
        vTaskDelay(200);
    }
}

void setup()
{
    Serial.begin(115200);
    stepperA.setMaxSpeed(10000);
    stepperA.setAcceleration(1500);
    stepperA.setMinPulseWidth(100);
    stepperB.setMaxSpeed(10000);
    stepperB.setAcceleration(1500);
    stepperB.setMinPulseWidth(100);
    stepperC.setMaxSpeed(10000);
    stepperC.setAcceleration(1500);
    stepperC.setMinPulseWidth(100);

    Serial.println("------Program---Start------");
    xTaskCreate(Task_MainController, "main controller", 1024 * 12, NULL, 1, &xMainController);
    xTaskCreate(Task_SerialRecv, "serial recv", 1024 * 4, NULL, 1, NULL);
    xTaskCreate(Task_SerialPrint, "serial print", 1024 * 4, NULL, 1, NULL);
    // Serial.println(uxTaskGetStackHighWaterMark(nullptr));
}

void loop()
{
}