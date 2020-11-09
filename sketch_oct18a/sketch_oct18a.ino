#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
//#include <ArduinoJson.h> //Json 사용을 위한 라이브러리 
MPU6050 mpu;

#define OUTPUT_READABLE_YAWPITCHROLL
#define INTERRUPT_PIN 15  // MPU6050 - ESP32 INTERRUPT_PIN 번호 

// MPU control/status vars  //센서 제어 및 상태들을 저장한 변수
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[1024]; // FIFO storage buffer , 기존 코드는 64 인데 FIFO overflow 방지를 위해 늘림

//orientation/motion vars  //ㅊ
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


// ===               인터럽트 발생유무 확인용                ===

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

// ===               setup 시작                ===+

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
       //I2C 셋팅 및 시작
        Wire.begin();
        Wire.setClock(500000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    Serial.begin(9600,SERIAL_8N1,16,17); // Xbee 용으로 9600 사용 , RX2, TX2를 사용했기 때문에 설정해줌 UART2로 통신
    while (!Serial); // wait for Leonardo enumeration, others continue immediately
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize(); //MPU센서 초기화
    pinMode(INTERRUPT_PIN, INPUT); //인터럽트핀(15) 입력으로 설정

    // 연결확인
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
    
    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // 초기 감도 셋팅
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true); //DMP 활성화

        // 인터럽트 핀과 함수를 연결, 인터럽트 PIN이 LOW -> HIGH로 올라갈 때 dmpDataReady 함수 호출
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));//INTERRUPT_PIN
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = 초기화에러 
        // 2 = DMP 업데이트 에러
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
}
// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {

    if (!dmpReady) return; //초기화 실패 시 함수 종료 
    // wait for MPU interrupt or extra packet(s) available
    
    //mpuInterrupt 변수 기다리다가 인터럽트가 발생하면 다음으로 넘어감
    while (!mpuInterrupt && fifoCount < packetSize) {
        if (mpuInterrupt && fifoCount < packetSize) {
          // try to get out of the infinite loop 
          fifoCount = mpu.getFIFOCount();
        }  
    }
    // reset interrupt flag and get INT_STATUS byte
    //인터럽트 변수 초기화
    mpuInterrupt = false;
    //mpu6050 상태 읽기
    mpuIntStatus = mpu.getIntStatus();

    // FIFO 버퍼 개수 얻기
   fifoCount = mpu.getFIFOCount();
    

    //=========== 대기 -> 초기화 -> 데이터 받아오기 ==========
    if ((mpuIntStatus & (0x01 << MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 16384) {
        mpu.resetFIFO(); //버퍼초기화
        fifoCount = mpu.getFIFOCount();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & (0x01 << MPU6050_INTERRUPT_DMP_INT_BIT)) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // FIFO 에서 데이터 받아오기
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;
  
    #ifdef OUTPUT_READABLE_YAWPITCHROLL       //YPR 값을 얻어서 시리얼에 출력하는 기능
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            // mpu.resetFIFO();
            //Serial.print("ypr\t");
            Serial.print(ypr[0] * 180/M_PI );
            Serial.print("\t");
            Serial.print(ypr[1] * 180/M_PI );
            Serial.print("\t");
            Serial.println(ypr[2] * 180/M_PI);
    #endif
   }
  //값을 JSON 형식으로 가공하는 부분
  //값을 JSON 형식으로 변환 
//             StaticJsonDocument<100> doc;  
//             JsonObject root = doc.to<JsonObject>();
//             root["Type"] = "MPU6050";
//             JsonObject value = root.createNestedObject("value");      
//             value["Yaw"] = ypr[0] * 180/M_PI ;
//             value["Pitch"] = ypr[1] * 180/M_PI;
//             value["Roll"] = ypr[2] * 180/M_PI;

}
