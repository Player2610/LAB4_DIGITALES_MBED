#include "mbed.h"
#include "Adafruit_SSD1306.h"
#include "MPU6050.h"

// ---------------------------
// Pines
// ---------------------------
DigitalOut trig(D2);
InterruptIn echo(D3);

// I2C compartido (asegúrate de que coincida con los pines de tu hardware)
I2C i2c(D14, D15); // SDA, SCL - ajusta según tu placa

// OLED
Adafruit_SSD1306_I2c oled(i2c, D4, 0x78, 64, 128);

// MPU6050 usando la nueva librería
MPU6050 mpu(D14, D15); // SDA, SCL

// ---------------------------
// Objetos y variables
// ---------------------------
Semaphore sem(0);
Queue<float, 5> cola;
Thread hilo_medicion;
Thread hilo_salida;
Timer timer;

// Variables para calibración
float accOffset[3] = {0, 0, 0};
float gyroOffset[3] = {0, 0, 0};
float angle[3] = {0, 0, 0};

// ---------------------------
// Interrupciones ultrasónico
// ---------------------------
void echo_rise() {
    timer.reset();
    timer.start();
}

void echo_fall() {
    timer.stop();
    sem.release();
}

// ---------------------------
// Hilo: medición de distancia
// ---------------------------
void medir_distancia() {
    while (true) {
        trig = 1;
        wait_us(10);
        trig = 0;

        sem.acquire();

        float tiempo_us = chrono::duration_cast<chrono::microseconds>(timer.elapsed_time()).count();
        float distancia_cm = tiempo_us / 58.0f;

        cola.try_put(&distancia_cm);
        ThisThread::sleep_for(500ms);
    }
}

// ---------------------------
// Hilo: mostrar en OLED
// ---------------------------
void mostrar_datos() {
    // Inicializar OLED
    oled.clearDisplay();
    oled.setTextColor(1);
    oled.setTextCursor(0, 0);
    oled.printf("Iniciando...");
    oled.display();
    
    ThisThread::sleep_for(1s);

    // Verificar conexión MPU6050
    if (!mpu.testConnection()) {
        oled.clearDisplay();
        oled.setTextCursor(0, 0);
        oled.printf("Error MPU6050!");
        oled.display();
        printf("Error: MPU6050 no conectado\n");
        while(1);
    }

    // Configurar MPU6050
    mpu.setSleepMode(false);
    mpu.setAcceleroRange(MPU6050_ACCELERO_RANGE_2G);
    mpu.setGyroRange(MPU6050_GYRO_RANGE_250);
    mpu.setBW(MPU6050_BW_256);
    
    // Calibración (toma muestras para offset)
    oled.clearDisplay();
    oled.setTextCursor(0, 0);
    oled.printf("Calibrando...\nNo mover!");
    oled.display();
    printf("Calibrando MPU6050... No mover el sensor!\n");
    
    mpu.getOffset(accOffset, gyroOffset, 100);
    
    printf("Calibracion completa\n");
    printf("Acc Offset: X=%.2f Y=%.2f Z=%.2f\n", accOffset[0], accOffset[1], accOffset[2]);
    printf("Gyro Offset: X=%.2f Y=%.2f Z=%.2f\n", gyroOffset[0], gyroOffset[1], gyroOffset[2]);
    
    ThisThread::sleep_for(1s);

    float accel[3], gyro[3];
    Timer intervalo;
    intervalo.start();

    while (true) {
        osEvent evt = cola.get();
        if (evt.status == osEventMessage) {
            float *dist = (float*)evt.value.p;

            // Leer acelerómetro y giroscopio (valores en m/s² y °/s)
            mpu.getAccelero(accel);
            mpu.getGyro(gyro);

            // Calcular el intervalo de tiempo para integración
            float dt = chrono::duration_cast<chrono::milliseconds>(intervalo.elapsed_time()).count() / 1000.0f;
            intervalo.reset();

            // Computar ángulos con filtro complementario
            mpu.computeAngle(angle, accOffset, gyroOffset, dt);

            // Mostrar en OLED
            oled.clearDisplay();
            oled.setTextCursor(0, 0);
            oled.printf("Dist: %.1f cm\n", *dist);
            oled.printf("Acc(m/s2):\n");
            oled.printf(" X:%.2f Y:%.2f\n", accel[0], accel[1]);
            oled.printf(" Z:%.2f\n", accel[2]);
            oled.printf("Gyro(deg/s):\n");
            oled.printf(" X:%.1f Y:%.1f\n", gyro[0], gyro[1]);
            oled.display();

            // Mostrar en consola serial
            printf("Dist: %.2f cm | ", *dist);
            printf("Acc: X=%.2f Y=%.2f Z=%.2f m/s² | ", accel[0], accel[1], accel[2]);
            printf("Gyro: X=%.2f Y=%.2f Z=%.2f deg/s | ", gyro[0], gyro[1], gyro[2]);
            printf("Ang: Roll=%.1f Pitch=%.1f Yaw=%.1f deg\n", angle[0], angle[1], angle[2]);
        }
        ThisThread::sleep_for(100ms);
    }
}

// ---------------------------
// Main
// ---------------------------
int main() {
    printf("\n=================================\n");
    printf("Sistema MPU6050 + Ultrasonico\n");
    printf("=================================\n");

    // Configurar interrupciones del ultrasonido
    echo.rise(&echo_rise);
    echo.fall(&echo_fall);

    // Iniciar hilos
    hilo_medicion.start(medir_distancia);
    hilo_salida.start(mostrar_datos);

    // Main no hace nada más, los hilos manejan todo
    while(1) {
        ThisThread::sleep_for(1s);
    }
}