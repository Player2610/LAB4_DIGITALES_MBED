#include "mbed.h"
#include "Adafruit_SSD1306.h"
#include <cstdio>


// Pines
DigitalOut trig(D2);
InterruptIn echo(D3);

I2C i2c(D14, D15);
Adafruit_SSD1306_I2c oled(i2c, D4, 0x78, 64, 128);


// RTOS
Semaphore sem(0);
Queue<float, 5> cola;
Thread hilo_medicion;
Thread hilo_salida;

// Temporizador
Timer timer;
    
void echo_isr_rise() {
    timer.reset();
    timer.start();
}

void echo_isr_fall() {
    timer.stop();
    sem.release(); // Despierta al hilo de medición
}

void medir_distancia() {
    while (true) {
        // Pulso de disparo
        trig = 1;
        wait_us(10);
        trig = 0;

        // Espera a que la ISR libere el semáforo
        sem.acquire();

        // Calcula duración del pulso
        float tiempo_us = chrono::duration_cast<chrono::microseconds>(timer.elapsed_time()).count();
        float distancia_cm = tiempo_us / 58.0f;

        // Envía a la cola
        cola.try_put(&distancia_cm);

        ThisThread::sleep_for(500ms);
    }
}

void mostrar_distancia() {
    while (true) {
        
        
        osEvent evt = cola.get();
        if (evt.status == osEventMessage) {
            float *d = (float*)evt.value.p;
            printf("Distancia: %.2f cm\n", *d);

        }
    }
}

int main() {
    echo.rise(&echo_isr_rise);
    echo.fall(&echo_isr_fall);

    hilo_medicion.start(medir_distancia);
    hilo_salida.start(mostrar_distancia);
}
