#include "mbed.h"
#include "drivers/DigitalOut.h"

#include "erpc_simple_server.hpp"
#include "erpc_basic_codec.hpp"
#include "erpc_crc16.hpp"
#include "UARTTransport.h"
#include "DynamicMessageBufferFactory.h"
#include "bbcar_control_server.h"
// Uncomment for actual BB Car operations
#include "bbcar.h"
#include <cstdint>

Ticker servo_ticker;
Ticker servo_feedback_ticker;

PwmIn servo0_f(D10), servo1_f(D12);
PwmOut servo0_c(D11), servo1_c(D13);
BBCar car(servo0_c, servo0_f, servo1_c, servo1_f, servo_ticker, servo_feedback_ticker);

BusInOut qti_pin(D4,D5,D6,D7);

/** erpc infrastructure */
ep::UARTTransport uart_transport(D1, D0, 9600);
ep::DynamicMessageBufferFactory dynamic_mbf;
erpc::BasicCodecFactory basic_cf;
erpc::Crc16 crc16;
erpc::SimpleServer rpc_server;

/** car service */
BBCarService_service car_control_service;

/****** erpc declarations *******/

bool tasking = true;

void stop(){
    car.stop();
    printf("Car stop.\n");
    tasking = false;
}

void goStraight(int32_t speed){
    car.goStraight(-speed);
    printf("Car go straight at speed %d.\n", speed);
    tasking = false;
}

void turn(int32_t speed, double factor){
    car.turn(-speed, factor);
    printf("Car turn at speed %d with a factor of %f.\n", speed, factor);
    tasking = false;
}

void spin(int32_t speed){
    car.spin(-speed);
    printf("Car spin at speed %d.\n", speed);
    tasking = false;
}


double distance = 0;
void QTI();
Thread QTI_thread;

void start(){
    tasking = !tasking;
    QTI_thread.start(QTI);
}

int startAngle0 = car.servo0.angle;
int startAngle1 = car.servo1.angle;

double getDistance(){
    double distance = 0;
    int endAngle0 = car.servo0.angle;
    int endAngle1 = car.servo1.angle;
    distance = -(double)(endAngle0 - startAngle0 + endAngle1 - startAngle1) / 2 * 6.5 * 3.14159 / 360;
    return distance;
}

double getSpeed(){
    int prevAngle0 = car.servo0.angle;
    int prevAngle1 = car.servo1.angle;
    ThisThread::sleep_for(500ms);
    double speed = -(double)(car.servo0.angle - prevAngle0 + car.servo1.angle - prevAngle1) / 2 * 6.5 * 3.14159 / 360 / 0.5;
    return speed;

}

void QTI(){
    car.stop();
    parallax_qti qti1(qti_pin);
    int pattern = 0b1111;
    startAngle0 = car.servo0.angle;
    startAngle1 = car.servo1.angle;
    while (true) {
        
        if (tasking){
            pattern = (int)qti1;
            printf("%d%d%d%d\n",pattern/8, pattern%8/4, pattern%4/2, pattern%2);
            switch (pattern) {
                case 0b0110: car.goStraight(-50); break;
                case 0b1000: car.turn(-50, -0.1); break;
                case 0b1100: car.turn(-50, -0.1); break;
                case 0b1110: car.turn(-50, -0.1); break;
                case 0b0100: car.turn(-50, -0.5); break;
                case 0b0010: car.turn(-50, 0.5); break;
                case 0b0011: car.turn(-50, 0.1); break;
                case 0b0111: car.turn(-50, 0.1); break;
                case 0b0001: car.turn(-50, 0.1); break;
                case 0b1111: car.goStraight(-50); break;
                case 0b0000: car.goStraight(50); break;
                default: car.stop();
            }
        }
        ThisThread::sleep_for(3ms);
    }
}


int main() {

    // Initialize the rpc server
    uart_transport.setCrc16(&crc16);

    printf("Initializing server.\n");
    rpc_server.setTransport(&uart_transport);
    rpc_server.setCodecFactory(&basic_cf);
    rpc_server.setMessageBufferFactory(&dynamic_mbf);

    // Add the led service to the server
    printf("Adding BBCar server.\n");
    rpc_server.addService(&car_control_service);
    
    

    // Run the server. This should never exit
    printf("Running server.\n");
    rpc_server.run();
    
    
}