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
DigitalInOut pin8(D8);
InterruptIn btn(BUTTON1);

/** erpc infrastructure */
ep::UARTTransport uart_transport(D1, D0, 9600);
ep::DynamicMessageBufferFactory dynamic_mbf;
erpc::BasicCodecFactory basic_cf;
erpc::Crc16 crc16;
erpc::SimpleServer rpc_server;

/** car service */
BBCarService_service car_control_service;

/****** erpc declarations *******/

bool tasking = false;

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
    car.stop();
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

/*****************************/

void turnDegree(double speed, double factor, int degree){
    if (factor < 0){
        int x = car.servo0.angle;
        int target_ang = x - 22 * degree / 6.5;
        printf("%d, %d\n", x, target_ang);
        if (degree == 0)
            return;
        while (car.servo0.angle > target_ang) {
            car.turn(speed, -0.1);
            printf("%d\n", car.servo0.angle);
            ThisThread::sleep_for(100ms);
        }
    }else {
        int x = car.servo1.angle;
        int target_ang = x + 22 * degree / 6.5;
        printf("%d, %d\n", x, target_ang);
        if (degree == 0)
            return;
        while (car.servo1.angle < target_ang) {
            car.turn(speed, 0.1);
            printf("%d\n", car.servo1.angle);
            ThisThread::sleep_for(100ms);
        }
    }
    
}


void QTI(){
    car.stop();
    parallax_qti qti1(qti_pin);
    int pattern = 0b1111;
    int pre_pattern = 0b1111;
    startAngle0 = car.servo0.angle;
    startAngle1 = car.servo1.angle;
    int cnt = 0;
    while (true) {
        
        if (tasking){
            pattern = (int)qti1;
            //printf("%d%d%d%d\n",pattern/8, pattern%8/4, pattern%4/2, pattern%2);
            pre_pattern = pattern;
            if (pattern != 0b1001)
                cnt = 0;
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
                case 0b1111: 
                    car.stop();
                    ThisThread::sleep_for(500ms);
                    car.turn(-50, -0.1); 
                    ThisThread::sleep_for(1s);
                    cnt = 0; 
                    break;
                case 0b0000: 
                    if (pre_pattern != 0b0110){
                        car.goStraight(50); 
                    }
                    break;
                case 0b1001: 
                    cnt++;
                    if (cnt > 5){
                        car.stop(); 
                        tasking = false;
                        cnt = 0;
                    }
                    break;
                case 0b1010: 
                    car.stop();
                    ThisThread::sleep_for(500ms);
                    car.goCertainDistance(-2.5);
                    while (car.checkDistance(1)){
                        car.goStraight(-50);
                        ThisThread::sleep_for(100ms);
                    }
                    car.stop();
                    ThisThread::sleep_for(500ms);
                    turnDegree(-50, -0.1, 90); 
                    break;
                case 0b0101: 
                    car.stop();
                    ThisThread::sleep_for(500ms);
                    car.goCertainDistance(-2.5);
                    while (car.checkDistance(1)){
                        car.goStraight(-50);
                        ThisThread::sleep_for(100ms);
                    }
                    car.stop();
                    ThisThread::sleep_for(500ms);
                    turnDegree(-50, 0.1, 90);
                    break;
                case 0b1101: 
                    car.goStraight(-50);
                    ThisThread::sleep_for(1500ms); 
                    break;
                default: car.stop(); cnt = 0; 
            }
        }

        ThisThread::sleep_for(3ms);
    }
}

double data[1000] = {0};
int angles[1000] = {0};
parallax_laserping ping1(pin8);

int scan() {
    int cnt = 0;
    data[cnt] = (float)ping1;
    angles[cnt++] = car.servo0.angle;
    car.spin(-50);
    ThisThread::sleep_for(2s);
    car.stop();
    ThisThread::sleep_for(1s);
    ThisThread::sleep_for(100ms);
    car.spin(50);
    for (int i = 0; i < 50; i++){
        data[cnt] = (float)ping1;
        angles[cnt++] = car.servo0.angle;
        ThisThread::sleep_for(50ms);
    }
    car.stop();
    for (int i = 0; i < 60; i++){
        data[cnt] = (float)ping1;
        angles[cnt++] = car.servo0.angle;
        ThisThread::sleep_for(50ms);
    }
    car.spin(-50);
    ThisThread::sleep_for(2s);
    car.stop();

    int prev_ang = 0;
    int barrier_cnt = 0;
    int split = 0;
    printf("%lf\n", data[0]);
    for (int i = 0; i < 1000; i++){
        if (data[i] == 0)
            break;
        if (abs(angles[i]) - abs(prev_ang)){
            //printf("%d:%d \t", i, abs(angles[i]) - abs(prev_ang));
            int n = data[i] / 10;
            for (int j = 0; j < n; j++){
                printf("*");
            }
            printf("\n");
            prev_ang = angles[i];
            if (data[i] < 180){
                if (data[i] <= 20){
                    barrier_cnt++;
                }else{
                    if (barrier_cnt >= 2){
                        split++;
                    }
                    barrier_cnt = 0;
                }
            }
            
        }
    }
    printf("%d\n", split);
    return split;
}

Thread LaserPing_thread;

void LaserPing(){
    while (true){
        if (tasking){
            double distance = (float)ping1;
            printf("%lf\n", distance);
            if (distance < 10) {
                tasking = false;
                car.stop();
                ThisThread::sleep_for(500ms);
                int split = scan();
                if (split == 2){
                    turnDegree(-50, -0.1, 270);
                    car.stop();
                    ThisThread::sleep_for(500ms);
                    car.goStraight(-50);
                    ThisThread::sleep_for(1s);
                }else if (split == 1){
                    car.spin(-100);
                    int goal_ang = car.servo0.angle - 11 / 6.5 * 5.5 *11;
                    while (car.servo0.angle > goal_ang){
                        ThisThread::sleep_for(100ms);
                    }
                    car.goStraight(50);
                    ThisThread::sleep_for(500ms);
                }
                
                car.stop();
                ThisThread::sleep_for(500ms);
                tasking = true;
            }
        }
        ThisThread::sleep_for(10ms);
    }
}


int main() {

    btn.rise(&start);

    // Initialize the rpc server
    uart_transport.setCrc16(&crc16);

    printf("Initializing server.\n");
    rpc_server.setTransport(&uart_transport);
    rpc_server.setCodecFactory(&basic_cf);
    rpc_server.setMessageBufferFactory(&dynamic_mbf);

    // Add the led service to the server
    printf("Adding BBCar server.\n");
    rpc_server.addService(&car_control_service);
    
    QTI_thread.start(QTI);
    LaserPing_thread.start(LaserPing);

    // Run the server. This should never exit
    printf("Running server.\n");
    rpc_server.run();
    
    
}