/*
Sends and Reads position of servos in degrees and prints them all.
 */

#include "mbed.h"
#include "platform/mbed_thread.h"
#include "BNO055.h"
#include "CAN3.h"
#include "ServoOut.h"

int myID = 5;

Serial pc(USBTX, USBRX);    //pc serial (tx, rx) uses USB PA_9 and PA_10 on Nucleo D1 and D0 pins
BNO055 bno(D4, D5);
SPI spi(D11, D12, D13);   // mosi, miso, sclk
CAN3 can3(spi, D10, D2);         // spi bus, CS for MCP2515 controller
ServoOut servoOut1(A0);   //A0);     // PA_0 is the servo output pulse

CANMessage canTx_msg;
CANMessage canRx_msg;

Timer t;

void bno_init(void)
{
    if(bno.check()) {
        pc.printf("BNO055 connected\r\n");
        bno.setmode(OPERATION_MODE_CONFIG);
        bno.SetExternalCrystal(1);
        //bno.set_orientation(1);
        bno.setmode(OPERATION_MODE_NDOF);  //Uses magnetometer
        //bno.setmode(OPERATION_MODE_NDOF_FMC_OFF);   //no magnetometer
        bno.set_angle_units(DEGREES);
    } else {
        pc.printf("BNO055 NOT connected\r\n Program Trap.");
        while(1);
    }
}

float unwrap(float previous_angle, float new_angle)
{
    float d = new_angle - previous_angle;
    //d = d > 180 ? d - 2 * 180 : (d < -180 ? d + 2 * 180 : d);
    if (d > 180) {
        d = d - 360;
    } else {
        if (d < -180) {
            d = d + 360;
        }
    }
    return previous_angle + d;
}

int main()
{
    servoOut1.pulseMin = 500;
    servoOut1.pulseMax = 2500;
    thread_sleep_for(500);
    servoOut1 = 500;
    thread_sleep_for(1000);
    servoOut1 = 2500;
    thread_sleep_for(1000);
    servoOut1 = myID*500 - 200;
    thread_sleep_for(1000);

    int initServo[5];
    float currAngles[5] = {-500,-500,-500,-500,-500};
    float yaw, oldYaw;
    float currTime;
    int estimate = 0;
    float estimateYaw;
    int delayCase;
    int bufferSize;
    float yawControl;

    pc.baud(115200);
    pc.printf("Starting Program... \n\r");
    bno_init();
    //can3.reset();            // reset the can bus interface
    can3.frequency(500000);    // set up for 500K baudrate
    char msg_send[8];
    char msg_read_char[8];
    //servoOut1.pulse_us = 1500;
    while(1) {
        pc.printf("Node %d is ready!\n\r", myID);
        bno.get_angles();
        pc.printf("Last BNO Yaw Position: %.2f\n\r", bno.euler.yaw);
        int triggerCode;
        while(1) {
            if(can3.read(&canRx_msg) == CAN_OK) {
                if(canRx_msg.id == 1) {
                    for (int i = 0; i < 8; i++) {
                        msg_read_char[i] = (char)canRx_msg.data[i];
                    }
                    sscanf(msg_read_char, "%d", &triggerCode);
                    if (triggerCode == 2021) {
                        estimate = 0;
                        pc.printf("%d\n\r", triggerCode);
                        pc.printf("Using BNO data...\n\r");
                        break;
                    } 
                    if (triggerCode == 2023) {
                        pc.printf("%d\n\r", triggerCode);
                        estimate = 1;
                        pc.printf("Not using BNO data...\n\r");
                        break;                        
                    }

                }
            }
        }
        pc.printf("Receiving Adjacency Matrix and experiment duration...\n\r");
        int i = 0;
        int adjMatrix[5][5];
        float duration;
        while (i < 6) {
            if(can3.read(&canRx_msg) == CAN_OK) {
                if(canRx_msg.id == 1) {
                    if (i < 5) {
                        for (int j = 0; j < 5; j++) {
                            adjMatrix[i][j] = canRx_msg.data[j] - '0';
                        }

                    } else {
                        pc.printf("Adjacency Matrix received: \n\r");
                        for(int k = 0; k < 5; k++) {
                            for(int m = 0; m < 5; m++) {
                                pc.printf("%d ", adjMatrix[k][m]);
                            }
                            pc.printf("\n\r");
                        }
                        for (int k = 0; k < 8; k++) {
                            msg_read_char[k] = (char)canRx_msg.data[k];
                        }
                        sscanf(msg_read_char, "%f", &duration);
                        pc.printf("Duration of experiment is %.2f seconds.\n\r", duration);
                    }
                }
                i++;
            }
        }
        i = 0;
        while (i < 5) {
            if(can3.read(&canRx_msg) == CAN_OK) {
                if(canRx_msg.id == 1) {
                    for (int k = 0; k < 8; k++) {
                        msg_read_char[k] = (char)canRx_msg.data[k];
                    }
                    sscanf(msg_read_char, "%d", &initServo[i]);
                    pc.printf("Servo %d PCM Position: %d\n\r", i, initServo[i]);
                }
                i++;
            }
        }
        
        i = 0;
        while (i < 1) {
            if(can3.read(&canRx_msg) == CAN_OK) {
                if(canRx_msg.id == 1) {
                    for (int k = 0; k < 8; k++) {
                        msg_read_char[k] = (char)canRx_msg.data[k];
                    }
                    int checkFlag;
                    sscanf(msg_read_char, "%d", &checkFlag);
                    if (checkFlag == 2222){
                        pc.printf("Delay info... \n\r");
                    } else {
                        i--;
                    }
                }
                i++;
            }
        }
        i = 0;
        while (i < 1) {
            if(can3.read(&canRx_msg) == CAN_OK) {
                if(canRx_msg.id == 1) {
                    for (int k = 0; k < 8; k++) {
                        msg_read_char[k] = (char)canRx_msg.data[k];
                    }
                    sscanf(msg_read_char, "%d", &delayCase);
                    pc.printf("Delay Case %d\n\r", delayCase);
                }
                i++;
            }
        }
        i = 0;
        while (i < 1) {
            if(can3.read(&canRx_msg) == CAN_OK) {
                if(canRx_msg.id == 1) {
                    for (int k = 0; k < 8; k++) {
                        msg_read_char[k] = (char)canRx_msg.data[k];
                    }
                    int checkFlag;
                    sscanf(msg_read_char, "%d", &bufferSize);
                    pc.printf("Delay %.1f, Buffer Size %d\n\r", (float)bufferSize/10, bufferSize);
                }
                i++;
            }
        }
        thread_sleep_for(300);


        //NEED to work on control part...


        int controlSignal = initServo[myID-1];
        servoOut1 = controlSignal;
        thread_sleep_for(3000);
        bno.get_angles();
        yaw = bno.euler.yaw;

        //Get initial angles for all
        t.start();
        while (t.read() < 0.5) {
            can3.write(&canTx_msg);
            if(can3.read(&canRx_msg) == CAN_OK) { //if message is available, read into msg
                //pc.printf("CAN RX id=%d data: %s", canRx_msg.id, canRx_msg.data);
                for (int i = 0; i < 8; i++) {
                    msg_read_char[i] = (char)canRx_msg.data[i];
                }
                sscanf(msg_read_char, "%f", &currAngles[canRx_msg.id-1]);
                thread_sleep_for(15-2*myID);  //allows them to use the CAN at different times
                //some wait 13, 11, 9, 7, 5 seconds
            }
        }
        while (t.read() < 1.5) {
            if(can3.read(&canRx_msg) == CAN_OK){
                pc.printf("Cleaning buffer...\n\r");
            }
        }

        float delayedAngles[5][bufferSize];
        for (int i = 0; i < 5; i++){
            for (int j = 0; j < bufferSize; j++){
                if (estimate == 0){
                    delayedAngles[i][j] = yaw;
                } else {
                    delayedAngles[i][j] =180.0/2000*controlSignal - 45;
                }
            }
        }
        //t.stop();
        t.reset();
        //t.start();
        int count = 0;
        while(t.read() < 2.0*duration) {
            //led = !led;
            count++;
            oldYaw = yaw;
            bno.get_angles();
            yaw = unwrap(oldYaw,bno.euler.yaw);
            estimateYaw = 180.0/2000*controlSignal - 45;

            //pc.printf("MyID: %d  Raw Yaw Value: %.2f Unwrapped Yaw %.2f \r\n", myID, bno.euler.yaw, yaw);
            if (estimate == 0) {
                currAngles[myID-1] = yaw;
                sprintf(msg_send, "%.1f\r\n", yaw);
            } else {
                currAngles[myID-1] = estimateYaw;
                sprintf(msg_send, "%.1f\r\n", estimateYaw);
            }
            
            for(int i=0; i<8; i++) {
                canTx_msg.data[i] = msg_send[i];
            }
            canTx_msg.id = myID;

            
            currTime = t.read();
            int writeTimer = currTime + 0.003*(6-myID);
            while (t.read() < currTime + 0.09) {
                if (t.read() >= writeTimer) {
                    can3.write(&canTx_msg);
                    writeTimer = writeTimer + 0.003*(6-myID);
                }
                if(can3.read(&canRx_msg) == CAN_OK) { //if message is available, read into msg
                    //pc.printf("CAN RX id=%d data: %s", canRx_msg.id, canRx_msg.data);
                    for (int i = 0; i < 8; i++) {
                        msg_read_char[i] = (char)canRx_msg.data[i];
                    }
                    sscanf(msg_read_char, "%f", &currAngles[canRx_msg.id-1]);
                    //thread_sleep_for(15-2*myID);  //allows them to use the CAN at different times
                    //some wait 13, 11, 9, 7, 5 seconds
                }
            }

            if (delayCase == 1) {
                yawControl = delayedAngles[myID-1][0];
            } else {
                yawControl = currAngles[myID-1];
            }

            for (int i = 0; i < 5; i++) {
                if (adjMatrix[myID-1][i] > 0) {
                    controlSignal = controlSignal + 0.5*adjMatrix[myID-1][i]*(int)(delayedAngles[i][0]-yawControl);
                }
            }
            for (int i = 0; i < 5; i++){
                for (int j = 0; j < bufferSize-1; j++){
                    delayedAngles[i][j] = delayedAngles[i][j+1];
                }
                delayedAngles[i][bufferSize-1] = currAngles[i];
            }
            if (controlSignal > 2500) {
                controlSignal = 2500;
            } else if (controlSignal < 500) {
                controlSignal = 500;
            }
            servoOut1 = controlSignal;
            if (count == 1) {
                pc.printf("No.\t Time\t Node 1\t Node 2\t Node 3\t Node 4\t Node 5\t PCM\n\r");
            }
            pc.printf("%d \t %.2f \t", count, t.read()/2);
            for(int i = 0; i < 5; i++) {
                pc.printf("%.1f\t", currAngles[i]);
            }
            pc.printf("%d\n\r", controlSignal);
        }//while(1)
        t.stop();
        t.reset();
        servoOut1 = 0;
        //led = 0;
    }
}//main
