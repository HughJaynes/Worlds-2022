#include "main.h"

void autonCode() {
    Motor LI (LIPORT);
    LI.tare_position();
    changeTilter();
    delay(3000);
    moveBase(-650, -650, 3000, 0.2, 0.1, 85);
    delay(3000);
    changeTilter();
    changeLiftUp();
    changeLiftUp();
    delay(200);
    changeRingOnOff();
    rotateBase(-60, 2000);
    delay(2000);
    moveBase(600, 600, 2000, 0.15, 0.25, 37);
    delay(3000);
    moveBase(-500,-100,2000);
    delay(2000);
    changeLiftDown();
    changeLiftDown();
    changeClamp();
    moveBase(-300,-300,2000);
    delay(2000);
    changeTilter();
    delay(250);
    rotateBase(135,2000);
    delay(2000);
    /*
    moveBase(-100, -200, 750, 0.4, 0.2, 110);
    delay(750);
    rotateBase(90, 800, 1, 2);
    delay(800);
    moveBase(350, 350, 750, 0.5, 0.05, 127);
    delay(750);
    rotateBase(135, 750, 1, 2);
    delay(750);
    changeTilter();
    changeLiftDown();
    changeLiftDown();
    changeClamp();
    delay(250);
    moveBase(500, 500, 750, 0.4, 0.2, 120);
    delay(750);
    rotateBase(170,1000,1,2);
    delay(1000);
    moveBase(675, 675, 1500);
    delay(1500);
    changeClamp();
    rotateBase(10, 500);
    delay(1000);
    changeRingOnOff();
    */
}
