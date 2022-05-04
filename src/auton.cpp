#include "main.h"

int afterBackIntake = 500;

void afterBackIntakeMinus () {
    afterBackIntake -= 10;
}

void afterBackIntakePlus () {
    afterBackIntake += 10;
}

void autonCode() {
    changeTilter();
    delay(afterBackIntake);
    moveBase(-650, -650, 3000, 0.2, 0.05, 90);
    delay(3000);
    changeTilter();
    /*
    delay(400);
    changeLiftUp();
    changeLiftUp();
    changeRingOnOff();
    rotateBase(295, 750);
    delay(750);
    moveBase(400, 400, 1300, 0.2, 0.2, 37);
    delay(1300);
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
    */
}