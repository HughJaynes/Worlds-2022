#include "main.h"

void liftControl (void* lPosPointer) {
	Motor LI (LIPORT);
    LI.tare_position();

    while (true) {
        LI.move(*(int *)lPosPointer - (LI.get_position()) * LIFTKP);
        delay(5);
    }
}