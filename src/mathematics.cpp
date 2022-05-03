#include "main.h"

// int max (int a, int b) {
//     if (a > b) {
//         return a;
//     }
// }

double abscap(double x, double abscap){
    if(x > abscap) return abscap;
  else if(x < -abscap) return -abscap;
  else return x;
}