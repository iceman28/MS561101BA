#include <stdio.h>
#include <unistd.h>
#include "MS561101BA.h"

MS561101BA Baro;

int main(){
  Baro.init();
  Baro.setRefPressure (1009.20);
  while (1) {
    Baro.readData(4096);
    while (!Baro.dataAvailable());
    printf("Temperatura:%5.2f ºC  Presión:%5.2f mbar  Altitud:%5.2f m \n", pres.temperature, pres.pressure, pres.altitude);
    usleep(500000);
  }
}