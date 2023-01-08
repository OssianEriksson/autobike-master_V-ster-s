#include <stdio.h>
#include <stdbool.h>

#include "../../configuration/configuration.h"


/**
 * Dummy code for lateral controller implementation testing
 *
 * @param Reference X position 1D array [m]
 * @param Reference Y position 1D array [m]
 * @param Reference Psi angle 1D array [rad]
 * @param Lenght of Xref and Yref arrays
 * @param Lenght of Psiref array
 * @param Estimated X position [m]
 * @param Estimated Y position [m]
 * @param Estimated Psi angle [rad]
 * @param K1 controller parameter
 * @param K2 controller parameter
 * @param e1_max controller parameter
 * @param idx passing pointer parameter. Last index from nearest point algorithm
 * @return Reference steer angle [rad]
 *
 * @author Aleix Ricou
 */

extern double lateralcontroller(double *Xref, double *Yref, double *Psiref, 
int Xarraysize, int Psiarraysize, double Xest, double Yest, double Psiest, 
double K1, double K2, double e1_max, int *idx)
{
    double referencesteer = Xref[2] + Yref[2] + Psiref[2] + Xest + Yest + 
Psiest - K1 - K2 - e1_max;
    *idx = Xarraysize;
    return referencesteer;
}

