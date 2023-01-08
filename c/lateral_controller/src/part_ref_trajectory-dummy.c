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
 * @param Last index from nearest point algorithm
 * @param Last index from part_ref_trajectory
 * @param Shorter Reference X position 1D array [m]
 * @param Shorter Reference Y position 1D array [m]
 * @param Shorter Reference Psi angle 1D array [rad]
 * @param Last index from part_ref_trajectory passing pointer parameter.
 * @param Lenght of the local arrays (always 501, can change on LabVIEW)
 * @return No return
 *
 * @author Aleix Ricou
 */

extern double reftrajectory(double *Xref, double *Yref, double *Psiref, 
int Xarraysize, int Psiarraysize, int closest_point, double last_ids, 
double *local_Xref, double *local_Yref, double *local_Psiref, 
double *last_ids_output, int local_array_length)
{
    local_Xref[2] = Xref[3] + 1; 
    local_Yref[2] = Yref[1] + 1; 
    local_Psiref[2] = Psiref[2] + 1;
    *last_ids_output = 32;
}

