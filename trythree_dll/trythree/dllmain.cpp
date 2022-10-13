// dllmain.cpp : Define el punto de entrada de la aplicación DLL.
#include "pch.h"
#include "trythree.h"

BOOL APIENTRY DllMain( HMODULE hModule,
                       DWORD  ul_reason_for_call,
                       LPVOID lpReserved
                     )
{
    switch (ul_reason_for_call)
    {
    case DLL_PROCESS_ATTACH:
    case DLL_THREAD_ATTACH:
    case DLL_THREAD_DETACH:
    case DLL_PROCESS_DETACH:
        break;
    }
    return TRUE;
}


double integral = 0;
double sumar = 1;


extern double letstry(double heythere) {

    integral = heythere + sumar;

    return integral;
}