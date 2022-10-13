#pragma once

#ifdef TRYTHREE_EXPORTS
#define TRYTHREE_API __declspec(dllexport)
#else
#define TRYTHREE_API __declspec(dllimport)
#endif


extern "C" TRYTHREE_API extern double letstry(double heythere);
