//-----------------------------------------------------------------------------
// Created on: 06 November 2015
// Created by: Sergey SLYADNEV
//-----------------------------------------------------------------------------

#ifndef ADPluginDraw_HeaderFile
#define ADPluginDraw_HeaderFile

#if defined _WIN32
  #if defined ADPluginDraw_LIB
    #define ADPluginDraw_EXPORT __declspec(dllexport)
  #else
    #define ADPluginDraw_EXPORT __declspec(dllimport)
  #endif
#else
  #define ADPluginDraw_EXPORT
#endif

#define ADPluginDraw_NotUsed(x)

#endif
