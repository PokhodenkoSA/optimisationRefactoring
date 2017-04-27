//-----------------------------------------------------------------------------
// Created on: 06 November 2015
// Created by: Sergey SLYADNEV
//-----------------------------------------------------------------------------

#ifndef ADPluginDraw_Plugin_HeaderFile
#define ADPluginDraw_Plugin_HeaderFile

// ADPluginDraw includes
#include <ADPluginDraw.h>

// OCCT includes
#include <Draw_Interpretor.hxx>

//! Draw plug-in providing custom commands for functional testing of
//! AD functionality.
class ADPluginDraw_Plugin
{
// Auxiliary functions:
public:

  static TCollection_AsciiString DataPath();

// Plug-in interface methods:
public:

  ADPluginDraw_EXPORT static void Factory(Draw_Interpretor&);

// Test commands for AD:
public:

  ADPluginDraw_EXPORT static void CommandsCurve   (Draw_Interpretor&);
  ADPluginDraw_EXPORT static void CommandsSurface (Draw_Interpretor&);

};

#endif
