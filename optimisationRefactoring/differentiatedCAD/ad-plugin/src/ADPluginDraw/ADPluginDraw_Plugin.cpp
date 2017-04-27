//-----------------------------------------------------------------------------
// Created on: 06 November 2015
// Created by: Sergey SLYADNEV
//-----------------------------------------------------------------------------

// ADPluginDraw includes
#include <ADPluginDraw_Plugin.h>

// OCCT includes
#include <Draw_Main.hxx>
#include <Draw_PluginMacro.hxx>
#include <OSD_Environment.hxx>

//! Returns path to the data files.
//! \return path to the data files.
TCollection_AsciiString ADPluginDraw_Plugin::DataPath()
{
  return OSD_Environment("AD_TEST_DATA").Value();
}

//! Entry point to AD plug-in for Draw. This is a right place
//! to register commands groups and activate license keys for value-added
//! components.
//! \param di [in] Draw interpreter instance.
void ADPluginDraw_Plugin::Factory(Draw_Interpretor& di)
{
  // Load command suites
  CommandsCurve  (di);
  CommandsSurface(di);
}

DPLUGIN(ADPluginDraw_Plugin)
