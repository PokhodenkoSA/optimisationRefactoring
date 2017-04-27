//-----------------------------------------------------------------------------
// Created on: 06 November 2015
// Created by: Sergey SLYADNEV
//-----------------------------------------------------------------------------

// ADPluginDraw includes
#include <ADPluginDraw_Plugin.h>
#include <ADPluginDraw_DrawTestSuite.h>

//! Test command.
//! \param di   [in] Draw Interpreter.
//! \param argc [in] number of arguments.
//! \param argv [in] arguments.
//! \return result status.
static int ADS_Test(Draw_Interpretor& /*di*/, int /*argc*/, const char** /*argv*/)
{
  std::cout << "Test command" << std::endl;

  return 0;
}

//-----------------------------------------------------------------------------
// Registration
//-----------------------------------------------------------------------------

//! Registers all ADE commands related to differentiation of surfaces.
//! \param di [in] Draw interpreter to register commands in.
void ADPluginDraw_Plugin::CommandsSurface(Draw_Interpretor& di)
{
  const char *grp = "CommandsSurface";

  di.Add("ads_test", "ads_test", __FILE__, ADS_Test, grp);
}
