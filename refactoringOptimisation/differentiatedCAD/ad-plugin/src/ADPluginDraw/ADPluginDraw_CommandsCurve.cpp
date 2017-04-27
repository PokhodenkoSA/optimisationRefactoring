//-----------------------------------------------------------------------------
// Created on: 06 November 2015
// Created by: Sergey SLYADNEV
//-----------------------------------------------------------------------------

// ADPluginDraw includes
#include <ADPluginDraw_Plugin.h>
#include <ADPluginDraw_DrawTestSuite.h>

// OCCT includes
#include <DrawTrSurf_BSplineCurve2d_AD.hxx>
#include <Geom_Curve.hxx>
#include <Geom2d_BSplineCurve.hxx>
#include <Geom2d_BSplineCurve_AD.hxx>
#include <Geom2d_Curve.hxx>
#include <GeomLib.hxx>
#include <gp_Ax2.hxx>
#include <TColStd_Array1OfInteger.hxx>
#include <TColStd_HSequenceOfReal.hxx>

//! AD sample namespace.
namespace AD
{
  //! Point in a space of weights.
  typedef adouble t_weight;

  Handle(Geom2d_BSplineCurve_AD) c, g;
  Handle(TColStd_HSequenceOfReal) params;

  //! Distance between two curves evaluated at given parameters.
  adouble Dist(const int w_idx, const t_weight& w)
  {
    if ( params.IsNull() )
      Standard_ProgramError::Raise("Parameters are not available");

    g->SetWeight(w_idx, w); // Update weight

    // Calculate total squared distance
    adouble squares = 0.0;
    for ( int i = 1; i <= params->Length(); ++i )
    {
      const double u = params->Value(i);

      gp_Pnt2d_AD c_AP;
      c->D0_AD( u, c_AP );

      gp_Pnt2d_AD g_AP;
      g->D0_AD( u, g_AP );

      adouble d = ( c_AP.X() - g_AP.X() ) * ( c_AP.X() - g_AP.X() ) + ( c_AP.Y() - g_AP.Y() ) * ( c_AP.Y() - g_AP.Y() );
      squares += d;
    }
    return squares;
  }
};

//! Type alias for the objective function to optimize.
typedef adouble (*OptFunction)(const int w_idx, const AD::t_weight& w);

//! Armijo rule for adaptive selection of step length in inexact line search.
class Opt_ArmijoRule
{
public:

  //! Search parameters.
  struct t_search_params
  {
    double       max_alpha;      //!< Right bound.
    int          num_iterations; //!< Max number of iterations.
    int          w_idx;          //!< Index of weight.
    AD::t_weight x_k;            //!< Position in the search space.
    AD::t_weight d_k;            //!< Direction of line search.
    AD::t_weight gradient;       //!< Calculated gradient.
    OptFunction  pFunc;          //!< Target function.

    t_search_params() : max_alpha      (0.0),
                        num_iterations (0),
                        w_idx          (0),
                        pFunc          (NULL)
    {}
  };

public:

  Opt_ArmijoRule() {}

public:

  //! Runs Armijo rule.
  //! \param params    [in]  search parameters.
  //! \param num_iters [out] consumed number of iterations.
  //! \param alpha     [out] resulting step size.
  //! \return true in case of success, false -- otherwise.
  bool Perform(const t_search_params& params,
               int&                   num_iters,
               double&                alpha)
  {
    num_iters = 0;
    alpha     = params.max_alpha;

    const double beta      = 0.2; // Interval reduction coefficient
    const double mu        = 1.0e-2;
    const double min_alpha = 1.0e-8;
    const double gradient  = params.gradient.getValue();
    const double f         = (*params.pFunc)(params.w_idx, params.x_k).getValue();
    const double phi_deriv = gradient;

    // Initial alpha
    bool isSolved = false;
    bool doStop   = false;

    // Main iterations
    do
    {
      ++num_iters;
      if ( num_iters < params.num_iterations )
      {
        double barrier = alpha*mu*phi_deriv;
        adouble new_x  = params.x_k + params.d_k*alpha;
        if ( new_x <= 0 )
        {
          alpha *= beta;

          if ( alpha < min_alpha )
          {
            doStop   = true;
            isSolved = false;
          }
          continue;
        }

        double phi = ((*params.pFunc)(params.w_idx, new_x) - f).getValue();

        if ( phi < barrier )
        {
          doStop   = true;
          isSolved = true;
        }
        else
        {
          alpha *= beta;

          if ( alpha < min_alpha )
          {
            doStop   = true;
            isSolved = false;
          }
        }
      }
      else
      {
        doStop   = true;
        isSolved = false;
      }
    }
    while ( !doStop );

    return isSolved;
  }

};

//! Test command.
//! \param di   [in] Draw Interpreter.
//! \param argc [in] number of arguments.
//! \param argv [in] arguments.
//! \return result status.
static int ADC_Test(Draw_Interpretor& di, int argc, const char** argv)
{
  /* ======================
   *  Prepare input curves
   * ====================== */

  if ( argc != 3 )
  {
    di << "Error: incorrect number of arguments\n";
    return 1;
  }

  Handle(Geom2d_Curve) C1 = DrawTrSurf::GetCurve2d( argv[1] );
  Handle(Geom2d_Curve) C2 = DrawTrSurf::GetCurve2d( argv[2] );

  AD::c = Handle(Geom2d_BSplineCurve_AD)::DownCast(C1);
  AD::g = Handle(Geom2d_BSplineCurve_AD)::DownCast(C2);

  if ( AD::c.IsNull() || AD::g.IsNull() )
  {
    di << "Error: one or both curves are not 2d splines\n";
    return 1;
  }

  // Perform discretization
  const int npts = 50;
  AD::params = new TColStd_HSequenceOfReal;
  const double range_c = ( AD::c->LastParameter() - AD::c->FirstParameter() );
  const double range_c_delta = range_c / npts;
  for ( int i = 0; i <= npts; ++i )
  {
    double param = AD::c->FirstParameter() + i*range_c_delta;
    if ( param > AD::c->LastParameter() )
      param = AD::c->LastParameter();

    AD::params->Append(param);
  }

  /* ============================================
   *  Perform optimization with steepest descent
   * ============================================ */

  const int    W_IDX          = 3;
  const double FD_DELTA       = 0.01;
  const double derivativeSeed = 1.;
  const double def_step       = 0.001;

  AD::t_weight x = AD::g->Weight(W_IDX);
  adouble y;

  // Sample
  x.setADValue(&derivativeSeed);

  const double prec = 1.0e-2;

  // Prepare Armijo rule
  Opt_ArmijoRule Armijo;
  Opt_ArmijoRule::t_search_params Armijo_params;
  Armijo_params.max_alpha      = 1.0;
  Armijo_params.num_iterations = 100;
  Armijo_params.w_idx          = W_IDX;
  Armijo_params.pFunc          = AD::Dist;

  // Loop for steepest descent
  const int maxNumberOfIterations = 100;
  int numberOfIterations = 0;
  while(1)
  {
    // Calculate gradient

    y = AD::Dist(W_IDX, x);
    double grad_y = *y.getADValue();

    // Calculate finite difference using middle scheme of differentiation
    double x_prev = x.getValue() - FD_DELTA;
    double x_next = x.getValue() + FD_DELTA;
    double finite_grad_y = ( ( AD::Dist(W_IDX, x_next) - AD::Dist(W_IDX, x_prev) ) / (2*FD_DELTA) ).getValue();

    // Dump some stuff explaining current iteration
    di << "\n---\n";
    di << "y = " << y.getValue() << "\n";
    di << "dy / dw = " << grad_y << "\n";
    di << "finite dy / dw = " << finite_grad_y << "\n";

    // Check stopping criterion
    if ( fabs(grad_y) < prec )
      break;

    // Configure Armijo rule
    Armijo_params.x_k      = x;
    Armijo_params.d_k      = -grad_y;
    Armijo_params.gradient = grad_y;
    int num_armijo_iters = 0;

    // Choose step adaptively
    double actual_step;
    if ( !Armijo.Perform(Armijo_params, num_armijo_iters, actual_step) )
      actual_step = def_step;

    di << "step = " << actual_step << "\n";

    // Steepest descent
    x -= actual_step * grad_y;
    if ( x <= 0 )
      x = 0.01;

    // Check to avoid infinite loops
    numberOfIterations++;
    if ( numberOfIterations > maxNumberOfIterations )
    {
      di << "Error: max number of iterations exceeded\n";
      break;
    }

    // Update axonometry
    //std::string cmd_c("display "); cmd_c += argv[1];
    //std::string cmd_g("display "); cmd_g += argv[2];
    //di.Eval("2dclear");
    //di.Eval( cmd_c.c_str() );
    //di.Eval( cmd_g.c_str() );
  }
  di << "Total number of iterations: " << numberOfIterations << "\n";
  return 0;
}

//! Command to create a differentiable B-curve with two-dimensional poles.
static Standard_Integer AD_BCurve2d( Draw_Interpretor& di,
                                     Standard_Integer n, const char** a )
{
  Standard_Integer k,i;

  Standard_Integer deg = Draw::Atoi(a[2]);
  Standard_Integer nbk = Draw::Atoi(a[3]);

  TColStd_Array1OfReal    knots(1, nbk);
  TColStd_Array1OfInteger mults(1, nbk);
  k = 4;
  Standard_Integer Sigma = 0;
  for (i = 1; i<=nbk; i++) {
    knots( i) = Draw::Atof(a[k]);
    k++;
    mults( i) = Draw::Atoi(a[k]);
    Sigma += mults(i);
    k++;
  }

  Standard_Boolean periodic = 0;
  Standard_Integer np;
  if (periodic)
    np = Sigma - mults(nbk);
  else
    np = Sigma - deg  -1;

  TColgp_Array1OfPnt2d poles  (1, np);
  TColStd_Array1OfReal weights(1, np);
    
  for (i = 1; i <= np; i++) {
    poles(i).SetCoord(Draw::Atof(a[k]),Draw::Atof(a[k+1]));
    k += 2;
    weights(i) = Draw::Atof(a[k]);
    k++;
  }

  Handle(Geom2d_BSplineCurve_AD) result =
    new Geom2d_BSplineCurve_AD(poles, weights, knots, mults, deg, periodic);

  Handle(DrawTrSurf_BSplineCurve2d_AD) dc = new DrawTrSurf_BSplineCurve2d_AD(result);
  Draw::Set(a[1], dc);

  return 0;
}

//! Maps curve to 3d space.
//! \param di   [in] Draw Interpreter.
//! \param argc [in] number of arguments.
//! \param argv [in] arguments.
//! \return result status.
static int ADC_To3d(Draw_Interpretor& di, int argc, const char** argv)
{
  if ( argc != 2 )
  {
    di << "Error: invalid number of arguments (check help for details)\n";
    return 1;
  }

  Handle(Geom2d_Curve) C2D = DrawTrSurf::GetCurve2d(argv[1]);
  if ( C2D.IsNull() )
  {
    di << "Error: there is no 2d curve with name " << argv[1] << "\n";
    return 1;
  }

  Handle(Geom2d_BSplineCurve) BCurve2D;
  if ( C2D->IsKind( STANDARD_TYPE(Geom2d_BSplineCurve_AD) ) )
  {
    Handle(Geom2d_BSplineCurve_AD) C2D_AD = Handle(Geom2d_BSplineCurve_AD)::DownCast(C2D);

    // Convert poles
    Handle(TColgp_HArray1OfPnt2dAdouble) AD_poles = C2D_AD->Poles();
    TColgp_Array1OfPnt2d poles( AD_poles->Lower(), AD_poles->Upper() );
    for ( int i = poles.Lower(); i <= poles.Upper(); ++i )
    {
      poles(i) = gp_Pnt2d( AD_poles->Value(i).X().getValue(), AD_poles->Value(i).Y().getValue() );
    }

    // Convert weights
    TColStd_Array1OfReal weights( AD_poles->Lower(), AD_poles->Upper() );
    for ( int i = weights.Lower(); i <= weights.Upper(); ++i )
    {
      const double w = C2D_AD->Weight(i).getValue();
      weights(i) = w;
    }

    BCurve2D = new Geom2d_BSplineCurve(poles, weights, C2D_AD->Knots(), C2D_AD->Multiplicities(), C2D_AD->Degree(), 0);
  }
  else
    BCurve2D = Handle(Geom2d_BSplineCurve)::DownCast(C2D);

  if ( BCurve2D.IsNull() )
  {
    di << "Error: 2d curve is not a b-curve\n";
    return 1;
  }

  gp_Ax2 xoy( gp::Origin(), gp::DZ() );
  Handle(Geom_Curve) C3D = GeomLib::To3d(xoy, BCurve2D);

  TCollection_AsciiString name(argv[1]); name += "_3d";
  DrawTrSurf::Set(name.ToCString(), C3D);

  return 0;
}

//-----------------------------------------------------------------------------
// Registration
//-----------------------------------------------------------------------------

//! Registers all ADE commands related to differentiation of curves.
//! \param di [in] Draw interpreter to register commands in.
void ADPluginDraw_Plugin::CommandsCurve(Draw_Interpretor& di)
{
  const char *grp = "CommandsCurve";

  di.Add("adc_bcurve2d", "adc_bcurve2d curve_name ...", __FILE__, AD_BCurve2d, grp);
  di.Add("adc_test",     "adc_test",                    __FILE__, ADC_Test,    grp);
  di.Add("adc_to3d",     "adc_to3d curve2d",            __FILE__, ADC_To3d,    grp);
}
