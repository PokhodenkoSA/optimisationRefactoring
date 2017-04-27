#ifndef Draw_View_Header
#define Draw_View_Header

#include <gp_Trsf.hxx>
#include <Draw_Window.hxx>

class Draw_Viewer;

class Draw_View : public Draw_Window
{
public:

  //! Constructor
  Draw_View(Standard_Integer theId,
            Draw_Viewer*     theViewer,
            Standard_Integer theX,
            Standard_Integer theY,
            Standard_Integer theWidth,
            Standard_Integer theHeight);

#if defined(_WIN32) || defined(__WIN32__)
  Draw_View(Standard_Integer theId,
            Draw_Viewer*     theViewer,
            Standard_Integer theX,
            Standard_Integer theY,
            Standard_Integer theWidth,
            Standard_Integer theHeight,
            HWND             theWindow);
#elif defined(__APPLE__) && !defined(MACOSX_USE_GLX)
  Draw_View(Standard_Integer theId,
            Draw_Viewer*     theViewer,
            Standard_Integer theX,
            Standard_Integer theY,
            Standard_Integer theWidth,
            Standard_Integer theHeight,
            NSWindow*        theWindow);
#endif

  //! Constructor.
  Draw_View(Standard_Integer theId,
            Draw_Viewer*     theViewer,
            const char*      theTitle);

  //! Destructor.
  ~Draw_View();

public: // @name getters and setters

  //! Sets horizontal offset.
  void SetDx(const Standard_Integer theDx)
  {
    myDx = theDx;
  }

  //! Sets vertical offset.
  void SetDy(const Standard_Integer theDy)
  {
    myDy = theDy;
  }

  //! Sets parameter of zoom.
  void SetZoom(const double theZoom)
  {
    myZoom = theZoom;
  }

  //! Sets view matrix.
  void SetMatrix(const gp_Trsf& theMatrix)
  {
    myMatrix = theMatrix;
  }

  //! Sets focal distance.
  void SetFocalDistance(const double theDistance)
  {
    myFocalDistance = theDistance;
  }

  //! Gets horizontal offset.
  const Standard_Integer GetDx() const
  {
    return myDx;
  }

  //! Gets vertical offset.
  const Standard_Integer GetDy() const
  {
    return myDy;
  }

  //! Gets parameter of zoom.
  const double GetZoom() const
  {
    return myZoom;
  }

  //! Gets matrix of view.
  const gp_Trsf& GetMatrix() const
  {
    return myMatrix;
  }

  //! Gets focal distance.
  const double GetFocalDistance() const
  {
    return myFocalDistance;
  }

public: //! @name public inline methods

  //! Returns type of view.
  const char* Type()
  {
    return myType;
  }

  //! Returns true value if current view in 2D mode.
  const Standard_Boolean Is2D() const
  {
    return myIs2D;
  }

  //! Returns true value if current view in perspective mode.
  const double IsPerspective() const
  {
    return myIsPers;
  }

public: //! @name view API

  //! Initialize view by the type.
  Standard_Boolean Init(const char* theType);

  //! Transformates view matrix.
  void Transform(const gp_Trsf& theTransformation);

  //! Resets frame of current view.
  void ResetFrame();

  //! Returns parameters of frame corners.
  void GetFrame(Standard_Integer& theX0,Standard_Integer& theY0,
                Standard_Integer& theX1,Standard_Integer& theY1);

  //! Perform window exposing.
  void WExpose();

protected:

  Standard_Integer       myId;
  Draw_Viewer*           myViewer;
  char                   myType[5];
  Standard_Boolean       myIsPers;
  Standard_Boolean       myIs2D;
  double          myFocalDistance;
  double          myZoom;
  gp_Trsf                myMatrix;
  Standard_Integer       myDx;
  Standard_Integer       myDy;
  Standard_Integer       myFrameX0;
  Standard_Integer       myFrameY0;
  Standard_Integer       myFrameX1;
  Standard_Integer       myFrameY1;
};

#endif
