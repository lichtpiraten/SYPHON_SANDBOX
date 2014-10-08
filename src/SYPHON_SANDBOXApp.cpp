#include "cinder/app/AppBasic.h"
#include "cinder/gl/gl.h"
#include "cinder/gl/Texture.h"

#include "CinderFreenect.h"

using namespace ci;
using namespace ci::app;
using namespace std;

class SYPHON_SANDBOXApp : public AppBasic {
  public:
	void prepareSettings( Settings* settings );
	void setup();
	void update();
	void draw();
	
	KinectRef		mKinect;
	gl::Texture		mColorTexture, mDepthTexture;	
};

void SYPHON_SANDBOXApp::prepareSettings( Settings* settings )
{
	settings->setWindowSize( 1280, 480 );
}

void SYPHON_SANDBOXApp::setup()
{
	mKinect = Kinect::create();
}

void SYPHON_SANDBOXApp::update()
{	
	if( mKinect->checkNewDepthFrame() )
		mDepthTexture = mKinect->getDepthImage();
	
	if( mKinect->checkNewVideoFrame() )
		mColorTexture = mKinect->getVideoImage();
}

void SYPHON_SANDBOXApp::draw()
{
	gl::clear(); 
	gl::setMatricesWindow( getWindowWidth(), getWindowHeight() );
	if( mDepthTexture )
		gl::draw( mDepthTexture );
	if( mColorTexture )
		gl::draw( mColorTexture, Vec2i( 640, 0 ) );
}

CINDER_APP_BASIC( SYPHON_SANDBOXApp, RendererGl )
