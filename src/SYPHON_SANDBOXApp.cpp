#include "cinder/app/AppBasic.h"
#include "cinder/gl/gl.h"
#include "cinder/gl/Texture.h"

#include "cinder/params/Params.h"

#include "CinderFreenect.h"
#include "cinderSyphon.h"
#include "cinderfx/Fluid2D.h"

using namespace ci;
using namespace ci::app;
using namespace std;
using namespace cinderfx;

class SANDOXXApp : public AppBasic {
public:
    void prepareSettings( Settings* settings );
    void setup();
    void update();
    void draw();
    
    // PARAMS
    params::InterfaceGlRef	mParams;
    
    
    KinectRef		mKinect;
    gl::TextureRef		mColorTexture, mDepthTexture;
    syphonServer mScreenSyphon; //each item to publish requires a different server
    syphonClient mClientSyphon; //our syphon client
    syphonServer mTextureSyphon;
    
private:
    float					mVelScale;
    float					mDenScale;
    ci::Vec2f				mPrevPos;
    cinderfx::Fluid2D		mFluid2D;
	ci::gl::Texture			mTex;
};

void SANDOXXApp::prepareSettings( Settings* settings )
{
    settings->setWindowSize( 640, 480 );
}

void SANDOXXApp::setup()
{
    glEnable( GL_TEXTURE_2D );
    
    mDenScale = 25;
    
    mFluid2D.set( 192, 192 );
    mFluid2D.setDensityDissipation( 0.99f );
    mVelScale = 3.0f*std::max( mFluid2D.resX(), mFluid2D.resY() );
   	
    // SETUP PARAMS
    mParams = params::InterfaceGl::create( "Depth", Vec2i( 200, 310 ) );
    mParams->addParam( "Height_Max", &Kinect::mHeightMax, "min=0 max=2048" );
    mParams->addParam( "Height_Min", &Kinect::mHeightMin, "min=100 max=2048" );
    
    mKinect = Kinect::create();
    mScreenSyphon.setName("Screen Output"); // set a name for each item to be published
    
    mFluid2D.enableDensity();
    mFluid2D.enableVorticityConfinement();
    mFluid2D.initSimData();
    
    mClientSyphon.setup();
    
    // in order for this to work, you must run simple server which is a syphon test application
    // feel free to change the app and server name for your specific case
    mClientSyphon.set("", "Simple Server");
    mTextureSyphon.setName("Texture Output");
    
    mClientSyphon.bind();

}

void SANDOXXApp::update()
{
    if( mKinect->checkNewDepthFrame() )
    {
        
        mDepthTexture = gl::Texture::create(mKinect->getDepthImage());
        
        for (auto& tp : Kinect::mTouchPoints)
        {
            float x = (tp.x/(float)640)*mFluid2D.resX();
            float y = (tp.y/(float)480)*mFluid2D.resY();
            mFluid2D.splatVelocity( x, y, mVelScale * Vec2f(1.0,1.0) );
            mFluid2D.splatDensity( x, y, mDenScale );
        }
    }
    
    if( mKinect->checkNewVideoFrame() )
        mColorTexture = gl::Texture::create(mKinect->getVideoImage());
}

void SANDOXXApp::draw()
{
    gl::clear();
    gl::setMatricesWindow( getWindowWidth(), getWindowHeight() );
    if( mDepthTexture )
        gl::draw( mDepthTexture );
    if( mColorTexture )
        gl::draw( mColorTexture, Vec2i( 0, 0 ) );/*
    Channel32f chan( mFluid2D.resX(), mFluid2D.resY(), mFluid2D.resX()*sizeof(float), 1, const_cast<float*>( mFluid2D.density().data() ) );
    
    if( ! mTex ) {
        mTex = gl::Texture( chan );
    } else {
        mTex.update( chan );
    }
    gl::color( Color( 1, 1, 1 ) );
    gl::draw( mTex, getWindowBounds() );
    */
    mClientSyphon.draw(Vec2f(16.f, 64.f)); //draw our client image
    
    mScreenSyphon.publishScreen(); //publish the screen's output
    mTextureSyphon.publishTexture(mColorTexture); //publish our texture without shader

    // DRAW PARAMS WINDOW
    mParams->draw();
}

CINDER_APP_BASIC( SANDOXXApp, RendererGl )
