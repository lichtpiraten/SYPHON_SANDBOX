/*
 Copyright (c) 2010, The Cinder Project, All rights reserved.
 
 This code is intended for use with the Cinder C++ library: http://libcinder.org
 
 Portions copyright Rui Madeira: http://ruim.pt
 
 Redistribution and use in source and binary forms, with or without modification, are permitted provided that
 the following conditions are met:
 
 * Redistributions of source code must retain the above copyright notice, this list of conditions and
	the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and
	the following disclaimer in the documentation and/or other materials provided with the distribution.
 
 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
 WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 POSSIBILITY OF SUCH DAMAGE.
 */


#include "CinderFreenect.h"
#include "libfreenect.h"
using namespace std;

#include <iostream>
#include <boost/algorithm/clamp.hpp>

namespace cinder {
    
    // statics
    std::mutex			Kinect::sContextMutex;
    freenect_context*	Kinect::sContext = 0;
    
    class ImageSourceKinectColor : public ImageSource {
    public:
        ImageSourceKinectColor( uint8_t *buffer, shared_ptr<Kinect::Obj> ownerObj )
        : ImageSource(), mOwnerObj( ownerObj ), mData( buffer )
        {
            setSize( 640, 480 );
            setColorModel( ImageIo::CM_RGB );
            setChannelOrder( ImageIo::RGB );
            setDataType( ImageIo::UINT8 );
        }
        
        ~ImageSourceKinectColor()
        {
            // let the owner know we are done with the buffer
            mOwnerObj->mColorBuffers.derefBuffer( mData );
        }
        
        virtual void load( ImageTargetRef target )
        {
            ImageSource::RowFunc func = setupRowFunc( target );
            
            for( int32_t row = 0; row < 480; ++row )
                ((*this).*func)( target, row, mData + row * 640 * 3 );
        }
        
    protected:
        shared_ptr<Kinect::Obj>		mOwnerObj;
        uint8_t						*mData;
    };
    
    class ImageSourceKinectInfrared : public ImageSource {
    public:
        ImageSourceKinectInfrared( uint8_t *buffer, shared_ptr<Kinect::Obj> ownerObj )
        : ImageSource(), mOwnerObj( ownerObj ), mData( buffer )
        {
            setSize( 640, 480 );
            setColorModel( ImageIo::CM_GRAY );
            setChannelOrder( ImageIo::Y );
            setDataType( ImageIo::UINT8 );
        }
        
        ~ImageSourceKinectInfrared()
        {
            // let the owner know we are done with the buffer
            mOwnerObj->mColorBuffers.derefBuffer( mData );
        }
        
        virtual void load( ImageTargetRef target )
        {
            ImageSource::RowFunc func = setupRowFunc( target );
            
            for( int32_t row = 0; row < 480; ++row )
                ((*this).*func)( target, row, mData + row * 640 * 1 );
        }
        
    protected:
        shared_ptr<Kinect::Obj>		mOwnerObj;
        uint8_t						*mData;
    };
    
    
    class ImageSourceKinectDepth : public ImageSource {
    public:
        ImageSourceKinectDepth( uint16_t *buffer, shared_ptr<Kinect::Obj> ownerObj )
        : ImageSource(), mOwnerObj( ownerObj ), mData( buffer )
        {
            setSize( 640, 480 );
            setColorModel( ImageIo::CM_GRAY );
            setChannelOrder( ImageIo::Y );
            setDataType( ImageIo::UINT16 );
        }
        
        ~ImageSourceKinectDepth()
        {
            // let the owner know we are done with the buffer
            mOwnerObj->mDepthBuffers.derefBuffer( mData );
        }
        
        virtual void load( ImageTargetRef target )
        {
            ImageSource::RowFunc func = setupRowFunc( target );
            
            for( int32_t row = 0; row < 480; ++row )
                ((*this).*func)( target, row, mData + row * 640 );
        }
        
    protected:
        shared_ptr<Kinect::Obj>		mOwnerObj;
        uint16_t					*mData;
    };
    
    // Used as the deleter for the shared_ptr returned by getImageData() and getDepthData()
    template<typename T>
    class KinectDataDeleter {
    public:
        KinectDataDeleter( Kinect::Obj::BufferManager<T> *bufferMgr, shared_ptr<Kinect::Obj> ownerObj )
        : mOwnerObj( ownerObj ), mBufferMgr( bufferMgr )
        {}
        
        void operator()( T *data ) {
            mBufferMgr->derefBuffer( data );
        }
        
        shared_ptr<Kinect::Obj>			mOwnerObj; // to prevent deletion of our parent Obj
        Kinect::Obj::BufferManager<T> *mBufferMgr;
    };
    
    Kinect::Kinect( Device device )
    : mObj( new Obj( device.mIndex, device.mDepthRegister ) )
    {
    }
    
    Kinect::Obj::Obj( int deviceIndex, bool depthRegister )
    : mColorBuffers( 640 * 480 * 3, this ), mDepthBuffers( 640 * 480, this ),
    mShouldDie( false ), mVideoInfrared( false ),
    mNewVideoFrame( false ), mNewDepthFrame( false )
    {
        if( freenect_open_device( getContext(), &mDevice, deviceIndex ) < 0 )
            throw ExcFailedOpenDevice();
        
        freenect_set_user( mDevice, this );
        freenect_update_tilt_state( mDevice );
        mTilt = freenect_get_tilt_degs( freenect_get_tilt_state( mDevice ) );
        freenect_set_led( mDevice, ::LED_GREEN );
        freenect_set_depth_callback( mDevice, depthImageCB );
        freenect_set_video_mode( mDevice, freenect_find_video_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_VIDEO_RGB) );
        
        if( depthRegister ) {
            freenect_set_depth_mode( mDevice, freenect_find_depth_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_DEPTH_REGISTERED));
        }
        else {
            freenect_set_depth_mode( mDevice, freenect_find_depth_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_DEPTH_11BIT));
        }
        
        mLastVideoFrameInfrared = mVideoInfrared;
        
        mThread = shared_ptr<thread>( new thread( threadedFunc, this ) );
    }
    
    Kinect::Obj::~Obj()
    {
        mShouldDie = true;
        mThread->join();
    }

    Kinect::buffer_queue_type Kinect::mLastBuffers = Kinect::buffer_queue_type();
    Kinect::touchpoints_type Kinect::mTouchPoints;
    int Kinect::mHeightMin = 800;
    int Kinect::mHeightMax = 1000;   
 
    void Kinect::depthImageCB( freenect_device *dev, void *d, uint32_t timestamp )
    {
        Kinect::Obj *kinectObj = reinterpret_cast<Kinect::Obj*>( freenect_get_user( dev ) );
        {
            lock_guard<recursive_mutex> lock( kinectObj->mMutex );
            
            uint16_t *depth = reinterpret_cast<uint16_t*>( d );
            
            kinectObj->mColorBuffers.derefActiveBuffer();					// finished with current active buffer
            
	constexpr size_t maxBuffers = 4;	    
		if (mHeightMin >= mHeightMax) 
			std::swap(mHeightMin,mHeightMax);
	
	  int diff = mHeightMax - mHeightMin;
	  if (diff == 0) diff = 500;
            while (mLastBuffers.size() < maxBuffers)
	 { 
	    mLastBuffers.push_front(kinectObj->mColorBuffers.getNewBuffer()); // request a new buffer
 
	    uint16_t lastPixel = 0;

		// Hack for Removing depth shadow (to be improved)
	    for( size_t p = 0; p < 640 * 480; ++p )
            {
		uint16_t v = depth[p];
		if (v < 2047)
		{
			lastPixel = v;
		} else
		{
			v = lastPixel;
		}
 	    	using boost::algorithm::clamp;
		int pix = 255 - clamp(int((int(v) - mHeightMin)*256 / diff),0,255);
		auto& destPixels = mLastBuffers.front(); 
		destPixels[p*3] = pix; 
		destPixels[p*3+1] = pix; 
		destPixels[p*3+2] = pix;
	    }  
	  }  
	  
	  for( size_t p = 0; p < 640 * 480 *3 ; ++p )
          {
		int buf = mLastBuffers.back()[p];
		for (int i = 0; i < maxBuffers-1; ++i)
		{
			buf += mLastBuffers[i][p];
		}
		buf /= maxBuffers;
		mLastBuffers.back()[p] = buf;
	  }
            getTouchPoints();

            kinectObj->mColorBuffers.setActiveBuffer( mLastBuffers.back() );			// set this new buffer to be the current active buffer
            mLastBuffers.pop_back();
	    kinectObj->mNewVideoFrame = true;								// flag that there's a new depth frame
        }
    }
    
    void Kinect::getTouchPoints()
    {
        if (mLastBuffers.size() < 2) return;
        const size_t blockSize = 16;
        Kinect::touchpoints_type points;
        for( size_t y = 0; y < 480 ; y += blockSize )
        {
            for ( size_t x = 0; x < 640 ; x += blockSize )
            {
                int sum = 0;
                Vec2i vec(0,0);
                for (int yy = y; yy < y+blockSize; ++yy)
                for (int xx = x; xx < x+blockSize; ++xx)
                {
                    int pos = (yy*640 + xx)*3;
                    using boost::algorithm::clamp;
                    int value = clamp(mLastBuffers.back()[pos] - mLastBuffers.front()[pos],0,255);
                    vec.x += (xx * value) / 256;
                    vec.y += (yy * value) / 256;
                    sum += value;
                }
                
                vec.x /= blockSize * blockSize;
                vec.y /= blockSize * blockSize;
                
                if (sum > blockSize*blockSize*10)
                {
                    
                    std::cout << sum << " " << vec.x << " " << vec.y << std::endl;
                    points.push_back(vec);
                }
            }
        }
        mTouchPoints = points;
    }
    
    void Kinect::threadedFunc( Kinect::Obj *kinectObj )
    {
        ci::ThreadSetup ts;
        
        freenect_start_depth( kinectObj->mDevice );
        freenect_start_video( kinectObj->mDevice );
        
        while( ( ! kinectObj->mShouldDie ) && ( freenect_process_events( getContext() ) >= 0 ) )
            ;
        
        freenect_close_device( kinectObj->mDevice );
    }
    
    freenect_context* Kinect::getContext()
    {
        // ultimately this should be replaced with a call_once
        lock_guard<mutex> contextLock( sContextMutex );
        if( ! sContext ) {
            if( freenect_init( &sContext, NULL ) < 0 )
                ; // throw ExcFailedFreenectInit(); // this seems to always fail
            freenect_set_log_level( sContext, FREENECT_LOG_ERROR );
        }
        return sContext;
    }
    
    int	Kinect::getNumDevices()
    {
        try {
            return freenect_num_devices( getContext() );
        }
        catch( ExcFailedFreenectInit &e ) { // a failed initialization implies 0 kinects.
            return 0;
        }
    }
    
    bool Kinect::checkNewVideoFrame()
    {
        lock_guard<recursive_mutex> lock( mObj->mMutex );
        bool oldValue = mObj->mNewVideoFrame;
        mObj->mNewVideoFrame = false;
        return oldValue;
    }
    
    bool Kinect::checkNewDepthFrame()
    {
        lock_guard<recursive_mutex> lock( mObj->mMutex );
        bool oldValue = mObj->mNewDepthFrame;
        mObj->mNewDepthFrame = false;
        return oldValue;
    }
    
    void Kinect::setTilt( float degrees )
    {
        mObj->mTilt = math<float>::clamp( degrees, -31, 31 );
        freenect_set_tilt_degs( mObj->mDevice, mObj->mTilt );
    }
    
    float Kinect::getTilt() const
    {
        return mObj->mTilt;
    }
    
    void Kinect::setLedColor( LedColor ledColorCode )
    {
        int code = ledColorCode;
        freenect_set_led( mObj->mDevice, (freenect_led_options)code );
    }
    
    Vec3f Kinect::getAccel() const
    {
        Vec3d raw;
        freenect_update_tilt_state( mObj->mDevice );
        freenect_get_mks_accel( freenect_get_tilt_state( mObj->mDevice ), &raw.x, &raw.y, &raw.z );
        return Vec3f( raw );
    }
    
    ImageSourceRef Kinect::getVideoImage()
    {
        // register a reference to the active buffer
        uint8_t *activeColor = mObj->mColorBuffers.refActiveBuffer();
        if( mObj->mLastVideoFrameInfrared )
            return ImageSourceRef( new ImageSourceKinectInfrared( activeColor, this->mObj ) );
        else
            return ImageSourceRef( new ImageSourceKinectColor( activeColor, this->mObj ) );
    }
    
    ImageSourceRef Kinect::getDepthImage()
    {
        // register a reference to the active buffer
        uint16_t *activeDepth = mObj->mDepthBuffers.refActiveBuffer();
        return ImageSourceRef( new ImageSourceKinectDepth( activeDepth, this->mObj ) );
    }
    
    std::shared_ptr<uint8_t> Kinect::getVideoData()
    {
        // register a reference to the active buffer
        uint8_t *activeColor = mObj->mColorBuffers.refActiveBuffer();
        return shared_ptr<uint8_t>( activeColor, KinectDataDeleter<uint8_t>( &mObj->mColorBuffers, mObj ) );
    }
    
    std::shared_ptr<uint16_t> Kinect::getDepthData()
    {
        // register a reference to the active buffer
        uint16_t *activeDepth = mObj->mDepthBuffers.refActiveBuffer();
        return shared_ptr<uint16_t>( activeDepth, KinectDataDeleter<uint16_t>( &mObj->mDepthBuffers, mObj ) );
    }
    
    void Kinect::setVideoInfrared( bool infrared )
    {
        if( mObj->mVideoInfrared != infrared ) {
            freenect_stop_video( mObj->mDevice );
            {
                lock_guard<recursive_mutex> lock( mObj->mMutex );
                
                mObj->mVideoInfrared = infrared;
                if( mObj->mVideoInfrared )
                    freenect_set_video_mode( mObj->mDevice, freenect_find_video_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_VIDEO_IR_8BIT) );
                else
                    freenect_set_video_mode( mObj->mDevice, freenect_find_video_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_VIDEO_RGB) );
            }
            freenect_start_video( mObj->mDevice );
        }
    }
    
    // Buffer management
    template<typename T>
    Kinect::Obj::BufferManager<T>::~BufferManager()
    {
        for( typename map<T*,size_t>::iterator bufIt = mBuffers.begin(); bufIt != mBuffers.end(); ++bufIt ) {
            delete [] bufIt->first;
        }
    }
    
    template<typename T>
    T* Kinect::Obj::BufferManager<T>::getNewBuffer()
    {
        lock_guard<recursive_mutex> lock( mKinectObj->mMutex );
        
        typename map<T*,size_t>::iterator bufIt;
        for( bufIt = mBuffers.begin(); bufIt != mBuffers.end(); ++bufIt ) {
            if( bufIt->second == 0 ) // 0 means free buffer
                break;
        }
        if( bufIt != mBuffers.end() ) {
            bufIt->second = 1;
            return bufIt->first;
        }
        else { // there were no available buffers - add a new one and return it
            T *newBuffer = new T[mAllocationSize];
            mBuffers[newBuffer] = 1;
            return newBuffer;
        }
    }
    
    template<typename T>
    void Kinect::Obj::BufferManager<T>::setActiveBuffer( T *buffer )
    {
        lock_guard<recursive_mutex> lock( mKinectObj->mMutex );
        // assign new active buffer
        mActiveBuffer = buffer;
    }
    
    template<typename T>
    T* Kinect::Obj::BufferManager<T>::refActiveBuffer()
    {
        lock_guard<recursive_mutex> lock( mKinectObj->mMutex );
        mBuffers[mActiveBuffer]++;
        return mActiveBuffer;
    }
    
    template<typename T>
    void Kinect::Obj::BufferManager<T>::derefActiveBuffer()
    {
        lock_guard<recursive_mutex> lock( mKinectObj->mMutex );
        if( mActiveBuffer ) 	// decrement use count on current active buffer
            mBuffers[mActiveBuffer]--;		
    }
    
    template<typename T>
    void Kinect::Obj::BufferManager<T>::derefBuffer( T *buffer )
    {
        lock_guard<recursive_mutex> lock( mKinectObj->mMutex );
        mBuffers[buffer]--;
    }
    
} // namespace cinder
