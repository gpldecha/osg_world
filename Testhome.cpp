#include <osg/NodeCallback>
#include <osg/PositionAttitudeTransform>
#include <osgViewer/Viewer>
#include <osg/MatrixTransform>
#include <osgDB/ReadFile>
#include <osgGA/TrackballManipulator>
#include <osgGA/NodeTrackerManipulator>
#include <osg/Texture2D>


#include "KeyboardHandler.h"
#include <osgDB/Registry>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include "opencv2/opencv.hpp"
#include <osg/CameraNode>
#include <chrono>
#include <thread>
#include <osg/ShapeDrawable>

#include <mjpeg_server/mjpeg_server.hpp>
#include <osgViewer/CompositeViewer>
#include <boost/lexical_cast.hpp>

#include "TransformAccumulator.h"


typedef std::chrono::duration<int, std::micro> microseconds_type;

bool manuallyPlaceCamera = false;

void toggleView()
{
   if (! manuallyPlaceCamera)
      manuallyPlaceCamera = true;
   else
      manuallyPlaceCamera = false;
}


void create_pyramid_geometry(osg::Geometry* pyramidGeometry){

    osg::Vec3Array* pyramidVertices = new osg::Vec3Array;
    pyramidVertices->push_back( osg::Vec3( 0, 0, 0) ); // front left
    pyramidVertices->push_back( osg::Vec3(10, 0, 0) ); // front right
    pyramidVertices->push_back( osg::Vec3(10,10, 0) ); // back right
    pyramidVertices->push_back( osg::Vec3( 0,10, 0) ); // back left
    pyramidVertices->push_back( osg::Vec3( 5, 5,10) ); // peak
    pyramidGeometry->setVertexArray( pyramidVertices );


    osg::DrawElementsUInt* pyramidBase = new osg::DrawElementsUInt(osg::PrimitiveSet::QUADS, 0);
    pyramidBase->push_back(3);
    pyramidBase->push_back(2);
    pyramidBase->push_back(1);
    pyramidBase->push_back(0);
    pyramidGeometry->addPrimitiveSet(pyramidBase);

    osg::DrawElementsUInt* pyramidFaceOne = new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLES, 0);
    pyramidFaceOne->push_back(0);
    pyramidFaceOne->push_back(1);
    pyramidFaceOne->push_back(4);
    pyramidGeometry->addPrimitiveSet(pyramidFaceOne);

    osg::DrawElementsUInt* pyramidFaceTwo = new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLES, 0);
    pyramidFaceTwo->push_back(1);
    pyramidFaceTwo->push_back(2);
    pyramidFaceTwo->push_back(4);
    pyramidGeometry->addPrimitiveSet(pyramidFaceTwo);

    osg::DrawElementsUInt* pyramidFaceThree = new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLES, 0);
    pyramidFaceThree->push_back(2);
    pyramidFaceThree->push_back(3);
    pyramidFaceThree->push_back(4);
    pyramidGeometry->addPrimitiveSet(pyramidFaceThree);

    osg::DrawElementsUInt* pyramidFaceFour = new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLES, 0);
    pyramidFaceFour->push_back(3);
    pyramidFaceFour->push_back(0);
    pyramidFaceFour->push_back(4);
    pyramidGeometry->addPrimitiveSet(pyramidFaceFour);

    osg::Vec4Array* colors = new osg::Vec4Array;
    colors->push_back(osg::Vec4(1.0f, 0.0f, 0.0f, 1.0f) ); //index 0 red
    colors->push_back(osg::Vec4(0.0f, 1.0f, 0.0f, 1.0f) ); //index 1 green
    colors->push_back(osg::Vec4(0.0f, 0.0f, 1.0f, 1.0f) ); //index 2 blue
    colors->push_back(osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f) ); //index 3 white
    colors->push_back(osg::Vec4(1.0f, 0.0f, 0.0f, 1.0f) ); //index 4 red

    pyramidGeometry->setColorArray(colors);
    pyramidGeometry->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
}



struct SnapImage : public osg::Camera::DrawCallback
{
    SnapImage(const std::string& filename):
        _filename(filename),
        _snapImage(false)
    {
        _image = new osg::Image;
    }

    virtual void operator () (osg::RenderInfo& renderInfo) const
    {

        //if (!_snapImage) return;

        osg::notify(osg::NOTICE)<<"Camera callback"<<std::endl;

        osg::Camera* camera = renderInfo.getCurrentCamera();
        osg::Viewport* viewport = camera ? camera->getViewport() : 0;

        osg::notify(osg::NOTICE)<<"Camera callback "<<camera<<" "<<viewport<<std::endl;

        if (viewport && _image.valid())
        {
            _image->readPixels(int(viewport->x()),int(viewport->y()),int(viewport->width()),int(viewport->height()),
                               GL_RGBA,
                               GL_UNSIGNED_BYTE);
            osgDB::writeImageFile(*_image, _filename);

            osg::notify(osg::NOTICE)<<"Taken screenshot, and written to '"<<_filename<<"'"<<std::endl;
        }

       // _snapImage = false;
    }

    std::string                         _filename;
    mutable bool                        _snapImage;
    mutable osg::ref_ptr<osg::Image>    _image;
};


struct SnapeImageHandler : public osgGA::GUIEventHandler
{

    SnapeImageHandler(int key,SnapImage* si):
        _key(key),
        _snapImage(si) {}

    bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
    {
        if (ea.getHandled()) return false;

        switch(ea.getEventType())
        {
            case(osgGA::GUIEventAdapter::KEYUP):
            {
                if (ea.getKey() == 'o' )
                {
                    osgViewer::View* view = dynamic_cast<osgViewer::View*>(&aa);
                    osg::Node* node = view ? view->getSceneData() : 0;
                    if (node)
                    {
                        osgDB::writeNodeFile(*node, "hud.osgt");
                        osgDB::writeNodeFile(*node, "hud.osgb");
                    }
                    return true;
                }

                if (ea.getKey() == _key)
                {
                    osg::notify(osg::NOTICE)<<"event handler"<<std::endl;
                    _snapImage->_snapImage = true;
                    return true;
                }

                break;
            }
        default:
            break;
        }

        return false;
    }

    int                     _key;
    osg::ref_ptr<SnapImage> _snapImage;
};

class VideoRecord : public osg::CameraNode::DrawCallback{

    public:

        VideoRecord(int port=9800){
            _snapImageOnNextFrame = false;
            image = new osg::Image;

            using namespace http::server;
            // Run server in background thread.
            std::size_t num_threads = 1;
            std::string doc_root = "./";
            //this initializes the redirect behavor, and the /_all handlers
            server_ptr s = init_streaming_server("0.0.0.0", boost::lexical_cast<std::string>(port), doc_root, num_threads);
            stmr = streamer_ptr(new streamer);
            register_streamer(s, stmr, "/stream_0");
            s->start();
        }
        virtual void operator () (const osg::CameraNode& camera) const{
            int x,y,width,height;
            x = camera.getViewport()->x();
            y = camera.getViewport()->y();
            width = camera.getViewport()->width();
            height = camera.getViewport()->height();
            image->readPixels(x,y,width,height,GL_RGB,GL_UNSIGNED_BYTE);
            cv::Mat cvImg(image->t(), image->s(), CV_8UC3);
            cvImg.data = (uchar*)image->data();
            cv::flip(cvImg, cvImg, 0);
            bool wait = false; //don't wait for there to be more than one webpage looking at us.
            int quality = 80;
            stmr->post_image(cvImg,quality, wait);
            //cv::imshow("video", cvImg);
            //if(cv::waitKey(30) >= 0)
             //   return;


       }
    protected:

    std::string _filename;
    mutable bool _snapImageOnNextFrame;
    osg::ref_ptr<osg::Image> image;
    cv::Mat cvImg;
    http::server::server_ptr s;
    http::server::streamer_ptr stmr;

};


class SetCameraParam{

public:

    SetCameraParam(osg::Camera* camera):
        camera(camera)
    {
        e_roll    = osg::Vec3d(0,1,0);
        e_pitch   = osg::Vec3d(1,0,0);
        e_yaw     = osg::Vec3d(0,0,1);
    }

    void set_camera(double x, double y, double z, double roll, double pitch, double yaw){
        cameraTrans.makeTranslate(x, y, z);
        cameraRotation.makeRotate(
           osg::DegreesToRadians(roll), e_roll, // roll
           osg::DegreesToRadians(pitch), e_pitch , // pitch
           osg::DegreesToRadians(yaw), e_yaw );
        myCameraMatrix = cameraRotation * cameraTrans;
        i = myCameraMatrix.inverse(myCameraMatrix);
        camera->setViewMatrix((osg::Matrix(i.ptr() ))*osg::Matrix::rotate( -M_PI/2.0, 1, 0, 0 ) );
    }


private:

    osg::Vec3d e_roll, e_pitch, e_yaw;
    osg::Camera* camera;
    osg::Matrixd myCameraMatrix;
    osg::Matrixd cameraRotation;
    osg::Matrixd cameraTrans;
    osg::Matrixd i;
};

void test_home(){

    osgViewer::Viewer viewer;

    osg::Group* root = new osg::Group();    // root of the scene
    osg::Geode* pyramidGeode = new osg::Geode();    // geode to collect drawables
    osg::Geometry* pyramidGeometry = new osg::Geometry(); //geometry instance to associate vertices and vertex data
    pyramidGeode->addDrawable(pyramidGeometry);


    /*transform = new osg::PositionAttitudeTransform;
    unitSphere = new osg::Sphere( osg::Vec3(0,0,0.0), 2.0);
    unitSphereDrawable = new osg::ShapeDrawable(unitSphere);
    basicShapesGeode = new osg::Geode();
    basicShapesGeode->addDrawable(unitSphereDrawable);*/

    create_pyramid_geometry(pyramidGeometry);

    // Declare and initialize a transform node.
    osg::PositionAttitudeTransform* pyramidTwoXForm1 = new osg::PositionAttitudeTransform();
    pyramidTwoXForm1->setPosition( osg::Vec3(10, 0, 0) );
    pyramidTwoXForm1->addChild(pyramidGeode);
    root->addChild(pyramidTwoXForm1);


    osg::PositionAttitudeTransform* pyramidTwoXForm2 = new osg::PositionAttitudeTransform();
    pyramidTwoXForm2->setPosition( osg::Vec3(-10, 0, 0) );
    pyramidTwoXForm2->addChild(pyramidGeode);
    root->addChild(pyramidTwoXForm2);

    osg::PositionAttitudeTransform* pyramidTwoXForm3 = new osg::PositionAttitudeTransform();
    pyramidTwoXForm3->setPosition( osg::Vec3(0, 0, 10) );
    pyramidTwoXForm3->addChild(pyramidGeode);
    root->addChild(pyramidTwoXForm3);


    //SnapImage* postDrawCallback = new SnapImage("/home/guillaume/Desktop/PostDrawCallback.png");
    //viewer.getCamera()->setPostDrawCallback(postDrawCallback);
    //viewer.addEventHandler(new SnapeImageHandler('p',postDrawCallback));

    //SnapImage* finalDrawCallback = new SnapImage("/home/guillaume/Desktop/FinalDrawCallback.png");
    //viewer.addEventHandler(new SnapeImageHandler('f',finalDrawCallback));

    osg::ref_ptr<VideoRecord> snapImageDrawCallback = new VideoRecord();
    viewer.getCamera()->setPostDrawCallback(snapImageDrawCallback.get());


    osgGA::TrackballManipulator *Tman = new osgGA::TrackballManipulator();
    viewer.setUpViewInWindow(0, 0, 600, 400);
    viewer.setCameraManipulator(Tman);
    viewer.setSceneData( root );

    //viewer.getCamera()->setFinalDrawCallback(finalDrawCallback);


    viewer.realize();

    osg::Vec3f eye, center, up;

    viewer.home();
    while( !viewer.done() )
    {
       viewer.frame();
       viewer.getCamera()->getViewMatrixAsLookAt(eye, center, up);
       std::cout<< "eye: " << eye[0] << " " << eye[1] << " " << eye[2] << std::endl;
       std::cout<< "center: " << center[0] << " " << center[1] << " " << center[2] << std::endl;
       std::cout<< "up: " << up[0] << " " << up[1] << " " << up[2] << std::endl;

    }

}

void test_no_dispaly_take_screen_captures(){

    int width = 600;
    int height = 400;
    osgViewer::Viewer viewer;
    osg::Camera *camera = viewer.getCamera();
    osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;
    traits->x = 0;
    traits->y = 0;
    traits->width = width;
    traits->height = height;
    traits->doubleBuffer = false;
    traits->sharedContext = 0;
    traits->pbuffer = true;
    traits->readDISPLAY();
    osg::GraphicsContext *gc = osg::GraphicsContext::createGraphicsContext(traits.get());
    camera->setGraphicsContext(gc);
    //camera->setClearColor(osg::Vec4(0.0f,0.0f,0.0f,0.0f));
    camera->setDrawBuffer(GL_FRONT);
    camera->setReadBuffer(GL_FRONT);
    camera->setViewport(new osg::Viewport(0, 0, width, height));
    double fovy, aspectRatio, near, far;
    camera->getProjectionMatrixAsPerspective(fovy, aspectRatio, near, far);
    double newAspectRatio = double(traits->width) / double(traits->height);
    double aspectRatioChange = newAspectRatio / aspectRatio;
    if (aspectRatioChange != 1.0){
        camera->getProjectionMatrix() *= osg::Matrix::scale(1.0/aspectRatioChange,1.0,1.0);
    }
    osg::Vec3d eye( 5.0, -73.8471, 10.0 );
    osg::Vec3d center( 5.0, -72.8471, 10.0 );
    osg::Vec3d up( 0.0, 0.0, 1.0 );
    camera->setViewMatrixAsLookAt( eye, center, up );


    osg::Group* root = new osg::Group();    // root of the scene
    osg::Geode* pyramidGeode = new osg::Geode();    // geode to collect drawables
    osg::Geometry* pyramidGeometry = new osg::Geometry(); //geometry instance to associate vertices and vertex data
    pyramidGeode->addDrawable(pyramidGeometry);


    create_pyramid_geometry(pyramidGeometry);

    // Declare and initialize a transform node.
    osg::PositionAttitudeTransform* pyramidTwoXForm1 = new osg::PositionAttitudeTransform();
    pyramidTwoXForm1->setPosition( osg::Vec3(10, 0, 0) );
    pyramidTwoXForm1->addChild(pyramidGeode);
    root->addChild(pyramidTwoXForm1);


    osg::PositionAttitudeTransform* pyramidTwoXForm2 = new osg::PositionAttitudeTransform();
    pyramidTwoXForm2->setPosition( osg::Vec3(-10, 0, 0) );
    pyramidTwoXForm2->addChild(pyramidGeode);
    root->addChild(pyramidTwoXForm2);

    osg::PositionAttitudeTransform* pyramidTwoXForm3 = new osg::PositionAttitudeTransform();
    pyramidTwoXForm3->setPosition( osg::Vec3(0, 0, 10) );
    pyramidTwoXForm3->addChild(pyramidGeode);
    root->addChild(pyramidTwoXForm3);

    // set the scene to render
    viewer.setSceneData(root);
    viewer.setCamera(camera);


    SnapImage* finalDrawCallback = new SnapImage("/home/guillaume/Desktop/FinalDrawCallback2.png");
    //viewer.getCamera()->setFinalDrawCallback(finalDrawCallback);
    osg::ref_ptr<VideoRecord> snapImageDrawCallback = new VideoRecord();
    viewer.getCamera()->setPostDrawCallback(snapImageDrawCallback.get());


    osg::Image *image = new osg::Image();
    //camera->attach(osg::Camera::COLOR_BUFFER0, image);

    viewer.setThreadingModel(osgViewer::Viewer::SingleThreaded);
    viewer.realize();
    viewer.frame();
   // osgDB::writeImageFile(*image, "/home/guillaume/Desktop/screenshot.png");

    double dt = 0.01;
    microseconds_type loop_rate_micro_s = microseconds_type(std::size_t(dt * 1000000.0));
    double z=0.0;

    auto start_time = std::chrono::steady_clock::now();
    auto end_time   = start_time + loop_rate_micro_s;
    while( !viewer.done() ){

        start_time = std::chrono::steady_clock::now();
        end_time   = start_time + loop_rate_micro_s;

        viewer.frame();
        pyramidTwoXForm1->setPosition(osg::Vec3(10, 0, z));

        z=z-0.01;
        std::this_thread::sleep_until(end_time);
    }

}

void test_no_display_multiple_cameras(){

    osg::Group* root = new osg::Group();    // root of the scene
    osg::Geode* pyramidGeode = new osg::Geode();    // geode to collect drawables
    osg::Geometry* pyramidGeometry = new osg::Geometry(); //geometry instance to associate vertices and vertex data
    pyramidGeode->addDrawable(pyramidGeometry);

    create_pyramid_geometry(pyramidGeometry);

    // Declare and initialize a transform node.
    osg::PositionAttitudeTransform* pyramidTwoXForm1 = new osg::PositionAttitudeTransform();
    pyramidTwoXForm1->setPosition( osg::Vec3(10, 0, 0) );
    pyramidTwoXForm1->addChild(pyramidGeode);
    root->addChild(pyramidTwoXForm1);

    osg::PositionAttitudeTransform* pyramidTwoXForm2 = new osg::PositionAttitudeTransform();
    pyramidTwoXForm2->setPosition( osg::Vec3(-10, 0, 0) );
    pyramidTwoXForm2->addChild(pyramidGeode);
    root->addChild(pyramidTwoXForm2);

    osg::PositionAttitudeTransform* pyramidTwoXForm3 = new osg::PositionAttitudeTransform();
    pyramidTwoXForm3->setPosition( osg::Vec3(0, 0, 10) );
    pyramidTwoXForm3->addChild(pyramidGeode);
    root->addChild(pyramidTwoXForm3);




    osgViewer::CompositeViewer viewer;
    osgViewer::View* view1 = new osgViewer::View;
    osg::Camera *camera1 = view1->getCamera();

    osgViewer::View* view2 = new osgViewer::View;
    osg::Camera *camera2 = view2->getCamera();



    // make a drone
    osg::PositionAttitudeTransform* transform = new osg::PositionAttitudeTransform();
    osg::Sphere* unitSphere = new osg::Sphere( osg::Vec3(0,0,0.0), 2.0);
    osg::ShapeDrawable* unitSphereDrawable = new osg::ShapeDrawable(unitSphere);
    osg::Geode* basicShapesGeode = new osg::Geode();
    basicShapesGeode->addDrawable(unitSphereDrawable);
    unitSphereDrawable->setColor(osg::Vec4(1.0, 0, 0, 1.0));

    transform->setPosition(osg::Vec3(15.0, 15.0, 15.0));

    osg::Quat ori =
           osg::Quat(osg::DegreesToRadians(0.0),    osg::Vec3(0,1,0)) *
           osg::Quat(osg::DegreesToRadians(0.0),   osg::Vec3(1,0,0)) *
           osg::Quat(osg::DegreesToRadians(0.0), osg::Vec3(0,0,1));

    transform->setAttitude(ori);
    transform->addChild(basicShapesGeode);
    root->addChild(transform);

   // create infinit plane

    osg::Box* terrain = new osg::Box(osg::Vec3(0, 0, 0), 1000, 1000, 0.01);
    osg::ShapeDrawable* terrainDraw = new osg::ShapeDrawable(terrain);
    osg::Geode* terrainGeode = new osg::Geode();
    terrainGeode->addDrawable(terrainDraw);
    root->addChild(terrainGeode);
    osg::Image* image = osgDB::readImageFile("/home/guillaume/cpp_workspace/osg_world/gras.jpg");
    osg::Texture2D* tex2d = new osg::Texture2D( image );
    tex2d->setWrap( osg::Texture::WRAP_S, osg::Texture::REPEAT );
    tex2d->setWrap( osg::Texture::WRAP_T, osg::Texture::REPEAT );
    terrainDraw->getOrCreateStateSet()->setTextureAttributeAndModes(0, tex2d, osg::StateAttribute::ON );




    //
    osg::PositionAttitudeTransform * followerOffset = new osg::PositionAttitudeTransform();
    followerOffset->setPosition( osg::Vec3(0.0, -2.0, -1.5) );
    osg::Quat orientation =
           osg::Quat(osg::DegreesToRadians(0.0),    osg::Vec3(0,1,0)) *
           osg::Quat(osg::DegreesToRadians(0.0),   osg::Vec3(1,0,0)) *
           osg::Quat(osg::DegreesToRadians(45.0), osg::Vec3(0,0,1));
    followerOffset->setAttitude(orientation);

    transform->addChild(followerOffset);

    transformAccumulator* tankFollowerWorldCoords = new transformAccumulator();
    tankFollowerWorldCoords->attachToGroup(followerOffset);

    {   // Camera one
        int width = 600;
        int height = 400;
        osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;
        traits->x = 0;
        traits->y = 0;
        traits->width = width;
        traits->height = height;
        traits->doubleBuffer = false;
        traits->sharedContext = 0;
        traits->pbuffer = true;
        traits->readDISPLAY();
        osg::GraphicsContext *gc = osg::GraphicsContext::createGraphicsContext(traits.get());
        camera1->setGraphicsContext(gc);
        //camera->setClearColor(osg::Vec4(0.0f,0.0f,0.0f,0.0f));
        camera1->setDrawBuffer(GL_FRONT);
        camera1->setReadBuffer(GL_FRONT);
        camera1->setViewport(new osg::Viewport(0, 0, width, height));
        double fovy, aspectRatio, near, far;
        camera1->getProjectionMatrixAsPerspective(fovy, aspectRatio, near, far);
        double newAspectRatio = double(traits->width) / double(traits->height);
        double aspectRatioChange = newAspectRatio / aspectRatio;
        if (aspectRatioChange != 1.0){
            camera1->getProjectionMatrix() *= osg::Matrix::scale(1.0/aspectRatioChange,1.0,1.0);
        }
        camera1->setViewMatrixAsLookAt(osg::Vec3d(1, 0, 0), osg::Vec3d(0, 0, 0), osg::Vec3d(0, 0, 0) );

        osg::ref_ptr<VideoRecord> snapImageDrawCallback = new VideoRecord(9800);
        camera1->setPostDrawCallback(snapImageDrawCallback.get());
        view1->setCamera(camera1);
        view1->setSceneData( root );
        viewer.addView(view1);
    }
    { // Camera two

        int width = 600;
        int height = 400;
        osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;
        traits->x = 0;
        traits->y = 0;
        traits->width = width;
        traits->height = height;
        traits->doubleBuffer = false;
        traits->sharedContext = 0;
        traits->pbuffer = true;
        traits->readDISPLAY();
        osg::GraphicsContext *gc = osg::GraphicsContext::createGraphicsContext(traits.get());
        camera2->setGraphicsContext(gc);
        //camera->setClearColor(osg::Vec4(0.0f,0.0f,0.0f,0.0f));
        camera2->setDrawBuffer(GL_FRONT);
        camera2->setReadBuffer(GL_FRONT);
        camera2->setViewport(new osg::Viewport(0, 0, width, height));
        double fovy, aspectRatio, near, far;
        camera2->getProjectionMatrixAsPerspective(fovy, aspectRatio, near, far);
        double newAspectRatio = double(traits->width) / double(traits->height);
        double aspectRatioChange = newAspectRatio / aspectRatio;
        if (aspectRatioChange != 1.0){
            camera2->getProjectionMatrix() *= osg::Matrix::scale(1.0/aspectRatioChange,1.0,1.0);
        }

        osg::Vec3d eye(54.8195, 46.7115, 54.6658);
        osg::Vec3d center(54.1876, 46.1825, 54.0993);
        osg::Vec3d up(-0.241998, -0.559683, 0.792585);
        camera2->setViewMatrixAsLookAt( eye, center, up );

        osg::ref_ptr<VideoRecord> snapImageDrawCallback = new VideoRecord(9801);
        camera2->setPostDrawCallback(snapImageDrawCallback.get());
        view2->setCamera(camera2);
        view2->setSceneData( root );
        viewer.addView(view2);
    }

    osgViewer::View* view_window = new osgViewer::View;
    osgGA::TrackballManipulator *Tman = new osgGA::TrackballManipulator();
    view_window->setCameraManipulator(Tman);
    view_window->getCamera()->setViewMatrix(tankFollowerWorldCoords->getMatrix());
    view_window->setSceneData(root);
    view_window->setLightingMode(osg::View::LightingMode::SKY_LIGHT);
    view_window->setUpViewInWindow(0, 0, 600, 400);
    viewer.addView(view_window);


    viewer.setThreadingModel(osgViewer::Viewer::SingleThreaded);
    viewer.realize();
    viewer.frame();
    double dt = 0.01;
    microseconds_type loop_rate_micro_s = microseconds_type(std::size_t(dt * 1000000.0));
    double z=0.0;
    double angle=0.0;

    SetCameraParam set_param_cam(camera1);


    auto start_time = std::chrono::steady_clock::now();
    auto end_time   = start_time + loop_rate_micro_s;
    while( !viewer.done() ){

        start_time = std::chrono::steady_clock::now();
        end_time   = start_time + loop_rate_micro_s;


        set_param_cam.set_camera(10,-50, 15, 0, 0, angle);

        angle=angle+0.01;

        viewer.frame();
        pyramidTwoXForm1->setPosition(osg::Vec3(10, 0, z));

        z=z-0.001;
        std::this_thread::sleep_until(end_time);
    }


}


int main(){
//test_home();
//test_no_dispaly_take_screen_captures();
    test_no_display_multiple_cameras();
}
