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

class ReferenceFrame{

public:

    ReferenceFrame(osg::Vec3 position, osg::Vec3 orientation, double radius=0.1, double length=5.0){
        x_cylinder = new osg::Cylinder(osg::Vec3(length/2.0,0,0), radius, length);
        y_cylinder = new osg::Cylinder(osg::Vec3(0,length/2.0,0), radius, length);
        z_cylinder = new osg::Cylinder(osg::Vec3(0,0,length/2.0), radius, length);

        x_cylinder->setRotation(osg::Quat(M_PI/2.0, osg::Vec3(0,1,0)));
        y_cylinder->setRotation(osg::Quat(M_PI/2.0, osg::Vec3(1,0,0)));

        x_cylin_drawable = new osg::ShapeDrawable(x_cylinder);
        y_cylin_drawable = new osg::ShapeDrawable(y_cylinder);
        z_cylin_drawable = new osg::ShapeDrawable(z_cylinder);

        x_cylin_drawable->setColor(osg::Vec4(1.0, 0.0, 0.0, 1.0));
        y_cylin_drawable->setColor(osg::Vec4(0.0, 1.0, 0.0, 1.0));
        z_cylin_drawable->setColor(osg::Vec4(0.0, 0.0, 1.0, 1.0));

        geode = new osg::Geode();
        geode->addDrawable(x_cylin_drawable);
        geode->addDrawable(y_cylin_drawable);
        geode->addDrawable(z_cylin_drawable);

        transform = new osg::PositionAttitudeTransform();
        transform->setPosition(position);
        osg::Quat q_orientation =
               osg::Quat(orientation[0], osg::Vec3(1,0,0)) *
               osg::Quat(orientation[1], osg::Vec3(0,1,0)) *
               osg::Quat(orientation[2], osg::Vec3(0,0,1));
        transform->setAttitude(q_orientation);
        transform->addChild(geode);
    }

    void update_position(double x, double y, double z){
        _position[0] = x;
        _position[1] = y;
        _position[2] = z;
        transform->setPosition(_position);
    }

    void update_orientation(double roll, double pitch, double yaw){
        _orientation = osg::Quat(roll, osg::Vec3(1,0,0)) *
                osg::Quat(pitch, osg::Vec3(0,1,0)) *
                osg::Quat(yaw, osg::Vec3(0,0,1));
        transform->setAttitude(_orientation);
    }

    osg::PositionAttitudeTransform* get_transform(){
        return transform.get();
    }


private:

   osg::ref_ptr<osg::Cylinder> x_cylinder;
   osg::ref_ptr<osg::Cylinder> y_cylinder;
   osg::ref_ptr<osg::Cylinder> z_cylinder;

   osg::ref_ptr<osg::ShapeDrawable> x_cylin_drawable;
   osg::ref_ptr<osg::ShapeDrawable> y_cylin_drawable;
   osg::ref_ptr<osg::ShapeDrawable> z_cylin_drawable;

   osg::ref_ptr<osg::Geode> geode;

   osg::Vec3 _position;
   osg::Quat _orientation;

   osg::ref_ptr<osg::PositionAttitudeTransform> transform;
};


void test_home(){

    osgViewer::Viewer viewer;

    osg::Group* root = new osg::Group();    // root of the scene

    ReferenceFrame reference_frame(osg::Vec3(0, 0, 0), osg::Vec3(0, 0, 0));

    root->addChild(reference_frame.get_transform());


    osgGA::TrackballManipulator *Tman = new osgGA::TrackballManipulator();
    viewer.setUpViewInWindow(0, 0, 600, 400);
    viewer.setCameraManipulator(Tman);
    viewer.setSceneData( root );
    viewer.realize();

    double x = 0;
    double y = 0;
    double z = 0;
    double roll = 0;
    double pitch = 0;
    double yaw = 0;

    double dt = 0.01;
    microseconds_type loop_rate_micro_s = microseconds_type(std::size_t(dt * 1000000.0));


    auto start_time = std::chrono::steady_clock::now();
    auto end_time   = start_time + loop_rate_micro_s;
    while( !viewer.done() ){

        start_time = std::chrono::steady_clock::now();
        end_time   = start_time + loop_rate_micro_s;

        //reference_frame.update_orientation(roll, pitch, yaw);
        //roll=roll+0.001;



        viewer.frame();

        std::this_thread::sleep_until(end_time);
    }
}

int main(){
    test_home();
}
