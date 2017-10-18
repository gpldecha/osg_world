#include <osg/NodeCallback>
#include <osg/PositionAttitudeTransform>
#include <osgViewer/Viewer>
#include <osg/MatrixTransform>
#include <osgDB/ReadFile> 
#include <osgGA/TrackballManipulator>
#include <osgGA/StateSetManipulator>
#include <osgViewer/CompositeViewer>
#include <osg/CameraNode>
#include "opencv2/opencv.hpp"



#include "KeyboardHandler.h"

bool manuallyPlaceCamera = false;

void toggleView()
{
   if (! manuallyPlaceCamera)
      manuallyPlaceCamera = true;
   else
      manuallyPlaceCamera = false;
}


void createView (osgViewer::CompositeViewer *viewer, osg::ref_ptr<osg::Group> scene, osgGA::TrackballManipulator* Tman,
                 int x, int y, int width, int height)
{
            double left,right,top,bottom,near,far, aspectratio;
            double frusht, fruswid, fudge;
            bool gotfrustum;
            osgViewer::View* view = new osgViewer::View;
            viewer->addView(view);
            view->setCameraManipulator(Tman);

            view->setSceneData(scene.get());
            view->getCamera()->setViewport(new osg::Viewport(x,y, width,height));
            view->getCamera()-> getProjectionMatrixAsFrustum(left,right,bottom,top,near,far);
            if (gotfrustum){
                aspectratio = (double)width/(double)height;
                frusht = top - bottom;
                fruswid = right - left;
                fudge = frusht*aspectratio/fruswid;
                right = right*fudge;
                left = left*fudge;
                view->getCamera()-> setProjectionMatrixAsFrustum(left, right, bottom, top, near,far);
            }
}

class VideoRecord : public osg::CameraNode::DrawCallback{

    public:

        VideoRecord(){
            _snapImageOnNextFrame = false;
            image = new osg::Image;
        }
        virtual void operator () (const osg::CameraNode& camera) const{
            std::cout<< "take a picture" << std::endl;
            int x,y,width,height;
            x = camera.getViewport()->x();
            y = camera.getViewport()->y();
            width = camera.getViewport()->width();
            height = camera.getViewport()->height();
            image->readPixels(x,y,width,height,GL_RGB,GL_UNSIGNED_BYTE);
            cv::Mat cvImg(image->t(), image->s(), CV_8UC3);
            cvImg.data = (uchar*)image->data();
            cv::flip(cvImg, cvImg, 0);
            cv::imshow("video", cvImg);
            if(cv::waitKey(30) >= 0)
                return;


       }
    protected:

    std::string _filename;
    mutable bool _snapImageOnNextFrame;
    osg::ref_ptr<osg::Image> image;
    cv::Mat cvImg;

};


int main()
{
   osg::Node* groundNode = NULL;
   osg::Node* tankNode = NULL;
   osg::Group* root = new osg::Group();
   osg::PositionAttitudeTransform* tankXform;

   groundNode = osgDB::readNodeFile("../NPS_Data/Models/JoeDirt/JoeDirt.flt");
   tankNode = osgDB::readNodeFile("../NPS_Data/Models/t72-tank/t72-tank_des.flt");

   // Create green Irish sky
   osg::ClearNode* backdrop = new osg::ClearNode;
   backdrop->setClearColor(osg::Vec4(0.0f,0.8f,0.0f,1.0f));
   root->addChild(backdrop);
   root->addChild(groundNode);

   tankXform = new osg::PositionAttitudeTransform();
   root->addChild(tankXform);
   tankXform->addChild(tankNode);

   tankXform->setPosition( osg::Vec3(10,10,8) );
   tankXform->setAttitude( osg::Quat(osg::DegreesToRadians(-45.0), osg::Vec3(0,0,1) ) );

   osgGA::TrackballManipulator *Tman1 = new osgGA::TrackballManipulator();
   osgGA::TrackballManipulator *Tman2 = new osgGA::TrackballManipulator();


   //osgImg->readPixels( 0, 0, width, height, GL_BGR, GL_UNSIGNED_BYTE );
   //cv::Mat cvImg(osgImg->t(), osgImg->s(), CV_8UC3);
   //cvImg.data = (uchar*)osgImg->data();

   // Flipping because of different origins
   //cv::flip(cvImg, cvImg, 0);

   osg::ref_ptr<VideoRecord> snapImageDrawCallback = new VideoRecord();

   // construct the viewer.
    osgViewer::CompositeViewer viewer;

    {
        osgViewer::View* view = new osgViewer::View;
        view->setName("View one");
        viewer.addView(view);
        view->setUpViewInWindow(0, 0, 600, 400);
        view->setSceneData( root );

        osgGA::TrackballManipulator *Tman = new osgGA::TrackballManipulator();
        osg::Vec3 centre = osg::Vec3(10.0,10.0,10.0);
        osg::Vec3 up = osg::Vec3(0.0,0.0,1.0);
        osg::Vec3 eye = osg::Vec3 (10,0.0, 10.0);
        Tman->setHomePosition(eye, centre, up);
        view->setCameraManipulator(Tman);


        osg::ref_ptr<osgGA::StateSetManipulator> statesetManipulator = new osgGA::StateSetManipulator;
        statesetManipulator->setStateSet(view->getCamera()->getOrCreateStateSet());
        view->addEventHandler( statesetManipulator.get() );
        view->getCamera()->setPostDrawCallback(snapImageDrawCallback.get());


    }
   /* {
        osgViewer::View* view = new osgViewer::View;
        view->setName("View two");
        viewer.addView(view);
        view->setUpViewOnSingleScreen(0);
        view->setSceneData( root );


        osgGA::TrackballManipulator *Tman = new osgGA::TrackballManipulator();
        osg::Vec3 centre = osg::Vec3(0.0,0.0,10.0);
        osg::Vec3 up = osg::Vec3(0.0,0.0,1.0);
        osg::Vec3 eye = osg::Vec3 (0.0,0.0, 10.0);
        Tman->setHomePosition(eye, centre, up);
        view->setCameraManipulator(Tman);
        osg::ref_ptr<osgGA::StateSetManipulator> statesetManipulator = new osgGA::StateSetManipulator;
        statesetManipulator->setStateSet(view->getCamera()->getOrCreateStateSet());
        view->addEventHandler( statesetManipulator.get() );
    }*/

   viewer.realize();
   while( !viewer.done() )
   {

       viewer.frame();
      // viewer1.frame();

   }
}
