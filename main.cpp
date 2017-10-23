#include <osg/PositionAttitudeTransform>
#include <osg/Group>
#include <osg/Node>
#include <osgDB/ReadFile>
#include <osgViewer/Viewer>

#include <osgText/Font>
#include <osgText/Text>
#include <osg/MatrixTransform>
#include <osg/Geode>
#include <osg/Projection>
#include <osg/ShapeDrawable>
#include <osg/Geometry>
#include <osgGA/TrackballManipulator>

// https://github.com/petercheng00/Indoor-Modeling/tree/master/NPS_Tutorials_src/NPS_Data
// http://trac.openscenegraph.org/projects/osg//wiki/Support/Tutorials
// http://trac.openscenegraph.org/projects/osg/wiki/Support/Tutorials/CameraControl

int main()
{
   osg::Group* root = NULL;
   osg::Node* tankNode = NULL;
   osg::Node* terrainNode = NULL;
   osg::PositionAttitudeTransform* tankXform;
   osg::Vec3 tankPosit;
   osg::Geode* HUDGeode = new osg::Geode();
   osgText::Text* textOne = new osgText::Text();
   osgText::Text* tankLabel = new osgText::Text();
   osg::Projection* HUDProjectionMatrix = new osg::Projection;
   osgViewer::Viewer viewer;


   root = new osg::Group();
   tankNode = osgDB::readNodeFile("/home/guillaume/catkin_ws/src/simulators/world_sim/models/jsz8zitavm-MQ27/MQ-27.obj");
   terrainNode = osgDB::readNodeFile("/home/guillaume/cpp_workspace/test_open_scene_graph/NPS_Data/Models/JoeDirt/JoeDirt.flt");


   osg::MatrixTransform* maTransform = new osg::MatrixTransform();
   maTransform->setMatrix(osg::Matrix::scale(0.01,0.01,0.01));


    maTransform->addChild(tankNode);
    root->addChild(maTransform);


   //tankXform = new osg::PositionAttitudeTransform();
   //tankPosit.set(5,5,8);
   //tankXform->setPosition( tankPosit );

   //tankXform = new osg::PositionAttitudeTransform();
   //tankPosit.set(5,5,8);
   //tankXform->setPosition( tankPosit );
   root->addChild(terrainNode);
   //root->addChild(tankXform);
   //tankXform->addChild(tankNode);


   //viewer.setUpViewer(osgProducer::Viewer::STANDARD_SETTINGS);
   viewer.setUpViewInWindow(0, 0, 600, 400);
   viewer.setSceneData( root );
   viewer.setCameraManipulator(new osgGA::TrackballManipulator());
   viewer.realize();

   while( !viewer.done() )
   {
      viewer.frame();
   }
}
