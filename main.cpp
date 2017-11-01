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

osg::Node* create_grid(double length, std::size_t num_cells){

    osg::Geode* grid_geode = new osg::Geode();

    double x_step_size = length/(double)num_cells;
    double y_step_size = x_step_size;

    std::size_t num_vertexes = 2*num_cells+2;

    osg::Vec3Array* x_vertexes = new osg::Vec3Array;

    x_vertexes->resize(num_vertexes);

    osg::Vec3Array* y_vertexes = new osg::Vec3Array;
    y_vertexes->resize(num_vertexes );

    double x = -length/2.0;
    for(std::size_t i=0; i < num_vertexes; i=i+2){
        (*x_vertexes)[i].set(x, -length/2.0, 0. );
        (*x_vertexes)[i+1].set(x, length/2.0, 0. );
        x += x_step_size;
    }

    double y = -length/2.0;
    for(std::size_t j = 0; j < num_vertexes; j=j+2){
        (*y_vertexes)[j].set(-length/2.0, y , 0. );
        (*y_vertexes)[j+1].set(length/2.0, y , 0. );
        y += y_step_size;
    }

    osg::Geometry* xlines_geometry = new osg::Geometry;
    xlines_geometry->setUseDisplayList(false);
    xlines_geometry->setVertexArray(x_vertexes);

    osg::Geometry* ylines_geometry = new osg::Geometry;
    ylines_geometry->setUseDisplayList(false);
    ylines_geometry->setVertexArray(y_vertexes);

    osg::Vec4Array* c = new osg::Vec4Array;
    c->push_back( osg::Vec4( 0.5, 0.5, 0.5, 1.0 ) );
    xlines_geometry->setColorArray( c, osg::Array::BIND_OVERALL );


    GLushort idxLines[num_vertexes];

    for(std::size_t i=0; i < num_vertexes; i++){
        idxLines[i]=i;
    }
    xlines_geometry->addPrimitiveSet(new osg::DrawElementsUShort(osg::PrimitiveSet::LINES, num_vertexes, idxLines));
    ylines_geometry->addPrimitiveSet(new osg::DrawElementsUShort(osg::PrimitiveSet::LINES, num_vertexes, idxLines));


    // draw plane
    osg::Geometry* ground_geometry = new osg::Geometry();
    osg::Vec3Array* ground_vertices = new osg::Vec3Array;
    ground_vertices->push_back( osg::Vec3(-length/2.0, -length/2.0, -0.01) );
    ground_vertices->push_back( osg::Vec3(-length/2.0,  length/2.0, -0.01) );
    ground_vertices->push_back( osg::Vec3( length/2.0,  length/2.0, -0.01) );
    ground_vertices->push_back( osg::Vec3( length/2.0, -length/2.0, -0.01) );
    ground_geometry->setVertexArray(ground_vertices);
    osg::DrawElementsUInt* ground_base = new osg::DrawElementsUInt(osg::PrimitiveSet::QUADS, 0);
    ground_base->push_back(3);
    ground_base->push_back(2);
    ground_base->push_back(1);
    ground_base->push_back(0);
    ground_geometry->addPrimitiveSet(ground_base);
    osg::Vec4Array* colors = new osg::Vec4Array;
    colors->push_back(osg::Vec4(90.0/255.0, 90.0/255.0,90.0/255.0, 1.0f) ); //index 0 red
    ground_geometry->setColorArray(colors, osg::Array::BIND_OVERALL );




    grid_geode->addDrawable(ground_geometry);
    grid_geode->addDrawable(xlines_geometry);
    grid_geode->addDrawable(ylines_geometry);

    grid_geode->getOrCreateStateSet()->setMode( GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::PROTECTED );
    return grid_geode;

}


int main()
{
   osg::Group* root = new osg::Group();


   osg::Node* node = create_grid(100, 25);
   root->addChild(node);

   osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;
   traits->x = 0;
   traits->y = 0;
   traits->width = 600;
   traits->height = 400;
   traits->windowDecoration = true;
   traits->doubleBuffer = true;
   traits->samples = 8;
   osg::ref_ptr<osg::GraphicsContext> gc = osg::GraphicsContext::createGraphicsContext(traits.get());
   if (gc.valid())
            {
                osg::notify(osg::INFO)<<"  GraphicsWindow has been created successfully."<<std::endl;
            }
            else
            {
                osg::notify(osg::NOTICE)<<"  GraphicsWindow has not been created successfully."<<std::endl;
            }


   osgViewer::Viewer viewer;
   viewer.setSceneData( root );
   viewer.getCamera()->setGraphicsContext(gc);
   viewer.getCamera()->setViewport(new osg::Viewport(0,0, 600, 400));
   viewer.setCameraManipulator(new osgGA::TrackballManipulator());
   viewer.realize();

   while( !viewer.done() )
   {
      viewer.frame();
   }
}
