#include <osg/NodeCallback>
#include <osg/PositionAttitudeTransform>
#include <osgViewer/Viewer>
#include <osg/MatrixTransform>
#include <osgDB/ReadFile>
#include <osgGA/TrackballManipulator>

#include <osg/GLExtensions>
#include <osg/Node>
#include <osg/Geometry>
#include <osg/Notify>
#include <osg/MatrixTransform>
#include <osg/Texture2D>
#include <osg/TextureRectangle>
#include <osg/Stencil>
#include <osg/ColorMask>
#include <osg/Depth>
#include <osg/Billboard>
#include <osg/Material>
#include <osg/AnimationPath>

#include <osgGA/TrackballManipulator>
#include <osgGA/FlightManipulator>
#include <osgGA/DriveManipulator>

#include <osgUtil/SmoothingVisitor>

#include <osgDB/Registry>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>

#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>


struct MyCameraPostDrawCallback : public osg::Camera::DrawCallback
{
    MyCameraPostDrawCallback(osg::Image* image):
        _image(image)
    {
    }

    virtual void operator () (const osg::Camera& /*camera*/) const
    {
        std::cout<< "take picture" << std::endl;
        if (_image && _image->getPixelFormat()==GL_RGBA && _image->getDataType()==GL_UNSIGNED_BYTE)
        {
            // we'll pick out the center 1/2 of the whole image,
            int column_start = _image->s()/4;
            int column_end = 3*column_start;

            int row_start = _image->t()/4;
            int row_end = 3*row_start;


            // and then invert these pixels
            for(int r=row_start; r<row_end; ++r)
            {
                unsigned char* data = _image->data(column_start, r);
                for(int c=column_start; c<column_end; ++c)
                {
                    (*data) = 255-(*data); ++data;
                    (*data) = 255-(*data); ++data;
                    (*data) = 255-(*data); ++data;
                    (*data) = 255; ++data;
                }
            }


            // dirty the image (increments the modified count) so that any textures
            // using the image can be informed that they need to update.
            _image->dirty();
        }
        else if (_image && _image->getPixelFormat()==GL_RGBA && _image->getDataType()==GL_FLOAT)
        {
            // we'll pick out the center 1/2 of the whole image,
            int column_start = _image->s()/4;
            int column_end = 3*column_start;

            int row_start = _image->t()/4;
            int row_end = 3*row_start;

            // and then invert these pixels
            for(int r=row_start; r<row_end; ++r)
            {
                float* data = (float*)_image->data(column_start, r);
                for(int c=column_start; c<column_end; ++c)
                {
                    (*data) = 1.0f-(*data); ++data;
                    (*data) = 1.0f-(*data); ++data;
                    (*data) = 1.0f-(*data); ++data;
                    (*data) = 1.0f; ++data;
                }
            }

            // dirty the image (increments the modified count) so that any textures
            // using the image can be informed that they need to update.
            _image->dirty();
        }

    }

    osg::Image* _image;
};


osg::Camera* create_camera(osg::Node* subgraph, unsigned tex_width, unsigned tex_height,
                   osg::Camera::RenderTargetImplementation renderImplementation,
                   bool useImage, bool useTextureRectangle, bool useHDR,
                   unsigned int samples, unsigned int colorSamples){

    osg::Camera* camera = new osg::Camera;

    // texture to render to and to use for rendering of flag.
    osg::Texture* texture = 0;
    if (useTextureRectangle)
    {
        osg::TextureRectangle* textureRect = new osg::TextureRectangle;
        textureRect->setTextureSize(tex_width, tex_height);
        textureRect->setInternalFormat(GL_RGBA);
        textureRect->setFilter(osg::Texture2D::MIN_FILTER,osg::Texture2D::LINEAR);
        textureRect->setFilter(osg::Texture2D::MAG_FILTER,osg::Texture2D::LINEAR);

        texture = textureRect;
    }
    else
    {
        osg::Texture2D* texture2D = new osg::Texture2D;
        texture2D->setTextureSize(tex_width, tex_height);
        texture2D->setInternalFormat(GL_RGBA);
        texture2D->setFilter(osg::Texture2D::MIN_FILTER,osg::Texture2D::LINEAR);
        texture2D->setFilter(osg::Texture2D::MAG_FILTER,osg::Texture2D::LINEAR);

        texture = texture2D;
    }

    if (useHDR)
    {
        texture->setInternalFormat(GL_RGBA16F_ARB);
        texture->setSourceFormat(GL_RGBA);
        texture->setSourceType(GL_FLOAT);
    }


    // set up the background color and clear mask.
    camera->setClearColor(osg::Vec4(0.1f,0.1f,0.3f,1.0f));
    camera->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    const osg::BoundingSphere& bs = subgraph->getBound();
    if (!bs.valid())
    {
        return camera;
    }

    float znear = 1.0f*bs.radius();
    float zfar  = 3.0f*bs.radius();

    // 2:1 aspect ratio as per flag geometry below.
    float proj_top   = 0.25f*znear;
    float proj_right = 0.5f*znear;

    znear *= 0.9f;
    zfar *= 1.1f;

    // set up projection.
    //camera->setProjectionMatrixAsFrustum(-proj_right,proj_right,-proj_top,proj_top,znear,zfar);


    // set view
    camera->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
    osg::Vec3d eye = osg::Vec3d(0, -10, 10);
    osg::Vec3d center = osg::Vec3d(10, 0, 0);
    osg::Vec3d up = osg::Vec3d(0, 1, 0);

    camera->setViewMatrixAsLookAt(eye, center, up);

    // set viewport
    camera->setViewport(0,0,tex_width,tex_height);

    // set the camera to render before the main camera.
    camera->setRenderOrder(osg::Camera::PRE_RENDER);

    // tell the camera to use OpenGL frame buffer object where supported.
    camera->setRenderTargetImplementation(renderImplementation);


    //if (useImage)
    //{
        osg::Image* image = new osg::Image;
        //image->allocateImage(tex_width, tex_height, 1, GL_RGBA, GL_UNSIGNED_BYTE);
        image->allocateImage(tex_width, tex_height, 1, GL_RGBA, GL_FLOAT);

        // attach the image so its copied on each frame.
        camera->attach(osg::Camera::COLOR_BUFFER, image, samples, colorSamples);

        camera->setPostDrawCallback(new MyCameraPostDrawCallback(image));

        // Rather than attach the texture directly to illustrate the texture's ability to
        // detect an image update and to subload the image onto the texture.  You needn't
        // do this when using an Image for copying to, as a separate camera->attach(..)
        // would suffice as well, but we'll do it the long way round here just for demonstration
        // purposes (long way round meaning we'll need to copy image to main memory, then
        // copy it back to the graphics card to the texture in one frame).
        // The long way round allows us to manually modify the copied image via the callback
        // and then let this modified image by reloaded back.
        texture->setImage(0, image);
    /*}else{
        // attach the texture and use it as the color buffer.
        camera->attach(osg::Camera::COLOR_BUFFER, texture, 0, 0, false,   samples, colorSamples);
    }*/

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

int main(int argc, char** argv){

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


    unsigned int tex_width = 600;
    unsigned int tex_height = 400;
    unsigned int samples = 0;
    unsigned int colorSamples = 0;
    bool useImage = true;
    // while (arguments.read("--image")) { useImage = true; }

    bool useTextureRectangle = false;
    // while (arguments.read("--texture-rectangle")) { useTextureRectangle = true; }

    bool useHDR = false;
    // while (arguments.read("--hdr")) { useHDR = true; }


     osg::Camera::RenderTargetImplementation renderImplementation = osg::Camera::PIXEL_BUFFER;

    osg::PositionAttitudeTransform* loadedModelTransform = new osg::PositionAttitudeTransform();
    loadedModelTransform->setPosition(osg::Vec3(0, -10, 0));
    osg::Camera* camera = create_camera(loadedModelTransform, tex_width, tex_height, renderImplementation, useImage, useTextureRectangle, useHDR, samples, colorSamples);

    root->addChild(loadedModelTransform);
    loadedModelTransform->addChild(camera);

     osgViewer::Viewer viewer;

     viewer.setUpViewInWindow(0, 0, 600, 400);
     viewer.setSceneData( root );
     viewer.setCameraManipulator(new osgGA::TrackballManipulator());
     viewer.realize();

     while( !viewer.done() )
     {
        viewer.frame();
     }

    return 0;
}
