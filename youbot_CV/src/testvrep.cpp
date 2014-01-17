#include "pointcloud_save.h"

//FIXME: should be both vrep native and ros plugin.
//it's basically a mess.

void vrepPlaceMesh()
{
	vrepLib=loadVrepLibrary("/home/hjeldin/DEV/youbot-stack/youbot_CV/lib/libv_rep.so");
	if(vrepLib == NULL)
	{
		std::cout << "WRONNNG" << std::endl;
		exit(1);
	}

	int wat = getVrepProcAddresses(vrepLib);
	if(wat == NULL)
	{
		std::cout << "dunno"<< std::endl;
		exit(1);
	}

    simFloat** vertices;
    simInt* verticesSizes;
    simInt** indices;
    simInt* indicesSizes;
    simChar** names;
    simInt elementCount=simImportMesh(0,"./mesh.obj",0,0.0001f,1.0f,&vertices,&verticesSizes,&indices,&indicesSizes,NULL,&names);
    std::cout << "imported? " << elementCount << std::endl;
    if (elementCount>0)
    {
        const float grey[3]={0.5f,0.5f,0.5f};
        for (int i=0;i<elementCount;i++)
        {
            simInt shapeHandle=simCreateMeshShape(2,20.0f*3.1415f/180.0f,vertices[i],verticesSizes[i],indices[i],indicesSizes[i],NULL);
            simSetObjectName(shapeHandle,names[i]);
            simSetShapeColor(shapeHandle,"",0,grey);
            simReleaseBuffer(names[i]);
            simReleaseBuffer((simChar*)indices[i]);
            simReleaseBuffer((simChar*)vertices[i]);
        }
        simReleaseBuffer((simChar*)names);
        simReleaseBuffer((simChar*)indicesSizes);
        simReleaseBuffer((simChar*)indices);
        simReleaseBuffer((simChar*)verticesSizes);
        simReleaseBuffer((simChar*)vertices);
    }	
}