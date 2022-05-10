#include <iostream>
#include <fstream>
#include <array>

#include "Eigen.h"
#include "VirtualSensor.h"

struct Vertex
{
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	// position stored as 4 floats (4th component is supposed to be 1.0)
	Vector4f position;
	// color stored as 4 unsigned char
	Vector4uc color;
};

bool WriteMesh(Vertex* vertices, unsigned int width, unsigned int height, const std::string& filename)
{
	float edgeThreshold = 0.01f; // 1cm

	// TODO 2: use the OFF file format to save the vertices grid (http://www.geomview.org/docs/html/OFF.html)
	// - have a look at the "off_sample.off" file to see how to store the vertices and triangles
	// - for debugging we recommend to first only write out the vertices (set the number of faces to zero)
	// - for simplicity write every vertex to file, even if it is not valid (position.x() == MINF) (note that all vertices in the off file have to be valid, thus, if a point is not valid write out a dummy point like (0,0,0))
	// - use a simple triangulation exploiting the grid structure (neighboring vertices build a triangle, two triangles per grid cell)
	// - you can use an arbitrary triangulation of the cells, but make sure that the triangles are consistently oriented
	// - only write triangles with valid vertices and an edge length smaller then edgeThreshold

	// TODO: Get number of vertices
	unsigned int nVertices = width * height;

	// TODO: Determine number of valid faces
	unsigned nFaces = 0; // (width - 1) * (height -1) * 2


	// Write off file
	std::ofstream outFile(filename);
	if (!outFile.is_open()) return false;

	// write header
	outFile << "COFF" << std::endl;

	outFile << "# numVertices numFaces numEdges" << std::endl;

	outFile << nVertices << " " << nFaces << " 0" << std::endl;

	// TODO: save vertices
	outFile << "# list of vertices" << std::endl;
	outFile << "# X Y Z R G B A" << std::endl;
	for(int i = 0; i < nVertices; i++){
		int R = (int)(vertices[i].color(0));
		int G = (int)(vertices[i].color(1));
		int B = (int)(vertices[i].color(2));
		int A = (int)(vertices[i].color(3));
		if (vertices[i].position(0) == MINF){
			outFile << "0.0 0.0 0.0 " << R << " " << G 
			<< " " << B << " " << A << std::endl;
		} else {
			outFile << vertices[i].position(0) << " " << vertices[i].position(1) 
			<< " " << vertices[i].position(2) << " " << R << " " << G 
			<< " " << B << " " << A << std::endl;
		}
	}

	// TODO: save valid faces
	outFile << "# list of faces" << std::endl;
	outFile << "# nVerticesPerFace idx0 idx1 idx2 ..." << std::endl;
	for(int i = 0; i < height; i++){
		for(int j = 0; j < width; j++){
			if (vertices[i*width+j].position(0) != MINF){
				Vector4f X1 = vertices[i*width+j].position;
				Vector4f X2 = vertices[i*width+j+width].position;
				if (j == 0){ // head
					Vector4f X3 = vertices[i*width+j+1].position;
					float d_12 = (X1 - X2).norm();
					float d_13 = (X1 - X3).norm();
					float d_23 = (X2 - X3).norm();
					if (d_12 <= edgeThreshold && d_13 <= edgeThreshold && d_23 <= edgeThreshold){
						outFile << "3 " << i*width+j << " " << i*width+j+width << " " << i*width+j+1 << std::endl;
					}
				} else if (j == width-1){ // tail
					Vector4f X3 = vertices[i*width+j+width-1].position;
					float d_12 = (X1 - X2).norm();
					float d_13 = (X1 - X3).norm();
					float d_23 = (X2 - X3).norm();
					if (d_12 <= edgeThreshold && d_13 <= edgeThreshold && d_23 <= edgeThreshold){
						outFile << "3 " << i*width+j+width-1 << " " << i*width+j+width << " " << i*width+j << std::endl;
					}
				} else { // middle
					Vector4f X3 = vertices[i*width+j+width-1].position;
					float d_12 = (X1 - X2).norm();
					float d_13 = (X1 - X3).norm();
					float d_23 = (X2 - X3).norm();
					if (d_12 <= edgeThreshold && d_13 <= edgeThreshold && d_23 <= edgeThreshold){
						outFile << "3 " << i*width+j+width-1 << " " << i*width+j+width << " " << i*width+j << std::endl;
					}

					X3 = vertices[i*width+j+1].position;
					d_12 = (X1 - X2).norm();
					d_13 = (X1 - X3).norm();
					d_23 = (X2 - X3).norm();
					if (d_12 <= edgeThreshold && d_13 <= edgeThreshold && d_23 <= edgeThreshold){
						outFile << "3 " << i*width+j << " " << i*width+j+width << " " << i*width+j+1 << std::endl;
					}
				}
			}
		}
	}


	// close file
	outFile.close();

	return true;
}

int main()
{
	// Make sure this path points to the data folder
	std::string filenameIn = "../../Data/rgbd_dataset_freiburg1_xyz/"; // small modifications
	std::string filenameBaseOut = "mesh_";

	// load video
	std::cout << "Initialize virtual sensor..." << std::endl;
	VirtualSensor sensor;
	if (!sensor.Init(filenameIn))
	{
		std::cout << "Failed to initialize the sensor!\nCheck file path!" << std::endl;
		return -1;
	}

	// convert video to meshes
	while (sensor.ProcessNextFrame())
	{
		// get ptr to the current depth frame
		// depth is stored in row major (get dimensions via sensor.GetDepthImageWidth() / GetDepthImageHeight())
		float* depthMap = sensor.GetDepth();
		
		// get ptr to the current color frame
		// color is stored as RGBX in row major (4 byte values per pixel, get dimensions via sensor.GetColorImageWidth() / GetColorImageHeight())
		BYTE* colorMap = sensor.GetColorRGBX();

		// get depth intrinsics
		Matrix3f depthIntrinsics = sensor.GetDepthIntrinsics(); // K
		Matrix3f depthIntrinsicsInv = depthIntrinsics.inverse(); // K^-1

		float fX = depthIntrinsics(0, 0);
		float fY = depthIntrinsics(1, 1);
		float cX = depthIntrinsics(0, 2);
		float cY = depthIntrinsics(1, 2);

		// compute inverse depth extrinsics
		Matrix4f depthExtrinsicsInv = sensor.GetDepthExtrinsics().inverse(); // E^-1

		Matrix4f trajectory = sensor.GetTrajectory();
		Matrix4f trajectoryInv = sensor.GetTrajectory().inverse();

		// TODO 1: back-projection
		// write result to the vertices array below, keep pixel ordering!
		// if the depth value at idx is invalid (MINF) write the following values to the vertices array
		// vertices[idx].position = Vector4f(MINF, MINF, MINF, MINF);
		// vertices[idx].color = Vector4uc(0,0,0,0);
		// otherwise apply back-projection and transform the vertex to world space, use the corresponding color from the colormap
		Vertex* vertices = new Vertex[sensor.GetDepthImageWidth() * sensor.GetDepthImageHeight()];
        for(int i = 0; i < sensor.GetDepthImageHeight(); i++){
			for(int j = 0; j < sensor.GetDepthImageWidth(); j++){
				if (depthMap[i*sensor.GetDepthImageWidth()+j] == MINF) {
				vertices[i*sensor.GetDepthImageWidth()+j].position = Vector4f(MINF, MINF, MINF, MINF);
		        vertices[i*sensor.GetDepthImageWidth()+j].color = Vector4uc(0,0,0,0);
			} else {
				// first calculate x',y',z' according to x y and density
                float x_ = j * depthMap[i*sensor.GetDepthImageWidth()+j];
				float y_ = i * depthMap[i*sensor.GetDepthImageWidth()+j];
				float z_ = depthMap[i*sensor.GetDepthImageWidth()+j];
				// E^-1 * K^-1 * (x', y', z')
				Vector3f X_ = Vector3f(x_,y_,z_);
				Vector3f X_c = depthIntrinsicsInv * X_;
				//std::cout << depthIntrinsicsInv << std::endl;

				// extend dimension from 3 to 4
				Vector4f X_h = Vector4f(X_c(0),X_c(1),X_c(2),1.0);
				// multiplied by E^-1
				//std::cout << X_h << std::endl;
				vertices[i*sensor.GetDepthImageWidth()+j].position = trajectoryInv * X_h;
				//std::cout << X_h << std::endl;
				//std::cout << depthExtrinsicsInv * X_h << std::endl;
				//std::cout << trajectoryInv * X_h << std::endl;

				// add color
				// shape of colorMap = 4* m_colorImageWidth*m_colorImageHeight
				unsigned char R = colorMap[(i*sensor.GetDepthImageWidth()+j)*4];
				unsigned char G = colorMap[(i*sensor.GetDepthImageWidth()+j)*4+1];
				unsigned char B = colorMap[(i*sensor.GetDepthImageWidth()+j)*4+2];
				unsigned char A = colorMap[(i*sensor.GetDepthImageWidth()+j)*4+3];
				vertices[i*sensor.GetDepthImageWidth()+j].color = Vector4uc(R,G,B,A);
				}
			}
		}

		// write mesh file
		std::stringstream ss;
		ss << filenameBaseOut << sensor.GetCurrentFrameCnt() << ".off";
		if (!WriteMesh(vertices, sensor.GetDepthImageWidth(), sensor.GetDepthImageHeight(), ss.str()))
		{
			std::cout << "Failed to write mesh!\nCheck file path!" << std::endl;
			return -1;
		}

		// free mem
		delete[] vertices;
	}

	return 0;
}