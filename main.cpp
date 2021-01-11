#include <iostream>
#include <fstream>
#include <array>
#include <filesystem>

#include "Eigen.h"

#include "VirtualSensor.h"

namespace fs = std::filesystem;

struct Vertex
{
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	// position stored as 4 floats (4th component is supposed to be 1.0)
	Vector4f position;

	// color stored as 4 unsigned char
	Vector4uc color;
};

float EuclidDistance(Vertex v1, Vertex v2) {
	return sqrt(pow(v1.position[0] - v2.position[0], 2) +
							pow(v1.position[1] - v2.position[1], 2) +
							pow(v1.position[2] - v2.position[2], 2));

}

bool ValidFace(Vertex v1, Vertex v2, Vertex v3, float edgeThreshold) {
	if (v1.position[0] == MINF)
		return false;
	if (v2.position[0] == MINF)
		return false;
	if (v3.position[0] == MINF)
		return false;
	if (EuclidDistance(v1, v2) > edgeThreshold)
		return false;
	if (EuclidDistance(v1, v3) > edgeThreshold)
		return false;
	if (EuclidDistance(v2, v3) > edgeThreshold)
		return false;
}


bool WriteMesh(Vertex* vertices, unsigned int width, unsigned int height, const std::string& filename)
{
	float edgeThreshold = 0.01f; // 1cm

	// TODO 2: use the OFF file format to save the vertices grid
	// (http://www.geomview.org/docs/html/OFF.html) - have a look at the
	// "off_sample.off" file to see how to store the vertices and triangles - for
	// debugging we recommend to first only write out the vertices (set the
	// number of faces to zero) - for simplicity write every vertex to file, even
	// if it is not valid (position.x() == MINF) (note that all vertices in the
	// off file have to be valid, thus, if a point is not valid write out a dummy
	// point like (0,0,0)) - use a simple triangulation exploiting the grid
	// structure (neighboring vertices build a triangle, two triangles per grid
	// cell) - you can use an arbitrary triangulation of the cells, but make sure
	// that the triangles are consistently oriented - only write triangles with
	// valid vertices and an edge length smaller then edgeThreshold

	unsigned int nVertices = width * height;

	// Write off file
	std::ofstream outFile(filename);
	if (!outFile.is_open()) return false;

	std::cout << "Writing mesh file..." << std::endl;

	// write header
	outFile << "COFF" << std::endl;
	int cnt = 0;

	for (int i=0; i<height; i++) {
		for (int j=0; j<width; j++) {
			int idx = i * width + j;
			if (vertices[idx].position[0] != MINF) {
				cnt += 1;
				outFile << vertices[idx].position[0] << " ";
				outFile << vertices[idx].position[1] << " ";
				outFile << vertices[idx].position[2] << " ";
				outFile << ((int)vertices[idx].color[0]) << " ";
				outFile << ((int)vertices[idx].color[1]) << " ";
				outFile << ((int)vertices[idx].color[2]) << " ";
				outFile << ((int)vertices[idx].color[3]) << " ";
				outFile << std::endl;
			} else {
				outFile << "0.0 0.0 0.0 0 0 0 0" << std::endl;
			}
		}
	}

	std::cout << (float)cnt / (height * width) << "% non nil vertices." << std::endl;

	std::cout << "Saving faces..." << std::endl;
	unsigned nFaces = 0;
	
	for (int i=0; i<height-1; i++) {
		for (int j=0; j<width-1; j++) {
			int idx_0 = i * width + j;
			int idx_1 = i * width + j + 1;
			int idx_2 = i * (width + 1) + j;
			int idx_3 = i * (width + 1) + j + 1;
			if (ValidFace(vertices[idx_0], vertices[idx_1], vertices[idx_2], edgeThreshold)) {
				nFaces += 1;
			}
			if (ValidFace(vertices[idx_1], vertices[idx_2], vertices[idx_3], edgeThreshold)) {
				nFaces += 1;
			}
		}
	}
	outFile << nVertices << " " << nFaces << " 0" << std::endl;

	for (int i=0; i<height-1; i++) {
		for (int j=0; j<width-1; j++) {
			int idx_0 = i * width + j;
			int idx_1 = i * width + j + 1;
			int idx_2 = i * (width + 1) + j;
			int idx_3 = i * (width + 1) + j + 1;
			if (ValidFace(vertices[idx_0], vertices[idx_1], vertices[idx_2], edgeThreshold)) {
				outFile << "3 " << idx_0 << " " << idx_1 << " " << idx_2 << std::endl;
			}
			if (ValidFace(vertices[idx_1], vertices[idx_2], vertices[idx_3], edgeThreshold)) {
				outFile << "3 " << idx_1 << " " << idx_3 << " " << idx_2 << std::endl;
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
	std::string filenameIn = "../../../data/rgbd_dataset_freiburg1_xyz/";
	std::string filenameBaseOut = "mesh_";

	// load video
	std::cout << "Initialize virtual sensor..." << std::endl;
	if (!(fs::exists(filenameIn)))
	{
		std::cout << "Failed to initialize the sensor!\nCheck file path!" << std::endl;
		return -1;
	}
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
		Matrix3f depthIntrinsics = sensor.GetDepthIntrinsics();
		float fX = depthIntrinsics(0, 0);
		float fY = depthIntrinsics(1, 1);
		float cX = depthIntrinsics(0, 2);
		float cY = depthIntrinsics(1, 2);

		// compute inverse depth extrinsics
		Matrix4f depthExtrinsicsInv = sensor.GetDepthExtrinsics().inverse();

		Matrix4f trajectory = sensor.GetTrajectory();
		Matrix4f trajectoryInv = sensor.GetTrajectory().inverse();

		// TODO 1: back-projection
		// write result to the vertices array below, keep pixel ordering!
		// if the depth value at idx is invalid (MINF) write the following values to the vertices array
		// vertices[idx].position = Vector4f(MINF, MINF, MINF, MINF);
		// vertices[idx].color = Vector4uc(0,0,0,0);
		// otherwise apply back-projection and transform the vertex to world space, use the corresponding color from the colormap

		// Instantiate array of Vertexes
		unsigned int m_size = sensor.GetDepthImageWidth() * sensor.GetDepthImageHeight();
		Vertex* vertices = new Vertex[m_size];
		int width = sensor.GetColorImageWidth();
		int height = sensor.GetColorImageHeight();

		for (int i=0; i<height; i++) {
			for (int j=0; j<width; j++) {
				int idx = i * width + j;
				Vector4uc color;
				Vector4f position;
				if (depthMap[idx] != MINF) {
					// transfer color map
					for (int k=0; k < 4; k++)
						color[k] = colorMap[idx * 4 + k];
					// perform backprojection
					float depth = depthMap[idx];
					Vector3f tmp = depthIntrinsics.inverse() * depth * Vector3f(i, j, 1);
					Vector4f pos = Vector4f(tmp[0], tmp[1], tmp[2], 1.0f);

					position = trajectoryInv * depthExtrinsicsInv * pos;
				} else {
					color = Vector4uc(0, 0, 0, 0);
					position = Vector4f(MINF, MINF, MINF, MINF);
				}
				vertices[idx].color = color;
				vertices[idx].position = position;
			}
		}
		
		std::cout << "finished processing vertices" << std::endl;

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
