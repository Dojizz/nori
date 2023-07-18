#pragma once

#include <nori/mesh.h>
#include <nori/octree.h>
#include <embree4/rtcore.h>


NORI_NAMESPACE_BEGIN

//void errorFunction(void* userPtr, enum RTCError error, const char* str)
//{
//	printf("error %d: %s\n", error, str);
//}

struct EmAccel {
private:
	
	void InitializeDevice();
	void InitializeScene();

public:
	std::vector<Mesh*> m_meshes;
	EmAccel(std::vector<Mesh*> meshes);
	bool RayIntersect(Ray3f& ray, Intersection& its, bool shadowRay, uint32_t& idx);
};

NORI_NAMESPACE_END