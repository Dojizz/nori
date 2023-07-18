#include <nori/emAccel.h>

NORI_NAMESPACE_BEGIN

static RTCDevice m_device;
static RTCScene m_scene;

void EmAccel::InitializeDevice()
{
	m_device = rtcNewDevice(NULL);
	if (!m_device)
		printf("error %d: cannot create device\n", rtcGetDeviceError(NULL));
	//rtcSetDeviceErrorFunction(m_device, errorFunction, NULL);
}

void EmAccel::InitializeScene()
{
	m_scene = rtcNewScene(m_device);
	// get essential data from the mesh
	for (int i = 0; i < m_meshes.size(); i++) {
		uint32_t verCount = m_meshes[i]->getVertexCount();
		uint32_t triCount = m_meshes[i]->getTriangleCount();
		RTCGeometry geom = rtcNewGeometry(m_device, RTC_GEOMETRY_TYPE_TRIANGLE);
		float* vertices = (float*)rtcSetNewGeometryBuffer(geom,
			RTC_BUFFER_TYPE_VERTEX,
			0,
			RTC_FORMAT_FLOAT3,
			3 * sizeof(float),
			verCount);

		unsigned* indices = (unsigned*)rtcSetNewGeometryBuffer(geom,
			RTC_BUFFER_TYPE_INDEX,
			0,
			RTC_FORMAT_UINT3,
			3 * sizeof(unsigned),
			triCount);

		if (vertices && indices)
		{
			// load vertices data
			for (int j = 0; j < verCount; j++)
			{
				vertices[3 * j] = m_meshes[i]->getVertexPositions()(0, j);
				vertices[3 * j + 1] = m_meshes[i]->getVertexPositions()(1, j);
				vertices[3 * j + 2] = m_meshes[i]->getVertexPositions()(2, j);
			}
			// load indices data
			for (int j = 0; j < triCount; j++)
			{
				indices[3 * j] = m_meshes[i]->getIndices()(0, j);
				indices[3 * j + 1] = m_meshes[i]->getIndices()(1, j);
				indices[3 * j + 2] = m_meshes[i]->getIndices()(2, j);
			}
		}
		rtcCommitGeometry(geom);
		rtcAttachGeometry(m_scene, geom);
		rtcReleaseGeometry(geom);
	}
	rtcCommitScene(m_scene);
}



EmAccel::EmAccel(std::vector<Mesh*> meshes)
{
	m_meshes = meshes;
	InitializeDevice();
	InitializeScene();
}

bool EmAccel::RayIntersect(Ray3f& ray, Intersection& its, bool shadowRay, uint32_t& idx)
{
	struct RTCRayHit rayhit;
	rayhit.ray.org_x = ray.o[0];
	rayhit.ray.org_y = ray.o[1];
	rayhit.ray.org_z = ray.o[2];
	rayhit.ray.dir_x = ray.d[0];
	rayhit.ray.dir_y = ray.d[1];
	rayhit.ray.dir_z = ray.d[2];
	rayhit.ray.tnear = ray.mint;
	rayhit.ray.tfar = ray.maxt;
	rayhit.ray.mask = -1;
	rayhit.ray.flags = 0;
	rayhit.hit.geomID = RTC_INVALID_GEOMETRY_ID;
	rayhit.hit.instID[0] = RTC_INVALID_GEOMETRY_ID;

	// shadow ray, use occluded
	if (shadowRay)
	{
		auto tfar = rayhit.ray.tfar;
		rtcOccluded1(m_scene, &rayhit.ray);
		if (tfar != rayhit.ray.tfar)
			return true;
		return false;
	}

	// normal ray intersect
	rtcIntersect1(m_scene, &rayhit);
	if (rayhit.hit.geomID == RTC_INVALID_GEOMETRY_ID)
		return false;
	// update its.uv, its.mesh, idx, 
	float u, v, t;
	if (m_meshes[rayhit.hit.geomID]->rayIntersect(rayhit.hit.primID, ray, u, v, t));
	{
		its.uv = Point2f(u, v);
		its.mesh = m_meshes[rayhit.hit.geomID];
		idx = rayhit.hit.primID;
		return true;
	}
	return false;
}

NORI_NAMESPACE_END