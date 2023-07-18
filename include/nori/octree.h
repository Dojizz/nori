// my implementation of octree
// currently only support one mesh

#pragma once
#include <nori/mesh.h>
#include <nori/bbox.h>
#include <vector>
#include <queue>
#include <iostream>
#include <algorithm>

NORI_NAMESPACE_BEGIN

// octree node
extern uint32_t node_count;
extern uint32_t leaf_node_count;
extern uint32_t leaf_node_tri_count;
extern uint32_t leaf_node_depth_count;

struct OctreeNode {
	typedef TBoundingBox<Point3f> BBox;
	typedef std::vector<uint32_t> V;

	OctreeNode() {};
	OctreeNode(BBox box, unsigned depth, Mesh* mesh)
		: m_bbox(box), m_depth(depth), m_mesh(mesh) {}
	OctreeNode(BBox box, V tris, unsigned depth, Mesh* mesh)
		: m_bbox(box), m_tris(tris), m_depth(depth), m_mesh(mesh) {};

	/// build from the given mesh
	//void build(Mesh* mesh); 
	BBox& GetSubBbox(int i, int j, int k);
	bool RayIntersect(Ray3f& ray, Intersection& its, bool shadowRay, uint32_t& idx);

	/// recursively build from given triangles
	static OctreeNode* build(BBox box,const V &tris, unsigned depth, Mesh* mesh);
	static inline int GetSubNum(int i, int j, int k) { return (i) + (j << 1) + (k << 2); }
	static bool CompareNode(OctreeNode* n1, OctreeNode* n2);
	static const unsigned tri_thresh = 10;
	static const unsigned d_thresh = 10;
	//static uint32_t node_count;
	
	// immediately initialized
	BBox m_bbox;
	V m_tris;
	unsigned m_depth;
	Mesh* m_mesh;

	// delay
	bool m_leaf;
	OctreeNode** m_children;
};

NORI_NAMESPACE_END