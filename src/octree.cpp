#include <nori/octree.h>

NORI_NAMESPACE_BEGIN
uint32_t node_count = 0;
uint32_t leaf_node_count = 0;
uint32_t leaf_node_tri_count = 0;
uint32_t leaf_node_depth_count = 0;

bool compare(std::pair<int, float> p1, std::pair<int, float> p2)
{
	return p1.second < p2.second;
}

OctreeNode::BBox& OctreeNode::GetSubBbox(int i, int j, int k)
{
	float half_x = (m_bbox.max[0] - m_bbox.min[0]) / 2;
	float half_y = (m_bbox.max[1] - m_bbox.min[1]) / 2;
	float half_z = (m_bbox.max[2] - m_bbox.min[2]) / 2;
	Point3f min(m_bbox.min[0] + half_x * i,
		m_bbox.min[1] + half_y * j,
		m_bbox.min[2] + half_z * k);
	Point3f max(m_bbox.min[0] + half_x * (i + 1),
		m_bbox.min[1] + half_y * (j + 1),
		m_bbox.min[2] + half_z * (k + 1));
	return BBox(min, max);
}

//void OctreeNode::build(Mesh* mesh)
//{
//	// init
//	m_depth = 1;
//	m_bbox = mesh->getBoundingBox();
//	unsigned tri_num = mesh->getTriangleCount();
//	for (int i = 0; i < tri_num; i++)
//		m_tris.push_back(i);
//	m_mesh = mesh;
//
//	// small mesh, directly return, suppose index is 0~num-1
//	if (tri_num < tri_thresh)
//	{
//		m_leaf = true;
//		m_children = nullptr;
//	}
//
//	// deal with eight children
//	m_leaf = false;
//	m_children = new OctreeNode*[8];
//	for (int i = 0; i < 2; i++)
//		for (int j = 0; j < 2; j++)
//			for (int k = 0; k < 2; k++)
//			{
//				BBox box = GetSubBbox(i, j, k);
//				// select triangles
//				V tris;
//				for (int n = 0; n < m_tris.size(); n++)
//				{
//					// triangle overlaps with box
//					if(box.overlaps(m_mesh->getBoundingBox(m_tris[n])))
//						tris.push_back(m_tris[n]);
//				}
//				m_children[GetSubNum(i, j, k)] = build(box, tris, m_depth + 1, m_mesh);
//			}
//}

OctreeNode* OctreeNode::build(BBox b,const V &t, unsigned d, Mesh* m)
{
	if (t.size() == 0)
		return nullptr;

	std::cout << "start building node: " << node_count << endl;
	node_count++;
	OctreeNode* node = new OctreeNode(b, d, m);
	
	// only few triangles, or max depth
	if (t.size() <= tri_thresh || d >= d_thresh)
	{
		std::cout << "reach leaf node, depth: " << node->m_depth << endl;
		std::cout << "leaf node triangle number: " << t.size() << endl;
		leaf_node_count++;
		leaf_node_depth_count += d;
		leaf_node_tri_count += t.size();
		node->m_leaf = true;
		node->m_children = nullptr;
		node->m_tris = t;
		return node;
	}

	// split to eight children
	node->m_leaf = false;
	node->m_children = new OctreeNode * [8];
	for (int i = 0; i < 2; i++)
		for (int j = 0; j < 2; j++)
			for (int k = 0; k < 2; k++)
			{
				float half_x = (node->m_bbox.max[0] - node->m_bbox.min[0]) / 2;
				float half_y = (node->m_bbox.max[1] - node->m_bbox.min[1]) / 2;
				float half_z = (node->m_bbox.max[2] - node->m_bbox.min[2]) / 2;
				Point3f min(node->m_bbox.min[0] + half_x * i,
					node->m_bbox.min[1] + half_y * j,
					node->m_bbox.min[2] + half_z * k);
				Point3f max(node->m_bbox.min[0] + half_x * (i + 1),
					node->m_bbox.min[1] + half_y * (j + 1),
					node->m_bbox.min[2] + half_z * (k + 1));
				// strangely if I use the function, it returns a wield box, totally confused...
				//BBox box(node->GetSubBbox(i, j, k)); 
				BBox box(min, max);
				V tris;
				for (int n = 0; n < t.size(); n++)
				{
					if(box.overlaps(m->getBoundingBox(t[n])))
						tris.push_back(t[n]);
				}
				node->m_children[GetSubNum(i, j, k)] = build(box, tris, d + 1, m);
			}
	// clear the node's m_tris, as they are not leaf nodes, the m_tris is useless
	return node;
}

// use a queue to calculate the intersection with ray
// add nullptr check!
bool OctreeNode::RayIntersect(Ray3f& ray, Intersection& its, bool shadowRay, uint32_t& idx_)
{
	std::queue<OctreeNode*> tool_queue;
	std::vector<OctreeNode*> result_vec;
	std::vector<std::pair<int, float>> node_order;
	tool_queue.push(this);

	// get intersect leaf node
	int count = 0;
	while (!tool_queue.empty())
	{
		OctreeNode* node = tool_queue.front();
		tool_queue.pop();
		// whether intersect with this node
		float t_near, t_far;
		if (!node->m_bbox.rayIntersect(ray, t_near, t_far))
			continue;
		// if leaf, push into result_queue
		if (node->m_leaf)
		{
			node_order.push_back(std::pair<int, float>(count, t_near));
			count++;
			result_vec.push_back(node);
			continue;
		}
		// not leaf, push back non null children
		for (int i = 0; i < 8; i++)
			// check if children[i] is nullptr
			if (node->m_children[i])
				tool_queue.push(node->m_children[i]);
	}


	// sort result_vec by m_nearT
	std::sort(node_order.begin(), node_order.end(), compare);
	
	// deal with intersect leaf nodes, check triangles
	bool foundIntersection = false;
	for (int j = 0; j < node_order.size(); j++)
	{
		OctreeNode *node = result_vec[node_order[j].first];
		// before check triangles, we can check the bbox first
		if (!node->m_bbox.rayIntersect(ray))
			continue;

		// whether intersect with leaf's triangle
		for (int i = 0; i < node->m_tris.size(); i++)
		{
			auto idx = node->m_tris[i];
			float u, v, t;
			if (m_mesh->rayIntersect(idx, ray, u, v, t))
			{
				if (shadowRay)
					return true;
				ray.maxt = its.t = t;
				its.uv = Point2f(u, v);
				its.mesh = m_mesh;
				idx_ = idx;
				foundIntersection = true;
			}
		}

		// if we find the intersection, no need to check the remaining nodes
		if (foundIntersection)
			return true;
	}
	return foundIntersection;
}

NORI_NAMESPACE_END