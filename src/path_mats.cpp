#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/sampler.h>

NORI_NAMESPACE_BEGIN

class MatSampleIntegrator : public Integrator {
public:
	MatSampleIntegrator(const PropertyList& props) {
        // nothing to do
    }

    Color3f Li(const Scene* scene, Sampler* sampler, const Ray3f& ray) const {
        Intersection its;
        if (scene->getEmitters().empty()) return Color3f(0.f);
        if (!scene->rayIntersect(ray, its))
            return Color3f(0.f);
        if (its.mesh->isEmitter() && its.shFrame.n.dot(-ray.d) > 0)
            return its.mesh->getEmitter()->sample();
        

        // hit something, start iterate
        Ray3f itRay = ray;
        Color3f accumulateR(1.f);

        // simple recursive version
        if (!m_iter)
        {
            if (sampler->next1D() > m_endP)
                return Color3f(0.f);
            BSDFQueryRecord rec(its.shFrame.toLocal(-itRay.d.normalized()));
            accumulateR *= its.mesh->getBSDF()->sample(rec, sampler->next2D());
            return accumulateR * Li(scene, sampler, Ray3f(its.p, its.shFrame.toWorld(rec.wo))) / m_endP;
        }

        // iterative version, first three intersection, do not RR
        for (int i = 0; i < 3; i++)
        {
            BSDFQueryRecord rec(its.shFrame.toLocal(-itRay.d.normalized()));
            accumulateR *= its.mesh->getBSDF()->sample(rec, sampler->next2D());
            itRay = Ray3f(its.p, its.shFrame.toWorld(rec.wo));
            // ray intersect
            if (!scene->rayIntersect(itRay, its))
                return Color3f(0.f);
            if (its.mesh->isEmitter() && its.shFrame.n.dot(-itRay.d) > 0)
                return accumulateR * its.mesh->getEmitter()->sample();
        }

        // RR
        while (sampler->next1D() < (1 - m_endP))
        {
            BSDFQueryRecord rec(its.shFrame.toLocal(-itRay.d.normalized()));
            accumulateR *= (its.mesh->getBSDF()->sample(rec, sampler->next2D()) /(1 - m_endP));
            itRay = Ray3f(its.p, its.shFrame.toWorld(rec.wo));
            // ray intersect
            if (!scene->rayIntersect(itRay, its))
                return Color3f(0.f);
            if (its.mesh->isEmitter() && its.shFrame.n.dot(-itRay.d) > 0)
                return accumulateR * its.mesh->getEmitter()->sample();
        }

        // RR terminate
        return Color3f(0.f);

    }

    std::string toString() const {
        return "MatSampleIntegrator[]";
    }

private:
    float m_endP = 0.01;
    bool m_iter = true;

};
NORI_REGISTER_CLASS(MatSampleIntegrator, "path_mats");
NORI_NAMESPACE_END