#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/sampler.h>

NORI_NAMESPACE_BEGIN

class EmsSampleIntegrator : public Integrator {
public:
    EmsSampleIntegrator(const PropertyList& props) {
        // nothing to do
    }

    Color3f sampleEmitter(const Intersection its, const Scene* scene, Sampler* sampler, const Ray3f& ray) const {
        Mesh* emitterMesh = scene->getEmitters()[floor(sampler->next1D() * scene->getEmitters().size())];
        float pdfEmitterChoose = 1.f / scene->getEmitters().size();

        // sample on the emitter
        Point3f sample(sampler->next1D(), sampler->next1D(), sampler->next1D());
        Vector3f nEmit;
        float pdfEmitter;
        uint32_t idxEmit = 0;
        Point3f samplePos = emitterMesh->sampleUniformPts(sampler, nEmit, pdfEmitter, idxEmit);

        // visibility
        Ray3f shadowRay(its.p, samplePos - its.p, 1e-6, 1 - 1e-6);
        Intersection itsShadow;
        if (scene->rayIntersect(shadowRay, itsShadow))
        {
            // if not self occluded, then it's truly occluded
            if (!(itsShadow.mesh == emitterMesh || (itsShadow.mesh == its.mesh
                && itsShadow.primIdx == its.primIdx)))
                return Color3f(0.f);
        }

        // radiance
        Vector3f dxy = samplePos - its.p;
        float distSquare = dxy.squaredNorm();
        dxy.normalize();
        float cos1 = its.shFrame.n.dot(dxy), cos2 = nEmit.dot(-dxy);
        // orientation error
        if (cos1 <= 0 || cos2 <= 0)
            return Color3f(0.f);
        float Gxy = (cos1 * cos2) / distSquare;
        // dxy, ray.d to local space
        auto wi = its.shFrame.toLocal(dxy);
        auto wo = its.shFrame.toLocal(-ray.d.normalized());
        Color3f refRadiance = its.mesh->getBSDF()->eval(BSDFQueryRecord(wi, wo, ESolidAngle)) *
            Gxy * emitterMesh->getEmitter()->sample();
        return (refRadiance / (pdfEmitterChoose * pdfEmitter));
    }
     
    Color3f Li_recur(const Scene* scene, Sampler* sampler, const Ray3f& ray) const {
        Intersection its;
        if (!scene->rayIntersect(ray, its))
            return Color3f(0.f);
        if (its.mesh->isEmitter() && its.shFrame.n.dot(-ray.d) > 0)
            return its.mesh->getEmitter()->sample();


        // hit something, start iterate
        Ray3f itRay = ray;

        if (sampler->next1D() > 1 - m_endP)
            return Color3f(0.f);
        Color3f L_light = sampleEmitter(its, scene, sampler, ray);
        // BRDF sampling
        BSDFQueryRecord rec(its.shFrame.toLocal(-ray.d.normalized()));
        Color3f weight = (its.mesh->getBSDF()->sample(rec, sampler->next2D()));
        itRay = Ray3f(its.p, its.shFrame.toWorld(rec.wo));
        Color3f L_BRDF = Li_recur(scene, sampler, itRay);
        return (L_light + weight * L_BRDF) / ((1 - m_endP));
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

        if (sampler->next1D() > 1 - m_endP)
            return Color3f(0.f);
        Color3f L_light = sampleEmitter(its, scene, sampler, ray);
        // BRDF sampling
        BSDFQueryRecord rec(its.shFrame.toLocal(-ray.d.normalized()));
        Color3f weight = (its.mesh->getBSDF()->sample(rec, sampler->next2D()));
        itRay = Ray3f(its.p, its.shFrame.toWorld(rec.wo));
        Color3f L_BRDF = Li_recur(scene, sampler, itRay);
        return 0.5 * (L_light + weight * L_BRDF) / ((1 - m_endP));

    }

    std::string toString() const {
        return "MatSampleIntegrator[]";
    }

private:
    float m_endP = 0.01;

};
NORI_REGISTER_CLASS(EmsSampleIntegrator, "path_ems");
NORI_NAMESPACE_END