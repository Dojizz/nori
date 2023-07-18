#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/sampler.h>

NORI_NAMESPACE_BEGIN

class MISIntegrator : public Integrator {
public:
    MISIntegrator(const PropertyList& props) {
        // nothing to do
    }

    // sample emitter, also return the pdf by solid angle
    Color3f sampleEmitter(const Intersection its, const Scene* scene, Sampler* sampler, const Ray3f& ray, 
        float &p_light, float &p_brdf) const {
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
        // float Gxy = (cos1 * cos2) / distSquare;
        float Gxy = cos1;
        // dxy, ray.d to local space
        auto wi = its.shFrame.toLocal(dxy);
        auto wo = its.shFrame.toLocal(-ray.d.normalized());
        Color3f refRadiance = its.mesh->getBSDF()->eval(BSDFQueryRecord(wi, wo, ESolidAngle)) *
            Gxy * emitterMesh->getEmitter()->sample();
        p_light = (pdfEmitterChoose * pdfEmitter) * distSquare / cos2;
        p_brdf = its.mesh->getBSDF()->pdf(BSDFQueryRecord(wi, wo, ESolidAngle));
        return (refRadiance);
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
        // RR
        if (sampler->next1D() > 1 - m_endP)
            return Color3f(0.f);
        // L_light = L_light * p_light / (p_light + p_brdf)
        float p_light = 0, p_brdf = 0;
        Color3f L_light = sampleEmitter(its, scene, sampler, ray, p_light, p_brdf);
        L_light = L_light / (p_light + p_brdf);
        if (isnan(L_light[0]) || isnan(L_light[1]) || isnan(L_light[2]))
        {
            L_light = Color3f(0.f);
        }
            

        // BRDF sampling
        BSDFQueryRecord rec(its.shFrame.toLocal(-ray.d.normalized()));
        Color3f weight = (its.mesh->getBSDF()->sample(rec, sampler->next2D()));
        itRay = Ray3f(its.p, its.shFrame.toWorld(rec.wo));
        Color3f L_BRDF = Li(scene, sampler, itRay);
        // itRay hit emitter, p_light != 0
        p_light = 0;
        p_brdf = its.mesh->getBSDF()->pdf(rec);
        Intersection nextIts;
        if (scene->rayIntersect(itRay, nextIts) && nextIts.mesh->isEmitter())
        {
            p_light = (1. / scene->getEmitters().size()) * (1. / nextIts.mesh->getAreaSum());
            p_light *= (nextIts.p - its.p).squaredNorm();
            p_light /= (its.p - nextIts.p).normalized().dot(nextIts.shFrame.n);
        }
        // only apply when its.mesh is diffuse, square is to dixiao the 1 / p_brdf in weight
        if (its.mesh->getBSDF()->isDiffuse())
            L_BRDF = L_BRDF * p_brdf / (p_light + p_brdf);
        if (isnan(L_BRDF[0]) || isnan(L_BRDF[1]) || isnan(L_BRDF[2]))
        {
            //L_BRDF = Color3f(1.f, 0.f, 0.f);
            L_BRDF = Color3f(0.f);
        }
        
        return (L_light + weight * L_BRDF) / ((1 - m_endP));
    }

    std::string toString() const {
        return "MatSampleIntegrator[]";
    }

private:
    float m_endP = 0.05;

};
NORI_REGISTER_CLASS(MISIntegrator, "path_mis");
NORI_NAMESPACE_END