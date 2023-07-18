#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/sampler.h>
#include <pcg32.h>

NORI_NAMESPACE_BEGIN
class WhittedIntegrator : public Integrator {
public:
	WhittedIntegrator(const PropertyList& props) {
        // nothing to do
    }

    Color3f Li(const Scene* scene, Sampler* sampler, const Ray3f& ray) const {

        Intersection its;
        Color3f emitRadiance(0.f);
        if (!scene->rayIntersect(ray, its))
            return Color3f(0.f);
        if (its.mesh->isEmitter() && its.shFrame.n.dot(-ray.d) > 0)
            emitRadiance = its.mesh->getEmitter()->sample();
   
        // no emitter, completely dark
        if (scene->getEmitters().empty()) return Color3f(0.f);

        // diffuse material
        if (its.mesh->getBSDF()->isDiffuse())
        {
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
                // if not self occluded
                if (!(itsShadow.mesh == emitterMesh || (itsShadow.mesh == its.mesh
                    && itsShadow.primIdx == its.primIdx)))
                    return emitRadiance;
            }

            // radiance
            Vector3f dxy = samplePos - its.p;
            float distSquare = dxy.squaredNorm();
            dxy.normalize();
            float cos1 = its.shFrame.n.dot(dxy), cos2 = nEmit.dot(-dxy);
            // orientation error
            if (cos1 <= 0 || cos2 <= 0)
                return emitRadiance;
            float Gxy = (cos1 * cos2) / distSquare;
            // dxy, ray.d to local space
            auto wi = its.shFrame.toLocal(dxy);
            auto wo = its.shFrame.toLocal(-ray.d.normalized());
            Color3f refRadiance = its.mesh->getBSDF()->eval(BSDFQueryRecord(wi, wo, ESolidAngle)) *
                Gxy * emitterMesh->getEmitter()->sample();
            return emitRadiance + (refRadiance / (pdfEmitterChoose * pdfEmitter));
        }
        
        // not diffuse, use russian roulette to decide whether sample
        float p = sampler->next1D();
        if (p > 0.95) return Color3f(0.f);
        
        BSDFQueryRecord sampleQuery(its.shFrame.toLocal(-ray.d.normalized()));
        Color3f weight = its.mesh->getBSDF()->sample(sampleQuery, sampler->next2D());
        return weight * Li(scene, sampler, Ray3f(its.p, its.shFrame.toWorld(sampleQuery.wo))) / 0.95;
    }

    std::string toString() const {
        return "WhittedIntegrator[]";
    }
};

NORI_REGISTER_CLASS(WhittedIntegrator, "whitted");

NORI_NAMESPACE_END