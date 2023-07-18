#include <nori/integrator.h>
#include <nori/scene.h>
#include <pcg32.h>

NORI_NAMESPACE_BEGIN

class AOIntegrator : public Integrator {
public:
    
    AOIntegrator(const PropertyList& props) {
        // nothing to do
    }
    
    Color3f Li(const Scene* scene, Sampler* sampler, const Ray3f& ray) const {
        /* Find the surface that is visible in the requested direction */
        Intersection its;
        if (!scene->rayIntersect(ray, its))
            return Color3f(0.0f);

        // intersect, start sample on the hemisphere
        Point3f pos = its.p;
        float sum = 0;
        pcg32 rng;
        for (int i = 0; i < sampleNum; i++) {
            float xi1 = rng.nextFloat(), xi2 = rng.nextFloat();
            float theta = acos(sqrt(xi1)), phi = 2 * M_PI * xi2;
            Vector3f sampleDir(sin(theta) * cos(phi), sin(theta) * sin(phi), cos(theta));
            // to world space
            sampleDir = its.geoFrame.toWorld(sampleDir);
            Ray3f shadowRay(pos, sampleDir, 0.001, FLT_MAX);
            if (!scene->rayIntersect(shadowRay))
                sum += 1;
        }
        sum /= sampleNum;
        return Color3f(sum);
    }
    
    std::string toString() const {
        return "AOIntegrator[]";
    }
private:
    uint32_t sampleNum = 25;
};

NORI_REGISTER_CLASS(AOIntegrator, "ao");

NORI_NAMESPACE_END