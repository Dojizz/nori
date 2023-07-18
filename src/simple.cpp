#include <nori/integrator.h>
#include <nori/scene.h>

NORI_NAMESPACE_BEGIN

class SimpleIntegrator : public Integrator {
public:
    SimpleIntegrator(const PropertyList& props) {
        m_pos = props.getPoint("position");
        m_color = props.getColor("energy");
    }

    Color3f Li(const Scene* scene, Sampler* sampler, const Ray3f& ray) const {
        /* Find the surface that is visible in the requested direction */
        Intersection its, itsTmp;
        if (!scene->rayIntersect(ray, its))
            return Color3f(0.0f);

        // intersect, check if in shadow
        Point3f pos = its.p;
        Vector3f d = m_pos - pos;
        Vector3f dNorm = d;
        dNorm.normalize();
        Ray3f shadowRay(pos, d, 0.001, 0.999);
        if (scene->rayIntersect(shadowRay, itsTmp))
            return Color3f(0.0f);
        float cosTheta = its.geoFrame.cosTheta(its.geoFrame.toLocal(dNorm));
        return m_color * clamp(cosTheta, 0., 1.) / (4 * M_PI * M_PI * d.squaredNorm());
    }

    std::string toString() const {
        return "SimpleIntegrator[]";
    }
private:
    Point3f m_pos;
    Color3f m_color;
};

NORI_REGISTER_CLASS(SimpleIntegrator, "simple");
NORI_NAMESPACE_END