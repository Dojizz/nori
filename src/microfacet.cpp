/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob
*/

#include <nori/bsdf.h>
#include <nori/frame.h>
#include <nori/warp.h>

NORI_NAMESPACE_BEGIN

class Microfacet : public BSDF {
public:
    Microfacet(const PropertyList &propList) {
        /* RMS surface roughness */
        m_alpha = propList.getFloat("alpha", 0.1f);

        /* Interior IOR (default: BK7 borosilicate optical glass) */
        m_intIOR = propList.getFloat("intIOR", 1.5046f);

        /* Exterior IOR (default: air) */
        m_extIOR = propList.getFloat("extIOR", 1.000277f);

        /* Albedo of the diffuse base material (a.k.a "kd") */
        m_kd = propList.getColor("kd", Color3f(0.5f));

        /* To ensure energy conservation, we must scale the 
           specular component by 1-kd. 

           While that is not a particularly realistic model of what 
           happens in reality, this will greatly simplify the 
           implementation. Please see the course staff if you're 
           interested in implementing a more realistic version 
           of this BRDF. */
        m_ks = 1 - m_kd.maxCoeff();
    }

    Vector3f reflect(Vector3f n, Vector3f wi) const {
        n.normalize();
        wi.normalize();
        Vector3f proj = wi.dot(n) * n;
        return -wi + 2. * proj;
    }

    float G1(const Vector3f wv, const Vector3f wh) const {
        if (wv.dot(wh) / wv.z() <= 0)
            return 0;
        float b = 1.0f / (m_alpha * Frame::tanTheta(wv));
        if (b >= 1.6)
            return 1;
        return (3.535 * b + 2.181 * b * b) / (1. + 2.276 * b + 2.577 * b * b);
    }

    /// Evaluate the BRDF for the given pair of directions
    Color3f eval(const BSDFQueryRecord &bRec) const {
        Vector3f wh = (bRec.wi + bRec.wo).normalized();
        float D = Warp::squareToBeckmannPdf(wh, m_alpha);
        float F = fresnel(wh.dot(bRec.wi), m_extIOR, m_intIOR);
        float G = G1(bRec.wi, wh) * G1(bRec.wo, wh);
        return m_kd * INV_PI + m_ks * D * F * G / (4. * bRec.wi.z() * bRec.wo.z() * wh.z());
    }

    /// Evaluate the sampling density of \ref sample() wrt. solid angles
    float pdf(const BSDFQueryRecord &bRec) const {
        if (Frame::cosTheta(bRec.wo) <= 0)
            return 0;
        Vector3f wh = (bRec.wi + bRec.wo).normalized();
        float Jh = 1 / (4. * abs(wh.dot(bRec.wo)));
        float D = Warp::squareToBeckmannPdf(wh, m_alpha);
        return m_ks * D * Jh + (1 - m_ks) * Frame::cosTheta(bRec.wo) * INV_PI;
    }

    /// Sample the BRDF
    Color3f sample(BSDFQueryRecord &bRec, const Point2f &sample) const {
    	//throw NoriException("MicrofacetBRDF::sample(): not implemented!");
        if (Frame::cosTheta(bRec.wi) <= 0)
            return Color3f(0.0f);
        bRec.wi.normalize();
        // Note: Once you have implemented the part that computes the scattered
        // direction, the last part of this function should simply return the
        // BRDF value divided by the solid angle density and multiplied by the
        // cosine factor from the reflection equation, i.e.
        // return eval(bRec) * Frame::cosTheta(bRec.wo) / pdf(bRec);
        float xi1 = sample[0], xi2 = sample[1];
        // diffuse
        if (xi1 < 1 - m_ks) {
            // reuse
            xi1 = xi1 / (1 - m_ks);
            bRec.wo = Warp::squareToCosineHemisphere(Point2f(xi1, xi2));
        }
        else {
            xi1 = (xi1 - (1 - m_ks)) / (m_ks);
            Vector3f n = Warp::squareToBeckmann(Point2f(xi1, xi2), m_alpha);
            bRec.wo = reflect(n, bRec.wi);
        }
        if (bRec.wo.z() < 0.f) {
            return Color3f(0.0f);
        }
        return eval(bRec) * Frame::cosTheta(bRec.wo) / pdf(bRec);
    }

    bool isDiffuse() const {
        /* While microfacet BRDFs are not perfectly diffuse, they can be
           handled by sampling techniques for diffuse/non-specular materials,
           hence we return true here */
        return true;
    }

    std::string toString() const {
        return tfm::format(
            "Microfacet[\n"
            "  alpha = %f,\n"
            "  intIOR = %f,\n"
            "  extIOR = %f,\n"
            "  kd = %s,\n"
            "  ks = %f\n"
            "]",
            m_alpha,
            m_intIOR,
            m_extIOR,
            m_kd.toString(),
            m_ks
        );
    }
private:
    float m_alpha;
    float m_intIOR, m_extIOR;
    float m_ks;
    Color3f m_kd;
};

NORI_REGISTER_CLASS(Microfacet, "microfacet");
NORI_NAMESPACE_END
