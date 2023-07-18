/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob
*/

#include <nori/bsdf.h>
#include <nori/frame.h>

NORI_NAMESPACE_BEGIN

/// Ideal dielectric BSDF
class Dielectric : public BSDF {
public:
    Dielectric(const PropertyList &propList) {
        /* Interior IOR (default: BK7 borosilicate optical glass) */
        m_intIOR = propList.getFloat("intIOR", 1.5046f);

        /* Exterior IOR (default: air) */
        m_extIOR = propList.getFloat("extIOR", 1.000277f);
    }

    Color3f eval(const BSDFQueryRecord &) const {
        /* Discrete BRDFs always evaluate to zero in Nori */
        return Color3f(0.0f);
    }

    float pdf(const BSDFQueryRecord &) const {
        /* Discrete BRDFs always evaluate to zero in Nori */
        return 0.0f;
    }

    Vector3f refract(const Vector3f wi) const {
        // ray from ext, refract to int
        Vector3f wo(0.f);
        if (Frame::cosTheta(wi) >= 0)
        {
            float sinThetaExt = Frame::sinTheta(wi);
            float sinThetaInt = m_extIOR * sinThetaExt / m_intIOR;
            if (sinThetaInt > 1 || sinThetaInt < -1)
                return wo;
            wo = Vector3f(
                -wi.x(),
                -wi.y(),
                -sqrt((wi.x() * wi.x() + wi.y() * wi.y()) * (1 - sinThetaInt * sinThetaInt)) / sinThetaInt
            );
        }
        // ray from int, refract to ext
        else
        {
            float sinThetaInt = Frame::sinTheta(wi);
            float sinThetaExt = m_intIOR * sinThetaInt / m_extIOR;
            if (sinThetaExt > 1 || sinThetaExt < -1)
                return wo;
            wo = Vector3f(
                -wi.x(),
                -wi.y(),
                sqrt((wi.x() * wi.x() + wi.y() * wi.y()) * (1 - sinThetaExt * sinThetaExt)) / sinThetaExt
            );
        }
        return wo.normalized();
    }

    // bRec.wi should be normalized
    Color3f sample(BSDFQueryRecord &bRec, const Point2f &sample) const {
        // ray from the ext
        bRec.measure = EDiscrete;
        float Ro1 = fresnel(Frame::cosTheta(bRec.wi), m_extIOR, m_intIOR);
        Vector3f refl(-bRec.wi.x(), -bRec.wi.y(), bRec.wi.z());
        Vector3f refr(refract(bRec.wi));
        // full reflect
        if (refr == Vector3f(0.f))
        {
            bRec.wo = refl;
            bRec.eta = 1.f;
            return Color3f(1.0f);
        }

        if (sample[0] <= Ro1)
        {
            bRec.wo = refl;
            bRec.eta = 1.f;
        }
        else
        {
            bRec.wo = refr;
            bRec.eta = Frame::cosTheta(bRec.wi) > 0 ? m_extIOR / m_intIOR : m_intIOR / m_extIOR;
        }
        return Color3f(1.0f);
    }

    std::string toString() const {
        return tfm::format(
            "Dielectric[\n"
            "  intIOR = %f,\n"
            "  extIOR = %f\n"
            "]",
            m_intIOR, m_extIOR);
    }
private:
    float m_intIOR, m_extIOR;
};

NORI_REGISTER_CLASS(Dielectric, "dielectric");
NORI_NAMESPACE_END
