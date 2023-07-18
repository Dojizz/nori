#include <nori/emitter.h>

NORI_NAMESPACE_BEGIN

class AreaLight : public Emitter {
public:
	AreaLight(const PropertyList& propList) {
		m_radiance = propList.getColor("radiance");
	}

	Color3f sample() const {
		return m_radiance;
	}

	std::string toString() const {
		return tfm::format(
			"AreaLight[\n"
			"  radiance = %s\n"
			"]", m_radiance.toString());
	}

private:
	Color3f m_radiance;
};

NORI_REGISTER_CLASS(AreaLight, "area");
NORI_NAMESPACE_END