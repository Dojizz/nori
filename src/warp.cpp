/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob
*/

#include <nori/warp.h>
#include <nori/vector.h>
#include <nori/frame.h>

NORI_NAMESPACE_BEGIN

Point2f Warp::squareToUniformSquare(const Point2f &sample) {
    /*Point2f v0(0.f, 0.f), v1(1.f, 0.f), v2(0.5f, 1.f);
    float xi0 = sample[0], xi1 = sample[1];
    float alpha = 1 - sqrt(xi0);
    float beta = xi1 * sqrt(xi0);
    return alpha * v0 + beta * v1 + (1 - alpha - beta) * v2;*/
    return sample;
}

float Warp::squareToUniformSquarePdf(const Point2f &sample) {
    return ((sample.array() >= 0).all() && (sample.array() <= 1).all()) ? 1.0f : 0.0f;
}

Point2f Warp::squareToTent(const Point2f &sample) {
    //throw NoriException("Warp::squareToTent() is not yet implemented!");
    float x = sample[0], y = sample[1];
    float newX = 0, newY = 0;
    if (x < 0.5 && x >= 0)
        newX = pow(2 * x, 0.5) - 1;
    else if (x >= 0.5 && x <= 1)
        newX = 1 - pow(2 - 2 * x, 0.5);
    if (y < 0.5 && y >= 0)
        newY = pow(2 * y, 0.5) - 1;
    else if (y >= 0.5 && y <= 1)
        newY = 1 - pow(2 - 2 * y, 0.5);
    return Point2f(newX, newY);
}

float Warp::squareToTentPdf(const Point2f &p) {
    //throw NoriException("Warp::squareToTentPdf() is not yet implemented!");
    float pX = 1 - abs(p[0]);
    float pY = 1 - abs(p[1]);
    return pX * pY;
}

Point2f Warp::squareToUniformDisk(const Point2f &sample) {
    //throw NoriException("Warp::squareToUniformDisk() is not yet implemented!");
    float x = sample[0], y = sample[1];
    float r = sqrt(x);
    float theta = 2 * M_PI * y;
    return Point2f(r * cos(theta), r * sin(theta));
}

float Warp::squareToUniformDiskPdf(const Point2f &p) {
    // throw NoriException("Warp::squareToUniformDiskPdf() is not yet implemented!");
    if (p[0] * p[0] + p[1] * p[1] <= 1)
        return 1 / M_PI;
    return 0;
}

Vector3f Warp::squareToUniformSphere(const Point2f &sample) {
    //throw NoriException("Warp::squareToUniformSphere() is not yet implemented!");
    float theta = acos(1 - sample[0] * 2);
    float phi = 2 * M_PI * sample[1];
    float x = sin(theta) * cos(phi);
    float y = sin(theta) * sin(phi);
    float z = cos(theta);
    return Vector3f(x, y, z);
}

float Warp::squareToUniformSpherePdf(const Vector3f &v) {
    //throw NoriException("Warp::squareToUniformSpherePdf() is not yet implemented!");
    return 1 / (4 * M_PI);
}

Vector3f Warp::squareToUniformHemisphere(const Point2f &sample) {
    //throw NoriException("Warp::squareToUniformHemisphere() is not yet implemented!");
    float theta = acos(1 - sample[0]);
    float phi = 2 * M_PI * sample[1];
    float x = sin(theta) * cos(phi);
    float y = sin(theta) * sin(phi);
    float z = cos(theta);
    return Vector3f(x, y, z);
}

float Warp::squareToUniformHemispherePdf(const Vector3f &v) {
    //throw NoriException("Warp::squareToUniformHemispherePdf() is not yet implemented!");
    if (v[2] < 0)
        return 0;
    return 1 / (2 * M_PI);
}

Vector3f Warp::squareToCosineHemisphere(const Point2f &sample) {
    //throw NoriException("Warp::squareToCosineHemisphere() is not yet implemented!");
    float theta = acos(1 - 2 * sample[0]) / 2;
    float phi = 2 * M_PI * sample[1];
    float x = sin(theta) * cos(phi);
    float y = sin(theta) * sin(phi);
    float z = cos(theta);
    return Vector3f(x, y, z);
}

float Warp::squareToCosineHemispherePdf(const Vector3f &v) {
    //throw NoriException("Warp::squareToCosineHemispherePdf() is not yet implemented!");
    if (v[2] < 0)
        return 0;
    return v[2] / M_PI;
}

Vector3f Warp::squareToBeckmann(const Point2f &sample, float alpha) {
    //throw NoriException("Warp::squareToBeckmann() is not yet implemented!");
    float theta = atan(sqrt(-alpha * alpha * log(sample[0])));
    float phi = 2 * M_PI * sample[1];
    float x = sin(theta) * cos(phi);
    float y = sin(theta) * sin(phi);
    float z = cos(theta);
    return Vector3f(x, y, z);

}

float Warp::squareToBeckmannPdf(const Vector3f &m, float alpha) {
    //throw NoriException("Warp::squareToBeckmannPdf() is not yet implemented!");
    if (m[2] <= 0)
        return 0;
    float alpha2 = alpha * alpha;
    float cosTheta = m.z();
    float tanTheta2 = (m.x() * m.x() + m.y() * m.y()) / (cosTheta * cosTheta);
    float cosTheta3 = cosTheta * cosTheta * cosTheta;
    float azimuthal = INV_PI;
    float longitudinal = exp(-tanTheta2 / alpha2) / (alpha2 * cosTheta3);
    return azimuthal * longitudinal; 
}

NORI_NAMESPACE_END
