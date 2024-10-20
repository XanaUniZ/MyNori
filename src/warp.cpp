/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob

    Nori is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License Version 3
    as published by the Free Software Foundation.

    Nori is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#include <nori/warp.h>
#include <nori/vector.h>
#include <nori/frame.h>
#include <math.h>

NORI_NAMESPACE_BEGIN

Point2f Warp::squareToUniformSquare(const Point2f &sample) {
    return sample;
}

float Warp::squareToUniformSquarePdf(const Point2f &sample) {
    if (!(sample.array() >= 0).all() && (sample.array() <= 1).all()){
        cout << sample[0] << endl;
        cout << sample[1] << endl;
        cout << endl;
    }
    return ((sample.array() >= 0).all() && (sample.array() <= 1).all()) ? 1.0f : 0.0f;
}

Point2f Warp::squareToTent(const Point2f &sample) {
    throw NoriException("Warp::squareToTent() is not yet implemented!");
}

float Warp::squareToTentPdf(const Point2f &p) {
    throw NoriException("Warp::squareToTentPdf() is not yet implemented!");
}

Point2f Warp::squareToUniformDisk(const Point2f &sample) {
    throw NoriException("Warp::squareToUniformDisk() is not yet implemented!");
}

float Warp::squareToUniformDiskPdf(const Point2f &p) {
    throw NoriException("Warp::squareToUniformDiskPdf() is not yet implemented!");
}

// float sign (Point2f p1, Point2f p2, Point2f p3)
// {
//     return (p1.x() - p3.x()) * (p2.y() - p3.y()) - (p2.x() - p3.x()) * (p1.y() - p3.y());
// }

// bool PointInTriangle (Point2f pt, Point2f v1, Point2f v2, Point2f v3)
// {
//     float d1, d2, d3;
//     bool has_neg, has_pos;

//     d1 = sign(pt, v1, v2);
//     d2 = sign(pt, v2, v3);
//     d3 = sign(pt, v3, v1);

//     has_neg = (d1 < 0) || (d2 < 0) || (d3 < 0);
//     has_pos = (d1 > 0) || (d2 > 0) || (d3 > 0);

//     return !(has_neg && has_pos);
// }

Point2f Warp::squareToUniformTriangle(const Point2f& sample) {
    Point2f res;
    float r1 = sample[0];
    float r2 = sample[1];

    // Calculate barycentric coordinates
    float alpha = 1.0f - sqrt(r1);
    float beta = (1.0f - r2) * sqrt(r1);
    float gamma = r2 * sqrt(r1);


    res[0] = beta; 
    res[1] = 1.0f - alpha - beta;
 
    return res;  // Return the 2D Cartesian point
}


float Warp::squareToUniformTrianglePdf(const Point2f& p) {
    // Check if the point is inside the triangle by ensuring the barycentric coordinates are valid
    if (p[0] >= 0.0f && p[1] >= 0.0f && (p[0] + p[1]) <= 1.0f) {
        return 2.0f;  // Constant PDF for a uniform distribution on the triangle
    } else {
        return 0.0f;  // Outside the triangle
    }
}


Vector3f Warp::squareToUniformSphere(const Point2f &sample) {
    Vector3f res;
    float  theta = acos(1 - 2 * sample[0]); 
    float phi = 2 * M_PI * sample[1]; 
    res[0] = cos(phi) * sin(theta);
    res[1] = sin(phi) * sin(theta);
    res[2] = cos(theta);
    return res;
}

float Warp::squareToUniformSpherePdf(const Vector3f &v) {
    if(v.norm() != 1.0f)
        return 0.0f;
    
    // return abs(sin(atan2(v[0], v[1]))) / (4*M_PI);
    // return abs(acos(v[2])) / (4*M_PI);
    // std::cout << v[1] << std::endl;
    return abs(v[1]) / (4*M_PI); 
}

Vector3f Warp::squareToUniformHemisphere(const Point2f &sample) {
    throw NoriException("Warp::squareToUniformHemisphere() is not yet implemented!");
}

float Warp::squareToUniformHemispherePdf(const Vector3f &v) {
    throw NoriException("Warp::squareToUniformHemispherePdf() is not yet implemented!");
}

Vector3f Warp::squareToCosineHemisphere(const Point2f &sample) {
    Vector3f res;
    float theta = asin(sqrt(sample[1])); 
    float phi = 2*M_PI*sample[0]; 
    res[0] = cos(phi) * sin(theta);
    res[2] = sin(phi) * sin(theta);
    res[1] = cos(theta);

    return res;
}

float Warp::squareToCosineHemispherePdf(const Vector3f &v) {
    if((v.norm() != 1.0f) || v[2] < 0)
        return 0.0f;
    float theta = M_PI/4 - asin(v[2]); 
    return cos(theta)/M_PI;
}

Vector3f Warp::squareToBeckmann(const Point2f &sample, float alpha) {
    throw NoriException("Warp::squareToBeckmann() is not yet implemented!");
}

float Warp::squareToBeckmannPdf(const Vector3f &m, float alpha) {
    throw NoriException("Warp::squareToBeckmannPdf() is not yet implemented!");
}

NORI_NAMESPACE_END
