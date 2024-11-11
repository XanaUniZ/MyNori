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
// ? Do we need to do this ?
Point2f Warp::squareToTent(const Point2f &sample) {
    // float x, y;

    // // Para el componente x
    // if (sample[0] < 0.5f) {
    //     x = sqrt(2.0f * sample[0]) - 1.0f;  
    // } else {
    //     x = 1.0f - sqrt(2.0f * (1.0f - sample[0]));  
    // }

    // // Para el componente y
    // if (sample[1] < 0.5f) {
    //     y = sqrt(2.0f * sample[1]) - 1.0f;  
    // } else {
    //     y = 1.0f - sqrt(2.0f * (1.0f - sample[1]));  
    // }

    // return Point2f(x, y); 
}

float Warp::squareToTentPdf(const Point2f &p) {
    // float pdf_x, pdf_y;

    // // Para el componente x
    // if (p[0] >= -1.0f && p[0] <= 1.0f) {
    //     pdf_x = 1.0f - fabs(p[0]);  // La probabilidad disminuye hacia los bordes
    // } else {
    //     pdf_x = 0.0f;  
    // }

    // // Para el componente y
    // if (p[1] >= -1.0f && p[1] <= 1.0f) {
    //     pdf_y = 1.0f - fabs(p[1]);  // La probabilidad disminuye hacia los bordes
    // } else {
    //     pdf_y = 0.0f;  
    // }

    // return pdf_x * pdf_y;  // La PDF total es el producto de las PDFs de x e y
}



Point2f Warp::squareToUniformDisk(const Point2f &sample) {
    Point2f res;
    float r = sqrt(sample[0]);        
    float theta = 2.0f * M_PI * sample[1]; 

    // Cartesians again
    res[0] = r * cos(theta);  
    res[1] = r * sin(theta);  

    return res;
}
float Warp::squareToUniformDiskPdf(const Point2f &p) {
    float distSquared = pow(p[0],2) + pow(p[1],2);  // Distance from origin

    if (distSquared <= 1.0f) {
        return 1.0f / M_PI;  // Uniform PDF inside the disk
    } else {
        return 0.0f;  // Outside the unit disk
    }
}

Point2f Warp::squareToUniformTriangle(const Point2f& sample) {
    Point2f res;
    float r1 = sample[0];
    float r2 = sample[1];

    // Calculate barycentric coordinates
    float alpha = 1.0f - sqrt(r1);
    float beta = (1.0f - r2) * sqrt(r1);
    float gamma = r2 * sqrt(r1);

    res[0] = alpha; 
    res[1] = beta;
 
    return res;  // Return the 2D Cartesian point
}

float Warp::squareToUniformTrianglePdf(const Point2f& p) {
    // Check if the point is inside the triangle
    if (p[0] >= 0.0f && p[1] >= 0.0f && (p[0] + p[1]) <= 1.0f) {
        return 2.0f;  // Constant PDF for a uniform distribution on the triangle
    } else {
        return 0.0f;  // Outside the triangle
    }
}


Vector3f Warp::squareToUniformSphere(const Point2f &sample) {
    float  theta = acos(1 - 2 * sample[0]); 
    float phi = 2 * M_PI * sample[1]; 
    float x = cos(phi) * sin(theta);
    float y = sin(phi) * sin(theta);
    float z = cos(theta);
    return Vector3f(x, y, z);
}

float Warp::squareToUniformSpherePdf(const Vector3f &v) {
    if(abs(v.norm() - 1.) > 0.00001f){
        return 0.0; 
    }
    return 1.0f / (4.0f*M_PI);
}

// ? ask xavi how to do the half of this one.
Vector3f Warp::squareToUniformHemisphere(const Point2f &sample) {
    // Sample[0] is used to determine the azimuthal angle phi
    float phi = 2.0f * M_PI * sample[0];  // Azimuthal angle between [0, 2π]
    
    // Calculation of cosTheta for uniform sampling
    float cosTheta =sample[1]; 
    
    float sinTheta = sqrt(1.0f - cosTheta * cosTheta);  // Compute sin(theta) from cos(theta)
    
    // Convert from spherical coordinates to Cartesian coordinates
    float x = sinTheta * cos(phi);
    float y = sinTheta * sin(phi);
    float z = cosTheta;

    // Return the 3D vector (x, y, z) that lies on the upper hemisphere (z >= 0)
    return Vector3f(x, y, z);
}



float Warp::squareToUniformHemispherePdf(const Vector3f &v) {
    // Ensure that v lies on the upper hemisphere, i.e., z >= 0
    if(abs(v.norm() - 1.) > 0.01f){
        return 0.0; 
    }
    
    if (v[2] >= 0.0f) {
        return 1.0f / (2.0f * M_PI);  // Uniform PDF for the hemisphere
    } else {
        return 0.0f;  // PDF is zero for points outside the hemisphere
    }
}


Vector3f Warp::squareToCosineHemisphere(const Point2f &sample) {
    float theta = asin(sqrt(sample.x()));
    float phi = 2 * M_PI * sample.y();
    float x = sin(theta) * cos(phi);
    float y = sin(theta) * sin(phi);
    float z = cos(theta);
    if (z < 0.0f) {
        z = -z;
    }
    return Vector3f(x, y, z);
    
}

float Warp::squareToCosineHemispherePdf(const Vector3f &v) {
    float p = v.z() / ( M_PI); //v.z()=cos(theta)
    if (v.z() < 0.0f) {
        return 0.0f;
    }
    else {
        return p;
    }


}


Vector3f Warp::squareToBeckmann(const Point2f &sample, float alpha) {
    // Sample theta_h using inverse CDF of Beckmann distribution
    float theta_h = atanf(alpha * sqrtf(-logf(1.0f - sample[0])));
    
    // Sample phi uniformly in [0, 2pi]
    float phi = 2.0f * M_PI * sample[1];
    
    // Convert spherical coordinates to Cartesian coordinates
    float sin_theta_h = sinf(theta_h);
    float x = sin_theta_h * cosf(phi);
    float y = sin_theta_h * sinf(phi);
    float z = cosf(theta_h);
    
    // Return the sampled vector on the hemisphere
    return Vector3f(x, y, z);
}



float Warp::squareToBeckmannPdf(const Vector3f &m, float alpha) {
    // Compute the cosine of the angle theta_h
    float cos_theta_h = m.z(); // z component is cos(theta_h)
    
    if (cos_theta_h <= 0) {
        return 0.0f; // Points below the hemisphere have zero probability
    }
    
    // Compute the Beckmann distribution function D(omega_h)
    float tan_theta_h = sqrtf(1.0f - cos_theta_h * cos_theta_h) / cos_theta_h;
    float exponent = - (tan_theta_h * tan_theta_h) / (alpha * alpha);
    float D = expf(exponent) / (M_PI * alpha * alpha * powf(cos_theta_h, 4));
    
    // Return the PDF as D(omega_h) * cos(theta_h)
    return D * cos_theta_h;
}


NORI_NAMESPACE_END
