#include <nori/warp.h>
#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/emitter.h>
#include <nori/bsdf.h>

NORI_NAMESPACE_BEGIN

class PathTracing : public Integrator {
public:
	PathTracing(const PropertyList& props) {
		/* No parameters this time */
	}

    Color3f DirectRadiance(const Intersection * its, const Point3f* origin) const{
        Color3f res = Color3f(0.);
        // If we intersect a ligth return direct radiance
        EmitterQueryRecord selfEmitterRecord(*origin);
        selfEmitterRecord.p = its->p;
        selfEmitterRecord.wi = (selfEmitterRecord.p - selfEmitterRecord.ref).normalized();
        selfEmitterRecord.dist = its->t;

        selfEmitterRecord.uv = its->uv;
        selfEmitterRecord.n = its->geoFrame.n;
        selfEmitterRecord.pdf = its->mesh->getEmitter()->pdf(selfEmitterRecord);

        Color3f direct_radiance = its->mesh->getEmitter()->eval(selfEmitterRecord);
        res += direct_radiance;
        return res;
    }

    Color3f Li(const Scene* scene, Sampler* sampler, const Ray3f& ray) const {
        Color3f Lo(0.0f);   // the radiance we will return
        int depth = 1;
        float survivalProb;
        Color3f throughput(1.0f);
        Ray3f bouncingRay = ray;
        Intersection its;
        while (true) {
            if (!scene->rayIntersect(bouncingRay, its)) {
                Lo += scene->getBackground(bouncingRay) * throughput;
                break;
            }
            if (its.mesh->isEmitter()) {
                Lo += DirectRadiance(&its, &ray.o) * throughput;
                break;
            }
            // We intersected with a surface
            Point2f sample = sampler->next2D();
            BSDFQueryRecord bsdfQR(its.toLocal(-bouncingRay.d), sample);
            Color3f brdfSample = its.mesh->getBSDF()->sample(bsdfQR, sample);
            // check if the brdf sample is valid (absorbed or invalid samples are not valid)
            if (brdfSample.isZero() || brdfSample.hasNaN()) { 
                break;
            }
            // now create a new ray with the sampled direction
            bouncingRay = Ray3f(its.p, its.toWorld(bsdfQR.wo));
            throughput *= brdfSample;
            if (depth > 2) {    // we want to ensure that the path has at least  bounces
                // start the russian roulette
                // The more BSDF color, the more probability of continuing (clampped at 0.95)
                survivalProb = std::min(throughput.maxCoeff(), 0.95f);
                if (sampler->next1D() > survivalProb) { // this is the russian roulette
                    break;  // if the ray dies, we stop the loop
                } 
                else {
                    throughput /= survivalProb; // if the ray survives, we need to update the throughput
                }
            }
            depth++;
        }
        return Lo;
    }

    std::string toString() const {
        return "Path Tracing []";
    }
};

NORI_REGISTER_CLASS(PathTracing, "path");
NORI_NAMESPACE_END