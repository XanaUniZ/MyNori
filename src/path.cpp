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

    Color3f recursiveLi(const Scene* scene, Sampler* sampler, const Ray3f& ray,
                        long int n_bounce, float term_prob) const {
    Color3f Lo(0.);
    // Find the surface that is visible in the requested direction
    Intersection its;

    // Pathtracing Recusion
    bool has_interseced = scene->rayIntersect(ray, its);
    if (!has_interseced)
        return scene->getBackground(ray);

    else if(its.mesh->isEmitter())
        return DirectRadiance(&its, &ray.o);

    else{
        // Russian roulete to choose termination
        if ( n_bounce < 2 || sampler->next1D() > term_prob){
            //continue
            BSDFQueryRecord BSDFQuery(its.toLocal(-ray.d),its.uv);

            Point2f sample = sampler->next2D();
            Color3f brdfSample = its.mesh->getBSDF()->sample(BSDFQuery, sample);

            if (brdfSample.isZero() || brdfSample.hasNaN()) {   
                return Color3f(0.0f);
            } 

            // now create a new ray with the sampled direction
            Ray3f new_ray = Ray3f(its.p, its.toWorld(BSDFQuery.wo));
            Color3f throughput = brdfSample;
            term_prob = 1. - std::min(throughput.maxCoeff(), 0.95f);
            return throughput * recursiveLi(scene, sampler, new_ray, n_bounce+1, term_prob);
        }
        else
            return Color3f(0.);
    }

        return Lo;
    }

    Color3f Li(const Scene* scene, Sampler* sampler, const Ray3f& ray) const {
       
        return recursiveLi(scene, sampler, ray, 0, 0.1);
    }

    std::string toString() const {
        return "Direct Material Integrator []";
    }
};

NORI_REGISTER_CLASS(PathTracing, "path");

NORI_NAMESPACE_END
