#include <nori/warp.h>
#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/emitter.h>
#include <nori/bsdf.h>

NORI_NAMESPACE_BEGIN

class PathTracingNEE : public Integrator {
public:
    PathTracingNEE(const PropertyList& props) {
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

    Color3f EmitterSampling(const Intersection * sceneIts, const Ray3f& sceneRay,
                            const Scene* scene, Sampler* sampler) const {
        float pdflight;
        const Emitter* sampled_em = scene->sampleEmitter(sampler->next1D(), pdflight);
        // const Emitter* sampled_em = scene->importanceSampleEmitterIntensive(random_01, pdflight, &emitterRecord, sampler);
        // const Emitter* sampled_em = scene->importanceSampleEmitter(random_01, pdflight);
        EmitterQueryRecord emitterRecord(sceneIts->p);

        Color3f res = Color3f(0.);

        // Sample the point sources, getting its radiance and direction
        Color3f Le = sampled_em->sample(emitterRecord, sampler->next2D(), 0.);


        Ray3f shadow_ray = Ray3f(sceneIts->p, emitterRecord.wi.normalized());
        Intersection shadow_ray_its;

        // Perform a visibility query (shadow ray) and compute intersection
        if ((!scene->rayIntersect(shadow_ray, shadow_ray_its) ||
            (emitterRecord.dist <= shadow_ray_its.t+0.1))){

            BSDFQueryRecord bsdfRecord(
                sceneIts->toLocal(-sceneRay.d),
                sceneIts->toLocal(emitterRecord.wi),
                sceneIts->uv,
                ESolidAngle
            );

            // Accumulate incident light * foreshortening * BSDF term
            Color3f bsdf = sceneIts->mesh->getBSDF()->eval(bsdfRecord);
            res += (Le * sceneIts->shFrame.n.dot(emitterRecord.wi) *
                bsdf) / pdflight;
        }
        else{
            // cout << "SHADOW -> " << shadow_ray_its.p << endl;
        }


        return res;
    }

    Color3f recursiveLi(const Scene* scene, Sampler* sampler, const Ray3f& ray,
                        long int n_bounce, float term_prob, bool directLight) const {
        Color3f Lo(0.);
        // Find the surface that is visible in the requested direction
        Intersection its;

        // Pathtracing Recusion
        bool has_interseced = scene->rayIntersect(ray, its);
        if (!has_interseced)
            return scene->getBackground(ray);

        else if(its.mesh->isEmitter() && directLight)
            return DirectRadiance(&its, &ray.o);

        else{ // Intersected with a surface

            // Emitter Sampling ray
            Lo += EmitterSampling(&its, ray, scene, sampler);

            // BSDF Sampling
            // Russian roulete to choose termination
            if ( n_bounce < 2 || sampler->next1D() > -1){
                // cout << "Nbounces: " << n_bounce << endl;
                // BSDF ray
                BSDFQueryRecord BSDFQuery(its.toLocal(-ray.d),its.uv);

                Point2f sample = sampler->next2D();
                Color3f brdfSample = its.mesh->getBSDF()->sample(BSDFQuery, sample);

                // if (!(brdfSample.isZero() || brdfSample.hasNaN())) {   
                //     return Color3f(0.);
                // } 

                Color3f throughput = brdfSample;
                if (BSDFQuery.measure == EDiscrete){
                    directLight = true;
                } else{
                    directLight = false;
                }
                

                // now create a new ray with the sampled BSDF direction
                Ray3f new_ray = Ray3f(its.p, its.toWorld(BSDFQuery.wo));
                term_prob = 1. - std::min(throughput.maxCoeff(), 0.95f);
                Lo += throughput * recursiveLi(scene, sampler, new_ray, n_bounce+1, term_prob,directLight);
            }   
        }
        return Lo;
    }

    Color3f Li(const Scene* scene, Sampler* sampler, const Ray3f& ray) const {
        
        return recursiveLi(scene, sampler, ray, 0, 0.1,true);
    }

    std::string toString() const {
        return "Direct Material Integrator []";
    }
};

NORI_REGISTER_CLASS(PathTracingNEE, "path_nee");

NORI_NAMESPACE_END
