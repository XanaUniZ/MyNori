#include <nori/warp.h>
#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/emitter.h>
#include <nori/bsdf.h>

NORI_NAMESPACE_BEGIN

class DirectEmitterSampling : public Integrator {
public:
    DirectEmitterSampling(const PropertyList& props) {
        /* No parameters this time */
    }

    Color3f DirectRadiance(const Intersection * its, const Point3f* origin) const{
        Color3f res = Color3f(0.);
        // If we intersect a ligth return direct radiance
        if (its->mesh->isEmitter()){
            EmitterQueryRecord selfEmitterRecord(*origin);
            selfEmitterRecord.p = its->p;
            selfEmitterRecord.wi = (selfEmitterRecord.p - selfEmitterRecord.ref).normalized();
            selfEmitterRecord.dist = its->t;

            selfEmitterRecord.uv = its->uv;
            selfEmitterRecord.n = its->geoFrame.n;
            selfEmitterRecord.pdf = its->mesh->getEmitter()->pdf(selfEmitterRecord);

            Color3f direct_radiance = its->mesh->getEmitter()->eval(selfEmitterRecord);
            res += direct_radiance;
        }
        return res;
    }

    Color3f Li(const Scene* scene, Sampler* sampler, const Ray3f& ray) const {
        Color3f Lo(0.);
        // Find the surface that is visible in the requested direction
        Intersection its;
        if (!scene->rayIntersect(ray, its))
            return scene->getBackground(ray);

        float pdflight;
        EmitterQueryRecord emitterRecord(its.p);

        Lo += DirectRadiance(&its, &ray.o);

        // Sample a ligth in the scene
        float random_01 = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
        const Emitter* em = scene->sampleEmitter(random_01, pdflight);
        // const Emitter* em = scene->importanceSampleEmitterIntensive(random_01, pdflight, &emitterRecord, sampler);
        // const Emitter* em = scene->importanceSampleEmitter(random_01, pdflight);

        // Sample the point sources, getting its radiance and direction
        Color3f Le = em->sample(emitterRecord, sampler->next2D(), 0.);

        Ray3f shadow_ray = Ray3f(its.p, emitterRecord.wi.normalized());
        Intersection shadow_ray_its;

        // if (its.mesh->isEmitter()){
        //     EmitterQueryRecord selfEmitterRecord(Vector3f(0.));
        //     selfEmitterRecord.p = its.p;
        //     selfEmitterRecord.wi = (selfEmitterRecord.p - selfEmitterRecord.ref).normalized();
        //     selfEmitterRecord.dist = its.t;

        //     selfEmitterRecord.uv = its.uv;
        //     selfEmitterRecord.n = its.geoFrame.n;
        //     selfEmitterRecord.pdf = its.mesh->getEmitter()->pdf(selfEmitterRecord);
            
        //     Color3f direct_radiance = its.mesh->getEmitter()->eval(selfEmitterRecord);
        //     Lo += direct_radiance;
        // }

        // Perform a visibility query (shadow ray) and compute intersection
        if ((!scene->rayIntersect(shadow_ray, shadow_ray_its) ||
            (emitterRecord.dist <= shadow_ray_its.t+0.1))){

            BSDFQueryRecord bsdfRecord(
                its.toLocal(-ray.d),
                its.toLocal(emitterRecord.wi),
                its.uv,
                ESolidAngle
            );

            // Accumulate incident light * foreshortening * BSDF term
            Color3f bsdf = its.mesh->getBSDF()->eval(bsdfRecord);
            Lo += (Le * its.shFrame.n.dot(emitterRecord.wi) *
                bsdf) / pdflight;
        }
        else{
            // cout << "SHADOW -> " << shadow_ray_its.p << endl;
        }

        return Lo;
    }

    std::string toString() const {
        return "Direct Whitted Integrator []";
    }
};

NORI_REGISTER_CLASS(DirectEmitterSampling, "direct_ems");

NORI_NAMESPACE_END
