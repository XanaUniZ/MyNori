#include <nori/warp.h>
#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/emitter.h>
#include <nori/bsdf.h>

NORI_NAMESPACE_BEGIN

class DirectMaterialSampling : public Integrator {
public:
    DirectMaterialSampling(const PropertyList& props) {
        /* No parameters this time */
    }

    Color3f Li(const Scene* scene, Sampler* sampler, const Ray3f& ray) const {
        Color3f Lo(0.);
        // Find the surface that is visible in the requested direction
        Intersection its;
        if (!scene->rayIntersect(ray, its))
            return scene->getBackground(ray);

        float pdflight;
        EmitterQueryRecord emitterRecord(its.p);

        // If we intersect a ligth return direct radiance 
        if (its.mesh->isEmitter()){
            EmitterQueryRecord selfEmitterRecord(Vector3f(0.));
            selfEmitterRecord.p = its.p;
            selfEmitterRecord.wi = (selfEmitterRecord.p - selfEmitterRecord.ref).normalized();
            selfEmitterRecord.dist = its.t;

            selfEmitterRecord.uv = its.uv;
            selfEmitterRecord.n = its.geoFrame.n;
            selfEmitterRecord.pdf = its.mesh->getEmitter()->pdf(selfEmitterRecord);
            
            Color3f direct_radiance = its.mesh->getEmitter()->eval(selfEmitterRecord);
            Lo += direct_radiance;
        }

        // Sample the direction given by the BSDF
        BSDFQueryRecord initialBSDFQuery(its.toLocal(-ray.d),its.uv);

        float random_01 = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
        its.mesh->getBSDF()->sample(initialBSDFQuery, random_01);

        Intersection itsSampledRay;
        Ray3f sampledRay(its.p, initialBSDFQuery.wo);

        if (!scene->rayIntersect(sampledRay, itsSampledRay)||
            (!itsSampledRay.mesh->isEmitter()))
            return scene->getBackground(ray);
        

        const Emitter* em = itsSampledRay.mesh->getEmitter();
        // const Emitter* em = scene->importanceSampleEmitterIntensive(random_01, pdflight, &emitterRecord, sampler);
        // const Emitter* em = scene->importanceSampleEmitter(random_01, pdflight);

        // Sample the point sources, getting its radiance and direction
        Color3f Le = em->sample(emitterRecord, sampler->next2D(), 0.);


        BSDFQueryRecord bsdfRecord(
            its.toLocal(-ray.d),
            its.toLocal(emitterRecord.wi),
            its.uv,
            ESolidAngle
        );

        // Accumulate incident light * foreshortening * BSDF term
        Color3f bsdf = its.mesh->getBSDF()->eval(bsdfRecord);
        Lo += (Le * its.shFrame.n.dot(emitterRecord.wi) * bsdf);

        return Lo;
    }

    std::string toString() const {
        return "Direct Material Integrator []";
    }
};

NORI_REGISTER_CLASS(DirectMaterialSampling, "direct_mats");

NORI_NAMESPACE_END
