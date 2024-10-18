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

    Color3f Li(const Scene* scene, Sampler* sampler, const Ray3f& ray) const {
        Color3f Lo(0.);
        // Find the surface that is visible in the requested direction
        Intersection its;
        if (!scene->rayIntersect(ray, its))
            return scene->getBackground(ray);

        float pdflight;
        EmitterQueryRecord emitterRecord(its.p);
        // Sample a ligth in the scene
        float random_01 = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
        const Emitter* em = scene->sampleEmitter(random_01, pdflight);


        // Sample the point sources, getting its radiance and direction
        Color3f Le = em->sample(emitterRecord, sampler->next2D(), 0.);

        Ray3f shadow_ray = Ray3f(its.p, emitterRecord.p - its.p);
        Intersection shadow_ray_its;

        if (its.mesh->isEmitter()){
            cout << its.mesh->isEmitter() << endl;
            EmitterQueryRecord selfEmitterRecord(its.p);
            Lo += its.mesh->getEmitter()->eval(selfEmitterRecord);
        }

        // Perform a visibility query (shadow ray) and compute intersection
        if ((!scene->rayIntersect(shadow_ray, shadow_ray_its) ||
            ((emitterRecord.p - its.p).norm() < shadow_ray_its.t))){
        // if (!scene->rayIntersect(ray_to_light, ray_to_light_its)){
            // Evaluate the BSDF using the outgoing and incoming directions
            BSDFQueryRecord bsdfRecord(
                its.toLocal(-ray.d),
                its.toLocal(emitterRecord.wi),
                its.uv,
                ESolidAngle
            );

            // Accumulate incident light * foreshortening * BSDF term
            Lo += (Le * its.shFrame.n.dot(emitterRecord.wi) *
                its.mesh->getBSDF()->eval(bsdfRecord)) / pdflight;
        }

        return Lo;
    }

    std::string toString() const {
        return "Direct Whitted Integrator []";
    }
};

NORI_REGISTER_CLASS(DirectEmitterSampling, "direct_ems");

NORI_NAMESPACE_END
