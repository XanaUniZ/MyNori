#include <nori/warp.h>
#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/emitter.h>
#include <nori/bsdf.h>

NORI_NAMESPACE_BEGIN

class DirectMIS : public Integrator {
public:
    DirectMIS(const PropertyList& props) {
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

    Color3f BSDFSampling(const Scene* scene, const Intersection * sceneIts, 
    const Ray3f& sceneRay, Sampler* sampler) const {
        Color3f res = Color3f(0.);
        float w_bsdf = 0.f;
        float p_mat_mat = 0.f, p_em_mat = 0.f;

        // Sample the direction given by the BSDF
        BSDFQueryRecord initialBSDFQuery(sceneIts->toLocal(-sceneRay.d),sceneIts->uv);

        Point2f sample = sampler->next2D();
        Color3f brdfSample = sceneIts->mesh->getBSDF()->sample(initialBSDFQuery, sample);

        Ray3f sampledRay(sceneIts->p, sceneIts->toWorld((initialBSDFQuery.wo)));

        Intersection itsSampledRay;
        if (!scene->rayIntersect(sampledRay, itsSampledRay)){
            Color3f backgroundColor = scene->getBackground(sampledRay);
            res += backgroundColor * brdfSample;
            return res;
        }

        if (itsSampledRay.mesh->isEmitter()){
            const Emitter* em;
            em = itsSampledRay.mesh->getEmitter();

            // Sample the point sources, getting its radiance and direction
            EmitterQueryRecord emitterRecord(sceneIts->p);
            emitterRecord.p = itsSampledRay.p;
            emitterRecord.wi = (emitterRecord.p - emitterRecord.ref).normalized();
            emitterRecord.dist = (emitterRecord.p - emitterRecord.ref).norm();

            emitterRecord.uv = itsSampledRay.uv;
            emitterRecord.n = itsSampledRay.geoFrame.n;

            Color3f Le = em->eval(emitterRecord);

            // Accumulate incident light * foreshortening * BSDF term
            Color3f bsdf = brdfSample;
            res += (Le * bsdf);
            p_em_mat = em->pdf(emitterRecord);
        }

        p_mat_mat = sceneIts->mesh->getBSDF()->pdf(initialBSDFQuery);
        if (p_em_mat + p_mat_mat > Epsilon){
            // compute the weight
            w_bsdf = p_mat_mat / (p_em_mat + p_mat_mat);
        }
        res *= w_bsdf;

        return res;
    }

    Color3f EmitterSampling(const Emitter* em, float pdflight, EmitterQueryRecord* emitterRecord,
                    const Scene* scene, const Intersection * sceneIts, 
                    const Ray3f& sceneRay, Sampler* sampler) const {
        
        Color3f res = Color3f(0.);
        float w_emmiter = 0.f;
        float p_em_em = 0.f, p_mat_em = 0.f;

        // Sample the point sources, getting its radiance and direction
        Color3f Le = em->sample(*emitterRecord, sampler->next2D(), 0.);


        Ray3f shadow_ray = Ray3f(sceneIts->p, emitterRecord->wi.normalized());
        Intersection shadow_ray_its;

        // Perform a visibility query (shadow ray) and compute intersection
        if ((!scene->rayIntersect(shadow_ray, shadow_ray_its) ||
            (emitterRecord->dist <= shadow_ray_its.t+0.1))){

            BSDFQueryRecord bsdfRecord(
                sceneIts->toLocal(-sceneRay.d),
                sceneIts->toLocal(emitterRecord->wi),
                sceneIts->uv,
                ESolidAngle
            );

            // Accumulate incident light * foreshortening * BSDF term
            Color3f bsdf = sceneIts->mesh->getBSDF()->eval(bsdfRecord);
            res += (Le * sceneIts->shFrame.n.dot(emitterRecord->wi) *
                bsdf) / pdflight;

            p_mat_em = sceneIts->mesh->getBSDF()->pdf(bsdfRecord);    //BRDF pdf for emitter sampling
            p_em_em = pdflight*em->pdf(*emitterRecord); 
            w_emmiter = p_em_em / (p_em_em + p_mat_em);
            res *= w_emmiter;
        }
        else{
            // cout << "SHADOW -> " << shadow_ray_its.p << endl;
        }


        return res;
    }

    Color3f Li(const Scene* scene, Sampler* sampler, const Ray3f& ray) const {
        Color3f Lo(0.);
        // Find the surface that is visible in the requested direction
        Intersection its;
        if (!scene->rayIntersect(ray, its))
            return scene->getBackground(ray);

        // Compute the direct radiance
        Lo += DirectRadiance(&its, &ray.o);

        // Sample a ray using the BSDF
        Lo += BSDFSampling(scene, &its, ray, sampler);

        // Sample a ray using Emitter sampling
        float pdflight;
        float random_01 = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
        const Emitter* sampled_em = scene->sampleEmitter(random_01, pdflight);
        // const Emitter* sampled_em = scene->importanceSampleEmitterIntensive(random_01, pdflight, &emitterRecord, sampler);
        // const Emitter* sampled_em = scene->importanceSampleEmitter(random_01, pdflight);
        EmitterQueryRecord emitterRecord(its.p);
        Lo += EmitterSampling(sampled_em, pdflight, &emitterRecord,
                            scene, &its, ray, sampler);


        return Lo;
    }

    std::string toString() const {
        return "Direct Material Integrator []";
    }
};

NORI_REGISTER_CLASS(DirectMIS, "direct_mis");

NORI_NAMESPACE_END
