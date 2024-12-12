#include <nori/warp.h>
#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/emitter.h>
#include <nori/bsdf.h>

NORI_NAMESPACE_BEGIN

class SingleScattering : public Integrator {
public:
    SingleScattering(const PropertyList& props) {
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
            double n_dot = sceneIts->shFrame.n.dot(emitterRecord.wi);
            if (n_dot > 10e-5 && !Le.hasNaN() && Le.maxCoeff() < 10e6){
                res += (Le * n_dot * bsdf) / pdflight;
            }

            // cout << "\npdflight: "<< pdflight << endl;
            // cout << "Le: "<< Le << endl;
            // cout << "bsdf: "<< bsdf << endl;
            // cout << "n.dot: "<< sceneIts->shFrame.n.dot(emitterRecord.wi) << endl;
        }
        else{
            // cout << "SHADOW -> " << shadow_ray_its.p << endl;
        }

        // if (res.hasNaN()){
        //     cout << "\nLe: " << Le << endl;
        // }
        return res;
    }

   
    Color3f GetLiSingleScattering(const Scene* scene, Sampler* sampler, const Ray3f& ray) const {
        // Inicializa la radiancia acumulada
        Color3f Lo(0.0f);

        // Parámetros del medio
        // float sigma_s = 1.0f;   // Coeficiente de scattering
        // float sigma_t = 1.0f;   // Coeficiente de extinción (sigma_a + sigma_s)
        float sigma_s = 0.1f;   // Coeficiente de scattering
        float sigma_t = 0.1f;   // Coeficiente de extinción (sigma_a + sigma_s)
        float stepSize = 0.1f;  // Tamaño del paso para el ray marching

        // Distancia máxima basada en la escena
        float t = 0.0f;
        float pdflight;
        int n_samples = 0;
        float phase_funct = 1./(4*M_PI);
        float maxDistance;

        Intersection its;
        bool has_interseced = scene->rayIntersect(ray, its);
        if (!has_interseced){
            // Lo += std::exp(-sigma_t * t) * scene->getBackground(ray);
            // maxDistance = 10;
        }

        else if(its.mesh->isEmitter()){
            Color3f direct = DirectRadiance(&its, &ray.o);
            Lo += std::exp(-sigma_t * its.t) * direct;
            Lo += direct;
            maxDistance = its.t;
        }

        else{
            Lo += std::exp(-sigma_t * its.t) * EmitterSampling(&its, ray, scene, sampler);
            // Lo += EmitterSampling(&its, ray, scene, sampler);
            maxDistance = its.t;
        }

        while (t < maxDistance) {
            // Punto de muestreo actual
            Point3f samplePoint = t*ray.d + ray.o;
            t += stepSize;          

            EmitterQueryRecord emitterRecord(samplePoint);
            const Emitter* em = scene->sampleEmitter(sampler->next1D(), pdflight);
            Color3f Le = em->sample(emitterRecord, sampler->next2D(), 0.);
            
            Ray3f shadow_ray = Ray3f(samplePoint, emitterRecord.wi.normalized());
            Intersection shadow_ray_its;

            // Perform a visibility query (shadow ray) and compute intersection
            if (((!scene->rayIntersect(shadow_ray, shadow_ray_its) ||
                (emitterRecord.dist <= shadow_ray_its.t+0.1)) &&
                !Le.hasNaN() && Le.maxCoeff() < 10e6)){

                // Beer-Lambert
                float T_ligth_to_crystal = std::exp(-sigma_t * shadow_ray_its.t);            
                float T_crystal_to_eye = std::exp(-sigma_t * t);  

                // Accumulate incident light * normal * BSDF term
                Lo += (T_crystal_to_eye * T_ligth_to_crystal * Le * phase_funct) / pdflight;
                // if(Lo.hasNaN()){
                //     cout << "Le: " << Le << endl;
                //     cout << "T_crystal_to_eye: " << T_crystal_to_eye << endl;
                //     cout << "T_ligth_to_crystal: " << T_ligth_to_crystal << endl;
                // }
                n_samples++;
            }
            else{
                // cout << "SHADOW -> " << shadow_ray_its.p << endl;
            }
            
        }

        return Lo / (n_samples+1);
    }

    Color3f Li(const Scene* scene, Sampler* sampler, const Ray3f& ray) const {
        
        return GetLiSingleScattering(scene, sampler, ray);
    }

    std::string toString() const {
        return "Direct Material Integrator []";
    }
};

NORI_REGISTER_CLASS(SingleScattering, "single_scattering");

NORI_NAMESPACE_END
