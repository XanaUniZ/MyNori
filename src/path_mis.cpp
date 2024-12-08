#include <nori/warp.h>
#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/emitter.h>
#include <nori/bsdf.h>

NORI_NAMESPACE_BEGIN

class PathTracingMIS : public Integrator {
public:
    PathTracingMIS(const PropertyList& props) {
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
        float w_emmiter = 0.f;
        float p_em_em = 0.f, p_mat_em = 0.f;

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
            // if (!(bsdf.isZero() || bsdf.hasNaN())) {   
            //     return Color3f(0.);
            // } 
            double n_dot = sceneIts->shFrame.n.dot(emitterRecord.wi);
            if (n_dot > 10e-5 && !Le.hasNaN() && Le.maxCoeff() < 10e6){
                res += (Le * n_dot * bsdf) / pdflight;
            }
            // res += (Le * n_dot * bsdf) / pdflight;

            
            
            bsdfRecord.wo = emitterRecord.wi;
            p_mat_em = sceneIts->mesh->getBSDF()->pdf(bsdfRecord);    //BRDF pdf for emitter sampling
            float emSampling_pdf = sampled_em->pdf(emitterRecord);
            // float emSampling_pdf = emitterRecord.pdf;
            // emSampling_pdf = emSampling_pdf < 0.5 ? 0.5 : emSampling_pdf;
            // cout << "\npdflight: "<< pdflight << endl;
            // cout << "emSampling_pdf: "<< emSampling_pdf << endl;
            p_em_em = pdflight*emSampling_pdf; 
            
            if (p_em_em + p_mat_em > Epsilon){ 
                w_emmiter = p_em_em / (p_em_em + p_mat_em);
            }
            // cout << "res: "<< res << endl;
            res *= w_emmiter;
            // if (res.hasNaN()){
            //     cout << "\nw_emmiter: "<< w_emmiter << endl; 
            //     cout << "res2: "<< res << endl;
            //     cout << "Le: "<< Le << endl;
            //     cout << "bsdf: "<< bsdf << endl;
            //     cout << "n.dot: "<< sceneIts->shFrame.n.dot(emitterRecord.wi) << endl;
            // }
            // cout << "res2: "<< res << endl;
            // cout << "Le: "<< Le << endl;
            // cout << "bsdf: "<< bsdf << endl;
            // cout << "n.dot: "<< sceneIts->shFrame.n.dot(emitterRecord.wi) << endl;


        }
        else{
            // cout << "SHADOW -> " << shadow_ray_its.p << endl;
        }


        return res;
    }

    Color3f BSDFSampling(const Intersection * sceneIts, const Ray3f& sceneRay,
                            const Scene* scene, Sampler* sampler,
                            Ray3f* new_ray, float& term_prob,bool *directLight) const {
        float w_bsdf = 0.f;
        float p_mat_mat = 0.f, p_em_mat = 0.f;
        // BSDF ray
        BSDFQueryRecord BSDFQuery(sceneIts->toLocal(-sceneRay.d),sceneIts->uv);

        Point2f sample = sampler->next2D();
        Color3f brdfSample = sceneIts->mesh->getBSDF()->sample(BSDFQuery, sample);

        if (brdfSample.isZero() || brdfSample.hasNaN()) {   
            return Color3f(0.);
        } 

        Color3f throughput;
        throughput = brdfSample;
        if (BSDFQuery.measure == EDiscrete){
            // cout << "\ndirectLight before: " << *directLight << endl;
            // cout << "term_prob before: " << term_prob << endl;
            *directLight = true;
            term_prob = 0.001;
            throughput = Color3f(1.);
            // term_prob +=0.1;
            // cout << "directLight after: " << *directLight << endl;
            // cout << "term_prob after: " << term_prob << endl;

        } else{
            *directLight = false;
            term_prob = static_cast<float>(1. - std::min(throughput.maxCoeff(), 0.95f));
        }


        Ray3f emitterRay(sceneIts->p, BSDFQuery.wo);
        Intersection itsRay;
        if(scene->rayIntersect(emitterRay, itsRay) && itsRay.mesh->isEmitter()){
            const Emitter* em = itsRay.mesh->getEmitter();
            const EmitterQueryRecord emitterRecord(itsRay.p);
            p_em_mat = 1./(scene->getLights().size()) * 
                        em->pdf(emitterRecord);

        }
    
        p_mat_mat = sceneIts->mesh->getBSDF()->pdf(BSDFQuery);
        if (p_em_mat + p_mat_mat > Epsilon){
            // compute the weight
            w_bsdf = p_mat_mat / (p_em_mat + p_mat_mat);
        }

        if (BSDFQuery.measure == EDiscrete){}
        else if (w_bsdf < 10e-6){
            throughput = Color3f(0.); 
        }
        else{
            throughput *= w_bsdf;
        }

        // now create a new ray with the sampled BSDF direction
        *new_ray = Ray3f(sceneIts->p, sceneIts->toWorld(BSDFQuery.wo));

        // if (BSDFQuery.measure == EDiscrete){
        //     cout << "throughput: " << *throughput << endl;

        // }
        return throughput;
    }

    Color3f recursiveLi(const Scene* scene, Sampler* sampler, const Ray3f& ray,
                        long int n_bounce, float term_prob, bool DirectLight) const {
        Color3f Lo(0.);
        // Find the surface that is visible in the requested direction
        Intersection its;

        // Pathtracing Recusion
        bool has_interseced = scene->rayIntersect(ray, its);
        if (!has_interseced)
            return scene->getBackground(ray);

        else if(its.mesh->isEmitter() && DirectLight){
            Color3f direct = DirectRadiance(&its, &ray.o);
            return direct;
        }

        else{ // Intersected with a surface

            // Emitter Sampling ray
            Lo += EmitterSampling(&its, ray, scene, sampler);

            // BSDF Sampling
            // Russian roulete to choose termination
            if ( n_bounce < 2 || term_prob <= 0.001 || sampler->next1D() > term_prob){
                float new_term_prob = 0.;
                Ray3f new_ray;
                Color3f throughput = BSDFSampling(&its, ray, scene, sampler,
                                                 &new_ray, new_term_prob,&DirectLight) / (1.0f - term_prob);
                Lo += throughput * recursiveLi(scene, sampler, new_ray, n_bounce+1, new_term_prob,DirectLight);
            }   
        }
        return Lo;
    }

    Color3f Li(const Scene* scene, Sampler* sampler, const Ray3f& ray) const {
       
        return recursiveLi(scene, sampler, ray, 0, 0.1 , true);
    }

    std::string toString() const {
        return "Direct Material Integrator []";
    }
};

NORI_REGISTER_CLASS(PathTracingMIS, "path_mis");

NORI_NAMESPACE_END
