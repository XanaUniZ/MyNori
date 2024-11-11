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

        // If we intersect a ligth return direct radiance
        if (its.mesh->isEmitter()){
            EmitterQueryRecord selfEmitterRecord(ray.o);
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

        Point2f sample = sampler->next2D();
        Color3f brdfSample = its.mesh->getBSDF()->sample(initialBSDFQuery, sample);

        // if (brdfSample.isZero() || brdfSample.hasNaN()) {   // if it is not valid, return black
        //     return Color3f(0.0f);
        // } 

        Ray3f sampledRay(its.p, its.toWorld((initialBSDFQuery.wo)));

        Intersection itsSampledRay;
        if (!scene->rayIntersect(sampledRay, itsSampledRay)){
            Color3f backgroundColor = scene->getBackground(sampledRay);
            Lo += backgroundColor * brdfSample;
            return Lo;
        }

        if (itsSampledRay.mesh->isEmitter()){
            const Emitter* em;
            em = itsSampledRay.mesh->getEmitter();

            // Sample the point sources, getting its radiance and direction
            EmitterQueryRecord emitterRecord(its.p);
            emitterRecord.p = itsSampledRay.p;
            emitterRecord.wi = (emitterRecord.p - emitterRecord.ref).normalized();
            emitterRecord.dist = (emitterRecord.p - emitterRecord.ref).norm();

            emitterRecord.uv = itsSampledRay.uv;
            emitterRecord.n = itsSampledRay.geoFrame.n;

            // EmitterQueryRecord emitterRecord(itsSampledRay.p);
            // emitterRecord.ref = sampledRay.o;
			// emitterRecord.wi = sampledRay.d;
			// emitterRecord.n = itsSampledRay.shFrame.n;

            Color3f Le = em->eval(emitterRecord);


            // BSDFQueryRecord bsdfRecord(
            //     its.toLocal(-ray.d),
            //     its.toLocal(emitterRecord.wi),
            //     its.uv,
            //     ESolidAngle
            // );

            // Accumulate incident light * foreshortening * BSDF term
            Color3f bsdf = brdfSample;
            // cout << "Le: " << Le << endl;
            // cout << "Cos(N): " << its.shFrame.n.dot(emitterRecord.wi) << endl;
            // cout << "BSDF: " << bsdf << endl;
            // its.shFrame.n.dot(emitterRecord.wi)
            Lo += (Le * bsdf);
        }

        return Lo;
    }

    std::string toString() const {
        return "Direct Material Integrator []";
    }
};

NORI_REGISTER_CLASS(DirectMaterialSampling, "direct_mats");

NORI_NAMESPACE_END
