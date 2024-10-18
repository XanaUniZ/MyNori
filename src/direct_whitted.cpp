#include <nori/warp.h>
#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/emitter.h>
#include <nori/bsdf.h>

NORI_NAMESPACE_BEGIN

class DirectWhittedIntegrator : public Integrator {
public:
    DirectWhittedIntegrator(const PropertyList& props) {
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
        // Get all lights in the scene
        const std::vector<Emitter*> lights = scene->getLights();

        // Iterate over all emitters
        for (unsigned int i = 0; i < lights.size(); ++i) {
            const Emitter* em = lights[i];
            // Sample the point sources, getting its radiance and direction
            Color3f Le = em->sample(emitterRecord, sampler->next2D(), 0.);

            // Perform a visibility query (shadow ray) and compute intersection
            /*
            * YOUR CODE HERE
            */
            Ray3f ray_to_light = Ray3f(its.p, emitterRecord.p - its.p);
            Intersection ray_to_light_its;

            if ((!scene->rayIntersect(ray_to_light, ray_to_light_its) || 
                ((emitterRecord.p - its.p).norm() < ray_to_light_its.t))){
            // if (!scene->rayIntersect(ray_to_light, ray_to_light_its)){
                // Evaluate the BSDF using the outgoing and incoming directions
                BSDFQueryRecord bsdfRecord(
                    its.toLocal(-ray.d),
                    its.toLocal(emitterRecord.wi),
                    its.uv,
                    ESolidAngle
                );

                // Accumulate incident light * foreshortening * BSDF term
                Lo += Le * its.shFrame.n.dot(emitterRecord.wi) *
                    its.mesh->getBSDF()->eval(bsdfRecord);
            }
            else{
                // Normal3f n = ray_to_light_its.shFrame.n.cwiseAbs();
                // Lo += Color3f(n.x(), n.y(), n.z());
            }
        }

        return Lo;
    }

    std::string toString() const {
        return "Direct Whitted Integrator []";
    }
};

NORI_REGISTER_CLASS(DirectWhittedIntegrator, "direct_whitted");

NORI_NAMESPACE_END
