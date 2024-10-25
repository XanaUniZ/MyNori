/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob

    v1 - Dec 01 2020
    Copyright (c) 2020 by Adrian Jarabo

    Nori is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License Version 3
    as published by the Free Software Foundation.

    Nori is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#include <nori/scene.h>
#include <nori/bitmap.h>
#include <nori/integrator.h>
#include <nori/sampler.h>
#include <nori/camera.h>
#include <nori/emitter.h>
#include <numeric>


NORI_NAMESPACE_BEGIN

Scene::Scene(const PropertyList &) {
    m_accel = new Accel();
    m_enviromentalEmitter = 0;
}

Scene::~Scene() {
    delete m_accel;
    delete m_sampler;
    delete m_camera;
    delete m_integrator;
}

void Scene::activate() {

    // Check if there's emitters attached to meshes, and
    // add them to the scene. 

    if (!m_sampler) {
        /* Create a default (independent) sampler */
        m_sampler = static_cast<Sampler*>(
            NoriObjectFactory::createInstance("independent", PropertyList()));
    }

    for(unsigned int i=0; i<m_meshes.size(); ++i ){
        if (m_meshes[i]->isEmitter()){
            m_emitters.push_back(m_meshes[i]->getEmitter());
        }
    }

    EmitterQueryRecord record(Vector3f(0.));
    emitters_pdf.reserve(m_emitters.size());
    for(unsigned int i=0; i<m_emitters.size(); ++i ){
        emitters_pdf.append(m_emitters[i]->sample(record, m_sampler->next2D(), 0.).getLuminance());
    }
    emitters_pdf.normalize();

    m_accel->build();

    if (!m_integrator)
        throw NoriException("No integrator was specified!");
    if (!m_camera)
        throw NoriException("No camera was specified!");
    
    cout << endl;
    cout << "Configuration: " << toString() << endl;
    cout << endl;
}

/// Sample emitter
const Emitter * Scene::sampleEmitter(float rnd, float &pdf) const {
	auto const & n = m_emitters.size();
	size_t index = std::min(static_cast<size_t>(std::floor(n*rnd)), n - 1);
	pdf = 1. / float(n);
	return m_emitters[index];
}

float Scene::pdfEmitter(const Emitter *em) const {
    return 1. / float(m_emitters.size());
}

const Emitter * Scene::importanceSampleEmitterIntensive(float rnd, float &pdf, EmitterQueryRecord* record, Sampler* sampler) const {
    std::vector<long double> all_luminances;
    for(int i=0; i < m_emitters.size(); i++){
        auto total_lumninance = m_emitters[i]->sample(*record, sampler->next2D(), 0.).getLuminance();
        all_luminances.push_back(total_lumninance);
    }

    // Sum of the vector so it adds up to one
    long double luminances_sum = std::accumulate(all_luminances.begin(), all_luminances.end(),0.0f);

    // Normalize luminances
    for (size_t i = 0; i < all_luminances.size(); ++i) {
        all_luminances[i] /= luminances_sum;
    }

    // Calculate the comulative distribution function
    std::vector<double> cdf(all_luminances.size());
    std::partial_sum(all_luminances.begin(), all_luminances.end(), cdf.begin());


    // Use upper_bound to find the index corresponding to the randomFloat
    auto it = std::upper_bound(cdf.begin(), cdf.end(), rnd);

    // Return the index of the first element greater than randomFloat
    size_t sampled_idx;
    if (it == cdf.end()) {
        // If randomFloat is exactly 1.0, or floating-point precision causes an issue,
        // clamp the result to the last index
        sampled_idx =  m_emitters.size() - 1;
    }
    else{
        sampled_idx =  it - cdf.begin();
    }

	pdf = all_luminances[sampled_idx];

	return m_emitters[sampled_idx];
}

float Scene::pdfImportanceEmitterIntensive(const Emitter *em, const Point3f& ref, EmitterQueryRecord* record, Sampler* sampler) const {
    std::vector<long double> all_luminances;
    for(int i=0; i < m_emitters.size(); i++){
        auto total_lumninance = m_emitters[i]->sample(*record, sampler->next2D(), 0.).getLuminance();
        all_luminances.push_back(total_lumninance);
    }

    // Sum of the vector so it adds up to one
    long double luminances_sum = std::accumulate(all_luminances.begin(), all_luminances.end(),0.0f);


    return em->eval(*record).getLuminance() / luminances_sum;
}

const Emitter * Scene::importanceSampleEmitter(float rnd, float &pdf) const {
    // Choose an index randomly 
    size_t index = emitters_pdf.sample(rnd);
    
	pdf = pdfImportanceEmitter(index);

	return m_emitters[index];
}

float Scene::pdfImportanceEmitter(const size_t em_idx) const {
	return emitters_pdf[em_idx];
}


void Scene::addChild(NoriObject *obj, const std::string& name) {
    switch (obj->getClassType()) {
        case EMesh: {
                Mesh *mesh = static_cast<Mesh *>(obj);
                m_accel->addMesh(mesh);
                m_meshes.push_back(mesh);
            }
            break;
        
        case EEmitter: {
				Emitter *emitter = static_cast<Emitter *>(obj);
				if (emitter->getEmitterType() == EmitterType::EMITTER_ENVIRONMENT)
				{
					if (m_enviromentalEmitter)
						throw NoriException("There can only be one enviromental emitter per scene!");
					m_enviromentalEmitter = emitter;
				}
				
                m_emitters.push_back(emitter);
			}
            break;

        case ESampler:
            if (m_sampler)
                throw NoriException("There can only be one sampler per scene!");
            m_sampler = static_cast<Sampler *>(obj);
            break;

        case ECamera:
            if (m_camera)
                throw NoriException("There can only be one camera per scene!");
            m_camera = static_cast<Camera *>(obj);
            break;
        
        case EIntegrator:
            if (m_integrator)
                throw NoriException("There can only be one integrator per scene!");
            m_integrator = static_cast<Integrator *>(obj);
            break;

        default:
            throw NoriException("Scene::addChild(<%s>) is not supported!",
                classTypeName(obj->getClassType()));
    }
}

Color3f Scene::getBackground(const Ray3f& ray) const
{
    if (!m_enviromentalEmitter)
        return Color3f(0);

    EmitterQueryRecord lRec(m_enviromentalEmitter, ray.o, ray.o + ray.d, Normal3f(0, 0, 1), Vector2f());
	return m_enviromentalEmitter->eval(lRec);
}


std::string Scene::toString() const {
    std::string meshes;
    for (size_t i=0; i<m_meshes.size(); ++i) {
        meshes += std::string("  ") + indent(m_meshes[i]->toString(), 2);
        if (i + 1 < m_meshes.size())
            meshes += ",";
        meshes += "\n";
    }

	std::string lights;
	for (size_t i = 0; i < m_emitters.size(); ++i) {
		lights += std::string("  ") + indent(m_emitters[i]->toString(), 2);
		if (i + 1 < m_emitters.size())
			lights += ",";
		lights += "\n";
	}


    return tfm::format(
        "Scene[\n"
        "  integrator = %s,\n"
        "  sampler = %s\n"
        "  camera = %s,\n"
        "  meshes = {\n"
        "  %s  }\n"
		"  emitters = {\n"
		"  %s  }\n"
        "]",
        indent(m_integrator->toString()),
        indent(m_sampler->toString()),
        indent(m_camera->toString()),
        indent(meshes, 2),
		indent(lights, 2)
    );
}

NORI_REGISTER_CLASS(Scene, "scene");
NORI_NAMESPACE_END
