#pragma once


#include <map>
#include <set>

#include "object.cpp"
#include "rigid.cpp"
#include "cloth.cpp"
#include "collision.cpp"

struct Scene;

struct Solver
{
    void solve(Edge &edge, Real delta_t) {
        Real3 x1 = edge.obj->positions[edge.v1];
        Real3 x2 = edge.obj->positions[edge.v2];

        Real w1 = edge.obj->inv_masses[edge.v1];
        Real w2 = edge.obj->inv_masses[edge.v2];
        Real w  = w1 + w2;

        if (w == 0.0) return;

        Real alpha = edge.compliance / delta_t / delta_t;

        Real C   = distance(x1, x2) - edge.rest_length;
        Real3 dC = (x2-x1) / distance(x2, x1);
        
        Real d_lambda  = (-C - alpha*edge.lambda) / (w + alpha);
        edge.lambda += d_lambda;

        edge.obj->positions[edge.v1] -= d_lambda * w1 * dC;
        edge.obj->positions[edge.v2] += d_lambda * w2 * dC;
    }

    void solve(ClothEdge &edge, Real delta_t) 
    {
        Real3 x1 = edge.cloth->positions[edge.v1];
        Real3 x2 = edge.cloth->positions[edge.v2];

        Real w1 = edge.cloth->inv_masses[edge.v1];
        Real w2 = edge.cloth->inv_masses[edge.v2];
        Real w  = w1 + w2;

        if (w == 0.0) return;

        Real alpha = edge.compliance / delta_t / delta_t;

        Real C   = distance(x1, x2) - edge.rest_length;
        Real3 dC = (x2-x1) / distance(x2, x1);
        
        Real d_lambda  = (-C - alpha*edge.lambda) / (w + alpha);
        edge.lambda   += d_lambda;

        edge.cloth->positions[edge.v1] -= d_lambda * w1 * dC;
        edge.cloth->positions[edge.v2] += d_lambda * w2 * dC;
    }

    void solve(Tetrahedron &tetra, Real delta_t) 
    {
        std::array<Real3, 4> positions = get_tetra_points(tetra.obj, tetra.vs);
        
        Real3 x0 = positions[0];
        Real3 x1 = positions[1];
        Real3 x2 = positions[2];
        Real3 x3 = positions[3];

        Real current_volume = tetra_volume(x0, x1, x2, x3); 

        // Constraint function: C = V - V₀
        Real C = 6 * (current_volume - tetra.rest_volume);

        // Gradienti del constraint rispetto a ogni vertice
        Real3 dC0 = -glm::cross(x2 - x1, x3 - x1);
        Real3 dC1 = -glm::cross(x3 - x2, x0 - x2);
        Real3 dC2 = -glm::cross(x0 - x3, x1 - x3);
        Real3 dC3 = -glm::cross(x1 - x0, x2 - x0);

        // Verifica che i gradienti siano consistenti con la normale
        Real3 center = (x0 + x1 + x2 + x3) / 4.0;

        Real3 to_center = center - positions[0];
        if (glm::dot(dC0, to_center) < 0) dC0 = -dC0;

        to_center = center - positions[1];
        if (glm::dot(dC1, to_center) < 0) dC1 = -dC1;

        to_center = center - positions[2];
        if (glm::dot(dC2, to_center) < 0) dC2 = -dC2;

        to_center = center - positions[3];
        if (glm::dot(dC3, to_center) < 0) dC3 = -dC3;

        // Pesi (inverse masses)
        Real w0 = tetra.obj->inv_masses[tetra.vs[0]];
        Real w1 = tetra.obj->inv_masses[tetra.vs[1]];
        Real w2 = tetra.obj->inv_masses[tetra.vs[2]];
        Real w3 = tetra.obj->inv_masses[tetra.vs[3]];

        // Somma pesata dei gradienti
        Real w_sum = w0 * glm::dot(dC0, dC0) + w1 * glm::dot(dC1, dC1) + w2 * glm::dot(dC2, dC2) + w3 * glm::dot(dC3, dC3);

        if (w_sum < 1e-12) return; // Evita divisione per zero

        // Compliance term
        Real alpha = tetra.compliance / (delta_t * delta_t);

        // Calcola delta lambda
        Real d_lambda = (- C - alpha * tetra.lambda) / (w_sum + alpha);
        tetra.lambda += d_lambda;

        // Applica correzioni
        if (w0 > 0.0) tetra.obj->positions[tetra.vs[0]] += - d_lambda * w0 * dC0;
        if (w1 > 0.0) tetra.obj->positions[tetra.vs[1]] += - d_lambda * w1 * dC1;
        if (w2 > 0.0) tetra.obj->positions[tetra.vs[2]] += - d_lambda * w2 * dC2;
        if (w3 > 0.0) tetra.obj->positions[tetra.vs[3]] += - d_lambda * w3 * dC3;
    }

    void solve(SpringConstraint &constraint, Real delta_t) {
        Real3 x1 = constraint.obj1->positions[constraint.v1];
        Real3 x2 = constraint.obj2->positions[constraint.v2];

        Real w1 = constraint.obj1->inv_masses[constraint.v1];
        Real w2 = constraint.obj2->inv_masses[constraint.v2];
        Real w  = w1 + w2;

        if (w == 0.0) return;

        Real alpha  = constraint.compliance / delta_t / delta_t;
        Real length = distance(x1, x2);

        if (length < 1e-6) return;

        Real C   = length - constraint.rest_length;
        Real3 dC = (x2-x1) / length;

        Real d_lambda      = (-C -alpha*constraint.lambda) / (w + alpha);
        constraint.lambda += d_lambda;

        constraint.obj1->positions[constraint.v1] -= d_lambda * w1 * dC;
        constraint.obj2->positions[constraint.v2] += d_lambda * w2 * dC;
    }

    void solve(CollisionConstraint &constraint, Real delta_t) {

        if (!constraint.active) return;

        Real3 x      = constraint.obj->positions[constraint.v];
        Real3 old_x  = constraint.obj->old_positions[constraint.v];
        Real3 x_goal = constraint.goal_position;

        Real w = constraint.obj->inv_masses[constraint.v];

        if (w == 0.0) return;

        Real alpha = constraint.compliance / delta_t / delta_t;

        Real C   = distance(x, x_goal);
        Real3 dC = (x-x_goal) / distance(x, x_goal);

        // Real C   = glm::dot(x - old_x, constraint.normal) - constraint.penetration;
        // Real3 dC = constraint.normal;

        Real d_lambda      = (-C - alpha*constraint.lambda) / (w + alpha);
        constraint.lambda += d_lambda;

        constraint.obj->positions[constraint.v] += d_lambda * w * dC;
    }


    void applyPositionCorrection(RigidBox* body, const Real3& r, const Real3& nw, Real d_lambda, Real sign)
    {
        if (body->is_static) return;

        Real3 nb = world_to_body(nw, body->orientation);

        Real3 pw =  sign * d_lambda * nw;
        Real3 pb = -sign * d_lambda * nb;

        body->position += pw / body->mass;

        Real3 tau    = glm::cross(r, pb);
        Real3 domega = body->inv_inertia_tensor * tau;
        Quat omega_q(domega.x, domega.y, domega.z, 0);

        body->orientation += 0.5 * quat_multiplication(omega_q, body->orientation);
        body->orientation  = glm::normalize(body->orientation);
    }

    void solve(FixedRigidSpringConstraint &constraint, Real delta_t) 
    {
        RigidBox *box  = constraint.box;
        Real3 rb       = constraint.body_attach;
        Real3 rw       = body_to_world(constraint.body_attach, box->position, box->orientation);
        Real3 d        = constraint.world_attach - rw;

        Real C   = glm::length(d) - constraint.rest_length;
        Real3 nw = glm::normalize(d);

        Real3 nb = world_to_body(nw, Real3(0.0), box->orientation);

        Real w = box->generalized_inverse_mass(constraint.body_attach, nb);

        Real alpha  = constraint.compliance / delta_t / delta_t;

        Real d_lambda      = (-C -alpha*constraint.lambda) / (w + alpha);
        constraint.lambda += d_lambda;

        applyPositionCorrection(box, rb, nw, d_lambda, -1.0);
    }

    void solve(RigidSpringConstraint &constraint, Real delta_t) 
    {
        if (constraint.active == false) return;

        RigidBox *b1 = constraint.b1;
        RigidBox *b2 = constraint.b2;

        if (b1 == b2) return;

        Real3 r1 = constraint.r1;
        Real3 r2 = constraint.r2;

        Real3 p1 = body_to_world(r1, b1->position, b1->orientation);
        Real3 p2 = body_to_world(r2, b2->position, b2->orientation);

        Real3 d = p2 - p1;
        Real C  = glm::length(d) - constraint.rest_length;

        if (C < 1e-6) return;

        Real3 nw = glm::normalize(d);
        Real3 nb1 = world_to_body(nw, Real3(0.0), b1->orientation);
        Real3 nb2 = world_to_body(nw, Real3(0.0), b2->orientation);

        Real w1 = b1->generalized_inverse_mass(r1, nb1);
        Real w2 = b2->generalized_inverse_mass(r2, nb2);

        Real alpha = constraint.compliance / delta_t / delta_t;

        Real d_lambda      = (-C -alpha*constraint.lambda) / (w1 + w2 + alpha);
        constraint.lambda += d_lambda;

        applyPositionCorrection(b1, r1, nw, d_lambda, -1.0);
        applyPositionCorrection(b2, r2, nw, d_lambda,  1.0);
    }

    void solve(RigidCollisionConstraint &constraint, Real delta_t) 
    {
        RigidBox *b1 = constraint.b1;
        RigidBox *b2 = constraint.b2;

        Real3 p1 = constraint.p1;
        Real3 p2 = constraint.p2;

        Real3 r1 = world_to_body(p1, b1->position, b1->orientation);
        Real3 r2 = world_to_body(p2, b2->position, b2->orientation);

        constraint.r1 = r1;
        constraint.r2 = r2;

        Real3 nw = constraint.n;

        Real np1 = glm::dot(p1, nw);
        Real np2 = glm::dot(p2, nw);
        Real dnp = np2 - np1;

        Real C = constraint.d - dnp;

        if (C <= 0.0) return;

        Real w1 = b1->generalized_inverse_mass(r1, world_to_body(nw, Real3(0.0), b1->orientation));
        Real w2 = b2->generalized_inverse_mass(r2, world_to_body(nw, Real3(0.0), b2->orientation));

        Real alpha = constraint.compliance / delta_t / delta_t;

        Real d_lambda = (-C -alpha*constraint.lambda) / (w1 + w2 + alpha);
        constraint.lambda += d_lambda;

        applyPositionCorrection(b1, r1, nw, d_lambda, -1.0);
        applyPositionCorrection(b2, r2, nw, d_lambda,  1.0);
    }
};

// =====================================================
// EXPORTING DATA
// =====================================================

Real getLength(const RigidSpringConstraint &constraint)
{
    const RigidBox *b1 = constraint.b1;
    const RigidBox *b2 = constraint.b2;

    if (b1 == b2) return 0.0;

    Real3 p1 = body_to_world(constraint.r1, b1->position, b1->orientation);
    Real3 p2 = body_to_world(constraint.r2, b2->position, b2->orientation);

    Real3 dir = p2 - p1;
    Real dist = glm::length(dir);

    return dist;
}

struct Scene 
{
    std::vector<TetraObject>      objects;
    std::vector<RigidBox>         rigid_objects;
    std::vector<SpringConstraint> constraints;
    std::vector<FixedRigidSpringConstraint> fixed_rigid_constraints;
    std::vector<RigidSpringConstraint> rigid_constraints;
    std::vector<SceneObject> scene_objects;
    std::vector<Cloth> cloths;
    Solver solver;

    Scene() = default;

    void clear() {

        for (auto & obj : rigid_objects) obj.clear();

        objects.clear();
        rigid_objects.clear();
        constraints.clear();
        fixed_rigid_constraints.clear();
        rigid_constraints.clear();
        scene_objects.clear();
        cloths.clear();
    }

    void addCloth(Cloth &cloth) {
        cloths.push_back(std::move(cloth));
    }

    void addCloth(Cloth &&cloth) {
        cloths.push_back(std::move(cloth));
    }

    void addSceneObject(SceneObject& obj) { 
        scene_objects.push_back(std::move(obj)); 
    }

    void addObject(TetraObject& obj) { 
        objects.push_back(std::move(obj)); 
    }

    void addObject(TetraObject&& obj) { 
        objects.push_back(std::move(obj)); 
    }

    void addRigidObject(RigidBox& obj) { 
        rigid_objects.push_back(std::move(obj)); 
    }

    void addRigidObject(RigidBox&& obj) { 
        rigid_objects.push_back(std::move(obj)); 
    }

    void addConstraint(SpringConstraint& constraint) { 
        constraints.push_back(std::move(constraint)); 
    }

    void addRigidConstraint(FixedRigidSpringConstraint& constraint) { 
        fixed_rigid_constraints.push_back(std::move(constraint)); 
    }

    void addRigidConstraint(RigidSpringConstraint& constraint) { 
        rigid_constraints.push_back(std::move(constraint)); 
    }

    void removeAllRigidConstraints() {
        rigid_constraints.clear();
    }

    void removeAllConstraints() {
        constraints.clear();
    }

    Real3 center() {
        if (objects.empty()) return Real3(0.0);

        AABB scene_aabb = objects[0].aabb;
        for (Index oi = 0; oi < objects.size(); oi++) {
            scene_aabb.min = glm::min(scene_aabb.min, objects[oi].aabb.min);
            scene_aabb.max = glm::max(scene_aabb.max, objects[oi].aabb.max);
        }
        return (scene_aabb.min + scene_aabb.max) * 0.5;
    }

    TetraObject& getObject(size_t index) {
        if (index < objects.size()) 
            return objects[index];

        throw std::out_of_range("Object index out of range");
    }

    RigidBox& getRigidObject(size_t index) 
    {
        if (index < rigid_objects.size()) return rigid_objects[index];

        throw std::out_of_range("Object index out of range");
    }
};

void export_scene_to_obj(Scene &scene, uint64_t frame, Real scale_factor, Real3 offset = Real3(0.0), std::string prefix = "")
{
    auto export_object_output = [&](const std::string& object_type) -> std::ofstream 
    {
        std::ostringstream oss;
        oss << "..\\..\\animation\\" << object_type << "_" << std::setw(4) << std::setfill('0') << frame << ".obj";

        std::string filename = oss.str();
        std::ofstream out(filename);

        if (!out.is_open()) std::cerr << "Errore: impossibile scrivere il file " << filename << "\n";

        return out;
    };

    auto box_out = export_object_output(prefix + "Boxs");

    int global_vertex_offset = 0; 
    std::vector<RigidBox> &boxes = scene.rigid_objects;

    static constexpr std::array<std::array<int, 4>, 6> cube_quads = {{
        {{3, 2, 1, 0}}, // -Y
        {{4, 5, 6, 7}}, // +Y
        {{0, 4, 7, 3}}, // -X
        {{1, 2, 6, 5}}, // +X
        {{0, 1, 5, 4}}, // +Z
        {{2, 3, 7, 6}}  // -Z
    }};

    auto rigidBoxToObj = [&](const RigidBox &box, const std::string& name, int vertex_offset) 
    {
        box_out << "g " << name << "\n";

        for (const auto& v : box.world_vertices) 
        {
            Real3 scaled_pos = (v + offset) / scale_factor;
            box_out << "v " << scaled_pos.x << " " 
                            << scaled_pos.y << " " 
                            << scaled_pos.z << "\n";
        }

        for (const auto& quad : cube_quads) 
        {
            box_out << "f " 
                    << (quad[0] + 1 + vertex_offset) << " "
                    << (quad[1] + 1 + vertex_offset) << " "
                    << (quad[2] + 1 + vertex_offset) << " "
                    << (quad[3] + 1 + vertex_offset) << "\n";
        }
    };

    for (size_t i = 0; i < boxes.size()-1; ++i) 
    {
        std::string name = "Box_" + std::to_string(i);
        rigidBoxToObj(boxes[i], name, global_vertex_offset);
        global_vertex_offset += 8;
    }

    box_out.close();
}

void export_wrap_displacement_to_obj(Scene &scene, uint64_t frame, Real max_displacement_perc, Real scale_factor = 1.0, Real3 offset = Real3(0.0), const std::string& prefix = "")
{
    if (scene.rigid_objects.empty()) 
    {
        std::cerr << "Error: no rigid objects in scene\n";
        return;
    }

    auto create_export_file = [&](const std::string& object_type) -> std::ofstream 
    {
        std::ostringstream oss;
        oss << "..\\..\\animation\\" << object_type << "_" << std::setw(4) << std::setfill('0') << frame << ".obj";
        
        std::ofstream out(oss.str());
        if (!out.is_open()) { std::cerr << "Error: cannot write file " << oss.str() << "\n"; }
        return out;
    };

    std::ofstream pallet_out = create_export_file(prefix + "Wrap");
    if (!pallet_out.is_open()) return;

    pallet_out << "# Wrapped Pallet Export - Frame " << frame << "\n";
    pallet_out << "o " << prefix << "Wrap\n";

    struct ConstraintData 
    {
        size_t p1, p2;
        Real displacement;
        Real perc;
        bool active;
    };

    std::vector<Real3>          points;
    std::vector<ConstraintData> constraint_data;

    constraint_data.reserve(scene.rigid_constraints.size());
    points.reserve(scene.rigid_constraints.size()*2);

    for (const auto& cons : scene.rigid_constraints) 
    {
        if (!cons.active) continue;

        Real3 p1 = body_to_world(cons.r1, cons.b1->position, cons.b1->orientation);
        Real3 p2 = body_to_world(cons.r2, cons.b2->position, cons.b2->orientation);
        Real curr_length = glm::length(p2 - p1);
        Real diff = curr_length - cons.rest_length;
        Real perc = diff / cons.rest_length;

        points.push_back(p1); points.push_back(p2);

        constraint_data.push_back({points.size()-2, points.size()-1, diff, perc, true});
    }

    for (const auto& cons_data : constraint_data) 
    {
        if (cons_data.active == false) continue;

        Real3 scaled_p1 = (points[cons_data.p1] + offset) / scale_factor;
        Real3 scaled_p2 = (points[cons_data.p2] + offset) / scale_factor;

        Real t = glm::clamp(cons_data.perc, 0.0, max_displacement_perc);
        t = t / max_displacement_perc;

        Real r = t;
        Real g = 1.0 - t;
        Real b = 0.0;

        pallet_out << "v " << (float) scaled_p1.x << " " 
                           << (float) scaled_p1.y << " " 
                           << (float) scaled_p1.z << " "
                           << (float) r << " " 
                           << (float) g << " " 
                           << (float) b << "\n";

        pallet_out << "v " << (float) scaled_p2.x << " " 
                           << (float) scaled_p2.y << " " 
                           << (float) scaled_p2.z << " "
                           << (float) r << " " 
                           << (float) g << " " 
                           << (float) b << "\n";
    }

    for (const auto& cons_data : constraint_data) 
        pallet_out << "l " 
                   << (cons_data.p1 + 1) << " " 
                   << (cons_data.p2 + 1) << "\n";

    pallet_out.close();
}

void export_wrap_to_obj(Scene &scene, uint64_t frame, Real scale_factor = 1.0, Real3 offset = Real3(0.0), const std::string& prefix = "")
{
    auto create_export_file = [&](const std::string& object_type) -> std::ofstream 
    {
        std::ostringstream oss;
        oss << "..\\..\\animation\\" << object_type << "_" << std::setw(4) << std::setfill('0') << frame << ".obj";
        
        std::ofstream out(oss.str());
        if (!out.is_open()) { std::cerr << "Error: cannot write file " << oss.str() << "\n"; }
        return out;
    };

    std::ofstream pallet_out = create_export_file(prefix + "Wrap");
    if (!pallet_out.is_open()) return;

    pallet_out << "# Wrapped Pallet Export - Frame " << frame << "\n";
    pallet_out << "o " << prefix << "Wrap\n";

    std::vector<Real3> unique_points;
    std::vector<std::pair<int, int>> constraint_lines;

    unique_points.reserve(scene.rigid_constraints.size() * 2);
    constraint_lines.reserve(scene.rigid_constraints.size());

    auto find_or_add_point = [&](const Real3& pos) -> int 
    {
        constexpr Real tolerance_sq = 1e-12; 

        for (size_t i = 0; i < unique_points.size(); ++i) 
        {
            Real3 diff = unique_points[i] - pos;
            if (glm::dot(diff, diff) < tolerance_sq) return static_cast<int>(i);
        }
        
        unique_points.push_back(pos);
        return static_cast<int>(unique_points.size() - 1);
    };

    for (const auto& cons : scene.rigid_constraints) 
    {
        if (!cons.active) continue;
        
        Real3 p1 = body_to_world(cons.r1, cons.b1->position, cons.b1->orientation);
        Real3 p2 = body_to_world(cons.r2, cons.b2->position, cons.b2->orientation);

        int idx1 = find_or_add_point(p1);
        int idx2 = find_or_add_point(p2);
        
        constraint_lines.push_back({idx1, idx2});
    }

    for (const auto& point : unique_points) 
    {
        Real3 scaled_pos = (point + offset) / scale_factor;
        pallet_out << "v " << scaled_pos.x << " " 
                           << scaled_pos.y << " " 
                           << scaled_pos.z << "\n";
    }

    for (const auto& [idx1, idx2] : constraint_lines)
        pallet_out << "l " << (idx1 + 1) << " " << (idx2 + 1) << "\n";

    pallet_out.close();
}

// ======= EXTRA  =======

void export_cloth_to_obj(Scene &scene, uint64_t frame, Real scale_factor = 1.0)
{
    if (scene.cloths.empty()) {
        std::cerr << "Errore: nessun cloth nella scena\n";
        return;
    }

    auto export_object_output = [&](const std::string& object_type) -> std::ofstream {
        std::ostringstream oss;
        oss << "..\\..\\animation\\" << object_type << "_" 
            << std::setw(4) << std::setfill('0') << frame << ".obj";
        
        std::string filename = oss.str();
        std::ofstream out(filename);
        if (!out.is_open()) {
            std::cerr << "Errore: impossibile scrivere il file " << filename << "\n";
        }
        return out;
    };

    auto cloth_out = export_object_output("cloth");
    if (!cloth_out.is_open()) return;

    Cloth &cloth = scene.cloths[0];

    cloth_out << "# Cloth Mesh Export - Frame " << frame << "\n";
    cloth_out << "o Cloth_0\n";

    for (const auto& v : cloth.positions) {
        cloth_out << "v " << v.x / scale_factor << " " 
                         << v.y  / scale_factor << " " 
                         << v.z  / scale_factor << "\n";
    }

    for (size_t f = 0; f < cloth.mesh.triangleIndices.size(); f += 3) {
        cloth_out << "f "
                  << (cloth.mesh.triangleIndices[f]     + 1) << " "
                  << (cloth.mesh.triangleIndices[f + 1] + 1) << " "
                  << (cloth.mesh.triangleIndices[f + 2] + 1) << "\n";
    }

    cloth_out.close();
    
    std::cout << "Cloth esportato nel frame " << frame << "\n";
}

void export_tetra_surface_with_normals(Scene &scene, uint64_t frame, uint32_t subdivisions_x, uint32_t subdivisions_y, uint32_t subdivisions_z, Real scale_factor = 1.0)
{
    if (scene.objects.empty()) {
        std::cerr << "Errore: nessun tetra object nella scena\n";
        return;
    }

    auto export_object_output = [&](const std::string& object_type) -> std::ofstream {
        std::ostringstream oss;
        oss << "..\\..\\animation\\" << object_type << "_" 
            << std::setw(4) << std::setfill('0') << frame << ".obj";
        
        std::string filename = oss.str();
        std::ofstream out(filename);
        if (!out.is_open()) {
            std::cerr << "Errore: impossibile scrivere il file " << filename << "\n";
        }
        return out;
    };

    auto tetra_out = export_object_output("tetra_surface_normals");
    if (!tetra_out.is_open()) return;

    TetraObject &tetra_obj = scene.objects[0];

    tetra_out << "# Tetra Surface Export with Normals - Frame " << frame << "\n";
    tetra_out << "o Tetra_Surface_0\n";

    // Vertici
    Real3 center(0.0);
    for (const auto& v : tetra_obj.positions) {
        tetra_out << "v " << v.x / scale_factor << " " 
                          << v.y / scale_factor << " " 
                          << v.z / scale_factor << "\n";
        center += v;
    }

    center /= (Real) tetra_obj.positions.size();

    auto get_grid_coordinates = [&](VertexIndex vi) -> std::tuple<uint32_t, uint32_t, uint32_t> {
        uint32_t total_xy  = (subdivisions_x + 1) * (subdivisions_y + 1);
        uint32_t z         = vi / total_xy;
        uint32_t remainder = vi % total_xy;
        uint32_t y         = remainder / (subdivisions_x + 1);
        uint32_t x         = remainder % (subdivisions_x + 1);
        return {x, y, z};
    };

    auto is_vertex_on_surface = [&](VertexIndex vi) -> bool {
        auto [x, y, z] = get_grid_coordinates(vi);

        return (x == 0) || (x == subdivisions_x) ||
               (y == 0) || (y == subdivisions_y) ||
               (z == 0) || (z == subdivisions_z);
    };


    auto is_face_on_surface = [&](const std::array<VertexIndex, 3>& face_vertices) -> bool {
        for (VertexIndex vi : face_vertices) {
            if (vi >= (subdivisions_x+1) * (subdivisions_y+1) * (subdivisions_z+1)) return false;
            if (!is_vertex_on_surface(vi)) return false;
        }

        auto [x0, y0, z0] = get_grid_coordinates(face_vertices[0]);
        auto [x1, y1, z1] = get_grid_coordinates(face_vertices[1]);
        auto [x2, y2, z2] = get_grid_coordinates(face_vertices[2]);
        
        // Controlla se sono tutti sulla faccia x=0
        if (x0 == 0 && x1 == 0 && x2 == 0) return true;
        // Controlla se sono tutti sulla faccia x=subdivisions_x
        if (x0 == subdivisions_x && x1 == subdivisions_x && x2 == subdivisions_x) return true;
        // Controlla se sono tutti sulla faccia y=0
        if (y0 == 0 && y1 == 0 && y2 == 0) return true;
        // Controlla se sono tutti sulla faccia y=subdivisions_y
        if (y0 == subdivisions_y && y1 == subdivisions_y && y2 == subdivisions_y) return true;
        // Controlla se sono tutti sulla faccia z=0
        if (z0 == 0 && z1 == 0 && z2 == 0) return true;
        // Controlla se sono tutti sulla faccia z=subdivisions_z
        if (z0 == subdivisions_z && z1 == subdivisions_z && z2 == subdivisions_z) return true;
        
        return false;
    };
    
    std::vector<std::array<VertexIndex, 3>> surface_faces;
    std::set<std::array<VertexIndex, 3>> all_faces;
    
    std::cout << "tetras: " << tetra_obj.tetras.size() << std::endl;

    for (const auto& tetra : tetra_obj.tetras) {
        std::array<std::array<VertexIndex, 3>, 4> faces = {{
            {{tetra.vs[0], tetra.vs[1], tetra.vs[2]}},
            {{tetra.vs[0], tetra.vs[1], tetra.vs[3]}},
            {{tetra.vs[0], tetra.vs[2], tetra.vs[3]}},
            {{tetra.vs[1], tetra.vs[2], tetra.vs[3]}}
        }};
        
        for (const auto& face : faces) {
            auto sorted_face = face;
            std::sort(sorted_face.begin(), sorted_face.end());
            all_faces.insert(sorted_face);
        }
    }

    std::cout << "diff. faces: " << all_faces.size() << std::endl;

    for (const auto &face : all_faces) {
        if (true || is_face_on_surface(face)) {
            surface_faces.push_back(face);
        }
    }


    for (auto& surface_face : surface_faces) {

        Real3 x1 = tetra_obj.positions[surface_face[0]];
        Real3 x2 = tetra_obj.positions[surface_face[1]];
        Real3 x3 = tetra_obj.positions[surface_face[2]];

        Real3 face_center = (x1 + x2 + x3) / 3.0;
        Real3 to_center   = face_center - center;
        Real3 normal      = glm::cross((x2-x1), (x3-x1));

        if (glm::dot(normal, to_center) < 0.0) {
            VertexIndex tmp = surface_face[0];
            surface_face[0] = surface_face[1];
            surface_face[1] = tmp;
        }

        tetra_out << "f " << (surface_face[0] + 1) << " " 
                          << (surface_face[1] + 1) << " " 
                          << (surface_face[2] + 1) << "\n";
    }

    tetra_out.close();
    
    std::cout << "TetraObject surface with normals esportato nel frame " << frame 
              << " (" << surface_faces.size() << " surface faces)\n";
}

void export_tetra_to_obj(Scene &scene, uint64_t frame, Real scale_factor = 1.0)
{
    if (scene.objects.empty()) {
        std::cerr << "Errore: nessun tetra object nella scena\n";
        return;
    }

    auto export_object_output = [&](const std::string& object_type) -> std::ofstream {
        std::ostringstream oss;
        oss << "..\\..\\animation\\" << object_type << "_" 
            << std::setw(4) << std::setfill('0') << frame << ".obj";
        
        std::string filename = oss.str();
        std::ofstream out(filename);
        if (!out.is_open()) {
            std::cerr << "Errore: impossibile scrivere il file " << filename << "\n";
        }
        return out;
    };

    auto tetra_out = export_object_output("tetra");
    if (!tetra_out.is_open()) return;

    TetraObject &tetra_obj = scene.objects[0];

    tetra_out << "# Tetra Mesh Export - Frame " << frame << "\n";
    tetra_out << "o Tetra_0\n";

    // Vertici
    for (const auto& v : tetra_obj.positions) {
        tetra_out << "v " << v.x / scale_factor << " " 
                         << v.y / scale_factor << " " 
                         << v.z / scale_factor << "\n";
    }

    // Facce triangolari (4 triangoli per ogni tetraedro)
    for (const auto& tetra : tetra_obj.tetras) {
        // Faccia 1: v0, v1, v2
        tetra_out << "f " << (tetra.vs[0] + 1) << " "
                          << (tetra.vs[1] + 1) << " " 
                          << (tetra.vs[2] + 1) << "\n";
        // Faccia 2: v0, v1, v3
        tetra_out << "f " << (tetra.vs[0] + 1) << " "
                          << (tetra.vs[1] + 1) << " " 
                          << (tetra.vs[3] + 1) << "\n";
        // Faccia 3: v0, v2, v3
        tetra_out << "f " << (tetra.vs[0] + 1) << " "
                          << (tetra.vs[2] + 1) << " " 
                          << (tetra.vs[3] + 1) << "\n";
        // Faccia 4: v1, v2, v3
        tetra_out << "f " << (tetra.vs[1] + 1) << " "
                          << (tetra.vs[2] + 1) << " " 
                          << (tetra.vs[3] + 1) << "\n";
    }

    tetra_out.close();
    
    std::cout << "TetraObject esportato nel frame " << frame << "\n";
}

void export_collision_scene_to_obj(const RigidBox& b1, const RigidBox& b2, const RigidCollisionInfo& info, const std::string& filename) 
{
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Errore: impossibile aprire il file " << filename << " per la scrittura" << std::endl;
        return;
    }

    uint32_t vertex_offset = 1; // .obj è 1-based

    // Export Box 1 come oggetto separato
    file << "# Box 1\n";
    file << "o Box1\n";  // 'o' crea un oggetto separato in Blender
    for (const auto& vertex : b1.world_vertices) {
        file << "v " << vertex.x << " " << vertex.y << " " << vertex.z << "\n";
    }

    // Facce del Box 1
    std::vector<std::array<int, 4>> box1_faces = {
        {1, 2, 3, 4},  // bottom
        {5, 6, 7, 8},  // top
        {1, 2, 6, 5},  // front
        {2, 3, 7, 6},  // right
        {3, 4, 8, 7},  // back
        {4, 1, 5, 8}   // left
    };

    for (const auto& face : box1_faces) {
        file << "f";
        for (int vertex_idx : face) {
            file << " " << vertex_idx;
        }
        file << "\n";
    }

    // Export Box 2 come oggetto separato
    file << "\n# Box 2\n";
    file << "o Box2\n";  // Nuovo oggetto
    vertex_offset += b1.world_vertices.size();
    
    for (const auto& vertex : b2.world_vertices) {
        file << "v " << vertex.x << " " << vertex.y << " " << vertex.z << "\n";
    }

    for (const auto& face : box1_faces) {
        file << "f";
        for (int vertex_idx : face) {
            file << " " << (vertex_idx + vertex_offset - 1);
        }
        file << "\n";
    }

    // Export Contact Manifold come oggetto separato
    file << "\n# Contact Manifold Polygon\n";
    file << "o ContactManifold\n";  // Oggetto separato per il manifold
    vertex_offset += b2.world_vertices.size();

    if (info.manifold_size > 0) {
        // Scrivi i vertici del manifold
        for (int i = 0; i < info.manifold_size; ++i) {
            const Real3& vertex = info.manifold[i];
            file << "v " << vertex.x << " " << vertex.y << " " << vertex.z << "\n";
        }
        
        // Crea il poligono del manifold
        if (info.manifold_size >= 3) {
            // Poligono principale (faccia)
            file << "f";
            for (int i = 0; i < info.manifold_size; ++i) {
                file << " " << (vertex_offset + i);
            }
            file << "\n";

            // Aggiungi anche le facce triangolate per una migliore visualizzazione
            if (info.manifold_size > 3) {
                for (int i = 1; i < info.manifold_size - 1; ++i) {
                    file << "f " << vertex_offset 
                         << " " << (vertex_offset + i) 
                         << " " << (vertex_offset + i + 1) << "\n";
                }
            }
        } else if (info.manifold_size == 2) {
            // Se sono solo 2 punti, crea una linea
            file << "l " << vertex_offset << " " << (vertex_offset + 1) << "\n";
        }
    }

    file.close();
    std::cout << "Scena di collisione esportata in: " << filename << std::endl;
    std::cout << "Manifold size: " << static_cast<int>(info.manifold_size) << " punti" << std::endl;
    
    // Debug: stampa i punti del manifold
    std::cout << "Punti del manifold:" << std::endl;
    for (int i = 0; i < info.manifold_size; ++i) {
        std::cout << "  " << i << ": (" << info.manifold[i].x << ", " 
                  << info.manifold[i].y << ", " << info.manifold[i].z << ")" << std::endl;
    }
    

}

void export_two_rigid_bodies_to_obj(Scene &scene, uint64_t frame, Real scale_factor = 1.0)
{
    if (scene.rigid_objects.size() < 2) {
        std::cerr << "Errore: meno di 2 rigid objects nella scena\n";
        return;
    }

    auto export_object_output = [&](const std::string& object_type) -> std::ofstream {
        std::ostringstream oss;
        oss << "..\\..\\animation\\" << object_type << "_" 
            << std::setw(4) << std::setfill('0') << frame << ".obj";
        
        std::string filename = oss.str();
        std::ofstream out(filename);
        if (!out.is_open()) {
            std::cerr << "Errore: impossibile scrivere il file " << filename << "\n";
        }
        return out;
    };

    auto rigid_out = export_object_output("two_rigid_bodies");
    if (!rigid_out.is_open()) return;

    RigidBox &rigid_box1 = scene.rigid_objects[0];
    RigidBox &rigid_box2 = scene.rigid_objects[1];

    rigid_out << "# Two Rigid Bodies with Spring Constraint - Frame " << frame << "\n";
    
    // Vertici del primo cubo rigido
    rigid_out << "o RigidBox1\n";
    for (const auto& v : rigid_box1.world_vertices) {
        rigid_out << "v " << v.x / scale_factor << " " 
                         << v.y / scale_factor << " " 
                         << v.z / scale_factor << "\n";
    }

    // Vertici del secondo cubo rigido
    rigid_out << "o RigidBox2\n";
    int vertex_offset = rigid_box1.world_vertices.size();
    for (const auto& v : rigid_box2.world_vertices) {
        rigid_out << "v " << v.x / scale_factor << " " 
                         << v.y / scale_factor << " " 
                         << v.z / scale_factor << "\n";
    }

    // Facce del primo cubo come quads
    std::vector<std::array<int, 4>> cube_quads = {
        {0,1,2,3}, // bottom
        {4,5,6,7}, // top
        {0,1,5,4}, // left
        {2,3,7,6}, // right
        {1,2,6,5}, // front
        {0,3,7,4}  // back
    };

    rigid_out << "g RigidBox1_Faces\n";
    for (const auto& quad : cube_quads) {
        rigid_out << "f " 
                  << (quad[0] + 1) << " "
                  << (quad[1] + 1) << " "
                  << (quad[2] + 1) << " "
                  << (quad[3] + 1) << "\n";
    }

    // Facce del secondo cubo come quads
    rigid_out << "g RigidBox2_Faces\n";
    for (const auto& quad : cube_quads) {
        rigid_out << "f " 
                  << (quad[0] + 1 + vertex_offset) << " "
                  << (quad[1] + 1 + vertex_offset) << " "
                  << (quad[2] + 1 + vertex_offset) << " "
                  << (quad[3] + 1 + vertex_offset) << "\n";
    }

    // Punti di connessione del vincolo
    if (!scene.rigid_constraints.empty()) {
        RigidSpringConstraint &constraint = scene.rigid_constraints[0];
        
        // Calcola le posizioni mondiali dei punti di attacco
        Real3 world_point1 = body_to_world(constraint.r1, rigid_box1.position, rigid_box1.orientation);
        Real3 world_point2 = body_to_world(constraint.r2, rigid_box2.position, rigid_box2.orientation);
        
        rigid_out << "o ConstraintPoints\n";
        rigid_out << "v " << world_point1.x / scale_factor << " "
                         << world_point1.y / scale_factor << " "
                         << world_point1.z / scale_factor << "\n";
        
        rigid_out << "v " << world_point2.x / scale_factor << " "
                         << world_point2.y / scale_factor << " "
                         << world_point2.z / scale_factor << "\n";
        
        // Linea che rappresenta il vincolo
        int constraint_vertex_offset = vertex_offset * 2; // 16 vertici totali (8 + 8)
        rigid_out << "l " << (constraint_vertex_offset + 1) << " " 
                         << (constraint_vertex_offset + 2) << "\n";
    }

    rigid_out.close();
    
    std::cout << "Two rigid bodies esportati nel frame " << frame << "\n";
}

void export_falling_rigid_to_obj(Scene &scene, uint64_t frame, Real scale_factor = 1.0)
{
    if (scene.rigid_objects.empty()) {
        std::cerr << "Errore: nessun rigid object nella scena\n";
        return;
    }

    auto export_object_output = [&](const std::string& object_type) -> std::ofstream {
        std::ostringstream oss;
        oss << "..\\..\\animation\\" << object_type << "_" 
            << std::setw(4) << std::setfill('0') << frame << ".obj";
        
        std::string filename = oss.str();
        std::ofstream out(filename);
        if (!out.is_open()) {
            std::cerr << "Errore: impossibile scrivere il file " << filename << "\n";
        }
        return out;
    };

    auto rigid_out = export_object_output("falling_rigid");
    if (!rigid_out.is_open()) return;

    RigidBox &rigid_box = scene.rigid_objects[0];

    rigid_out << "# Falling Rigid Box with Constraint - Frame " << frame << "\n";
    
    // Vertici del cubo rigido
    rigid_out << "o RigidBox\n";
    for (const auto& v : rigid_box.world_vertices) {
        rigid_out << "v " << v.x / scale_factor << " " 
                         << v.y / scale_factor << " " 
                         << v.z / scale_factor << "\n";
    }

    // Facce del cubo
    std::vector<std::array<int, 3>> cube_faces = {
        {0,1,2}, {0,2,3}, // bottom
        {0,1,5}, {0,5,4}, // left
        {4,5,6}, {4,6,7}, // top
        {2,3,7}, {2,7,6}, // right
        {1,2,6}, {1,6,5}, // front
        {0,3,7}, {0,7,4}  // back
    };

    for (const auto& f : cube_faces) {
        rigid_out << "f " 
                  << (f[0] + 1) << " "
                  << (f[1] + 1) << " "
                  << (f[2] + 1) << "\n";
    }

    // Punto di ancoraggio (world anchor point)
    // Per trovare il punto di ancoraggio, dobbiamo accedere al vincolo
    if (!scene.fixed_rigid_constraints.empty()) {
        FixedRigidSpringConstraint &constraint = scene.fixed_rigid_constraints[0];
        
        rigid_out << "o AnchorPoint\n";
        rigid_out << "v " << constraint.world_attach.x / scale_factor << " "
                         << constraint.world_attach.y / scale_factor << " "
                         << constraint.world_attach.z / scale_factor << "\n";
        
        // Vertice attaccato sul cubo (calcolato dalla posizione corrente)
        Real3 attached_vertex = body_to_world(constraint.body_attach, 
                                            rigid_box.position, 
                                            rigid_box.orientation);
        
        rigid_out << "v " << attached_vertex.x / scale_factor << " "
                         << attached_vertex.y / scale_factor << " "
                         << attached_vertex.z / scale_factor << "\n";
        
        // Linea che rappresenta il vincolo
        rigid_out << "l 9 10\n"; // 9 = anchor point, 10 = vertex on box
    }

    rigid_out.close();
    
    std::cout << "Falling rigid box esportato nel frame " << frame << "\n";
}

