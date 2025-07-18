#include "opengl-framework/opengl-framework.hpp"
#include "utils.hpp"
#include <cmath>
#include "iostream"

std::string toLog = "";
float previousTime = 0.f;
float deltaTime = 0.f;
float currentTime = 0.f;

glm::vec2 random_point_in_vector(glm::vec2 bounds)
{
    return glm::vec2(bounds.x * utils::rand(0.f, 1.f), bounds.y * utils::rand(0.f, 1.f));
}

glm::vec2 random_point_in_circle(float radius = 1.f)
{
    float length = sqrt(utils::rand(0.f, 1.f)) * radius;
    float angle = utils::rand(0.f, 2.f * 3.14f);
    return glm::vec2(cos(angle), sin(angle)) * length;
}

float lerp(float start, float end, float t)
{
    return (end - start) * t + start;
}

glm::vec2 lerp(glm::vec2 start, glm::vec2 end, float t)
{
    return (end - start) * t + start;
}

glm::vec3 lerp(glm::vec3 start, glm::vec3 end, float t)
{
    return (end - start) * t + start;
}

glm::vec4 lerp(glm::vec4 start, glm::vec4 end, float t)
{
    return (end - start) * t + start;
}

struct GameObject;

std::vector<GameObject *> objects = std::vector<GameObject *>();
std::vector<GameObject *> new_objects = std::vector<GameObject *>();
struct GameObjectComponent
{
    GameObject *object;

    virtual void start()
    {
    }

    virtual void physics_process(float delta)
    {
    }

    virtual void late_physics_process(float delta)
    {
    }

    virtual void render()
    {
    }
};

struct GameObject
{
    bool markedForGarbageCollection = false;
    glm::vec2 position = glm::vec2(0, 0);
    float size = 1.f;
    float rotation = 0.f;

    std::vector<GameObjectComponent *> components = std::vector<GameObjectComponent *>();
    void _go_start()
    {
        start();

        for (GameObjectComponent *component : components)
        {
            if (component == nullptr)
                continue;
            component->start();
        }
    }

    virtual void start()
    {
    }

    void _go_physics_process(float delta)
    {
        physics_process(delta);

        for (GameObjectComponent *component : components)
        {
            if (component == nullptr)
                continue;
            component->physics_process(delta);
        }
    }

    virtual void physics_process(float delta)
    {
    }

    void _go_late_physics_process(float delta)
    {
        late_physics_process(delta);

        for (GameObjectComponent *component : components)
        {
            if (component == nullptr)
                continue;
            component->late_physics_process(delta);
        }
    }

    virtual void late_physics_process(float delta)
    {
    }

    void _go_render()
    {
        render();

        for (GameObjectComponent *component : components)
        {
            if (component == nullptr)
                continue;
            component->render();
        }
    }

    virtual void render()
    {
    }
};

struct RigidBody : GameObjectComponent
{

    glm::vec2 linear_velocity = glm::vec2(0, 0);
    glm::vec2 _acceleration = glm::vec2(0, 0);
    float mass = 1.f;
    int RK4_steps = 1;

    float get_inv_mass()
    {
        if (mass == 0)
            return 0.f;
        return 1 / mass;
    }

    void add_force(glm::vec2 force)
    {
        add_acceleration(force * get_inv_mass());
    }

    void add_acceleration(glm::vec2 acceleration)
    {
        _acceleration += acceleration;
    }

    void add_impulse(glm::vec2 impulse)
    {
        add_velocity_change(impulse * get_inv_mass());
    }

    void add_velocity_change(glm::vec2 velocity_change)
    {
        linear_velocity += velocity_change;
    }

    void translate(glm::vec2 translation)
    {
        object->position += translation;
    }

    virtual void late_physics_process(float delta) override
    {
        // integrate acceleration
        add_velocity_change(glm::vec2(
            RK4(_acceleration.x, RK4_steps, delta),
            RK4(_acceleration.y, RK4_steps, delta)));

        // integrate velocity
        translate(delta * linear_velocity);
    }

    float RK4(float acceleration, int steps, float deltaTime)
    {
        float y0 = 0.f;
        float t = 0;
        float dt = deltaTime / steps;

        for (int i = 0; i < steps; i++)
        {
            float k1 = AccFunction(t, acceleration);
            float k2 = AccFunction(t + dt / 2, acceleration + k1 * dt / 2);
            float k3 = AccFunction(t + dt / 2, acceleration + k2 * dt / 2);
            float k4 = AccFunction(t + dt, acceleration + k3 * dt);
            y0 += (dt / 6) * (k1 + 2 * k2 + 2 * k3 + k4);
            t += dt;
        }

        return y0;
    }

    float AccFunction(float time, float y)
    {
        return y;
    }

    static float get_friction(float viscosity, float diameter)
    {
        return -3 * 3.14 * viscosity * diameter;
    }

    glm::vec2 get_friction_force(float viscosity, float diameter)
    {
        return get_friction(viscosity, diameter) * linear_velocity;
    }
};

struct SphereRenderer : GameObjectComponent
{
    glm::vec4 color = glm::vec4(1, 1, 1, 1);

    virtual void render() override
    {
        glm::vec2 screen_pos = object->position;
        // screen_pos += glm::vec2(0.5, 0.5);
        screen_pos.x /= 1000;
        screen_pos.y /= 1000;
        utils::draw_disk(screen_pos, object->size / 1000, color);
    }
};

struct ParametricFunctionRenderer : GameObjectComponent
{
    ParametricFunctionRenderer(GameObject *obj, std::function<glm::vec2(float)> func)
    {
        object = obj;
        function = func;
    }

    std::function<glm::vec2(float)> function;
    float start = 0.f;
    float end = 1.f;
    int steps = 100;
    glm::vec4 start_color = glm::vec4(1, 1, 1, 1);
    glm::vec4 end_color = glm::vec4(1, 0, 0, 1);

    virtual void physics_process(float delta) override
    {
    }
    virtual void render() override
    {
        for (int i = 0; i < steps; i++)
        {
            float t = lerp(start, end, i / (float)(steps - 1));
            float t_next = lerp(start, end, (i + 1) / (float)(steps - 1));
            glm::vec2 point = function(t);
            point += object->position;
            glm::vec2 point_next = function(t_next);
            point_next += object->position;
            utils::draw_line(
                point,
                point_next,
                0.01f, lerp(start_color, end_color, t));
        }
    }
};

struct Particle; // Forward declaration

struct SphereCollider : GameObjectComponent
{
    glm::vec2 previous_pos = glm::vec2(0, 0);

    glm::vec2 intersection(glm::vec2 a_origin, glm::vec2 a_dir, glm::vec2 b_origin, glm::vec2 b_dir)
    {
        glm::mat2 m = glm::mat2(a_dir, -b_dir);
        glm::vec2 b = b_origin - a_origin;
        glm::vec2 result = glm::inverse(m) * b;
        if (result.x < 0 || result.y < 0 || result.x > 1 || result.y > 1)
        {
            return glm::vec2(-1, -1); // no intersection
        }
        return result;
    }

    void physics_process(float delta) override;
};

struct Particle : GameObject
{
    float start_size = 1.f;
    float end_size = 0.f;
    glm::vec4 start_color;
    glm::vec4 end_color;
    float life_time = -1.f;
    float age = 0.f;

    RigidBody *rb = new RigidBody();
    SphereRenderer *rend = new SphereRenderer();
    SphereCollider *collider = new SphereCollider();

    virtual void start() override
    {
        GameObject::start();
        rb->object = this;
        rend->object = this;
        collider->object = this;

        components.push_back(rb);
        components.push_back(rend);
        components.push_back(collider);

        start_size = 3.f;
        end_size = 100.f;
        start_color = glm::vec4(0.f, 0.f, 0.f, 1.f);
        end_color = glm::vec4(1.f, 0.f, 0.f, 0.f);
        rb->mass = 1000.f;
        // rb->add_force(random_point_in_circle(1000000.f));
        life_time = utils::rand(0, 1);
    }

    virtual void physics_process(float delta) override
    {
        // rb->add_acceleration(glm::vec2(0, -9.80));
        // rb->add_force(rb->get_friction_force(0.0000181f, size));

        // rb->add_force(random_point_in_circle(100.f));

        if (life_time < 0)
        {
            markedForGarbageCollection = true;
            return;
        }

        age += delta;
        if (age > life_time)
        {
            size = end_size; // DELETE
            rend->color = end_color;
        }
        else
        {
            float coef = age / life_time;
            size = lerp(start_size, end_size, coef);
            rend->color = glm::vec4(
                lerp(start_color.x, end_color.x, coef),
                lerp(start_color.y, end_color.y, coef),
                lerp(start_color.z, end_color.z, coef),
                lerp(start_color.w, end_color.w, coef));
        }
    }

    ~Particle()
    {
        delete rb;
        delete rend;
        delete collider;
    }
};

struct Curve : GameObject
{
    std::function<glm::vec2(float)> function;
    ParametricFunctionRenderer *renderer;

    Curve(std::function<glm::vec2(float)> func)
    {
        function = func;
        renderer = new ParametricFunctionRenderer(this, function);
        components.push_back(renderer);
    }

    virtual void start() override
    {
        GameObject::start();
        position = glm::vec2(0, 0);
    }

    virtual void render() override
    {
        // position = lerp(glm::vec2(-1, -1), glm::vec2(1, 1), sin(currentTime));
        renderer->steps = lerp(0, 1000, currentTime / 100);

        if (renderer->steps > 1000)
            renderer->steps = 1000;
    }
};

void SphereCollider::physics_process(float delta)
{

    glm::vec2 old_to_new_vec = object->position - previous_pos;

    //
    glm::vec2 screen_pos = glm::vec2(100, -100);
    screen_pos += glm::vec2(0.5, 0.5);
    screen_pos.x /= gl::window_width_in_screen_coordinates();
    screen_pos.y /= gl::window_height_in_screen_coordinates();

    glm::vec2 screen_pos2 = glm::vec2(-100, -100);
    screen_pos2 += glm::vec2(0.5, 0.5);
    screen_pos2.x /= gl::window_width_in_screen_coordinates();
    screen_pos2.y /= gl::window_height_in_screen_coordinates();

    utils::draw_line(screen_pos2, screen_pos, 0.01f, glm::vec4(0, 1, 0, 1));

    //
    if (old_to_new_vec.x == 0 && old_to_new_vec.y == 0)
    {
        return; // no movement
    }

    glm::vec2 intersec = intersection(glm::vec2(-100, -100), glm::vec2(0, 200), previous_pos, old_to_new_vec);

    if (intersec.x > 0)
    {
        glm::vec2 normal = glm::normalize(glm::vec2(glm::vec2(-200, 0)));

        Particle *p = (Particle *)object;
        // p->rb->linear_velocity = glm::reflect(old_to_new_vec, normal);
    }

    previous_pos = object->position;
}

std::vector<GameObject *> spread_particles(float width, float radius)
{
    std::vector<GameObject *> particles;
    float sadius = radius / sqrt(2);

    for (float x = 0; x <= width; x += 2 * radius)
    {
        for (float y = 0; y <= width; y += 2 * radius)
        {
            Particle *p = new Particle();
            p->position = random_point_in_circle(radius - sadius) + glm::vec2(x - width / 2, y - width / 2);
            p->size = 0.001f;
            particles.push_back(p);
        }
    }

    return particles;
}

std::vector<GameObject *> spread_particles_along_parametric_curve(float steps, std::function<glm::vec2(float)> func)
{
    std::vector<GameObject *> particles;
    for (int i = 0; i < steps; i++)
    {
        Particle *p = new Particle();
        float t = lerp(0, 1, i / (float)(steps - 1));
        p->position = func(t) * 1000.f;

        float t_next = lerp(0, 1, (i + 1) / (float)(steps - 1));
        glm::vec2 force = func(t_next) * 1000.f - p->position;
        p->rb->add_force(glm::vec2(-1.f * force.y, force.x));

        particles.push_back(p);
    }

    return particles;
}

glm::vec2 _bezier1(glm::vec2 vec1, glm::vec2 vec2, glm::vec2 vec3, float t)
{
    glm::vec2 v1tov2 = lerp(vec1, vec2, t);

    return lerp(lerp(glm::vec2(0, 0), vec1, t), v1tov2, t) + lerp(v1tov2, lerp(vec2, vec3, t), t);
}

glm::vec2 _bezier2(glm::vec2 vec1, glm::vec2 vec2, glm::vec2 vec3, float t)
{
    float t_one_minus = 1 - t;

    return t_one_minus * t_one_minus * vec1 + 2 * t * t_one_minus * vec2 + t * t * vec3;
}

struct ParticleSpawner : GameObject
{

    virtual void physics_process(float delta) override
    {
        // Spawn particles at a fixed rate
        static float time_since_last_spawn = 0.f;
        time_since_last_spawn += delta;

        if (time_since_last_spawn > 0.1f) // Spawn every 0.1 seconds
        {
            spawn();
            time_since_last_spawn = 0.f;
        }
    }

    virtual void start() override
    {
        GameObject::start();
        init_spawn();
    }

    void init_spawn()
    {

        new_objects.push_back(new Curve([](float t) -> glm::vec2
                                        { return glm::vec2(cos(t * 3.14f * 2.f), sin(t * 3.14f * 2.f)) * 0.5f; }));

        new_objects.push_back(new Curve([](float t) -> glm::vec2
                                        { return _bezier1(glm::vec2(1.f, 1.f), glm::vec2(1.f, 0.f), glm::vec2(-1.f, -1.f), t); }));

        new_objects.push_back(new Curve([](float t) -> glm::vec2
                                        { return _bezier2(glm::vec2(1.f, 1.f), glm::vec2(1.f, 0.f), glm::vec2(-1.f, -1.f), t); }));

        spawn();
    }

    void spawn()
    {
        std::vector<GameObject *> new_particles = spread_particles_along_parametric_curve(100.f, [](float t) -> glm::vec2
                                                                                          { return glm::vec2(cos(t * 3.14f * 2.f), sin(t * 3.14f * 2.f)) * 0.5f; });

        new_objects.insert(new_objects.end(), new_particles.begin(), new_particles.end());

        new_particles = spread_particles_along_parametric_curve(100.f, [](float t) -> glm::vec2
                                                                { return _bezier1(glm::vec2(1.f, 1.f), glm::vec2(1.f, 0.f), glm::vec2(-1.f, -1.f), t); });
        new_objects.insert(new_objects.end(), new_particles.begin(), new_particles.end());

        new_particles = spread_particles_along_parametric_curve(100.f, [](float t) -> glm::vec2
                                                                { return _bezier2(glm::vec2(1.f, 1.f), glm::vec2(1.f, 0.f), glm::vec2(-1.f, -1.f), t); });
        new_objects.insert(new_objects.end(), new_particles.begin(), new_particles.end());

        toLog += std::to_string(objects.size());
    }
};

int main()
{
    gl::init("Particules!");
    gl::maximize_window();
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE);

    // TODO: create an array of particles

    objects.push_back(new ParticleSpawner());

    for (GameObject *particle : objects)
    {
        particle->_go_start();
    }

    while (gl::window_is_open())
    {
        currentTime = gl::time_in_seconds();
        deltaTime = currentTime - previousTime;

        glClearColor(0.f, 0.f, 0.f, 1.f);
        glClear(GL_COLOR_BUFFER_BIT);

        for (GameObject *particle : objects)
        {
            if (particle == nullptr)
                continue;
            particle->_go_physics_process(gl::delta_time_in_seconds());
        }

        for (GameObject *particle : objects)
        {
            if (particle == nullptr)
                continue;
            particle->_go_late_physics_process(gl::delta_time_in_seconds());
        }

        for (GameObject *particle : objects)
        {
            if (particle == nullptr)
                continue;
            particle->_go_render();
        }

        if (toLog != "")
        {
            // log to console
            std::cout << toLog << std::endl;
            toLog = "";
        }

        if (new_objects.size() > 0)
        {
            for (GameObject *obj : new_objects)
            {
                objects.push_back(obj);
                obj->_go_start();
            }
            new_objects.clear();
        }

        objects.erase(std::remove_if(begin(objects), end(objects), [](GameObject *obj)
                                     {
            if (obj->markedForGarbageCollection){
                delete obj;
                return true;
            }
            return false;
        }),
                      end(objects));

        previousTime = currentTime;
    }
}