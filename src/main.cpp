#include "opengl-framework/opengl-framework.hpp"
#include "utils.hpp"
#include <cmath>
#include "iostream"

std::string toLog = "";

struct GameObject;

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
    glm::vec2 position = glm::vec2(0, 0);
    float size = 1.f;
    float rotation = 0.f;

    std::vector<GameObjectComponent*> components = std::vector<GameObjectComponent*>();
    void _go_start()
    {
        start();

        for (GameObjectComponent* component : components)
        {
            component->start();
        }
    }

    virtual void start()
    {
    }

    void _go_physics_process(float delta)
    {
        physics_process(delta);

        for (GameObjectComponent* component : components)
        {
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
            component->late_physics_process(delta);
        }
    }

    virtual void late_physics_process(float delta)
    {
    }

    void _go_render(){
        render();

        for (GameObjectComponent* component : components)
        {
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
        screen_pos += glm::vec2(0.5, 0.5);
        screen_pos.x /= gl::window_width_in_screen_coordinates();
        screen_pos.y /= gl::window_height_in_screen_coordinates();
        utils::draw_disk(screen_pos, object->size / (2 * gl::window_width_in_screen_coordinates()), color);
    }
};

struct SphereCollider : GameObjectComponent
{
    glm::vec2 previous_pos = glm::vec2(0, 0);

    void physics_process(float delta) override
    {
        glm::vec2 old_to_new_vec = object->position - previous_pos;


        //
        glm::vec2 screen_pos = object->position;
        screen_pos += glm::vec2(0.5, 0.5);
        screen_pos.x /= gl::window_width_in_screen_coordinates();
        screen_pos.y /= gl::window_height_in_screen_coordinates();

        glm::vec2 screen_pos2 = previous_pos;
        screen_pos2 += glm::vec2(0.5, 0.5);
        screen_pos2.x /= gl::window_width_in_screen_coordinates();
        screen_pos2.y /= gl::window_height_in_screen_coordinates();

        utils::draw_line(screen_pos2, screen_pos, 0.01f, glm::vec4(0, 1, 0, 1));
        //


        previous_pos = object->position;
    }

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
};

struct Particle : GameObject
{
    float start_size = 1.f;
    float end_size = 0.f;
    glm::vec4 start_color;
    glm::vec4 end_color;
    float life_time = -1.f;
    float age = 0.f;

    RigidBody* rb =new RigidBody();
    SphereRenderer* rend = new SphereRenderer();
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

        position.x = utils::rand(-1, 1) * (gl::window_width_in_screen_coordinates());
        position.y = utils::rand(-1, 1) * (gl::window_height_in_screen_coordinates());
        start_size = 100.f;
        end_size = 50.f;
        start_color = glm::vec4(1.f, 1.f, 1.f, 1.f);
        end_color = glm::vec4(1.f, 1.f, 1.f, 0.5f);
        rb->mass = 100.f;
        rb->add_force(glm::vec2(utils::rand(-10, 10) * (gl::window_width_in_screen_coordinates() / 2), utils::rand(-10, 10) * (gl::window_height_in_screen_coordinates() / 2)));
        life_time = utils::rand(0, 100);
    }

    virtual void physics_process(float delta) override
    {
        rb->add_acceleration(glm::vec2(0, -9.80));
        rb->add_force(rb->get_friction_force(0.0000181f, size));

        if (life_time < 0)
        {
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


    float lerp(float start, float end, float t)
    {
        return (end - start) * t + start;
    }
};

int main()
{
    gl::init("Particules!");
    gl::maximize_window();
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE);

    // TODO: create an array of particles
    std::vector<Particle> particles = std::vector<Particle>(1000);


    for (Particle &particle : particles)
    {
        particle._go_start();
    }

    while (gl::window_is_open())
    {
        glClearColor(0.f, 0.f, 0.f, 1.f);
        glClear(GL_COLOR_BUFFER_BIT);

        for (Particle &particle : particles)
        {
            particle._go_physics_process(gl::delta_time_in_seconds());
        }

        for (Particle &particle : particles)
        {
            particle._go_late_physics_process(gl::delta_time_in_seconds());
        }

        for (Particle &particle : particles)
        {
            particle._go_render();
        }

        if (toLog != "")
        {
            std::cout << toLog;
            toLog = "";
        }
    }

    
}