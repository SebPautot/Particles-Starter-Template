#version 410
out vec4 out_color;
in vec3  _position;

void main()
{
    vec3  N         = normalize(cross(dFdx(_position), dFdy(_position)));
    vec3  light_dir = normalize(vec3(10, 2, 5));
    float t         = max(0., dot(light_dir, N));
    out_color       = vec4(vec3(t), 1.f);
}