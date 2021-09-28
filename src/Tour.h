#pragma once

#include "SceneObject.h"

#include <vector>

class Tour
{
    struct Stop
    {
        DirectX::XMVECTOR Pos;
        DirectX::XMVECTOR Quat;
    };

    std::vector<Stop> m_stops;
    float m_mu{0};

public:
    DirectX::XMMATRIX Update(std::vector<SceneObjects::BaseObject *> const &objects, float delta);

private:
    void UpdateRoute(std::vector<SceneObjects::BaseObject *> const &objects);
};