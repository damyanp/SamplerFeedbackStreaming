#include "pch.h"

#include "Tour.h"

#include "SceneObject.h"

using namespace DirectX;

XMMATRIX Tour::Update(std::vector<SceneObjects::BaseObject *> const &objects, float delta)
{
    UpdateRoute(objects);

    m_mu += delta * 0.01f;

    float t = m_mu - std::floor(m_mu);

    size_t index[4];
    index[0] = static_cast<size_t>(std::floor(m_mu)) % m_stops.size();
    index[1] = (index[0] + 1) % m_stops.size();
    index[2] = (index[1] + 1) % m_stops.size();
    index[3] = (index[2] + 1) % m_stops.size();

    float at = std::min(std::max(0.0f, t / 0.6f - 0.2f), 1.0f);

    float st = (std::sin(t * XM_PI - XM_PIDIV2) + 1) / 2;
    float sat = (std::sin(at * XM_PI - XM_PIDIV2) + 1) / 2;


    auto p = XMVectorCatmullRom(m_stops[index[0]].Pos, m_stops[index[1]].Pos, m_stops[index[2]].Pos,
                                m_stops[index[3]].Pos, st);

    auto q = XMQuaternionSlerp(m_stops[index[1]].Quat, m_stops[index[2]].Quat, sat);

    return XMMatrixMultiply(XMMatrixRotationQuaternion(q), XMMatrixTranslationFromVector(p));
}

void Tour::UpdateRoute(std::vector<SceneObjects::BaseObject *> const &objects)
{
    if (m_stops.size() == objects.size())
        return;

    struct Planet
    {
        XMVECTOR Pos;
        float Scale;
    };

    std::vector<Planet> planets;
    planets.reserve(objects.size());

    std::transform(objects.begin(), objects.end(), back_inserter(planets), [](SceneObjects::BaseObject *o) {
        XMVECTOR scale, rotQuat, position;
        XMMatrixDecompose(&scale, &rotQuat, &position, o->GetModelMatrix());

        return Planet{position, std::max({XMVectorGetX(scale), XMVectorGetY(scale), XMVectorGetZ(scale)})};
    });

    XMVECTOR lastPos = XMVectorZero();
    m_stops.clear();
    m_stops.reserve(planets.size());

    while (!planets.empty())
    {
        // sort by distance from lastPos
        std::sort(planets.begin(), planets.end(), [lastPos](auto &l, auto &r) {
            return XMVectorGetX(XMVector3Length(l.Pos - lastPos)) > XMVectorGetX(XMVector3Length(r.Pos - lastPos));
        });

        Planet const &p = planets.back();

        XMVECTOR toLastPos = XMVector3Normalize(p.Pos - lastPos);
        if (XMVector3Equal(toLastPos, XMVectorZero()))
        {
            toLastPos = XMVectorSet(1, 0, 0, 0);
        }

        XMVECTOR newPos = p.Pos + (toLastPos * p.Scale * 3);

        XMMATRIX m = XMMatrixLookAtLH(newPos, p.Pos, XMVectorSet(0, 1, 0, 0));

        XMVECTOR scale, rotQuat, position;
        XMMatrixDecompose(&scale, &rotQuat, &position, m);

        lastPos = newPos;
        m_stops.push_back(Stop{position, rotQuat});
        planets.pop_back();
    }

    m_mu = 0;
}