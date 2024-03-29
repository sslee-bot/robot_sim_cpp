#include "ss_algorithm/control/WheeledMobileRobotController/Jang2009.h"

Jang2009::Jang2009(double gamma_1, double gamma_2, double h)
    : m_gamma_1(gamma_1), m_gamma_2(gamma_2), m_h(h)
{
    // TODO: Check validities of the controller parameters
}

Jang2009::~Jang2009()
{
}

void Jang2009::setParams(const std::vector<double>& params)
{
    try {
        m_gamma_1 = params.at(0);
        m_gamma_2 = params.at(1);
        m_h = params.at(2);
    }
    catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
    }
}

Eigen::Vector2d Jang2009::poseControl(const Eigen::Vector3d& currentState,
                                      const Eigen::Vector3d& desiredState)
{
    double e, phi, alpha;

    Eigen::Vector3d delta = desiredState - currentState;
    delta[2] = wrapAngle(delta[2]);

    e = delta.head(2).norm();
    phi = wrapAngle(desiredState[2] - std::atan2(delta[1], delta[0]));
    alpha = wrapAngle(phi - delta[2]);

    double linearVel = 0.0, angularVel = 0.0;
    linearVel = m_gamma_1 * e * std::cos(alpha);
    if (alpha == 0.0) {
        angularVel = 0.0;
    }
    else {
        angularVel = -m_gamma_2 * alpha -
                     m_gamma_1 * std::cos(alpha) * std::sin(alpha) / alpha * (alpha + m_h * phi);
    }

    Eigen::Vector2d ret;
    ret << linearVel, angularVel;
    return ret;
}
