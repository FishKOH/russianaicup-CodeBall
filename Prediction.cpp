#include "Prediction.h"

model::Rules g_rules;

double clamp(double x, double minValue, double maxValue)
{
    return std::min(std::max(minValue, x), maxValue);
}

namespace predict {


Prediction::Prediction()
{

}

void Prediction::predictBall(model::Ball &testBall)
{
    ballTrack.clear();

    for (int t=1; t<36/*magic*/; t+=1)
    {
        move(testBall, 1.0/g_rules.TICKS_PER_SECOND * 5/*magic*/);
        if (fabs(testBall.x) > g_rules.arena.width/2 - g_rules.BALL_RADIUS)
        {
            double correction = fabs(testBall.x) - (g_rules.arena.width/2 - g_rules.BALL_RADIUS);
            if (testBall.x > 0)
                correction *= -1;
            testBall.x += correction;
            testBall.velocity_x *= -g_rules.BALL_ARENA_E;
        }
        if (fabs(testBall.z) > g_rules.arena.depth/2 - g_rules.BALL_RADIUS and
                fabs(testBall.x) > g_rules.arena.goal_width/2)
        {
            double correction = fabs(testBall.z) - (g_rules.arena.depth/2 - g_rules.BALL_RADIUS);
            if (testBall.z > 0)
                correction *= -1;
            testBall.z += correction;
            testBall.velocity_z *= -g_rules.BALL_ARENA_E;
        }
        ballTrack.emplace_back(testBall.x, testBall.z, testBall.y);
    }
}

}
