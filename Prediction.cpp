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

bool Prediction::predictJumpInNextTick(model::Robot bot, model::Ball ball)
{
    Point2D delta_pos(ball.x - bot.x, ball.z - bot.z);
    double delta_pos_dist = delta_pos.dist();

    Point2D target_velocity(delta_pos.normalize(delta_pos_dist)*g_rules.ROBOT_MAX_GROUND_SPEED);

    bot.x += target_velocity.x * 1.0/60;
    bot.z += target_velocity.z * 1.0/60;
    bot.velocity_x += std::min(fabs(target_velocity.x-bot.velocity_x), g_rules.ROBOT_ACCELERATION /60.0);
    bot.velocity_x += std::min(fabs(target_velocity.x-bot.velocity_x), g_rules.ROBOT_ACCELERATION /60.0);

    std::vector<Point3D> pJ;
    int nJ;
    double jumpSpeed;
    std::tie(nJ, jumpSpeed, pJ) = findJumpSpeed(bot);
    return (nJ != -1 );
}

std::tuple<int, double, std::vector<Point3D> > Prediction::findJumpSpeed(model::Robot bot)
{
    bool isFind = false;
    double jumpSpeed = g_rules.ROBOT_MAX_JUMP_SPEED;

    std::vector<Point3D> res;
    int n = -1;

    if (not bot.touch)
    {
        std::tie(n, res)= predictBot(bot, 0);
        return std::make_tuple(n, jumpSpeed, res);
    }
    for (jumpSpeed = g_rules.ROBOT_MAX_JUMP_SPEED; jumpSpeed > 0; jumpSpeed -= 1.5/*magic*/)
    {
        res.clear();
        n = -1;
        bot.velocity_y = jumpSpeed;
        model::Robot copybot = bot;
        std::tie(n, res) = predictBot(copybot, jumpSpeed);
        if (n != -1)
        {
            if (res.back().y < ballTrack[n].y)
            {
                break;
            }
        }
    }

    return std::make_tuple(n, jumpSpeed, res);
}

std::tuple<int, std::vector<Point3D> > Prediction::predictBot(model::Robot bot, double jumpSpeed)
{
    std::vector<Point3D> res;
    int n = -1;
    if (bot.touch)
        bot.velocity_y = jumpSpeed;

    for (int i=0; i<tick; ++i)
    {
        move(bot, periodInSec);
        //TODO: collide_with_arena
        if (bot.velocity_y < 0)
            break;

        res.emplace_back(bot.x, bot.z, bot.y);
        if (res.back().distTo(ballTrack[i]) <= g_rules.BALL_RADIUS + g_rules.ROBOT_RADIUS)
        {
            n = i;
            break;
        }
    }

    return std::make_tuple(n, res);
}

void Prediction::predictBall(model::Ball &testBall)
{
    ballTrack.clear();

    for (int t=1; t<tick; t+=1)
    {
        move(testBall, periodInSec);
        //TODO: collide_with_arena
        if (fabs(testBall.x) > g_rules.arena.width/2 - g_rules.BALL_RADIUS)
        {
            double correction = fabs(testBall.x) - (g_rules.arena.width/2 - g_rules.BALL_RADIUS);
            if (testBall.x > 0)
                correction *= -1;
            testBall.x += 2*correction;
            testBall.velocity_x *= -g_rules.BALL_ARENA_E;
        }
        double zLimit;
        if (fabs(testBall.x) > g_rules.arena.goal_width/2 or
            testBall.y > g_rules.arena.goal_height)
            zLimit = g_rules.arena.depth/2 - g_rules.BALL_RADIUS;
        else
            zLimit = g_rules.arena.depth/2 + g_rules.arena.goal_depth - g_rules.BALL_RADIUS;

        if (fabs(testBall.z) > zLimit)
        {
            double correction = fabs(testBall.z) - zLimit;
            if (testBall.z > 0)
                correction *= -1;
            testBall.z += 2*correction;
            testBall.velocity_z *= -g_rules.BALL_ARENA_E;
        }
        if (testBall.y < g_rules.BALL_RADIUS)
        {
            testBall.y -= 2*(testBall.y - g_rules.BALL_RADIUS);
            testBall.velocity_y *= -g_rules.BALL_ARENA_E;
        }
        if (testBall.y > g_rules.arena.height - g_rules.BALL_RADIUS)
        {
            testBall.y -= 2*(testBall.y - (g_rules.arena.height - g_rules.BALL_RADIUS));
            testBall.velocity_y *= -g_rules.BALL_ARENA_E;
        }

        ballTrack.emplace_back(testBall.x, testBall.z, testBall.y);
    }
}

}
