#include "MyStrategy.h"

#include <iostream>
#include <sstream>
#include <cmath>
using namespace model;

MyStrategy::MyStrategy() { }

void MyStrategy::act(const Robot& me, const Rules& rules, const Game& game, Action& action) {

    //update rules or init rules
    g_rules = rules;
    static bool start = true;
    if (isNewRound(game))
        start = true;
    Role myRole = chooseRole(me, game);
    static DefenderState defState;
    if (start/*needChooseRole on start or after goal*/)
    {
        start = false;
        defState = DefenderState::ToGate;
    }

    Point3D bot(me.x, me.z, me.y);

    model::Ball testBall = game.ball;
    for (int t=1; t<36/*magic*/; t+=1)
    {
        predict::move(testBall, 1.0/g_rules.TICKS_PER_SECOND * 5/*magic*/);
        spheres.push_back(Sphere{testBall.x, testBall.y, testBall.z, 3.0, 1.0, 0.0, 0.0, 1.0});
    }

    if (myRole == Role::Defender)
    {
        if (defState == DefenderState::ToGate)
        {
            if (bot.distTo(0.0, -rules.arena.depth/2 + rules.arena.bottom_radius, 1.0)<1/*magic*/)
                defState = DefenderState::OnGate;
            else
                goToPoint(me, action, 0.0, -rules.arena.depth/2 + rules.arena.bottom_radius);
        }
        if (defState == DefenderState::OnGate) {
            bool isWarning;
            double x,z;
            std::tie(isWarning, x, z) = goalWarning();
            if (isWarning)
            {
                std::stringstream t;
                t << "WARNING!\nLOL";
                std::string t_str;
                std::getline(t, t_str);
                texts.push_back(t_str);
                goToPoint(me, action, x, z);
                if (bot.distTo(game.ball.x, game.ball.z, game.ball.y)
                        < rules.BALL_RADIUS + rules.ROBOT_MIN_RADIUS + 1/*magic*/)
                {
                    action.jump_speed = rules.ROBOT_MAX_JUMP_SPEED;
                }
            }
            else
            {
                goToPoint(me,
                          action,
                          clamp(-game.ball.x,
                                -rules.arena.goal_width/2 + rules.arena.goal_top_radius,
                                rules.arena.goal_width/2 - rules.arena.goal_top_radius),
                          -rules.arena.depth/2 + rules.arena.bottom_radius);
                //надо выбить мяч подальше от ворот
                if (game.ball.z < -rules.arena.depth/4 /*magic*/)
                {
                    goToPoint(me, action, game.ball.x, game.ball.z);
                    if (bot.distTo(game.ball.x, game.ball.z, game.ball.y)
                            < rules.BALL_RADIUS + rules.ROBOT_MIN_RADIUS + 1/*magic*/)
                    {
                        action.jump_speed = rules.ROBOT_MAX_JUMP_SPEED;
                    }
                }
            }
        }
    }
    else if (myRole == Role::Attacker)
    {
        Point3D ball(game.ball.x, game.ball.z, game.ball.y);

        bool pathIsFind =false;
        //не прыгай если не достанешь
        bool jump = (   ball.distTo(me.x, me.z, me.y) < (rules.BALL_RADIUS + rules.ROBOT_MAX_RADIUS + 1.5/*magic*/)
                     && me.y < ball.y );
        double t = 0;
        for (auto& sph : spheres)
        {
            t += 1.0/12.0;
            // Если мяч не вылетит за пределы арены
            // (произойдет столкновение со стеной, которое мы не рассматриваем),
            // и при этом мяч будет находится ближе к вражеским воротам, чем робот,
            if (   sph.z > me.z
                && abs(game.ball.x) < (rules.arena.width / 2.0)
                && abs(game.ball.z) < (rules.arena.depth / 2.0) )
            {
                // Посчитаем, с какой скоростью робот должен бежать,
                // Чтобы прийти туда же, где будет мяч, в то же самое время
                Point2D delta_pos(sph.x - me.x, sph.z - me.z);
                double delta_pos_dist = delta_pos.dist();
                double need_speed = delta_pos_dist / t;
                // Если эта скорость лежит в допустимом отрезке
                if (0.5 * rules.ROBOT_MAX_GROUND_SPEED < need_speed
                    && need_speed < rules.ROBOT_MAX_GROUND_SPEED ) {
                    // То это и будет наше текущее действие
                    Point2D target_velocity(delta_pos.normalize(delta_pos_dist)*need_speed);
                    action.target_velocity_x = target_velocity.x;
                    action.target_velocity_z = target_velocity.z;
                    action.target_velocity_y = 0.0;
                    action.jump_speed = jump ? rules.ROBOT_MAX_JUMP_SPEED : 0.0;
                    action.use_nitro = false;
                    pathIsFind = true;
                    break;
                }
            }
        }
        if (not pathIsFind)
        {
            goToPoint(me, action, 0.0, -rules.arena.depth/2 + rules.arena.bottom_radius);
        }
    }

//    debug
    {
        std::stringstream t;
        t << "id=" << me.id << " x="<< me.x<<" z="<< me.z<<" role=" << static_cast<int>(myRole) << " " << static_cast<int>(defState);
        std::string t_str;
        std::getline(t, t_str);
        texts.push_back(t_str);
//        lines.push_back(Line{me.x, me.y, me.z, me.x+me.velocity_x*10, me.y+me.velocity_y*10, me.z+me.velocity_z*10, 1.0, 1.0, 0.0, 0.0, 1.0});
//        spheres.push_back(Sphere{me.x, me.y, me.z, 3.0, 1.0, 0.0, 0.0, 1.0});
    }
}

bool MyStrategy::isNewRound(const Game &game)
{
    return game.ball.x ==0 && game.ball.z == 0 && game.ball.velocity_x == 0;
}

std::string MyStrategy::custom_rendering()
{
    std::stringstream t;

    for (auto& text : texts)
    {
        t << "{\"Text\": \"" << text << "\"},";
    }
    for (auto& line : lines)
    {
        t << "{\"Line\": "
             "{\"x1\": " << line.x1
          << ",\"y1\": " << line.y1
          << ",\"z1\": " << line.z1
          << ",\"x2\": " << line.x2
          << ",\"y2\": " << line.y2
          << ",\"z2\": " << line.z2
          << ",\"width\": " << line.width
          << ",\"r\": " << line.r
          << ",\"g\": " << line.g
          << ",\"b\": " << line.b
          << ",\"a\": " << line.a
          << "}},";
    }
    for (auto& sph : spheres)
    {
        t << "{\"Sphere\": "
          << "{\"x\": " << sph.x
          << ",\"y\": " << sph.y
          << ",\"z\": " << sph.z
          << ",\"radius\": " << sph.radius
          << ",\"r\": " << sph.r
          << ",\"g\": " << sph.g
          << ",\"b\": " << sph.b
          << ",\"a\": " << sph.a
          << "}},";
    }
    texts.clear();
    lines.clear();
    spheres.clear();
    std::string t_str;
    std::getline(t, t_str);
    if (not t_str.empty())
        t_str.pop_back();

    std::string result_str = "[" + t_str + "]";
    return result_str;
}

MyStrategy::Role MyStrategy::chooseRole(const Robot &me, const Game &game)
{
    Role resultRole = Role::Defender;
    for (auto& robot : game.robots)
    {
        if (robot.is_teammate && robot.id != me.id) {
            if (robot.z < me.z) {
                resultRole = Role::Attacker;
            }
        }
    }
    return resultRole;
}

void MyStrategy::goToPoint(const Robot &me, Action &action, double x, double z)
{
    action.target_velocity_x = (x - me.x)*g_rules.ROBOT_MAX_GROUND_SPEED;
    action.target_velocity_z = (z - me.z)*g_rules.ROBOT_MAX_GROUND_SPEED;
}

std::tuple<bool, double, double> MyStrategy::goalWarning()
{
    bool isWarning = false;
    double x=0,z=0;
    for (auto& sph : spheres)
    {
        if (fabs(sph.x) < (g_rules.arena.goal_width / 2.0)) {
            isWarning = true;
            x = sph.x;
            z = sph.z;
            break;
        }
    }
    return std::make_tuple(isWarning, x, z);
}
