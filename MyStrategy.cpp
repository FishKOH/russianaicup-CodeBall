#include "MyStrategy.h"

#include <iostream>
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
    Point3D botV(me.velocity_x, me.velocity_z, me.velocity_y);

    debug << "id=" << me.id << " x="<< me.x<<" z="<< me.z<<" role=" << static_cast<int>(myRole) << " " << static_cast<int>(defState);
    print();

    model::Ball testBall = game.ball;
    pred.predictBall(testBall);
    for (auto& predictedBall : pred.ballTrack)
        spheres.push_back(Sphere{predictedBall.x, predictedBall.y, predictedBall.z, 0.3, 1.0, 0.0, 0.0, 0.5});

    if (myRole == Role::Defender)
    {        
        bool isWarning;
        Point3D goal;
        double t;
        std::tie(isWarning, goal, t) = goalWarning();
        if (isWarning)
        {
            debug << "WARNING";
            print();
            goToPoint(me, action, goal.x, goal.z, Go::InTime, t);

            //TODO: predict jump
            std::vector<Point3D> pJ;
            int nJ;
            double jumpSpeed;
            std::tie(nJ, jumpSpeed, pJ) = pred.findJumpSpeed(me);
            for (auto & b : pJ)
                spheres.push_back(Sphere{b.x, b.y, b.z, 0.2, 0.0, 1.0, 1.0, 0.5});

            if (nJ != -1)
            {
                action.jump_speed = jumpSpeed;
                debug << "findjumpspeed: " << jumpSpeed ;
                print();
                spheres.back().radius = 0.5;
                spheres.back().g = 0.5;
                debug << "jump is detected: " << nJ;
                print();
            }
//            std::vector<Point3D> pJ;
//            int nJ;
//            std::tie(nJ, pJ) = pred.predictBot(me, g_rules.ROBOT_MAX_JUMP_SPEED);
//            for (auto & b : pJ)
//                spheres.push_back(Sphere{b.x, b.y, b.z, 0.2, 0.0, 1.0, 1.0, 0.5});
//            if (nJ != -1)
//            {
//                if (pred.ballTrack[nJ].y < pJ[nJ].y)
//                {
//                    double needJumpSpeed = g_rules.ROBOT_MAX_JUMP_SPEED / pJ[nJ].y * pred.ballTrack[nJ].y;
//                    debug << "jump is bad: " << pred.ballTrack[nJ].y << " " << pJ[nJ].y << " try speed=" << needJumpSpeed ;
//                    print();
//                    std::tie(nJ, pJ) = pred.predictBot(me, needJumpSpeed);
//                    for (auto & b : pJ)
//                        spheres.push_back(Sphere{b.x, b.y, b.z, 0.2, 1.0, 0.5, 0.0, 0.5});
//                    if (nJ != -1)
//                    {
//                        spheres.back().radius = 0.5;
//                        spheres.back().r = 0.5;
//                        debug << "jump is detected: " << nJ;
//                        print();
//                        action.jump_speed = needJumpSpeed;
//                    }
//                }
//                else
//                {
//                    spheres.back().radius = 0.5;
//                    spheres.back().g = 0.5;
//                    debug << "jump is detected: " << nJ;
//                    print();
//                    action.jump_speed = g_rules.ROBOT_MAX_JUMP_SPEED;
//                }
//            }

//            if (bot.distTo(game.ball.x, game.ball.z, game.ball.y)
//                    < g_rules.BALL_RADIUS + g_rules.ROBOT_MIN_RADIUS + 1/*magic*/)
//            {
//                action.jump_speed = g_rules.ROBOT_MAX_JUMP_SPEED;
//            }
            lines.push_back(Line{goal.x, 0, goal.z, goal.x, g_rules.arena.height, goal.z, 0.2, 0, 0, 0, 1});
        }
        else
        {
            //надо выбить мяч подальше от ворот
            //TODO: if i run before opponent/anybody
//                if (game.ball.z < -g_rules.arena.depth/4 /*magic*/)
//                {
//                    debug << "vipnut ball";
//                    print();
//                    goToPoint(me, action, game.ball.x, game.ball.z);
//                    if (bot.distTo(game.ball.x, game.ball.z, game.ball.y)
//                            < g_rules.BALL_RADIUS + g_rules.ROBOT_MIN_RADIUS + 1/*magic*/)
//                    {
//                        debug << "jump";
//                        print();
//                        action.jump_speed = g_rules.ROBOT_MAX_JUMP_SPEED;
//                    }
//                }
//                else
            {
                debug << "look at ball";
                print();
                goToPoint(me,
                          action,
                          clamp(game.ball.x,
                                -g_rules.arena.goal_width/2 + g_rules.arena.goal_top_radius,
                                g_rules.arena.goal_width/2 - g_rules.arena.goal_top_radius),
                          -g_rules.arena.depth/2 - g_rules.arena.goal_depth + g_rules.arena.goal_top_radius,
                          Go::Stand);
            }
        }
    }
    else if (myRole == Role::Attacker)
    {
        Point3D ball(game.ball.x, game.ball.z, game.ball.y);

        if (isNewRound(game))
        {
            Point2D botVel(me.velocity_x, me.velocity_z);
            debug << "fast "<< bot.distTo(0.0, 0.0, 1.0) << " " << game.current_tick << " " << botVel.dist();
            print();

            Point2D delta_pos(0 - me.x, 0 - me.z);
            double delta_pos_dist = delta_pos.dist();
            Point2D target_velocity(delta_pos.normalize(delta_pos_dist)*g_rules.ROBOT_MAX_GROUND_SPEED);
            action.target_velocity_x = target_velocity.x;
            action.target_velocity_z = target_velocity.z;

            std::vector<Point3D> pJ;
            int nJ;
            double jumpSpeed;
            std::tie(nJ, jumpSpeed, pJ) = pred.findJumpSpeed(me);
            for (auto & b : pJ)
                spheres.push_back(Sphere{b.x, b.y, b.z, 0.2, 0.0, 1.0, 1.0, 0.5});

            if (nJ != -1)
            {
                action.jump_speed = jumpSpeed;
                debug << "findjumpspeed: " << jumpSpeed ;
                print();
                spheres.back().radius = 0.5;
                spheres.back().g = 0.5;
                debug << "jump is detected: " << nJ;
                print();
            }
//            std::vector<Point3D> pJ;
//            int nJ;
//            std::tie(nJ, pJ) = pred.predictBot(me, g_rules.ROBOT_MAX_JUMP_SPEED);
//            for (auto & b : pJ)
//                spheres.push_back(Sphere{b.x, b.y, b.z, 0.2, 0.0, 1.0, 1.0, 0.5});
//            if (nJ != -1)
//            {
//                if (pred.ballTrack[nJ].y < pJ[nJ].y)
//                {
//                    double needJumpSpeed = g_rules.ROBOT_MAX_JUMP_SPEED / pJ[nJ].y * pred.ballTrack[nJ].y;
//                    debug << "jump is bad: " << pred.ballTrack[nJ].y << " " << pJ[nJ].y << " try speed=" << needJumpSpeed ;
//                    print();
//                    std::tie(nJ, pJ) = pred.predictBot(me, needJumpSpeed);
//                    for (auto & b : pJ)
//                        spheres.push_back(Sphere{b.x, b.y, b.z, 0.2, 1.0, 0.5, 0.0, 0.5});
//                    if (nJ != -1)
//                    {
//                        spheres.back().radius = 0.5;
//                        spheres.back().r = 0.5;
//                        debug << "jump is detected: " << nJ;
//                        print();
//                        action.jump_speed = needJumpSpeed;
//                    }
//                }
//                else
//                {
//                    spheres.back().radius = 0.5;
//                    spheres.back().g = 0.5;
//                    debug << "jump is detected: " << nJ;
//                    print();
//                    action.jump_speed = g_rules.ROBOT_MAX_JUMP_SPEED;
//                }
//            }
        }
        else
        {
            bool pathIsFind =false;
            //не прыгай если не достанешь
            //TODO: predict jump
//            bool jump = (ball.distTo(me.x, me.z, me.y) < (g_rules.BALL_RADIUS + g_rules.ROBOT_MAX_RADIUS + 1.5/*magic*/)
//                         and me.y < ball.y
//                         and me.z < ball.z);

            std::vector<Point3D> pJ;
            int nJ;
            double jumpSpeed;
            std::tie(nJ, jumpSpeed, pJ) = pred.findJumpSpeed(me);
            for (auto & b : pJ)
                spheres.push_back(Sphere{b.x, b.y, b.z, 0.2, 0.0, 1.0, 1.0, 0.5});

            if (nJ != -1)
            {
                action.jump_speed = jumpSpeed;
                debug << "findjumpspeed: " << jumpSpeed ;
                print();
                spheres.back().radius = 0.5;
                spheres.back().g = 0.5;
                debug << "jump is detected: " << nJ;
                print();
                if (me.z > ball.z or pJ[nJ].z > pred.ballTrack[nJ].z)
                {
                    debug << "but i dont jump because warning goal";
                    print();
                    action.jump_speed = 0;
                }
            }

//            std::vector<Point3D> pJ;
//            int nJ;
//            std::tie(nJ, pJ) = pred.predictBot(me, g_rules.ROBOT_MAX_JUMP_SPEED);
//            for (auto & b : pJ)
//                spheres.push_back(Sphere{b.x, b.y, b.z, 0.2, 0.0, 1.0, 1.0, 0.5});
//            if (nJ != -1)
//            {
//                if (pred.ballTrack[nJ].y < pJ[nJ].y)
//                {
//                    double needJumpSpeed = g_rules.ROBOT_MAX_JUMP_SPEED / pJ[nJ].y * pred.ballTrack[nJ].y/ pJ[nJ].y * pred.ballTrack[nJ].y;
//                    debug << "jump is bad: " << pred.ballTrack[nJ].y << " " << pJ[nJ].y << " try speed=" << needJumpSpeed ;
//                    print();
//                    std::tie(nJ, pJ) = pred.predictBot(me, needJumpSpeed);
//                    for (auto & b : pJ)
//                        spheres.push_back(Sphere{b.x, b.y, b.z, 0.2, 1.0, 0.5, 0.0, 0.5});
//                    if (nJ != -1)
//                    {
//                        spheres.back().radius = 0.5;
//                        spheres.back().r = 0.5;
//                        debug << "jump is detected: " << nJ;
//                        print();
//                        action.jump_speed = needJumpSpeed;
//                    }
//                }
//                else
//                {
//                    spheres.back().radius = 0.5;
//                    spheres.back().g = 0.5;
//                    debug << "jump is detected: " << nJ;
//                    action.jump_speed = g_rules.ROBOT_MAX_JUMP_SPEED;
//                }

//                if (me.z > ball.z or pJ[nJ].z > pred.ballTrack[nJ].z)
//                {
//                    debug << "but i dont jump because warning goal";
//                    action.jump_speed = 0;
//                }
//                print();
//            }
            double t = 0;
            for (auto& predictedBall : pred.ballTrack)
            {
                t += 1.0/12.0;
                // Если мяч не вылетит за пределы арены
                // (произойдет столкновение со стеной, которое мы не рассматриваем),
                // и при этом мяч будет находится ближе к вражеским воротам, чем робот,
                double s_z = me.z + me.velocity_z * t - g_rules.ROBOT_ACCELERATION * t * t /2;
                if (predictedBall.z > s_z and predictedBall.y < HIGHT + g_rules.BALL_RADIUS)
                {
                    // Посчитаем, с какой скоростью робот должен бежать,
                    // Чтобы прийти туда же, где будет мяч, в то же самое время
                    Point2D delta_pos(predictedBall.x - me.x, predictedBall.z - me.z);
                    double delta_pos_dist = delta_pos.dist();
                    double need_speed = delta_pos_dist / t;
                    // Если эта скорость лежит в допустимом отрезке
                    if (/*0.5 * g_rules.ROBOT_MAX_GROUND_SPEED < need_speed
                        &&*/ need_speed < g_rules.ROBOT_MAX_GROUND_SPEED ) {
                        // То это и будет наше текущее действие
                        Point2D target_velocity(delta_pos.normalize(delta_pos_dist)*need_speed);
                        action.target_velocity_x = target_velocity.x;
                        action.target_velocity_z = target_velocity.z;
                        action.target_velocity_y = 0.0;
//                        action.jump_speed = jump ? g_rules.ROBOT_MAX_JUMP_SPEED : 0.0;
                        action.use_nitro = false;
                        pathIsFind = true;
                        debug << "go to ball:" << t*12;
                        print();
                        spheres.push_back(Sphere{predictedBall.x, predictedBall.y, predictedBall.z, 0.4, 0.0, 0.0, 1.0, 0.7});
                        break;
                    }
                    else
                    {
                        debug << "ball not dosijim:" << t*12 << " need_speed = " << need_speed;
                        print();
                        spheres.push_back(Sphere{predictedBall.x, predictedBall.y, predictedBall.z, 0.4, 1.0, 1.0, 0.0, 0.7});
                    }
                }
            }
            if (not pathIsFind)
            {
                debug << "go to gate";
                print();
                goToPoint(me, action, pred.ballTrack[12].x, pred.ballTrack[12].z, Go::Stand);
                spheres.push_back(Sphere{pred.ballTrack[12].x, pred.ballTrack[12].y, pred.ballTrack[12].z, 0.4, 1.0, 0.0, 1.0, 0.7});
            }
        }
    }

//    debug
    {
//        lines.push_back(Line{me.x, me.y, me.z, me.x+me.velocity_x*10, me.y+me.velocity_y*10, me.z+me.velocity_z*10, 1.0, 1.0, 0.0, 0.0, 1.0});
//        spheres.push_back(Sphere{me.x, me.y, me.z, 3.0, 1.0, 0.0, 0.0, 1.0});
    }
}

bool MyStrategy::isNewRound(const Game &game)
{
    return game.ball.x ==0 && game.ball.z == 0 && game.ball.velocity_x == 0;
}

void MyStrategy::print()
{
    std::string t_str;
    std::getline(debug, t_str);
    debug.clear();
    texts.push_back(t_str);
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

void MyStrategy::goToPoint(const Robot &me, Action &action, double x, double z, Go accuracy, double t)
{
    if (accuracy == Go::InTime)
    {
        Point2D delta_pos(x - me.x, z - me.z);
        double delta_pos_dist = delta_pos.dist();
        double need_speed = delta_pos_dist / t;
        // Если эта скорость лежит в допустимом отрезке
        need_speed = clamp(need_speed,
                           0, g_rules.ROBOT_MAX_GROUND_SPEED);
        Point2D target_velocity(delta_pos.normalize(delta_pos_dist)*need_speed);
        action.target_velocity_x = target_velocity.x;
        action.target_velocity_z = target_velocity.z;
    }
    else if (accuracy == Go::Stand)
    {
        Point2D v(me.velocity_x, me.velocity_z);
        double t = v.dist()/g_rules.ROBOT_ACCELERATION;
        double s = v.dist()*t - g_rules.ROBOT_ACCELERATION * t *t /2;

        Point2D delta_pos(x - me.x, z - me.z);
        double delta_pos_dist = delta_pos.dist();
        if (s >= delta_pos_dist)
        {
            debug << "try to stop";
            print();
            action.target_velocity_x = 0;
            action.target_velocity_z = 0;
        }
        else
        {
            debug << "try to go fast";
            print();
            action.target_velocity_x = (x - me.x)*g_rules.ROBOT_MAX_GROUND_SPEED;
            action.target_velocity_z = (z - me.z)*g_rules.ROBOT_MAX_GROUND_SPEED;
        }
    }
    else
    {
        action.target_velocity_x = (x - me.x)*g_rules.ROBOT_MAX_GROUND_SPEED;
        action.target_velocity_z = (z - me.z)*g_rules.ROBOT_MAX_GROUND_SPEED;
    }
}

std::tuple<bool, Point3D, double> MyStrategy::goalWarning()
{
    bool isWarning = false;
    Point3D goal;
    double t=0;
    for (auto& predictedBall : pred.ballTrack)
    {
        t += 1.0/12.0;
        if (fabs(predictedBall.x) < (g_rules.arena.goal_width / 2.0) and
                predictedBall.z < -g_rules.arena.depth/2) {
            isWarning = true;
            goal = predictedBall;
            break;
        }
    }
    return std::make_tuple(isWarning, goal, t);
}
