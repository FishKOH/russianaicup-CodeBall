#if defined(_MSC_VER) && (_MSC_VER >= 1200)
#pragma once
#endif

#ifndef _MY_STRATEGY_H_
#define _MY_STRATEGY_H_
#include <sstream>
#include <tuple>

#include "Strategy.h"

#include "Prediction.h"

const double EPS = 1e-5;
const double HIGHT = 3.75;
const double HIGHT_TIME = 0.5;

class MyStrategy : public Strategy {
    enum class Role{ Attacker, Defender };
    enum class DefenderState{ToGate, OnGate};
    enum class Go{Stand, InTime, VeryFast};
public:
    MyStrategy();

    void act(const model::Robot& me, const model::Rules& rules, const model::Game& game, model::Action& action) override;
    std::string custom_rendering() override;
private:
    bool isNewRound(const model::Game& game);
    Role chooseRole(const model::Robot& me, const model::Game& game);
    void goToPoint(const model::Robot& me, model::Action& action, double x, double z, Go accuracy = Go::VeryFast, double t = 0);
    std::tuple<bool, Point3D, double> goalWarning();
private:
    predict::Prediction pred;
//    debug
private:
    std::stringstream debug;
    void print();

    struct Line{
        double x1, y1, z1, x2, y2, z2, width, r, g, b, a;
    };
    struct Sphere{
        double x, y, z, radius, r, g, b, a;
    };
    std::vector<Line> lines;
    std::vector<Sphere> spheres;
    std::vector<std::string> texts;
};

#endif
