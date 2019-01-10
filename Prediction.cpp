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

}
