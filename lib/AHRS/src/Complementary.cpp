#include "Complementary.hpp"

void Complementary::begin(float sampleRate, float tau, float state)
{
  _dt = 1.0f / sampleRate;
  _alpha = tau / (tau + _dt);
  _state = state;
}

float Complementary::update(float rate, float position)
{
  _state = _alpha * (_state + rate * _dt) + (1.0f - _alpha) * position;
  return _state;
}
