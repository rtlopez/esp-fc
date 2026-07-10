#pragma once

// complementary filter for fusing rate and position measurements
class Complementary
{
public:
  void begin(float sampleRate, float tau, float state = 0.0f)
  {
    _dt = 1.0f / sampleRate;
    _alpha = tau / (tau + _dt);
    _state = state;
  }

  float update(float rate, float position)
  {
    _state = _alpha * (_state + rate * _dt) + (1.0f - _alpha) * position;
    return _state;
  }

private:
  float _dt{0.001f};
  float _alpha{0.998f};
  float _state{0.0f};
};
