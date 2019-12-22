#ifndef FILTER_SIMPLE_H_
#define FILTER_SIMPLE_H_

class FirstOrderFilter {
public:
  // first order filter
  FirstOrderFilter(double x0, double ts, double dt) {
    k = (dt / ts) / (1. + dt / ts);
    x = x0;
  }

  void update(double x) {
    this->x = (1. - k) * this->x + k * x;
  }
  double k, x;
};

#endif

