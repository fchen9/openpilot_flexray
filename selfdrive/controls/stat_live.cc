#include <vector>
#include <cmath>
#include "stat_live.h"

// tracks realtime mean and standard deviation without storing any data
RunningStat::RunningStat(const double *priors, int max_trackable) {
  this->max_trackable = max_trackable;
  if (priors != NULL) {
    // initialize from history
    M = priors[0];
    S = priors[1];
    n = priors[2];
    M_last = M;
    S_last = S;
  } else
    reset();
}

void RunningStat::reset() {
  M = 0.;
  S = 0.;
  M_last = 0.;
  S_last = 0.;
  n = 0;
}

void RunningStat::push_data(double new_data) {
  // short term memory hack
  if (max_trackable < 0 || n < max_trackable)
    n += 1;
  if (n == 0) {
    M_last = new_data;
    M = M_last;
    S_last = 0.;
  } else {
    M = M_last + (new_data - M_last) / n;
    S = S_last + (new_data - M_last) * (new_data - M);
    M_last = M;
    S_last = S;
  }
}

double RunningStat::mean() { return M; }

double RunningStat::variance() {
  if (n >= 2)
    return S / (n - 1.);
  else
    return 0;
}

double RunningStat::std() {
  return std::sqrt(variance());
}

std::vector<double> RunningStat::params_to_save() {
  double v[3] = {M, S, n};
  return std::vector<double>(v, v + sizeof(v) / sizeof(v[0]));
}

RunningStatFilter::RunningStatFilter(const double *raw_priors, const double *filtered_priors, int max_trackable):
  raw_stat(raw_priors, max_trackable),
  filtered_stat(filtered_priors, max_trackable)
{
}

void RunningStatFilter::reset() {
  raw_stat.reset();
  filtered_stat.reset();
}

void RunningStatFilter::push_and_update(double new_data) {
  double _std_last = raw_stat.std();
  raw_stat.push_data(new_data);
  double _delta_std = raw_stat.std() - _std_last;
  if (_delta_std <= 0)
    filtered_stat.push_data(new_data);
  else {
    // filtered_stat.push_data(self.filtered_stat.mean())
  }
}
