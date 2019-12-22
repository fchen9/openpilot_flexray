#ifndef STAT_LIVE_H_
#define STAT_LIVE_H_

class RunningStat {
public:
  // tracks realtime mean and standard deviation without storing any data
  RunningStat(const double *priors=NULL, int max_trackable=-1);
  void reset();
  void push_data(double new_data);
  double mean();
  double variance();
  double std();
  std::vector<double> params_to_save();

  int max_trackable;
  double M, S, M_last, S_last, n;
};

class RunningStatFilter {
public:
  RunningStatFilter(const double *raw_priors=NULL, const double *filtered_priors=NULL, int max_trackable=-1);
  void reset();
  void push_and_update(double new_data);

  RunningStat raw_stat, filtered_stat;
};

#endif
