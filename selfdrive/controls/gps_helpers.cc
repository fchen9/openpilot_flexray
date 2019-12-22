#include <tuple>
#include <vector>
#include <string>
#include "gps_helpers.h"

static const std::vector<std::tuple<std::string, std::vector<double>>> _RHD_REGION_MAP = {
  std::make_tuple(std::string("AUS"), std::vector<double>({-54.76, -9.23, 112.91, 159.11})), \
  std::make_tuple(std::string("IN1"), std::vector<double>({6.75, 28.10, 68.17, 97.4})), \
  std::make_tuple(std::string("IN2"), std::vector<double>({28.09, 35.99, 72.18, 80.87})), \
  std::make_tuple(std::string("IRL"), std::vector<double>({51.42, 55.38, -10.58, -5.99})), \
  std::make_tuple(std::string("JP1"), std::vector<double>({32.66, 45.52, 137.27, 146.02})), \
  std::make_tuple(std::string("JP2"), std::vector<double>({32.79, 37.60, 131.41, 137.28})), \
  std::make_tuple(std::string("JP3"), std::vector<double>({24.04, 34.78, 122.93, 131.42})), \
  std::make_tuple(std::string("NZ"), std::vector<double>({-52.61, -29.24, 166, 178.84})), \
  std::make_tuple(std::string("SF"), std::vector<double>({-35.14, -22.13, 16.07, 33.21})), \
  std::make_tuple(std::string("UK"), std::vector<double>({49.9, 60.84, -8.62, 1.77}))
};

bool is_rhd_region(double latitude, double longitude) {
  for(const auto &t: _RHD_REGION_MAP)
    if (latitude >= std::get<1>(t)[0] and latitude <= std::get<1>(t)[1] and \
      longitude >= std::get<1>(t)[2] and longitude <= std::get<1>(t)[3])
      return true;
  return false;
}
