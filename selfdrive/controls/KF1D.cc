#include "KF1D.h"

void KF1D::init(double x0_0, double x1_0, double A0_0, double A0_1, double A1_0, double A1_1,
          double C0, double C1, double K0_0, double K1_0) {
  this->x0_0 = x0_0;
  this->x1_0 = x1_0;
  this->A0_0 = A0_0;
  this->A0_1 = A0_1;
  this->A1_0 = A1_0;
  this->A1_1 = A1_1;
  this->C0_0 = C0;
  this->C0_1 = C1;
  this->K0_0 = K0_0;
  this->K1_0 = K1_0;

  this->A_K_0 = this->A0_0 - this->K0_0 * this->C0_0;
  this->A_K_1 = this->A0_1 - this->K0_0 * this->C0_1;
  this->A_K_2 = this->A1_0 - this->K1_0 * this->C0_0;
  this->A_K_3 = this->A1_1 - this->K1_0 * this->C0_1;
}

void KF1D::update(double meas) {
  double x0_0 = this->A_K_0 * this->x0_0 + this->A_K_1 * this->x1_0 + this->K0_0 * meas;
  double x1_0 = this->A_K_2 * this->x0_0 + this->A_K_3 * this->x1_0 + this->K1_0 * meas;
  this->x0_0 = x0_0;
  this->x1_0 = x1_0;
}
