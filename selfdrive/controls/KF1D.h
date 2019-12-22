#ifndef KF1D_H
#define KF1D_H

class KF1D {
  public:
    void init(double x0_0, double x1_0, double A0_0, double A0_1, double A1_0, double A1_1,
              double C0, double C1, double K0_0, double K1_0);
    void update(double meas);

    double x0_0;
    double x1_0;
    double K0_0;
    double K1_0;
    double A0_0;
    double A0_1;
    double A1_0;
    double A1_1;
    double C0_0;
    double C0_1;
    double A_K_0;
    double A_K_1;
    double A_K_2;
    double A_K_3;
};

#endif
