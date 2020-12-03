
extern "C"{

double mass;

void set_mass(double x){ mass = x;}

double rotational_inertia;

void set_rotational_inertia(double x){ rotational_inertia = x;}

double center_to_front;

void set_center_to_front(double x){ center_to_front = x;}

double center_to_rear;

void set_center_to_rear(double x){ center_to_rear = x;}

double stiffness_front;

void set_stiffness_front(double x){ stiffness_front = x;}

double stiffness_rear;

void set_stiffness_rear(double x){ stiffness_rear = x;}

}
extern "C" {
#include <math.h>
/******************************************************************************
 *                      Code generated with sympy 1.6.1                       *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_8619663750913260227) {
   out_8619663750913260227[0] = delta_x[0] + nom_x[0];
   out_8619663750913260227[1] = delta_x[1] + nom_x[1];
   out_8619663750913260227[2] = delta_x[2] + nom_x[2];
   out_8619663750913260227[3] = delta_x[3] + nom_x[3];
   out_8619663750913260227[4] = delta_x[4] + nom_x[4];
   out_8619663750913260227[5] = delta_x[5] + nom_x[5];
   out_8619663750913260227[6] = delta_x[6] + nom_x[6];
   out_8619663750913260227[7] = delta_x[7] + nom_x[7];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_858903895054632697) {
   out_858903895054632697[0] = -nom_x[0] + true_x[0];
   out_858903895054632697[1] = -nom_x[1] + true_x[1];
   out_858903895054632697[2] = -nom_x[2] + true_x[2];
   out_858903895054632697[3] = -nom_x[3] + true_x[3];
   out_858903895054632697[4] = -nom_x[4] + true_x[4];
   out_858903895054632697[5] = -nom_x[5] + true_x[5];
   out_858903895054632697[6] = -nom_x[6] + true_x[6];
   out_858903895054632697[7] = -nom_x[7] + true_x[7];
}
void H_mod_fun(double *state, double *out_3085471901237330838) {
   out_3085471901237330838[0] = 1.0;
   out_3085471901237330838[1] = 0.0;
   out_3085471901237330838[2] = 0.0;
   out_3085471901237330838[3] = 0.0;
   out_3085471901237330838[4] = 0.0;
   out_3085471901237330838[5] = 0.0;
   out_3085471901237330838[6] = 0.0;
   out_3085471901237330838[7] = 0.0;
   out_3085471901237330838[8] = 0.0;
   out_3085471901237330838[9] = 1.0;
   out_3085471901237330838[10] = 0.0;
   out_3085471901237330838[11] = 0.0;
   out_3085471901237330838[12] = 0.0;
   out_3085471901237330838[13] = 0.0;
   out_3085471901237330838[14] = 0.0;
   out_3085471901237330838[15] = 0.0;
   out_3085471901237330838[16] = 0.0;
   out_3085471901237330838[17] = 0.0;
   out_3085471901237330838[18] = 1.0;
   out_3085471901237330838[19] = 0.0;
   out_3085471901237330838[20] = 0.0;
   out_3085471901237330838[21] = 0.0;
   out_3085471901237330838[22] = 0.0;
   out_3085471901237330838[23] = 0.0;
   out_3085471901237330838[24] = 0.0;
   out_3085471901237330838[25] = 0.0;
   out_3085471901237330838[26] = 0.0;
   out_3085471901237330838[27] = 1.0;
   out_3085471901237330838[28] = 0.0;
   out_3085471901237330838[29] = 0.0;
   out_3085471901237330838[30] = 0.0;
   out_3085471901237330838[31] = 0.0;
   out_3085471901237330838[32] = 0.0;
   out_3085471901237330838[33] = 0.0;
   out_3085471901237330838[34] = 0.0;
   out_3085471901237330838[35] = 0.0;
   out_3085471901237330838[36] = 1.0;
   out_3085471901237330838[37] = 0.0;
   out_3085471901237330838[38] = 0.0;
   out_3085471901237330838[39] = 0.0;
   out_3085471901237330838[40] = 0.0;
   out_3085471901237330838[41] = 0.0;
   out_3085471901237330838[42] = 0.0;
   out_3085471901237330838[43] = 0.0;
   out_3085471901237330838[44] = 0.0;
   out_3085471901237330838[45] = 1.0;
   out_3085471901237330838[46] = 0.0;
   out_3085471901237330838[47] = 0.0;
   out_3085471901237330838[48] = 0.0;
   out_3085471901237330838[49] = 0.0;
   out_3085471901237330838[50] = 0.0;
   out_3085471901237330838[51] = 0.0;
   out_3085471901237330838[52] = 0.0;
   out_3085471901237330838[53] = 0.0;
   out_3085471901237330838[54] = 1.0;
   out_3085471901237330838[55] = 0.0;
   out_3085471901237330838[56] = 0.0;
   out_3085471901237330838[57] = 0.0;
   out_3085471901237330838[58] = 0.0;
   out_3085471901237330838[59] = 0.0;
   out_3085471901237330838[60] = 0.0;
   out_3085471901237330838[61] = 0.0;
   out_3085471901237330838[62] = 0.0;
   out_3085471901237330838[63] = 1.0;
}
void f_fun(double *state, double dt, double *out_6583399880768770353) {
   out_6583399880768770353[0] = state[0];
   out_6583399880768770353[1] = state[1];
   out_6583399880768770353[2] = state[2];
   out_6583399880768770353[3] = state[3];
   out_6583399880768770353[4] = state[4];
   out_6583399880768770353[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_6583399880768770353[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_6583399880768770353[7] = state[7];
}
void F_fun(double *state, double dt, double *out_2355999061078345037) {
   out_2355999061078345037[0] = 1;
   out_2355999061078345037[1] = 0;
   out_2355999061078345037[2] = 0;
   out_2355999061078345037[3] = 0;
   out_2355999061078345037[4] = 0;
   out_2355999061078345037[5] = 0;
   out_2355999061078345037[6] = 0;
   out_2355999061078345037[7] = 0;
   out_2355999061078345037[8] = 0;
   out_2355999061078345037[9] = 1;
   out_2355999061078345037[10] = 0;
   out_2355999061078345037[11] = 0;
   out_2355999061078345037[12] = 0;
   out_2355999061078345037[13] = 0;
   out_2355999061078345037[14] = 0;
   out_2355999061078345037[15] = 0;
   out_2355999061078345037[16] = 0;
   out_2355999061078345037[17] = 0;
   out_2355999061078345037[18] = 1;
   out_2355999061078345037[19] = 0;
   out_2355999061078345037[20] = 0;
   out_2355999061078345037[21] = 0;
   out_2355999061078345037[22] = 0;
   out_2355999061078345037[23] = 0;
   out_2355999061078345037[24] = 0;
   out_2355999061078345037[25] = 0;
   out_2355999061078345037[26] = 0;
   out_2355999061078345037[27] = 1;
   out_2355999061078345037[28] = 0;
   out_2355999061078345037[29] = 0;
   out_2355999061078345037[30] = 0;
   out_2355999061078345037[31] = 0;
   out_2355999061078345037[32] = 0;
   out_2355999061078345037[33] = 0;
   out_2355999061078345037[34] = 0;
   out_2355999061078345037[35] = 0;
   out_2355999061078345037[36] = 1;
   out_2355999061078345037[37] = 0;
   out_2355999061078345037[38] = 0;
   out_2355999061078345037[39] = 0;
   out_2355999061078345037[40] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_2355999061078345037[41] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_2355999061078345037[42] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_2355999061078345037[43] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_2355999061078345037[44] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_2355999061078345037[45] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_2355999061078345037[46] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_2355999061078345037[47] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_2355999061078345037[48] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_2355999061078345037[49] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_2355999061078345037[50] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_2355999061078345037[51] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_2355999061078345037[52] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_2355999061078345037[53] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_2355999061078345037[54] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_2355999061078345037[55] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_2355999061078345037[56] = 0;
   out_2355999061078345037[57] = 0;
   out_2355999061078345037[58] = 0;
   out_2355999061078345037[59] = 0;
   out_2355999061078345037[60] = 0;
   out_2355999061078345037[61] = 0;
   out_2355999061078345037[62] = 0;
   out_2355999061078345037[63] = 1;
}
void h_25(double *state, double *unused, double *out_1789494232869446028) {
   out_1789494232869446028[0] = state[6];
}
void H_25(double *state, double *unused, double *out_2004749318170240106) {
   out_2004749318170240106[0] = 0;
   out_2004749318170240106[1] = 0;
   out_2004749318170240106[2] = 0;
   out_2004749318170240106[3] = 0;
   out_2004749318170240106[4] = 0;
   out_2004749318170240106[5] = 0;
   out_2004749318170240106[6] = 1;
   out_2004749318170240106[7] = 0;
}
void h_24(double *state, double *unused, double *out_1939302670909251642) {
   out_1939302670909251642[0] = state[4];
   out_1939302670909251642[1] = state[5];
}
void H_24(double *state, double *unused, double *out_1934920567542525664) {
   out_1934920567542525664[0] = 0;
   out_1934920567542525664[1] = 0;
   out_1934920567542525664[2] = 0;
   out_1934920567542525664[3] = 0;
   out_1934920567542525664[4] = 1;
   out_1934920567542525664[5] = 0;
   out_1934920567542525664[6] = 0;
   out_1934920567542525664[7] = 0;
   out_1934920567542525664[8] = 0;
   out_1934920567542525664[9] = 0;
   out_1934920567542525664[10] = 0;
   out_1934920567542525664[11] = 0;
   out_1934920567542525664[12] = 0;
   out_1934920567542525664[13] = 1;
   out_1934920567542525664[14] = 0;
   out_1934920567542525664[15] = 0;
}
void h_30(double *state, double *unused, double *out_611763722893468362) {
   out_611763722893468362[0] = state[4];
}
void H_30(double *state, double *unused, double *out_7609149592778943174) {
   out_7609149592778943174[0] = 0;
   out_7609149592778943174[1] = 0;
   out_7609149592778943174[2] = 0;
   out_7609149592778943174[3] = 0;
   out_7609149592778943174[4] = 1;
   out_7609149592778943174[5] = 0;
   out_7609149592778943174[6] = 0;
   out_7609149592778943174[7] = 0;
}
void h_26(double *state, double *unused, double *out_6096847754234300817) {
   out_6096847754234300817[0] = state[7];
}
void H_26(double *state, double *unused, double *out_7709342473109449379) {
   out_7709342473109449379[0] = 0;
   out_7709342473109449379[1] = 0;
   out_7709342473109449379[2] = 0;
   out_7709342473109449379[3] = 0;
   out_7709342473109449379[4] = 0;
   out_7709342473109449379[5] = 0;
   out_7709342473109449379[6] = 0;
   out_7709342473109449379[7] = 1;
}
void h_27(double *state, double *unused, double *out_6214855148167677561) {
   out_6214855148167677561[0] = state[3];
}
void H_27(double *state, double *unused, double *out_8896731580615568486) {
   out_8896731580615568486[0] = 0;
   out_8896731580615568486[1] = 0;
   out_8896731580615568486[2] = 0;
   out_8896731580615568486[3] = 1;
   out_8896731580615568486[4] = 0;
   out_8896731580615568486[5] = 0;
   out_8896731580615568486[6] = 0;
   out_8896731580615568486[7] = 0;
}
void h_29(double *state, double *unused, double *out_8291732904716067589) {
   out_8291732904716067589[0] = state[1];
}
void H_29(double *state, double *unused, double *out_7034590978923406999) {
   out_7034590978923406999[0] = 0;
   out_7034590978923406999[1] = 1;
   out_7034590978923406999[2] = 0;
   out_7034590978923406999[3] = 0;
   out_7034590978923406999[4] = 0;
   out_7034590978923406999[5] = 0;
   out_7034590978923406999[6] = 0;
   out_7034590978923406999[7] = 0;
}
void h_28(double *state, double *unused, double *out_5688467549079859006) {
   out_5688467549079859006[0] = state[5];
   out_5688467549079859006[1] = state[6];
}
void H_28(double *state, double *unused, double *out_8952753045779020530) {
   out_8952753045779020530[0] = 0;
   out_8952753045779020530[1] = 0;
   out_8952753045779020530[2] = 0;
   out_8952753045779020530[3] = 0;
   out_8952753045779020530[4] = 0;
   out_8952753045779020530[5] = 1;
   out_8952753045779020530[6] = 0;
   out_8952753045779020530[7] = 0;
   out_8952753045779020530[8] = 0;
   out_8952753045779020530[9] = 0;
   out_8952753045779020530[10] = 0;
   out_8952753045779020530[11] = 0;
   out_8952753045779020530[12] = 0;
   out_8952753045779020530[13] = 0;
   out_8952753045779020530[14] = 1;
   out_8952753045779020530[15] = 0;
}
}

extern "C"{
#define DIM 8
#define EDIM 8
#define MEDIM 8
typedef void (*Hfun)(double *, double *, double *);

void predict(double *x, double *P, double *Q, double dt);
const static double MAHA_THRESH_25 = 3.841459;
void update_25(double *, double *, double *, double *, double *);
const static double MAHA_THRESH_24 = 5.991465;
void update_24(double *, double *, double *, double *, double *);
const static double MAHA_THRESH_30 = 3.841459;
void update_30(double *, double *, double *, double *, double *);
const static double MAHA_THRESH_26 = 3.841459;
void update_26(double *, double *, double *, double *, double *);
const static double MAHA_THRESH_27 = 3.841459;
void update_27(double *, double *, double *, double *, double *);
const static double MAHA_THRESH_29 = 3.841459;
void update_29(double *, double *, double *, double *, double *);
const static double MAHA_THRESH_28 = 5.991465;
void update_28(double *, double *, double *, double *, double *);
}

#include <eigen3/Eigen/Dense>
#include <iostream>

typedef Eigen::Matrix<double, DIM, DIM, Eigen::RowMajor> DDM;
typedef Eigen::Matrix<double, EDIM, EDIM, Eigen::RowMajor> EEM;
typedef Eigen::Matrix<double, DIM, EDIM, Eigen::RowMajor> DEM;

void predict(double *in_x, double *in_P, double *in_Q, double dt) {
  typedef Eigen::Matrix<double, MEDIM, MEDIM, Eigen::RowMajor> RRM;

  double nx[DIM] = {0};
  double in_F[EDIM*EDIM] = {0};

  // functions from sympy
  f_fun(in_x, dt, nx);
  F_fun(in_x, dt, in_F);


  EEM F(in_F);
  EEM P(in_P);
  EEM Q(in_Q);

  RRM F_main = F.topLeftCorner(MEDIM, MEDIM);
  P.topLeftCorner(MEDIM, MEDIM) = (F_main * P.topLeftCorner(MEDIM, MEDIM)) * F_main.transpose();
  P.topRightCorner(MEDIM, EDIM - MEDIM) = F_main * P.topRightCorner(MEDIM, EDIM - MEDIM);
  P.bottomLeftCorner(EDIM - MEDIM, MEDIM) = P.bottomLeftCorner(EDIM - MEDIM, MEDIM) * F_main.transpose();

  P = P + dt*Q;

  // copy out state
  memcpy(in_x, nx, DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
}

// note: extra_args dim only correct when null space projecting
// otherwise 1
template <int ZDIM, int EADIM, bool MAHA_TEST>
void update(double *in_x, double *in_P, Hfun h_fun, Hfun H_fun, Hfun Hea_fun, double *in_z, double *in_R, double *in_ea, double MAHA_THRESHOLD) {
  typedef Eigen::Matrix<double, ZDIM, ZDIM, Eigen::RowMajor> ZZM;
  typedef Eigen::Matrix<double, ZDIM, DIM, Eigen::RowMajor> ZDM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, EDIM, Eigen::RowMajor> XEM;
  //typedef Eigen::Matrix<double, EDIM, ZDIM, Eigen::RowMajor> EZM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, 1> X1M;
  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> XXM;

  double in_hx[ZDIM] = {0};
  double in_H[ZDIM * DIM] = {0};
  double in_H_mod[EDIM * DIM] = {0};
  double delta_x[EDIM] = {0};
  double x_new[DIM] = {0};


  // state x, P
  Eigen::Matrix<double, ZDIM, 1> z(in_z);
  EEM P(in_P);
  ZZM pre_R(in_R);

  // functions from sympy
  h_fun(in_x, in_ea, in_hx);
  H_fun(in_x, in_ea, in_H);
  ZDM pre_H(in_H);

  // get y (y = z - hx)
  Eigen::Matrix<double, ZDIM, 1> pre_y(in_hx); pre_y = z - pre_y;
  X1M y; XXM H; XXM R;
  if (Hea_fun){
    typedef Eigen::Matrix<double, ZDIM, EADIM, Eigen::RowMajor> ZAM;
    double in_Hea[ZDIM * EADIM] = {0};
    Hea_fun(in_x, in_ea, in_Hea);
    ZAM Hea(in_Hea);
    XXM A = Hea.transpose().fullPivLu().kernel();


    y = A.transpose() * pre_y;
    H = A.transpose() * pre_H;
    R = A.transpose() * pre_R * A;
  } else {
    y = pre_y;
    H = pre_H;
    R = pre_R;
  }
  // get modified H
  H_mod_fun(in_x, in_H_mod);
  DEM H_mod(in_H_mod);
  XEM H_err = H * H_mod;

  // Do mahalobis distance test
  if (MAHA_TEST){
    XXM a = (H_err * P * H_err.transpose() + R).inverse();
    double maha_dist = y.transpose() * a * y;
    if (maha_dist > MAHA_THRESHOLD){
      R = 1.0e16 * R;
    }
  }

  // Outlier resilient weighting
  double weight = 1;//(1.5)/(1 + y.squaredNorm()/R.sum());

  // kalman gains and I_KH
  XXM S = ((H_err * P) * H_err.transpose()) + R/weight;
  XEM KT = S.fullPivLu().solve(H_err * P.transpose());
  //EZM K = KT.transpose(); TODO: WHY DOES THIS NOT COMPILE?
  //EZM K = S.fullPivLu().solve(H_err * P.transpose()).transpose();
  //std::cout << "Here is the matrix rot:\n" << K << std::endl;
  EEM I_KH = Eigen::Matrix<double, EDIM, EDIM>::Identity() - (KT.transpose() * H_err);

  // update state by injecting dx
  Eigen::Matrix<double, EDIM, 1> dx(delta_x);
  dx  = (KT.transpose() * y);
  memcpy(delta_x, dx.data(), EDIM * sizeof(double));
  err_fun(in_x, delta_x, x_new);
  Eigen::Matrix<double, DIM, 1> x(x_new);

  // update cov
  P = ((I_KH * P) * I_KH.transpose()) + ((KT.transpose() * R) * KT);

  // copy out state
  memcpy(in_x, x.data(), DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
  memcpy(in_z, y.data(), y.rows() * sizeof(double));
}



extern "C"{

      void update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
        update<1,3,0>(in_x, in_P, h_25, H_25, NULL, in_z, in_R, in_ea, MAHA_THRESH_25);
      }
    
      void update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
        update<2,3,0>(in_x, in_P, h_24, H_24, NULL, in_z, in_R, in_ea, MAHA_THRESH_24);
      }
    
      void update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
        update<1,3,0>(in_x, in_P, h_30, H_30, NULL, in_z, in_R, in_ea, MAHA_THRESH_30);
      }
    
      void update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
        update<1,3,0>(in_x, in_P, h_26, H_26, NULL, in_z, in_R, in_ea, MAHA_THRESH_26);
      }
    
      void update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
        update<1,3,0>(in_x, in_P, h_27, H_27, NULL, in_z, in_R, in_ea, MAHA_THRESH_27);
      }
    
      void update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
        update<1,3,0>(in_x, in_P, h_29, H_29, NULL, in_z, in_R, in_ea, MAHA_THRESH_29);
      }
    
      void update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
        update<2,3,0>(in_x, in_P, h_28, H_28, NULL, in_z, in_R, in_ea, MAHA_THRESH_28);
      }
    
}
