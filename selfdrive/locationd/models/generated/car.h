/******************************************************************************
 *                      Code generated with sympy 1.6.1                       *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_8619663750913260227);
void inv_err_fun(double *nom_x, double *true_x, double *out_858903895054632697);
void H_mod_fun(double *state, double *out_3085471901237330838);
void f_fun(double *state, double dt, double *out_6583399880768770353);
void F_fun(double *state, double dt, double *out_2355999061078345037);
void h_25(double *state, double *unused, double *out_1789494232869446028);
void H_25(double *state, double *unused, double *out_2004749318170240106);
void h_24(double *state, double *unused, double *out_1939302670909251642);
void H_24(double *state, double *unused, double *out_1934920567542525664);
void h_30(double *state, double *unused, double *out_611763722893468362);
void H_30(double *state, double *unused, double *out_7609149592778943174);
void h_26(double *state, double *unused, double *out_6096847754234300817);
void H_26(double *state, double *unused, double *out_7709342473109449379);
void h_27(double *state, double *unused, double *out_6214855148167677561);
void H_27(double *state, double *unused, double *out_8896731580615568486);
void h_29(double *state, double *unused, double *out_8291732904716067589);
void H_29(double *state, double *unused, double *out_7034590978923406999);
void h_28(double *state, double *unused, double *out_5688467549079859006);
void H_28(double *state, double *unused, double *out_8952753045779020530);
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
void set_mass(double x);

void set_rotational_inertia(double x);

void set_center_to_front(double x);

void set_center_to_rear(double x);

void set_stiffness_front(double x);

void set_stiffness_rear(double x);
