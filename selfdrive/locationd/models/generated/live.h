/******************************************************************************
 *                      Code generated with sympy 1.6.1                       *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_1847404068503406573);
void inv_err_fun(double *nom_x, double *true_x, double *out_3055979383745148386);
void H_mod_fun(double *state, double *out_4008233588180934488);
void f_fun(double *state, double dt, double *out_3163902208628321100);
void F_fun(double *state, double dt, double *out_7013027676671194941);
void h_3(double *state, double *unused, double *out_7997793136628034172);
void H_3(double *state, double *unused, double *out_842917538478058895);
void h_4(double *state, double *unused, double *out_3714171096527543312);
void H_4(double *state, double *unused, double *out_672402472805165894);
void h_9(double *state, double *unused, double *out_8210254470489241260);
void H_9(double *state, double *unused, double *out_5352254331024636587);
void h_10(double *state, double *unused, double *out_620709179702102037);
void H_10(double *state, double *unused, double *out_6597818255808491129);
void h_12(double *state, double *unused, double *out_1301793299749987358);
void H_12(double *state, double *unused, double *out_5568204363156282988);
void h_31(double *state, double *unused, double *out_5066977733614745169);
void H_31(double *state, double *unused, double *out_3157420646774991982);
void h_32(double *state, double *unused, double *out_2801054764548007054);
void H_32(double *state, double *unused, double *out_5235302005972902195);
void h_13(double *state, double *unused, double *out_1817031717236844203);
void H_13(double *state, double *unused, double *out_209249914117808371);
void h_14(double *state, double *unused, double *out_8210254470489241260);
void H_14(double *state, double *unused, double *out_5352254331024636587);
void h_19(double *state, double *unused, double *out_1396788722174934361);
void H_19(double *state, double *unused, double *out_919574479922205406);
#define DIM 23
#define EDIM 22
#define MEDIM 22
typedef void (*Hfun)(double *, double *, double *);

void predict(double *x, double *P, double *Q, double dt);
const static double MAHA_THRESH_3 = 3.841459;
void update_3(double *, double *, double *, double *, double *);
const static double MAHA_THRESH_4 = 7.814728;
void update_4(double *, double *, double *, double *, double *);
const static double MAHA_THRESH_9 = 7.814728;
void update_9(double *, double *, double *, double *, double *);
const static double MAHA_THRESH_10 = 7.814728;
void update_10(double *, double *, double *, double *, double *);
const static double MAHA_THRESH_12 = 7.814728;
void update_12(double *, double *, double *, double *, double *);
const static double MAHA_THRESH_31 = 7.814728;
void update_31(double *, double *, double *, double *, double *);
const static double MAHA_THRESH_32 = 9.487729;
void update_32(double *, double *, double *, double *, double *);
const static double MAHA_THRESH_13 = 7.814728;
void update_13(double *, double *, double *, double *, double *);
const static double MAHA_THRESH_14 = 7.814728;
void update_14(double *, double *, double *, double *, double *);
const static double MAHA_THRESH_19 = 7.814728;
void update_19(double *, double *, double *, double *, double *);