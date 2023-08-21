/*
 * Copyright 2019 Gianluca Frison, Dimitris Kouzoupis, Robin Verschueren,
 * Andrea Zanelli, Niels van Duijkeren, Jonathan Frey, Tommaso Sartor,
 * Branimir Novoselnik, Rien Quirynen, Rezart Qelibari, Dang Doan,
 * Jonas Koenemann, Yutao Chen, Tobias Schöls, Jonas Schlagenhauf, Moritz Diehl
 *
 * This file is part of acados.
 *
 * The 2-Clause BSD License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.;
 */


// standard
#include <stdio.h>
#include <stdlib.h>
// acados
#include "acados/utils/print.h"
#include "acados/utils/math.h"
#include "acados_c/ocp_nlp_interface.h"
#include "acados_c/external_function_interface.h"
#include "acados_solver_angular_ode_drone.h"

// blasfeo
#include "blasfeo/include/blasfeo_d_aux_ext_dep.h"

#define NX     ANGULAR_ODE_DRONE_NX
#define NZ     ANGULAR_ODE_DRONE_NZ
#define NU     ANGULAR_ODE_DRONE_NU
#define NP     ANGULAR_ODE_DRONE_NP
#define NBX    ANGULAR_ODE_DRONE_NBX
#define NBX0   ANGULAR_ODE_DRONE_NBX0
#define NBU    ANGULAR_ODE_DRONE_NBU
#define NSBX   ANGULAR_ODE_DRONE_NSBX
#define NSBU   ANGULAR_ODE_DRONE_NSBU
#define NSH    ANGULAR_ODE_DRONE_NSH
#define NSG    ANGULAR_ODE_DRONE_NSG
#define NSPHI  ANGULAR_ODE_DRONE_NSPHI
#define NSHN   ANGULAR_ODE_DRONE_NSHN
#define NSGN   ANGULAR_ODE_DRONE_NSGN
#define NSPHIN ANGULAR_ODE_DRONE_NSPHIN
#define NSBXN  ANGULAR_ODE_DRONE_NSBXN
#define NS     ANGULAR_ODE_DRONE_NS
#define NSN    ANGULAR_ODE_DRONE_NSN
#define NG     ANGULAR_ODE_DRONE_NG
#define NBXN   ANGULAR_ODE_DRONE_NBXN
#define NGN    ANGULAR_ODE_DRONE_NGN
#define NY0    ANGULAR_ODE_DRONE_NY0
#define NY     ANGULAR_ODE_DRONE_NY
#define NYN    ANGULAR_ODE_DRONE_NYN
#define NH     ANGULAR_ODE_DRONE_NH
#define NPHI   ANGULAR_ODE_DRONE_NPHI
#define NHN    ANGULAR_ODE_DRONE_NHN
#define NPHIN  ANGULAR_ODE_DRONE_NPHIN
#define NR     ANGULAR_ODE_DRONE_NR


int main()
{

    angular_ode_drone_solver_capsule *acados_ocp_capsule = angular_ode_drone_acados_create_capsule();
    // there is an opportunity to change the number of shooting intervals in C without new code generation
    int N = ANGULAR_ODE_DRONE_N;
    // allocate the array and fill it accordingly
    double* new_time_steps = NULL;
    int status = angular_ode_drone_acados_create_with_discretization(acados_ocp_capsule, N, new_time_steps);

    if (status)
    {
        printf("angular_ode_drone_acados_create() returned status %d. Exiting.\n", status);
        exit(1);
    }

    ocp_nlp_config *nlp_config = angular_ode_drone_acados_get_nlp_config(acados_ocp_capsule);
    ocp_nlp_dims *nlp_dims = angular_ode_drone_acados_get_nlp_dims(acados_ocp_capsule);
    ocp_nlp_in *nlp_in = angular_ode_drone_acados_get_nlp_in(acados_ocp_capsule);
    ocp_nlp_out *nlp_out = angular_ode_drone_acados_get_nlp_out(acados_ocp_capsule);
    ocp_nlp_solver *nlp_solver = angular_ode_drone_acados_get_nlp_solver(acados_ocp_capsule);
    void *nlp_opts = angular_ode_drone_acados_get_nlp_opts(acados_ocp_capsule);

    // initial condition
    int idxbx0[NBX0];
    idxbx0[0] = 0;
    idxbx0[1] = 1;
    idxbx0[2] = 2;
    idxbx0[3] = 3;
    idxbx0[4] = 4;
    idxbx0[5] = 5;
    idxbx0[6] = 6;
    idxbx0[7] = 7;
    idxbx0[8] = 8;
    idxbx0[9] = 9;
    idxbx0[10] = 10;
    idxbx0[11] = 11;
    idxbx0[12] = 12;
    idxbx0[13] = 13;
    idxbx0[14] = 14;
    idxbx0[15] = 15;
    idxbx0[16] = 16;
    idxbx0[17] = 17;
    idxbx0[18] = 18;
    idxbx0[19] = 19;
    idxbx0[20] = 20;
    idxbx0[21] = 21;
    idxbx0[22] = 22;
    idxbx0[23] = 23;
    idxbx0[24] = 24;
    idxbx0[25] = 25;
    idxbx0[26] = 26;
    idxbx0[27] = 27;
    idxbx0[28] = 28;
    idxbx0[29] = 29;
    idxbx0[30] = 30;
    idxbx0[31] = 31;
    idxbx0[32] = 32;

    double lbx0[NBX0];
    double ubx0[NBX0];
    lbx0[0] = 0.01671975356866851;
    ubx0[0] = 0.01671975356866851;
    lbx0[1] = 0.006881174966562625;
    ubx0[1] = 0.006881174966562625;
    lbx0[2] = 0.04919681213063508;
    ubx0[2] = 0.04919681213063508;
    lbx0[3] = 0.4951477515859966;
    ubx0[3] = 0.4951477515859966;
    lbx0[4] = 0.12685958906181688;
    ubx0[4] = 0.12685958906181688;
    lbx0[5] = 0.8699894559817574;
    ubx0[5] = 0.8699894559817574;
    lbx0[6] = 0.5208096221357555;
    ubx0[6] = 0.5208096221357555;
    lbx0[7] = 0.49631481008938494;
    ubx0[7] = 0.49631481008938494;
    lbx0[8] = 0.13046759629137042;
    ubx0[8] = 0.13046759629137042;
    lbx0[9] = 0.8604627369219243;
    ubx0[9] = 0.8604627369219243;
    lbx0[10] = 0.5066454187134531;
    ubx0[10] = 0.5066454187134531;
    lbx0[11] = 0.7111443391024831;
    ubx0[11] = 0.7111443391024831;
    lbx0[12] = 0.3059300389646824;
    ubx0[12] = 0.3059300389646824;
    lbx0[13] = 0.7008998047454811;
    ubx0[13] = 0.7008998047454811;
    lbx0[14] = 0.2626633592490298;
    ubx0[14] = 0.2626633592490298;
    lbx0[15] = -0.0058751316405896824;
    ubx0[15] = -0.0058751316405896824;
    lbx0[16] = 0.0012713720619063071;
    ubx0[16] = 0.0012713720619063071;
    lbx0[17] = 0.001312650505056322;
    ubx0[17] = 0.001312650505056322;
    lbx0[18] = -0.04426189708218288;
    ubx0[18] = -0.04426189708218288;
    lbx0[19] = -0.00218445739921455;
    ubx0[19] = -0.00218445739921455;
    lbx0[20] = -0.00007774446273687511;
    ubx0[20] = -0.00007774446273687511;
    lbx0[21] = 0.04079976746739129;
    ubx0[21] = 0.04079976746739129;
    lbx0[22] = 0.1393233002304765;
    ubx0[22] = 0.1393233002304765;
    lbx0[23] = 0.13750885568777424;
    ubx0[23] = 0.13750885568777424;
    lbx0[24] = 0.056270604261873566;
    ubx0[24] = 0.056270604261873566;
    lbx0[25] = 0.03314603506357502;
    ubx0[25] = 0.03314603506357502;
    lbx0[26] = 0.12936945657190763;
    ubx0[26] = 0.12936945657190763;
    lbx0[27] = 0.123762672297924;
    ubx0[27] = 0.123762672297924;
    lbx0[28] = 0.052304350960935685;
    ubx0[28] = 0.052304350960935685;
    lbx0[29] = 0.0327847317879912;
    ubx0[29] = 0.0327847317879912;
    lbx0[30] = 0.12881825948240552;
    ubx0[30] = 0.12881825948240552;
    lbx0[31] = 0.12304302137588012;
    ubx0[31] = 0.12304302137588012;
    lbx0[32] = 0.05208419216633014;
    ubx0[32] = 0.05208419216633014;

    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "idxbx", idxbx0);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "lbx", lbx0);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "ubx", ubx0);

    // initialization for state values
    double x_init[NX];
    x_init[0] = 0.0;
    x_init[1] = 0.0;
    x_init[2] = 0.0;
    x_init[3] = 0.0;
    x_init[4] = 0.0;
    x_init[5] = 0.0;
    x_init[6] = 0.0;
    x_init[7] = 0.0;
    x_init[8] = 0.0;
    x_init[9] = 0.0;
    x_init[10] = 0.0;
    x_init[11] = 0.0;
    x_init[12] = 0.0;
    x_init[13] = 0.0;
    x_init[14] = 0.0;
    x_init[15] = 0.0;
    x_init[16] = 0.0;
    x_init[17] = 0.0;
    x_init[18] = 0.0;
    x_init[19] = 0.0;
    x_init[20] = 0.0;
    x_init[21] = 0.0;
    x_init[22] = 0.0;
    x_init[23] = 0.0;
    x_init[24] = 0.0;
    x_init[25] = 0.0;
    x_init[26] = 0.0;
    x_init[27] = 0.0;
    x_init[28] = 0.0;
    x_init[29] = 0.0;
    x_init[30] = 0.0;
    x_init[31] = 0.0;
    x_init[32] = 0.0;

    // initial value for control input
    double u0[NU];
    u0[0] = 0.0;
    u0[1] = 0.0;
    u0[2] = 0.0;
    u0[3] = 0.0;

    // prepare evaluation
    int NTIMINGS = 1;
    double min_time = 1e12;
    double kkt_norm_inf;
    double elapsed_time;
    int sqp_iter;

    double xtraj[NX * (N+1)];
    double utraj[NU * N];


    // solve ocp in loop
    int rti_phase = 0;

    for (int ii = 0; ii < NTIMINGS; ii++)
    {
        // initialize solution
        for (int i = 0; i < N; i++)
        {
            ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "x", x_init);
            ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "u", u0);
        }
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, N, "x", x_init);
        ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "rti_phase", &rti_phase);
        status = angular_ode_drone_acados_solve(acados_ocp_capsule);
        ocp_nlp_get(nlp_config, nlp_solver, "time_tot", &elapsed_time);
        min_time = MIN(elapsed_time, min_time);
    }

    /* print solution and statistics */
    for (int ii = 0; ii <= nlp_dims->N; ii++)
        ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, ii, "x", &xtraj[ii*NX]);
    for (int ii = 0; ii < nlp_dims->N; ii++)
        ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, ii, "u", &utraj[ii*NU]);

    printf("\n--- xtraj ---\n");
    d_print_exp_tran_mat( NX, N+1, xtraj, NX);
    printf("\n--- utraj ---\n");
    d_print_exp_tran_mat( NU, N, utraj, NU );
    // ocp_nlp_out_print(nlp_solver->dims, nlp_out);

    printf("\nsolved ocp %d times, solution printed above\n\n", NTIMINGS);

    if (status == ACADOS_SUCCESS)
    {
        printf("angular_ode_drone_acados_solve(): SUCCESS!\n");
    }
    else
    {
        printf("angular_ode_drone_acados_solve() failed with status %d.\n", status);
    }

    // get solution
    ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, 0, "kkt_norm_inf", &kkt_norm_inf);
    ocp_nlp_get(nlp_config, nlp_solver, "sqp_iter", &sqp_iter);

    angular_ode_drone_acados_print_stats(acados_ocp_capsule);

    printf("\nSolver info:\n");
    printf(" SQP iterations %2d\n minimum time for %d solve %f [ms]\n KKT %e\n",
           sqp_iter, NTIMINGS, min_time*1000, kkt_norm_inf);

    // free solver
    status = angular_ode_drone_acados_free(acados_ocp_capsule);
    if (status) {
        printf("angular_ode_drone_acados_free() returned status %d. \n", status);
    }
    // free solver capsule
    status = angular_ode_drone_acados_free_capsule(acados_ocp_capsule);
    if (status) {
        printf("angular_ode_drone_acados_free_capsule() returned status %d. \n", status);
    }

    return status;
}
