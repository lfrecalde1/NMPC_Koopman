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

#ifndef ACADOS_SOLVER_angular_ode_drone_H_
#define ACADOS_SOLVER_angular_ode_drone_H_

#include "acados/utils/types.h"

#include "acados_c/ocp_nlp_interface.h"
#include "acados_c/external_function_interface.h"

#define ANGULAR_ODE_DRONE_NX     33
#define ANGULAR_ODE_DRONE_NZ     0
#define ANGULAR_ODE_DRONE_NU     4
#define ANGULAR_ODE_DRONE_NP     0
#define ANGULAR_ODE_DRONE_NBX    0
#define ANGULAR_ODE_DRONE_NBX0   33
#define ANGULAR_ODE_DRONE_NBU    4
#define ANGULAR_ODE_DRONE_NSBX   0
#define ANGULAR_ODE_DRONE_NSBU   0
#define ANGULAR_ODE_DRONE_NSH    0
#define ANGULAR_ODE_DRONE_NSG    0
#define ANGULAR_ODE_DRONE_NSPHI  0
#define ANGULAR_ODE_DRONE_NSHN   0
#define ANGULAR_ODE_DRONE_NSGN   0
#define ANGULAR_ODE_DRONE_NSPHIN 0
#define ANGULAR_ODE_DRONE_NSBXN  0
#define ANGULAR_ODE_DRONE_NS     0
#define ANGULAR_ODE_DRONE_NSN    0
#define ANGULAR_ODE_DRONE_NG     0
#define ANGULAR_ODE_DRONE_NBXN   0
#define ANGULAR_ODE_DRONE_NGN    0
#define ANGULAR_ODE_DRONE_NY0    37
#define ANGULAR_ODE_DRONE_NY     37
#define ANGULAR_ODE_DRONE_NYN    33
#define ANGULAR_ODE_DRONE_N      76
#define ANGULAR_ODE_DRONE_NH     0
#define ANGULAR_ODE_DRONE_NPHI   0
#define ANGULAR_ODE_DRONE_NHN    0
#define ANGULAR_ODE_DRONE_NPHIN  0
#define ANGULAR_ODE_DRONE_NR     0

#ifdef __cplusplus
extern "C" {
#endif


// ** capsule for solver data **
typedef struct angular_ode_drone_solver_capsule
{
    // acados objects
    ocp_nlp_in *nlp_in;
    ocp_nlp_out *nlp_out;
    ocp_nlp_out *sens_out;
    ocp_nlp_solver *nlp_solver;
    void *nlp_opts;
    ocp_nlp_plan_t *nlp_solver_plan;
    ocp_nlp_config *nlp_config;
    ocp_nlp_dims *nlp_dims;

    // number of expected runtime parameters
    unsigned int nlp_np;

    /* external functions */
    // dynamics

    external_function_param_casadi *discr_dyn_phi_fun;
    external_function_param_casadi *discr_dyn_phi_fun_jac_ut_xt;
    external_function_param_casadi *discr_dyn_phi_fun_jac_ut_xt_hess;


    // cost






    // constraints




} angular_ode_drone_solver_capsule;

ACADOS_SYMBOL_EXPORT angular_ode_drone_solver_capsule * angular_ode_drone_acados_create_capsule(void);
ACADOS_SYMBOL_EXPORT int angular_ode_drone_acados_free_capsule(angular_ode_drone_solver_capsule *capsule);

ACADOS_SYMBOL_EXPORT int angular_ode_drone_acados_create(angular_ode_drone_solver_capsule * capsule);

ACADOS_SYMBOL_EXPORT int angular_ode_drone_acados_reset(angular_ode_drone_solver_capsule* capsule, int reset_qp_solver_mem);

/**
 * Generic version of angular_ode_drone_acados_create which allows to use a different number of shooting intervals than
 * the number used for code generation. If new_time_steps=NULL and n_time_steps matches the number used for code
 * generation, the time-steps from code generation is used.
 */
ACADOS_SYMBOL_EXPORT int angular_ode_drone_acados_create_with_discretization(angular_ode_drone_solver_capsule * capsule, int n_time_steps, double* new_time_steps);
/**
 * Update the time step vector. Number N must be identical to the currently set number of shooting nodes in the
 * nlp_solver_plan. Returns 0 if no error occurred and a otherwise a value other than 0.
 */
ACADOS_SYMBOL_EXPORT int angular_ode_drone_acados_update_time_steps(angular_ode_drone_solver_capsule * capsule, int N, double* new_time_steps);
/**
 * This function is used for updating an already initialized solver with a different number of qp_cond_N.
 */
ACADOS_SYMBOL_EXPORT int angular_ode_drone_acados_update_qp_solver_cond_N(angular_ode_drone_solver_capsule * capsule, int qp_solver_cond_N);
ACADOS_SYMBOL_EXPORT int angular_ode_drone_acados_update_params(angular_ode_drone_solver_capsule * capsule, int stage, double *value, int np);
ACADOS_SYMBOL_EXPORT int angular_ode_drone_acados_update_params_sparse(angular_ode_drone_solver_capsule * capsule, int stage, int *idx, double *p, int n_update);

ACADOS_SYMBOL_EXPORT int angular_ode_drone_acados_solve(angular_ode_drone_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT int angular_ode_drone_acados_free(angular_ode_drone_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT void angular_ode_drone_acados_print_stats(angular_ode_drone_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT int angular_ode_drone_acados_custom_update(angular_ode_drone_solver_capsule* capsule, double* data, int data_len);


ACADOS_SYMBOL_EXPORT ocp_nlp_in *angular_ode_drone_acados_get_nlp_in(angular_ode_drone_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT ocp_nlp_out *angular_ode_drone_acados_get_nlp_out(angular_ode_drone_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT ocp_nlp_out *angular_ode_drone_acados_get_sens_out(angular_ode_drone_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT ocp_nlp_solver *angular_ode_drone_acados_get_nlp_solver(angular_ode_drone_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT ocp_nlp_config *angular_ode_drone_acados_get_nlp_config(angular_ode_drone_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT void *angular_ode_drone_acados_get_nlp_opts(angular_ode_drone_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT ocp_nlp_dims *angular_ode_drone_acados_get_nlp_dims(angular_ode_drone_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT ocp_nlp_plan_t *angular_ode_drone_acados_get_nlp_plan(angular_ode_drone_solver_capsule * capsule);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif  // ACADOS_SOLVER_angular_ode_drone_H_
