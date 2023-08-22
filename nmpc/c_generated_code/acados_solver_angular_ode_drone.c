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
#include <assert.h>
// acados
// #include "acados/utils/print.h"
#include "acados_c/ocp_nlp_interface.h"
#include "acados_c/external_function_interface.h"

// example specific
#include "angular_ode_drone_model/angular_ode_drone_model.h"
#include "angular_ode_drone_constraints/angular_ode_drone_constraints.h"




#include "acados_solver_angular_ode_drone.h"

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
// #define N      ANGULAR_ODE_DRONE_N
#define NH     ANGULAR_ODE_DRONE_NH
#define NPHI   ANGULAR_ODE_DRONE_NPHI
#define NHN    ANGULAR_ODE_DRONE_NHN
#define NPHIN  ANGULAR_ODE_DRONE_NPHIN
#define NR     ANGULAR_ODE_DRONE_NR


// ** solver data **

angular_ode_drone_solver_capsule * angular_ode_drone_acados_create_capsule(void)
{
    void* capsule_mem = malloc(sizeof(angular_ode_drone_solver_capsule));
    angular_ode_drone_solver_capsule *capsule = (angular_ode_drone_solver_capsule *) capsule_mem;

    return capsule;
}


int angular_ode_drone_acados_free_capsule(angular_ode_drone_solver_capsule *capsule)
{
    free(capsule);
    return 0;
}


int angular_ode_drone_acados_create(angular_ode_drone_solver_capsule* capsule)
{
    int N_shooting_intervals = ANGULAR_ODE_DRONE_N;
    double* new_time_steps = NULL; // NULL -> don't alter the code generated time-steps
    return angular_ode_drone_acados_create_with_discretization(capsule, N_shooting_intervals, new_time_steps);
}


int angular_ode_drone_acados_update_time_steps(angular_ode_drone_solver_capsule* capsule, int N, double* new_time_steps)
{
    if (N != capsule->nlp_solver_plan->N) {
        fprintf(stderr, "angular_ode_drone_acados_update_time_steps: given number of time steps (= %d) " \
            "differs from the currently allocated number of " \
            "time steps (= %d)!\n" \
            "Please recreate with new discretization and provide a new vector of time_stamps!\n",
            N, capsule->nlp_solver_plan->N);
        return 1;
    }

    ocp_nlp_config * nlp_config = capsule->nlp_config;
    ocp_nlp_dims * nlp_dims = capsule->nlp_dims;
    ocp_nlp_in * nlp_in = capsule->nlp_in;

    for (int i = 0; i < N; i++)
    {
        ocp_nlp_in_set(nlp_config, nlp_dims, nlp_in, i, "Ts", &new_time_steps[i]);
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "scaling", &new_time_steps[i]);
    }
    return 0;
}

/**
 * Internal function for angular_ode_drone_acados_create: step 1
 */
void angular_ode_drone_acados_create_1_set_plan(ocp_nlp_plan_t* nlp_solver_plan, const int N)
{
    assert(N == nlp_solver_plan->N);

    /************************************************
    *  plan
    ************************************************/
    nlp_solver_plan->nlp_solver = SQP_RTI;

    nlp_solver_plan->ocp_qp_solver_plan.qp_solver = FULL_CONDENSING_HPIPM;

    nlp_solver_plan->nlp_cost[0] = LINEAR_LS;
    for (int i = 1; i < N; i++)
        nlp_solver_plan->nlp_cost[i] = LINEAR_LS;

    nlp_solver_plan->nlp_cost[N] = LINEAR_LS;

    for (int i = 0; i < N; i++)
    {
        nlp_solver_plan->nlp_dynamics[i] = DISCRETE_MODEL;
        // discrete dynamics does not need sim solver option, this field is ignored
        nlp_solver_plan->sim_solver_plan[i].sim_solver = INVALID_SIM_SOLVER;
    }

    for (int i = 0; i < N; i++)
    {nlp_solver_plan->nlp_constraints[i] = BGH;
    }
    nlp_solver_plan->nlp_constraints[N] = BGH;
}


/**
 * Internal function for angular_ode_drone_acados_create: step 2
 */
ocp_nlp_dims* angular_ode_drone_acados_create_2_create_and_set_dimensions(angular_ode_drone_solver_capsule* capsule)
{
    ocp_nlp_plan_t* nlp_solver_plan = capsule->nlp_solver_plan;
    const int N = nlp_solver_plan->N;
    ocp_nlp_config* nlp_config = capsule->nlp_config;

    /************************************************
    *  dimensions
    ************************************************/
    #define NINTNP1MEMS 17
    int* intNp1mem = (int*)malloc( (N+1)*sizeof(int)*NINTNP1MEMS );

    int* nx    = intNp1mem + (N+1)*0;
    int* nu    = intNp1mem + (N+1)*1;
    int* nbx   = intNp1mem + (N+1)*2;
    int* nbu   = intNp1mem + (N+1)*3;
    int* nsbx  = intNp1mem + (N+1)*4;
    int* nsbu  = intNp1mem + (N+1)*5;
    int* nsg   = intNp1mem + (N+1)*6;
    int* nsh   = intNp1mem + (N+1)*7;
    int* nsphi = intNp1mem + (N+1)*8;
    int* ns    = intNp1mem + (N+1)*9;
    int* ng    = intNp1mem + (N+1)*10;
    int* nh    = intNp1mem + (N+1)*11;
    int* nphi  = intNp1mem + (N+1)*12;
    int* nz    = intNp1mem + (N+1)*13;
    int* ny    = intNp1mem + (N+1)*14;
    int* nr    = intNp1mem + (N+1)*15;
    int* nbxe  = intNp1mem + (N+1)*16;

    for (int i = 0; i < N+1; i++)
    {
        // common
        nx[i]     = NX;
        nu[i]     = NU;
        nz[i]     = NZ;
        ns[i]     = NS;
        // cost
        ny[i]     = NY;
        // constraints
        nbx[i]    = NBX;
        nbu[i]    = NBU;
        nsbx[i]   = NSBX;
        nsbu[i]   = NSBU;
        nsg[i]    = NSG;
        nsh[i]    = NSH;
        nsphi[i]  = NSPHI;
        ng[i]     = NG;
        nh[i]     = NH;
        nphi[i]   = NPHI;
        nr[i]     = NR;
        nbxe[i]   = 0;
    }

    // for initial state
    nbx[0]  = NBX0;
    nsbx[0] = 0;
    ns[0] = NS - NSBX;
    nbxe[0] = 33;
    ny[0] = NY0;

    // terminal - common
    nu[N]   = 0;
    nz[N]   = 0;
    ns[N]   = NSN;
    // cost
    ny[N]   = NYN;
    // constraint
    nbx[N]   = NBXN;
    nbu[N]   = 0;
    ng[N]    = NGN;
    nh[N]    = NHN;
    nphi[N]  = NPHIN;
    nr[N]    = 0;

    nsbx[N]  = NSBXN;
    nsbu[N]  = 0;
    nsg[N]   = NSGN;
    nsh[N]   = NSHN;
    nsphi[N] = NSPHIN;

    /* create and set ocp_nlp_dims */
    ocp_nlp_dims * nlp_dims = ocp_nlp_dims_create(nlp_config);

    ocp_nlp_dims_set_opt_vars(nlp_config, nlp_dims, "nx", nx);
    ocp_nlp_dims_set_opt_vars(nlp_config, nlp_dims, "nu", nu);
    ocp_nlp_dims_set_opt_vars(nlp_config, nlp_dims, "nz", nz);
    ocp_nlp_dims_set_opt_vars(nlp_config, nlp_dims, "ns", ns);

    for (int i = 0; i <= N; i++)
    {
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nbx", &nbx[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nbu", &nbu[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nsbx", &nsbx[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nsbu", &nsbu[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "ng", &ng[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nsg", &nsg[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nbxe", &nbxe[i]);
    }
    ocp_nlp_dims_set_cost(nlp_config, nlp_dims, 0, "ny", &ny[0]);
    for (int i = 1; i < N; i++)
        ocp_nlp_dims_set_cost(nlp_config, nlp_dims, i, "ny", &ny[i]);

    for (int i = 0; i < N; i++)
    {
    }
    ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, N, "nh", &nh[N]);
    ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, N, "nsh", &nsh[N]);
    ocp_nlp_dims_set_cost(nlp_config, nlp_dims, N, "ny", &ny[N]);
    free(intNp1mem);
return nlp_dims;
}


/**
 * Internal function for angular_ode_drone_acados_create: step 3
 */
void angular_ode_drone_acados_create_3_create_and_set_functions(angular_ode_drone_solver_capsule* capsule)
{
    const int N = capsule->nlp_solver_plan->N;

    /************************************************
    *  external functions
    ************************************************/

#define MAP_CASADI_FNC(__CAPSULE_FNC__, __MODEL_BASE_FNC__) do{ \
        capsule->__CAPSULE_FNC__.casadi_fun = & __MODEL_BASE_FNC__ ;\
        capsule->__CAPSULE_FNC__.casadi_n_in = & __MODEL_BASE_FNC__ ## _n_in; \
        capsule->__CAPSULE_FNC__.casadi_n_out = & __MODEL_BASE_FNC__ ## _n_out; \
        capsule->__CAPSULE_FNC__.casadi_sparsity_in = & __MODEL_BASE_FNC__ ## _sparsity_in; \
        capsule->__CAPSULE_FNC__.casadi_sparsity_out = & __MODEL_BASE_FNC__ ## _sparsity_out; \
        capsule->__CAPSULE_FNC__.casadi_work = & __MODEL_BASE_FNC__ ## _work; \
        external_function_param_casadi_create(&capsule->__CAPSULE_FNC__ , 0); \
    }while(false)




    // discrete dynamics
    capsule->discr_dyn_phi_fun = (external_function_param_casadi *) malloc(sizeof(external_function_param_casadi)*N);
    for (int i = 0; i < N; i++)
    {
        MAP_CASADI_FNC(discr_dyn_phi_fun[i], angular_ode_drone_dyn_disc_phi_fun);
    }

    capsule->discr_dyn_phi_fun_jac_ut_xt = (external_function_param_casadi *) malloc(sizeof(external_function_param_casadi)*N);
    for (int i = 0; i < N; i++)
    {
        MAP_CASADI_FNC(discr_dyn_phi_fun_jac_ut_xt[i], angular_ode_drone_dyn_disc_phi_fun_jac);
    }
    capsule->discr_dyn_phi_fun_jac_ut_xt_hess = (external_function_param_casadi *) malloc(sizeof(external_function_param_casadi)*N);
    for (int i = 0; i < N; i++)
    {
        MAP_CASADI_FNC(discr_dyn_phi_fun_jac_ut_xt_hess[i], angular_ode_drone_dyn_disc_phi_fun_jac_hess);
    }

#undef MAP_CASADI_FNC
}


/**
 * Internal function for angular_ode_drone_acados_create: step 4
 */
void angular_ode_drone_acados_create_4_set_default_parameters(angular_ode_drone_solver_capsule* capsule) {
    // no parameters defined
}


/**
 * Internal function for angular_ode_drone_acados_create: step 5
 */
void angular_ode_drone_acados_create_5_set_nlp_in(angular_ode_drone_solver_capsule* capsule, const int N, double* new_time_steps)
{
    assert(N == capsule->nlp_solver_plan->N);
    ocp_nlp_config* nlp_config = capsule->nlp_config;
    ocp_nlp_dims* nlp_dims = capsule->nlp_dims;

    /************************************************
    *  nlp_in
    ************************************************/
//    ocp_nlp_in * nlp_in = ocp_nlp_in_create(nlp_config, nlp_dims);
//    capsule->nlp_in = nlp_in;
    ocp_nlp_in * nlp_in = capsule->nlp_in;

    // set up time_steps
    

    if (new_time_steps) {
        angular_ode_drone_acados_update_time_steps(capsule, N, new_time_steps);
    } else {// all time_steps are identical
        double time_step = 0.02857142857142857;
        for (int i = 0; i < N; i++)
        {
            ocp_nlp_in_set(nlp_config, nlp_dims, nlp_in, i, "Ts", &time_step);
            ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "scaling", &time_step);
        }
    }

    /**** Dynamics ****/
    for (int i = 0; i < N; i++)
    {
        ocp_nlp_dynamics_model_set(nlp_config, nlp_dims, nlp_in, i, "disc_dyn_fun", &capsule->discr_dyn_phi_fun[i]);
        ocp_nlp_dynamics_model_set(nlp_config, nlp_dims, nlp_in, i, "disc_dyn_fun_jac",
                                   &capsule->discr_dyn_phi_fun_jac_ut_xt[i]);
        ocp_nlp_dynamics_model_set(nlp_config, nlp_dims, nlp_in, i, "disc_dyn_fun_jac_hess",
                                   &capsule->discr_dyn_phi_fun_jac_ut_xt_hess[i]);
    }

    /**** Cost ****/
    double* yref_0 = calloc(NY0, sizeof(double));
    // change only the non-zero elements:
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, 0, "yref", yref_0);
    free(yref_0);
    double* yref = calloc(NY, sizeof(double));
    // change only the non-zero elements:

    for (int i = 1; i < N; i++)
    {
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "yref", yref);
    }
    free(yref);
    double* yref_e = calloc(NYN, sizeof(double));
    // change only the non-zero elements:
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N, "yref", yref_e);
    free(yref_e);
   double* W_0 = calloc(NY0*NY0, sizeof(double));
    // change only the non-zero elements:
    W_0[0+(NY0) * 0] = 5;
    W_0[1+(NY0) * 1] = 5;
    W_0[2+(NY0) * 2] = 10;
    W_0[15+(NY0) * 15] = 10;
    W_0[16+(NY0) * 16] = 10;
    W_0[17+(NY0) * 17] = 0.01;
    W_0[18+(NY0) * 18] = 10;
    W_0[19+(NY0) * 19] = 10;
    W_0[20+(NY0) * 20] = 10;
    W_0[33+(NY0) * 33] = 0.06666666666666667;
    W_0[34+(NY0) * 34] = 3.3333333333333335;
    W_0[35+(NY0) * 35] = 3.3333333333333335;
    W_0[36+(NY0) * 36] = 3.3333333333333335;
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, 0, "W", W_0);
    free(W_0);
    double* W = calloc(NY*NY, sizeof(double));
    // change only the non-zero elements:
    W[0+(NY) * 0] = 5;
    W[1+(NY) * 1] = 5;
    W[2+(NY) * 2] = 10;
    W[15+(NY) * 15] = 10;
    W[16+(NY) * 16] = 10;
    W[17+(NY) * 17] = 0.01;
    W[18+(NY) * 18] = 10;
    W[19+(NY) * 19] = 10;
    W[20+(NY) * 20] = 10;
    W[33+(NY) * 33] = 0.06666666666666667;
    W[34+(NY) * 34] = 3.3333333333333335;
    W[35+(NY) * 35] = 3.3333333333333335;
    W[36+(NY) * 36] = 3.3333333333333335;

    for (int i = 1; i < N; i++)
    {
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "W", W);
    }
    free(W);
    double* W_e = calloc(NYN*NYN, sizeof(double));
    // change only the non-zero elements:
    W_e[0+(NYN) * 0] = 5;
    W_e[1+(NYN) * 1] = 5;
    W_e[2+(NYN) * 2] = 10;
    W_e[15+(NYN) * 15] = 10;
    W_e[16+(NYN) * 16] = 10;
    W_e[17+(NYN) * 17] = 0.01;
    W_e[18+(NYN) * 18] = 10;
    W_e[19+(NYN) * 19] = 10;
    W_e[20+(NYN) * 20] = 10;
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N, "W", W_e);
    free(W_e);
    double* Vx_0 = calloc(NY0*NX, sizeof(double));
    // change only the non-zero elements:
    Vx_0[0+(NY0) * 0] = 1;
    Vx_0[1+(NY0) * 1] = 1;
    Vx_0[2+(NY0) * 2] = 1;
    Vx_0[3+(NY0) * 3] = 1;
    Vx_0[4+(NY0) * 4] = 1;
    Vx_0[5+(NY0) * 5] = 1;
    Vx_0[6+(NY0) * 6] = 1;
    Vx_0[7+(NY0) * 7] = 1;
    Vx_0[8+(NY0) * 8] = 1;
    Vx_0[9+(NY0) * 9] = 1;
    Vx_0[10+(NY0) * 10] = 1;
    Vx_0[11+(NY0) * 11] = 1;
    Vx_0[12+(NY0) * 12] = 1;
    Vx_0[13+(NY0) * 13] = 1;
    Vx_0[14+(NY0) * 14] = 1;
    Vx_0[15+(NY0) * 15] = 1;
    Vx_0[16+(NY0) * 16] = 1;
    Vx_0[17+(NY0) * 17] = 1;
    Vx_0[18+(NY0) * 18] = 1;
    Vx_0[19+(NY0) * 19] = 1;
    Vx_0[20+(NY0) * 20] = 1;
    Vx_0[21+(NY0) * 21] = 1;
    Vx_0[22+(NY0) * 22] = 1;
    Vx_0[23+(NY0) * 23] = 1;
    Vx_0[24+(NY0) * 24] = 1;
    Vx_0[25+(NY0) * 25] = 1;
    Vx_0[26+(NY0) * 26] = 1;
    Vx_0[27+(NY0) * 27] = 1;
    Vx_0[28+(NY0) * 28] = 1;
    Vx_0[29+(NY0) * 29] = 1;
    Vx_0[30+(NY0) * 30] = 1;
    Vx_0[31+(NY0) * 31] = 1;
    Vx_0[32+(NY0) * 32] = 1;
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, 0, "Vx", Vx_0);
    free(Vx_0);
    double* Vu_0 = calloc(NY0*NU, sizeof(double));
    // change only the non-zero elements:
    Vu_0[33+(NY0) * 0] = 1;
    Vu_0[34+(NY0) * 1] = 1;
    Vu_0[35+(NY0) * 2] = 1;
    Vu_0[36+(NY0) * 3] = 1;
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, 0, "Vu", Vu_0);
    free(Vu_0);
    double* Vx = calloc(NY*NX, sizeof(double));
    // change only the non-zero elements:
    Vx[0+(NY) * 0] = 1;
    Vx[1+(NY) * 1] = 1;
    Vx[2+(NY) * 2] = 1;
    Vx[3+(NY) * 3] = 1;
    Vx[4+(NY) * 4] = 1;
    Vx[5+(NY) * 5] = 1;
    Vx[6+(NY) * 6] = 1;
    Vx[7+(NY) * 7] = 1;
    Vx[8+(NY) * 8] = 1;
    Vx[9+(NY) * 9] = 1;
    Vx[10+(NY) * 10] = 1;
    Vx[11+(NY) * 11] = 1;
    Vx[12+(NY) * 12] = 1;
    Vx[13+(NY) * 13] = 1;
    Vx[14+(NY) * 14] = 1;
    Vx[15+(NY) * 15] = 1;
    Vx[16+(NY) * 16] = 1;
    Vx[17+(NY) * 17] = 1;
    Vx[18+(NY) * 18] = 1;
    Vx[19+(NY) * 19] = 1;
    Vx[20+(NY) * 20] = 1;
    Vx[21+(NY) * 21] = 1;
    Vx[22+(NY) * 22] = 1;
    Vx[23+(NY) * 23] = 1;
    Vx[24+(NY) * 24] = 1;
    Vx[25+(NY) * 25] = 1;
    Vx[26+(NY) * 26] = 1;
    Vx[27+(NY) * 27] = 1;
    Vx[28+(NY) * 28] = 1;
    Vx[29+(NY) * 29] = 1;
    Vx[30+(NY) * 30] = 1;
    Vx[31+(NY) * 31] = 1;
    Vx[32+(NY) * 32] = 1;
    for (int i = 1; i < N; i++)
    {
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "Vx", Vx);
    }
    free(Vx);

    
    double* Vu = calloc(NY*NU, sizeof(double));
    // change only the non-zero elements:
    
    Vu[33+(NY) * 0] = 1;
    Vu[34+(NY) * 1] = 1;
    Vu[35+(NY) * 2] = 1;
    Vu[36+(NY) * 3] = 1;

    for (int i = 1; i < N; i++)
    {
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "Vu", Vu);
    }
    free(Vu);
    double* Vx_e = calloc(NYN*NX, sizeof(double));
    // change only the non-zero elements:
    
    Vx_e[0+(NYN) * 0] = 1;
    Vx_e[1+(NYN) * 1] = 1;
    Vx_e[2+(NYN) * 2] = 1;
    Vx_e[3+(NYN) * 3] = 1;
    Vx_e[4+(NYN) * 4] = 1;
    Vx_e[5+(NYN) * 5] = 1;
    Vx_e[6+(NYN) * 6] = 1;
    Vx_e[7+(NYN) * 7] = 1;
    Vx_e[8+(NYN) * 8] = 1;
    Vx_e[9+(NYN) * 9] = 1;
    Vx_e[10+(NYN) * 10] = 1;
    Vx_e[11+(NYN) * 11] = 1;
    Vx_e[12+(NYN) * 12] = 1;
    Vx_e[13+(NYN) * 13] = 1;
    Vx_e[14+(NYN) * 14] = 1;
    Vx_e[15+(NYN) * 15] = 1;
    Vx_e[16+(NYN) * 16] = 1;
    Vx_e[17+(NYN) * 17] = 1;
    Vx_e[18+(NYN) * 18] = 1;
    Vx_e[19+(NYN) * 19] = 1;
    Vx_e[20+(NYN) * 20] = 1;
    Vx_e[21+(NYN) * 21] = 1;
    Vx_e[22+(NYN) * 22] = 1;
    Vx_e[23+(NYN) * 23] = 1;
    Vx_e[24+(NYN) * 24] = 1;
    Vx_e[25+(NYN) * 25] = 1;
    Vx_e[26+(NYN) * 26] = 1;
    Vx_e[27+(NYN) * 27] = 1;
    Vx_e[28+(NYN) * 28] = 1;
    Vx_e[29+(NYN) * 29] = 1;
    Vx_e[30+(NYN) * 30] = 1;
    Vx_e[31+(NYN) * 31] = 1;
    Vx_e[32+(NYN) * 32] = 1;
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N, "Vx", Vx_e);
    free(Vx_e);



    /**** Constraints ****/

    // bounds for initial stage
    // x0
    int* idxbx0 = malloc(NBX0 * sizeof(int));
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

    double* lubx0 = calloc(2*NBX0, sizeof(double));
    double* lbx0 = lubx0;
    double* ubx0 = lubx0 + NBX0;
    // change only the non-zero elements:
    lbx0[0] = 0.00036519812824219926;
    ubx0[0] = 0.00036519812824219926;
    lbx0[1] = 0.00014104792683700723;
    ubx0[1] = 0.00014104792683700723;
    lbx0[2] = -0.0004763591686937582;
    ubx0[2] = -0.0004763591686937582;
    lbx0[3] = 0.902884634931744;
    ubx0[3] = 0.902884634931744;
    lbx0[4] = 0.3099417047474593;
    ubx0[4] = 0.3099417047474593;
    lbx0[5] = 0.21244907124923412;
    ubx0[5] = 0.21244907124923412;
    lbx0[6] = 0.444971961171808;
    ubx0[6] = 0.444971961171808;
    lbx0[7] = 0.902693984038697;
    ubx0[7] = 0.902693984038697;
    lbx0[8] = 0.3097370015083993;
    ubx0[8] = 0.3097370015083993;
    lbx0[9] = 0.21265226166560217;
    ubx0[9] = 0.21265226166560217;
    lbx0[10] = 0.4452049504164997;
    ubx0[10] = 0.4452049504164997;
    lbx0[11] = 0.6598909373223302;
    ubx0[11] = 0.6598909373223302;
    lbx0[12] = 0.14348535440655671;
    ubx0[12] = 0.14348535440655671;
    lbx0[13] = 0.34846581442791874;
    ubx0[13] = 0.34846581442791874;
    lbx0[14] = 0.5140029573707102;
    ubx0[14] = 0.5140029573707102;
    lbx0[15] = -0.00028102280892471456;
    ubx0[15] = -0.00028102280892471456;
    lbx0[16] = 0.0001876999553687142;
    ubx0[16] = 0.0001876999553687142;
    lbx0[17] = -0.000013559328138805736;
    ubx0[17] = -0.000013559328138805736;
    lbx0[18] = 0.00012229611728413733;
    ubx0[18] = 0.00012229611728413733;
    lbx0[19] = -0.0006359830361039068;
    ubx0[19] = -0.0006359830361039068;
    lbx0[20] = 0.000010162458291138454;
    ubx0[20] = 0.000010162458291138454;
    lbx0[21] = 0.5161775417621723;
    ubx0[21] = 0.5161775417621723;
    lbx0[22] = 0.12831596942221302;
    ubx0[22] = 0.12831596942221302;
    lbx0[23] = 0.2512864768228637;
    ubx0[23] = 0.2512864768228637;
    lbx0[24] = 0.14859384162942746;
    ubx0[24] = 0.14859384162942746;
    lbx0[25] = 0.5156755095336953;
    ubx0[25] = 0.5156755095336953;
    lbx0[26] = 0.12868055675485354;
    ubx0[26] = 0.12868055675485354;
    lbx0[27] = 0.2507656351919968;
    ubx0[27] = 0.2507656351919968;
    lbx0[28] = 0.14840622518086843;
    ubx0[28] = 0.14840622518086843;
    lbx0[29] = 0.5161034952567787;
    ubx0[29] = 0.5161034952567787;
    lbx0[30] = 0.12836987493075375;
    ubx0[30] = 0.12836987493075375;
    lbx0[31] = 0.25120949632049494;
    ubx0[31] = 0.25120949632049494;
    lbx0[32] = 0.1485661467218956;
    ubx0[32] = 0.1485661467218956;

    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "idxbx", idxbx0);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "lbx", lbx0);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "ubx", ubx0);
    free(idxbx0);
    free(lubx0);
    // idxbxe_0
    int* idxbxe_0 = malloc(33 * sizeof(int));
    
    idxbxe_0[0] = 0;
    idxbxe_0[1] = 1;
    idxbxe_0[2] = 2;
    idxbxe_0[3] = 3;
    idxbxe_0[4] = 4;
    idxbxe_0[5] = 5;
    idxbxe_0[6] = 6;
    idxbxe_0[7] = 7;
    idxbxe_0[8] = 8;
    idxbxe_0[9] = 9;
    idxbxe_0[10] = 10;
    idxbxe_0[11] = 11;
    idxbxe_0[12] = 12;
    idxbxe_0[13] = 13;
    idxbxe_0[14] = 14;
    idxbxe_0[15] = 15;
    idxbxe_0[16] = 16;
    idxbxe_0[17] = 17;
    idxbxe_0[18] = 18;
    idxbxe_0[19] = 19;
    idxbxe_0[20] = 20;
    idxbxe_0[21] = 21;
    idxbxe_0[22] = 22;
    idxbxe_0[23] = 23;
    idxbxe_0[24] = 24;
    idxbxe_0[25] = 25;
    idxbxe_0[26] = 26;
    idxbxe_0[27] = 27;
    idxbxe_0[28] = 28;
    idxbxe_0[29] = 29;
    idxbxe_0[30] = 30;
    idxbxe_0[31] = 31;
    idxbxe_0[32] = 32;
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "idxbxe", idxbxe_0);
    free(idxbxe_0);

    /* constraints that are the same for initial and intermediate */
    // u
    int* idxbu = malloc(NBU * sizeof(int));
    
    idxbu[0] = 0;
    idxbu[1] = 1;
    idxbu[2] = 2;
    idxbu[3] = 3;
    double* lubu = calloc(2*NBU, sizeof(double));
    double* lbu = lubu;
    double* ubu = lubu + NBU;
    
    lbu[0] = 3;
    ubu[0] = 15;
    lbu[1] = -0.3;
    ubu[1] = 0.3;
    lbu[2] = -0.3;
    ubu[2] = 0.3;
    lbu[3] = -0.3;
    ubu[3] = 0.3;

    for (int i = 0; i < N; i++)
    {
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "idxbu", idxbu);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "lbu", lbu);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "ubu", ubu);
    }
    free(idxbu);
    free(lubu);















    /* terminal constraints */















}


/**
 * Internal function for angular_ode_drone_acados_create: step 6
 */
void angular_ode_drone_acados_create_6_set_opts(angular_ode_drone_solver_capsule* capsule)
{
    const int N = capsule->nlp_solver_plan->N;
    ocp_nlp_config* nlp_config = capsule->nlp_config;
    void *nlp_opts = capsule->nlp_opts;

    /************************************************
    *  opts
    ************************************************/


    int nlp_solver_exact_hessian = 1;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "exact_hess", &nlp_solver_exact_hessian);

    int exact_hess_dyn = 1;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "exact_hess_dyn", &exact_hess_dyn);

    int exact_hess_cost = 1;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "exact_hess_cost", &exact_hess_cost);

    int exact_hess_constr = 1;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "exact_hess_constr", &exact_hess_constr);
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "globalization", "fixed_step");int full_step_dual = 0;
    ocp_nlp_solver_opts_set(nlp_config, capsule->nlp_opts, "full_step_dual", &full_step_dual);

    double nlp_solver_step_length = 1;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "step_length", &nlp_solver_step_length);

    double levenberg_marquardt = 0;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "levenberg_marquardt", &levenberg_marquardt);

    /* options QP solver */

    int nlp_solver_ext_qp_res = 0;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "ext_qp_res", &nlp_solver_ext_qp_res);
    // set HPIPM mode: should be done before setting other QP solver options
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qp_hpipm_mode", "BALANCE");



    int qp_solver_iter_max = 50;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qp_iter_max", &qp_solver_iter_max);

int print_level = 0;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "print_level", &print_level);

    int ext_cost_num_hess = 0;
}


/**
 * Internal function for angular_ode_drone_acados_create: step 7
 */
void angular_ode_drone_acados_create_7_set_nlp_out(angular_ode_drone_solver_capsule* capsule)
{
    const int N = capsule->nlp_solver_plan->N;
    ocp_nlp_config* nlp_config = capsule->nlp_config;
    ocp_nlp_dims* nlp_dims = capsule->nlp_dims;
    ocp_nlp_out* nlp_out = capsule->nlp_out;

    // initialize primal solution
    double* xu0 = calloc(NX+NU, sizeof(double));
    double* x0 = xu0;

    // initialize with x0
    
    x0[0] = 0.00036519812824219926;
    x0[1] = 0.00014104792683700723;
    x0[2] = -0.0004763591686937582;
    x0[3] = 0.902884634931744;
    x0[4] = 0.3099417047474593;
    x0[5] = 0.21244907124923412;
    x0[6] = 0.444971961171808;
    x0[7] = 0.902693984038697;
    x0[8] = 0.3097370015083993;
    x0[9] = 0.21265226166560217;
    x0[10] = 0.4452049504164997;
    x0[11] = 0.6598909373223302;
    x0[12] = 0.14348535440655671;
    x0[13] = 0.34846581442791874;
    x0[14] = 0.5140029573707102;
    x0[15] = -0.00028102280892471456;
    x0[16] = 0.0001876999553687142;
    x0[17] = -0.000013559328138805736;
    x0[18] = 0.00012229611728413733;
    x0[19] = -0.0006359830361039068;
    x0[20] = 0.000010162458291138454;
    x0[21] = 0.5161775417621723;
    x0[22] = 0.12831596942221302;
    x0[23] = 0.2512864768228637;
    x0[24] = 0.14859384162942746;
    x0[25] = 0.5156755095336953;
    x0[26] = 0.12868055675485354;
    x0[27] = 0.2507656351919968;
    x0[28] = 0.14840622518086843;
    x0[29] = 0.5161034952567787;
    x0[30] = 0.12836987493075375;
    x0[31] = 0.25120949632049494;
    x0[32] = 0.1485661467218956;


    double* u0 = xu0 + NX;

    for (int i = 0; i < N; i++)
    {
        // x0
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "x", x0);
        // u0
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "u", u0);
    }
    ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, N, "x", x0);
    free(xu0);
}


/**
 * Internal function for angular_ode_drone_acados_create: step 8
 */
//void angular_ode_drone_acados_create_8_create_solver(angular_ode_drone_solver_capsule* capsule)
//{
//    capsule->nlp_solver = ocp_nlp_solver_create(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_opts);
//}

/**
 * Internal function for angular_ode_drone_acados_create: step 9
 */
int angular_ode_drone_acados_create_9_precompute(angular_ode_drone_solver_capsule* capsule) {
    int status = ocp_nlp_precompute(capsule->nlp_solver, capsule->nlp_in, capsule->nlp_out);

    if (status != ACADOS_SUCCESS) {
        printf("\nocp_nlp_precompute failed!\n\n");
        exit(1);
    }

    return status;
}


int angular_ode_drone_acados_create_with_discretization(angular_ode_drone_solver_capsule* capsule, int N, double* new_time_steps)
{
    // If N does not match the number of shooting intervals used for code generation, new_time_steps must be given.
    if (N != ANGULAR_ODE_DRONE_N && !new_time_steps) {
        fprintf(stderr, "angular_ode_drone_acados_create_with_discretization: new_time_steps is NULL " \
            "but the number of shooting intervals (= %d) differs from the number of " \
            "shooting intervals (= %d) during code generation! Please provide a new vector of time_stamps!\n", \
             N, ANGULAR_ODE_DRONE_N);
        return 1;
    }

    // number of expected runtime parameters
    capsule->nlp_np = NP;

    // 1) create and set nlp_solver_plan; create nlp_config
    capsule->nlp_solver_plan = ocp_nlp_plan_create(N);
    angular_ode_drone_acados_create_1_set_plan(capsule->nlp_solver_plan, N);
    capsule->nlp_config = ocp_nlp_config_create(*capsule->nlp_solver_plan);

    // 3) create and set dimensions
    capsule->nlp_dims = angular_ode_drone_acados_create_2_create_and_set_dimensions(capsule);
    angular_ode_drone_acados_create_3_create_and_set_functions(capsule);

    // 4) set default parameters in functions
    angular_ode_drone_acados_create_4_set_default_parameters(capsule);

    // 5) create and set nlp_in
    capsule->nlp_in = ocp_nlp_in_create(capsule->nlp_config, capsule->nlp_dims);
    angular_ode_drone_acados_create_5_set_nlp_in(capsule, N, new_time_steps);

    // 6) create and set nlp_opts
    capsule->nlp_opts = ocp_nlp_solver_opts_create(capsule->nlp_config, capsule->nlp_dims);
    angular_ode_drone_acados_create_6_set_opts(capsule);

    // 7) create and set nlp_out
    // 7.1) nlp_out
    capsule->nlp_out = ocp_nlp_out_create(capsule->nlp_config, capsule->nlp_dims);
    // 7.2) sens_out
    capsule->sens_out = ocp_nlp_out_create(capsule->nlp_config, capsule->nlp_dims);
    angular_ode_drone_acados_create_7_set_nlp_out(capsule);

    // 8) create solver
    capsule->nlp_solver = ocp_nlp_solver_create(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_opts);
    //angular_ode_drone_acados_create_8_create_solver(capsule);

    // 9) do precomputations
    int status = angular_ode_drone_acados_create_9_precompute(capsule);

    return status;
}

/**
 * This function is for updating an already initialized solver with a different number of qp_cond_N. It is useful for code reuse after code export.
 */
int angular_ode_drone_acados_update_qp_solver_cond_N(angular_ode_drone_solver_capsule* capsule, int qp_solver_cond_N)
{
    printf("\nacados_update_qp_solver_cond_N() failed, since no partial condensing solver is used!\n\n");
    // Todo: what is an adequate behavior here?
    exit(1);
    return -1;
}


int angular_ode_drone_acados_reset(angular_ode_drone_solver_capsule* capsule, int reset_qp_solver_mem)
{

    // set initialization to all zeros

    const int N = capsule->nlp_solver_plan->N;
    ocp_nlp_config* nlp_config = capsule->nlp_config;
    ocp_nlp_dims* nlp_dims = capsule->nlp_dims;
    ocp_nlp_out* nlp_out = capsule->nlp_out;
    ocp_nlp_in* nlp_in = capsule->nlp_in;
    ocp_nlp_solver* nlp_solver = capsule->nlp_solver;

    double* buffer = calloc(NX+NU+NZ+2*NS+2*NSN+NBX+NBU+NG+NH+NPHI+NBX0+NBXN+NHN+NPHIN+NGN, sizeof(double));

    for(int i=0; i<N+1; i++)
    {
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "x", buffer);
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "u", buffer);
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "sl", buffer);
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "su", buffer);
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "lam", buffer);
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "t", buffer);
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "z", buffer);
        if (i<N)
        {
            ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "pi", buffer);
        }
    }

    free(buffer);
    return 0;
}




int angular_ode_drone_acados_update_params(angular_ode_drone_solver_capsule* capsule, int stage, double *p, int np)
{
    int solver_status = 0;

    int casadi_np = 0;
    if (casadi_np != np) {
        printf("acados_update_params: trying to set %i parameters for external functions."
            " External function has %i parameters. Exiting.\n", np, casadi_np);
        exit(1);
    }

    const int N = capsule->nlp_solver_plan->N;
    if (stage < N && stage >= 0)
    {
        capsule->discr_dyn_phi_fun[stage].set_param(capsule->discr_dyn_phi_fun+stage, p);
        capsule->discr_dyn_phi_fun_jac_ut_xt[stage].set_param(capsule->discr_dyn_phi_fun_jac_ut_xt+stage, p);
        capsule->discr_dyn_phi_fun_jac_ut_xt_hess[stage].set_param(capsule->discr_dyn_phi_fun_jac_ut_xt_hess+stage, p);
    

        // constraints
    

        // cost
        if (stage == 0)
        {
        }
        else // 0 < stage < N
        {
        }
    }

    else // stage == N
    {
        // terminal shooting node has no dynamics
        // cost
        // constraints
    
    }

    return solver_status;
}


int angular_ode_drone_acados_update_params_sparse(angular_ode_drone_solver_capsule * capsule, int stage, int *idx, double *p, int n_update)
{
    int solver_status = 0;

    int casadi_np = 0;
    if (casadi_np < n_update) {
        printf("angular_ode_drone_acados_update_params_sparse: trying to set %d parameters for external functions."
            " External function has %d parameters. Exiting.\n", n_update, casadi_np);
        exit(1);
    }
    // for (int i = 0; i < n_update; i++)
    // {
    //     if (idx[i] > casadi_np) {
    //         printf("angular_ode_drone_acados_update_params_sparse: attempt to set parameters with index %d, while"
    //             " external functions only has %d parameters. Exiting.\n", idx[i], casadi_np);
    //         exit(1);
    //     }
    //     printf("param %d value %e\n", idx[i], p[i]);
    // }

    return solver_status;
}

int angular_ode_drone_acados_solve(angular_ode_drone_solver_capsule* capsule)
{
    // solve NLP
    int solver_status = ocp_nlp_solve(capsule->nlp_solver, capsule->nlp_in, capsule->nlp_out);

    return solver_status;
}


int angular_ode_drone_acados_free(angular_ode_drone_solver_capsule* capsule)
{
    // before destroying, keep some info
    const int N = capsule->nlp_solver_plan->N;
    // free memory
    ocp_nlp_solver_opts_destroy(capsule->nlp_opts);
    ocp_nlp_in_destroy(capsule->nlp_in);
    ocp_nlp_out_destroy(capsule->nlp_out);
    ocp_nlp_out_destroy(capsule->sens_out);
    ocp_nlp_solver_destroy(capsule->nlp_solver);
    ocp_nlp_dims_destroy(capsule->nlp_dims);
    ocp_nlp_config_destroy(capsule->nlp_config);
    ocp_nlp_plan_destroy(capsule->nlp_solver_plan);

    /* free external function */
    // dynamics
    for (int i = 0; i < N; i++)
    {
        external_function_param_casadi_free(&capsule->discr_dyn_phi_fun[i]);
        external_function_param_casadi_free(&capsule->discr_dyn_phi_fun_jac_ut_xt[i]);
        external_function_param_casadi_free(&capsule->discr_dyn_phi_fun_jac_ut_xt_hess[i]);
    }
    free(capsule->discr_dyn_phi_fun);
    free(capsule->discr_dyn_phi_fun_jac_ut_xt);
    free(capsule->discr_dyn_phi_fun_jac_ut_xt_hess);

    // cost

    // constraints

    return 0;
}


void angular_ode_drone_acados_print_stats(angular_ode_drone_solver_capsule* capsule)
{
    int sqp_iter, stat_m, stat_n, tmp_int;
    ocp_nlp_get(capsule->nlp_config, capsule->nlp_solver, "sqp_iter", &sqp_iter);
    ocp_nlp_get(capsule->nlp_config, capsule->nlp_solver, "stat_n", &stat_n);
    ocp_nlp_get(capsule->nlp_config, capsule->nlp_solver, "stat_m", &stat_m);

    
    double stat[1200];
    ocp_nlp_get(capsule->nlp_config, capsule->nlp_solver, "statistics", stat);

    int nrow = sqp_iter+1 < stat_m ? sqp_iter+1 : stat_m;

    printf("iter\tres_stat\tres_eq\t\tres_ineq\tres_comp\tqp_stat\tqp_iter\talpha");
    if (stat_n > 8)
        printf("\t\tqp_res_stat\tqp_res_eq\tqp_res_ineq\tqp_res_comp");
    printf("\n");
    printf("iter\tqp_stat\tqp_iter\n");
    for (int i = 0; i < nrow; i++)
    {
        for (int j = 0; j < stat_n + 1; j++)
        {
            tmp_int = (int) stat[i + j * nrow];
            printf("%d\t", tmp_int);
        }
        printf("\n");
    }
}

int angular_ode_drone_acados_custom_update(angular_ode_drone_solver_capsule* capsule, double* data, int data_len)
{
    (void)capsule;
    (void)data;
    (void)data_len;
    printf("\ndummy function that can be called in between solver calls to update parameters or numerical data efficiently in C.\n");
    printf("nothing set yet..\n");
    return 1;

}



ocp_nlp_in *angular_ode_drone_acados_get_nlp_in(angular_ode_drone_solver_capsule* capsule) { return capsule->nlp_in; }
ocp_nlp_out *angular_ode_drone_acados_get_nlp_out(angular_ode_drone_solver_capsule* capsule) { return capsule->nlp_out; }
ocp_nlp_out *angular_ode_drone_acados_get_sens_out(angular_ode_drone_solver_capsule* capsule) { return capsule->sens_out; }
ocp_nlp_solver *angular_ode_drone_acados_get_nlp_solver(angular_ode_drone_solver_capsule* capsule) { return capsule->nlp_solver; }
ocp_nlp_config *angular_ode_drone_acados_get_nlp_config(angular_ode_drone_solver_capsule* capsule) { return capsule->nlp_config; }
void *angular_ode_drone_acados_get_nlp_opts(angular_ode_drone_solver_capsule* capsule) { return capsule->nlp_opts; }
ocp_nlp_dims *angular_ode_drone_acados_get_nlp_dims(angular_ode_drone_solver_capsule* capsule) { return capsule->nlp_dims; }
ocp_nlp_plan_t *angular_ode_drone_acados_get_nlp_plan(angular_ode_drone_solver_capsule* capsule) { return capsule->nlp_solver_plan; }
