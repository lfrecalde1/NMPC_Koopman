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
#include "acados_c/sim_interface.h"
#include "acados_sim_solver_angular_ode_drone.h"

#define NX     ANGULAR_ODE_DRONE_NX
#define NZ     ANGULAR_ODE_DRONE_NZ
#define NU     ANGULAR_ODE_DRONE_NU
#define NP     ANGULAR_ODE_DRONE_NP


int main()
{
    int status = 0;
    sim_solver_capsule *capsule = angular_ode_drone_acados_sim_solver_create_capsule();
    status = angular_ode_drone_acados_sim_create(capsule);

    if (status)
    {
        printf("acados_create() returned status %d. Exiting.\n", status);
        exit(1);
    }

    sim_config *acados_sim_config = angular_ode_drone_acados_get_sim_config(capsule);
    sim_in *acados_sim_in = angular_ode_drone_acados_get_sim_in(capsule);
    sim_out *acados_sim_out = angular_ode_drone_acados_get_sim_out(capsule);
    void *acados_sim_dims = angular_ode_drone_acados_get_sim_dims(capsule);

    // initial condition
    double x_current[NX];
    x_current[0] = 0.0;
    x_current[1] = 0.0;
    x_current[2] = 0.0;
    x_current[3] = 0.0;
    x_current[4] = 0.0;
    x_current[5] = 0.0;
    x_current[6] = 0.0;
    x_current[7] = 0.0;
    x_current[8] = 0.0;
    x_current[9] = 0.0;
    x_current[10] = 0.0;
    x_current[11] = 0.0;
    x_current[12] = 0.0;
    x_current[13] = 0.0;
    x_current[14] = 0.0;
    x_current[15] = 0.0;
    x_current[16] = 0.0;
    x_current[17] = 0.0;
    x_current[18] = 0.0;
    x_current[19] = 0.0;
    x_current[20] = 0.0;
    x_current[21] = 0.0;
    x_current[22] = 0.0;
    x_current[23] = 0.0;
    x_current[24] = 0.0;
    x_current[25] = 0.0;
    x_current[26] = 0.0;
    x_current[27] = 0.0;
    x_current[28] = 0.0;
    x_current[29] = 0.0;
    x_current[30] = 0.0;
    x_current[31] = 0.0;
    x_current[32] = 0.0;

  
    x_current[0] = 0.000014058635206510516;
    x_current[1] = 0.000038301197974813814;
    x_current[2] = -0.0000489050129798521;
    x_current[3] = 0.1597098877312515;
    x_current[4] = 0.7398746998444931;
    x_current[5] = 0.41756527529028914;
    x_current[6] = 0.7479188795062459;
    x_current[7] = 0.1597128050026188;
    x_current[8] = 0.7398606649385295;
    x_current[9] = 0.41758425419978545;
    x_current[10] = 0.7479001382331167;
    x_current[11] = 0.7916555055419454;
    x_current[12] = 0.15296923748113314;
    x_current[13] = 0.19636987630753633;
    x_current[14] = 0.3756036285361878;
    x_current[15] = -0.00002069465558984008;
    x_current[16] = 0.0001133652194493795;
    x_current[17] = -0.000041382904411561814;
    x_current[18] = -0.00013558921526946174;
    x_current[19] = -0.0000034062708143223465;
    x_current[20] = 0.00005380621893838568;
    x_current[21] = 0.27864986379455786;
    x_current[22] = 0.0776502508710669;
    x_current[23] = 0.21384317550542128;
    x_current[24] = 0.043636373590812474;
    x_current[25] = 0.2787930843534001;
    x_current[26] = 0.07763877488150561;
    x_current[27] = 0.21387946951989856;
    x_current[28] = 0.04360118129902983;
    x_current[29] = 0.2788550789267549;
    x_current[30] = 0.07763380322959645;
    x_current[31] = 0.21389516659211477;
    x_current[32] = 0.04358595505551924;
    
  


    // initial value for control input
    double u0[NU];
    u0[0] = 0.0;
    u0[1] = 0.0;
    u0[2] = 0.0;
    u0[3] = 0.0;

    int n_sim_steps = 3;
    // solve ocp in loop
    for (int ii = 0; ii < n_sim_steps; ii++)
    {
        sim_in_set(acados_sim_config, acados_sim_dims,
            acados_sim_in, "x", x_current);
        status = angular_ode_drone_acados_sim_solve(capsule);

        if (status != ACADOS_SUCCESS)
        {
            printf("acados_solve() failed with status %d.\n", status);
        }

        sim_out_get(acados_sim_config, acados_sim_dims,
               acados_sim_out, "x", x_current);
        
        printf("\nx_current, %d\n", ii);
        for (int jj = 0; jj < NX; jj++)
        {
            printf("%e\n", x_current[jj]);
        }
    }

    printf("\nPerformed %d simulation steps with acados integrator successfully.\n\n", n_sim_steps);

    // free solver
    status = angular_ode_drone_acados_sim_free(capsule);
    if (status) {
        printf("angular_ode_drone_acados_sim_free() returned status %d. \n", status);
    }

    angular_ode_drone_acados_sim_solver_free_capsule(capsule);

    return status;
}
