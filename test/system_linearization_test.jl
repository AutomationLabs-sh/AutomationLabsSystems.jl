# Copyright (c) 2022: Pierre Blaud and contributors
########################################################
# This Source Code Form is subject to the terms of the #
# Mozilla Public License, v. 2.0. If a copy of the MPL #
# was not distributed with this file,  				   #
# You can obtain one at https://mozilla.org/MPL/2.0/.  #
########################################################
module SystemsLinearizationTest

using Test
using AutomationLabsSystems
using AutomationLabsIdentification
using MathematicalSystems
using LazySets
using MLJ
using Flux

@testset "Linearize a non linear discrete system" begin

    densenet = machine("./models_saved/densenet_train_result.jls")

    f = MLJ.fitted_params(MLJ.fitted_params(densenet).machine).best_fitted_params[1]

    u_cons = [
        -1 1
        -1 1
    ]

    nbr_state = 4
    nbr_input = 2

    sys = proceed_system(f, nbr_state, nbr_input, "discrete"; input_constraint = u_cons)

    x_ref = [0.65, 0.65, 0.65, 0.65]
    u_ref = [1.2, 1.2]
    sys_l = proceed_system_linearization(sys, x_ref, u_ref)

    @test sys != sys_l

    A = [
        0.9660291756980193 0.008484343227303959 0.01939889850468557 -0.004673818610570313
        0.004013346951636243 0.9580492141418617 0.004447110839625346 0.025867371605228312
        -0.0004278251091284768 -0.005642248622235599 0.9817292321924587 0.004899331055170451
        -0.003206302536513883 -0.0023416433183096608 0.004552624958212237 0.9812636167718572
    ]

    @test sys_l.A == A

    B = [
        0.005005890406841962 -0.0009054445894797758
        0.0018200405922851048 0.00951261859582414
        0.00026973288447389165 0.012237568961798867
        0.01599142671835957 0.0032893008515649247
    ]

    @test sys_l.B == B


end

@testset "Linearize a non linear continuous system" begin


    densenet = machine("./models_saved/densenet_train_result.jls")

    f = MLJ.fitted_params(MLJ.fitted_params(densenet).machine).best_fitted_params[1]

    u_cons = [
        -1 1
        -1 1
    ]

    nbr_state = 4
    nbr_input = 2

    sys = proceed_system(f, nbr_state, nbr_input, "continuous"; input_constraint = u_cons)

    x_ref = [0.65, 0.65, 0.65, 0.65]
    u_ref = [1.2, 1.2]
    sys_l = proceed_system_linearization(sys, x_ref, u_ref)

    @test sys != sys_l

    A = [
        0.9660291756980193 0.008484343227303959 0.01939889850468557 -0.004673818610570313
        0.004013346951636243 0.9580492141418617 0.004447110839625346 0.025867371605228312
        -0.0004278251091284768 -0.005642248622235599 0.9817292321924587 0.004899331055170451
        -0.003206302536513883 -0.0023416433183096608 0.004552624958212237 0.9812636167718572
    ]

    @test sys_l.A == A

    B = [
        0.005005890406841962 -0.0009054445894797758
        0.0018200405922851048 0.00951261859582414
        0.00026973288447389165 0.012237568961798867
        0.01599142671835957 0.0032893008515649247
    ]

    @test sys_l.B == B

end


### Discrete ###
@testset "Linearize a discrete linear system" begin

    A = [1 1; 0 0.9]
    B = [1; 0.5]
    nbr_state = 2
    nbr_input = 1

    x_cons = [
        -5.0 5.0
        -5.0 5.0
    ]
    u_cons = [-1.0 1.0]

    sys = proceed_system(
        A,
        B,
        nbr_state,
        nbr_input,
        "discrete";
        input_constraint = u_cons,
        state_constraint = x_cons,
    )

    x_ref = [0.65, 0.65, 0.65, 0.65]
    u_ref = [1.2, 1.2]
    sys_l = proceed_system_linearization(sys, x_ref, u_ref)

    @test sys == sys_l
    @test sys.A == A 
    @test sys.B == [1.0; 0.5;;] 

end

### Continuous ###
@testset "Linearize a discrete linear system" begin

    A = [1 1; 0 0.9]
    B = [1; 0.5]
    nbr_state = 2
    nbr_input = 1

    x_cons = [
        -5.0 5.0
        -5.0 5.0
    ]
    u_cons = [-1.0 1.0]

    sys = proceed_system(
        A,
        B,
        nbr_state,
        nbr_input,
        "continuous";
        input_constraint = u_cons,
        state_constraint = x_cons,
    )

    x_ref = [0.65, 0.65, 0.65, 0.65]
    u_ref = [1.2, 1.2]
    sys_l = proceed_system_linearization(sys, x_ref, u_ref)

    @test sys == sys_l
    @test sys.A == A 
    @test sys.B == [1.0; 0.5;;] 

end

end
