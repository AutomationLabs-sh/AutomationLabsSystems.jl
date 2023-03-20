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
using MathematicalSystems
using LazySets
using MLJ
using Flux

@testset "Discretize a linear continuous system" begin

    A = [1 1; 0 0.9]
    B = [1; 0.5]

    nbr_state = 2
    nbr_input = 1

    x_cons = [
        -5.0 5.0
        -5.0 5.0
    ]

    u_cons = [-1.0 1.0]

    sys_c = proceed_system(
        A,
        B,
        nbr_state,
        nbr_input,
        "continuous";
        input_constraint = u_cons,
        state_constraint = x_cons,
    )

    sample_time = 5.0
    sys_d = proceed_system_discretization(sys_c, sample_time)

    @test typeof(sys_c) == ConstrainedLinearControlContinuousSystem{
        Float64,
        Matrix{Float64},
        Matrix{Float64},
        Hyperrectangle{Float64,Vector{Float64},Vector{Float64}},
        Hyperrectangle{Float64,Vector{Float64},Vector{Float64}},
    }
    @test typeof(sys_d) == ConstrainedLinearControlDiscreteSystem{
        Float64,
        Matrix{Float64},
        Matrix{Float64},
        Hyperrectangle{Float64,Vector{Float64},Vector{Float64}},
        Hyperrectangle{Float64,Vector{Float64},Vector{Float64}},
    }

    @test sys_c.A != sys_d.A
    @test sys_c.B != sys_d.B
    @test sys_c.X == sys_d.X
    @test sys_c.U == sys_d.U

end

end
