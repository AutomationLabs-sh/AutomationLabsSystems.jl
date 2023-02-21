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

    model_origin = "identification"

    densenet = machine("./models_saved/densenet_train_result.jls")

    u_cons = [-1 1;
              -1 1]

    sys = proceed_system("discrete", model_origin; 
                            f = densenet, 
                            input_constraint = u_cons,
                            nbr_state = 4,
                            nbr_input = 2,
                            )

    x_ref = [0.65, 0.65, 0.65, 0.65]
    u_ref = [1.2, 1.2]
    sys_l = proceed_system_linearization(sys, x_ref, u_ref)

    @test typeof(sys) == ConstrainedBlackBoxControlDiscreteSystem{Flux.Chain{Tuple{Flux.Dense{typeof(identity), Matrix{Float32}, Bool}, Flux.Chain{Tuple{Flux.SkipConnection{Flux.Chain{Tuple{Flux.Dense{typeof(NNlib.relu), Matrix{Float32}, Vector{Float32}}}}, typeof(vcat)}}}, Flux.Dense{typeof(identity), Matrix{Float32}, Bool}}}, Hyperrectangle{Float64, Vector{Float64}, Vector{Float64}}, Hyperrectangle{Float64, Vector{Float64}, Vector{Float64}}}
    @test typeof(sys_l) == ConstrainedLinearControlDiscreteSystem{Float64, Matrix{Float64}, Matrix{Float64}, Hyperrectangle{Float64, Vector{Float64}, Vector{Float64}}, Hyperrectangle{Float64, Vector{Float64}, Vector{Float64}}}  

end

@testset "Linearize a non linear continuous system" begin

    model_origin = "identification"

    densenet = machine("./models_saved/densenet_train_result.jls")

    u_cons = [-1 1;
              -1 1]

    sys = proceed_system("continuous", model_origin; 
                            f = densenet, 
                            input_constraint = u_cons,
                            nbr_state = 4,
                            nbr_input = 2,
                            )

    x_ref = [0.65, 0.65, 0.65, 0.65]
    u_ref = [1.2, 1.2]
    sys_l = proceed_system_linearization(sys, x_ref, u_ref)

    @test typeof(sys) == ConstrainedBlackBoxControlContinuousSystem{Chain{Tuple{Dense{typeof(identity), Matrix{Float32}, Bool}, Chain{Tuple{SkipConnection{Chain{Tuple{Dense{typeof(relu), Matrix{Float32}, Vector{Float32}}}}, typeof(vcat)}}}, Dense{typeof(identity), Matrix{Float32}, Bool}}}, Hyperrectangle{Float64, Vector{Float64}, Vector{Float64}}, Hyperrectangle{Float64, Vector{Float64}, Vector{Float64}}}
    @test typeof(sys_l) == ConstrainedLinearControlContinuousSystem{Float64, Matrix{Float64}, Matrix{Float64}, Hyperrectangle{Float64, Vector{Float64}, Vector{Float64}}, Hyperrectangle{Float64, Vector{Float64}, Vector{Float64}}}

end

end