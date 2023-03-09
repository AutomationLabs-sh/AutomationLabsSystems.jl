# Copyright (c) 2022: Pierre Blaud and contributors
########################################################
# This Source Code Form is subject to the terms of the #
# Mozilla Public License, v. 2.0. If a copy of the MPL #
# was not distributed with this file,  				   #
# You can obtain one at https://mozilla.org/MPL/2.0/.  #
########################################################
module SystemsTest

using Test
using AutomationLabsSystems
using AutomationLabsIdentification
using MathematicalSystems
using LazySets
using MLJ
using Flux


### Discrete ###
@testset "Linear user system with state and input constraints" begin

    model_origin = "user"
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

    @test typeof(sys) == ConstrainedLinearControlDiscreteSystem{
        Float64,
        Matrix{Float64},
        Matrix{Float64},
        Hyperrectangle{Float64,Vector{Float64},Vector{Float64}},
        Hyperrectangle{Float64,Vector{Float64},Vector{Float64}},
    }
    @test sys.A == A
    @test sys.B == [1.0; 0.5;;]
    @test LazySets.low(sys.X) == [-5.0, -5.0]
    @test LazySets.high(sys.X) == [5.0, 5.0]
    @test LazySets.low(sys.U) == [-1.0]
    @test LazySets.high(sys.U) == [1.0]
end

@testset "Linear user system with input constraints" begin

    model_origin = "user"
    A = [
        1 1
        0 0.9
    ]
    B = [1; 0.5]
    nbr_state = 2
    nbr_input = 1

    u_cons = [-1 1]

    sys = proceed_system(
        A,
        B, 
        nbr_state,
        nbr_input,
        "discrete";
        input_constraint = u_cons,
    )

    @test typeof(sys) == ConstrainedLinearControlDiscreteSystem{
        Float64,
        Matrix{Float64},
        Matrix{Float64},
        Hyperrectangle{Float64,Vector{Float64},Vector{Float64}},
        Hyperrectangle{Float64,Vector{Float64},Vector{Float64}},
    }
    @test sys.A == A
    @test sys.B == [1.0; 0.5;;]
    @test LazySets.low(sys.X) == [0.0, 0.0]
    @test LazySets.high(sys.X) == [0.0, 0.0]
    @test LazySets.low(sys.U) == [-1.0]
    @test LazySets.high(sys.U) == [1.0]
end

@testset "Linear user system without constraints" begin

    A = [
        1 1
        0 0.9
    ]
    B = [1; 0.5]

    nbr_state = 2
    nbr_input = 1

    sys = proceed_system(
        A,
        B, 
        nbr_state,
        nbr_input,
        "discrete";
    )

    @test typeof(sys) == LinearControlDiscreteSystem{
        Float64,
        Matrix{Float64},
        Matrix{Float64},
    }

    @test sys.A == A
    @test sys.B == [1.0; 0.5;;]
end

@testset "Non linear user system with input and state constraints" begin

    model_origin = "user"
    f = function in(x)
        cos(x)
    end

    u_cons = [-1 1]
    x_cons = [-1 1]

    nbr_state = 1
    nbr_input = 1

    sys = proceed_system(
        f,
        nbr_state,
        nbr_input,
        "discrete";
        input_constraint = u_cons,
        state_constraint = x_cons,
    )

    @test typeof(sys) == ConstrainedBlackBoxControlDiscreteSystem{
        typeof(in),
        Hyperrectangle{Float64,Vector{Float64},Vector{Float64}},
        Hyperrectangle{Float64,Vector{Float64},Vector{Float64}},
    }
    @test sys.f == f
    @test LazySets.low(sys.X) == [-1.0]
    @test LazySets.high(sys.X) == [1.0]
    @test LazySets.low(sys.U) == [-1.0]
    @test LazySets.high(sys.U) == [1.0]
end

@testset "Non linear user system with input constraints" begin

    model_origin = "user"
    f = function in(x)
        cos(x)
    end

    u_cons = [-1 1]

    nbr_state = 1
    nbr_input = 1

    sys = proceed_system(
        f,
        nbr_state,
        nbr_input,
        "discrete";
        input_constraint = u_cons,
    )

    @test typeof(sys) == ConstrainedBlackBoxControlDiscreteSystem{
        typeof(in),
        Hyperrectangle{Float64,Vector{Float64},Vector{Float64}},
        Hyperrectangle{Float64,Vector{Float64},Vector{Float64}},
    }
    @test sys.f == f
    @test LazySets.low(sys.X) == [0.0]
    @test LazySets.high(sys.X) == [0.0]
    @test LazySets.low(sys.U) == [-1.0]
    @test LazySets.high(sys.U) == [1.0]
end

@testset "Non linear user system without constraints" begin

    f = function in(x)
        cos(x)
    end

    nbr_state = 1
    nbr_input = 1

    sys = proceed_system(
        f,
        nbr_state,
        nbr_input,
        "discrete";
    )

    @test typeof(sys) == BlackBoxControlDiscreteSystem{
        typeof(in),
    }

    @test sys.f == f

end

### Continuous ### 

@testset "Linear user system with state and input constraints" begin

    model_origin = "user"
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

    @test typeof(sys) == ConstrainedLinearControlContinuousSystem{
        Float64,
        Matrix{Float64},
        Matrix{Float64},
        Hyperrectangle{Float64,Vector{Float64},Vector{Float64}},
        Hyperrectangle{Float64,Vector{Float64},Vector{Float64}},
    }
    @test sys.A == A
    @test sys.B == [1.0; 0.5;;]
    @test LazySets.low(sys.X) == [-5.0, -5.0]
    @test LazySets.high(sys.X) == [5.0, 5.0]
    @test LazySets.low(sys.U) == [-1.0]
    @test LazySets.high(sys.U) == [1.0]
end

@testset "Linear user system continuous with input constraints" begin

    model_origin = "user"
    A = [
        1 1
        0 0.9
    ]
    B = [1; 0.5]
    nbr_state = 2
    nbr_input = 1

    u_cons = [-1 1]

    sys = proceed_system(
        A,
        B, 
        nbr_state,
        nbr_input,
        "continuous";
        input_constraint = u_cons,
    )

    @test typeof(sys) == ConstrainedLinearControlContinuousSystem{
        Float64,
        Matrix{Float64},
        Matrix{Float64},
        Hyperrectangle{Float64,Vector{Float64},Vector{Float64}},
        Hyperrectangle{Float64,Vector{Float64},Vector{Float64}},
    }
    @test sys.A == A
    @test sys.B == [1.0; 0.5;;]
    @test LazySets.low(sys.X) == [0.0, 0.0]
    @test LazySets.high(sys.X) == [0.0, 0.0]
    @test LazySets.low(sys.U) == [-1.0]
    @test LazySets.high(sys.U) == [1.0]
end

@testset "Linear user system continuous without constraints" begin

    A = [
        1 1
        0 0.9
    ]
    B = [1; 0.5]
    nbr_state = 2
    nbr_input = 1


    sys = proceed_system(
        A,
        B, 
        nbr_state,
        nbr_input,
        "continuous";
    )

    @test typeof(sys) == LinearControlContinuousSystem{
        Float64,
        Matrix{Float64},
        Matrix{Float64},
    }

    @test sys.A == A
    @test sys.B == [1.0; 0.5;;]
end

@testset "Non linear user system with input and state constraints" begin

    model_origin = "user"
    f = function in(x)
        cos(x)
    end

    u_cons = [-1 1]
    x_cons = [-1 1]

    nbr_state = 1
    nbr_input = 1

    sys = proceed_system(
        f,
        nbr_state,
        nbr_input,
        "continuous";
        input_constraint = u_cons,
        state_constraint = x_cons,
    )

    @test typeof(sys) == ConstrainedBlackBoxControlContinuousSystem{
        typeof(in),
        Hyperrectangle{Float64,Vector{Float64},Vector{Float64}},
        Hyperrectangle{Float64,Vector{Float64},Vector{Float64}},
    }
    @test sys.f == f
    @test LazySets.low(sys.X) == [-1.0]
    @test LazySets.high(sys.X) == [1.0]
    @test LazySets.low(sys.U) == [-1.0]
    @test LazySets.high(sys.U) == [1.0]
end

@testset "Non linear user system with input constraints" begin

    model_origin = "user"
    f = function in(x)
        cos(x)
    end

    u_cons = [-1 1]

    nbr_state = 1
    nbr_input = 1

    sys = proceed_system(
        f,
        nbr_state,
        nbr_input,
        "continuous";
        input_constraint = u_cons,
    )

    @test typeof(sys) == ConstrainedBlackBoxControlContinuousSystem{
        typeof(in),
        Hyperrectangle{Float64,Vector{Float64},Vector{Float64}},
        Hyperrectangle{Float64,Vector{Float64},Vector{Float64}},
    }
    @test sys.f == f
    @test LazySets.low(sys.X) == [0.0]
    @test LazySets.high(sys.X) == [0.0]
    @test LazySets.low(sys.U) == [-1.0]
    @test LazySets.high(sys.U) == [1.0]
end

@testset "Non linear user system without constraints" begin

    f = function in(x)
        cos(x)
    end

    nbr_state = 1
    nbr_input = 1

    sys = proceed_system(
        f,
        nbr_state,
        nbr_input,
        "continuous",    
    )

    @test typeof(sys) == BlackBoxControlContinuousSystem{
        typeof(in),
    }

    @test sys.f == f
end

### Model from identification ###
# Not yet necessarely #

#=

@testset "Linear discrete identification system with input constraints" begin

    model_origin = "identification"

    linear_regressor_machine = machine("./models_saved/linear_regressor_train_result.jls")

    u_cons = [
        -1 1
        -1 1
    ]

    sys = proceed_system(
        "discrete",
        model_origin;
        f = linear_regressor_machine,
        input_constraint = u_cons,
        nbr_state = 4,
        nbr_input = 2,
    )

    @test typeof(sys) == ConstrainedLinearControlDiscreteSystem{
        Float32,
        Matrix{Float32},
        Matrix{Float32},
        Hyperrectangle{Float64,Vector{Float64},Vector{Float64}},
        Hyperrectangle{Float64,Vector{Float64},Vector{Float64}},
    }

    @test LazySets.low(sys.X) == [0.0, 0.0, 0.0, 0.0]
    @test LazySets.high(sys.X) == [0.0, 0.0, 0.0, 0.0]
    @test LazySets.low(sys.U) == [-1.0, -1.0]
    @test LazySets.high(sys.U) == [1.0, 1.0]

end

@testset "Non linear discrete identification system with input and state constraints" begin

    model_origin = "identification"

    densenet = machine("./models_saved/densenet_train_result.jls")

    u_cons = [
        -1 1
        -1 1
    ]
    x_cons = [
        -5 5
        -5 5
        -5 5
        -5 5
    ]

    sys = proceed_system(
        "discrete",
        model_origin;
        f = densenet,
        input_constraint = u_cons,
        state_constraint = x_cons,
        nbr_state = 4,
        nbr_input = 2,
    )

    @test typeof(sys) == ConstrainedBlackBoxControlDiscreteSystem{
        Chain{
            Tuple{
                Dense{typeof(identity),Matrix{Float32},Bool},
                Chain{
                    Tuple{
                        SkipConnection{
                            Chain{
                                Tuple{Dense{typeof(relu),Matrix{Float32},Vector{Float32}}},
                            },
                            typeof(vcat),
                        },
                        SkipConnection{
                            Chain{
                                Tuple{Dense{typeof(relu),Matrix{Float32},Vector{Float32}}},
                            },
                            typeof(vcat),
                        },
                        SkipConnection{
                            Chain{
                                Tuple{Dense{typeof(relu),Matrix{Float32},Vector{Float32}}},
                            },
                            typeof(vcat),
                        },
                    },
                },
                Dense{typeof(identity),Matrix{Float32},Bool},
            },
        },
        Hyperrectangle{Float64,Vector{Float64},Vector{Float64}},
        Hyperrectangle{Float64,Vector{Float64},Vector{Float64}},
    }

    @test LazySets.low(sys.X) == [-5.0, -5.0, -5.0, -5.0]
    @test LazySets.high(sys.X) == [5.0, 5.0, 5.0, 5.0]
    @test LazySets.low(sys.U) == [-1.0, -1.0]
    @test LazySets.high(sys.U) == [1.0, 1.0]

end

@testset "Linear continuous identification system with input constraints" begin

    model_origin = "identification"

    linear_regressor_machine = machine("./models_saved/linear_regressor_train_result.jls")

    u_cons = [
        -1 1
        -1 1
    ]

    sys = proceed_system(
        "continuous",
        model_origin;
        f = linear_regressor_machine,
        input_constraint = u_cons,
        nbr_state = 4,
        nbr_input = 2,
    )

    @test typeof(sys) == ConstrainedLinearControlContinuousSystem{
        Float32,
        Matrix{Float32},
        Matrix{Float32},
        Hyperrectangle{Float64,Vector{Float64},Vector{Float64}},
        Hyperrectangle{Float64,Vector{Float64},Vector{Float64}},
    }

    @test LazySets.low(sys.X) == [0.0, 0.0, 0.0, 0.0]
    @test LazySets.high(sys.X) == [0.0, 0.0, 0.0, 0.0]
    @test LazySets.low(sys.U) == [-1.0, -1.0]
    @test LazySets.high(sys.U) == [1.0, 1.0]

end

@testset "Non linear continuous identification system with input and state constraints" begin

    model_origin = "identification"

    densenet = machine("./models_saved/densenet_train_result.jls")

    u_cons = [
        -1 1
        -1 1
    ]
    x_cons = [
        -5 5
        -5 5
        -5 5
        -5 5
    ]

    sys = proceed_system(
        "continuous",
        model_origin;
        f = densenet,
        input_constraint = u_cons,
        state_constraint = x_cons,
        nbr_state = 4,
        nbr_input = 2,
    )

    @test typeof(sys) == ConstrainedBlackBoxControlContinuousSystem{
        Chain{
            Tuple{
                Dense{typeof(identity),Matrix{Float32},Bool},
                Chain{
                    Tuple{
                        SkipConnection{
                            Chain{
                                Tuple{Dense{typeof(relu),Matrix{Float32},Vector{Float32}}},
                            },
                            typeof(vcat),
                        },
                        SkipConnection{
                            Chain{
                                Tuple{Dense{typeof(relu),Matrix{Float32},Vector{Float32}}},
                            },
                            typeof(vcat),
                        },
                        SkipConnection{
                            Chain{
                                Tuple{Dense{typeof(relu),Matrix{Float32},Vector{Float32}}},
                            },
                            typeof(vcat),
                        },
                    },
                },
                Dense{typeof(identity),Matrix{Float32},Bool},
            },
        },
        Hyperrectangle{Float64,Vector{Float64},Vector{Float64}},
        Hyperrectangle{Float64,Vector{Float64},Vector{Float64}},
    }

    @test LazySets.low(sys.X) == [-5.0, -5.0, -5.0, -5.0]
    @test LazySets.high(sys.X) == [5.0, 5.0, 5.0, 5.0]
    @test LazySets.low(sys.U) == [-1.0, -1.0]
    @test LazySets.high(sys.U) == [1.0, 1.0]

end
=#
end
