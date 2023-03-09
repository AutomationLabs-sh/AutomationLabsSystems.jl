# Copyright (c) 2022: Pierre Blaud and contributors
########################################################
# This Source Code Form is subject to the terms of the #
# Mozilla Public License, v. 2.0. If a copy of the MPL #
# was not distributed with this file,  				   #
# You can obtain one at https://mozilla.org/MPL/2.0/.  #
########################################################

"""
    proceed_system
A function for system creation from linear discrete or continuous model.

The following variables are mendatories:
* `A`: a state matrix.
* `B`: a input matrix.
* `nbr_state`: the state number.
* `nbr_input`: the input number.
* `variation`: continuous or discrete variation.

It is possible to define optional variables kws.
"""
function proceed_system(A, B, nbr_state, nbr_input, variation; kws_...)

    # Get argument kws
    dict_kws = Dict{Symbol,Any}(kws_)
    kws = get(dict_kws, :kws, kws_)

    if variation == "continuous"
        model = ContinuousLinearModel(A, B, nbr_state, nbr_input)
    elseif variation == "discrete"
        model = DiscreteLinearModel(A, B, nbr_state, nbr_input)
    end

    # Set the system with constraints

    if haskey(kws, :input_constraint) == true && haskey(kws, :state_constraint) == true
        # System with input and state constraints
        return system =
            _controller_system_design(model, kws[:input_constraint], kws[:state_constraint])

    elseif haskey(kws, :input_constraint) == true && haskey(kws, :state_constraint) == false
        # System with input and no state constraints
        return system = _controller_system_design(model, kws[:input_constraint])

    elseif haskey(kws, :input_constraint) == false &&
           haskey(kws, :state_constraint) == false
        return system = _controller_system_design(model)
    else
        # Issues
        @error "There are no input nor state constraints, at least input constaint is mandatory"
    end
end

"""
    proceed_system
A function for system creation from non-linear discrete or continuous model.

The following variables are mendatories:
* `f`: a non-linear function.
* `nbr_state`: the state number.
* `nbr_input`: the input number.
* `variation`: continuous or discrete variation.

It is possible to define optional variables kws.
"""
function proceed_system(f, nbr_state, nbr_input, variation; kws_...)

    # Get argument kws
    dict_kws = Dict{Symbol,Any}(kws_)
    kws = get(dict_kws, :kws, kws_)

    if variation == "continuous"
        model = ContinuousNonLinearModel(f, nbr_state, nbr_input)
    elseif variation == "discrete"
        model = DiscreteNonLinearModel(f, nbr_state, nbr_input)
    end

    # Set the system with constraints

    if haskey(kws, :input_constraint) == true && haskey(kws, :state_constraint) == true
        # System with input and state constraints
        return system =
            _controller_system_design(model, kws[:input_constraint], kws[:state_constraint])

    elseif haskey(kws, :input_constraint) == true && haskey(kws, :state_constraint) == false
        # System with input and no state constraints
        return system = _controller_system_design(model, kws[:input_constraint])

    elseif haskey(kws, :input_constraint) == false &&
           haskey(kws, :state_constraint) == false
        return system = _controller_system_design(model)
    else
        # Issues
        @error "There are no input nor state constraints, at least input constraint is mandatory"
    end
end

"""
    proceed_system_linearization
Function that linearises a system from MathematicalSystems at state and input references. 
The function uses ForwardDiff package and the jacobian function.

** Required fields **
* `system`: the mathematital system that as in it the julia non-linear function `f`.
* `state`: references state point.
* `input`: references input point.
"""
function proceed_system_linearization(
    system::MathematicalSystems.BlackBoxControlContinuousSystem,
    state::Vector{Float64},
    input::Vector{Float64},
)

    #Linearization to get A and B at references values (state and input)
    VectorXU = vcat(state, input)
    JacobianMatrix = ForwardDiff.jacobian(system.f, VectorXU)
    A_sys = JacobianMatrix[1:system.statedim, 1:system.statedim]
    B_sys = JacobianMatrix[1:system.statedim, system.statedim+1:end]

    return MathematicalSystems.@system x' = A_sys * x + B_sys * u 
end

function proceed_system_linearization(
    system::MathematicalSystems.BlackBoxControlDiscreteSystem,
    state::Vector{Float64},
    input::Vector{Float64},
)

    #Linearization to get A and B at references values (state and input)
    VectorXU = vcat(state, input)
    JacobianMatrix = ForwardDiff.jacobian(system.f, VectorXU)
    A_sys = JacobianMatrix[1:system.statedim, 1:system.statedim]
    B_sys = JacobianMatrix[1:system.statedim, system.statedim+1:end]

    return MathematicalSystems.@system x' = A_sys * x + B_sys * u 
end

function proceed_system_linearization(
    system::MathematicalSystems.ConstrainedBlackBoxControlContinuousSystem,
    state::Vector{Float64},
    input::Vector{Float64},
)

    #Linearization to get A and B at references values (state and input)
    VectorXU = vcat(state, input)
    JacobianMatrix = ForwardDiff.jacobian(system.f, VectorXU)
    A_sys = JacobianMatrix[1:system.statedim, 1:system.statedim]
    B_sys = JacobianMatrix[1:system.statedim, system.statedim+1:end]

    return MathematicalSystems.@system x' = A_sys * x + B_sys * u x ∈ system.X u ∈ system.U
end

function proceed_system_linearization(
    system::MathematicalSystems.ConstrainedBlackBoxControlDiscreteSystem,
    state::Vector{Float64},
    input::Vector{Float64},
)

    #Linearization to get A and B at references values (state and input)
    VectorXU = vcat(state, input)
    JacobianMatrix = ForwardDiff.jacobian(system.f, VectorXU)
    A_sys = JacobianMatrix[1:system.statedim, 1:system.statedim]
    B_sys = JacobianMatrix[1:system.statedim, system.statedim+1:end]

    return MathematicalSystems.@system x⁺ = A_sys * x + B_sys * u x ∈ system.X u ∈ system.U
end

function proceed_system_linearization(
    system::MathematicalSystems.ConstrainedLinearControlDiscreteSystem,
    state::Vector{Float64},
    input::Vector{Float64},
)

    return system
end

function proceed_system_linearization(
    system::MathematicalSystems.ConstrainedLinearControlContinuousSystem,
    state::Vector{Float64},
    input::Vector{Float64},
)

    return system
end


"""
    proceed_system_model_evaluation
Function that return the types of the model inside the systems from AutomationLabsIdentification.

** Required fields **
* `system`: the mathematital system that as in it the julia non-linear function `f` from AutomationLabsIdentification.
"""
function proceed_system_model_evaluation(
    system::MathematicalSystems.ConstrainedBlackBoxControlDiscreteSystem,
)

    if typeof(system.f) == Flux.Chain{
        Tuple{
            Flux.Dense{typeof(identity),Matrix{Float32},Bool},
            Flux.Chain{
                NTuple{4,Flux.Dense{typeof(NNlib.relu),Matrix{Float32},Vector{Float32}}},
            },
            Flux.Dense{typeof(identity),Matrix{Float32},Bool},
        },
    }
        model_type = AutomationLabsIdentification.Fnn()

    elseif typeof(system.f) == Flux.Chain{
        Tuple{
            AutomationLabsIdentification.DenseIcnn{typeof(identity),Matrix{Float32},Bool},
            Flux.Chain{
                Tuple{
                    AutomationLabsIdentification.DenseIcnn{
                        typeof(NNlib.relu),
                        Matrix{Float32},
                        Vector{Float32},
                    },
                    AutomationLabsIdentification.DenseIcnn{
                        typeof(NNlib.relu),
                        Matrix{Float32},
                        Vector{Float32},
                    },
                },
            },
            AutomationLabsIdentification.DenseIcnn{typeof(identity),Matrix{Float32},Bool},
        },
    }
        model_type = AutomationLabsIdentification.Icnn()

    elseif typeof(system.f) == Flux.Chain{
        Tuple{
            Flux.Dense{typeof(identity),Matrix{Float32},Bool},
            Flux.Chain{
                Tuple{
                    Flux.SkipConnection{
                        Flux.Chain{
                            Tuple{
                                Flux.Dense{
                                    typeof(NNlib.relu),
                                    Matrix{Float32},
                                    Vector{Float32},
                                },
                            },
                        },
                        typeof(+),
                    },
                },
            },
            Flux.Dense{typeof(identity),Matrix{Float32},Bool},
        },
    }
        model_type = AutomationLabsIdentification.ResNet()

    elseif typeof(system.f) == Flux.Chain{
        Tuple{
            Flux.Dense{typeof(identity),Matrix{Float32},Bool},
            Flux.Chain{
                Tuple{
                    Flux.SkipConnection{
                        Flux.Chain{
                            Tuple{
                                Flux.Dense{
                                    typeof(NNlib.relu),
                                    Matrix{Float32},
                                    Vector{Float32},
                                },
                            },
                        },
                        typeof(vcat),
                    },
                },
            },
            Flux.Dense{typeof(identity),Matrix{Float32},Bool},
        },
    }
        model_type = AutomationLabsIdentification.DenseNet()

    elseif typeof(system.f) == Flux.Chain{
        Tuple{
            Flux.Dense{typeof(identity),Matrix{Float32},Bool},
            Flux.Chain{
                NTuple{
                    4,
                    Flux.SkipConnection{
                        Flux.Parallel{
                            typeof(+),
                            Tuple{
                                Flux.Dense{
                                    typeof(NNlib.relu),
                                    Matrix{Float32},
                                    Vector{Float32},
                                },
                                Flux.Chain{
                                    Tuple{
                                        Flux.Dense{
                                            typeof(NNlib.relu),
                                            Matrix{Float32},
                                            Vector{Float32},
                                        },
                                        Flux.Dense{
                                            typeof(NNlib.relu),
                                            Matrix{Float32},
                                            Vector{Float32},
                                        },
                                    },
                                },
                            },
                        },
                        typeof(+),
                    },
                },
            },
            Flux.Dense{typeof(identity),Matrix{Float32},Bool},
        },
    }
        model_type = AutomationLabsIdentification.PolyNet()

    elseif typeof(system.f) == Function
        model_type = typeof(system.f)

    end

    return model_type
end

"""
    proceed_system_discretization
Function that linearises a system from MathematicalSystems at state and input references. 
The function uses ForwardDiff package and the jacobian function.

** Required fields **
* `system`: the continuous mathematital system that as in it the julia linear or non-linear function `f`.
* `sample_time`: the sample time for discretization.
"""
function proceed_system_discretization(
    system::MathematicalSystems.ConstrainedLinearControlContinuousSystem,
    sample_time,
)

    D = 0
    C = ones(size(system.A, 1), size(system.A, 2))
    sys_c = ControlSystems.ss(system.A, system.B, C, D)
    sys_d = ControlSystems.c2d(sys_c, sample_time)
    A_sys = sys_d.A
    B_sys = sys_d.B

    return MathematicalSystems.@system x⁺ = A_sys * x + B_sys * u x ∈ system.X u ∈ system.U
end
