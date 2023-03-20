# Copyright (c) 2022: Pierre Blaud and contributors
########################################################
# This Source Code Form is subject to the terms of the #
# Mozilla Public License, v. 2.0. If a copy of the MPL #
# was not distributed with this file,  				   #
# You can obtain one at https://mozilla.org/MPL/2.0/.  #
########################################################

"""
_controller_system_design
A function for design the system (model and constraints) with MathematicalSystems from non linear model.
"""
function _controller_system_design(model::DiscreteNonLinearModel)

    # Set the system
    return MathematicalSystems.BlackBoxControlDiscreteSystem(
        model.f,
        model.nbr_state,
        model.nbr_input,
    )
end

"""
_controller_system_design
A function for design the system (model and constraints) with MathematicalSystems from non linear model.
"""
function _controller_system_design(model::DiscreteNonLinearModel, input_constraint)

    # Set constraints with Lazy Sets
    lower_state_constraints = zeros(model.nbr_state, 1)[:, 1]
    higher_state_constraints = zeros(model.nbr_state, 1)[:, 1]
    lower_input_constraints = input_constraint[:, 1]
    higher_input_constraints = input_constraint[:, 2]

    X = LazySets.Hyperrectangle(
        low = lower_state_constraints,
        high = higher_state_constraints,
    )
    U = LazySets.Hyperrectangle(
        low = lower_input_constraints,
        high = higher_input_constraints,
    )

    # Set the system
    return MathematicalSystems.ConstrainedBlackBoxControlDiscreteSystem(
        model.f,
        model.nbr_state,
        model.nbr_input,
        X,
        U,
    )
end

"""
_controller_system_design
A function for design the system (model and constraints) with MathematicalSystems from non linear model.
"""
function _controller_system_design(
    model::DiscreteNonLinearModel,
    input_constraint,
    state_constraint,
)

    # Set constraints with Lazy Sets
    lower_state_constraints = state_constraint[:, 1]
    higher_state_constraints = state_constraint[:, 2]
    lower_input_constraints = input_constraint[:, 1]
    higher_input_constraints = input_constraint[:, 2]

    X = LazySets.Hyperrectangle(
        low = lower_state_constraints,
        high = higher_state_constraints,
    )
    U = LazySets.Hyperrectangle(
        low = lower_input_constraints,
        high = higher_input_constraints,
    )

    # Set the system
    return MathematicalSystems.ConstrainedBlackBoxControlDiscreteSystem(
        model.f,
        model.nbr_state,
        model.nbr_input,
        X,
        U,
    )
end

"""
_controller_system_design
A function for design the system (model and constraints) with MathematicalSystems from linear model.
"""
function _controller_system_design(model::DiscreteLinearModel)

    # Set the system
    return MathematicalSystems.@system x⁺ = model.A * x + model.B * u
end

"""
_controller_system_design
A function for design the system (model and constraints) with MathematicalSystems from linear model.
"""
function _controller_system_design(model::DiscreteLinearModel, input_constraint)

    # Set constraints with Lazy Sets
    lower_state_constraints = zeros(model.nbr_state, 1)[:, 1]
    higher_state_constraints = zeros(model.nbr_state, 1)[:, 1]
    lower_input_constraints = input_constraint[:, 1]
    higher_input_constraints = input_constraint[:, 2]

    X = LazySets.Hyperrectangle(
        low = lower_state_constraints,
        high = higher_state_constraints,
    )
    U = LazySets.Hyperrectangle(
        low = lower_input_constraints,
        high = higher_input_constraints,
    )

    # Set the system
    return MathematicalSystems.@system x⁺ = model.A * x + model.B * u x ∈ X u ∈ U
end

"""
_controller_system_design
A function for design the system (model and constraints) with MathematicalSystems from linear model.
"""
function _controller_system_design(
    model::DiscreteLinearModel,
    input_constraint,
    state_constraint,
)

    # Set constraints with Lazy Sets
    lower_state_constraints = state_constraint[:, 1]
    higher_state_constraints = state_constraint[:, 2]
    lower_input_constraints = input_constraint[:, 1]
    higher_input_constraints = input_constraint[:, 2]

    X = LazySets.Hyperrectangle(
        low = lower_state_constraints,
        high = higher_state_constraints,
    )
    U = LazySets.Hyperrectangle(
        low = lower_input_constraints,
        high = higher_input_constraints,
    )

    # Set the system
    return MathematicalSystems.@system x⁺ = model.A * x + model.B * u x ∈ X u ∈ U
    #MathematicalSystems.ConstrainedLinearControlDiscreteSystem(model.A, model.B, X, U)
end

"""
_controller_system_design
A function for design the system (model and constraints) with MathematicalSystems from non linear model.
"""
function _controller_system_design(model::ContinuousNonLinearModel)

    # Set the system
    return MathematicalSystems.BlackBoxControlContinuousSystem(
        model.f,
        model.nbr_state,
        model.nbr_input,
    )
end

"""
_controller_system_design
A function for design the system (model and constraints) with MathematicalSystems from non linear model.
"""
function _controller_system_design(model::ContinuousNonLinearModel, input_constraint)

    # Set constraints with Lazy Sets
    lower_state_constraints = zeros(model.nbr_state, 1)[:, 1]
    higher_state_constraints = zeros(model.nbr_state, 1)[:, 1]
    lower_input_constraints = input_constraint[:, 1]
    higher_input_constraints = input_constraint[:, 2]

    X = LazySets.Hyperrectangle(
        low = lower_state_constraints,
        high = higher_state_constraints,
    )
    U = LazySets.Hyperrectangle(
        low = lower_input_constraints,
        high = higher_input_constraints,
    )

    # Set the system
    return MathematicalSystems.ConstrainedBlackBoxControlContinuousSystem(
        model.f,
        model.nbr_state,
        model.nbr_input,
        X,
        U,
    )
end

"""
_controller_system_design
A function for design the system (model and constraints) with MathematicalSystems from non linear model.
"""
function _controller_system_design(
    model::ContinuousNonLinearModel,
    input_constraint,
    state_constraint,
)

    # Set constraints with Lazy Sets
    lower_state_constraints = state_constraint[:, 1]
    higher_state_constraints = state_constraint[:, 2]
    lower_input_constraints = input_constraint[:, 1]
    higher_input_constraints = input_constraint[:, 2]

    X = LazySets.Hyperrectangle(
        low = lower_state_constraints,
        high = higher_state_constraints,
    )
    U = LazySets.Hyperrectangle(
        low = lower_input_constraints,
        high = higher_input_constraints,
    )

    # Set the system
    return MathematicalSystems.ConstrainedBlackBoxControlContinuousSystem(
        model.f,
        model.nbr_state,
        model.nbr_input,
        X,
        U,
    )
end

"""
_controller_system_design
A function for design the system (model and constraints) with MathematicalSystems from linear model.
"""
function _controller_system_design(model::ContinuousLinearModel)

    # Set the system
    return MathematicalSystems.@system x' = model.A * x + model.B * u
end

"""
_controller_system_design
A function for design the system (model and constraints) with MathematicalSystems from linear model.
"""
function _controller_system_design(model::ContinuousLinearModel, input_constraint)

    # Set constraints with Lazy Sets
    lower_state_constraints = zeros(model.nbr_state, 1)[:, 1]
    higher_state_constraints = zeros(model.nbr_state, 1)[:, 1]
    lower_input_constraints = input_constraint[:, 1]
    higher_input_constraints = input_constraint[:, 2]

    X = LazySets.Hyperrectangle(
        low = lower_state_constraints,
        high = higher_state_constraints,
    )
    U = LazySets.Hyperrectangle(
        low = lower_input_constraints,
        high = higher_input_constraints,
    )

    # Set the system
    return MathematicalSystems.@system x' = model.A * x + model.B * u x ∈ X u ∈ U
end

"""
_controller_system_design
A function for design the system (model and constraints) with MathematicalSystems from linear model.
"""
function _controller_system_design(
    model::ContinuousLinearModel,
    input_constraint,
    state_constraint,
)

    # Set constraints with Lazy Sets
    lower_state_constraints = state_constraint[:, 1]
    higher_state_constraints = state_constraint[:, 2]
    lower_input_constraints = input_constraint[:, 1]
    higher_input_constraints = input_constraint[:, 2]

    X = LazySets.Hyperrectangle(
        low = lower_state_constraints,
        high = higher_state_constraints,
    )
    U = LazySets.Hyperrectangle(
        low = lower_input_constraints,
        high = higher_input_constraints,
    )

    # Set the system
    return MathematicalSystems.@system x' = model.A * x + model.B * u x ∈ X u ∈ U
    #MathematicalSystems.ConstrainedLinearControlDiscreteSystem(model.A, model.B, X, U)
end
