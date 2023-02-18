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
function _controller_system_design(
    model::DiscreteNonLinearModel,
    input_constraint,
)

    # Set constraints with Lazy Sets
    lower_state_constraints = zeros(model.nbr_state, 1)[:, 1]
    higher_state_constraints = zeros(model.nbr_state, 1)[:, 1]
    lower_input_constraints = input_constraint[:, 1]
    higher_input_constraints = input_constraint[:, 2]

    X = LazySets.Hyperrectangle(low = lower_state_constraints, high = higher_state_constraints,)
    U = LazySets.Hyperrectangle(low = lower_input_constraints, high = higher_input_constraints)
  
    # Set the system
    return MathematicalSystems.ConstrainedBlackBoxControlDiscreteSystem(model.f, model.nbr_state, model.nbr_input, X, U)
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

    X = LazySets.Hyperrectangle(low = lower_state_constraints, high = higher_state_constraints,)
    U = LazySets.Hyperrectangle(low = lower_input_constraints, high = higher_input_constraints)
  
    # Set the system
    return MathematicalSystems.ConstrainedBlackBoxControlDiscreteSystem(model.f, model.nbr_state, model.nbr_input, X, U)
end

"""
_controller_system_design
A function for design the system (model and constraints) with MathematicalSystems from linear model.
"""
function _controller_system_design(
    model::DiscreteLinearModel,
    input_constraint,
)

    # Set constraints with Lazy Sets
    lower_state_constraints = zeros(model.nbr_state, 1)[:, 1]
    higher_state_constraints = zeros(model.nbr_state, 1)[:, 1]
    lower_input_constraints = input_constraint[:, 1]
    higher_input_constraints = input_constraint[:, 2]

    X = LazySets.Hyperrectangle(low = lower_state_constraints, high = higher_state_constraints,)
    U = LazySets.Hyperrectangle(low = lower_input_constraints, high = higher_input_constraints)
  
    # Set the system
    return MathematicalSystems.@system x⁺ = model.A*x + model.B*u x∈X u∈U
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

    X = LazySets.Hyperrectangle(low = lower_state_constraints, high = higher_state_constraints,)
    U = LazySets.Hyperrectangle(low = lower_input_constraints, high = higher_input_constraints)
  
    # Set the system
    return MathematicalSystems.@system x⁺ = model.A*x + model.B*u x∈X u∈U 
    #MathematicalSystems.ConstrainedLinearControlDiscreteSystem(model.A, model.B, X, U)
end

"""
_controller_system_design
A function for design the system (model and constraints) with MathematicalSystems from non linear model.
"""
function _controller_system_design(
    model::ContinuousNonLinearModel,
    input_constraint,
)

    # Set constraints with Lazy Sets
    lower_state_constraints = zeros(model.nbr_state, 1)[:, 1]
    higher_state_constraints = zeros(model.nbr_state, 1)[:, 1]
    lower_input_constraints = input_constraint[:, 1]
    higher_input_constraints = input_constraint[:, 2]

    X = LazySets.Hyperrectangle(low = lower_state_constraints, high = higher_state_constraints,)
    U = LazySets.Hyperrectangle(low = lower_input_constraints, high = higher_input_constraints)
  
    # Set the system
    return MathematicalSystems.ConstrainedBlackBoxControlContinuousSystem(model.f, model.nbr_state, model.nbr_input, X, U)
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

    X = LazySets.Hyperrectangle(low = lower_state_constraints, high = higher_state_constraints,)
    U = LazySets.Hyperrectangle(low = lower_input_constraints, high = higher_input_constraints)
  
    # Set the system
    return MathematicalSystems.ConstrainedBlackBoxControlContinuousSystem(model.f, model.nbr_state, model.nbr_input, X, U)
end

"""
_controller_system_design
A function for design the system (model and constraints) with MathematicalSystems from linear model.
"""
function _controller_system_design(
    model::ContinuousLinearModel,
    input_constraint,
)

    # Set constraints with Lazy Sets
    lower_state_constraints = zeros(model.nbr_state, 1)[:, 1]
    higher_state_constraints = zeros(model.nbr_state, 1)[:, 1]
    lower_input_constraints = input_constraint[:, 1]
    higher_input_constraints = input_constraint[:, 2]

    X = LazySets.Hyperrectangle(low = lower_state_constraints, high = higher_state_constraints,)
    U = LazySets.Hyperrectangle(low = lower_input_constraints, high = higher_input_constraints)
  
    # Set the system
    return MathematicalSystems.@system x' = model.A*x + model.B*u x∈X u∈U
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

    X = LazySets.Hyperrectangle(low = lower_state_constraints, high = higher_state_constraints,)
    U = LazySets.Hyperrectangle(low = lower_input_constraints, high = higher_input_constraints)
  
    # Set the system
    return MathematicalSystems.@system x' = model.A*x + model.B*u x∈X u∈U 
    #MathematicalSystems.ConstrainedLinearControlDiscreteSystem(model.A, model.B, X, U)
end






#=

### First case non linear MathematicalSystems from AutomationLabsIdentification without state constraint### 
"""
_controller_system_design
A function for design the system (model and constraints) with MathematicalSystems for system from a tune model with mlj.
"""
function _controller_system_design(
    model_type::Union{AutomationLabsIdentification.Fnn, 
                 AutomationLabsIdentification.Icnn,
                 AutomationLabsIdentification.ResNet, 
                 AutomationLabsIdentification.DenseNet, 
                 AutomationLabsIdentification.Rbf, 
                 AutomationLabsIdentification.PolyNet, 
                 AutomationLabsIdentification.NeuralNetODE_type1, 
                 AutomationLabsIdentification.NeuralNetODE_type2},
    machine_mlj,
    input_constraint,
    nbr_state,
    nbr_input
)

    # Set constraints with Lazy Sets
    lower_state_constraints = zeros(nbr_state, 1)[:, 1]
    higher_state_constraints = zeros(nbr_state, 1)[:, 1]
    lower_input_constraints = input_constraint[:, 1]
    higher_input_constraints = input_constraint[:, 2]

    x_cons = LazySets.Hyperrectangle(low = lower_state_constraints, high = higher_state_constraints,)
    u_cons = LazySets.Hyperrectangle(low = lower_input_constraints, high = higher_input_constraints)
     
    # Extract best model from the machine
    f_model = MLJ.fitted_params(MLJ.fitted_params(machine_mlj).machine).best_fitted_params[1]

    # Set the system
    return system = MathematicalSystems.ConstrainedBlackBoxControlDiscreteSystem(f_model, nbr_state, nbr_input, x_cons, u_cons)
end

### Second case non linear MathematicalSystems from AutomationLabsIdentification with state constraint### 

"""
_controller_system_design
A function for design the system (model and constraints) with MathematicalSystems for system from a tune model with mlj.
"""
function _controller_system_design(
    model_type::Union{AutomationLabsIdentification.Fnn, 
                 AutomationLabsIdentification.Icnn,
                 AutomationLabsIdentification.ResNet, 
                 AutomationLabsIdentification.DenseNet, 
                 AutomationLabsIdentification.Rbf, 
                 AutomationLabsIdentification.PolyNet, 
                 AutomationLabsIdentification.NeuralNetODE_type1, 
                 AutomationLabsIdentification.NeuralNetODE_type2},
    machine_mlj,
    input_constraint,
    state_constraint,
    nbr_state,
    nbr_input
)

    # Set constraints with Lazy Sets
    lower_state_constraints = state_constraint[:, 1]
    higher_state_constraints = state_constraint[:, 2]
    lower_input_constraints = input_constraint[:, 1]
    higher_input_constraints = input_constraint[:, 2]

    x_cons = LazySets.Hyperrectangle(low = lower_state_constraints, high = higher_state_constraints,)
    u_cons = LazySets.Hyperrectangle(low = lower_input_constraints, high = higher_input_constraints)
     
    # Extract best model from the machine
    f_model = MLJ.fitted_params(MLJ.fitted_params(machine_mlj).machine).best_fitted_params[1]

    # Set the system
    return MathematicalSystems.ConstrainedBlackBoxControlDiscreteSystem(f_model, nbr_state, nbr_input, x_cons, u_cons)
end

### Third case linear MathematicalSystems from AutomationLabsIdentification without state constraint### 

"""
    _controller_system_design
A function for design the system (model and constraints) with MathematicalSystems for system from a tune model with mlj.
"""
function _controller_system_design(
    model_type::MLJMultivariateStatsInterface.MultitargetLinearRegressor,
    machine_mlj,
    input_constraint,
    nbr_state,
    nbr_input)

    # Set constraints with Lazy Sets
    lower_state_constraints = zeros(nbr_state, 1)[:, 1]
    higher_state_constraints = zeros(nbr_state, 1)[:, 1]
    lower_input_constraints = input_constraint[:, 1]
    higher_input_constraints = input_constraint[:, 2]
 
    # Set constraints with Lazy Sets
    x_cons = LazySets.Hyperrectangle(low = lower_state_constraints, high = higher_state_constraints,)
    u_cons = LazySets.Hyperrectangle(low = lower_input_constraints, high = higher_input_constraints)
            
    # Extract model from the machine
    AB_t = MLJ.fitted_params(machine_mlj).coefficients
    AB = copy(AB_t')
    A = AB[:, 1:nbr_state]
    B = AB[:, nbr_state+1: end]

    # Set the system
    return MathematicalSystems.ConstrainedLinearControlDiscreteSystem(A, B, x_cons, u_cons)
end

### Fourth case linear MathematicalSystems from AutomationLabsIdentification with state constraint### 

"""
    _controller_system_design
A function for design the system (model and constraints) with MathematicalSystems for system from a tune model with mlj.
"""
function _controller_system_design(
    model_type::MLJMultivariateStatsInterface.MultitargetLinearRegressor,
    machine_mlj,
    input_constraint,
    state_constraint,
    nbr_state,
    nbr_input)

    # There are state constraints 

    # Set constraints with Lazy Sets
    lower_state_constraints = state_constraint[:, 1]
    higher_state_constraints = state_constraint[:, 2]
    lower_input_constraints = input_constraint[:, 1]
    higher_input_constraints = input_constraint[:, 2]

    # Set constraints with Lazy Sets
    x_cons = LazySets.Hyperrectangle(low = lower_state_constraints, high = higher_state_constraints,)
    u_cons = LazySets.Hyperrectangle(low = lower_input_constraints, high = higher_input_constraints)
            
    # Extract model from the machine
    AB_t = MLJ.fitted_params(machine_mlj).coefficients
    AB = copy(AB_t')
    A = AB[:, 1:nbr_state]
    B = AB[:, nbr_state+1: end]

    # Set the system
    return MathematicalSystems.ConstrainedLinearControlDiscreteSystem(A, B, x_cons, u_cons)
end

### Fifth case linear MathematicalSystems from users with state constraint### 

"""
    _controller_system_design
A function for design the system (model and constraints) with MathematicalSystems for system from a simple linear.
"""
function _controller_system_design(
    model_mlj::LinearModel,
    A,
    B,
    input_constraint,
    state_constraint,
    nbr_state,
    nbr_input)

    # There are state constraints 

    # Set constraints with Lazy Sets
    lower_state_constraints = state_constraint[:, 1]
    higher_state_constraints = state_constraint[:, 2]
    lower_input_constraints = input_constraint[:, 1]
    higher_input_constraints = input_constraint[:, 2]

    # Set constraints with Lazy Sets
    x_cons = LazySets.Hyperrectangle(low = lower_state_constraints, high = higher_state_constraints,)
    u_cons = LazySets.Hyperrectangle(low = lower_input_constraints, high = higher_input_constraints)
            
    # Set the system
    return MathematicalSystems.ConstrainedLinearControlDiscreteSystem(A, B, x_cons, u_cons)
end

"""
    _controller_system_design
A function for design the system (model and constraints) with MathematicalSystems for system from a tune model with mlj.
"""
function _controller_system_design(
    model_type::LinearModel,
    A,
    B,
    input_constraint,
    nbr_state,
    nbr_input)

    # Set constraints with Lazy Sets
    lower_state_constraints = zeros(nbr_state, 1)[:, 1]
    higher_state_constraints = zeros(nbr_state, 1)[:, 1]
    lower_input_constraints = input_constraint[:, 1]
    higher_input_constraints = input_constraint[:, 2]
 
    # Set constraints with Lazy Sets
    x_cons = LazySets.Hyperrectangle(low = lower_state_constraints, high = higher_state_constraints,)
    u_cons = LazySets.Hyperrectangle(low = lower_input_constraints, high = higher_input_constraints)
            
    # Set the system
    return MathematicalSystems.ConstrainedLinearControlDiscreteSystem(A, B, x_cons, u_cons)
end



### Third case linear MathematicalSystems from simple linear model ### 

#=
"""
    _controller_system_design
A function for design the system (model and constrants) with MathematicalSystems for model predictive control and economic model predictive control.

"""
function _controller_system_design(
            model_mlj::LinearModel,
            lower_state_constraints,
            higher_state_constraints,
            lower_input_constraints,
            higher_input_constraints;  
            kws_...
    )

    # Get argument kws
    dict_kws = Dict{Symbol,Any}(kws_)
    kws = get(dict_kws, :kws, kws_)

    # Evaluate if the state constraint is selected
    if haskey(kws, :mpc_state_constraint) == true
        #there are state constraints 
        mpc_state_constraint = kws[:mpc_state_constraint]

    # Get state and input number
    nbr_state = size(lower_state_constraints, 1)
    nbr_input = size(lower_input_constraints, 1)
    
    # Set constraints with Lazy Sets
    x_cons = LazySets.Hyperrectangle(low = lower_state_constraints, high = higher_state_constraints,)
    u_cons = LazySets.Hyperrectangle(low = lower_input_constraints, high = higher_input_constraints)
            
    # Extract model from the machine
    AB_t = MLJ.fitted_params(machine_mlj).coefficients
    AB = copy(AB_t')
    A = AB[:, 1:4]
    B = AB[:, 5: end]

    # Set the system
    system = MathematicalSystems.ConstrainedLinearControlDiscreteSystem(A, B, x_cons, u_cons)

    return system
end
=#

### Fourth case non linear MathematicalSystems from simple non linear model ### 

=#