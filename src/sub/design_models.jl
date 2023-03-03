# Copyright (c) 2022: Pierre Blaud and contributors
########################################################
# This Source Code Form is subject to the terms of the #
# Mozilla Public License, v. 2.0. If a copy of the MPL #
# was not distributed with this file,  				   #
# You can obtain one at https://mozilla.org/MPL/2.0/.  #
########################################################

#to do find a better way to get the AutomationLabsIdentification type
function _get_mlj_model_type(
    machine_mlj::MLJ.Machine{MLJMultivariateStatsInterface.MultitargetLinearRegressor,true},
)

    return machine_mlj.model
end

function _get_mlj_model_type(machine_mlj)

    return MLJ.fitted_params(MLJ.fitted_params(machine_mlj).machine).best_model.builder
end

"""
    _extract_model_from_machine
A function for design the system (model and constraints) with MathematicalSystems from linear model.
"""
function _extract_model_from_machine(
    model_type::Union{
        AutomationLabsIdentification.Fnn,
        AutomationLabsIdentification.Icnn,
        AutomationLabsIdentification.ResNet,
        AutomationLabsIdentification.DenseNet,
        AutomationLabsIdentification.Rbf,
        AutomationLabsIdentification.PolyNet,
        AutomationLabsIdentification.NeuralNetODE_type1,
        AutomationLabsIdentification.NeuralNetODE_type2,
    },
    machine_mlj,
)

    # Extract best model from the machine
    return MLJ.fitted_params(MLJ.fitted_params(machine_mlj).machine).best_fitted_params[1]
end

"""
    _extract_model_from_machine
A function for design the system (model and constraints) with MathematicalSystems for system from a tune model with mlj.
"""
function _extract_model_from_machine(
    model_type::MLJMultivariateStatsInterface.MultitargetLinearRegressor,
    machine_mlj,
    nbr_state,
)

    # Extract model from the machine
    AB_t = MLJ.fitted_params(machine_mlj).coefficients
    AB = copy(AB_t')
    A = AB[:, 1:nbr_state]
    B = AB[:, nbr_state+1:end]

    # Set the system
    return A, B
end
