# Copyright (c) 2022: Pierre Blaud and contributors
########################################################
# This Source Code Form is subject to the terms of the #
# Mozilla Public License, v. 2.0. If a copy of the MPL #
# was not distributed with this file,  				   #
# You can obtain one at https://mozilla.org/MPL/2.0/.  #
########################################################

"""
    proceed_system
A function for system creation.

The following variables are mendatories:
* `model_origin`: a String for origin of the model.

It is possible to define optional variables kws.
"""
function proceed_system(model_method, model_origin; kws_...)

    # Get argument kws
    dict_kws = Dict{Symbol,Any}(kws_)
    kws = get(dict_kws, :kws, kws_)

    if model_origin == "user"

        # Evaluate if linear state matrix is present
        if haskey(kws, :A) == true && haskey(kws, :B) == true && model_method == "continuous"
            # It is a linear model
            A = kws[:A]
            B = kws[:B]
            nbr_state = size(A, 1)
            nbr_input = size(B, 2)
            model = ContinuousLinearModel(A, B, nbr_state, nbr_input)
        end

        # Evaluate if linear state matrix is present
        if haskey(kws, :A) == true && haskey(kws, :B) == true && model_method == "discrete"
            # It is a linear model
            A = kws[:A]
            B = kws[:B]
            nbr_state = size(A, 1)
            nbr_input = size(B, 2)
            model = DiscreteLinearModel(A, B, nbr_state, nbr_input)
        end

        # Evaluate if f and non linear
        if haskey(kws, :f) == true && model_method == "continuous"
            # It is a non linear model
            f = kws[:f]
            nbr_state = kws[:nbr_state]
            nbr_input = kws[:nbr_input]
            model = ContinuousNonLinearModel(f, nbr_state, nbr_input)
        end

        # Evaluate if f and non linear
        if haskey(kws, :f) == true && model_method == "discrete"
            # It is a non linear model
            f = kws[:f]
            nbr_state = kws[:nbr_state]
            nbr_input = kws[:nbr_input]
            model = DiscreteNonLinearModel(f, nbr_state, nbr_input)
        end

    end

    if model_origin == "identification"
        #extract the model type 

        if haskey(kws, :machine_mlj) == true && model_method == "continuous"

            machine_mlj = kws[:machine_mlj]
            # Get the type of the model from the machine_mlj
            model_mlj_type = _get_mlj_model_type(machine_mlj)

            if typeof(model_mlj_type) == MLJMultivariateStatsInterface.MultitargetLinearRegressor
                nbr_state = kws[:nbr_state]
                nbr_input = kws[:nbr_input]
                A, B = _extract_model_from_machine(model_mlj_type, machine_mlj, nbr_state)
                model = ContinuousLinearModel(A, B, nbr_state, nbr_input)
            else
                model_machine = _extract_model_from_machine(model_mlj_type, machine_mlj)
                nbr_state = kws[:nbr_state]
                nbr_input = kws[:nbr_input]
                model = ContinuousNonLinearModel(model_machine, nbr_state, nbr_input)
            end
        end

        if haskey(kws, :machine_mlj) == true && model_method == "discrete"

            machine_mlj = kws[:machine_mlj]
            # Get the type of the model from the machine_mlj
            model_mlj_type = _get_mlj_model_type(machine_mlj)

            if typeof(model_mlj_type) == MLJMultivariateStatsInterface.MultitargetLinearRegressor
                nbr_state = kws[:nbr_state]
                nbr_input = kws[:nbr_input]
                A, B = _extract_model_from_machine(model_mlj_type, machine_mlj, nbr_state)
                #nbr_state = size(A, 1)
                #nbr_input = size(B, 2)
                model = DiscreteLinearModel(A, B, nbr_state, nbr_input)
            else
                model_machine = _extract_model_from_machine(model_mlj_type, machine_mlj)
                nbr_state = kws[:nbr_state]
                nbr_input = kws[:nbr_input]
                model = DiscreteNonLinearModel(model_machine, nbr_state, nbr_input)
            end
        end

    end

    # Set the system with constraints

    if haskey(kws, :input_constraint) == true && haskey(kws, :state_constraint) == true
        # System with input and state constraints
        return system = _controller_system_design(model, kws[:input_constraint], kws[:state_constraint])

    elseif haskey(kws, :input_constraint) == true && haskey(kws, :state_constraint) == false
        # System with input and no state constraints
        return system = _controller_system_design(model, kws[:input_constraint])

    elseif haskey(kws, :input_constraint) == false && haskey(kws, :state_constraint) == false
        # Issues
        @error "There are no input nor state constraints"
    else
        # Issues
        @error "There are no input nor state constraints"
    end
        
end