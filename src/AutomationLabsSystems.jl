# Copyright (c) 2022: Pierre Blaud and contributors
########################################################
# This Source Code Form is subject to the terms of the #
# Mozilla Public License, v. 2.0. If a copy of the MPL #
# was not distributed with this file,  				   #
# You can obtain one at https://mozilla.org/MPL/2.0/.  #
########################################################

module AutomationLabsSystems

import MathematicalSystems
import AutomationLabsIdentification
import MLJ
import MLJMultivariateStatsInterface
import LazySets
import Flux
import ForwardDiff
import NNlib
import ControlSystems

export proceed_system
export proceed_system_linearization

export proceed_system_discretization 
export proceed_system_model_evaluation

# Load files
include("types/types.jl")
include("sub/design_models.jl")
include("sub/design_systems.jl")
include("main/main_systems.jl")

end
