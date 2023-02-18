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

export proceed_system

# Load files
include("types.jl")
include("design_models.jl")
include("design_systems.jl")
include("main_systems.jl")

end
