# Copyright (c) 2022: Pierre Blaud and contributors
########################################################
# This Source Code Form is subject to the terms of the #
# Mozilla Public License, v. 2.0. If a copy of the MPL #
# was not distributed with this file,  				   #
# You can obtain one at https://mozilla.org/MPL/2.0/.  #
########################################################

# Test subfunctions from AutomationPod cli package
print("Testing AutomationLabsSystems sub functions...")
took_seconds = @elapsed include("./systems_test.jl");
println("done (took ", took_seconds, " seconds)")

