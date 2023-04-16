# AutomationLabsSystems Changelog

## v0.1.9

* Add proceed_system_evaluation and proceed_system_constraints_evaluation methods.
 
## v0.1.8

* Remove dependencies AutomationLabsIdentification, MLJ, MLJMultivariateStatsInterface.
* Remove design_models.jl.
* Add type models for removing the AutomationLabsIdentification dependency.
* proceed_system_model_evaluation types modifications.

## v0.1.7

* Add dict for return for _extract_model_from_machine functions.
* Tests improvement.

## v0.1.6

* Modification of proceed_system_model_evaluation with the name of neural networks sub-layers.

## v0.1.5

* Remove neuralode_type1 and keep only neuralode_type2 as neuralODE from AutomationLabsIdentification.jl.

## v0.1.4

* Modification of proceed_system (linear and non-linear model). 
* Add system without constraints (linear and non-linear model). 
* Improve tests.

## v0.1.3

* Julia code formatter.
* Add Actions.
* Improve tests for continuous integration.
* Project.toml [compat] entry deps with upper-bounded.

## v0.1.2

* Add continuous linear system discretization function.
* Improve tests.

## v0.1.1

* Add system linearization.
* Add model evaluation from a system.
* Improve tests.
## v0.1.0

* Initial public release.
