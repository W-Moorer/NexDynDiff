#pragma once
#include <limits>
#include <string>
#include <iostream>

namespace nexdyndiff
{
	#define NEXDYNDIFF_COMMON_HANDLER_CONTENTS(ModelClassName, ParamsClassName)         \
		private:                                                                   \
			ModelClassName* model = nullptr;                                       \
			int idx = -1;                                                          \
																				   \
		public:                                                                    \
			inline Handler() {}     \
			inline Handler(ModelClassName* model, int idx) : model(model), idx(idx) {}     \
			inline int get_idx() const { return this->idx; }                       \
			inline ParamsClassName get_params() const { return this->model->get_params(*this); } \
			inline void set_params(const ParamsClassName& params) { this->model->set_params(*this, params); } \
			inline ModelClassName* get_model() { return this->model; }             \
			inline const ModelClassName* get_model() const { return this->model; } \
			inline bool is_valid() const { return this->model != nullptr; } \
			inline void exit_if_not_valid(const std::string& where_) const { if (!this->is_valid()) { std::cout << "nexdyndiff error: Invalid handler found in " << where_ << std::endl; exit(-1); } } \


	/**
	* The following are utility macros for defining parameters with custom names and validity checks.
	*/

	// A macro for a parameter with a custom validator that is triggered when setting.
	#define NEXDYNDIFF_PARAM(Type, ParamName, Default, Validator) \
		Type ParamName = Default; \
		inline auto set_##ParamName(Type value) { \
			if (!Validator(value)) { \
				std::cout << "nexdyndiff error: invalid value " << value << " for NEXDYNDIFF_PARAM " <<  #ParamName << std::endl; exit(-1); \
			} \
			this->ParamName = value; return *this; \
		} \
		inline Type get_##ParamName() const { return this->ParamName; }

	// Useful macros for common parameter types.
	#define NEXDYNDIFF_PARAM_NO_VALIDATION(Type, ParamName, Default) \
		NEXDYNDIFF_PARAM(Type, ParamName, Default, [](Type value)->bool { return true; })
	#define NEXDYNDIFF_PARAM_NON_NEGATIVE(Type, ParamName, Default) \
		NEXDYNDIFF_PARAM(Type, ParamName, Default, [](Type value)->bool { return 0.0 <= value; })
	#define NEXDYNDIFF_PARAM_POSITIVE(Type, ParamName, Default) \
		NEXDYNDIFF_PARAM(Type, ParamName, Default, [](Type value)->bool { return std::numeric_limits<double>::epsilon() < value; })

	// Common parameters
	#define NEXDYNDIFF_PARAM_ELASTICITY_ONLY()	NEXDYNDIFF_PARAM_NO_VALIDATION(bool,	elasticity_only, false)
	#define NEXDYNDIFF_PARAM_SCALE()				NEXDYNDIFF_PARAM_NON_NEGATIVE(double, scale, 1.0)
	#define NEXDYNDIFF_PARAM_YOUNGS_MODULUS()	NEXDYNDIFF_PARAM_NON_NEGATIVE(double, youngs_modulus, 1e3)
	#define NEXDYNDIFF_PARAM_POISSONS_RATIO()	NEXDYNDIFF_PARAM(double, poissons_ratio, 0.3, [](double value) { return 0.0 < value && value <= 0.5; })
	#define NEXDYNDIFF_PARAM_DAMPING()			NEXDYNDIFF_PARAM_NON_NEGATIVE(double, damping, 0.0)
	#define NEXDYNDIFF_PARAM_STRAIN_LIMITING()	NEXDYNDIFF_PARAM_NON_NEGATIVE(double, strain_limit, std::numeric_limits<double>::max()) \
											NEXDYNDIFF_PARAM_NON_NEGATIVE(double, strain_limit_stiffness, 1e3)
	#define NEXDYNDIFF_PARAM_STIFFNESS()			NEXDYNDIFF_PARAM_NON_NEGATIVE(double, stiffness, 1e3)
	#define NEXDYNDIFF_PARAM_TOLERANCE()			NEXDYNDIFF_PARAM_NO_VALIDATION(double, tolerance, std::numeric_limits<double>::max())


	/**
	* @brief A multi handler is a collection of handlers that share the same parameters.
	* A use case example is mesh attachments where multiple attachments types (point-point, point-edge, ...) share the same parameters.
	*/
	template<typename Handler, typename Params>
	struct MultiEnergyHandler
	{
	private:
		std::vector<Handler> handlers;

	public:
		MultiEnergyHandler(const std::vector<Handler>& handlers) : handlers(handlers) {}
		inline Params get_params() const { return this->handlers[0].get_params(); }
		inline void set_params(const Params& params) { for (Handler& handler : this->handlers) { handler.set_params(params); } }
	};
}