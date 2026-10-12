#pragma once

#ifdef CH46_HEADLESS_TEST
#ifdef CH46_CAPTURE_DRAW_TEST
#include <map>
inline std::map<int,float>& testCockpitDraws(){static std::map<int,float> values;return values;}
#endif
// Standalone physics tests only. Production builds still use DCS's real API.
class EDPARAM
{
public:
    void* getParamHandle(const char*) { return nullptr; }
    void setParamNumber(void*, double) {}
    // TEST-01: complete the no-cockpit stub for delivered callback tests.
    double getParamNumber(void*) { return 0; }
    void setParamString(void*, const char*) {}
    void setCockpitDrawArg(int arg, float value) {
#ifdef CH46_CAPTURE_DRAW_TEST
        testCockpitDraws()[arg]=value;
#endif
    }
    void setExternalDrawArg(int, float) {}
    float getCockpitDrawArg(int arg) {
#ifdef CH46_CAPTURE_DRAW_TEST
        return testCockpitDraws()[arg];
#else
        return 0;
#endif
    }
    float getExternalDrawArg(int) { return 0; }
};
#else
#include "ccParametersAPI.h"


class EDPARAM
{
public:
	EDPARAM()
	{
		ed_param_api = ed_get_cockpit_param_api();
	}
	cockpit_param_api ed_param_api;

	void* getParamHandle(const char* name)
	{
		return ed_param_api.get_parameter_handle(name);
	}

	void setParamNumber(void* handle, double value)
	{
		ed_param_api.update_parameter_with_number(handle, value);
	}

	void setParamString(void* handle, const char* string)
	{
		ed_param_api.update_parameter_with_string(handle, string);
	}

	double getParamNumber(void* handle)
	{
		double res{ 0 };
		ed_param_api.parameter_value_to_number(handle, res, false);
		return res;
	}

	void getParamString(void* handle, char* buffer, unsigned int buffer_size)
	{
		ed_param_api.parameter_value_to_string(handle, buffer, buffer_size);
	}

	int compareParams(void* handle1, void* handle2)
	{
		return ed_param_api.compare_parameters(handle1, handle2);
	}

	void dispatchAction(int device, int command, double value)
	{
		ed_param_api.dispatch_action_to_device(device, command, value);
	}

	void setCockpitDrawArg(int arg, float value)
	{
		ed_param_api.set_draw_argument(arg, value);
	}
	float getCockpitDrawArg(int arg)
	{
		return ed_param_api.get_draw_argument(arg);
	}
	void setExternalDrawArg(int arg, float value)
	{
		ed_param_api.set_external_draw_argument(arg, value);
	}
	float getExternalDrawArg(int arg)
	{
		return ed_param_api.get_external_draw_argument(arg);
	}

};
#endif
