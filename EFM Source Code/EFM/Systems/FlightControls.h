#pragma once

// Flight controls in the AH-6J are very simple. Other aircraft will require hydraulics and flight control systems (FCS, SAS etc.)

class FlightControls
{
private:
    // CTRL-01 (0.1.4): keep physical pedals separate from keyboard yaw.
    // Releasing a yaw key removes only the keyboard contribution, so a
    // currently held physical pedal is neither latched nor silently centered.
    double pedalAxis = 0.0;
    double keyboardPedal = 0.0;
    void updatePedal() { PedalInput = limit(pedalAxis + keyboardPedal, -1.0, 1.0); }
public:
	FlightControls() {}
	~FlightControls() {}

	EDPARAM cockpitAPI;

	double CollectiveInput = 0.0;// Raw collective input
	double PedalInput = 0.0;	// Raw pedal input	
	double PitchInput = 0.0;	// Raw pitch input
	double RollInput = 0.0;		// Raw roll input

	double rollTrim = 0.0;
	double pitchTrim = 0.0;

	double pitchOutput = 0.0;
	double rollOutput = 0.0;

	// These are used for the controls indicator
	void* PITCH_INPUT = cockpitAPI.getParamHandle("PITCH_INPUT");
	void* ROLL_INPUT = cockpitAPI.getParamHandle("ROLL_INPUT");
	void* PEDAL_INPUT = cockpitAPI.getParamHandle("PEDAL_INPUT");
	void* COLLECTIVE_INPUT = cockpitAPI.getParamHandle("COLLECTIVE_INPUT");

	void init()
	{
		CollectiveInput = 0.0;
		PedalInput = 0.0;
		PitchInput = 0.0;
		RollInput = 0.0;
		rollTrim = 0.0;
		pitchTrim = 0.0;
		pedalAxis = keyboardPedal = 0.0;
        // CTRL-02: reset cached outputs immediately, not on a later sim tick.
        pitchOutput = rollOutput = 0.0;
	}

	void release()
	{
        init(); // CTRL-02: one complete reset for release and direct initialization.
	}

    void setPedalAxis(double value) {
        pedalAxis = limit(value, -1.0, 1.0);
        updatePedal();
    }

	void setPedLeft()
	{
        keyboardPedal = limit(keyboardPedal + 0.008, -1.0, 1.0);
        updatePedal();
	}
	void setPedRight()
	{
        keyboardPedal = limit(keyboardPedal - 0.008, -1.0, 1.0);
        updatePedal();
	}
	void setPedStop()
	{
        // CTRL-01: key-up used to retain up to +/-0.5 pedal indefinitely.
        keyboardPedal = 0.0;
        updatePedal();
	}
	void setCollUp()
	{
		CollectiveInput = limit(CollectiveInput + 0.007, 0, 1.0);
	}
	void setCollDown()
	{
		CollectiveInput = limit(CollectiveInput - 0.007, 0, 1.0);
	}
	void setCyclicForward()
	{
		PitchInput = limit(PitchInput + 0.007, -1.0, 1.0);
	}
	void setCyclicBack()
	{
		PitchInput = limit(PitchInput - 0.007, -1.0, 1.0);
	}
	void setCyclicLeft()
	{
		RollInput = limit(RollInput - 0.007, -1.0, 1.0);
	}
	void setCyclicRight()
	{
		RollInput = limit(RollInput + 0.007, -1.0, 1.0);
	}

	/*
	void setCommand(int command, const float value)
	{
		if (command == JoystickRoll)
		{
			RollInput = limit(value, -1.0, 1.0);
		}
		else if (command == JoystickPitch)
		{
			PitchInput = limit(-value, -1.0, 1.0);
		}
		else if (command == JoystickYaw)
		{
			PedalInput = limit(-value, -1.0, 1.0);
		}
		else if (command == JoystickThrottle)
		{
			CollectiveInput = limit(((-value + 1.0) / 2.0), 0.0, 1.0);
		}

		else if (command == trimUp)
		{
			pitchTrim = limit(flightControls.pitchTrim - 0.0015, -1, 1);
		}
		else if (command == trimDown)
		{
			pitchTrim = limit(flightControls.pitchTrim + 0.0015, -1, 1);
		}
		else if (command == trimLeft)
		{
			rollTrim = limit(flightControls.rollTrim - 0.0015, -1, 1);
		}
		else if (command == trimRight)
		{
			rollTrim = limit(flightControls.rollTrim + 0.0015, -1, 1);
		}
	}*/

	void update()
	{
		pitchOutput = limit(PitchInput + pitchTrim, -1, 1);
		rollOutput = limit(RollInput + rollTrim, -1, 1);

		//----- for lua Control Indicator ------------------------
		cockpitAPI.setParamNumber(PITCH_INPUT, pitchOutput);
		cockpitAPI.setParamNumber(ROLL_INPUT, rollOutput);
		cockpitAPI.setParamNumber(PEDAL_INPUT, PedalInput);
		cockpitAPI.setParamNumber(COLLECTIVE_INPUT, CollectiveInput);
	}

};
