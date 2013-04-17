#include <gtest/gtest.h>

#include <hardware_interface/actuator_state_interface.h>

struct ActuatorData
{
	double pos;
	double vel;
	double eff;
};

TEST(actuator_interface, state_test)
{
	ActuatorData data;

	hardware_interface::ActuatorStateInterface stateInterface;
	stateInterface.registerStateHandle("one", &(data.pos), &(data.vel), &(data.eff));
	hardware_interface::StateHandle stateHandle = stateInterface.getStateHandle("one");

	data.pos = 2.0;

	EXPECT_TRUE(stateHandle.getPosition() == 2.0);
	EXPECT_ANY_THROW( stateInterface.getStateHandle("two") );
}

#include <hardware_interface/actuator_command_interface.h>

struct ActuatorCommand
{
	double cmd;
};

TEST(actuator_interface, command_test)
{
	ActuatorData data;

	hardware_interface::ActuatorStateInterface stateInterface;
	stateInterface.registerStateHandle("one", &(data.pos), &(data.vel), &(data.eff));
	hardware_interface::StateHandle stateHandle = stateInterface.getStateHandle("one");

	ActuatorCommand cmd;

	hardware_interface::ActuatorCommandInterface cmdInterface;
	cmdInterface.registerCommandHandle(stateHandle, &(cmd.cmd));
	hardware_interface::CommandHandle cmdHandle = cmdInterface.getCommandHandle("one");

	cmdHandle.setCommand(2.0);

	EXPECT_TRUE(data.pos = 2.0);
	EXPECT_ANY_THROW( cmdInterface.getCommandHandle("two") );
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
