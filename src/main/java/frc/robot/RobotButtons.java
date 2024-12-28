// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.Swerve;

/** Add your docs here. */
public class RobotButtons {
  private final CommandPS5Controller driverController = new CommandPS5Controller(OperatorConstants.DRIVER_CONTROLLER_PORT);
    private final int translationAxis = PS5Controller.Axis.kLeftY.value;
    private final int strafeAxis = PS5Controller.Axis.kLeftX.value;
    private final int rotationAxis = PS5Controller.Axis.kRightX.value;

    public void loadButtons(Swerve swerve){
        swerve.setDefaultCommand(
            new TeleopSwerve(
                    swerve,
                    () -> driverController.getRawAxis(translationAxis),
                    () -> driverController.getRawAxis(strafeAxis),
                    () -> driverController.getRawAxis(rotationAxis),
                    () -> false));

        driverController.touchpad().onTrue(new InstantCommand(() -> swerve.zeroHeading()));
        // driverController.options().onTrue(swerve.Lock());
    }

}
