// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ExampleSubsystem;

/**
 * An example command that uses an example subsystem.
 */
public class TankDrive extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final DriveTrain m_driveTrain;
    private final double leftOutput, rightOutput;

    /**
     * Creates a new TankDrive.
     *
     * @param subsystem The subsystem used by this command.
     */
    public TankDrive(DriveTrain driveTrain, double leftOutput, double rightOutput) {
        m_driveTrain = driveTrain;
        this.leftOutput = leftOutput;
        this.rightOutput = rightOutput;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(driveTrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_driveTrain.setMotorTankDrive(leftOutput, rightOutput);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_driveTrain.setMotorTankDrive(0, 0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
