
package frc.robot.commands;

import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj2.command.CommandBase;

import edu.wpi.first.wpilibj.Joystick;

/**
 *
 */
public class JoystickControl extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final DriveTrain m_driveTrain;

  public final Joystick m_joystick1 = new Joystick(0);;

  public JoystickControl(DriveTrain driveTrain) {
    m_driveTrain = driveTrain;
    addRequirements(driveTrain);
  }

  // Called just before this Command runs the first time
  public void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  public void execute() {
    m_driveTrain.tankDrive(m_joystick1.getRawAxis(1), m_joystick1.getRawAxis(3));

    // m_driveTrain.arcadeDrive(m_joystick1.getY() * -1.0, m_joystick1.getX());
  }

  // Make this return true when this Command no longer needs to run execute()
  public boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  public void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  public void interrupted() {
  }
}
