package frc.robot.commands;

import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj2.command.CommandBase;

import edu.wpi.first.wpilibj.Timer;

/**
 *
 */
public class AutoMove extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final Timer m_timer = new Timer();

  private final DriveTrain m_driveTrain;

  int i = 0;

  public AutoMove(DriveTrain driveTrain) {
    m_driveTrain = driveTrain;
    addRequirements(driveTrain);
  }

  // Called repeatedly when this Command is scheduled to run
  public void execute() {
    i++;
    // Drive for 2 seconds
    if (i % 5 == 0) {
      // System.out.println(m_timer.get() + "l/r speed:" + m_driveTrain.getCurrentSpeeds().leftMetersPerSecond + ", "
      //     + m_driveTrain.getCurrentSpeeds().rightMetersPerSecond
      // + " |||| l/r:"+ m_driveTrain.getEncoderLeft() + ", " +
      // m_driveTrain.getEncoderRight()
      // + "|||| l/r rear:"+ m_driveTrain.getEncoderLeftRear() + ", " +
      // m_driveTrain.getEncoderRightRear()
      // );
    }
    if (m_timer.get() < 4.0) {
      // m_driveTrain.arcadeDrive(0.4, 0.0); // drive forwards half speed
      // m_driveTrain.tankDrive(0.2, 0.2); // drive forwards half speed
      // m_driveTrain.drive(0.6, 0.0);
      
      // } else if (m_timer.get() < 4.0) {
      //   // m_driveTrain.arcadeDrive(0.4, 0.0); // drive forwards half speed
      //   m_driveTrain.tankDrive(0.3, 0.3); // drive forwards half speed
      //   // m_driveTrain.drive(0.5, 0.0);
      // } else if (m_timer.get() < 6.0) {
      //   // m_driveTrain.arcadeDrive(0.4, 0.0); // drive forwards half speed
      //   m_driveTrain.tankDrive(0.4, 0.4); // drive forwards half speed
      //   // m_driveTrain.drive(0.5, 0.0);
      // } else if (m_timer.get() < 8.0) {
      //   // m_driveTrain.arcadeDrive(0.4, 0.0); // drive forwards half speed
      //   m_driveTrain.tankDrive(0.5, 0.5); // drive forwards half speed
      //   // m_driveTrain.drive(0.5, 0.0);
      // } else if (m_timer.get() < 10.0) {
      //   // m_driveTrain.arcadeDrive(0.4, 0.0); // drive forwards half speed
      //   m_driveTrain.tankDrive(0.6, 0.6); // drive forwards half speed
      //   // m_driveTrain.drive(0.5, 0.0);

    } else {
      m_driveTrain.setSpeed(0); // stop robot
    }
  }

  // Make this return true when this Command no longer nee ds to run execute()
  public boolean isFinished() {
    if (m_timer.get() < 10) {
      return false;
    }
    m_driveTrain.setSpeed(0); // stop robot
    return true;
  }

  // Called once after isFinished returns true
  public void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  public void interrupted() {
  }

  public void initialize() {
    m_timer.reset();
    i = 0;
    m_timer.start();
  }
}
