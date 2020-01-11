// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


package org.usfirst.frc9998.Droids2019.commands;
import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc9998.Droids2019.Robot;

import edu.wpi.first.wpilibj.Joystick;


/**
 *
 */
public class JoystickControl extends Command {

    public Joystick joystick1;

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_DECLARATIONS
 
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_DECLARATIONS

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR
    public JoystickControl() {

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_SETTING

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_SETTING
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
        requires(Robot.driveTrain);

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        joystick1 = Robot.oi.getJoystick1();
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        int DPAD = joystick1.getPOV();
        double target = 0;
        if (DPAD != -1) {
          // This handles the angles as special cases and makes them align
          // correctly to the Rocket
          if ((DPAD == 45) || (DPAD == 225)) {
            target = DPAD-16.25;
            TurnTo turn = new TurnTo(target);
            turn.start();
          } else {
            if ((DPAD == 135) || (DPAD == 305)) {
              target = DPAD+16.25;
              TurnTo turn = new TurnTo(target);
              turn.start();
            }
            else {
              // this handles the normal straight, back, left, right directions
              TurnTo turn = new TurnTo(DPAD);
              turn.start();
            }
          }
        } else {
            Robot.driveTrain.arcadeDrive(Math.pow(joystick1.getRawAxis(1),1.0) * -0.5, Math.pow(joystick1.getRawAxis(2),1.0)*0.5);
        }
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
    }
}
