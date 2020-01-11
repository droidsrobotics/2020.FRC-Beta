package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.kauailabs.navx.frc.AHRS;

import com.revrobotics.CANEncoder;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
// import com.revrobotics.ControlType;
// import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import frc.robot.commands.JoystickControl;

// import edu.wpi.first.wpilibj.I2C;
// import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort.Port;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
// import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;

import edu.wpi.first.wpilibj.controller.PIDController;

import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
// import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

import frc.robot.Constants.DriveConstants;

/**
 *
 */
public class DriveTrain extends SubsystemBase {
    private final CANSparkMax m_leftMaster = new CANSparkMax(DriveConstants.kLeftMotor1Port, MotorType.kBrushless); // front
    private final CANSparkMax m_leftFollower = new CANSparkMax(DriveConstants.kLeftMotor2Port, MotorType.kBrushless); // rear
    private final CANSparkMax m_rightMaster = new CANSparkMax(DriveConstants.kRightMotor1Port, MotorType.kBrushless); // front
    private final CANSparkMax m_rightFollower = new CANSparkMax(DriveConstants.kRightMotor2Port, MotorType.kBrushless); // rear
    private final SpeedControllerGroup m_leftMotors = new SpeedControllerGroup(m_leftMaster, m_leftFollower);
    private final SpeedControllerGroup m_rightMotors = new SpeedControllerGroup(m_rightMaster, m_rightFollower);

    private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);
    // private final DifferentialDrive differentialDrive = new
    // DifferentialDrive(m_leftMaster, m_rightMaster);
    private CANEncoder m_leftEncoder;
    private CANEncoder m_leftRearEncoder;
    private CANEncoder m_rightEncoder;
    private CANEncoder m_rightRearEncoder;



    public double initialAngle = 0.0;
    private final AHRS m_gyro = new AHRS(Port.kUSB);

    private final PIDController m_leftPIDController = new PIDController(.35, 0, 0);
    private final PIDController m_rightPIDController = new PIDController(.35, 0, 0);

    // private final DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(DriveConstants.kTrackwidthMeters);

    private final DifferentialDriveOdometry m_odometry;

    public DriveTrain() {

        // navX.reset();

        m_drive.setSafetyEnabled(true);
        m_drive.setExpiration(0.1);
        m_drive.setMaxOutput(1.0);

        m_leftEncoder = m_leftMaster.getEncoder();
        m_leftEncoder.setPosition(0.0);
        m_leftRearEncoder = m_leftFollower.getEncoder();
        m_leftRearEncoder.setPosition(0.0);
        m_rightEncoder = m_rightMaster.getEncoder();
        m_rightEncoder.setPosition(0.0);
        m_rightRearEncoder = m_rightFollower.getEncoder();
        m_rightRearEncoder.setPosition(0.0);

        // Set the distance per pulse for the drive encoders. We can simply use the
        // distance traveled for one rotation of the wheel divided by the encoder
        // resolution.
        m_leftEncoder.setPositionConversionFactor(Math.PI * DriveConstants.kWheelDiameterMeters / DriveConstants.kGearRatio);
        m_rightEncoder.setPositionConversionFactor(Math.PI * DriveConstants.kWheelDiameterMeters / DriveConstants.kGearRatio);
        m_leftRearEncoder.setPositionConversionFactor(Math.PI * DriveConstants.kWheelDiameterMeters / DriveConstants.kGearRatio);
        m_rightRearEncoder.setPositionConversionFactor(Math.PI * DriveConstants.kWheelDiameterMeters / DriveConstants.kGearRatio);

        m_leftEncoder.setVelocityConversionFactor(Math.PI * DriveConstants.kWheelDiameterMeters / (60 * DriveConstants.kGearRatio));
        m_rightEncoder.setVelocityConversionFactor(Math.PI * DriveConstants.kWheelDiameterMeters / (60 * DriveConstants.kGearRatio));
        m_leftRearEncoder.setVelocityConversionFactor(Math.PI * DriveConstants.kWheelDiameterMeters / (60 * DriveConstants.kGearRatio));
        m_rightRearEncoder.setVelocityConversionFactor(Math.PI * DriveConstants.kWheelDiameterMeters / (60 * DriveConstants.kGearRatio));

        m_leftMaster.setIdleMode(IdleMode.kCoast);
        m_leftMaster.setSmartCurrentLimit(50);
        m_leftFollower.setIdleMode(IdleMode.kCoast);
        m_leftFollower.setSmartCurrentLimit(50);
        m_rightMaster.setIdleMode(IdleMode.kCoast);
        m_rightMaster.setSmartCurrentLimit(50);
        m_rightFollower.setIdleMode(IdleMode.kCoast);
        m_rightFollower.setSmartCurrentLimit(50);

        m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));

        setDefaultCommand(new JoystickControl(this));

    }

    @Override
    public void periodic() {
        // Put code here to be run every loop
        m_odometry.update(Rotation2d.fromDegrees(getHeading()), m_leftEncoder.getPosition(),
                m_rightEncoder.getPosition());
    }

    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    /**
     * Returns the current wheel speeds of the robot.
     *
     * @return The current wheel speeds.
     */
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(m_leftEncoder.getVelocity(), m_rightEncoder.getVelocity());
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        m_odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
    }

    public void arcadeDrive(double xSpeed, double zRotation) {
        m_drive.arcadeDrive(xSpeed, zRotation);
        // System.out.println("Left encoder:"+ getEncoderLeft());
        // System.out.println("Right encoder:"+ getEncoderRight());
    }

    public void setSpeed(double speed) {
        m_drive.tankDrive(speed, speed); // Move
    }

    public void tankDrive(double left, double right) {
        m_drive.tankDrive(left, right);
    }

    public void tankDriveVolts(double leftVolts, double rightVolts) {
        m_leftMotors.setVoltage(leftVolts);
        m_rightMotors.setVoltage(-rightVolts);
    }

    /**
     * Resets the drive encoders to currently read a position of 0.
     */
    public void resetEncoders() {
        m_leftEncoder.setPosition(0);  
        m_rightEncoder.setPosition(0);
    }

    /**
     * Gets the average distance of the two encoders.
     *
     * @return the average of the two encoder readings
     */
    public double getAverageEncoderDistance() {
        return (m_leftEncoder.getPosition() + m_rightEncoder.getPosition()) / 2.0;
    }

    /**
     * Gets the left drive encoder.
     *
     * @return the left drive encoder
     */
    public CANEncoder getLeftEncoder() {
        return m_leftEncoder;
    }

    /**
     * Gets the right drive encoder.
     *
     * @return the right drive encoder
     */
    public CANEncoder getRightEncoder() {
        return m_rightEncoder;
    }

    /**
     * Sets the max output of the drive. Useful for scaling the drive to drive more
     * slowly.
     *
     * @param maxOutput the maximum output to which the drive will be constrained
     */
    public void setMaxOutput(double maxOutput) {
        m_drive.setMaxOutput(maxOutput);
    }

    /**
     * Zeroes the heading of the robot.
     */
    public void zeroHeading() {
        m_gyro.reset();
    }

    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from 180 to 180
     */
    public double getHeading() {
        return Math.IEEEremainder(m_gyro.getAngle(), 360) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
    }

    /**
     * Returns the turn rate of the robot.
     *
     * @return The turn rate of the robot, in degrees per second
     */
    public double getTurnRate() {
        return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
    }

    public double readRawAngle() {
        // System.out.println("Angle: "+ (navX.getAngle() - initialAngle));
        // return navX.getAngle();
        return (0.0);
    }

    public double getYaw() {
        double x;
        double curAngle = this.readRawAngle();
        if ((curAngle) % 360 < 0) {
            x = 360 + (curAngle) % 360;
        } else {
            x = (curAngle) % 360;
        }
        return (x);
    }

    public void resetGyro() {
        // initialAngle = this.readRawAngle();
        // navX.reset();
    }

    public double minDistance(double target) {
        double distanceCW = target - getYaw();

        if ((distanceCW > -180) && (distanceCW < 180)) {
            return (distanceCW);
        }
        if (distanceCW > 180) {
            return (distanceCW - 360);
        }
        return (distanceCW + 360);
    }

    public boolean clockwise(double target) {
        return (minDistance(target) > 0.0);
    }

    /**
     * Returns the angle of the robot as a Rotation2d.
     *
     * @return The angle of the robot.
     */
    public Rotation2d getAngle() {
        // may need to negate the angle because WPILib gyros are CW positive.
        // System.out.println("Angle: "+ (navX.getAngle() - initialAngle));
        return Rotation2d.fromDegrees(getYaw());
    }

    public double getEncoderLeftRear() {
        return (m_leftRearEncoder.getPosition());
    }

    public double getEncoderRightRear() {
        return (-1.0 * m_rightRearEncoder.getPosition());
    }

    public double getEncoderLeft() {
        return (m_leftEncoder.getPosition());
    }

    public double getEncoderRight() {
        return (-1.0 * m_rightEncoder.getPosition());
    }

    public double getVelocityLeftRear() {
        return (m_leftRearEncoder.getVelocity());
    }

    public double getVelocityRightRear() {
        return (-1.0 * m_rightRearEncoder.getVelocity());
    }

    public double getVelocityLeft() {
        return (m_leftEncoder.getVelocity());
    }

    public double getVelocityRight() {
        return (-1.0 * m_rightEncoder.getVelocity());
    }

    /**
     * Returns the current wheel speeds.
     *
     * @return The current wheel speeds.
     */
    public DifferentialDriveWheelSpeeds getCurrentSpeeds() {
        return new DifferentialDriveWheelSpeeds(getVelocityLeft(), getVelocityRight());
    }

    /**
     * Sets the desired wheel speeds.
     *
     * @param speeds The desired wheel speeds.
     */
    public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
        double leftOutput = m_leftPIDController.calculate(getVelocityLeft(), speeds.leftMetersPerSecond);
        double rightOutput = m_rightPIDController.calculate(getVelocityRight(), speeds.rightMetersPerSecond);
        System.out.println("l/r feedb:" + (0.199 + 0.287 * speeds.leftMetersPerSecond + leftOutput) + ", "
                + (0.205 + 0.286 * speeds.rightMetersPerSecond + rightOutput) + " ||||| l/r speed:" + getVelocityLeft()
                + ", " + getVelocityRight());
        tankDrive(0.199 + 0.287 * speeds.leftMetersPerSecond + leftOutput,
                0.205 + 0.286 * speeds.rightMetersPerSecond + rightOutput);
        // tankDrive(.4, .4);

        // m_leftGroup.set(leftOutput);
        // m_rightGroup.set(rightOutput);
    }

    /**
     * Drives the robot with the given linear velocity and angular velocity.
     *
     * @param xSpeed Linear velocity in m/s.
     * @param rot    Angular velocity in rad/s.
     */
    @SuppressWarnings("ParameterName")
    public void drive(double xSpeed, double rot) {
        var wheelSpeeds = DriveConstants.kDriveKinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0.0, rot));
        setSpeeds(wheelSpeeds);
    }

    /**
     * Updates the field-relative position.
     */
    public void updateOdometry() {
        // m_odometry.update(getAngle(), getCurrentSpeeds());
    }

}
