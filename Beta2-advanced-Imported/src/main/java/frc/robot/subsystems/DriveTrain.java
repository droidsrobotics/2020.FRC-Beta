package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import frc.robot.commands.JoystickControl;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANEncoder;
// import com.revrobotics.ControlType;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax.IdleMode;

// import edu.wpi.first.wpilibj.I2C;
// import edu.wpi.first.wpilibj.SPI;
// import edu.wpi.first.wpilibj.SerialPort.Port;
// import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.controller.PIDController;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;

/**
 *
 */
public class DriveTrain extends SubsystemBase {
    private final CANSparkMax m_leftMaster = new CANSparkMax(2, MotorType.kBrushless); // front
    private final CANSparkMax m_leftFollower = new CANSparkMax(1, MotorType.kBrushless); // rear
    private final CANSparkMax m_rightMaster = new CANSparkMax(3, MotorType.kBrushless); // front
    private final CANSparkMax m_rightFollower = new CANSparkMax(4, MotorType.kBrushless); // rear
    private final SpeedControllerGroup m_leftGroup = new SpeedControllerGroup(m_leftMaster, m_leftFollower);
    private final SpeedControllerGroup m_rightGroup = new SpeedControllerGroup(m_rightMaster, m_rightFollower);

    private final DifferentialDrive differentialDrive = new DifferentialDrive(m_leftGroup, m_rightGroup);
    // private final DifferentialDrive differentialDrive = new DifferentialDrive(m_leftMaster, m_rightMaster);
    private CANEncoder m_leftEncoder;
    private CANEncoder m_leftRearEncoder;
    private CANEncoder m_rightEncoder;
    private CANEncoder m_rightRearEncoder;

    public static final double kMaxSpeed = 3.0; // meters per second
    public static final double kMaxAngularSpeed = 2 * Math.PI; // one rotation per second

    private static final double kTrackWidth = 21.0 * 0.0254; // 21 inch in meters
    private static final double kWheelRadius = 3.0 * 0.0254; // 3 inch in meters
    private static final int kEncoderResolution = 42; // neo built-in
    private static final double kGearRatio = (50.0 / 14.0) * (48.0 / 16.0); // AM14U4 ratio 10.71

    public double initialAngle = 0.0;
    // private final AHRS navX = new AHRS(Port.kUSB);

    private final PIDController m_leftPIDController = new PIDController(.35, 0, 0);
    private final PIDController m_rightPIDController = new PIDController(.35, 0, 0);

    private final DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(kTrackWidth);

    private final DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(m_kinematics);

    public DriveTrain() {

        // navX.reset();

        differentialDrive.setSafetyEnabled(true);
        differentialDrive.setExpiration(0.1);
        differentialDrive.setMaxOutput(1.0);

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
        m_leftEncoder.setPositionConversionFactor(2.0 * Math.PI * kWheelRadius / kGearRatio);
        m_rightEncoder.setPositionConversionFactor(2.0 * Math.PI * kWheelRadius / kGearRatio);
        m_leftRearEncoder.setPositionConversionFactor(2.0 * Math.PI * kWheelRadius / kGearRatio);
        m_rightRearEncoder.setPositionConversionFactor(2.0 * Math.PI * kWheelRadius / kGearRatio);

        m_leftEncoder.setVelocityConversionFactor(2.0 * Math.PI * kWheelRadius / (60 * kGearRatio));
        m_rightEncoder.setVelocityConversionFactor(2.0 * Math.PI * kWheelRadius / (60 * kGearRatio));
        m_leftRearEncoder.setVelocityConversionFactor(2.0 * Math.PI * kWheelRadius / (60 * kGearRatio));
        m_rightRearEncoder.setVelocityConversionFactor(2.0 * Math.PI * kWheelRadius / (60 * kGearRatio));

        m_leftMaster.setIdleMode(IdleMode.kCoast);
        m_leftMaster.setSmartCurrentLimit(50);
        m_leftFollower.setIdleMode(IdleMode.kCoast);
        m_leftFollower.setSmartCurrentLimit(50);
        m_rightMaster.setIdleMode(IdleMode.kCoast);
        m_rightMaster.setSmartCurrentLimit(50);
        m_rightFollower.setIdleMode(IdleMode.kCoast);
        m_rightFollower.setSmartCurrentLimit(50);

        setDefaultCommand(new JoystickControl(this));

    }

    public void setSpeed(double speed) {
        differentialDrive.tankDrive(speed, speed); // Move
    }

    public void tankDrive(double left, double right) {
        differentialDrive.tankDrive(left, right);
    }

    @Override
    public void periodic() {
        // Put code here to be run every loop
    }

    public void arcadeDrive(double xSpeed, double zRotation) {
        differentialDrive.arcadeDrive(xSpeed, zRotation);
        // System.out.println("Left encoder:"+ getEncoderLeft());
        // System.out.println("Right encoder:"+ getEncoderRight());
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
        System.out.println("l/r feedb:" + (0.199+ 0.287*speeds.leftMetersPerSecond + leftOutput) + ", " + (0.205 + 0.286*speeds.rightMetersPerSecond + rightOutput) + " ||||| l/r speed:" + getVelocityLeft() + ", " + getVelocityRight());
        tankDrive(0.199+ 0.287*speeds.leftMetersPerSecond + leftOutput, 0.205 + 0.286*speeds.rightMetersPerSecond + rightOutput);
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
        var wheelSpeeds = m_kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0.0, rot));
        setSpeeds(wheelSpeeds);
    }

    /**
     * Updates the field-relative position.
     */
    public void updateOdometry() {
        m_odometry.update(getAngle(), getCurrentSpeeds());
    }
}
