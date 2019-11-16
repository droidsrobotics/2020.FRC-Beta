package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort.Port;
import com.kauailabs.navx.frc.AHRS;

public class Gyro extends SubsystemBase {
    public double initialAngle = 0.0;
    private AHRS navX;

    public Gyro() {
        navX = new AHRS(Port.kUSB);
    }

    @Override
    public void periodic() {
        // Put code here to be run every loop

    }

    public double readAngle() {
        // System.out.println("Angle: "+ (navX.getAngle() - initialAngle));
        return navX.getAngle() - initialAngle;
    }

    public double getYaw() {
        double x;
        double curAngle = this.readAngle();
        if ((curAngle) % 360 < 0) {
            x = 360 + (curAngle) % 360;
        } else {
            x = (curAngle) % 360;
        }
        return (x);
    }

    public void resetGyro() {
        initialAngle = this.readAngle();
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

}