package frc.robot.util;
import frc.robot.Constants;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class VisionMath {

    private double zRobot = Constants.kZRobotInches;
    private double cameraRotation = Constants.kCameraRotation;

    private double m_xRobot,m_yRobot;
    public double getRobotX(){
        return m_xRobot;
    }
    public double getRobotY(){
        return m_yRobot;
    }

    public void findRobotPos(){
        NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
		NetworkTableEntry txe = limelightTable.getEntry("tx");
        double tx = txe.getDouble(0.0);
        tx *= (Math.PI/180.0);
		NetworkTableEntry tye = limelightTable.getEntry("ty");
        double ty = tye.getDouble(0.0) - 90;
        ty *= (Math.PI/180.0);
        
        double r = zRobot / (
            (Math.cos(ty)*Math.cos(cameraRotation))
            - (Math.sin(ty)*Math.cos(tx)*Math.sin(cameraRotation))
            );

        double xCam = r * (
            (Math.sin(ty)*Math.cos(tx)*Math.cos(cameraRotation))
            +(Math.cos(ty)*Math.sin(cameraRotation))
        );

        double yCam = r * (Math.sin(ty)*Math.sin(tx));

        double g = Gyro.getYaw() * (Math.PI/180.0);
        m_yRobot = (xCam * Math.sin(g)) + (yCam * Math.cos(g));
        m_xRobot = (xCam * Math.cos(g)) - (yCam * Math.sin(g));
    }
}
