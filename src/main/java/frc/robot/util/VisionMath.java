package frc.robot.util;
import frc.robot.Constants;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class VisionMath {

    private double zRobot = Constants.kZRobotInches;
    private double cameraRotation = Constants.kCameraRotation;

    public double findR(){
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
        return r;
    }

    public double findX(double r){
        NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
		NetworkTableEntry txe = limelightTable.getEntry("tx");
        double tx = txe.getDouble(0.0);
        tx *= (Math.PI/180.0);
		NetworkTableEntry tye = limelightTable.getEntry("ty");
        double ty = tye.getDouble(0.0) - 90;
        ty *= (Math.PI/180.0);

        double x = r * (
            (Math.sin(ty)*Math.cos(tx)*Math.cos(cameraRotation))
            +(Math.cos(ty)*Math.sin(cameraRotation))
        );
        return x;
    }

    public double findY(double r){
        NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
		NetworkTableEntry txe = limelightTable.getEntry("tx");
        double tx = txe.getDouble(0.0);
        tx *= (Math.PI/180.0);
		NetworkTableEntry tye = limelightTable.getEntry("ty");
        double ty = tye.getDouble(0.0) - 90;
        ty *= (Math.PI/180.0);

        double y = r * (Math.sin(ty)*Math.sin(tx));
        return y;
    }
}
