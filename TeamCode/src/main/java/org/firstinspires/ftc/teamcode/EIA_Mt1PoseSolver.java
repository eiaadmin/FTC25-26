package org.firstinspires.ftc.teamcode;
import com.pedropathing.geometry.Pose;
public class EIA_Mt1PoseSolver {
    // Camera transform (meters)
    private static final double CAM_X = 0.11693;   // right positive
    private static final double CAM_Y = -0.16415;  // forward positive (+Y). Camera is behind center → negative
    private static final double CAM_Z = 0.14286;   // height
    // Camera orientation
    private static final double CAM_YAW_RAD = Math.toRadians(180.0);   // facing backward
    private static final double CAM_PITCH_RAD = Math.toRadians(13.5);  // tilted up
    // Tunable: scale for converting ta → distance
    private static final double TA_TO_DISTANCE = 1.80;   // you can tune this using real data
    /**
     * Solve MT1 pose from LL tx, ty, ta.
     */
    public Pose solvePoseFromLL(double tx, double ty, double ta, Pose tagPosePedro) {
        // 1) Convert angles to radians
        double txRad = Math.toRadians(tx);
        double tyRad = Math.toRadians(ty);
        // 2) Distance estimate from area
        double distance = TA_TO_DISTANCE / Math.sqrt(ta);
        // 3) Ray in camera frame (account for pitch)
        double cameraYaw = txRad;
        double cameraPitch = tyRad + CAM_PITCH_RAD;
        // Direction vector of ray in camera coordinates
        double dx_cam = Math.sin(cameraYaw) * Math.cos(cameraPitch);
        double dy_cam = Math.cos(cameraYaw) * Math.cos(cameraPitch);
        double dz_cam = Math.sin(cameraPitch);
        // 4) Convert camera-ray vector into field frame
        // Apply camera yaw (rotate 180°)
        double dx_field = -dx_cam;   // rotate 180° yaw
        double dy_field = -dy_cam;
        double dz_field = dz_cam;
        // 5) Scale ray to match distance estimate
        double fx = distance * dx_field;
        double fy = distance * dy_field;
        // 6) Camera global position = tag pose - ray
        double camGlobalX = tagPosePedro.getX() - fx;
        double camGlobalY = tagPosePedro.getY() - fy;
        // 7) Robot global position = camera global - offset
        double robotX = camGlobalX - CAM_X;
        double robotY = camGlobalY - CAM_Y;
        // 8) Robot heading = direction from robot to tag, corrected for robot forward = +Y
        double dx = tagPosePedro.getX() - robotX;
        double dy = tagPosePedro.getY() - robotY;
        // FTC heading (0 = +X). We convert to Pedro later.
        double headingFTC = Math.atan2(dy, dx);
        // Convert FTC heading to Pedro (0 = +Y)
        // PedroHeading = FTCHeading - 90° (π/2)
        double headingPedro = headingFTC - Math.PI / 2.0;
        // normalize
        while (headingPedro > Math.PI) headingPedro -= 2 * Math.PI;
        while (headingPedro <= -Math.PI) headingPedro += 2 * Math.PI;
        return new Pose(robotX, robotY, headingPedro);
    }
}