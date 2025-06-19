package jp.jaxa.iss.kibo.rpc.sampleapk.oasis;

import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;
import jp.jaxa.iss.kibo.rpc.api.KiboRpcApi;
import jp.jaxa.iss.kibo.rpc.sampleapk.vision.ArMarkerDetector;
import org.opencv.core.Mat;
import android.util.Log;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.HashMap;
import java.util.Arrays;
import jp.jaxa.iss.kibo.rpc.sampleapk.oasis.DetectedItemInfo;

/**
 * Utility class for Oasis-related operations (points, quaternions, scanning,
 * image capture, saving).
 */
public class OasisUtils {
    public static List<Point> getOasisPoints() {
        List<Point> points = new ArrayList<>();
        points.add(new Point(10.925d, -9.85d, 4.695d));
        points.add(new Point(11.175d, -8.975d, 5.195d));
        points.add(new Point(10.7d, -7.925d, 5.195d));
        points.add(new Point(11.175d, -6.875d, 4.685d));
        return points;
    }

    public static List<Quaternion> getOasisQuaternions(int areaIdx) {
        List<Quaternion> quaternions = new ArrayList<>();
        /*
            quaternions.add(new Quaternion(0.000000f, 0.500000f, 0.000000f, 0.866025f)); // Yaw: 0, Pitch: 60
            quaternions.add(new Quaternion(0.000000f, -0.500000f, 0.000000f, 0.866025f)); // Yaw: 0, Pitch: -60
            quaternions.add(new Quaternion(0.000000f, 0.000000f, 0.382683f, 0.923880f)); // Yaw: 45, Pitch: 0
            quaternions.add(new Quaternion(0.000000f, 0.000000f, 0.382683f, -0.923880f)); // Yaw: 315, Pitch: 0
            quaternions.add(new Quaternion(0.000000f, 0.000000f, 0.923880f, -0.382683f)); // Yaw: 225, Pitch: 0
            quaternions.add(new Quaternion(0.000000f, 0.000000f, 0.923880f, 0.382683f)); // Yaw: 135, Pitch: 0
            quaternions.add(new Quaternion(0.500000f, 0.000000f, 0.866025f, 0.000000f)); // Yaw: 180, Pitch: 60
            quaternions.add(new Quaternion(-0.500000f, -0.000000f, 0.866025f, 0.000000f)); // Yaw: 180, Pitch: -60
         */
        switch (areaIdx) {
            //หมุนน้อยสุด
            case 0:
            quaternions.add(new Quaternion(0.000000f, 0.500000f, 0.000000f, 0.866025f)); // Yaw: 0, Pitch: 60
            quaternions.add(new Quaternion(0.000000f, -0.500000f, 0.000000f, 0.866025f)); // Yaw: 0, Pitch: -60
            quaternions.add(new Quaternion(0.000000f, 0.000000f, 0.382683f, 0.923880f)); // Yaw: 45, Pitch: 0
            quaternions.add(new Quaternion(0.000000f, 0.000000f, 0.382683f, -0.923880f)); // Yaw: 315, Pitch: 0
            quaternions.add(new Quaternion(0.000000f, 0.000000f, 0.923880f, -0.382683f)); // Yaw: 225, Pitch: 0
            quaternions.add(new Quaternion(0.000000f, 0.000000f, 0.923880f, 0.382683f)); // Yaw: 135, Pitch: 0
            quaternions.add(new Quaternion(0.500000f, 0.000000f, 0.866025f, 0.000000f)); // Yaw: 180, Pitch: 60
            quaternions.add(new Quaternion(-0.500000f, -0.000000f, 0.866025f, 0.000000f)); // Yaw: 180, Pitch: -60
                break;
            case 1:
            quaternions.add(new Quaternion(-0.500000f, -0.000000f, 0.866025f, 0.000000f)); // Yaw: 180, Pitch: -60
            quaternions.add(new Quaternion(0.500000f, 0.000000f, 0.866025f, 0.000000f)); // Yaw: 180, Pitch: 60
            quaternions.add(new Quaternion(0.000000f, 0.000000f, 0.923880f, 0.382683f)); // Yaw: 135, Pitch: 0
            quaternions.add(new Quaternion(0.000000f, 0.000000f, 0.923880f, -0.382683f)); // Yaw: 225, Pitch: 0
            quaternions.add(new Quaternion(0.000000f, 0.000000f, 0.382683f, -0.923880f)); // Yaw: 315, Pitch: 0
            quaternions.add(new Quaternion(0.000000f, 0.000000f, 0.382683f, 0.923880f)); // Yaw: 45, Pitch: 0
            quaternions.add(new Quaternion(0.000000f, -0.500000f, 0.000000f, 0.866025f)); // Yaw: 0, Pitch: -60
            quaternions.add(new Quaternion(0.000000f, 0.500000f, 0.000000f, 0.866025f)); // Yaw: 0, Pitch: 60
                break;
            case 2:
                quaternions.add(new Quaternion(0.000000f, 0.500000f, 0.000000f, 0.866025f)); // Yaw: 0, Pitch: 60
                quaternions.add(new Quaternion(0.000000f, -0.500000f, 0.000000f, 0.866025f)); // Yaw: 0, Pitch: -60
                quaternions.add(new Quaternion(0.000000f, 0.000000f, 0.382683f, 0.923880f)); // Yaw: 45, Pitch: 0
                quaternions.add(new Quaternion(0.000000f, 0.000000f, 0.382683f, -0.923880f)); // Yaw: 315, Pitch: 0
                quaternions.add(new Quaternion(0.000000f, 0.000000f, 0.923880f, -0.382683f)); // Yaw: 225, Pitch: 0
                quaternions.add(new Quaternion(0.000000f, 0.000000f, 0.923880f, 0.382683f)); // Yaw: 135, Pitch: 0
                quaternions.add(new Quaternion(0.500000f, 0.000000f, 0.866025f, 0.000000f)); // Yaw: 180, Pitch: 60
                quaternions.add(new Quaternion(-0.500000f, -0.000000f, 0.866025f, 0.000000f)); // Yaw: 180, Pitch: -60
                break;
            case 3:
                quaternions.add(new Quaternion(-0.500000f, -0.000000f, 0.866025f, 0.000000f)); // Yaw: 180, Pitch: -60
                quaternions.add(new Quaternion(0.500000f, 0.000000f, 0.866025f, 0.000000f)); // Yaw: 180, Pitch: 60
                quaternions.add(new Quaternion(0.000000f, 0.000000f, 0.923880f, 0.382683f)); // Yaw: 135, Pitch: 0
                quaternions.add(new Quaternion(0.000000f, 0.000000f, 0.923880f, -0.382683f)); // Yaw: 225, Pitch: 0
                quaternions.add(new Quaternion(0.000000f, 0.000000f, 0.382683f, -0.923880f)); // Yaw: 315, Pitch: 0
                quaternions.add(new Quaternion(0.000000f, 0.000000f, 0.382683f, 0.923880f)); // Yaw: 45, Pitch: 0
                quaternions.add(new Quaternion(0.000000f, -0.500000f, 0.000000f, 0.866025f)); // Yaw: 0, Pitch: -60
                quaternions.add(new Quaternion(0.000000f, 0.500000f, 0.000000f, 0.866025f)); // Yaw: 0, Pitch: 60
                break;

            default:
                break;
        }

        // --- Pitch = 60 degrees ---
        /*
         * quaternions.add(new Quaternion(0.000000f, 0.500000f, 0.000000f, 0.866025f));
         * // Yaw: 0, Pitch: 60
         * quaternions.add(new Quaternion(0.500000f, 0.000000f, 0.866025f, 0.000000f));
         * // Yaw: 180, Pitch: 60
         * 
         * // --- Pitch = 0 degrees ---
         * quaternions.add(new Quaternion(0.000000f, 0.000000f, 0.382683f, 0.923880f));
         * // Yaw: 45, Pitch: 0
         * quaternions.add(new Quaternion(0.000000f, 0.000000f, 0.382683f, -0.923880f));
         * // Yaw: 315, Pitch: 0
         * quaternions.add(new Quaternion(0.000000f, 0.000000f, 0.923880f, -0.382683f));
         * // Yaw: 225, Pitch: 0
         * quaternions.add(new Quaternion(0.000000f, 0.000000f, 0.923880f, 0.382683f));
         * // Yaw: 135, Pitch: 0
         * 
         * // --- Pitch = -60 degrees ---
         * quaternions.add(new Quaternion(0.000000f, -0.500000f, 0.000000f, 0.866025f));
         * // Yaw: 0, Pitch: -60
         * quaternions.add(new Quaternion(-0.500000f, -0.000000f, 0.866025f,
         * 0.000000f)); // Yaw: 180, Pitch: -60
         */
        return quaternions;
    }

    public static Mat captureImageAt(KiboRpcApi api, Point point, Quaternion quaternion) {
        api.moveTo(point, quaternion, false);
        Mat image = api.getMatNavCam();
        if (image == null) {
            Log.e("OasisUtils", String.format("Failed to capture image at point: %s, quaternion: %s. Image is null.",
                    point, quaternion));
            return null;
        }
        return image;
    }

    public static void saveOasisImage(KiboRpcApi api, int areaIdx, int orientationIdx, Mat image) {
        String fileName = String.format("OasisArea%d_%d.png", areaIdx, orientationIdx);
        api.saveMatImage(image, fileName);
    }

    /**
     * Move the robot closer to the AR marker while keeping the camera angle within
     * 30 degrees.
     * 
     * @param api              KiboRpcApi instance
     * @param tvec             double[3] translation vector from pose estimation
     *                         (camera to marker, in camera frame)
     * @param currentQuat      current robot orientation (world frame)
     * @param approachDistance desired distance to marker (meters)
     * @return true if move command issued, false if already within distance/angle
     */
    public static boolean moveCloserToMarker(KiboRpcApi api, double[] tvec, Quaternion currentQuat,
            double approachDistance) {
        if (tvec == null || tvec.length != 3)
            return false;
        double dx = tvec[0];
        double dy = tvec[1];
        double dz = tvec[2];
        double distance = Math.sqrt(dx * dx + dy * dy + dz * dz);
        if (distance <= approachDistance)
            return false; // Already close enough
        double angle = Math.toDegrees(Math.acos(dz / distance));
        if (angle > 30.0)
            return false; // Angle constraint violated
        // Compute move vector in camera/robot frame
        double scale = (distance - approachDistance) / distance;
        double moveX = dx * scale;
        double moveY = dy * scale;
        double moveZ = dz * scale;
        Point relativeGoal = new Point(moveX, moveY, moveZ);
        api.relativeMoveTo(relativeGoal, currentQuat, false);
        android.util.Log.i("OasisUtils",
                String.format("Relative move closer to AR marker: [%.3f, %.3f, %.3f] (distance: %.3f, angle: %.2f)",
                        moveX, moveY, moveZ, distance, angle));
        return true;
    }

    /**
     * Draw and save the paper boundary for the first detected marker.
     */
    private static void drawAndSavePaperBoundary(Mat image, int areaIdx, int orientationIdx,
            ArMarkerDetector.DetectionResult result, Mat rvecsMat, Mat tvecsMat, Mat[] mats, KiboRpcApi api,
            Map<Integer, DetectedItemInfo> detectedItemsMap) {
        if (result.corners.size() > 0) {
            int arId = (int) result.markerIds.get(0, 0)[0];
            if (!detectedItemsMap.containsKey(arId)) {
                Mat rvec = rvecsMat.row(0);
                Mat tvec = tvecsMat.row(0);
                // Mat cropped =
                // jp.jaxa.iss.kibo.rpc.sampleapk.vision.ArMarkerDetector.cropPaperArea(image,
                // rvec, tvec, mats[0], mats[1]);
                // api.saveMatImage(cropped,
                // String.format("OasisArea%d_%d_AR%d_paper_boundary.png", areaIdx,
                // orientationIdx, arId));
                // Call drawPaperBoundary instead
                jp.jaxa.iss.kibo.rpc.sampleapk.vision.ArMarkerDetector.drawPaperBoundary(image, rvec, tvec, mats[0],
                        mats[1]);
                api.saveMatImage(image,
                        String.format("OasisArea%d_%d_AR%d_paper_boundary.png", areaIdx, orientationIdx, arId));
            }
        }
    }

    /**
     * Detect and store all new AR markers found in the image, return number of new
     * markers stored.
     */
    private static int detectAndStoreAllArMarkers(
            Mat image,
            int areaIdx,
            int orientationIdx,
            Point point,
            Quaternion orientation,
            KiboRpcApi api,
            Map<Integer, DetectedItemInfo> detectedItemsMap) {
        int newMarkers = 0;
        ArMarkerDetector.DetectionResult result = ArMarkerDetector.detectAndDrawMarkers(image,
                org.opencv.aruco.Aruco.DICT_5X5_250);
        if (!result.hasMarkers())
            return 0;
        float markerLength = 0.0575f;
        double[][] navCamIntrinsics = api.getNavCamIntrinsics();
        Mat[] mats = ArMarkerDetector.extractCameraIntrinsics(navCamIntrinsics[0], navCamIntrinsics[1]);
        Mat rvecsMat = new Mat();
        Mat tvecsMat = new Mat();
        org.opencv.aruco.Aruco.estimatePoseSingleMarkers(result.corners, markerLength, mats[0], mats[1], rvecsMat,
                tvecsMat);
        drawAndSavePaperBoundary(image, areaIdx, orientationIdx, result, rvecsMat, tvecsMat, mats, api,
                detectedItemsMap);
        for (int j = 0; j < result.markerIds.rows(); j++) {
            int arId = (int) result.markerIds.get(j, 0)[0];
            if (detectedItemsMap.containsKey(arId))
                continue; // Only store new markers
            double[] tvec = tvecsMat.get(j, 0);
            double dx = tvec[0], dy = tvec[1], dz = tvec[2];
            double distance = Math.sqrt(dx * dx + dy * dy + dz * dz);
            double angle = Math.toDegrees(Math.acos(dz / distance));
            String fileName = String.format("OasisArea%d_%d_AR%d.png", areaIdx, orientationIdx, arId);
            api.saveMatImage(image, fileName);
            java.util.List<String> items = new java.util.ArrayList<>();
            DetectedItemInfo info = new DetectedItemInfo(
                    areaIdx, orientationIdx, point, orientation, arId,
                    Arrays.copyOf(tvec, tvec.length), angle, items, fileName);
            detectedItemsMap.put(arId, info);
            Log.i("OasisUtils",
                    String.format("AR marker FOUND (ID %d) in area %d, orientation %d (point: %s, quaternion: %s)",
                            arId, areaIdx, orientationIdx, point.toString(), orientation.toString()));
            newMarkers++;
        }
        return newMarkers;
    }

    /**
     * Scan the oasis area for AR markers, save only valid images, and store
     * detection info by AR ID.
     * Uses moveCloserToMarker for constraint handling. Easy to maintain and extend.
     * 
     * @param api              KiboRpcApi instance
     * @param areaIdx          Area index
     * @param oasisPoints      List of area points
     * @param oasisQuaternions List of orientations
     * @param detectedItemsMap Map to store DetectedItemInfo by AR marker ID
     * @param visitedArIds     Set of visited AR marker IDs (should be shared across
     *                         all scans)
     * @return number of AR markers found
     */
    public static int scanOasisArea(KiboRpcApi api, int areaIdx, List<Point> oasisPoints, Map<Integer, DetectedItemInfo> detectedItemsMap,
            java.util.Set<Integer> visitedArIds, int totalAR) {
        List<Quaternion>    oasisQuaternions = getOasisQuaternions(areaIdx);
        int totalNewMarkers = 0;
        for (int i = 0; i < oasisQuaternions.size(); i++) {
            Mat image = captureImageAt(api, oasisPoints.get(areaIdx), oasisQuaternions.get(i));
            if (image == null) {
                Log.w("OasisUtils", "Captured image is empty; skipping orientation " + i);
                continue;
            }
            int newMarkers = detectAndStoreAllArMarkers(image, areaIdx, i, oasisPoints.get(areaIdx),
                    oasisQuaternions.get(i), api, detectedItemsMap);
            totalNewMarkers += newMarkers;
            // Visit all unvisited AR markers after each detection
            visitAllUnvisitedArMarkers(api, detectedItemsMap, visitedArIds);
            if (totalAR + newMarkers == 4 || (newMarkers > 0 && areaIdx != 3)) break; // Early exit after first detection(s) in this
            // area
        }
        return totalNewMarkers;
    }

    /**
     * Check if an AR marker ID has been visited.
     * 
     * @param arId         AR marker ID
     * @param visitedArIds Set of visited AR marker IDs
     * @return true if visited, false otherwise
     */
    public static boolean isArVisited(int arId, java.util.Set<Integer> visitedArIds) {
        return visitedArIds.contains(arId);
    }

    /**
     * Compute the AR marker's world position from the robot's pose and tvec (marker
     * position in camera frame).
     * 
     * @param robotPos  Robot's world position (Point)
     * @param robotQuat Robot's world orientation (Quaternion)
     * @param tvec      Marker position in camera frame (double[3])
     * @return AR marker's world position (Point)
     */
    public static Point computeArMarkerWorldPosition(Point robotPos, Quaternion robotQuat, double[] tvec) {
        // Convert quaternion to rotation matrix
        double x = robotQuat.getX();
        double y = robotQuat.getY();
        double z = robotQuat.getZ();
        double w = robotQuat.getW();
        double[][] R = new double[3][3];
        R[0][0] = 1 - 2 * y * y - 2 * z * z;
        R[0][1] = 2 * x * y - 2 * z * w;
        R[0][2] = 2 * x * z + 2 * y * w;
        R[1][0] = 2 * x * y + 2 * z * w;
        R[1][1] = 1 - 2 * x * x - 2 * z * z;
        R[1][2] = 2 * y * z - 2 * x * w;
        R[2][0] = 2 * x * z - 2 * y * w;
        R[2][1] = 2 * y * z + 2 * x * w;
        R[2][2] = 1 - 2 * x * x - 2 * y * y;
        // Transform tvec from camera to world frame
        double wx = robotPos.getX() + R[0][0] * tvec[0] + R[0][1] * tvec[1] + R[0][2] * tvec[2];
        double wy = robotPos.getY() + R[1][0] * tvec[0] + R[1][1] * tvec[1] + R[1][2] * tvec[2];
        double wz = robotPos.getZ() + R[2][0] * tvec[0] + R[2][1] * tvec[1] + R[2][2] * tvec[2];
        return new Point(wx, wy, wz);
    }

    /**
     * Visit each unvisited AR marker in the detectedItemsMap.
     * Moves to each AR marker's position and orientation, captures an image, and
     * marks it as visited.
     * 
     * @param api              KiboRpcApi instance
     * @param detectedItemsMap Map of AR marker ID to DetectedItemInfo
     * @param visitedArIds     Set of visited AR marker IDs (will be updated)
     */
    public static void visitAllUnvisitedArMarkers(
            KiboRpcApi api,
            Map<Integer, DetectedItemInfo> detectedItemsMap,
            java.util.Set<Integer> visitedArIds) {
        final double DESIRED_DISTANCE = 0.7; // meters (adjust as needed)
        for (DetectedItemInfo info : detectedItemsMap.values()) {
            if (isArVisited(info.arId, visitedArIds))
                continue;
            // Compute AR marker world position
            Point arWorldPos = computeArMarkerWorldPosition(info.point, info.orientation, info.tvec);
            // Compute approach point (DESIRED_DISTANCE away from marker, along camera Z
            // axis)
            // For simplicity, approach along the same direction as the robot's orientation
            double dx = info.point.getX() - arWorldPos.getX();
            double dy = info.point.getY() - arWorldPos.getY();
            double dz = info.point.getZ() - arWorldPos.getZ();
            double dist = Math.sqrt(dx * dx + dy * dy + dz * dz);
            double ax = arWorldPos.getX() + (dx / dist) * DESIRED_DISTANCE;
            double ay = arWorldPos.getY() + (dy / dist) * DESIRED_DISTANCE;
            double az = arWorldPos.getZ() + (dz / dist) * DESIRED_DISTANCE;
            Point approachPoint = new Point(ax, ay, az);
            // Move to approach point and orientation
            captureImageAt(api, approachPoint, info.orientation);
            // Mark as visited
            visitedArIds.add(info.arId);
            // (Optional) Process the image or perform additional logic here
        }
    }

    // The following methods require access to the api instance, so should be called
    // from YourService with api passed in.
}
