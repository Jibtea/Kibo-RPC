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
 * Utility class for Oasis-related operations (points, quaternions, scanning, image capture, saving).
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

    public static List<Quaternion> getOasisQuaternions() {
        List<Quaternion> quaternions = new ArrayList<>();
        quaternions.add(new Quaternion(0f, 0f, -0.707f, 0.707f));
        quaternions.add(new Quaternion(0.707f, 0f, 0.707f, 0.707f));
        quaternions.add(new Quaternion(0f, -0.707f, 0.707f, -0.707f));
        quaternions.add(new Quaternion(0.707f, 0.707f, -0.707f, -0.707f));
        return quaternions;
    }

    public static Mat captureImageAt(KiboRpcApi api, Point point, Quaternion quaternion) {
        api.moveTo(point, quaternion, false);
        Mat image = api.getMatNavCam();
        if (image == null) {
            Log.e("OasisUtils", String.format("Failed to capture image at point: %s, quaternion: %s. Image is null.", point, quaternion));
            return null;
        }
        return image;
    }

    public static void saveOasisImage(KiboRpcApi api, int areaIdx, int orientationIdx, Mat image) {
        String fileName = String.format("OasisArea%d_%d.png", areaIdx, orientationIdx);
        api.saveMatImage(image, fileName);
    }

    /**
     * Move the robot closer to the AR marker while keeping the camera angle within 30 degrees.
     * @param api KiboRpcApi instance
     * @param tvec double[3] translation vector from pose estimation (camera to marker, in camera frame)
     * @param currentQuat current robot orientation (world frame)
     * @param approachDistance desired distance to marker (meters)
     * @return true if move command issued, false if already within distance/angle
     */
    public static boolean moveCloserToMarker(KiboRpcApi api, double[] tvec, Quaternion currentQuat, double approachDistance) {
        if (tvec == null || tvec.length != 3) return false;
        double dx = tvec[0];
        double dy = tvec[1];
        double dz = tvec[2];
        double distance = Math.sqrt(dx*dx + dy*dy + dz*dz);
        if (distance <= approachDistance) return false; // Already close enough
        double angle = Math.toDegrees(Math.acos(dz / distance));
        if (angle > 30.0) return false; // Angle constraint violated
        // Compute move vector in camera/robot frame
        double scale = (distance - approachDistance) / distance;
        double moveX = dx * scale;
        double moveY = dy * scale;
        double moveZ = dz * scale;
        Point relativeGoal = new Point(moveX, moveY, moveZ);
        api.relativeMoveTo(relativeGoal, currentQuat, false);
        android.util.Log.i("OasisUtils", String.format("Relative move closer to AR marker: [%.3f, %.3f, %.3f] (distance: %.3f, angle: %.2f)",
            moveX, moveY, moveZ, distance, angle));
        return true;
    }

    /**
     * Detect and store all new AR markers found in the image, return number of new markers stored.
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
        ArMarkerDetector.DetectionResult result = ArMarkerDetector.detectAndDrawMarkers(image, org.opencv.aruco.Aruco.DICT_5X5_250);
        if (!result.hasMarkers()) return 0;
        float markerLength = 0.0575f;
        double[][] navCamIntrinsics = api.getNavCamIntrinsics();
        Mat[] mats = ArMarkerDetector.extractCameraIntrinsics(navCamIntrinsics[0], navCamIntrinsics[1]);
        Mat rvecsMat = new Mat();
        Mat tvecsMat = new Mat();
        org.opencv.aruco.Aruco.estimatePoseSingleMarkers(result.corners, markerLength, mats[0], mats[1], rvecsMat, tvecsMat);
        for (int j = 0; j < result.markerIds.rows(); j++) {
            int arId = (int)result.markerIds.get(j, 0)[0];
            if (detectedItemsMap.containsKey(arId)) continue; // Only store new markers
            double[] tvec = tvecsMat.get(j, 0);
            double dx = tvec[0], dy = tvec[1], dz = tvec[2];
            double distance = Math.sqrt(dx*dx + dy*dy + dz*dz);
            double angle = Math.toDegrees(Math.acos(dz / distance));
            String fileName = String.format("OasisArea%d_%d_AR%d.png", areaIdx, orientationIdx, arId);
            api.saveMatImage(image, fileName);
            java.util.List<String> items = new java.util.ArrayList<>();
            DetectedItemInfo info = new DetectedItemInfo(
                areaIdx, orientationIdx, point, orientation, arId,
                Arrays.copyOf(tvec, tvec.length), angle, items, fileName
            );
            detectedItemsMap.put(arId, info);
            Log.i("OasisUtils", String.format("AR marker FOUND (ID %d) in area %d, orientation %d (point: %s, quaternion: %s)", arId, areaIdx, orientationIdx, point.toString(), orientation.toString()));
            newMarkers++;
        }
        return newMarkers;
    }

    /**
     * Scan the oasis area for AR markers, save only valid images, and store detection info by AR ID.
     * Uses moveCloserToMarker for constraint handling. Easy to maintain and extend.
     * @param api KiboRpcApi instance
     * @param areaIdx Area index
     * @param oasisPoints List of area points
     * @param oasisQuaternions List of orientations
     * @param detectedItemsMap Map to store DetectedItemInfo by AR marker ID
     * @return number of AR markers found
     */
    public static int scanOasisArea(KiboRpcApi api, int areaIdx, List<Point> oasisPoints, List<Quaternion> oasisQuaternions, Map<Integer, DetectedItemInfo> detectedItemsMap) {
        int totalNewMarkers = 0;
        for (int i = 0; i < oasisQuaternions.size(); i++) {
            Mat image = captureImageAt(api, oasisPoints.get(areaIdx), oasisQuaternions.get(i));
            if (image == null) {
                Log.w("OasisUtils", "Captured image is empty; skipping orientation " + i);
                continue;
            }
            int newMarkers = detectAndStoreAllArMarkers(image, areaIdx, i, oasisPoints.get(areaIdx), oasisQuaternions.get(i), api, detectedItemsMap);
            totalNewMarkers += newMarkers;
            if (newMarkers > 0) break; // Early exit after first detection(s) in this area
        }
        return totalNewMarkers;
    }

    // The following methods require access to the api instance, so should be called from YourService with api passed in.
}
