package jp.jaxa.iss.kibo.rpc.sampleapk.oasis;

import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;
import jp.jaxa.iss.kibo.rpc.api.KiboRpcApi;
import jp.jaxa.iss.kibo.rpc.sampleapk.vision.ArMarkerDetector;
import org.opencv.core.Mat;
import android.util.Log;
import java.util.ArrayList;
import java.util.List;

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

    public static int scanOasisArea(KiboRpcApi api, int areaIdx, List<Point> oasisPoints, List<Quaternion> oasisQuaternions) {
        int arFound = 0;
        for (int i = 0; i < oasisQuaternions.size(); i++) {
            Mat image = captureImageAt(api, oasisPoints.get(areaIdx), oasisQuaternions.get(i));
            if (image == null) {
                Log.w("OasisUtils", "Captured image is empty; skipping orientxation " + i);
                continue;
            }
            // Detect markers and draw bounding boxes in one step
            ArMarkerDetector.DetectionResult result = ArMarkerDetector.detectAndDrawMarkers(image, org.opencv.aruco.Aruco.DICT_5X5_250);
            saveOasisImage(api, areaIdx, i, image);
            if (result.hasMarkers()) {
                arFound++;
                // Modular: extract intrinsics and log pose
                float markerLength = 0.05f; // Set to your marker's real-world size in meters
                Mat[] mats = ArMarkerDetector.extractCameraIntrinsics(api.getNavCamIntrinsics());
                ArMarkerDetector.logMarkerPose(result, markerLength, mats[0], mats[1], oasisQuaternions.get(i));
                Log.i("OasisUtils", String.format("AR marker FOUND in area %d, orientation %d (point: %s, quaternion: %s)", areaIdx, i, oasisPoints.get(areaIdx).toString(), oasisQuaternions.get(i).toString()));
                break; // Early exit: stop scanning more orientations for this area
            } else {
                Log.i("OasisUtils", String.format("No AR marker found in area %d, orientation %d", areaIdx, i));
            }
        }
        return arFound;
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

    // The following methods require access to the api instance, so should be called from YourService with api passed in.
}
