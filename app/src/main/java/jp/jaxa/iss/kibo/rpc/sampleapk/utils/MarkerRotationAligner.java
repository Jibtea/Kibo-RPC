package jp.jaxa.iss.kibo.rpc.sampleapk.vision;

import jp.jaxa.iss.kibo.rpc.api.KiboRpcApi;
import jp.jaxa.iss.kibo.rpc.lib.Quaternion;
import org.opencv.core.Mat;
import org.opencv.core.Point;

import java.util.List;

public class MarkerRotationAligner {

    private final KiboRpcApi api;
    private final int dictionaryId;

    public MarkerRotationAligner(KiboRpcApi api, int dictionaryId) {
        this.api = api;
        this.dictionaryId = dictionaryId;
    }

    public void rotateUntilMarkerTopRight() {
        final double yawStep = Math.toRadians(5);
        final int maxTries = 72;
        int tries = 0;

        while (tries < maxTries) {
            Mat image = api.getMatNavCam();
            MarkerDetectionResult result = ArMarkerDetector.detectMarkersWithIds(image, dictionaryId);

            if (!result.corners.isEmpty()) {
                Point center = getMarkerCenter(result.corners.get(0));

                if (isTopRight(image, center)) {
                    api.reportMessage("✅ Marker is at top-right.");
                    break;
                } else {
                    api.reportMessage("🔄 Marker found, rotating...");
                }
            } else {
                api.reportMessage("❌ Marker not found. Rotating...");
            }

            Quaternion currentOrientation = api.getRobotOrientation();
            Quaternion newOrientation = rotateYaw(currentOrientation, yawStep);
            api.moveTo(api.getRobotPosition(), newOrientation, true);
            tries++;
        }

        if (tries == maxTries) {
            api.reportMessage("⚠️ Max tries reached, marker not aligned.");
        }
    }

    private Point getMarkerCenter(Mat cornerMat) {
        double cx = 0, cy = 0;
        for (int i = 0; i < 4; i++) {
            double[] pt = cornerMat.get(0, i);
            cx += pt[0];
            cy += pt[1];
        }
        return new Point(cx / 4.0, cy / 4.0);
    }

    private boolean isTopRight(Mat image, Point center) {
        int width = image.cols();
        int height = image.rows();
        return center.x > width * 0.6 && center.y < height * 0.4;
    }

    private Quaternion rotateYaw(Quaternion q, double deltaYaw) {
        double yaw = Math.atan2(2.0 * (q.getW() * q.getZ() + q.getX() * q.getY()),
                                1.0 - 2.0 * (q.getY() * q.getY() + q.getZ() * q.getZ()));
        double newYaw = yaw + deltaYaw;
        double cy = Math.cos(newYaw * 0.5);
        double sy = Math.sin(newYaw * 0.5);
        return new Quaternion(0, 0, sy, cy);
    }
}
