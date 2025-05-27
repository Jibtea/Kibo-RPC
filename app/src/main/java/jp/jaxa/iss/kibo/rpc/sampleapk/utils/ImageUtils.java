package jp.jaxa.iss.kibo.rpc.sampleapk.utils;

import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import java.util.ArrayList;
import java.util.List;

public class ImageUtils {
    // Resize image
    public static Mat resizeImg(Mat img, int width) {
        int height = (int) (img.rows() * ((double) width / img.cols()));
        Mat resizedImg = new Mat();
        Imgproc.resize(img, resizedImg, new Size(width, height));
        return resizedImg;
    }

    // Rotate image
    public static Mat rotImg(Mat img, int angle) {
        org.opencv.core.Point center = new org.opencv.core.Point(img.cols() / 2.0, img.rows() / 2.0);
        Mat rotatedMat = Imgproc.getRotationMatrix2D(center, angle, 1.0);
        Mat rotatedImg = new Mat();
        Imgproc.warpAffine(img, rotatedImg, rotatedMat, img.size());
        return rotatedImg;
    }

    // Remove multiple direction (duplicate points)
    public static List<org.opencv.core.Point> removeDuplicates(List<org.opencv.core.Point> points) {
        double length = 10;
        List<org.opencv.core.Point> filteredList = new ArrayList<>();
        for (org.opencv.core.Point point : points) {
            boolean isInclude = false;
            for (org.opencv.core.Point checkPoint : filteredList) {
                double distance = calculateDistance(point, checkPoint);
                if (distance <= length) {
                    isInclude = true;
                    break;
                }
            }
            if (!isInclude) {
                filteredList.add(point);
            }
        }
        return filteredList;
    }

    // Calculate distance between two points
    public static double calculateDistance(org.opencv.core.Point p1, org.opencv.core.Point p2) {
        double dx = p1.x - p2.x;
        double dy = p1.y - p2.y;
        return Math.sqrt(Math.pow(dx, 2) + Math.pow(dy, 2));
    }

    // Maximum value of array
    public static int getMaxIndex(int[] array) {
        int max = 0;
        int maxIndex = 0;
        for (int i = 0; i < array.length; i++) {
            if (array[i] > max) {
                max = array[i];
                maxIndex = i;
            }
        }
        return maxIndex;
    }

    /**
     * Undistorts the given image using camera intrinsics.
     * @param image Input image
     * @param intrinsics Camera intrinsics (double[2][5] or similar)
     * @return Undistorted image
     */
    public static Mat undistortImage(Mat image, double[][] intrinsics) {
        org.opencv.core.Mat cameraMatrix = new org.opencv.core.Mat(3, 3, org.opencv.core.CvType.CV_64F);
        cameraMatrix.put(0, 0, intrinsics[0]);
        org.opencv.core.Mat cameraCoefficients = new org.opencv.core.Mat(1, 5, org.opencv.core.CvType.CV_64F);
        cameraCoefficients.put(0, 0, intrinsics[1]);
        cameraCoefficients.convertTo(cameraCoefficients, org.opencv.core.CvType.CV_64F);
        org.opencv.core.Mat undistortImg = new org.opencv.core.Mat();
        org.opencv.calib3d.Calib3d.undistort(image, undistortImg, cameraMatrix, cameraCoefficients);
        return undistortImg;
    }
}
