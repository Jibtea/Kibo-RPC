package jp.jaxa.iss.kibo.rpc.sampleapk.vision;

import jp.jaxa.iss.kibo.rpc.sampleapk.utils.ImageUtils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import java.util.ArrayList;
import java.util.List;

public class TemplateMatcher {
    public static int[] matchTemplates(Mat undistortImg, Mat[] templates) {
        int[] templateMatchCnt = new int[templates.length];
        for (int tempNum = 0; tempNum < templates.length; tempNum++) {
            int matchCnt = 0;
            List<org.opencv.core.Point> matches = new ArrayList<>();
            Mat template = templates[tempNum].clone();
            Mat targetImg = undistortImg.clone();
            int widthMin = 20;
            int widthMax = 100;
            int changeWidth = 5;
            int changeAngle = 45;
            for (int size = widthMin; size < widthMax; size += changeWidth) {
                for (int angle = 0; angle < 360; angle += changeAngle) {
                    Mat resizedTemp = ImageUtils.resizeImg(template, size);
                    Mat rotResizedTemp = ImageUtils.rotImg(resizedTemp, angle);
                    Mat result = new Mat();
                    Imgproc.matchTemplate(targetImg, rotResizedTemp, result, Imgproc.TM_CCOEFF_NORMED);
                    double threshold = 0.7;
                    Core.MinMaxLocResult mmlr = Core.minMaxLoc(result);
                    double maxVal = mmlr.maxVal;
                    if (maxVal >= threshold) {
                        Mat thresholdedResult = new Mat();
                        Imgproc.threshold(result, thresholdedResult, threshold, 1.0, Imgproc.THRESH_TOZERO);
                        for (int y = 0; y < thresholdedResult.rows(); y++) {
                            for (int x = 0; x < thresholdedResult.cols(); x++) {
                                if (thresholdedResult.get(y, x)[0] > 0) {
                                    matches.add(new org.opencv.core.Point(x, y));
                                }
                            }
                        }
                    }
                }
            }
            List<org.opencv.core.Point> filteredMatches = ImageUtils.removeDuplicates(matches);
            matchCnt += filteredMatches.size();
            templateMatchCnt[tempNum] = matchCnt;
        }
        return templateMatchCnt;
    }
}
