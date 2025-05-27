package jp.jaxa.iss.kibo.rpc.sampleapk.util;

import android.content.res.AssetManager;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.util.Log;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import org.opencv.android.Utils;
import java.io.IOException;
import java.io.InputStream;
import jp.jaxa.iss.kibo.rpc.sampleapk.vision.TemplateMatcher;
import jp.jaxa.iss.kibo.rpc.sampleapk.utils.ImageUtils;

/**
 * Utility class for area recognition and template loading.
 */
public class AreaRecognitionUtils {
    public static Mat[] loadTemplates(AssetManager assetManager, String[] templateFileNames, String TAG) {
        Mat[] templates = new Mat[templateFileNames.length];
        for (int i = 0; i < templateFileNames.length; i++) {
            try (InputStream inputStream = assetManager.open(templateFileNames[i])) {
                Bitmap bitmap = BitmapFactory.decodeStream(inputStream);
                Mat mat = new Mat();
                Utils.bitmapToMat(bitmap, mat);
                Imgproc.cvtColor(mat, mat, Imgproc.COLOR_BGR2GRAY);
                templates[i] = mat;
            } catch (IOException e) {
                Log.e(TAG, "Failed to load template: " + templateFileNames[i], e);
            }
        }
        return templates;
    }
}
