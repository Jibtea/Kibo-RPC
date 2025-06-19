package jp.jaxa.iss.kibo.rpc.sampleapk.vision;

import android.content.Context;
import org.tensorflow.lite.Interpreter;
import java.io.FileInputStream;
import java.io.IOException;
import java.nio.MappedByteBuffer;
import java.nio.channels.FileChannel;
import android.content.res.AssetFileDescriptor;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import java.util.HashMap;
import java.util.Map;

public class TFLiteObjectDetector {
    private Interpreter tflite;
    private static final int INPUT_SIZE = 300; // Change to your model's input size
    private static final int NUM_CLASSES = 11; // Number of item classes
    public static final String[] ITEM_LABELS = {
        "coin", "compass", "coral", "crystal", "emerald",
        "fossil", "key", "letter", "shell", "treasure_box", "item11"
    };

    public TFLiteObjectDetector(Context context) throws IOException {
        tflite = new Interpreter(loadModelFile(context, "model/model.tflite"));
    }

    private MappedByteBuffer loadModelFile(Context context, String modelPath) throws IOException {
        AssetFileDescriptor fileDescriptor = context.getAssets().openFd(modelPath);
        FileInputStream inputStream = new FileInputStream(fileDescriptor.getFileDescriptor());
        FileChannel fileChannel = inputStream.getChannel();
        long startOffset = fileDescriptor.getStartOffset();
        long declaredLength = fileDescriptor.getDeclaredLength();
        return fileChannel.map(FileChannel.MapMode.READ_ONLY, startOffset, declaredLength);
    }

    // Preprocess OpenCV Mat to float array for TFLite
    private float[][][][] preprocess(Mat mat) {
        Mat resized = new Mat();
        Imgproc.resize(mat, resized, new org.opencv.core.Size(INPUT_SIZE, INPUT_SIZE));
        float[][][][] input = new float[1][INPUT_SIZE][INPUT_SIZE][3];
        for (int y = 0; y < INPUT_SIZE; y++) {
            for (int x = 0; x < INPUT_SIZE; x++) {
                double[] pixel = resized.get(y, x);
                input[0][y][x][0] = (float) (pixel[0] / 255.0); // B
                input[0][y][x][1] = (float) (pixel[1] / 255.0); // G
                input[0][y][x][2] = (float) (pixel[2] / 255.0); // R
            }
        }
        return input;
    }

    // Run inference and return detection results
    public Map<String, Float> detect(Mat mat) {
        float[][][][] input = preprocess(mat);
        // Example output arrays for SSD MobileNet (adjust for your model)
        float[][][] boxes = new float[1][10][4];
        float[][] scores = new float[1][10];
        float[][] classes = new float[1][10];
        float[] count = new float[1];
        Object[] inputs = {input};
        Map<Integer, Object> outputs = new HashMap<>();
        outputs.put(0, boxes);
        outputs.put(1, classes);
        outputs.put(2, scores);
        outputs.put(3, count);
        tflite.runForMultipleInputsOutputs(inputs, outputs);
        // Parse results (return label and score for each detection above threshold)
        Map<String, Float> detected = new HashMap<>();
        for (int i = 0; i < (int) count[0]; i++) {
            int classIdx = (int) classes[0][i];
            float score = scores[0][i];
            if (score > 0.5f && classIdx >= 0 && classIdx < ITEM_LABELS.length) {
                detected.put(ITEM_LABELS[classIdx], score);
            }
        }
        return detected;
    }
}
