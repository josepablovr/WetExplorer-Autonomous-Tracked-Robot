import cv2
from tflite_support.task import core
from tflite_support.task import processor
from tflite_support.task import vision
import utils

def evaluate_image(model_path: str, image_path: str) -> None:
    """Evaluate a single image using a TFLite model.
    
    Args:
        model_path: Path to the TFLite object detection model.
        image_path: Path to the input image.
    """
    # Initialize the object detection model
    base_options = core.BaseOptions(file_name=model_path)
    detection_options = processor.DetectionOptions(max_results=3, score_threshold=0.7)
    options = vision.ObjectDetectorOptions(
        base_options=base_options, detection_options=detection_options)
    detector = vision.ObjectDetector.create_from_options(options)

    # Read the image
    image = cv2.imread(image_path)
    if image is None:
        raise FileNotFoundError(f"Image not found: {image_path}")

    # Convert the image from BGR to RGB as required by the TFLite model
    rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

    # Create a TensorImage object from the RGB image
    input_tensor = vision.TensorImage.create_from_array(rgb_image)

    # Run object detection
    detection_result = detector.detect(input_tensor)

    # Visualize results on the image
    result_image = utils.visualize(image, detection_result)

    # Display the result
    cv2.imshow("Detection Result", result_image)
    cv2.waitKey(0)  # Wait for a key press to close the window
    cv2.destroyAllWindows()

# Example usage
if __name__ == "__main__":
    model_path = "ring2.tflite"  # Path to the model
    image_path = "t10_Color.png"  # Path to the input image
    evaluate_image(model_path, image_path)
