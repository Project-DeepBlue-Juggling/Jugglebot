import cv2
import numpy as np
from PIL import Image, ImageOps, ImageFilter
import colorsys

"""
Interactive Hue and Saturation Filter

This script allows you to interactively adjust hue and saturation ranges to find the
best filter values for an image. The filtered image is displayed in a live preview
window, which can be adjusted using the trackbars for hue and saturation ranges.

Once you have found the right filter values, you can use the other script
"filter_by_HSV_save_image" to apply the filter to the image and save the output.

Usage:

1. Set the 'image_path' variable to the path of the image you want to process.
2. Run this script, and a window will appear displaying the filtered image.
3. Adjust the hue and saturation range using the trackbars to find the best filter values.
4. Note down the hue and saturation ranges.
5. Press the 'q' key to exit the window.
6. Use the "filter_by_HSV_save_image" script to apply the filter and save the output image.
   Set the hue and saturation range values you noted down in the script.

Note: Make sure to change the 'image_path' variable to suit your image.
"""


def apply_hue_saturation_filter(image, hue_range, saturation_range):
    image = image.convert("RGBA")

    def filter_hue_saturation(pixel):
        r, g, b, a = pixel
        h, l, s = colorsys.rgb_to_hls(r / 255.0, g / 255.0, b / 255.0)
        if hue_range[0] <= h <= hue_range[1] and saturation_range[0] <= s <= saturation_range[1]:
            return (255, 255, 255, a)
        else:
            return (0, 0, 0, a)

    pixels = image.load()
    for i in range(image.size[0]):
        for j in range(image.size[1]):
            pixels[i, j] = filter_hue_saturation(pixels[i, j])

    return image


def main(image_path):
    image = Image.open(image_path)
    hue_range = (0.1, 0.3)
    saturation_range = (1.0, 1.0)

    cv2.namedWindow('Filtered Image', cv2.WINDOW_NORMAL)
    cv2.createTrackbar('Hue Min', 'Filtered Image', int(hue_range[0] * 100), 100, lambda x: None)
    cv2.createTrackbar('Hue Max', 'Filtered Image', int(hue_range[1] * 100), 100, lambda x: None)
    cv2.createTrackbar('Saturation Min', 'Filtered Image', int(saturation_range[0] * 100), 100, lambda x: None)
    cv2.createTrackbar('Saturation Max', 'Filtered Image', int(saturation_range[1] * 100), 100, lambda x: None)

    while True:
        hue_min = cv2.getTrackbarPos('Hue Min', 'Filtered Image') / 100.0
        hue_max = cv2.getTrackbarPos('Hue Max', 'Filtered Image') / 100.0
        sat_min = cv2.getTrackbarPos('Saturation Min', 'Filtered Image') / 100.0
        sat_max = cv2.getTrackbarPos('Saturation Max', 'Filtered Image') / 100.0

        filtered_image = apply_hue_saturation_filter(image, (hue_min, hue_max), (sat_min, sat_max))
        cv2.imshow('Filtered Image', cv2.cvtColor(np.array(filtered_image), cv2.COLOR_RGBA2BGR))

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break

    cv2.destroyAllWindows()


if __name__ == "__main__":
    image_path = "image.jpg"
    main(image_path)
