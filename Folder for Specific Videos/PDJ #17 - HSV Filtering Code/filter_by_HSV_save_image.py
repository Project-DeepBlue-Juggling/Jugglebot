from PIL import Image, ImageOps, ImageFilter
import colorsys


def apply_hue_filter(image_path, hue_range, sat_range):
    image = Image.open(image_path).convert("RGBA")
    hue_min, hue_max = hue_range
    sat_min, sat_max = sat_range

    def filter_hue(pixel):
        r, g, b, a = pixel
        h, l, s = colorsys.rgb_to_hls(r / 255.0, g / 255.0, b / 255.0)
        if hue_min <= h <= hue_max and sat_min <= s <= sat_max:
            return (255, 255, 255, a)
        else:
            return (0, 0, 0, a)

    pixels = image.load()  # Load the pixel data
    for i in range(image.size[0]):  # For each column
        for j in range(image.size[1]):  # For each row
            pixels[i, j] = filter_hue(pixels[i, j])  # Apply the hue filter

    output_path = image_path.rsplit(".", 1)[0] + "_filtered.png"
    image.save(output_path)


# Example usage:
image_path = "image.jpg"
hue_range = (0.41, 0.60)
sat_range = (0.59, 1.0)
apply_hue_filter(image_path, hue_range, sat_range)
