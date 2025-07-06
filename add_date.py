from PIL import Image, ImageDraw, ImageFont
from datetime import datetime

def add_date_to_image(image_path, output_path, font_path="arial.ttf", font_size=25):
    """
    Resizes the image to 125x125 pixels, adds the current date to the center, and saves it.

    :param image_path: Path to the input image file.
    :param output_path: Path to save the output image file.
    :param font_path: Path to the font file (default is 'arial.ttf').
    :param font_size: Font size for the date text (default is 20).
    """
    # Load the original image and resize it to 125x125 pixels
    original_image = Image.open(image_path)
    original_image.save("copy.png")  # Save a copy of the original image
    resized_image = original_image.resize((125, 125)).convert("RGBA")  # Resize and convert

    # Get the current date
    current_date = datetime.now().strftime("%Y-%m-%d")

    # Create a drawing context
    draw = ImageDraw.Draw(resized_image)

    # Define font (fallback to default if arial not available)
    try:
        font = ImageFont.truetype(font_path, font_size)
    except IOError:
        font = ImageFont.load_default()

    # Calculate text position (center)
    bbox = draw.textbbox((0, 0), current_date, font=font)
    text_width, text_height = bbox[2] - bbox[0], bbox[3] - bbox[1]
    x = (resized_image.width - text_width) // 2
    y = (resized_image.height - text_height) // 2

    # Draw the text onto the image
    draw.text((x, y), current_date, fill="black", font=font)

    # Save the resized image with the date
    resized_image.save(output_path, format="PNG")

# Example usage:
add_date_to_image("battery_texture_upgraded.png", "sim_ur5/mujoco_env/assets/objects/battery_AA/dated_image.png")
add_date_to_image("battery_texture_upgraded.png", "dated_image.png")
