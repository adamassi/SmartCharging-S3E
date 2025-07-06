from PIL import Image, ImageDraw, ImageFont
from datetime import datetime

def add_date_to_image(image_path, output_path, font_path="arial.ttf", font_size=15):
    """
    Adds the current date to the bottom-right corner of an image and saves it.

    :param image_path: Path to the input image file.
    :param output_path: Path to save the output image file.
    :param font_path: Path to the font file (default is 'arial.ttf').
    :param font_size: Font size for the date text (default is 20).
    """
    # Load the original image
    original_image = Image.open(image_path)  # Copy the original image
    original_image.save("copy.png")
    original_image = Image.open("copy.png")
    image = original_image.copy().convert("RGBA")  # Convert the copied image to RGBA

    # Get the current date
    current_date = datetime.now().strftime("%Y-%m-%d")

    # Create a drawing context
    draw = ImageDraw.Draw(image)

    # Define font (fallback to default if arial not available)
    try:
        font = ImageFont.truetype(font_path, font_size)
    except IOError:
        font = ImageFont.load_default()

    # Calculate text position (bottom-right corner with padding)
    text = current_date
    bbox = draw.textbbox((0, 0), text, font=font)
    text_width, text_height = bbox[2] - bbox[0], bbox[3] - bbox[1]
    padding = 10
    x = image.width - text_width - padding
    y = image.height - text_height - padding

    # Draw the text onto the image
    draw.text((x, y), text, fill="black", font=font)

    # Save the new image
    image.save(output_path, format="PNG")

# Example usage
add_date_to_image("battery_texture_upgraded.png", r"sim_ur5/mujoco_env/assets/objects/battery_AA/dated_image.png")
# add_date_to_image("battery_texture_upgraded.png", "dated_image.png")