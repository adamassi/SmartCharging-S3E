from PIL import Image, ImageDraw, ImageFont
from datetime import datetime

def create_date_image(output_path, font_path="arial.ttf", font_size=40):
    """
    Creates a PNG file containing the current date.

    :param output_path: Path to save the output PNG file.
    :param font_path: Path to the font file (default is 'arial.ttf').
    :param font_size: Font size for the date text (default is 50).
    """
    # Get the current date
    current_date = datetime.now().strftime("%Y-%m-%d")

    # Create a blank image (white background)
    image_size = (1224, 1224)  # Width x Height
    image = Image.new("RGBA", image_size, "white")

    # Create a drawing context
    draw = ImageDraw.Draw(image)

    # Define font (fallback to default if arial not available)
    try:
        font = ImageFont.truetype(font_path, font_size)
    except IOError:
        font = ImageFont.load_default()

    # Calculate text position (center)
    bbox = draw.textbbox((0, 0), current_date, font=font)
    text_width, text_height = bbox[2] - bbox[0], bbox[3] - bbox[1]
    x = (image.width - text_width) // 2
    y = (image.height - text_height) // 2

    # Draw the date onto the image
    draw.text((x, y), current_date, fill="black", font=font)

    # Save the image
    image.save(output_path, format="PNG")

# Example usage:
create_date_image("dated_image3.png")
create_date_image("sim_ur5/mujoco_env/assets/objects/battery_AA/dated_image3.png")