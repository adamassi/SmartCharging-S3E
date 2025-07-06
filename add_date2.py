from PIL import Image, ImageDraw, ImageFont
from datetime import datetime

def add_date_to_new_image(output_path, size=(256, 256), background="green"):
    # Create a blank image
    image = Image.new("RGBA", size, background)

    # Get the current date
    current_date = datetime.now().strftime("%Y-%m-%d")

    # Draw the date on the image
    draw = ImageDraw.Draw(image)
    try:
        font = ImageFont.truetype("arial.ttf", 20)
    except IOError:
        font = ImageFont.load_default()

    bbox = draw.textbbox((0, 0), current_date, font=font)
    text_width, text_height = bbox[2] - bbox[0], bbox[3] - bbox[1]
    x = image.width - text_width - 10
    y = image.height - text_height - 10
    draw.text((x, y), current_date, fill="black", font=font)

    # Save it
    image.save(output_path, format="PNG")

# Always generate from scratch
add_date_to_new_image("dated_image.png")
