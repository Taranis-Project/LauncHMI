from PIL import Image, ImageDraw, ImageFont

def create_transparent_image(width, height, text, output_file):
    # Create a new transparent image (RGBA mode)
    img = Image.new("RGBA", (width, height), (0, 0, 0, 0))  # RGBA mode allows transparency

    # Initialize ImageDraw to draw on the image
    draw = ImageDraw.Draw(img)

    # Try to load a basic font (Arial)
    try:
        font = ImageFont.truetype("arial.ttf", 30)  # Change path if needed
    except IOError:
        font = ImageFont.load_default()

    # Get the size of the text to center it using the textbbox method
    text_bbox = draw.textbbox((0, 0), text, font=font)
    text_width = text_bbox[2] - text_bbox[0]
    text_height = text_bbox[3] - text_bbox[1]

    # Calculate the position to center the text
    text_x = (width - text_width) / 2
    text_y = (height - text_height) / 2

    # Draw the text on the image (black color for text)
    draw.text((text_x, text_y), text, font=font, fill="black")

    # Save the image as a PNG (supports transparency)
    img.save(output_file)

# Generate a transparent image with text
create_transparent_image(200, 50, "       ", "transparent_button.png")