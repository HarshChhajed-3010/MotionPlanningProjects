from PIL import Image

# Load the JPEG file
jpeg_path = "input.jpg"  # Path to the input JPEG file
output_ppm_path = "output.ppm"  # Path for the output PPM file

# Open the image and resize it to 1000x1000
image = Image.open(jpeg_path)
image = image.resize((1000, 1000), Image.ANTIALIAS)
image = image.convert("RGB")  # Ensure the image is in RGB format

# Save the image as PPM in P6 format
with open(output_ppm_path, "wb") as ppm_file:
    # Write PPM header
    ppm_file.write(b"P6\n")
    ppm_file.write(b"1000 1000\n")  # Image dimensions
    ppm_file.write(b"255\n")  # Maximum color value

    # Write pixel data
    ppm_file.write(image.tobytes())  # Raw RGB pixel data
