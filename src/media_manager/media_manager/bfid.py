# region convert function
def int_to_binary_array(value, bits):
        """Convert an integer to a list of bits (MSB first)."""
        return [(value >> i) & 1 for i in range(bits - 1, -1, -1)]

def binary_array_to_int(binary_array):
    """Convert a list of bits (MSB first) to an integer."""
    value = 0
    for bit in binary_array:
        value = (value << 1) | bit
    return value

# endregion

def extract_binary_from_image(image, start_x=0, start_y=0, bit_size=3, total_bits=64):
    """
    Extracts 64 bits from the top-left of the image.
    Each bit is represented by a 3x3 pixel block (white=1, black=0).
    """
    bits = []
    for i in range(total_bits):
        x = start_x + (i % 32) * bit_size
        y = start_y + (i // 32) * bit_size

        roi = image[y:y + bit_size, x:x + bit_size]  # Extract the bit region
        avg_color = np.mean(roi)  # Compute the average brightness
        bit = 1 if avg_color > 128 else 0  # Threshold to classify white or black
        bits.append(bit)

    # Convert to integer values
    sec = binary_array_to_int(bits[:32])  # First 32 bits for sec (int32)
    nanosec = binary_array_to_int(bits[32:])  # Last 32 bits for nanosec (uint32)

    return sec, nanosec

def draw_binary_on_image(image, sec, nanosec, start_x=0, start_y=0, bit_size=3):
    """
    Draws binary representation of sec (int32) and nanosec (uint32) on the image using NumPy.
    Each bit is represented as a `bit_size x bit_size` pixel block.
    """
    sec_bits = int_to_binary_array(sec, 32)
    nanosec_bits = int_to_binary_array(nanosec, 32)
    bits = sec_bits + nanosec_bits  # Concatenate both numbers (64 bits total)

    for i, bit in enumerate(bits):
        x = start_x + (i % 32) * bit_size
        y = start_y + (i // 32) * bit_size
        color = 255 if bit == 1 else 0  # White (1) or Black (0)

        # Set pixel values using NumPy slicing
        image[y:y + bit_size, x:x + bit_size] = color

    return image