/// takes a yuyv image (4:2:2 subsample of yuv values of ordering YUY'V as byte stream)
/// and converts it to an RGB buffer (u32 per pixel for RGBA (1 byte per channel))
pub fn yuyv_to_rgb(yuv_buffer: &Vec<u8>, width: usize, height: usize) -> Vec<u32> {
    let mut display_buffer = vec![0u32; width * height];
    let mut yuv_iter = yuv_buffer.iter().peekable();
    let mut rgb_iter = display_buffer.iter_mut();
    while yuv_iter.peek().is_some() {
        // this is a 4:2:2 subsample of yuv pixels (y per pixel, shared uv per 2 pixels)
        let mut two_pixels = yuv_iter.by_ref().take(4);
        let y1 = *two_pixels.next().unwrap() as f32;
        let u = *two_pixels.next().unwrap() as f32;
        let y2 = *two_pixels.next().unwrap() as f32;
        let v = *two_pixels.next().unwrap() as f32;
        let r1 = (y1 + 1.371 * (v - 128.0)).clamp(0.0, 255.0);
        let g1 = (y1 - 0.698 * (v - 128.0) + 0.336 * (u - 128.0)).clamp(0.0, 255.0);
        let b1 = (y1 + 1.732 * (u - 128.0)).clamp(0.0, 255.0);

        let r2 = (y2 + 1.371 * (v - 128.0)).clamp(0.0, 255.0);
        let g2 = (y2 - 0.698 * (v - 128.0) + 0.336 * (u - 128.0)).clamp(0.0, 255.0);
        let b2 = (y2 + 1.732 * (u - 128.0)).clamp(0.0, 255.0);
        // if you want to see how trippy it is to just display the yuv values as rgb, uncomment the following lines
        // *rgb_iter.next().unwrap() = ((y1 as u32) << 16) | ((u as u32) << 8) | v as u32;
        // *rgb_iter.next().unwrap() = ((y2 as u32) << 16) | ((u as u32) << 8) | v as u32;
        *rgb_iter.next().unwrap() = ((r1 as u32) << 16) | ((g1 as u32) << 8) | (b1 as u32);
        *rgb_iter.next().unwrap() = ((r2 as u32) << 16) | ((g2 as u32) << 8) | (b2 as u32);
    }
    display_buffer
}

/// Stacks two RGB pixel arrays vertically, adjusting for differing widths and heights.
///
/// # Arguments
///
/// * `top` - The pixel array of the top image (flattened RGB).
/// * `top_width` - Width of the top image.
/// * `bottom` - The pixel array of the bottom image (flattened RGB).
/// * `bottom_width` - Width of the bottom image.
///
/// # Returns
///
/// A tuple containing:
/// * The new pixel array for the stacked image.
/// * The width of the stacked image.
/// * The height of the stacked image.
///
/// # Panics
///
/// Panics if the lengths of the input pixel arrays are not divisible by their respective widths.
pub(crate) fn stack_images(
    top: Vec<u32>,
    top_width: usize,
    bottom: Vec<u32>,
    bottom_width: usize,
) -> (Vec<u32>, usize, usize) {
    // Validate input dimensions
    let top_height = top.len() / top_width;
    assert_eq!(
        top.len() % top_width,
        0,
        "Top image data is not aligned with its width."
    );

    let bottom_height = bottom.len() / bottom_width;
    assert_eq!(
        bottom.len() % bottom_width,
        0,
        "Bottom image data is not aligned with its width."
    );

    // Calculate the dimensions of the new image
    let new_width = top_width.max(bottom_width);
    let new_height = top_height + bottom_height;

    // Create the new image buffer
    let mut stacked_image = vec![0u32; new_width * new_height];

    // Copy the top image into the stacked image buffer
    for row in 0..top_height {
        let start_src = row * top_width;
        let end_src = start_src + top_width;
        let start_dst = row * new_width;

        stacked_image[start_dst..start_dst + top_width].copy_from_slice(&top[start_src..end_src]);
    }

    // Copy the bottom image into the stacked image buffer
    for row in 0..bottom_height {
        let start_src = row * bottom_width;
        let end_src = start_src + bottom_width;
        let start_dst = (top_height + row) * new_width;

        stacked_image[start_dst..start_dst + bottom_width]
            .copy_from_slice(&bottom[start_src..end_src]);
    }

    (stacked_image, new_width, new_height)
}
