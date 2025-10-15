// This file is a quick proof of concept that what I propose is possible (creating a ros2 bag file from scratch)
// a ros2 bag file is essentially an sqlite db with a message table with the raw messages
// the message row has a BLOB column with the raw bytes
// the challenge is being able to read and write to this blob format reliably
// in this file, we explore if existing cdr libs are able to read (and thus write) cdr reliably
// to generate the input file for this program, use output_hex.sh on a ros2 bag file (assuming the top video feed msg is the first row)
// you also need to install @foxglove/cdr to your environment

const fs = require('fs');
const { CdrReader } = require('@foxglove/cdr');
// Helper function to convert YUV422_YUY2 to RGB
function yuv422ToRgb(yuvData, width, height) {
    const rgbData = new Uint8Array(width * height*3); // RGB has 3 bytes per pixel
    let rgbIndex = 0;

    for (let i = 0; i < yuvData.length; i += 4) {
        const y0 = yuvData[i];
        const u = yuvData[i + 1];
        const y1 = yuvData[i + 2];
        const v = yuvData[i + 3];

        const rgb0 = yuvToRgb(y0, u, v);
        const rgb1 = yuvToRgb(y1, u, v);

        // if we use uint32 instead with alpha pixel
        // rgbData[rgbIndex++] = (rgb0[0]<<24)|(rgb0[1]<<16)|(rgb0[2]<<8)|0;
        // rgbData[rgbIndex++] = (rgb1[0]<<24)|(rgb1[1]<<16)|(rgb1[2]<<8)|0;
        rgbData[rgbIndex++] = rgb0[0];
        rgbData[rgbIndex++] = rgb0[1];
        rgbData[rgbIndex++] = rgb0[2];
        rgbData[rgbIndex++] = rgb1[0];
        rgbData[rgbIndex++] = rgb1[1];
        rgbData[rgbIndex++] = rgb1[2];
    }

    return rgbData;
}

// Helper function to convert a single YUV to RGB
function yuvToRgb(y, u, v) {
    const c = y - 16;
    const d = u - 128;
    const e = v - 128;

    const r = Math.max(0, Math.min(255, (298 * c + 409 * e + 128) >> 8));
    const g = Math.max(0, Math.min(255, (298 * c - 100 * d - 208 * e + 128) >> 8));
    const b = Math.max(0, Math.min(255, (298 * c + 516 * d + 128) >> 8));

    return [r, g, b];
}

// Read file and decode
const file = '/home/ryan/git/cdr_playground/output_file.hex';
const bytes = Uint8Array.from(Buffer.from(fs.readFileSync(file, 'utf8'), 'hex'));
const reader = new CdrReader(bytes);

// Read image metadata
const sec = reader.int32();
const nanosec = reader.uint32();
const frame_id = reader.string();
const height = reader.uint32();
const width = reader.uint32();
const encoding = reader.string();
const isBigEndian = reader.uint8();
const step = reader.uint32();
// log all the metadata
console.log(`sec: ${sec}, nanosec: ${nanosec}, frame_id: ${frame_id}, height: ${height}, width: ${width}, encoding: ${encoding}, isBigEndian: ${isBigEndian}, step: ${step}`);
const img = reader.uint8Array();

// Ensure encoding is YUV422_YUY2
if (encoding !== 'yuv422_yuy2') {
    throw new Error(`Unsupported encoding: ${encoding}`);
}

// Convert and save as an image
const rgbData = yuv422ToRgb(img, width, height);

function saveAsPPM(rgbArray, width, height, filePath) {
    // Validate input dimensions
    if (rgbArray.length !== width * height * 3) {
        throw new Error("RGB array size does not match the given dimensions");
    }

    // Create PPM header
    const header = `P6\n${width} ${height}\n255\n`;

    // Write header and binary pixel data to file
    const file = fs.createWriteStream(filePath);
    file.write(header, 'ascii'); // Write the header in ASCII
    file.write(rgbArray);       // Write the RGB data as binary
    file.end();

    console.log(`PPM image saved to ${filePath}`);
}

saveAsPPM(Buffer.from(rgbData.buffer), width, height, '/home/ryan/git/cdr_playground/output.ppm');
