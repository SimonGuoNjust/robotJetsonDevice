#include "rgb2yuv.cuh"
#include <stdio.h>
#include <stdint.h>

__global__ void convert_rgb_to_yu12_kernel(uint8_t *rgb_input, uint8_t *yu12_output)
{
    int y_idx = threadIdx.y + blockIdx.y * blockDim.y;
    int x_idx = threadIdx.x + blockIdx.x * blockDim.x;
    int idx = x_idx + y_idx * gridDim.x	* blockDim.x;
    int width = 1280, height = 720;

    int R, G, B, Y, U, V;
    B = rgb_input[(1280 * y_idx + x_idx) * 3 + 2];
    R = rgb_input[(1280 * y_idx + x_idx) * 3];
    G = rgb_input[(1280 * y_idx + x_idx) * 3 + 1];

    Y = ((66 * R + 129 * G + 25 * B + 128) >> 8) + 16;
	U = ((-38 * R - 74 * G + 112 * B + 128) >> 8) + 128;
	V = ((112 * R - 94 * G - 18 * B + 128) >> 8) + 128;

    yu12_output[1280 * y_idx + x_idx] = (uint8_t)((Y < 0) ? 0 : ((Y > 255) ? 255 : Y));
    int start_u_output = 1280 * 720;
    int start_v_output = 1280 * 720 * 5 / 4;
    if ((y_idx % 2 == 0) && (x_idx % 2 == 0)){
        yu12_output[start_u_output + 640 / 2 * y_idx + x_idx / 2] = (uint8_t)((U < 0) ? 0 : ((U > 255) ? 255 : U));
        yu12_output[start_v_output + 640 / 2 * y_idx + x_idx / 2] = (uint8_t)((V < 0) ? 0 : ((V > 255) ? 255 : V));
    }
}

void convert_rgb_to_yu12(uint8_t *input, uint8_t *output)
{
    uint8_t *dev_input, *dev_output;
    cudaMalloc((void**)&dev_input, 1280 * 720 * 3 * sizeof(uint8_t));
    cudaMalloc((void**)&dev_output, 1280 * 720 * 3 * sizeof(uint8_t) / 2);

    cudaMemcpy(dev_input, input, 1280 * 720 * 3 * sizeof(uint8_t), cudaMemcpyHostToDevice);
    dim3 blocks(1280 / 32, 720 / 16);
    dim3 threads(32, 16);
    convert_rgb_to_yu12_kernel<<< blocks, threads >>> (dev_input, dev_output);
    cudaMemcpy(output, dev_output, 1280 * 720 * 3 * sizeof(uint8_t) / 2, cudaMemcpyDeviceToHost);
    cudaFree(dev_input);
	cudaFree(dev_output);
}