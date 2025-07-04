#define _GNU_SOURCE
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#include <stdint.h>
#include <sched.h>
#include <time.h>
#include <termios.h>

#include "rammel.h"
#include "matrix.h"


size_t buffer_size = VOXELS_X*VOXELS_Y*VOXELS_Z * 2 * sizeof(pixel_t);
int content_fd;
void* content_map;
pixel_t* content_slice;
size_t volume_size = VOXELS_X*VOXELS_Y*VOXELS_Z * sizeof(pixel_t);

void vox_blit(const char* filename) {
    FILE* cyc_fd = fopen(filename, "rb");
    if (cyc_fd == NULL) {
        perror("open");
        exit(1);
    }

    fseek(cyc_fd, 0, SEEK_END);
    int file_size = ftell(cyc_fd);
    fseek(cyc_fd, 0, SEEK_SET);

    int frames = file_size / volume_size;

    int rtot = 0;
    do {
        int rnow = fread(content_map + rtot, 1, volume_size - rtot, cyc_fd);
        if (rnow <= 0) {
            fseek(cyc_fd, 0, SEEK_SET);
        }

        rtot += rnow;

        if ((uint)rtot >= volume_size) {
            rtot = 0;
            //sleep(1);
        }

    } while (frames > 1);

    fclose(cyc_fd);
}

int main(int argc, char** argv) {

    if (argc != 2) {
        //printf("%s <dump.data>\n", argv[0]);
        //exit(1);
    }

    content_fd = shm_open("/vortex_double_buffer", O_RDWR, 0666);
    if (content_fd == -1) {
        perror("shm_open");
        exit(1);
    }


    if (ftruncate(content_fd, buffer_size) == -1) {
        perror("ftruncate");
        exit(1);
    }

    content_map = mmap(NULL, buffer_size, PROT_WRITE, MAP_SHARED, content_fd, 0);
    if (content_map == MAP_FAILED) {
        perror("mmap");
        exit(1);
    }
    content_slice = (pixel_t*)content_map;

    if (argc == 2) {
        vox_blit(argv[1]);
    } else {
        for (int x = 0; x < VOXELS_X; ++x) {
            for (int y = 0; y < VOXELS_Y; ++y) {
                for (int z = 0; z < VOXELS_Z; ++z) {
                    pixel_t pix = 0;
                    if ((y&7)==0 && (z==VOXELS_Z/2)/*(z&7)==0*/) {
                        pix |= 0b11100000;
                    }
                    if ((x&7)==0 && (z==VOXELS_Z/2)/*(z&7)==0*/) {
                        pix |= 0b00011100;
                    }
                    if ((x&7)==0 && (y&7)==0) {
                        pix |= 0b00000011;
                    }


                    content_slice[VOXEL_INDEX(x, y, z)] = pix;
                }
            }
        }
    }


    munmap(content_map, buffer_size);
    close(content_fd);

    return 0;
}




